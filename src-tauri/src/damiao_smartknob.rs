//! DM-H3510 support for the shared host-side SmartKnob haptic engine.
//!
//! Configuration and confirmed enable/disable use the existing DAMIAO session.
//! Its slow background register reader is stopped before takeover: only the
//! haptic loop sends MIT targets, and this module receives the fast feedback.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use can_transport::{CanBus, CanFilter, CanFrame, CanId, FrameKind};
use tokio::task::JoinHandle;

use crate::damiao::{DamiaoMode, DamiaoSession, DamiaoState};
use crate::meow_smartknob::{KnobCalibration, KnobFeedback};
use crate::smartknob::KnobConfig;

pub const PEAK_TORQUE_NM: f64 = 0.45;
pub const DEFAULT_TORQUE_LIMIT_NM: f64 = 0.12;
pub const DEFAULT_MAX_TORQUE_PERMILLE: u16 = 400;
const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(20);
const SEND_TIMEOUT: Duration = Duration::from_millis(20);
// DAMIAO TIMEOUT counts 50 us ticks. This volatile 100 ms drive watchdog
// remains independent of the host's stricter 20 ms feedback/timing guards.
const WATCHDOG_TICKS: f64 = 2000.0;

/// Keep the shared mode vocabulary and geometry, with defaults suited to a
/// 0.18 Nm rated / 0.45 Nm peak direct-drive hub motor.
pub fn preset_configs() -> Vec<KnobConfig> {
    crate::smartknob::preset_configs()
        .into_iter()
        .map(|mut config| {
            config.strength_scale = config.strength_scale.min(0.06);
            config.friction_compensation = 0.0;
            config.click_torque_nm = config.click_torque_nm.min(0.015);
            config.d_gain = config.d_gain.min(0.08);
            config
        })
        .collect()
}

#[derive(Clone, Copy)]
struct Mapping {
    motor_id: u16,
    master_id: u16,
    p_max: f64,
    v_max: f64,
    t_max: f64,
}

impl Mapping {
    fn from_state(state: &DamiaoState) -> Result<Self> {
        validate_motor_id(state.motor_id)?;
        let mapping = Self {
            motor_id: state.motor_id,
            master_id: state
                .feedback_can_id
                .ok_or_else(|| anyhow!("DAMIAO SmartKnob feedback CAN ID is unconfirmed"))?,
            p_max: f64::from(state.p_max),
            v_max: f64::from(state.v_max),
            t_max: f64::from(state.t_max),
        };
        for (name, value) in [
            ("PMAX", mapping.p_max),
            ("VMAX", mapping.v_max),
            ("TMAX", mapping.t_max),
        ] {
            if !value.is_finite() || value <= 0.0 {
                bail!("DAMIAO SmartKnob {name} must be finite and positive");
            }
        }
        if mapping.t_max * 2.0 / 4095.0 > 0.01 {
            bail!("DM-H3510 TMAX mapping is too coarse for haptic control (one torque count exceeds 0.01 Nm)");
        }
        // Parameter and target echoes cannot be distinguished from telemetry
        // reliably if the same identifier is used for both directions.
        if mapping.master_id == mapping.motor_id || mapping.master_id == 0x7FF {
            bail!(
                "DAMIAO SmartKnob requires a separate feedback CAN ID (not the motor ID or 0x7FF)"
            );
        }
        Ok(mapping)
    }
}

#[derive(Clone, Copy)]
struct Feedback {
    position_rad: f64,
    velocity_rad_s: f64,
    torque_nm: f32,
    status: u8,
    mos_temp_c: f32,
    rotor_temp_c: f32,
}

#[derive(Default)]
struct FeedbackState {
    last: Option<Feedback>,
    received_at: Option<Instant>,
    unwrapped_position_rad: f64,
    error: Option<String>,
}

impl FeedbackState {
    fn update(&mut self, feedback: Feedback, p_max: f64, now: Instant) {
        self.unwrapped_position_rad = match self.last {
            Some(previous) => {
                self.unwrapped_position_rad
                    + unwrap_delta(feedback.position_rad - previous.position_rad, p_max)
            }
            None => feedback.position_rad,
        };
        self.last = Some(feedback);
        self.received_at = Some(now);
        // Faults and receive failures are sticky until the session is stopped.
        if feedback.status > 1 {
            self.error = Some(format!(
                "DM-H3510 motor fault 0x{:X}: {}",
                feedback.status,
                status_name(feedback.status)
            ));
        }
    }
}

pub(crate) struct DamiaoKnobMotor {
    session: Arc<DamiaoSession>,
    bus: Arc<dyn CanBus>,
    mapping: Mapping,
    feedback: Arc<Mutex<FeedbackState>>,
    receiver: Mutex<Option<JoinHandle<()>>>,
    stopped: AtomicBool,
}

impl DamiaoKnobMotor {
    pub(crate) async fn start(
        session: Arc<DamiaoSession>,
        bus: Arc<dyn CanBus>,
        shutdown: &AtomicBool,
    ) -> Result<Self> {
        validate_motor_id(session.snapshot().motor_id)?;
        // A dedicated SmartKnob session must never keep the old 50 Hz target,
        // 100 ms refresh, or 200 ms precise-position poll alive after takeover.
        session.stop_stream();
        session.force_stop();
        startup_allowed(shutdown)?;
        session
            .disable()
            .await
            .context("disable DM-H3510 before SmartKnob startup")?;
        startup_allowed(shutdown)?;
        let mapping = Mapping::from_state(&session.snapshot())?;
        session
            .write_register(0x09, WATCHDOG_TICKS)
            .await
            .context("set volatile DM-H3510 communication watchdog")?;
        startup_allowed(shutdown)?;
        // This is a volatile mode write; no flash save or zero-save is sent.
        session
            .switch_mode(DamiaoMode::Mit)
            .await
            .context("select DM-H3510 MIT mode")?;
        startup_allowed(shutdown)?;
        let mut rx = bus
            .subscribe(CanFilter::exact_standard(mapping.master_id))
            .await?;
        let feedback = Arc::new(Mutex::new(FeedbackState::default()));
        let shared = feedback.clone();
        let receiver = tokio::spawn(async move {
            loop {
                match rx.recv().await {
                    Ok(frame) => {
                        if let Some(value) = decode_feedback(mapping, &frame) {
                            shared
                                .lock()
                                .unwrap()
                                .update(value, mapping.p_max, Instant::now());
                        }
                    }
                    Err(error) => {
                        shared.lock().unwrap().error = Some(format!(
                            "DM-H3510 SmartKnob feedback receive failed: {error}"
                        ));
                        break;
                    }
                }
            }
        });
        let motor = Self {
            session,
            bus,
            mapping,
            feedback,
            receiver: Mutex::new(Some(receiver)),
            stopped: AtomicBool::new(false),
        };
        let result: Result<()> = async {
            startup_allowed(shutdown)?;
            // The existing lifecycle preloads zero T/KP/KD while Disabled and
            // confirms fresh Enabled feedback before returning.
            motor
                .session
                .enable()
                .await
                .context("enable DM-H3510 SmartKnob MIT")?;
            startup_allowed(shutdown)?;
            motor.send_raw(0.0, 0).await?;
            tokio::time::timeout(Duration::from_millis(100), async {
                loop {
                    startup_allowed(shutdown)?;
                    if motor.feedback().is_ok() {
                        break Ok::<(), anyhow::Error>(());
                    }
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
            })
            .await
            .context("fresh DM-H3510 SmartKnob feedback after enable")??;
            Ok(())
        }
        .await;
        if let Err(error) = result {
            return match motor.stop().await {
                Ok(()) => Err(error),
                Err(cleanup) => Err(anyhow!(
                    "{error:#}; DM-H3510 cleanup also failed: {cleanup:#}"
                )),
            };
        }
        Ok(motor)
    }

    pub(crate) fn feedback(&self) -> Result<KnobFeedback> {
        let shared = self.feedback.lock().unwrap();
        if let Some(error) = &shared.error {
            bail!("{error}");
        }
        let value = shared
            .last
            .ok_or_else(|| anyhow!("DM-H3510 feedback is unavailable"))?;
        validate_feedback(value, shared.received_at, Instant::now())?;
        Ok(KnobFeedback {
            raw_rev: shared.unwrapped_position_rad / std::f64::consts::TAU,
            velocity_rev_per_s: value.velocity_rad_s / std::f64::consts::TAU,
            torque_nm: Some(value.torque_nm),
            driver_temp_c: Some(value.mos_temp_c),
            motor_temp_c: Some(value.rotor_temp_c),
            online: true,
            enabled: true,
            error: None,
        })
    }

    pub(crate) async fn send_torque(&self, torque_nm: f64, max_permille: u16) -> Result<f64> {
        if self.stopped.load(Ordering::Acquire) {
            bail!("DM-H3510 SmartKnob motor has been stopped");
        }
        self.feedback()?;
        self.send_raw(torque_nm, max_permille).await
    }

    async fn send_raw(&self, torque_nm: f64, max_permille: u16) -> Result<f64> {
        let (frame, actual) = torque_frame(self.mapping, torque_nm, max_permille)?;
        tokio::time::timeout(SEND_TIMEOUT, self.bus.send(frame))
            .await
            .context("DM-H3510 SmartKnob MIT send timed out")??;
        Ok(actual)
    }

    pub(crate) async fn stop(&self) -> Result<()> {
        self.stopped.store(true, Ordering::Release);
        // Confirm Disabled even when sending a neutral target failed. Neither
        // the send result nor a cached Disabled sample is enough to release it.
        let _ = self.send_raw(0.0, 0).await;
        let result = self
            .session
            .disable()
            .await
            .context("confirm DM-H3510 Disabled");
        if result.is_ok() {
            if let Some(task) = self.receiver.lock().unwrap().take() {
                task.abort();
            }
        }
        result
    }

    pub(crate) async fn clear_error(&self) -> Result<()> {
        if !self.stopped.load(Ordering::Acquire) {
            bail!("Stop DM-H3510 SmartKnob before clearing its fault");
        }
        self.session.clear_fault().await
    }

    pub(crate) fn calibration(&self) -> KnobCalibration {
        KnobCalibration {
            torque_factor: 1.0,
            static_positive_nm: None,
            static_negative_nm: None,
            kinetic_positive_nm: None,
            kinetic_negative_nm: None,
            detail: format!(
                "DM-H3510 MIT; PMAX={:.5} rad, VMAX={:.5} rad/s, TMAX={:.5} Nm; no factory friction calibration",
                self.mapping.p_max, self.mapping.v_max, self.mapping.t_max
            ),
        }
    }

    pub(crate) fn friction_torque(
        &self,
        velocity_rad_s: f64,
        _restoring_nm: f64,
        cap_nm: f64,
    ) -> f64 {
        // H3510 has no compatible Meow factory-friction table. The optional
        // manual Coulomb compensation is inactive at rest and defaults to zero.
        if !velocity_rad_s.is_finite() || !cap_nm.is_finite() || velocity_rad_s.abs() < 0.05 {
            0.0
        } else {
            velocity_rad_s.signum() * cap_nm.clamp(0.0, 0.03)
        }
    }
}

impl Drop for DamiaoKnobMotor {
    fn drop(&mut self) {
        if let Some(task) = self.receiver.lock().unwrap().take() {
            task.abort();
        }
        // No detached task can keep feeding the drive watchdog after Drop.
        self.session.force_stop();
    }
}

fn validate_motor_id(motor_id: u16) -> Result<()> {
    if motor_id > 0x0F {
        bail!("DM-H3510 IDs above 0x0F overlap the feedback status bits; independent CC/1 state confirmation is required. Select a motor ID in 0x00..=0x0F before starting SmartKnob");
    }
    Ok(())
}

fn startup_allowed(shutdown: &AtomicBool) -> Result<()> {
    if shutdown.load(Ordering::SeqCst) {
        bail!("DM-H3510 SmartKnob startup cancelled by application shutdown");
    }
    Ok(())
}

fn unwrap_delta(delta: f64, p_max: f64) -> f64 {
    // The protocol position wraps at +/-PMAX, independent of shaft turns.
    let span = 2.0 * p_max;
    (delta + p_max).rem_euclid(span) - p_max
}

fn validate_feedback(value: Feedback, received_at: Option<Instant>, now: Instant) -> Result<()> {
    let age = received_at.map(|at| now.saturating_duration_since(at));
    if !age.is_some_and(|age| age <= FEEDBACK_TIMEOUT) {
        bail!("DM-H3510 SmartKnob feedback is older than 20 ms");
    }
    if value.status != 1 {
        bail!(
            "DM-H3510 SmartKnob requires Enabled feedback, got {} (0x{:X})",
            status_name(value.status),
            value.status
        );
    }
    Ok(())
}

fn decode_feedback(mapping: Mapping, frame: &CanFrame) -> Option<Feedback> {
    if frame.id() != CanId::Standard(mapping.master_id)
        || !matches!(frame.kind(), FrameKind::Data | FrameKind::Fd { .. })
        || frame.data().len() != 8
    {
        return None;
    }
    let data = frame.data();
    // Parameter replies share the same feedback identifier.
    if u16::from_le_bytes([data[0], data[1]]) == mapping.motor_id
        && matches!(data[2], 0x33 | 0x55 | 0xAA | 0xCC)
    {
        return None;
    }
    if data[0] & 0x0F != (mapping.motor_id & 0x0F) as u8 {
        return None;
    }
    let status = data[0] >> 4;
    if !matches!(status, 0 | 1 | 3 | 4 | 5 | 8..=14) {
        return None;
    }
    let position = u16::from_be_bytes([data[1], data[2]]);
    let velocity = (u16::from(data[3]) << 4) | u16::from(data[4] >> 4);
    let torque = (u16::from(data[4] & 15) << 8) | u16::from(data[5]);
    Some(Feedback {
        position_rad: decode(position, mapping.p_max, 65535.0),
        // The two midpoint codes straddle zero by half an LSB. Treat only
        // those codes as stationary so wide VMAX mappings cannot inject
        // phantom damping or friction; every other code retains its scale.
        velocity_rad_s: if matches!(velocity, 2047 | 2048) {
            0.0
        } else {
            decode(velocity, mapping.v_max, 4095.0)
        },
        torque_nm: decode(torque, mapping.t_max, 4095.0) as f32,
        status,
        mos_temp_c: f32::from(data[6]),
        rotor_temp_c: f32::from(data[7]),
    })
}

fn decode(code: u16, maximum: f64, scale: f64) -> f64 {
    f64::from(code) * 2.0 * maximum / scale - maximum
}

fn torque_frame(mapping: Mapping, torque_nm: f64, max_permille: u16) -> Result<(CanFrame, f64)> {
    if !torque_nm.is_finite() || max_permille > 1000 {
        bail!("DM-H3510 torque must be finite and maximum output within 0..=1000 permille");
    }
    let limit = (PEAK_TORQUE_NM * f64::from(max_permille) / 1000.0).min(mapping.t_max);
    let torque = torque_nm.clamp(-limit, limit);
    let encode = |value: f64| (value + mapping.t_max) * 4095.0 / (2.0 * mapping.t_max);
    let lower = encode(-limit).ceil() as u16;
    let upper = encode(limit).floor() as u16;
    let encoded = if lower <= upper {
        (encode(torque) as u16).clamp(lower, upper)
    } else {
        // The odd number of intervals gives no exact zero code. Use the
        // protocol's conventional neutral code (half an LSB below zero).
        2047
    };
    // P/V neutral values are immaterial because KP/KD are exactly zero.
    let payload = [
        0x7F,
        0xFF,
        0x7F,
        0xF0,
        0,
        0,
        (encoded >> 8) as u8,
        encoded as u8,
    ];
    let frame = CanFrame::new_data(mapping.motor_id, &payload)?;
    Ok((frame, decode(encoded, mapping.t_max, 4095.0)))
}

fn status_name(code: u8) -> &'static str {
    match code {
        0 => "Disabled",
        1 => "Enabled",
        3 => "Output encoder calibration fault",
        4 => "Sensor output fault",
        5 => "Motor encoder calibration fault",
        8 => "Over-voltage",
        9 => "Under-voltage",
        10 => "Over-current",
        11 => "MOS over-temperature",
        12 => "Motor over-temperature",
        13 => "Communication lost",
        14 => "Overload",
        _ => "Unknown",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::damiao::DamiaoConfig;
    use can_transport::{CanCapabilities, CanIoError, CanRx};
    use std::collections::HashMap;
    use std::sync::atomic::{AtomicU16, AtomicU8};
    use tokio::sync::mpsc;

    struct TestRx(mpsc::UnboundedReceiver<CanFrame>);

    #[async_trait::async_trait]
    impl CanRx for TestRx {
        async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
            self.0.recv().await.ok_or(CanIoError::Disconnected)
        }
        fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
            match self.0.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
            }
        }
    }

    /// A protocol-level motor double. It only responds to received commands;
    /// this exposes any accidental background traffic after SmartKnob takeover.
    struct MotorBus {
        registers: Mutex<HashMap<u8, u32>>,
        subscribers: Mutex<Vec<(CanFilter, mpsc::UnboundedSender<CanFrame>)>>,
        frames: Mutex<Vec<CanFrame>>,
        status: AtomicU8,
        velocity_code: AtomicU16,
        lose_disable_reply: AtomicBool,
    }

    impl MotorBus {
        fn new() -> Arc<Self> {
            Arc::new(Self {
                registers: Mutex::new(HashMap::from([
                    (0x15, 12.5f32.to_bits()),
                    (0x16, 280.0f32.to_bits()),
                    (0x17, 0.5f32.to_bits()),
                    (0x50, 0.0f32.to_bits()),
                    (0x51, 0.0f32.to_bits()),
                    (0x09, 0),
                    (0x0A, 1),
                ])),
                subscribers: Mutex::new(Vec::new()),
                frames: Mutex::new(Vec::new()),
                status: AtomicU8::new(0),
                velocity_code: AtomicU16::new(2047),
                lose_disable_reply: AtomicBool::new(false),
            })
        }
        fn publish(&self, frame: CanFrame) {
            for (filter, sender) in self.subscribers.lock().unwrap().iter() {
                if filter.matches(&frame) {
                    let _ = sender.send(frame);
                }
            }
        }
    }

    #[async_trait::async_trait]
    impl CanBus for MotorBus {
        async fn send(&self, frame: CanFrame) -> std::result::Result<(), CanIoError> {
            self.frames.lock().unwrap().push(frame.clone());
            let data = frame.data();
            if frame.id() == CanId::Standard(0x7FF) && data.len() >= 4 {
                match data[2] {
                    0x33 | 0x55 => {
                        let mut registers = self.registers.lock().unwrap();
                        if data[2] == 0x55 {
                            registers.insert(
                                data[3],
                                u32::from_le_bytes(data[4..8].try_into().unwrap()),
                            );
                        }
                        let value = *registers.get(&data[3]).unwrap_or(&0);
                        let mut reply = [1, 0, data[2], data[3], 0, 0, 0, 0];
                        reply[4..8].copy_from_slice(&value.to_le_bytes());
                        drop(registers);
                        self.publish(CanFrame::new_data(0, &reply).unwrap());
                    }
                    _ => {}
                }
            } else if frame.id() == CanId::Standard(1) && data.len() == 8 {
                let special = data[..7] == [0xFF; 7];
                if special && data[7] == 0xFC {
                    self.status.store(1, Ordering::Release);
                }
                if special && data[7] == 0xFD {
                    self.status.store(0, Ordering::Release);
                    if self.lose_disable_reply.load(Ordering::Acquire) {
                        return Ok(());
                    }
                }
                let mut feedback: [u8; 8] = sample(self.status.load(Ordering::Acquire), 32767)
                    .data()
                    .try_into()
                    .unwrap();
                let velocity = self.velocity_code.load(Ordering::Acquire);
                feedback[3] = (velocity >> 4) as u8;
                feedback[4] = (((velocity & 15) << 4) as u8) | (feedback[4] & 15);
                self.publish(CanFrame::new_data(0, &feedback).unwrap());
            }
            Ok(())
        }
        async fn subscribe(
            &self,
            filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            let (sender, receiver) = mpsc::unbounded_channel();
            self.subscribers.lock().unwrap().push((filter, sender));
            Ok(Box::new(TestRx(receiver)))
        }
        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: false,
                max_dlen: 8,
            }
        }
    }

    #[tokio::test]
    async fn shared_haptic_loop_applies_h3510_defaults_and_keeps_reduced_limits() {
        let bus = MotorBus::new();
        let session = DamiaoSession::start(
            bus.clone(),
            DamiaoConfig {
                motor_id: 1,
                master_id: 0,
                mode: DamiaoMode::Mit,
                p_max: 12.5,
                v_max: 30.0,
                t_max: 0.5,
            },
        )
        .await
        .unwrap();
        let mut knob = crate::smartknob::SmartKnob::start_damiao(
            session,
            bus.clone(),
            1,
            1,
            &AtomicBool::new(false),
            crate::smartknob::InitialSettings::default(),
        )
        .await
        .unwrap();
        tokio::time::timeout(Duration::from_millis(100), async {
            while !knob.state().running {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .unwrap();
        assert_eq!(knob.state().torque_limit_nm, DEFAULT_TORQUE_LIMIT_NM);
        assert_eq!(
            knob.state().max_torque_permille,
            DEFAULT_MAX_TORQUE_PERMILLE
        );
        assert!(knob
            .state()
            .calibration
            .unwrap()
            .detail
            .contains("TMAX=0.50000"));
        knob.set_tuning(4.0, 0.02, 0.06, 0.03, 50, 0.0, 0.015);
        knob.set_config(8);
        tokio::time::timeout(Duration::from_millis(100), async {
            while knob.state().config_index != 8 {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .unwrap();
        let state = knob.state();
        assert!(state.running && state.enabled && state.error.is_none());
        assert_eq!(state.torque_limit_nm, 0.03);
        assert_eq!(state.max_torque_permille, 50);
        knob.stop_damiao().await.unwrap();
        assert!(!knob.state().running && !knob.state().enabled);
        assert_eq!(bus.status.load(Ordering::Acquire), 0);
    }

    #[tokio::test]
    async fn moving_shaft_rejects_startup_without_poisoning_stop_recovery() {
        let bus = MotorBus::new();
        bus.registers.lock().unwrap().insert(0x0A, 2);
        bus.velocity_code.store(2055, Ordering::Release);
        let session = DamiaoSession::start(
            bus.clone(),
            DamiaoConfig {
                motor_id: 1,
                master_id: 0,
                mode: DamiaoMode::PositionVelocity,
                p_max: 12.5,
                v_max: 280.0,
                t_max: 0.5,
            },
        )
        .await
        .unwrap();
        let result =
            DamiaoKnobMotor::start(session.clone(), bus.clone(), &AtomicBool::new(false)).await;
        assert!(result.is_err());
        assert!(format!("{:#}", result.err().unwrap()).contains("still moving"));
        assert!(
            !session.requires_reconnect(),
            "a precondition failure must retain disable recovery"
        );
        session.disable().await.unwrap();
        assert_eq!(bus.status.load(Ordering::Acquire), 0);
        assert!(
            !bus.frames
                .lock()
                .unwrap()
                .iter()
                .any(|frame| frame.id() == CanId::Standard(1)
                    && frame.data() == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]),
            "startup must never enable a moving shaft"
        );
    }

    #[tokio::test]
    async fn lifecycle_has_one_sender_and_retries_stop_after_lost_confirmation() {
        let bus = MotorBus::new();
        let session = DamiaoSession::start(
            bus.clone(),
            DamiaoConfig {
                motor_id: 1,
                master_id: 0,
                mode: DamiaoMode::Mit,
                p_max: 12.5,
                v_max: 30.0,
                t_max: 0.5,
            },
        )
        .await
        .unwrap();
        let motor = DamiaoKnobMotor::start(session, bus.clone(), &AtomicBool::new(false))
            .await
            .unwrap();
        assert!(motor.feedback().unwrap().enabled);
        assert_eq!(motor.mapping.v_max, 280.0);
        assert_eq!(motor.feedback().unwrap().velocity_rev_per_s, 0.0);
        assert_eq!(motor.friction_torque(0.0, 0.0, 0.03), 0.0);
        assert!((motor.send_torque(0.1, 400).await.unwrap() - 0.1).abs() < 1.0 / 4095.0);
        assert_eq!(bus.registers.lock().unwrap().get(&0x09), Some(&2000));
        let sent = bus.frames.lock().unwrap().len();
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(
            motor.feedback().is_err(),
            "stale fast feedback must stop control"
        );
        assert_eq!(
            bus.frames.lock().unwrap().len(),
            sent,
            "the old session must not poll or keep feeding the watchdog"
        );
        bus.lose_disable_reply.store(true, Ordering::Release);
        assert!(
            motor.stop().await.is_err(),
            "a transmitted disable without fresh confirmation is uncertain"
        );
        assert!(motor.send_torque(0.1, 400).await.is_err());
        bus.lose_disable_reply.store(false, Ordering::Release);
        motor.stop().await.unwrap();
        assert_eq!(bus.status.load(Ordering::Acquire), 0);
        for frame in bus.frames.lock().unwrap().iter() {
            if frame.id() == CanId::Standard(0x7FF) {
                assert_ne!(frame.data()[2], 0xAA);
            }
            assert_ne!(
                frame.data(),
                &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
            );
        }
    }

    fn mapping() -> Mapping {
        Mapping {
            motor_id: 1,
            master_id: 0,
            p_max: 12.5,
            v_max: 30.0,
            t_max: 0.5,
        }
    }

    fn sample(status: u8, position: u16) -> CanFrame {
        CanFrame::new_data(
            0,
            &[
                status << 4 | 1,
                (position >> 8) as u8,
                position as u8,
                0x7F,
                0xF7,
                0xFF,
                30,
                31,
            ],
        )
        .unwrap()
    }

    #[test]
    fn high_motor_ids_are_rejected_before_any_enable() {
        assert!(validate_motor_id(0).is_ok());
        assert!(validate_motor_id(15).is_ok());
        assert!(validate_motor_id(0x14).is_err());
        assert!(validate_motor_id(255).is_err());
    }

    #[test]
    fn velocity_midpoint_codes_are_zero_without_erasing_adjacent_motion() {
        for v_max in [30.0, 280.0] {
            let live_mapping = Mapping { v_max, ..mapping() };
            for code in [2046u16, 2047, 2048, 2049] {
                let mut data: [u8; 8] = sample(1, 32767).data().try_into().unwrap();
                data[3] = (code >> 4) as u8;
                data[4] = (((code & 15) << 4) as u8) | (data[4] & 15);
                let feedback =
                    decode_feedback(live_mapping, &CanFrame::new_data(0, &data).unwrap()).unwrap();
                if matches!(code, 2047 | 2048) {
                    assert_eq!(feedback.velocity_rad_s, 0.0);
                } else {
                    assert_eq!(feedback.velocity_rad_s, decode(code, v_max, 4095.0));
                    assert_ne!(feedback.velocity_rad_s, 0.0);
                }
            }
        }
    }

    #[test]
    fn torque_uses_live_mapping_and_zero_firmware_gains() {
        let (frame, actual) = torque_frame(mapping(), 0.1, 1000).unwrap();
        assert_eq!(frame.id(), CanId::Standard(1));
        assert_eq!(frame.data()[3] & 15, 0);
        assert_eq!(frame.data()[4], 0);
        assert_eq!(frame.data()[5], 0);
        assert_eq!(frame.data()[6] & 0xF0, 0);
        assert!((actual - 0.1).abs() < 1.0 / 4095.0);
        let (_, larger_range) = torque_frame(
            Mapping {
                t_max: 10.0,
                ..mapping()
            },
            0.1,
            1000,
        )
        .unwrap();
        assert!((larger_range - 0.1).abs() < 20.0 / 4095.0);
    }

    #[test]
    fn output_is_bounded_by_motor_peak_and_session_permille() {
        for (requested, permille, expected) in
            [(9.0, 1000, 0.45), (-9.0, 400, -0.18), (9.0, 0, 0.0)]
        {
            let (_, actual) = torque_frame(mapping(), requested, permille).unwrap();
            assert!((actual - expected).abs() <= 1.0 / 4095.0);
        }
        for t_max in [0.5, 1.0, 10.0] {
            for sign in [-1.0, 1.0] {
                let (_, actual) =
                    torque_frame(Mapping { t_max, ..mapping() }, sign * 20.0, 1000).unwrap();
                assert!(actual.abs() <= PEAK_TORQUE_NM);
            }
        }
        assert!(torque_frame(mapping(), f64::NAN, 400).is_err());
        assert!(torque_frame(mapping(), 0.0, 1001).is_err());
    }

    #[test]
    fn mapping_boundary_is_unwrapped_independently_of_shaft_revolutions() {
        let now = Instant::now();
        let mut state = FeedbackState::default();
        let high = decode_feedback(mapping(), &sample(1, 65530)).unwrap();
        let low = decode_feedback(mapping(), &sample(1, 5)).unwrap();
        state.update(high, 12.5, now);
        let before = state.unwrapped_position_rad;
        state.update(low, 12.5, now);
        assert!((state.unwrapped_position_rad - before - 10.0 * 25.0 / 65535.0).abs() < 1e-9);
        assert!((unwrap_delta(-24.9, 12.5) - 0.1).abs() < 1e-9);
        assert!((unwrap_delta(24.9, 12.5) + 0.1).abs() < 1e-9);
        assert_eq!(unwrap_delta(6.0, 12.5), 6.0);
    }

    #[test]
    fn telemetry_routing_and_parameter_replies_are_filtered() {
        let feedback = decode_feedback(mapping(), &sample(1, 65535)).unwrap();
        assert_eq!(feedback.position_rad, 12.5);
        assert_eq!(feedback.mos_temp_c, 30.0);
        assert_eq!(feedback.rotor_temp_c, 31.0);
        assert!(decode_feedback(
            mapping(),
            &CanFrame::new_data(2, sample(1, 1).data()).unwrap()
        )
        .is_none());
        assert!(decode_feedback(
            mapping(),
            &CanFrame::new_data(0, &[1, 0, 0x33, 0x15, 0, 0, 0, 0]).unwrap()
        )
        .is_none());
        assert!(decode_feedback(
            mapping(),
            &CanFrame::new_data(0, &[0x12, 0, 0, 0, 0, 0, 0, 0]).unwrap()
        )
        .is_none());
    }

    #[test]
    fn stale_disabled_and_faulted_feedback_are_rejected() {
        let now = Instant::now();
        let value = decode_feedback(mapping(), &sample(1, 32767)).unwrap();
        assert!(validate_feedback(value, Some(now), now).is_ok());
        assert!(validate_feedback(value, Some(now), now + Duration::from_millis(21)).is_err());
        assert!(validate_feedback(value, None, now).is_err());
        assert!(validate_feedback(Feedback { status: 0, ..value }, Some(now), now).is_err());
        assert!(validate_feedback(
            Feedback {
                status: 13,
                ..value
            },
            Some(now),
            now
        )
        .is_err());
    }
}
