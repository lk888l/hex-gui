//! New-protocol HEX motors used by the host SmartKnob haptic loop.
//!
//! Keep torque in physical Nm in the shared loop, then convert once at the
//! wire boundary. Factory friction is stored in raw command Nm, so its physical
//! equivalent is divided by the factor before being added to the loop output:
//! `(desired + friction_raw / factor) * factor`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use can_transport::{CanBus, CanFrame};
use hex_motor::canopen::rpdo_config::{build_rpdo_config_writes, RpdoRecipe};
use hex_motor::canopen::sdo;
use hex_motor::canopen::tpdo_config::TpdoEntry;
use hex_motor::cia402::PdoProfile;
use hex_motor::meow_motor::{
    MeowFactoryCalibration, MeowFrictionCalibration, MeowMitTarget, MeowMotorCanSettingsStatus,
    MeowMotorLifecycle, MeowMotorLiveState, MeowMotorLogic, MeowMotorManager, MeowMotorMode,
    MeowMotorTarget,
};
use serde::Serialize;

use crate::calibration_transport::CalibrationHeartbeat;

const SDO_TIMEOUT: Duration = Duration::from_millis(500);
const POSITION_TIMEOUT: Duration = Duration::from_millis(20);
const STATE_TIMEOUT: Duration = Duration::from_millis(100);
const SEND_TIMEOUT: Duration = Duration::from_millis(20);

const STATIC_FRICTION_RATIO: f64 = 0.90;
// Factory kinetic friction is an average at the calibration speed. Leave
// margin for angle/temperature variation: 90% sustained unloaded motion on
// the geared 4310 in the release test despite zero restoring demand.
const KINETIC_FRICTION_RATIO: f64 = 0.75;

/// Same physical-Nm range used by Motor Control: raw 0x4576 / factory factor.
pub(crate) fn physical_peak_torque(peak: Option<f32>, factor: f64) -> Result<f64> {
    let raw_peak =
        f64::from(peak.ok_or_else(|| anyhow!("motor peak torque 0x4576 is unavailable"))?);
    let physical = raw_peak / factor;
    if !raw_peak.is_finite()
        || raw_peak <= 0.0
        || !factor.is_finite()
        || factor <= 0.0
        || !physical.is_finite()
        || physical <= 0.0
    {
        bail!("motor peak torque or factory torque factor is invalid");
    }
    Ok(physical)
}

#[derive(Debug, Clone)]
pub(crate) struct KnobFeedback {
    /// Continuous output-shaft revolutions, already unwrapped by the driver.
    pub raw_rev: f64,
    pub velocity_rev_per_s: f64,
    pub torque_nm: Option<f32>,
    pub driver_temp_c: Option<f32>,
    pub motor_temp_c: Option<f32>,
    pub online: bool,
    pub enabled: bool,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct KnobCalibration {
    pub torque_factor: f64,
    /// Displayed in physical Nm; the stored factory values are raw-domain Nm.
    pub static_positive_nm: Option<f64>,
    pub static_negative_nm: Option<f64>,
    pub kinetic_positive_nm: Option<f64>,
    pub kinetic_negative_nm: Option<f64>,
    pub detail: String,
}

pub(crate) struct MeowKnobMotor {
    manager: Arc<MeowMotorManager>,
    bus: Arc<dyn CanBus>,
    node_id: u8,
    epoch: u64,
    segment: u64,
    brs: bool,
    peak_torque_nm: f64,
    torque_factor: f64,
    friction: Option<MeowFrictionCalibration>,
    calibration: KnobCalibration,
    motion: Mutex<FrictionMotion>,
    heartbeat: Mutex<Option<CalibrationHeartbeat>>,
    stopped: AtomicBool,
}

impl MeowKnobMotor {
    pub(crate) async fn start(
        manager: Arc<MeowMotorManager>,
        bus: Arc<dyn CanBus>,
        node_id: u8,
        torque_limit_nm: f64,
        shutdown: &AtomicBool,
        host_node_id: u8,
    ) -> Result<Self> {
        startup_allowed(shutdown)?;
        if !torque_limit_nm.is_finite() || torque_limit_nm < 0.0 {
            bail!("SmartKnob total torque limit must be finite and non-negative");
        }
        if host_node_id == node_id {
            bail!("SmartKnob motor and host heartbeat node IDs must differ");
        }
        let heartbeat = CalibrationHeartbeat::start(bus.clone(), host_node_id)
            .map_err(|error| anyhow!(error))?;
        let result = Self::initialize(&manager, &bus, node_id, torque_limit_nm, shutdown).await;
        match result {
            Ok((live, brs, peak_torque_nm, torque_factor, friction, calibration)) => {
                let motor = Self {
                    manager,
                    bus,
                    node_id,
                    epoch: live.session_epoch,
                    segment: live.measurements.position_accumulation_segment,
                    brs,
                    peak_torque_nm,
                    torque_factor,
                    friction,
                    calibration,
                    motion: Mutex::new(FrictionMotion::default()),
                    heartbeat: Mutex::new(Some(heartbeat)),
                    stopped: AtomicBool::new(false),
                };
                if let Err(error) = startup_allowed(shutdown).and_then(|_| motor.feedback()) {
                    let cleanup = motor.stop().await;
                    return Err(with_cleanup(error, cleanup));
                }
                Ok(motor)
            }
            Err(error) => {
                let cleanup = recover_stop(&manager, &bus, node_id).await;
                heartbeat.stop().await;
                Err(with_cleanup(error, cleanup))
            }
        }
    }

    async fn initialize(
        manager: &MeowMotorManager,
        bus: &Arc<dyn CanBus>,
        node_id: u8,
        torque_limit_nm: f64,
        shutdown: &AtomicBool,
    ) -> Result<(
        MeowMotorLiveState,
        bool,
        f64,
        f64,
        Option<MeowFrictionCalibration>,
        KnobCalibration,
    )> {
        manager
            .identify(node_id)
            .await
            .context("identify Meow SmartKnob")?;
        let epoch = manager.status(node_id)?.session_epoch;
        startup_allowed(shutdown)?;
        // A transport failure is different from a successfully read, absent
        // calibration. Never silently command with an unknown torque factor.
        let factory = manager
            .read_factory_calibration(node_id)
            .await
            .context("read SmartKnob factory calibration 0x4001")?;
        if manager.status(node_id)?.session_epoch != epoch {
            bail!("SmartKnob motor session changed while reading factory calibration");
        }
        let (factor, friction, calibration) = calibration_values(factory)?;
        startup_allowed(shutdown)?;
        manager
            .initialize(node_id, PdoProfile::from_event_timer_ms(1)?)
            .await
            .context("initialize Meow SmartKnob at 1000 Hz")?;
        startup_allowed(shutdown)?;
        let info = manager
            .list()
            .into_iter()
            .find(|info| info.node_id == node_id)
            .ok_or_else(|| anyhow!("SmartKnob motor disappeared during initialization"))?;
        if info.session_epoch != epoch {
            bail!("SmartKnob motor session changed after reading factory calibration");
        }
        let brs = match info.can_settings {
            MeowMotorCanSettingsStatus::Available(settings) => settings.transmit_pdo_brs,
            _ => bail!("SmartKnob CAN settings are unavailable"),
        };
        let peak = f64::from(
            info.peak_torque_nm
                .filter(|value| value.is_finite() && *value > 0.0)
                .ok_or_else(|| anyhow!("SmartKnob peak torque 0x4576 is unavailable"))?,
        );
        let physical_peak = physical_peak_torque(Some(peak as f32), factor)?;
        let max_torque = crate::smartknob::motor_output_permille(torque_limit_nm, physical_peak);
        // Initialize leaves the drive Disabled. Explicitly select uncompressed
        // MIT: another app may previously have selected 0x4103 compressed MIT.
        sdo::download_u8(bus.as_ref(), node_id, 0x4103, 1, 0, Some(SDO_TIMEOUT))
            .await
            .context("select uncompressed MIT")?;
        for write in build_rpdo_config_writes(&torque_recipe(node_id))? {
            startup_allowed(shutdown)?;
            if manager.status(node_id)?.session_epoch != epoch {
                bail!("SmartKnob motor session changed during RPDO setup");
            }
            sdo::download(
                bus.as_ref(),
                node_id,
                write.index,
                write.subindex,
                &write.data,
                Some(SDO_TIMEOUT),
            )
            .await
            .with_context(|| {
                format!(
                    "SmartKnob RPDO mapping {:04X}:{}",
                    write.index, write.subindex
                )
            })?;
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        startup_allowed(shutdown)?;
        manager.set_max_torque(node_id, max_torque).await?;
        startup_allowed(shutdown)?;
        // This writes every zero target first, then 0x4401=4, then confirms
        // a new TPDO2. KP/KD=0 keeps all haptic spring/damping on the host.
        manager
            .set_mode_sdo(
                node_id,
                MeowMotorTarget::Mit(MeowMitTarget {
                    position_rev: 0.0,
                    velocity_rev_per_s: 0.0,
                    torque_nm: 0.0,
                    kp: 0,
                    kd: 0,
                    kp_kd_limit_permille: 0,
                }),
            )
            .await
            .context("enable Meow SmartKnob MIT")?;
        let live = manager.status(node_id)?;
        if live.session_epoch != epoch {
            bail!("SmartKnob motor session changed during enable");
        }
        Ok((live, brs, peak, factor, friction, calibration))
    }

    pub(crate) fn feedback(&self) -> Result<KnobFeedback> {
        let live = self.manager.status(self.node_id)?;
        validate_feedback(&live, self.epoch, self.segment, Instant::now())?;
        let measurements = live.measurements;
        let raw_rev = measurements
            .accumulated_position_rev
            .filter(|value| value.is_finite())
            .ok_or_else(|| anyhow!("SmartKnob continuous position is unavailable"))?;
        let velocity_rev_per_s = measurements
            .velocity_rev_per_s
            .filter(|value| value.is_finite())
            .ok_or_else(|| anyhow!("SmartKnob velocity is unavailable"))?;
        Ok(KnobFeedback {
            raw_rev,
            velocity_rev_per_s,
            torque_nm: measurements.torque_permille.map(|value| {
                (f64::from(value) * self.peak_torque_nm / 1000.0 / self.torque_factor) as f32
            }),
            driver_temp_c: measurements.driver_temp_c,
            motor_temp_c: measurements.motor_temp_c,
            online: true,
            enabled: true,
            error: None,
        })
    }

    pub(crate) async fn send_torque(&self, physical_nm: f64, max_permille: u16) -> Result<f64> {
        if self.stopped.load(Ordering::Acquire) {
            bail!("SmartKnob motor has been stopped");
        }
        self.feedback()?;
        let raw = raw_torque(
            physical_nm,
            self.torque_factor,
            self.peak_torque_nm,
            max_permille,
        )?;
        self.send_raw(raw, max_permille).await?;
        Ok(f64::from(raw) / self.torque_factor)
    }

    async fn send_raw(&self, raw_nm: f32, max_permille: u16) -> Result<()> {
        let frame = torque_frame(self.node_id, raw_nm, max_permille, self.brs)?;
        tokio::time::timeout(SEND_TIMEOUT, self.bus.send(frame))
            .await
            .context("SmartKnob RPDO send timed out")??;
        Ok(())
    }

    pub(crate) async fn stop(&self) -> Result<()> {
        self.stopped.store(true, Ordering::Release);
        // Keep the heartbeat until zero+disable has completed, then release it.
        // A subsequent Stop may retry disable after a transient bus failure.
        let zero = self.send_raw(0.0, 0).await;
        let disable = disable_motor(&self.manager, self.bus.as_ref(), self.node_id).await;
        let heartbeat = self.heartbeat.lock().unwrap().take();
        if let Some(heartbeat) = heartbeat {
            heartbeat.stop().await;
        }
        match disable {
            Ok(()) => Ok(()), // Confirmed Disabled is sufficient even if zero send failed.
            Err(error) => Err(with_cleanup(error, zero)),
        }
    }

    pub(crate) fn calibration(&self) -> KnobCalibration {
        self.calibration.clone()
    }

    pub(crate) fn physical_peak_torque_nm(&self) -> f64 {
        self.peak_torque_nm / self.torque_factor
    }

    pub(crate) fn friction_torque(
        &self,
        position_rad: f64,
        velocity_rad_s: f64,
        restoring_nm: f64,
        cap_nm: f64,
    ) -> f64 {
        if cap_nm <= 0.0 {
            self.reset_friction();
            return 0.0;
        }
        let now = Instant::now();
        let (intent, breakaway) = {
            let mut motion = self.motion.lock().unwrap();
            let intent = motion.update(position_rad, velocity_rad_s, now);
            (intent, motion.breakaway_scale(now))
        };
        self.friction
            .map(|friction| {
                calibrated_friction(
                    friction,
                    self.torque_factor,
                    velocity_rad_s,
                    intent,
                    restoring_nm,
                    cap_nm,
                    breakaway,
                )
            })
            .unwrap_or(0.0)
    }

    pub(crate) fn reset_friction(&self) {
        *self.motion.lock().unwrap() = FrictionMotion::default();
    }
}

fn startup_allowed(shutdown: &AtomicBool) -> Result<()> {
    if shutdown.load(Ordering::SeqCst) {
        bail!("SmartKnob startup cancelled by application shutdown");
    }
    Ok(())
}

/// Retry cleanup when startup failed before a full SmartKnob session existed.
/// This deliberately needs neither a valid RPDO mapping nor Initialized state.
/// The caller retains its recovery marker until disable is confirmed.
pub(crate) async fn recover_stop(
    manager: &MeowMotorManager,
    bus: &Arc<dyn CanBus>,
    node_id: u8,
) -> Result<()> {
    let zero = sdo::download_f32(bus.as_ref(), node_id, 0x4102, 3, 0.0, Some(SDO_TIMEOUT))
        .await
        .map_err(anyhow::Error::from);
    match disable_motor(manager, bus.as_ref(), node_id).await {
        Ok(()) => Ok(()),
        Err(error) => Err(with_cleanup(error, zero)),
    }
}

async fn disable_motor(manager: &MeowMotorManager, bus: &dyn CanBus, node_id: u8) -> Result<()> {
    match manager.disable(node_id).await {
        Ok(()) => Ok(()),
        Err(manager_error) => {
            // The manager rejects an offline/invalidated identity before sending
            // anything. Cleanup must still try the disable-only wire command;
            // this path never clears a fault or re-enables output.
            let fallback: Result<()> = async {
                sdo::download_u8(bus, node_id, 0x4401, 0, 0, Some(SDO_TIMEOUT)).await?;
                let mode = sdo::upload_u8(bus, node_id, 0x4402, 0, Some(SDO_TIMEOUT)).await?;
                if mode != 0 {
                    bail!("SmartKnob disable readback is 0x{mode:02X}, expected Disabled");
                }
                Ok(())
            }
            .await;
            fallback.with_context(|| format!("SmartKnob manager disable failed: {manager_error}"))
        }
    }
}

fn with_cleanup(error: anyhow::Error, cleanup: Result<()>) -> anyhow::Error {
    match cleanup {
        Ok(()) => error,
        Err(cleanup) => anyhow!("{error:#}; SmartKnob cleanup also failed: {cleanup:#}"),
    }
}

fn torque_recipe(node_id: u8) -> RpdoRecipe {
    RpdoRecipe {
        rpdo_index: 0,
        cob_id: 0x200 + u16::from(node_id),
        entries: vec![
            TpdoEntry {
                index: 0x4102,
                subindex: 3,
                bit_len: 32,
            },
            TpdoEntry {
                index: 0x4572,
                subindex: 0,
                bit_len: 16,
            },
        ],
        transmission_type: 255,
    }
}

fn torque_frame(node_id: u8, raw_nm: f32, max_permille: u16, brs: bool) -> Result<CanFrame> {
    let mut data = [0u8; 6];
    data[..4].copy_from_slice(&raw_nm.to_le_bytes());
    data[4..].copy_from_slice(&max_permille.to_le_bytes());
    let id = 0x200 + u16::from(node_id);
    Ok(if brs {
        CanFrame::new_fd(id, &data, true)?
    } else {
        CanFrame::new_data(id, &data)?
    })
}

fn raw_torque(physical_nm: f64, factor: f64, peak_nm: f64, max_permille: u16) -> Result<f32> {
    if !physical_nm.is_finite()
        || !factor.is_finite()
        || factor <= 0.0
        || !peak_nm.is_finite()
        || peak_nm <= 0.0
        || max_permille > 1000
    {
        bail!("invalid SmartKnob torque or output limit");
    }
    let raw = physical_nm * factor;
    if !raw.is_finite() {
        bail!("SmartKnob calibrated torque overflow");
    }
    let limit = peak_nm * f64::from(max_permille) / 1000.0;
    Ok(raw.clamp(-limit, limit) as f32)
}

fn calibration_values(
    factory: MeowFactoryCalibration,
) -> Result<(f64, Option<MeowFrictionCalibration>, KnobCalibration)> {
    let (factor, friction, detail) = match factory {
        MeowFactoryCalibration::Valid(value) => (
            value.torque.factor, value.friction,
            if value.friction.is_some() {
                "Factory torque factor and directional static/kinetic friction applied (0x4001 v1)".to_string()
            } else {
                "Factory torque factor applied; friction is not calibrated, so automatic friction compensation is disabled".to_string()
            },
        ),
        MeowFactoryCalibration::Missing { reason, .. } => (
            1.0, None, format!("No usable factory calibration ({reason}); factor 1.0, automatic friction compensation disabled"),
        ),
    };
    if !factor.is_finite() || factor <= 0.0 {
        bail!("invalid factory torque factor");
    }
    let view = KnobCalibration {
        torque_factor: factor,
        static_positive_nm: friction.map(|value| value.static_pos_raw_nm / factor),
        static_negative_nm: friction.map(|value| value.static_neg_raw_nm / factor),
        kinetic_positive_nm: friction.map(|value| value.kinetic_pos_raw_nm / factor),
        kinetic_negative_nm: friction.map(|value| value.kinetic_neg_raw_nm / factor),
        detail,
    };
    Ok((factor, friction, view))
}

fn validate_feedback(
    live: &MeowMotorLiveState,
    epoch: u64,
    segment: u64,
    now: Instant,
) -> Result<()> {
    if live.session_epoch != epoch || !live.connection.online {
        bail!("SmartKnob motor session changed or went offline");
    }
    if !matches!(live.lifecycle, MeowMotorLifecycle::Initialized) {
        bail!("SmartKnob motor needs initialization: {:?}", live.lifecycle);
    }
    if live.logic != Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit)) {
        bail!("SmartKnob motor left MIT mode: {:?}", live.logic);
    }
    if !live.measurements.position_accumulation_valid
        || live.measurements.position_accumulation_segment != segment
    {
        bail!("SmartKnob continuous position lost its baseline");
    }
    for (name, timestamp, timeout) in [
        ("TPDO1", live.connection.last_tpdo1, POSITION_TIMEOUT),
        ("TPDO2", live.connection.last_tpdo2, STATE_TIMEOUT),
    ] {
        if timestamp.is_none_or(|time| now.saturating_duration_since(time) > timeout) {
            bail!("SmartKnob {name} feedback is stale");
        }
    }
    Ok(())
}

/// Resolve slow hand motion before velocity feedback clears its noise floor.
/// A leaky, bounded displacement accumulator rejects encoder jitter, unwinds
/// on reversal and forgets old direction when released. All positions are
/// output-shaft radians; the driver already accounts for the 10:1 gearbox.
#[derive(Default)]
struct FrictionMotion {
    last: Option<(f64, Instant)>,
    displacement: f64,
    filtered_velocity: f64,
    net_travel: f64,
    total_travel: f64,
    breakaway_since: Option<Instant>,
    still_since: Option<Instant>,
}

impl FrictionMotion {
    fn update(&mut self, position: f64, velocity: f64, now: Instant) -> f64 {
        if !position.is_finite() || !velocity.is_finite() {
            *self = Self::default();
            return 0.0;
        }
        if let Some((previous, at)) = self.last {
            let dt = now.saturating_duration_since(at).as_secs_f64();
            if dt > 0.020 {
                self.displacement = 0.0;
                self.filtered_velocity = 0.0;
                self.net_travel = 0.0;
                self.total_travel = 0.0;
                self.breakaway_since = None;
                self.still_since = None;
            } else {
                let decay = (-dt / 0.040).exp();
                let delta = position - previous;
                self.displacement = (self.displacement * decay + delta).clamp(-0.0008, 0.0008);
                self.net_travel = self.net_travel * decay + delta;
                self.total_travel = self.total_travel * decay + delta.abs();
                // Raw velocity sign feedback can excite gearbox/cogging
                // rebounds. A 10 ms filter keeps slow hand motion intact
                // without chasing each high-frequency velocity reversal.
                self.filtered_velocity +=
                    (velocity - self.filtered_velocity) * (1.0 - (-dt / 0.010).exp());
            }
        } else {
            self.filtered_velocity = velocity;
        }
        self.last = Some((position, now));
        let positional =
            self.displacement.signum() * smoothstep((self.displacement.abs() - 0.0001) / 0.0005);
        let moving = smoothstep((self.filtered_velocity.abs() - 0.002) / 0.018);
        // A hand turn has net travel; a gearbox vibration mostly retraces
        // itself. Suppress assistance for repeated reversals without adding
        // drag to a steady, arbitrarily slow hand turn. This also requires
        // encoder motion before trusting a velocity spike at rest.
        let coherence = self.net_travel.abs() / self.total_travel.max(0.0001);
        let confidence = smoothstep((coherence - 0.2) / 0.6);
        let intent =
            ((1.0 - moving) * positional + moving * self.filtered_velocity.signum()) * confidence;
        // A cogging rebound or velocity zero crossing is not a fresh hand
        // breakaway. Rearming on every reversal injects energy into a free
        // shaft. Require 100 ms of genuinely quiet position AND velocity.
        if velocity.abs() <= 0.002
            && self.filtered_velocity.abs() <= 0.002
            && self.displacement.abs() <= 0.0001
        {
            let at = *self.still_since.get_or_insert(now);
            if now.saturating_duration_since(at) >= Duration::from_millis(100) {
                self.breakaway_since = None;
            }
        } else {
            self.still_since = None;
        }
        if intent.abs() > 0.1 && self.breakaway_since.is_none() {
            self.breakaway_since = Some(now);
        }
        intent
    }

    fn breakaway_scale(&self, now: Instant) -> f64 {
        self.breakaway_since
            .map(|at| (-now.saturating_duration_since(at).as_secs_f64() / 0.040).exp())
            .unwrap_or(1.0)
    }
}

/// Directional Stribeck compensation. At rest only an existing restoring
/// request can unlock stiction; a centered/free knob never receives a static
/// kick. Smooth transitions and under-compensation avoid a discontinuous sign
/// torque at zero speed. `cap_nm` remains the operator's physical-Nm ceiling.
/// Once hand motion is observed it takes priority over an opposing spring,
/// even below the old 0.02..0.12 rad/s velocity gate.
fn calibrated_friction(
    friction: MeowFrictionCalibration,
    factor: f64,
    velocity: f64,
    motion_intent: f64,
    restoring: f64,
    cap_nm: f64,
    breakaway_scale: f64,
) -> f64 {
    if !velocity.is_finite()
        || !motion_intent.is_finite()
        || !breakaway_scale.is_finite()
        || !restoring.is_finite()
        || !cap_nm.is_finite()
        || cap_nm <= 0.0
        || !factor.is_finite()
        || factor <= 0.0
    {
        return 0.0;
    }
    let static_for = |direction: f64| {
        if direction >= 0.0 {
            friction.static_pos_raw_nm
        } else {
            friction.static_neg_raw_nm
        }
    };
    let speed = velocity.abs();
    let intent = motion_intent.clamp(-1.0, 1.0);
    let movement = intent.abs();
    let static_activation = if restoring.abs() <= 0.001 {
        0.0
    } else {
        ((restoring.abs() - 0.001) / 0.02).tanh() * restoring.signum()
    };
    let at_rest = STATIC_FRICTION_RATIO * static_for(restoring) * static_activation;
    let kinetic = if intent >= 0.0 {
        friction.kinetic_pos_raw_nm
    } else {
        friction.kinetic_neg_raw_nm
    };
    let transition_speed = (friction.kinetic_reference_speed_rad_per_s * 0.15).max(0.05);
    // Static friction is a breakaway aid, not a sustainable propulsive torque.
    // Taper its excess over kinetic friction after motion starts from rest;
    // a velocity-only Stribeck curve can otherwise sustain a slow free spin.
    let kinetic_assist = KINETIC_FRICTION_RATIO * kinetic;
    let moving = intent.signum()
        * (kinetic_assist
            + (STATIC_FRICTION_RATIO * static_for(intent) - kinetic_assist)
                * (-(speed / transition_speed).powi(2)).exp()
                * breakaway_scale.clamp(0.0, 1.0));
    // Do not keep a breakaway boost pointed against an observed hand turn.
    // Fade that term early; the moving assist still grows continuously with
    // confidence, rather than jumping straight to full static friction.
    let static_weight = if intent * restoring < 0.0 {
        1.0 - smoothstep(movement / 0.1)
    } else {
        1.0 - movement
    };
    let physical = (static_weight * at_rest + movement * moving) / factor;
    physical.clamp(-cap_nm, cap_nm)
}

fn smoothstep(value: f64) -> f64 {
    let x = value.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physical_peak_matches_motor_control_and_rejects_unknown_specs() {
        assert_eq!(physical_peak_torque(Some(2.5), 0.5).unwrap(), 5.0);
        assert_eq!(physical_peak_torque(Some(2.5), 2.0).unwrap(), 1.25);
        assert!(physical_peak_torque(None, 1.0).is_err());
        for raw in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(physical_peak_torque(Some(raw), 1.0).is_err());
        }
        for factor in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(physical_peak_torque(Some(2.5), factor).is_err());
        }
    }

    fn friction() -> MeowFrictionCalibration {
        MeowFrictionCalibration {
            static_pos_raw_nm: 0.2,
            static_neg_raw_nm: 0.3,
            kinetic_pos_raw_nm: 0.1,
            kinetic_neg_raw_nm: 0.15,
            kinetic_reference_speed_rad_per_s: 1.0,
            calibration_temperature_c: 25.0,
        }
    }

    #[test]
    fn static_compensation_requires_restoring_intent_and_uses_each_direction() {
        assert_eq!(
            calibrated_friction(friction(), 2.0, 0.0, 0.0, 0.0, 1.0, 1.0),
            0.0
        );
        assert_eq!(
            calibrated_friction(friction(), 2.0, 0.01, 0.0, 0.0, 1.0, 1.0),
            0.0
        );
        let positive = calibrated_friction(friction(), 2.0, 0.0, 0.0, 0.2, 1.0, 1.0);
        let negative = calibrated_friction(friction(), 2.0, 0.0, 0.0, -0.2, 1.0, 1.0);
        assert!((positive - 0.09).abs() < 1e-6);
        assert!((negative + 0.135).abs() < 1e-6);
        assert_eq!(
            calibrated_friction(friction(), 2.0, 0.0, 0.0, 0.2, 0.03, 1.0),
            0.03
        );
        assert_eq!(
            calibrated_friction(friction(), 2.0, 0.0, 0.0, 0.2, 0.0, 1.0),
            0.0
        );
    }

    #[test]
    fn moving_compensation_converges_to_directional_kinetic_friction() {
        assert!(
            (calibrated_friction(friction(), 2.0, 1.0, 1.0, -0.1, 1.0, 1.0) - 0.0375).abs() < 1e-6
        );
        assert!(
            (calibrated_friction(friction(), 2.0, -1.0, -1.0, 0.1, 1.0, 1.0) + 0.05625).abs()
                < 1e-6
        );
        let now = Instant::now();
        for boundary in [-0.02, -0.002, 0.002, 0.02] {
            let sample = |velocity| {
                let intent = FrictionMotion::default().update(0.0, velocity, now);
                calibrated_friction(friction(), 2.0, velocity, intent, 0.05, 1.0, 1.0)
            };
            let before = sample(boundary - 1e-7);
            let after = sample(boundary + 1e-7);
            assert!((after - before).abs() < 1e-5);
        }
    }

    #[test]
    fn slow_hand_motion_overrides_opposing_spring_and_reverses() {
        let start = Instant::now();
        let mut motion = FrictionMotion::default();
        let mut position = 0.0;
        for direction in [1.0, -1.0] {
            for tick in 0..200 {
                position += direction * 0.01 * 0.001;
                let elapsed = if direction > 0.0 { tick } else { 200 + tick };
                let intent = motion.update(
                    position,
                    direction * 0.01,
                    start + Duration::from_millis(elapsed),
                );
                let torque = calibrated_friction(
                    friction(),
                    2.0,
                    direction * 0.01,
                    intent,
                    -direction * 0.2,
                    0.15,
                    motion.breakaway_scale(start + Duration::from_millis(elapsed)),
                );
                if tick > 80 {
                    assert!(
                        torque * direction > 0.025,
                        "slow motion should assist the hand: {torque}"
                    );
                    assert!(torque.abs() <= 0.135);
                }
            }
        }
        for tick in 400..700 {
            let intent = motion.update(position, 0.0, start + Duration::from_millis(tick));
            let torque = calibrated_friction(
                friction(),
                2.0,
                0.0,
                intent,
                0.0,
                0.15,
                motion.breakaway_scale(start + Duration::from_millis(tick)),
            );
            if tick > 600 {
                assert_eq!(torque, 0.0);
            }
        }
    }

    #[test]
    fn encoder_jitter_cannot_arm_static_assistance() {
        let now = Instant::now();
        let mut motion = FrictionMotion::default();
        for tick in 0..1000 {
            let noise = if tick % 2 == 0 { 0.00002 } else { -0.00002 };
            let intent = motion.update(noise, noise * 10.0, now + Duration::from_millis(tick));
            assert_eq!(intent, 0.0);
            assert_eq!(
                calibrated_friction(friction(), 2.0, 0.0, intent, 0.0, 0.15, 1.0),
                0.0
            );
        }
        motion.update(0.001, 0.0, now + Duration::from_millis(1000));
        assert_eq!(motion.update(0.001, 0.0, now + Duration::from_secs(2)), 0.0);
    }

    #[test]
    fn displacement_detects_motion_even_with_zero_velocity_readback() {
        let now = Instant::now();
        let mut motion = FrictionMotion::default();
        let mut intent = 0.0;
        for tick in 0..100 {
            intent = motion.update(
                tick as f64 * 0.00001,
                0.0,
                now + Duration::from_millis(tick),
            );
        }
        let assist = calibrated_friction(friction(), 2.0, 0.0, intent, -0.2, 0.15, 1.0);
        assert!(
            assist > 0.04,
            "encoder motion should beat an opposing spring: {assist}"
        );
    }

    #[test]
    fn transient_velocity_reversal_does_not_flip_full_friction_assist() {
        let now = Instant::now();
        let mut motion = FrictionMotion::default();
        for tick in 0..100 {
            motion.update(
                tick as f64 * 0.00005,
                0.05,
                now + Duration::from_millis(tick),
            );
        }
        let rebound = motion.update(0.0049, -0.05, now + Duration::from_millis(100));
        assert!(
            rebound > 0.0,
            "a brief rebound should not receive full accelerating torque"
        );
        let mut reversed = rebound;
        for tick in 1..100 {
            reversed = motion.update(
                0.0049 - tick as f64 * 0.00005,
                -0.05,
                now + Duration::from_millis(100 + tick),
            );
        }
        assert_eq!(
            reversed, -1.0,
            "a sustained hand reversal must receive assistance"
        );
    }

    #[test]
    fn repeated_gearbox_rebounds_lose_assistance_but_a_steady_turn_keeps_it() {
        let now = Instant::now();
        let mut shaking = FrictionMotion::default();
        let mut turning = FrictionMotion::default();
        for tick in 0..1000 {
            let seconds = tick as f64 / 1000.0;
            let phase = seconds * std::f64::consts::TAU * 20.0;
            let at = now + Duration::from_millis(tick);
            let shake_intent = shaking.update(
                0.002 * phase.sin(),
                0.002 * std::f64::consts::TAU * 20.0 * phase.cos(),
                at,
            );
            let hand_intent = turning.update(0.01 * seconds, 0.01, at);
            if tick > 300 {
                assert!(
                    shake_intent.abs() < 0.2,
                    "rebound assist must fade: {shake_intent}"
                );
                assert!(
                    hand_intent > 0.7,
                    "slow monotonic hand motion must stay light"
                );
            }
        }
    }

    #[test]
    fn moving_static_boost_expires_and_rebounds_cannot_rearm_it() {
        let now = Instant::now();
        let mut motion = FrictionMotion::default();
        for tick in 0..500 {
            let at = now + Duration::from_millis(tick);
            let intent = motion.update(tick as f64 * 0.00003, 0.03, at);
            let scale = motion.breakaway_scale(at);
            if tick == 0 {
                assert_eq!(scale, 1.0);
            }
            let assist = calibrated_friction(friction(), 2.0, 0.03, intent, 0.0, 0.15, scale);
            if tick > 400 {
                assert!(
                    (assist - 0.0375).abs() < 0.00001,
                    "sustained motion must not keep static-friction torque: {assist}"
                );
            }
        }
        let reversed = now + Duration::from_millis(500);
        motion.update(0.0149, -0.03, reversed);
        assert!(motion.breakaway_scale(reversed) < 0.00001);
        for tick in 501..900 {
            motion.update(0.0149, 0.0, now + Duration::from_millis(tick));
        }
        assert!(motion.breakaway_since.is_none());
        let restart = now + Duration::from_millis(900);
        for tick in 0..20 {
            motion.update(
                0.015 + tick as f64 * 0.00003,
                0.03,
                restart + Duration::from_millis(tick),
            );
        }
        assert!(motion.breakaway_since.is_some_and(|at| at >= restart));
    }

    #[test]
    fn calibration_is_applied_once_and_raw_safety_limit_is_preserved() {
        // A 0.1 raw-Nm friction term is not multiplied by factor a second time.
        assert!((raw_torque(0.3 + 0.1 / 2.0, 2.0, 10.0, 1000).unwrap() - 0.7).abs() < 1e-6);
        assert_eq!(raw_torque(100.0, 2.0, 10.0, 50).unwrap(), 0.5);
        assert_eq!(raw_torque(-100.0, 2.0, 10.0, 50).unwrap(), -0.5);
        assert_eq!(raw_torque(1.0, 2.0, 10.0, 0).unwrap(), 0.0);
        assert!(raw_torque(f64::NAN, 1.0, 10.0, 100).is_err());
        assert!(raw_torque(1.0, 0.0, 10.0, 100).is_err());
    }

    #[test]
    fn new_protocol_frame_has_float_torque_and_raw_output_ceiling() {
        let recipe = torque_recipe(1);
        assert_eq!(recipe.cob_id, 0x201);
        assert_eq!(recipe.entries[0].index, 0x4102);
        assert_eq!(recipe.entries[1].index, 0x4572);
        let frame = torque_frame(1, -0.125, 125, false).unwrap();
        assert_eq!(frame.data(), &[0, 0, 0, 0xBE, 125, 0]);
        assert_eq!(frame.kind(), can_transport::FrameKind::Data);
        assert_eq!(
            torque_frame(1, 0.0, 0, true).unwrap().kind(),
            can_transport::FrameKind::Fd { brs: true }
        );
    }

    #[test]
    fn feedback_fences_session_position_segment_mode_and_freshness() {
        let now = Instant::now();
        let mut live = MeowMotorLiveState::empty(now);
        live.session_epoch = 7;
        live.lifecycle = MeowMotorLifecycle::Initialized;
        live.logic = Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit));
        live.connection.online = true;
        live.connection.last_tpdo1 = Some(now);
        live.connection.last_tpdo2 = Some(now);
        live.measurements.position_accumulation_valid = true;
        live.measurements.position_accumulation_segment = 3;
        assert!(validate_feedback(&live, 7, 3, now).is_ok());
        assert!(validate_feedback(&live, 8, 3, now).is_err());
        assert!(validate_feedback(&live, 7, 4, now).is_err());
        assert!(validate_feedback(
            &live,
            7,
            3,
            now + POSITION_TIMEOUT + Duration::from_millis(1)
        )
        .is_err());
        live.logic = Some(MeowMotorLogic::Disabled);
        assert!(validate_feedback(&live, 7, 3, now).is_err());
    }
}
