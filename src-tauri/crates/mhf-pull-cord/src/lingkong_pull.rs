//! MHF5010 pull-cord controller using Lingkong's classic-CAN current and speed modes.
//! Commands and feedback remain in protocol counts. Conflicting manuals and
//! observed saturation do not establish either scale's physical torque.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::pull_cord::{
    A1CurrentConfig, A2SpeedConfig, Phase, PullCordConfig, PullCordMode, PullCordState, Zone,
    A2_PROTOCOL_MAX_RPM,
};
use can_transport::{CanBus, CanFilter, CanFrame, CanIoError, CanRx, FrameKind};
use magnetic_encoder::EncoderMonitor;
use tokio::task::JoinHandle;
use tokio::time::MissedTickBehavior;

use crate::runtime_support::{CordMotionReceiver, MotionPublisher, PullCordCommandDiagnostics};
use crate::trajectory::{CordTrajectory, DirectionSample, TrajectoryRecorder};

const PERIOD: Duration = Duration::from_millis(5);
const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(150);
const STATUS_TIMEOUT: Duration = Duration::from_millis(400);
const SETUP_TIMEOUT: Duration = Duration::from_millis(300);
const SEND_TIMEOUT: Duration = Duration::from_millis(20);
const SHUTDOWN_REPLY_TIMEOUT: Duration = Duration::from_millis(100);
const TURNAROUND_DELAY: Duration = Duration::from_millis(2);
const READ_ATTEMPTS: usize = 3;
const PEAK_TORQUE_NM: f64 = 0.53;
const HOMING_STALL_SPEED_M_S: f64 = 0.02;
const HOMING_STALL_HOLD: Duration = Duration::from_millis(600);

/// Read the outlet direction for telemetry and endpoint mapping.
/// Shared slots also follow reconnects, independently of UI/chassis operation.
#[derive(Clone, Default)]
pub struct PullCordEncoder {
    pub monitor: Arc<tokio::sync::Mutex<Option<EncoderMonitor>>>,
    pub center_deg: Arc<Mutex<f64>>,
}

impl PullCordEncoder {
    async fn direction(&self) -> Option<DirectionSample> {
        let snapshot = self.monitor.lock().await.as_ref()?.snapshot();
        if !snapshot.angle_is_usable()
            || snapshot
                .position_age
                .is_none_or(|age| age > FEEDBACK_TIMEOUT)
        {
            return None;
        }
        let center = *self.center_deg.lock().unwrap();
        let angle = snapshot.position?.angle_degrees;
        let at = snapshot.position_received_at?;
        let valid_until = (at + FEEDBACK_TIMEOUT).min(snapshot.status_valid_until?);
        (angle.is_finite() && center.is_finite()).then(|| DirectionSample {
            offset_deg: crate::signed_degrees(angle - center),
            center_deg: center,
            at,
            valid_until,
        })
    }
}

/// Keep the legacy model's defaults intact for historical data/tests while
/// giving the direct-drive assembly neutral compensation and gentle homing.
#[cfg(test)]
pub fn default_config() -> PullCordConfig {
    let mut config = PullCordConfig::default();
    config.zero_friction_assist_nm = 0.0;
    config.alert_pulse_nm = 0.0;
    config.homing_torque_nm = 0.06;
    normalize_mhf_config(&mut config);
    config
}

pub(crate) fn normalize_mhf_config(config: &mut PullCordConfig) {
    if config.a1_current.is_none() {
        config.a1_current = Some(A1CurrentConfig::from_legacy(config));
    }
    if config.a2_speed.is_none() {
        if config.control_mode == PullCordMode::A2Speed {
            A2SpeedConfig::normalize_legacy_speeds(config);
        }
        config.a2_speed = Some(A2SpeedConfig::from_legacy(config));
    }
    config.spring_compensation.enabled = false;
    config.friction_compensation.mode = crate::pull_cord::FrictionMode::Off;
    // The magnetic handle can already be at the stop on startup. Apply its
    // settling criterion even to saved templates with the former 400 ms hold.
    config.homing_stall_speed_m_s = HOMING_STALL_SPEED_M_S;
    config.homing_stall_hold_ms = HOMING_STALL_HOLD.as_millis() as u64;
}

#[derive(Default)]
struct HomingStall {
    since: Option<Instant>,
    last_sample: Option<Instant>,
    observed_for: Duration,
}

impl HomingStall {
    fn observe(&mut self, speed_m_s: f64, now: Instant) {
        if self
            .last_sample
            .is_some_and(|at| now.duration_since(at) > FEEDBACK_TIMEOUT)
        {
            self.since = None;
        }
        self.last_sample = Some(now);
        self.observed_for = if speed_m_s.abs() < HOMING_STALL_SPEED_M_S {
            now.duration_since(*self.since.get_or_insert(now))
        } else {
            self.since = None;
            Duration::ZERO
        };
    }

    fn confirmed(&self) -> bool {
        self.observed_for >= HOMING_STALL_HOLD
    }

    fn progress(&self) -> f64 {
        (self.observed_for.as_secs_f64() / HOMING_STALL_HOLD.as_secs_f64()).min(1.0)
    }
}

pub(crate) fn validate_mhf_config(config: &PullCordConfig) -> Result<(), String> {
    config.validate()?;
    let maximum_speed_m_s = A2_PROTOCOL_MAX_RPM / 60.0 * config.metres_per_rev();
    if config.homing_speed_m_s > maximum_speed_m_s {
        return Err(format!(
            "MHF5010 homing speed target must not exceed the {A2_PROTOCOL_MAX_RPM} rpm protocol limit ({maximum_speed_m_s:.3} m/s at this drum radius)"
        ));
    }
    if config.control_mode == PullCordMode::A2Speed && config.retract_speed_m_s > maximum_speed_m_s
    {
        return Err(format!(
            "MHF5010 retract speed target must not exceed the {A2_PROTOCOL_MAX_RPM} rpm protocol limit ({maximum_speed_m_s:.3} m/s at this drum radius)"
        ));
    }
    Ok(())
}

fn can_id(node: u8) -> u16 {
    0x140 + u16::from(node)
}

fn command(code: u8) -> [u8; 8] {
    let mut bytes = [0; 8];
    bytes[0] = code;
    bytes
}

impl PullCordMode {
    fn opcode(self) -> u8 {
        match self {
            Self::A1Current => 0xA1,
            Self::A2Speed => 0xA2,
        }
    }
}

fn a1_command(current_raw: i16) -> [u8; 8] {
    let mut bytes = command(0xA1);
    bytes[4..6].copy_from_slice(&current_raw.to_le_bytes());
    bytes
}

fn a2_command(limit_raw: u16, speed_raw: i32) -> [u8; 8] {
    let mut bytes = command(0xA2);
    bytes[2..4].copy_from_slice(&limit_raw.to_le_bytes());
    bytes[4..8].copy_from_slice(&speed_raw.to_le_bytes());
    bytes
}

/// This drive enforces the RAM limit even when it ignores A2's limit field.
fn global_torque_command(limit_raw: u16) -> [u8; 8] {
    let mut bytes = command(0xC1);
    bytes[1] = 0x1E;
    bytes[4..6].copy_from_slice(&limit_raw.to_le_bytes());
    bytes
}

fn speed_raw(speed_m_s: f64, config: &PullCordConfig) -> i32 {
    (speed_m_s / config.metres_per_rev() * 36000.0).round() as i32
}

/// V2.36 pp. 14–15: volatile RAM settings, reapplied on every start.
fn ram_settings(config: &PullCordConfig) -> [[u8; 8]; 3] {
    let a2 = config.a2_speed.expect("normalized A2 config");
    let mut pid = command(0xC1);
    pid[1] = 0x0B;
    pid[2..4].copy_from_slice(&a2.speed_kp.to_le_bytes());
    pid[4..6].copy_from_slice(&a2.speed_ki.to_le_bytes());
    pid[6..8].copy_from_slice(&a2.speed_kd.to_le_bytes());
    let parameter = |id, value: i32| {
        let mut bytes = command(0xC1);
        bytes[1] = id;
        bytes[4..8].copy_from_slice(&value.to_le_bytes());
        bytes
    };
    [
        pid,
        parameter(
            0x20,
            speed_raw(
                config.homing_speed_m_s.max(config.retract_speed_m_s),
                config,
            ),
        ),
        parameter(0x26, a2.speed_ramp_dps2 as i32),
    ]
}

async fn configure_ram(
    bus: &dyn CanBus,
    rx: &mut MotorReplies,
    node: u8,
    config: &PullCordConfig,
    stop: &AtomicBool,
) -> Result<(), String> {
    for bytes in ram_settings(config) {
        if stop.load(Ordering::Acquire) {
            return Err("pull-cord configuration cancelled".into());
        }
        exchange(bus, rx, node, bytes, SEND_TIMEOUT).await?;
    }
    if stop.load(Ordering::Acquire) {
        return Err("pull-cord configuration cancelled".into());
    }
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct Status1 {
    temp_c: i8,
    voltage_v: f64,
    state: u8,
    error: u8,
}

fn parse_status1(data: &[u8]) -> Option<Status1> {
    (data.len() == 8 && data[0] == 0x9A).then(|| Status1 {
        temp_c: data[1] as i8,
        voltage_v: f64::from(u16::from_le_bytes([data[2], data[3]])) / 100.0,
        state: data[6],
        error: data[7],
    })
}

// The V2.36 CAN manual lists 0x10/0x00 for closed/running. The connected
// MHF5010 returned 0x20 after Close and 0x30 after Run, with no error bit and
// the documented slow-blinking LED while closed. Keep both observed pairs
// explicit; the meaning of other state values is unknown.
fn running_state_for_closed(closed: u8) -> Option<u8> {
    match closed {
        0x10 => Some(0x00),
        0x20 => Some(0x30),
        _ => None,
    }
}

#[derive(Debug, Clone, Copy)]
struct Status2 {
    temp_c: i8,
    iq_raw: i16,
    speed_dps: i16,
}

fn parse_status2(data: &[u8], mode: PullCordMode) -> Option<Status2> {
    (data.len() == 8 && data[0] == mode.opcode()).then(|| Status2 {
        temp_c: data[1] as i8,
        iq_raw: i16::from_le_bytes([data[2], data[3]]),
        speed_dps: i16::from_le_bytes([data[4], data[5]]),
    })
}

/// 0x92 returns a signed 56-bit, little-endian angle in 0.01 degree units.
fn parse_angle(data: &[u8]) -> Option<i64> {
    if data.len() != 8 || data[0] != 0x92 {
        return None;
    }
    let mut bytes = [0; 8];
    bytes[..7].copy_from_slice(&data[1..]);
    bytes[7] = if data[7] & 0x80 != 0 { 0xFF } else { 0 };
    Some(i64::from_le_bytes(bytes))
}

async fn send(bus: &dyn CanBus, node: u8, bytes: [u8; 8]) -> Result<(), String> {
    let frame = CanFrame::new_data(can_id(node), &bytes).map_err(|e| e.to_string())?;
    tokio::time::timeout(SEND_TIMEOUT, bus.send(frame))
        .await
        .map_err(|_| "CAN send timed out".to_string())?
        .map_err(|e| e.to_string())
}

/// Keep the drive's processing interval across transactions, including the
/// handover from startup to the control loop. Idle time already satisfies it.
struct MotorReplies {
    rx: Box<dyn CanRx>,
    next_request_at: tokio::time::Instant,
}

impl MotorReplies {
    async fn subscribe(bus: &dyn CanBus, node: u8) -> Result<Self, String> {
        let rx = bus
            .subscribe(CanFilter::exact_standard(can_id(node)))
            .await
            .map_err(|e| e.to_string())?;
        Ok(Self {
            rx,
            next_request_at: tokio::time::Instant::now() + TURNAROUND_DELAY,
        })
    }
}

/// Attempt every shutdown command even if an earlier exchange failed, then
/// confirm the drive is actually closed. The replies share the request CAN ID.
pub(crate) async fn disable_motor(
    bus: &dyn CanBus,
    node: u8,
    mode: PullCordMode,
) -> Result<(), String> {
    let mut errors = Vec::new();
    let mut rx = match MotorReplies::subscribe(bus, node).await {
        Ok(rx) => Some(rx),
        Err(error) => {
            errors.push(format!("CAN receive subscription: {error}"));
            None
        }
    };
    let zero_limit = (mode == PullCordMode::A2Speed).then(|| global_torque_command(0));
    for bytes in
        zero_limit
            .into_iter()
            .chain([command(mode.opcode()), command(0x81), command(0x80)])
    {
        let result = if let Some(rx) = rx.as_mut() {
            let timeout = if bytes[0] == 0xC1 {
                SEND_TIMEOUT
            } else {
                SHUTDOWN_REPLY_TIMEOUT
            };
            exchange(bus, rx, node, bytes, timeout).await.map(|_| ())
        } else {
            tokio::time::sleep(TURNAROUND_DELAY).await;
            send(bus, node, bytes).await
        };
        if let Err(error) = result {
            errors.push(format!("0x{:02X}: {error}", bytes[0]));
        }
    }
    if let Some(rx) = rx.as_mut() {
        match exchange(bus, rx, node, command(0x9A), SHUTDOWN_REPLY_TIMEOUT).await {
            Ok(bytes) => match parse_status1(&bytes) {
                Some(status)
                    if running_state_for_closed(status.state).is_some() && status.error == 0 => {}
                Some(status) => errors.push(format!(
                    "drive did not confirm closed state: 0x{:02X}, error 0x{:02X}",
                    status.state, status.error
                )),
                None => errors.push("invalid 0x9A shutdown status".into()),
            },
            Err(error) => errors.push(format!("0x9A: {error}")),
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "MHF shutdown CAN send failed: {}",
            errors.join("; ")
        ))
    }
}

pub struct LingkongPull {
    node_id: u8,
    bus: Arc<dyn CanBus>,
    state: Arc<Mutex<PullCordState>>,
    config: Arc<Mutex<PullCordConfig>>,
    stop: Arc<AtomicBool>,
    rehome: Arc<AtomicBool>,
    task: Option<JoinHandle<()>>,
    motion: CordMotionReceiver,
    trajectory: CordTrajectory,
    diagnostics: Arc<Mutex<PullCordCommandDiagnostics>>,
}

impl LingkongPull {
    pub(crate) async fn start(
        bus: Arc<dyn CanBus>,
        node_id: u8,
        mut config: PullCordConfig,
        diagnostics: Arc<Mutex<PullCordCommandDiagnostics>>,
        stop: Arc<AtomicBool>,
        encoder: PullCordEncoder,
    ) -> Result<Self, String> {
        if !(1..=32).contains(&node_id) {
            return Err("Lingkong motor ID must be 1..32".into());
        }
        // MHF5010 has neither the old spring nor the HEX4310 factory friction table.
        normalize_mhf_config(&mut config);
        validate_mhf_config(&config)?;
        let mode = config.control_mode;
        let mut replies = MotorReplies::subscribe(bus.as_ref(), node_id).await?;
        let status = request_reply(bus.as_ref(), &mut replies, node_id, 0x9A).await?;
        let status = parse_status1(&status).ok_or("invalid MHF status response")?;
        if !(7.4..=32.0).contains(&status.voltage_v) {
            tokio::time::sleep(TURNAROUND_DELAY).await;
            let _ = send(bus.as_ref(), node_id, command(0x80)).await;
            return Err(format!("unexpected MHF status: {status:?}"));
        }
        if status.error != 0 {
            tokio::time::sleep(TURNAROUND_DELAY).await;
            let _ = send(bus.as_ref(), node_id, command(0x80)).await;
            return Err(format!(
                "MHF motor error 0x{:02X}; clear it on the drive before starting",
                status.error
            ));
        }
        if !matches!(status.state, 0 | 0x10 | 0x20 | 0x30) {
            log::warn!(
                "MHF node {node_id} reported undocumented initial state 0x{:02X}; trying 0x80 Close before Run",
                status.state
            );
        }
        let setup = async {
            // Each command and its reply use the same CAN ID. Wait for the
            // echoed Close/Run reply before sending the next request; otherwise
            // the host can transmit 0x9A while the drive transmits the echo.
            request_reply(bus.as_ref(), &mut replies, node_id, 0x80).await?;
            let closed = request_reply(bus.as_ref(), &mut replies, node_id, 0x9A).await?;
            let expected_running = running_state_for_closed(closed[6]);
            if expected_running.is_none() || closed[7] != 0 {
                return Err(format!(
                    "MHF motor did not confirm closed state after 0x80: initial 0x{:02X}, now 0x{:02X}, error 0x{:02X}",
                    status.state, closed[6], closed[7]
                ));
            }
            if mode == PullCordMode::A2Speed {
                exchange(bus.as_ref(), &mut replies, node_id, global_torque_command(0), SEND_TIMEOUT).await?;
                configure_ram(bus.as_ref(), &mut replies, node_id, &config, &stop).await?;
            }
            if stop.load(Ordering::Acquire) {
                return Err("pull-cord start cancelled".into());
            }
            request_reply(bus.as_ref(), &mut replies, node_id, 0x88).await?;
            let running = request_reply(bus.as_ref(), &mut replies, node_id, 0x9A).await?;
            if Some(running[6]) != expected_running || running[7] != 0 {
                return Err(format!(
                    "MHF motor did not confirm running state: closed 0x{:02X}, now 0x{:02X}, error 0x{:02X}",
                    closed[6], running[6], running[7]
                ));
            }
            let mut zero = request_reply(bus.as_ref(), &mut replies, node_id, mode.opcode()).await?;
            let mut feedback_at = Instant::now();
            let angle = request_reply(bus.as_ref(), &mut replies, node_id, 0x92).await?;
            let angle = parse_angle(&angle).ok_or("invalid 0x92 angle response".to_string())?;
            // An angle read can succeed on a retry after the first speed sample
            // expired. Get fresh feedback at zero output before starting homing.
            if feedback_at.elapsed() > FEEDBACK_TIMEOUT {
                zero = request_reply(bus.as_ref(), &mut replies, node_id, mode.opcode()).await?;
                feedback_at = Instant::now();
            }
            let feedback = parse_status2(&zero, mode).ok_or("invalid MHF motion response")?;
            // A second subscriber processes feedback while the transaction
            // subscriber retains the last reply's turnaround deadline.
            let rx = bus
                .subscribe(CanFilter::exact_standard(can_id(node_id)))
                .await
                .map_err(|error| error.to_string())?;
            Ok((angle, (feedback, feedback_at), running[6], rx))
        }
        .await;
        let (initial_angle, initial_feedback, running_state, rx) = match setup {
            Ok(value) => value,
            Err(error) => {
                // A lost 0x9A reply does not prove Run was rejected. Close
                // even when a setup step failed before it could be verified.
                return Err(match disable_motor(bus.as_ref(), node_id, mode).await {
                    Ok(()) => error,
                    Err(stop_error) => format!("{error}; {stop_error}"),
                });
            }
        };
        if stop.load(Ordering::Acquire) {
            return Err(match disable_motor(bus.as_ref(), node_id, mode).await {
                Ok(()) => "pull-cord start cancelled".into(),
                Err(stop_error) => format!("pull-cord start cancelled; {stop_error}"),
            });
        }

        let state = Arc::new(Mutex::new(PullCordState::idle(config)));
        {
            let mut state = state.lock().unwrap();
            state.running = true;
            state.node_id = Some(node_id);
            state.phase = Phase::Homing;
            state.online = true;
            state.motor_temp_c = Some(f32::from(status.temp_c));
            state.bus_voltage_v = Some(status.voltage_v);
            state.peak_torque_nm = Some(PEAK_TORQUE_NM as f32);
            state.global_torque_limit_raw = (mode == PullCordMode::A2Speed).then_some(0);
        }
        let run_id = {
            let mut d = diagnostics.lock().unwrap();
            let run_id = d.run_id.wrapping_add(1).max(1);
            *d = PullCordCommandDiagnostics {
                active: true,
                node_id: Some(node_id),
                run_id,
                ..Default::default()
            };
            run_id
        };
        let config = Arc::new(Mutex::new(config));
        let rehome = Arc::new(AtomicBool::new(false));
        let (motion_tx, motion) = MotionPublisher::new();
        let (recorder, trajectory) =
            TrajectoryRecorder::new(Instant::now(), stop.clone(), rehome.clone());
        let task = tokio::spawn(run_loop(Loop {
            bus: bus.clone(),
            rx,
            ack_rx: replies,
            node_id,
            state: state.clone(),
            config: config.clone(),
            stop: stop.clone(),
            rehome: rehome.clone(),
            diagnostics: diagnostics.clone(),
            motion: motion_tx,
            trajectory: recorder,
            initial_angle,
            initial_feedback,
            running_state,
            run_id,
            encoder,
        }));
        Ok(Self {
            node_id,
            bus,
            state,
            config,
            stop,
            rehome,
            task: Some(task),
            motion,
            trajectory,
            diagnostics,
        })
    }

    pub fn node_id(&self) -> u8 {
        self.node_id
    }
    pub fn snapshot(&self) -> PullCordState {
        self.state.lock().unwrap().clone()
    }
    pub fn subscribe_motion(&self) -> CordMotionReceiver {
        self.motion.clone()
    }
    /// Per-run endpoint feedback for local consumers, independent of GUI polling.
    pub fn trajectory(&self) -> CordTrajectory {
        self.trajectory.clone()
    }
    pub fn set_config(&self, mut config: PullCordConfig) -> Result<(), String> {
        if self.state.lock().unwrap().phase == Phase::Fault
            || self.task.as_ref().is_none_or(|task| task.is_finished())
        {
            return Err("pull-cord control is stopped or faulted; stop the session and start again before applying live settings".into());
        }
        normalize_mhf_config(&mut config);
        validate_mhf_config(&config)?;
        let mut current = self.config.lock().unwrap();
        crate::pull_cord::validate_live_config(&current, &config)?;
        *current = config;
        // The loop publishes applied config only after RAM writes are acknowledged.
        Ok(())
    }
    pub fn rehome(&self) {
        self.rehome.store(true, Ordering::Release);
    }
    pub async fn stop(&mut self) -> Result<(), String> {
        self.stop.store(true, Ordering::Release);
        let task_error = if let Some(task) = self.task.take() {
            task.await
                .err()
                .map(|error| format!("MHF control task failed: {error}"))
        } else {
            None
        };
        let mode = self.state.lock().unwrap().config.control_mode;
        let stop_error = disable_motor(self.bus.as_ref(), self.node_id, mode)
            .await
            .err();
        self.diagnostics.lock().unwrap().active = false;
        let result = match (task_error, stop_error) {
            (None, None) => Ok(()),
            (Some(error), None) | (None, Some(error)) => Err(error),
            (Some(task), Some(stop)) => Err(format!("{task}; {stop}")),
        };
        if let Err(error) = &result {
            let mut state = self.state.lock().unwrap();
            state.phase = Phase::Fault;
            state.fault = Some(error.clone());
            state.online = false;
        }
        result
    }
}

async fn request_reply(
    bus: &dyn CanBus,
    rx: &mut MotorReplies,
    node: u8,
    code: u8,
) -> Result<[u8; 8], String> {
    exchange(bus, rx, node, command(code), SETUP_TIMEOUT).await
}

async fn exchange(
    bus: &dyn CanBus,
    rx: &mut MotorReplies,
    node: u8,
    bytes: [u8; 8],
    timeout: Duration,
) -> Result<[u8; 8], String> {
    // Only repeat reads. A lost acknowledgement does not prove a motion or
    // state-changing command was rejected by the drive.
    let attempts = if matches!(bytes[0], 0x9A | 0x92) {
        READ_ATTEMPTS
    } else {
        1
    };
    for _ in 0..attempts {
        if let Some(reply) = exchange_once(bus, rx, node, bytes, timeout).await? {
            return Ok(reply);
        }
    }
    Err(format!(
        "MHF node {node} did not reply to 0x{:02X} after {attempts} attempt(s)",
        bytes[0]
    ))
}

async fn exchange_once(
    bus: &dyn CanBus,
    replies: &mut MotorReplies,
    node: u8,
    bytes: [u8; 8],
    timeout: Duration,
) -> Result<Option<[u8; 8]>, String> {
    let code = bytes[0];
    tokio::time::sleep_until(replies.next_request_at).await;
    // This subscriber survives multiple transactions. An earlier queued
    // response must not acknowledge a new request with the same opcode.
    loop {
        match replies.rx.try_recv() {
            Ok(Some(_)) | Err(CanIoError::Lagged { .. }) => continue,
            Ok(None) => break,
            Err(error) => return Err(error.to_string()),
        }
    }
    send(bus, node, bytes).await?;
    replies.next_request_at = tokio::time::Instant::now() + TURNAROUND_DELAY;
    let deadline = Instant::now() + timeout;
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Ok(None);
        }
        let frame = match tokio::time::timeout(left, replies.rx.recv()).await {
            Ok(Ok(frame)) => frame,
            Ok(Err(CanIoError::Lagged { .. })) => continue,
            Ok(Err(error)) => return Err(error.to_string()),
            Err(_) => return Ok(None),
        };
        if frame.id() == can_id(node).into()
            && frame.kind() == FrameKind::Data
            && frame.data().len() == 8
            && frame.data()[0] == code
            && (!matches!(code, 0x80 | 0x81 | 0x88 | 0xC1) || frame.data() == bytes)
        {
            replies.next_request_at = tokio::time::Instant::now() + TURNAROUND_DELAY;
            let mut bytes = [0; 8];
            bytes.copy_from_slice(frame.data());
            return Ok(Some(bytes));
        }
    }
}

struct Loop {
    bus: Arc<dyn CanBus>,
    rx: Box<dyn CanRx>,
    ack_rx: MotorReplies,
    node_id: u8,
    state: Arc<Mutex<PullCordState>>,
    config: Arc<Mutex<PullCordConfig>>,
    stop: Arc<AtomicBool>,
    rehome: Arc<AtomicBool>,
    diagnostics: Arc<Mutex<PullCordCommandDiagnostics>>,
    motion: MotionPublisher,
    trajectory: TrajectoryRecorder,
    initial_angle: i64,
    initial_feedback: (Status2, Instant),
    running_state: u8,
    run_id: u64,
    encoder: PullCordEncoder,
}

impl Loop {
    async fn exchange(&mut self, bytes: [u8; 8]) -> Result<(), String> {
        exchange(
            self.bus.as_ref(),
            &mut self.ack_rx,
            self.node_id,
            bytes,
            SEND_TIMEOUT,
        )
        .await
        .map(|_| ())
    }

    async fn write_global_limit(&mut self, raw: u16) -> Result<(), String> {
        self.state.lock().unwrap().global_torque_limit_raw = None;
        self.exchange(global_torque_command(raw)).await?;
        self.state.lock().unwrap().global_torque_limit_raw = Some(raw);
        Ok(())
    }
}

enum MotorControl {
    A1Current,
    A2Speed {
        ram: [[u8; 8]; 3],
        last_speed_raw: i32,
    },
}

#[derive(Default)]
struct MotorOutput {
    zone: Option<Zone>,
    current_raw: i16,
    limit_raw: u16,
    speed_raw: i32,
}

impl MotorControl {
    /// No length is available while homing. Both modes send a zero command on
    /// the tick that establishes the origin. None asks the loop to resample.
    async fn step(
        &mut self,
        ctx: &mut Loop,
        cfg: &PullCordConfig,
        length: Option<f64>,
        cord_speed: f64,
        just_homed: bool,
    ) -> Result<Option<MotorOutput>, String> {
        if ctx.stop.load(Ordering::Acquire) {
            return Ok(None);
        }
        match self {
            Self::A1Current => {
                let a1 = cfg.a1_current.expect("normalized A1 config");
                let profile = length.map(|m| a1.running_current(m, cord_speed, cfg));
                let requested = match profile {
                    _ if just_homed => 0.0,
                    Some((_, raw)) => raw,
                    None => {
                        // Taper homing current over the last 0.2 m/s of inward speed.
                        let inward_speed = -cord_speed;
                        let scale = ((cfg.homing_speed_m_s - inward_speed) / 0.2).clamp(0.0, 1.0);
                        f64::from(a1.homing_raw) * scale
                    }
                };
                let current_raw = a1.command(requested, cfg.retract_positive);
                ctx.exchange(a1_command(current_raw)).await?;
                Ok(Some(MotorOutput {
                    zone: profile.map(|(zone, _)| zone),
                    current_raw,
                    ..Default::default()
                }))
            }
            Self::A2Speed {
                ram,
                last_speed_raw,
            } => {
                let a2 = cfg.a2_speed.expect("normalized A2 config");
                let profile = length.map(|m| a2.zone_limit_raw(m, cord_speed, cfg));
                let (limit_raw, target_m_s) = match profile {
                    _ if just_homed => (0, 0.0),
                    Some((_, limit)) => (limit, cfg.retract_speed_m_s),
                    None => (a2.limit_raw(f64::from(a2.homing_raw)), cfg.homing_speed_m_s),
                };
                let signed_speed = if cfg.retract_positive {
                    target_m_s
                } else {
                    -target_m_s
                };
                let target_raw = if limit_raw == 0 {
                    0
                } else {
                    speed_raw(signed_speed, cfg)
                };
                let next_ram = ram_settings(cfg);
                if next_ram != *ram {
                    // Drop the actual RAM limit before zero speed and reacquire
                    // speed feedback before selecting the next directional limit.
                    let result = async {
                        ctx.write_global_limit(0).await?;
                        ctx.exchange(a2_command(0, 0)).await?;
                        *last_speed_raw = 0;
                        configure_ram(
                            ctx.bus.as_ref(),
                            &mut ctx.ack_rx,
                            ctx.node_id,
                            cfg,
                            &ctx.stop,
                        )
                        .await?;
                        ctx.exchange(a2_command(0, 0)).await
                    }
                    .await;
                    result.map_err(|e| format!("MHF A2 RAM configuration failed: {e}"))?;
                    *ram = next_ram;
                    return Ok(None);
                }
                let previous_limit = ctx.state.lock().unwrap().global_torque_limit_raw;
                if let Some(old) =
                    previous_limit.filter(|old| *old < limit_raw && target_raw != *last_speed_raw)
                {
                    // Install a new speed under the previous lower RAM limit.
                    ctx.exchange(a2_command(old, target_raw))
                        .await
                        .map_err(|e| {
                            format!("MHF speed update before torque increase failed: {e}")
                        })?;
                }
                if ctx.stop.load(Ordering::Acquire) {
                    return Ok(None);
                }
                if previous_limit != Some(limit_raw) {
                    ctx.write_global_limit(limit_raw)
                        .await
                        .map_err(|e| format!("MHF global torque limit update failed: {e}"))?;
                }
                if ctx.stop.load(Ordering::Acquire) {
                    return Ok(None);
                }
                ctx.exchange(a2_command(limit_raw, target_raw)).await?;
                *last_speed_raw = target_raw;
                Ok(Some(MotorOutput {
                    zone: profile.map(|(zone, _)| zone),
                    limit_raw,
                    speed_raw: target_raw,
                    ..Default::default()
                }))
            }
        }
    }
}

async fn run_loop(mut ctx: Loop) {
    // Read the acknowledged startup configuration, even if a live edit has
    // already reached the separate configuration mailbox.
    let initial_config = ctx.state.lock().unwrap().config;
    let mode = initial_config.control_mode;
    let mut motor = match mode {
        PullCordMode::A1Current => MotorControl::A1Current,
        PullCordMode::A2Speed => MotorControl::A2Speed {
            ram: ram_settings(&initial_config),
            last_speed_raw: 0,
        },
    };
    let started = Instant::now();
    let mut ticker = tokio::time::interval(PERIOD);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut ticks = 0u64;
    let mut phase = Phase::Homing;
    let mut origin = None::<i64>;
    let mut angle = ctx.initial_angle;
    let mut angle_at = Instant::now();
    let mut last_angle = angle;
    let mut angle_sign = None::<f64>;
    let (mut status2, mut status2_at) = ctx.initial_feedback;
    let mut status1_at = Instant::now();
    let mut status_error = 0u8;
    let mut voltage_v = None::<f64>;
    let mut homing_started = Instant::now();
    let mut homing_stall = HomingStall::default();
    let fault = loop {
        if ctx.stop.load(Ordering::Acquire) {
            break None;
        }
        tokio::select! {
            // Drain queued feedback before a new control tick, particularly
            // after an online RAM update paused the transaction stream.
            biased;
            frame = ctx.rx.recv() => {
                match frame {
                    Ok(frame) => {
                        let data = frame.data();
                        if let Some(s) = parse_status2(data, mode) {
                            let received = Instant::now();
                            if phase == Phase::Homing {
                                let speed_m_s = f64::from(s.speed_dps) / 360.0 * initial_config.metres_per_rev();
                                homing_stall.observe(speed_m_s, received);
                            }
                            status2 = s;
                            status2_at = received;
                        } else if let Some(s) = parse_status1(data) {
                            status1_at = Instant::now();
                            status_error = s.error;
                            voltage_v = Some(s.voltage_v);
                            if s.state != ctx.running_state {
                                break Some(format!(
                                    "MHF motor left confirmed running state: 0x{:02X} (expected 0x{:02X})",
                                    s.state, ctx.running_state
                                ));
                            }
                        } else if let Some(next) = parse_angle(data) {
                            let delta = next - last_angle;
                            // Direction may first be observed after a stationary home.
                            if angle_sign.is_none() && delta.abs() >= 50 && status2.speed_dps.unsigned_abs() >= 20 {
                                angle_sign = Some(if delta.signum() == i64::from(status2.speed_dps.signum()) { 1.0 } else { -1.0 });
                            }
                            last_angle = next;
                            angle = next;
                            angle_at = Instant::now();
                        }
                    }
                    Err(e) => break Some(format!("MHF CAN receive failed: {e}")),
                }
            }
            _ = ticker.tick() => {
                ticks += 1;
                if ctx.rehome.load(Ordering::Acquire) {
                    // Readers reject the pending request until the old frame is gone.
                    ctx.trajectory.reset_frame();
                    ctx.rehome.store(false, Ordering::Release);
                    phase = Phase::Homing;
                    origin = None;
                    homing_started = Instant::now();
                    homing_stall = HomingStall::default();
                    ctx.motion.unavailable();
                }
                let cfg = *ctx.config.lock().unwrap();
                let direction = ctx.encoder.direction().await;
                let now = Instant::now();
                if now.duration_since(status2_at) > FEEDBACK_TIMEOUT {
                    break Some(format!("MHF {:02X} feedback timed out", mode.opcode()));
                }
                if now.duration_since(angle_at) > FEEDBACK_TIMEOUT {
                    break Some("MHF angle feedback timed out".into());
                }
                if now.duration_since(status1_at) > STATUS_TIMEOUT {
                    break Some("MHF status feedback timed out".into());
                }
                if status_error != 0 {
                    break Some(format!("MHF motor error 0x{status_error:02X}"));
                }
                if phase == Phase::Homing && now.duration_since(homing_started).as_secs_f64() > cfg.homing_timeout_s {
                    break Some("homing timed out; release the handle and check that it can settle at the magnetic stop before retrying".into());
                }

                let shaft_sign = if cfg.retract_positive { 1.0 } else { -1.0 };
                let cord_speed = -f64::from(status2.speed_dps) / 360.0 * cfg.metres_per_rev() * shaft_sign;
                let just_homed = phase == Phase::Homing && homing_stall.confirmed();
                if just_homed {
                    origin = Some(angle);
                    phase = Phase::Running;
                }
                // Use the observed MHF angle convention until motion confirms its sign.
                let length = origin.map(|origin| {
                    -(angle - origin) as f64 / 36000.0 * angle_sign.unwrap_or(1.0) * cfg.metres_per_rev() * shaft_sign
                });
                let output = match motor.step(&mut ctx, &cfg, length, cord_speed, just_homed).await {
                    Ok(Some(output)) => output,
                    Ok(None) => {
                        if phase == Phase::Homing {
                            homing_stall = HomingStall::default();
                        }
                        continue;
                    }
                    Err(error) => break Some(error),
                };
                // Offset angle and status polls to avoid three transactions in one tick.
                if ticks % 4 == 0 {
                    if let Err(error) = ctx.exchange(command(0x92)).await {
                        break Some(error);
                    }
                }
                if ticks % 20 == 2 {
                    if let Err(error) = ctx.exchange(command(0x9A)).await {
                        break Some(error);
                    }
                }
                let mut s = ctx.state.lock().unwrap();
                s.phase = phase;
                s.config = cfg;
                s.length_m = length;
                s.cord_speed_m_s = Some(cord_speed);
                s.zone = output.zone;
                s.encoder_offset_deg = direction.map(|d| d.offset_deg);
                s.stationary_current_raw = cfg.a1_current
                    .filter(|_| mode == PullCordMode::A1Current && phase == Phase::Running)
                    .map(|a1| f64::from(a1.near_raw));
                s.measured_current_raw = Some(status2.iq_raw);
                s.current_command_raw = output.current_raw;
                s.torque_limit_raw = output.limit_raw;
                s.speed_command_dps = f64::from(output.speed_raw) / 100.0;
                s.motor_speed_dps = Some(status2.speed_dps);
                s.position_angle_deg = Some(angle as f64 / 100.0);
                s.bus_voltage_v = voltage_v;
                let feedback_age = now.duration_since(status2_at).as_millis() as u64;
                s.a1_feedback_age_ms = Some(feedback_age); // Also the legacy motion-feedback age in A2.
                s.a2_feedback_age_ms = (mode == PullCordMode::A2Speed).then_some(feedback_age);
                s.angle_feedback_age_ms = Some(now.duration_since(angle_at).as_millis() as u64);
                s.motor_status_age_ms = Some(now.duration_since(status1_at).as_millis() as u64);
                s.motor_temp_c = Some(f32::from(status2.temp_c));
                s.online = true;
                let vdes = f64::from(output.speed_raw) / 36000.0;
                s.vdes_rev_per_s = vdes;
                s.homing_progress = homing_stall.progress();
                drop(s);
                ctx.motion.publish(phase, length, Some(cord_speed), cfg.retract_positive, cfg.retract_speed_m_s);
                let motor_valid_until = (angle_at.min(status2_at) + FEEDBACK_TIMEOUT).min(status1_at + STATUS_TIMEOUT);
                ctx.trajectory.record(now, length, angle_at, cord_speed, motor_valid_until, direction);
                let mut d = ctx.diagnostics.lock().unwrap();
                if d.run_id == ctx.run_id {
                    d.updated_at = Some(now);
                    d.generation = d.generation.wrapping_add(1);
                    d.run_elapsed_us = started.elapsed().as_micros().min(u128::from(u64::MAX)) as u64;
                    d.phase = phase;
                    d.zone = output.zone;
                    d.length_m = length;
                    d.cord_speed_m_s = Some(cord_speed);
                    d.vdes_rev_per_s = vdes as f32;
                    d.send_ok = true;
                }
            }
        }
    };
    ctx.motion.unavailable();
    ctx.trajectory.gap();
    let fault = if let Some(error) = fault {
        log::warn!("MHF pull-cord node {}: {error}", ctx.node_id);
        Some(
            match disable_motor(ctx.bus.as_ref(), ctx.node_id, mode).await {
                Ok(()) => error,
                Err(stop_error) => format!("{error}; {stop_error}"),
            },
        )
    } else {
        None
    };
    let mut d = ctx.diagnostics.lock().unwrap();
    d.active = false;
    if fault.is_some() {
        d.phase = Phase::Fault;
        d.send_ok = false;
    }
    drop(d);
    let mut s = ctx.state.lock().unwrap();
    if let Some(error) = fault {
        s.phase = Phase::Fault;
        s.fault = Some(error);
        s.online = false;
        s.encoder_offset_deg = None;
    }
    s.current_command_raw = 0;
    s.stationary_current_raw = None;
    s.torque_limit_raw = 0;
    s.global_torque_limit_raw = None;
    s.speed_command_dps = 0.0;
    s.vdes_rev_per_s = 0.0;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runtime_support::{CordMotion, MotionPermit};
    use can_transport::{CanCapabilities, CanIoError};
    use std::collections::BTreeMap;
    use std::sync::atomic::AtomicUsize;
    use tokio::sync::mpsc;

    struct ScriptRx(mpsc::UnboundedReceiver<CanFrame>);
    #[async_trait::async_trait]
    impl CanRx for ScriptRx {
        async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
            self.0.recv().await.ok_or(CanIoError::Disconnected)
        }
        fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
            match self.0.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(_) => Err(CanIoError::Disconnected),
            }
        }
    }

    #[derive(Default)]
    struct ScriptFeedback {
        angle: i64,
        speed_dps: i16,
        iq_raw: i16,
    }

    struct ScriptBus {
        sent: Mutex<Vec<u8>>,
        frames: Mutex<Vec<[u8; 8]>>,
        a2_sent: Mutex<Vec<(u16, i32)>>,
        ram_sent: Mutex<Vec<[u8; 8]>>,
        global_limit: Mutex<u16>,
        corrupt_ram_reply: AtomicBool,
        cancel_on_ram: Mutex<Option<Arc<AtomicBool>>>,
        speed_on_ram: Mutex<Option<i16>>,
        reply: Mutex<Vec<(CanFilter, mpsc::UnboundedSender<CanFrame>)>>,
        state: Mutex<u8>,
        lose_running_status: bool,
        fail_close_once: AtomicBool,
        ignore_close: AtomicBool,
        last_reply_at: Mutex<Option<Instant>>,
        early_requests: AtomicUsize,
        lost_replies: Mutex<BTreeMap<u8, usize>>,
        feedback: Mutex<ScriptFeedback>,
    }
    impl ScriptBus {
        fn emit(&self, frame: CanFrame) {
            self.reply
                .lock()
                .unwrap()
                .retain(|(filter, tx)| !filter.matches(&frame) || tx.send(frame).is_ok());
        }

        fn new(lose_running_status: bool) -> Self {
            Self {
                sent: Mutex::new(Vec::new()),
                frames: Mutex::new(Vec::new()),
                a2_sent: Mutex::new(Vec::new()),
                ram_sent: Mutex::new(Vec::new()),
                global_limit: Mutex::new(1900),
                corrupt_ram_reply: AtomicBool::new(false),
                cancel_on_ram: Mutex::new(None),
                speed_on_ram: Mutex::new(None),
                reply: Mutex::new(Vec::new()),
                state: Mutex::new(0x10),
                lose_running_status,
                fail_close_once: AtomicBool::new(false),
                ignore_close: AtomicBool::new(false),
                last_reply_at: Mutex::new(None),
                early_requests: AtomicUsize::new(0),
                lost_replies: Mutex::new(BTreeMap::new()),
                feedback: Mutex::new(ScriptFeedback::default()),
            }
        }
    }
    #[async_trait::async_trait]
    impl CanBus for ScriptBus {
        async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
            assert_eq!(frame.id(), can_id(1).into());
            assert_eq!(frame.data().len(), 8);
            let code = frame.data()[0];
            self.sent.lock().unwrap().push(code);
            self.frames
                .lock()
                .unwrap()
                .push(frame.data().try_into().unwrap());
            // The drive can ignore a new request while finishing its reply,
            // even though the host has already received that reply.
            if self
                .last_reply_at
                .lock()
                .unwrap()
                .is_some_and(|at| at.elapsed() < Duration::from_millis(2))
            {
                self.early_requests.fetch_add(1, Ordering::Relaxed);
                return Ok(());
            }
            if code == 0x80 && self.fail_close_once.swap(false, Ordering::AcqRel) {
                return Err(CanIoError::Disconnected);
            }
            match code {
                0x80 if !self.ignore_close.load(Ordering::Acquire) => {
                    let mut state = self.state.lock().unwrap();
                    *state = if matches!(*state, 0x20 | 0x30) {
                        0x20
                    } else {
                        0x10
                    };
                }
                0x88 => {
                    let mut state = self.state.lock().unwrap();
                    *state = if *state == 0x20 { 0x30 } else { 0x00 };
                }
                _ => {}
            }
            let response = match code {
                0x9A if self.lose_running_status
                    && matches!(*self.state.lock().unwrap(), 0 | 0x30) =>
                {
                    None
                }
                0x9A => {
                    let mut data = command(0x9A);
                    data[1] = 25;
                    data[2..4].copy_from_slice(&1600_u16.to_le_bytes());
                    data[6] = *self.state.lock().unwrap();
                    Some(data)
                }
                0xA1 | 0xA2 => {
                    if code == 0xA2 {
                        let limit = u16::from_le_bytes(frame.data()[2..4].try_into().unwrap());
                        assert_eq!(
                            limit,
                            *self.global_limit.lock().unwrap(),
                            "A2 may only follow a matching RAM limit"
                        );
                        self.a2_sent.lock().unwrap().push((
                            limit,
                            i32::from_le_bytes(frame.data()[4..8].try_into().unwrap()),
                        ));
                    }
                    let mut data = command(code);
                    data[2..4].copy_from_slice(&self.feedback.lock().unwrap().iq_raw.to_le_bytes());
                    data[4..6]
                        .copy_from_slice(&self.feedback.lock().unwrap().speed_dps.to_le_bytes());
                    Some(data)
                }
                0xC1 => {
                    let mut data: [u8; 8] = frame.data().try_into().unwrap();
                    self.ram_sent.lock().unwrap().push(data);
                    if data[1] == 0x1E {
                        *self.global_limit.lock().unwrap() = u16::from_le_bytes([data[4], data[5]]);
                    }
                    if let Some(stop) = self.cancel_on_ram.lock().unwrap().take() {
                        stop.store(true, Ordering::Release);
                    }
                    if let Some(speed) = self.speed_on_ram.lock().unwrap().take() {
                        self.feedback.lock().unwrap().speed_dps = speed;
                    }
                    if self.corrupt_ram_reply.load(Ordering::Acquire) {
                        data[4] ^= 1;
                    }
                    Some(data)
                }
                0x92 => {
                    let mut data = command(0x92);
                    data[1..]
                        .copy_from_slice(&self.feedback.lock().unwrap().angle.to_le_bytes()[..7]);
                    Some(data)
                }
                0x80 | 0x81 | 0x88 => Some(command(code)),
                _ => None,
            };
            if let Some(data) = response {
                *self.last_reply_at.lock().unwrap() = Some(Instant::now());
                if let Some(left) = self.lost_replies.lock().unwrap().get_mut(&code) {
                    if *left > 0 {
                        *left -= 1;
                        return Ok(());
                    }
                }
                let frame = CanFrame::new_data(can_id(1), &data).unwrap();
                self.emit(frame);
            }
            Ok(())
        }
        async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            self.reply.lock().unwrap().push((filter, tx));
            Ok(Box::new(ScriptRx(rx)))
        }
        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: false,
                max_dlen: 8,
            }
        }
    }

    async fn wait_for_motion(
        rx: &mut CordMotionReceiver,
        predicate: impl Fn(&CordMotion) -> bool,
    ) -> CordMotion {
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let motion = *rx.borrow_and_update();
                if predicate(&motion) {
                    return motion;
                }
                rx.changed().await.unwrap();
            }
        })
        .await
        .expect("pull-cord feedback did not reach the expected state")
    }

    #[test]
    fn homing_requires_600_ms_of_observed_low_speed() {
        let start = Instant::now();
        for speed in [0.0, 0.019, -0.019] {
            let mut stall = HomingStall::default();
            assert!(!stall.confirmed());
            for ms in (0..=500).step_by(100) {
                stall.observe(speed, start + Duration::from_millis(ms));
                assert!(!stall.confirmed());
            }
            stall.observe(speed, start + Duration::from_millis(599));
            assert!(!stall.confirmed());
            stall.observe(speed, start + Duration::from_millis(600));
            assert!(stall.confirmed());
            assert_eq!(stall.progress(), 1.0);
        }
    }

    #[test]
    fn homing_restarts_the_full_hold_at_or_above_threshold_in_either_direction() {
        let start = Instant::now();
        for speed in [0.02, -0.02, 0.03, -0.03] {
            let mut stall = HomingStall::default();
            for ms in (0..=500).step_by(100) {
                stall.observe(0.0, start + Duration::from_millis(ms));
            }
            stall.observe(speed, start + Duration::from_millis(550));
            assert_eq!(stall.progress(), 0.0);
            for ms in (600..=1100).step_by(100) {
                stall.observe(0.0, start + Duration::from_millis(ms));
                assert!(!stall.confirmed());
            }
            stall.observe(0.0, start + Duration::from_millis(1199));
            assert!(!stall.confirmed());
            stall.observe(0.0, start + Duration::from_millis(1200));
            assert!(stall.confirmed());
        }
    }

    #[test]
    fn homing_does_not_count_a_feedback_gap_as_stationary_time() {
        let start = Instant::now();
        let mut stall = HomingStall::default();
        for ms in (0..=500).step_by(100) {
            stall.observe(0.0, start + Duration::from_millis(ms));
        }
        stall.observe(0.0, start + Duration::from_millis(651));
        assert!(!stall.confirmed());
        assert_eq!(stall.progress(), 0.0);
    }

    #[tokio::test]
    async fn stationary_home_and_rehome_publish_valid_zero_after_600_ms() {
        let bus = Arc::new(ScriptBus::new(false));
        bus.feedback.lock().unwrap().angle = 123_456;
        let mut cfg = default_config();
        cfg.homing_stall_hold_ms = 400;
        cfg.homing_stall_speed_m_s = 0.1;
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            cfg,
            Default::default(),
            Default::default(),
            Default::default(),
        )
        .await
        .unwrap();
        let mut rx = app.subscribe_motion();
        let started = Instant::now();
        let first = wait_for_motion(&mut rx, |m| m.permit == MotionPermit::Ready).await;
        assert!(started.elapsed() >= Duration::from_millis(600));
        assert_eq!(first.length_m, Some(0.0));
        assert_eq!(first.speed_m_s, Some(0.0));
        assert_eq!(first.retract_target_speed_m_s, cfg.retract_speed_m_s);
        assert_eq!(app.snapshot().length_m, Some(0.0));
        assert_eq!(app.snapshot().homing_progress, 1.0);
        assert_eq!(app.snapshot().config.homing_stall_hold_ms, 600);
        assert_eq!(app.snapshot().config.homing_stall_speed_m_s, 0.02);

        cfg.a2_speed.as_mut().unwrap().near_raw = 24;
        cfg.a2_speed.as_mut().unwrap().retract_raw = 24;
        cfg.retract_speed_m_s = 1.5;
        app.set_config(cfg).unwrap();
        assert_eq!(app.snapshot().config.homing_stall_hold_ms, 600);
        assert_eq!(app.snapshot().config.homing_stall_speed_m_s, 0.02);
        wait_for_motion(&mut rx, |motion| {
            app.snapshot().config.a2_speed.unwrap().near_raw == 24
                && motion.retract_target_speed_m_s == 1.5
        })
        .await;
        // A one-degree offset is far below the former 3 cm travel gate.
        bus.feedback.lock().unwrap().angle += 100;
        let restarted = Instant::now();
        app.rehome();
        let pending = wait_for_motion(&mut rx, |m| m.permit == MotionPermit::Unavailable).await;
        assert_eq!(pending.length_m, None);
        assert!(pending.invalidation_epoch > first.invalidation_epoch);
        let next = wait_for_motion(&mut rx, |m| m.permit == MotionPermit::Ready).await;
        assert!(restarted.elapsed() >= Duration::from_millis(600));
        assert_eq!(next.length_m, Some(0.0));
        assert_eq!(app.snapshot().position_angle_deg, Some(1235.56));
        app.stop().await.unwrap();
    }

    #[tokio::test]
    async fn payout_after_stationary_home_learns_angle_sign_for_both_retract_directions() {
        for retract_positive in [true, false] {
            for angle_sign in [1_i64, -1] {
                let bus = Arc::new(ScriptBus::new(false));
                bus.feedback.lock().unwrap().angle = 123_456;
                let mut cfg = default_config();
                cfg.retract_positive = retract_positive;
                let mut app = LingkongPull::start(
                    bus.clone(),
                    1,
                    cfg,
                    Default::default(),
                    Default::default(),
                    Default::default(),
                )
                .await
                .unwrap();
                let mut rx = app.subscribe_motion();
                let zero = wait_for_motion(&mut rx, |m| m.permit == MotionPermit::Ready).await;
                assert_eq!(zero.length_m, Some(0.0));
                let shaft_sign = if retract_positive { 1 } else { -1 };
                *bus.feedback.lock().unwrap() = ScriptFeedback {
                    angle: 123_456 - 9000 * shaft_sign * angle_sign,
                    speed_dps: -100 * shaft_sign as i16,
                    ..Default::default()
                };
                let payout =
                    wait_for_motion(&mut rx, |m| m.length_m.is_some_and(|x| x > 0.0)).await;
                assert!((payout.length_m.unwrap() - cfg.metres_per_rev() / 4.0).abs() < 1e-9);
                assert!(payout.speed_m_s.unwrap() > 0.0);
                assert_eq!(payout.retract_positive, retract_positive);
                app.stop().await.unwrap();
            }
        }
    }

    #[tokio::test]
    async fn endpoint_uses_homed_motor_and_live_magnetic_feedback_in_both_modes() {
        for mode in [PullCordMode::A1Current, PullCordMode::A2Speed] {
            let bus = Arc::new(ScriptBus::new(false));
            let encoder = PullCordEncoder::default();
            let setup = magnetic_encoder::EncoderConfig {
                auto_start_node: false,
                ..Default::default()
            };
            *encoder.monitor.lock().await =
                Some(EncoderMonitor::start(bus.clone(), setup).await.unwrap());
            *encoder.center_deg.lock().unwrap() = 90.0;
            let config = PullCordConfig {
                control_mode: mode,
                retract_positive: false,
                ..crate::default_config()
            };
            let mut app = crate::start(bus.clone(), 1, config, Default::default(), encoder)
                .await
                .unwrap();
            let trace = app.trajectory();
            let mut motion = app.subscribe_motion();
            wait_for_motion(&mut motion, |m| m.permit == MotionPermit::Ready).await;
            assert!(trace.latest().is_none());

            *bus.feedback.lock().unwrap() = ScriptFeedback {
                angle: 36000,
                speed_dps: 90,
                ..Default::default()
            };
            let position = |counts: u32| {
                let mut data = [0; 8];
                data[..4].copy_from_slice(&counts.to_le_bytes());
                CanFrame::new_data(setup.protocol.position_cob_id(), &data).unwrap()
            };
            let magnet =
                |status: u8| CanFrame::new_data(setup.protocol.status_cob_id(), &[status, 0]).unwrap();
            bus.emit(position(2048)); // 180° magnetic angle, 90° clockwise from centre.
            bus.emit(magnet(0));
            wait_for_motion(&mut motion, |_| trace.latest().is_some_and(|p| p.y_m < -0.1)).await;
            let p = trace.latest().unwrap();
            assert!(p.x_m.abs() < 1e-9);
            assert!((p.radial_speed_m_s - config.metres_per_rev() / 4.0).abs() < 1e-9);
            bus.emit(magnet(3));
            wait_for_motion(&mut motion, |_| trace.latest().is_none()).await;
            bus.emit(position(0));
            bus.emit(magnet(0));
            wait_for_motion(&mut motion, |_| trace.latest().is_some_and(|p| p.y_m > 0.1)).await;
            assert_ne!(trace.latest().unwrap().segment, p.segment);
            if mode == PullCordMode::A2Speed {
                app.rehome();
                assert!(trace.latest().is_none());
                wait_for_motion(&mut motion, |m| m.permit == MotionPermit::Unavailable).await;
                assert!(trace.since(None).points.is_empty());
            }
            app.stop().await.unwrap();
            assert!(trace.latest().is_none());
            if mode == PullCordMode::A1Current {
                assert!(!trace.since(None).points.is_empty());
            }
        }
    }

    #[tokio::test]
    async fn continuous_motion_still_times_out_and_closes_the_motor() {
        let bus = Arc::new(ScriptBus::new(false));
        bus.feedback.lock().unwrap().speed_dps = 100;
        let mut cfg = default_config();
        cfg.homing_timeout_s = 0.12;
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            cfg,
            Default::default(),
            Default::default(),
            Default::default(),
        )
        .await
        .unwrap();
        tokio::time::timeout(Duration::from_secs(2), async {
            while app.snapshot().phase != Phase::Fault {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .unwrap();
        let fault = app.snapshot().fault.unwrap();
        assert!(fault.contains("homing timed out"), "{fault}");
        assert!(!fault.contains("3 cm"));
        assert_eq!(*bus.state.lock().unwrap(), 0x10);
        app.stop().await.unwrap();
    }

    #[tokio::test]
    async fn startup_confirms_close_and_run_then_stop_closes_drive() {
        let bus = Arc::new(ScriptBus::new(false));
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            default_config(),
            Arc::new(Mutex::new(PullCordCommandDiagnostics::default())),
            Arc::new(AtomicBool::new(false)),
            Default::default(),
        )
        .await
        .unwrap();
        app.stop().await.unwrap();
        let sent = bus.sent.lock().unwrap().clone();
        assert_eq!(
            &sent[..11],
            &[0x9A, 0x80, 0x9A, 0xC1, 0xC1, 0xC1, 0xC1, 0x88, 0x9A, 0xA2, 0x92]
        );
        assert_eq!(bus.frames.lock().unwrap()[3], global_torque_command(0));
        assert_eq!(&sent[sent.len() - 5..], &[0xC1, 0xA2, 0x81, 0x80, 0x9A]);
    }

    #[tokio::test]
    async fn observed_mhf_state_pair_confirms_close_and_run() {
        let bus = Arc::new(ScriptBus::new(false));
        *bus.state.lock().unwrap() = 0x20;
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            default_config(),
            Arc::new(Mutex::new(PullCordCommandDiagnostics::default())),
            Arc::new(AtomicBool::new(false)),
            Default::default(),
        )
        .await
        .unwrap();
        assert_eq!(
            &bus.sent.lock().unwrap()[..11],
            &[0x9A, 0x80, 0x9A, 0xC1, 0xC1, 0xC1, 0xC1, 0x88, 0x9A, 0xA2, 0x92]
        );
        assert_eq!(*bus.state.lock().unwrap(), 0x30);
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert!(app.snapshot().fault.is_none());
        app.stop().await.unwrap();
        assert_eq!(*bus.state.lock().unwrap(), 0x20);
        assert_eq!(bus.early_requests.load(Ordering::Relaxed), 0);
    }

    #[tokio::test]
    async fn status_and_angle_reads_recover_from_two_lost_replies() {
        let bus = Arc::new(ScriptBus::new(false));
        bus.lost_replies
            .lock()
            .unwrap()
            .extend([(0x9A, 2), (0x92, 2)]);
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            default_config(),
            Default::default(),
            Default::default(),
            Default::default(),
        )
        .await
        .unwrap();
        // The last angle reply arrived after the original motion sample expired.
        assert_eq!(bus.frames.lock().unwrap().last(), Some(&a2_command(0, 0)));
        app.stop().await.unwrap();
        let sent = bus.sent.lock().unwrap();
        assert_eq!(sent.iter().filter(|&&code| code == 0x92).count(), 3);
        assert_eq!(sent.iter().filter(|&&code| code == 0x9A).count(), 6);
        assert_eq!(sent.iter().filter(|&&code| code == 0x88).count(), 1);
        assert_eq!(bus.early_requests.load(Ordering::Relaxed), 0);
    }

    #[tokio::test]
    async fn lost_motion_reply_is_not_retried_and_startup_closes_the_drive() {
        for mode in [PullCordMode::A2Speed, PullCordMode::A1Current] {
            let bus = Arc::new(ScriptBus::new(false));
            bus.lost_replies.lock().unwrap().insert(mode.opcode(), 1);
            let cfg = PullCordConfig {
                control_mode: mode,
                ..default_config()
            };
            let error = LingkongPull::start(
                bus.clone(),
                1,
                cfg,
                Default::default(),
                Default::default(),
                Default::default(),
            )
            .await
            .err()
            .unwrap();
            assert!(
                error.contains(&format!("did not reply to 0x{:02X}", mode.opcode())),
                "{error}"
            );
            let sent = bus.sent.lock().unwrap();
            // One initial target and the distinct zero command during shutdown.
            assert_eq!(
                sent.iter().filter(|&&code| code == mode.opcode()).count(),
                2
            );
            assert_eq!(sent.iter().filter(|&&code| code == 0x88).count(), 1);
            assert_eq!(*bus.state.lock().unwrap(), 0x10);
            if mode == PullCordMode::A1Current {
                assert!(!sent.contains(&0xC1) && !sent.contains(&0xA2));
            }
        }
    }

    #[tokio::test]
    async fn queued_status_cannot_confirm_a_new_request_and_read_retries_are_bounded() {
        let bus = ScriptBus::new(false);
        let mut replies = MotorReplies::subscribe(&bus, 1).await.unwrap();
        // Leave an old, valid-looking status in this long-lived subscription.
        send(&bus, 1, command(0x9A)).await.unwrap();
        bus.lost_replies.lock().unwrap().insert(0x9A, 10);
        let error = exchange(
            &bus,
            &mut replies,
            1,
            command(0x9A),
            Duration::from_millis(5),
        )
        .await
        .unwrap_err();
        assert!(error.contains("after 3 attempt(s)"), "{error}");
        assert_eq!(*bus.sent.lock().unwrap(), vec![0x9A; 4]);
        assert!(!bus.sent.lock().unwrap().contains(&0x88));
    }

    #[tokio::test]
    async fn undocumented_state_that_will_not_close_never_runs() {
        let bus = Arc::new(ScriptBus::new(false));
        *bus.state.lock().unwrap() = 0x40;
        bus.ignore_close.store(true, Ordering::Release);
        let result = LingkongPull::start(
            bus.clone(),
            1,
            default_config(),
            Arc::new(Mutex::new(PullCordCommandDiagnostics::default())),
            Arc::new(AtomicBool::new(false)),
            Default::default(),
        )
        .await;
        assert!(result.err().unwrap().contains("initial 0x40, now 0x40"));
        assert!(!bus.sent.lock().unwrap().contains(&0x88));
    }

    #[tokio::test]
    async fn lost_run_status_still_closes_the_drive() {
        let bus = Arc::new(ScriptBus::new(true));
        let result = LingkongPull::start(
            bus.clone(),
            1,
            default_config(),
            Arc::new(Mutex::new(PullCordCommandDiagnostics::default())),
            Arc::new(AtomicBool::new(false)),
            Default::default(),
        )
        .await;
        assert!(result.is_err());
        let sent = bus.sent.lock().unwrap().clone();
        assert_eq!(
            &sent[..9],
            &[0x9A, 0x80, 0x9A, 0xC1, 0xC1, 0xC1, 0xC1, 0x88, 0x9A]
        );
        assert_eq!(&sent[sent.len() - 5..], &[0xC1, 0xA2, 0x81, 0x80, 0x9A]);
    }

    #[tokio::test]
    async fn failed_close_is_reported_and_can_be_retried() {
        let bus = Arc::new(ScriptBus::new(false));
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            default_config(),
            Arc::new(Mutex::new(PullCordCommandDiagnostics::default())),
            Arc::new(AtomicBool::new(false)),
            Default::default(),
        )
        .await
        .unwrap();
        bus.fail_close_once.store(true, Ordering::Release);
        assert!(app.stop().await.unwrap_err().contains("0x80"));
        assert_eq!(app.snapshot().phase, Phase::Fault);
        app.stop().await.unwrap();
        let sent = bus.sent.lock().unwrap();
        assert_eq!(&sent[sent.len() - 2..], &[0x80, 0x9A]);
    }

    #[test]
    fn a2_encoding_and_ram_parameters_match_v236() {
        assert_eq!(a2_command(990, 36000), [0xA2, 0, 0xde, 3, 0xa0, 0x8c, 0, 0]);
        assert_eq!(
            a2_command(850, -36000),
            [0xA2, 0, 0x52, 3, 0x60, 0x73, 0xff, 0xff]
        );
        let mut cfg = default_config();
        let a2 = cfg.a2_speed.as_mut().unwrap();
        a2.max_raw = 950;
        a2.speed_kp = 1000;
        a2.speed_ki = 12;
        a2.speed_kd = 2000;
        a2.speed_ramp_dps2 = 600000;
        cfg.retract_speed_m_s = cfg.metres_per_rev();
        assert_eq!(
            ram_settings(&cfg),
            [
                [0xC1, 0x0b, 0xe8, 3, 12, 0, 0xd0, 7],
                [0xC1, 0x20, 0, 0, 0xa0, 0x8c, 0, 0],
                [0xC1, 0x26, 0, 0, 0xc0, 0x27, 9, 0],
            ]
        );
        assert_eq!(
            global_torque_command(950),
            [0xC1, 0x1e, 0, 0, 0xb6, 3, 0, 0]
        );

        cfg.drum_radius_m = 0.026;
        cfg.retract_speed_m_s = 1000.0 / 60.0 * cfg.metres_per_rev();
        validate_mhf_config(&cfg).unwrap();
        assert_eq!(speed_raw(cfg.retract_speed_m_s, &cfg), 600_000);
        assert_eq!(speed_raw(-cfg.retract_speed_m_s, &cfg), -600_000);
        assert_eq!(
            ram_settings(&cfg)[1],
            [0xC1, 0x20, 0, 0, 0xc0, 0x27, 9, 0]
        );
    }

    #[test]
    fn replies_decode_signed_values_and_reject_other_commands() {
        for mode in [PullCordMode::A1Current, PullCordMode::A2Speed] {
            let mut frame = [mode.opcode(), 25, 0x9c, 0xff, 0x38, 0xff, 0, 0];
            let s = parse_status2(&frame, mode).unwrap();
            assert_eq!((s.iq_raw, s.speed_dps), (-100, -200));
            frame[0] = if mode == PullCordMode::A1Current {
                0xA2
            } else {
                0xA1
            };
            assert!(parse_status2(&frame, mode).is_none());
        }
        assert_eq!(
            parse_angle(&[0x92, 0x60, 0x79, 0xff, 0xff, 0xff, 0xff, 0xff]),
            Some(-34464)
        );
        assert_eq!(a1_command(-990), [0xA1, 0, 0, 0, 0x22, 0xfc, 0, 0]);
        assert_eq!(a1_command(2048), [0xA1, 0, 0, 0, 0x00, 0x08, 0, 0]);
    }

    #[test]
    fn both_speed_targets_obey_the_1000_rpm_protocol_bound() {
        let mut cfg = default_config();
        cfg.drum_radius_m = 0.026;
        let max = 1000.0 / 60.0 * cfg.metres_per_rev();
        cfg.homing_speed_m_s = max;
        cfg.retract_speed_m_s = max;
        assert!(validate_mhf_config(&cfg).is_ok());
        cfg.retract_speed_m_s = max + 0.01;
        assert!(validate_mhf_config(&cfg).is_err());
        cfg.retract_speed_m_s = max;
        cfg.homing_speed_m_s = max + 0.01;
        assert!(validate_mhf_config(&cfg).is_err());
    }

    async fn wait_for_state(
        app: &LingkongPull,
        matches: impl Fn(&PullCordState) -> bool,
    ) -> PullCordState {
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let snapshot = app.snapshot();
                if matches(&snapshot) {
                    return snapshot;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("expected MHF state")
    }

    #[tokio::test]
    async fn a1_uses_signed_homing_and_zoned_current_then_allows_an_a2_restart() {
        for sign in [-1_i16, 1] {
            let bus = Arc::new(ScriptBus::new(false));
            let mut cfg = crate::default_config();
            cfg.control_mode = PullCordMode::A1Current;
            cfg.retract_positive = sign > 0;
            cfg.zone_near_end_m = cfg.metres_per_rev();
            cfg.zone_alert_start_m = 3.0 * cfg.metres_per_rev();
            cfg.homing_speed_m_s = 0.4;
            cfg.a1_current = Some(A1CurrentConfig {
                homing_raw: 400,
                near_raw: 100,
                payout_current_raw: 40,
                retract_current_raw: 200,
                alert_raw: 500,
                max_raw: 350,
                full_current_raw: 200,
            });
            // Already retracting faster than the homing soft limit at startup.
            bus.feedback.lock().unwrap().speed_dps = sign * 1000;
            bus.feedback.lock().unwrap().iq_raw = -72;
            let mut app = crate::start(bus.clone(), 1, cfg, Default::default(), Default::default())
                .await
                .unwrap();
            let state = wait_for_state(&app, |s| s.measured_current_raw == Some(-72)).await;
            assert_eq!(state.current_command_raw, 0);
            assert_eq!(state.speed_command_dps, 0.0);
            assert_eq!(state.global_torque_limit_raw, None);
            assert!(state.a1_feedback_age_ms.is_some() && state.a2_feedback_age_ms.is_none());
            assert!(bus
                .frames
                .lock()
                .unwrap()
                .iter()
                .filter(|f| f[0] == 0xA1)
                .all(|f| *f == a1_command(0)));
            let mut next = cfg;
            next.control_mode = PullCordMode::A2Speed;
            assert!(app.set_config(next).is_err());
            next = cfg;
            next.a1_current.as_mut().unwrap().max_raw += 1;
            assert!(app.set_config(next).is_err());

            bus.feedback.lock().unwrap().speed_dps = 0;
            wait_for_state(&app, |s| {
                s.phase == Phase::Homing && s.current_command_raw == sign * 350
            })
            .await;
            let running = wait_for_state(&app, |s| {
                s.phase == Phase::Running && s.current_command_raw == sign * 100
            })
            .await;
            assert_eq!(running.length_m, Some(0.0));
            let motion =
                crate::motion::read_motion(&app.subscribe_motion(), Instant::now()).unwrap();
            assert_eq!(motion.retract_target_speed_m_s, cfg.retract_speed_m_s);
            // Change only the measured speed. Every frame must still pull inward
            // with the selected magnitude on either motor installation.
            for (speed_dps, expected) in [(-100 * sign, 40), (100 * sign, 200), (0, 100)] {
                bus.feedback.lock().unwrap().speed_dps = speed_dps;
                let sample = wait_for_state(&app, |s| {
                    s.motor_speed_dps == Some(speed_dps) && s.current_command_raw == sign * expected
                })
                .await;
                assert_eq!(sample.length_m, Some(0.0));
                assert!(bus
                    .frames
                    .lock()
                    .unwrap()
                    .contains(&a1_command(sign * expected)));
            }
            bus.feedback.lock().unwrap().angle = -72_000 * i64::from(sign);
            wait_for_state(&app, |s| {
                s.zone == Some(Zone::RampUp) && s.current_command_raw == sign * 300
            })
            .await;
            bus.feedback.lock().unwrap().angle = -144_000 * i64::from(sign);
            wait_for_state(&app, |s| {
                s.zone == Some(Zone::Alert) && s.current_command_raw == sign * 350
            })
            .await;
            cfg.a1_current.as_mut().unwrap().alert_raw = 275;
            app.set_config(cfg).unwrap();
            wait_for_state(&app, |s| {
                s.current_command_raw == sign * 275 && s.config == cfg
            })
            .await;
            bus.feedback.lock().unwrap().speed_dps = -100 * sign;
            wait_for_state(&app, |s| s.current_command_raw == sign * 215).await;
            cfg.a1_current.as_mut().unwrap().payout_current_raw = 65;
            app.set_config(cfg).unwrap();
            wait_for_state(&app, |s| {
                s.current_command_raw == sign * 240 && s.config == cfg
            })
            .await;
            app.rehome();
            wait_for_state(&app, |s| {
                s.phase == Phase::Homing && s.current_command_raw == sign * 350
            })
            .await;
            app.stop().await.unwrap();
            {
                let frames = bus.frames.lock().unwrap();
                assert!(frames.contains(&a1_command(sign * 275)));
                assert!(frames.contains(&a1_command(sign * 215)));
                assert!(frames.contains(&a1_command(sign * 240)));
                assert!(frames.iter().all(|f| !matches!(f[0], 0xA2 | 0xC1)));
                assert_eq!(
                    &frames[frames.len() - 4..],
                    &[a1_command(0), command(0x81), command(0x80), command(0x9A)]
                );
            }

            cfg.control_mode = PullCordMode::A2Speed;
            let mut app = crate::start(bus.clone(), 1, cfg, Default::default(), Default::default())
                .await
                .unwrap();
            wait_for_state(&app, |s| {
                s.torque_limit_raw == cfg.a2_speed.unwrap().homing_raw
            })
            .await;
            cfg.control_mode = PullCordMode::A1Current;
            assert!(app.set_config(cfg).is_err());
            app.stop().await.unwrap();
        }
    }

    #[tokio::test]
    async fn lost_running_a1_reply_closes_with_zero_current_and_invalidates_motion() {
        let bus = Arc::new(ScriptBus::new(false));
        let cfg = PullCordConfig {
            control_mode: PullCordMode::A1Current,
            ..crate::default_config()
        };
        let mut app = crate::start(bus.clone(), 1, cfg, Default::default(), Default::default())
            .await
            .unwrap();
        wait_for_state(&app, |s| {
            s.phase == Phase::Running && s.current_command_raw < 0
        })
        .await;
        let motion = app.subscribe_motion();
        bus.lost_replies.lock().unwrap().insert(0xA1, 1);
        let failed = wait_for_state(&app, |s| s.phase == Phase::Fault).await;
        assert!(failed.fault.unwrap().contains("0xA1"));
        assert_eq!(failed.current_command_raw, 0);
        assert!(crate::motion::read_motion(&motion, Instant::now()).is_err());
        assert_eq!(*bus.state.lock().unwrap(), 0x10);
        assert!(!bus.sent.lock().unwrap().contains(&0xA2));
        app.stop().await.unwrap();
    }

    #[tokio::test]
    async fn a2_homing_and_live_reference_keep_native_raw_values() {
        for positive in [true, false] {
            let bus = Arc::new(ScriptBus::new(false));
            bus.feedback.lock().unwrap().iq_raw = -123;
            let mut cfg = default_config();
            cfg.retract_positive = positive;
            cfg.max_torque_permille = 0;
            let a2 = cfg.a2_speed.as_mut().unwrap();
            a2.homing_raw = 990;
            a2.max_raw = 1200;
            a2.full_torque_raw = 950;
            let mut app = LingkongPull::start(
                bus.clone(),
                1,
                cfg,
                Default::default(),
                Default::default(),
                Default::default(),
            )
            .await
            .unwrap();
            let expected_speed = speed_raw(
                cfg.homing_speed_m_s * if positive { 1.0 } else { -1.0 },
                &cfg,
            );
            let state = wait_for_state(&app, |s| {
                s.torque_limit_raw == 990 && s.measured_current_raw.is_some()
            })
            .await;
            assert_eq!(state.phase, Phase::Homing);
            assert!(bus.a2_sent.lock().unwrap().contains(&(990, expected_speed)));
            assert_eq!(state.current_command_raw, 0);
            assert_eq!(state.measured_current_raw, Some(-123));
            assert_eq!(state.commanded_torque_nm, None);
            assert_eq!(state.measured_torque_nm, None);
            assert_eq!(state.measured_force_n, None);
            assert_eq!(state.global_torque_limit_raw, Some(990));
            let mut next = cfg;
            next.a2_speed.as_mut().unwrap().full_torque_raw = 850;
            app.set_config(next).unwrap();
            wait_for_state(&app, |s| {
                s.config.a2_speed.unwrap().full_torque_raw == 850 && s.torque_limit_raw == 990
            })
            .await;
            // A reference-only update must not pause motion or rewrite RAM.
            // Zero during initialization, three static parameters, then homing.
            assert_eq!(bus.ram_sent.lock().unwrap().len(), 5);
            next.a2_speed.as_mut().unwrap().max_raw = 950;
            next.a2_speed.as_mut().unwrap().speed_kd = 2;
            next.homing_speed_m_s = 0.1;
            app.set_config(next).unwrap();
            wait_for_state(&app, |s| s.torque_limit_raw == 950 && s.config == next).await;
            let mut expected = vec![global_torque_command(0)];
            expected.extend(ram_settings(&next));
            expected.push(global_torque_command(950));
            assert_eq!(&bus.ram_sent.lock().unwrap()[5..], &expected);
            assert_eq!(app.snapshot().global_torque_limit_raw, Some(950));
            app.stop().await.unwrap();
            assert_eq!(bus.a2_sent.lock().unwrap().last(), Some(&(0, 0)));
        }
    }

    #[tokio::test]
    async fn incorrect_ram_echo_blocks_start_before_run() {
        let bus = Arc::new(ScriptBus::new(false));
        bus.corrupt_ram_reply.store(true, Ordering::Release);
        let error = LingkongPull::start(
            bus.clone(),
            1,
            default_config(),
            Default::default(),
            Default::default(),
            Default::default(),
        )
        .await
        .err()
        .unwrap();
        assert!(error.contains("0xC1"), "{error}");
        assert!(!bus.sent.lock().unwrap().contains(&0x88));
        assert_eq!(*bus.state.lock().unwrap(), 0x10);
    }

    #[tokio::test]
    async fn cancelled_ram_initialization_never_enables_the_motor() {
        let bus = Arc::new(ScriptBus::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        *bus.cancel_on_ram.lock().unwrap() = Some(stop.clone());
        let error = LingkongPull::start(
            bus.clone(),
            1,
            default_config(),
            Default::default(),
            stop,
            Default::default(),
        )
        .await
        .err()
        .unwrap();
        assert!(error.contains("cancelled"), "{error}");
        assert!(!bus.sent.lock().unwrap().contains(&0x88));
        assert_eq!(
            *bus.ram_sent.lock().unwrap(),
            vec![global_torque_command(0); 2]
        );
        assert_eq!(*bus.state.lock().unwrap(), 0x10);
    }

    #[tokio::test]
    async fn live_ram_update_uses_new_payout_feedback_before_resuming() {
        let bus = Arc::new(ScriptBus::new(false));
        let mut cfg = default_config();
        let a2 = cfg.a2_speed.as_mut().unwrap();
        a2.payout_raw = 20;
        a2.near_raw = 100;
        a2.retract_raw = 300;
        a2.max_raw = 500;
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            cfg,
            Default::default(),
            Default::default(),
            Default::default(),
        )
        .await
        .unwrap();
        wait_for_state(&app, |s| s.phase == Phase::Running).await;
        bus.feedback.lock().unwrap().speed_dps = 200;
        wait_for_state(&app, |s| s.torque_limit_raw == 300).await;
        let sent_before = bus.a2_sent.lock().unwrap().len();
        *bus.speed_on_ram.lock().unwrap() = Some(-200);
        cfg.a2_speed.as_mut().unwrap().max_raw = 600;
        cfg.a2_speed.as_mut().unwrap().speed_kp += 1;
        app.set_config(cfg).unwrap();
        wait_for_state(&app, |s| s.config == cfg && s.torque_limit_raw == 20).await;
        {
            let sent = bus.a2_sent.lock().unwrap();
            let changed = &sent[sent_before..];
            let pause = changed.iter().position(|target| *target == (0, 0)).unwrap();
            assert!(
                changed[pause..].iter().all(|&(limit, _)| limit <= 20),
                "{changed:?}"
            );
        }
        app.stop().await.unwrap();
    }

    #[tokio::test]
    async fn every_restart_reapplies_all_ram_settings_and_zero_cap_keeps_zero_speed() {
        let bus = Arc::new(ScriptBus::new(false));
        let mut cfg = default_config();
        cfg.a2_speed.as_mut().unwrap().max_raw = 0;
        for restart in 1..=2 {
            let mut app = LingkongPull::start(
                bus.clone(),
                1,
                cfg,
                Default::default(),
                Default::default(),
                Default::default(),
            )
            .await
            .unwrap();
            wait_for_state(&app, |s| s.measured_current_raw.is_some()).await;
            assert_eq!(app.snapshot().torque_limit_raw, 0);
            assert_eq!(app.snapshot().speed_command_dps, 0.0);
            app.stop().await.unwrap();
            let writes = bus.ram_sent.lock().unwrap();
            assert_eq!(writes.len(), restart * 5);
            let mut expected = vec![global_torque_command(0)];
            expected.extend(ram_settings(&cfg));
            expected.push(global_torque_command(0)); // shutdown
            assert_eq!(&writes[(restart - 1) * 5..], &expected);
        }
        assert!(bus
            .a2_sent
            .lock()
            .unwrap()
            .iter()
            .all(|target| *target == (0, 0)));
        assert!(!bus.sent.lock().unwrap().contains(&0xA1));
    }

    #[tokio::test]
    async fn failed_live_ram_write_closes_without_sending_new_target() {
        let bus = Arc::new(ScriptBus::new(false));
        let cfg = default_config();
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            cfg,
            Default::default(),
            Default::default(),
            Default::default(),
        )
        .await
        .unwrap();
        wait_for_state(&app, |s| s.torque_limit_raw > 0).await;
        bus.corrupt_ram_reply.store(true, Ordering::Release);
        let mut next = cfg;
        next.a2_speed.as_mut().unwrap().max_raw = 700;
        next.a2_speed.as_mut().unwrap().homing_raw = 700;
        app.set_config(next).unwrap();
        let failed = wait_for_state(&app, |s| s.phase == Phase::Fault).await;
        assert!(failed
            .fault
            .unwrap()
            .contains("global torque limit update failed"));
        assert_eq!(failed.config, cfg);
        assert_eq!(failed.global_torque_limit_raw, None);
        assert_eq!(*bus.state.lock().unwrap(), 0x10);
        assert!(!bus
            .a2_sent
            .lock()
            .unwrap()
            .iter()
            .any(|&(limit, _)| limit == 700));
        assert!(app.set_config(cfg).unwrap_err().contains("faulted"));
        bus.corrupt_ram_reply.store(false, Ordering::Release);
        app.stop().await.unwrap();
    }

    #[tokio::test]
    async fn missing_global_limit_ack_stops_without_retrying_or_resuming_motion() {
        let bus = Arc::new(ScriptBus::new(false));
        let mut cfg = default_config();
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            cfg,
            Default::default(),
            Default::default(),
            Default::default(),
        )
        .await
        .unwrap();
        wait_for_state(&app, |s| s.torque_limit_raw == 41).await;
        bus.lost_replies.lock().unwrap().insert(0xC1, 1);
        cfg.a2_speed.as_mut().unwrap().homing_raw = 7;
        app.set_config(cfg).unwrap();
        let failed = wait_for_state(&app, |s| s.phase == Phase::Fault).await;
        assert!(failed
            .fault
            .unwrap()
            .contains("global torque limit update failed"));
        assert_eq!(failed.global_torque_limit_raw, None);
        assert_eq!(
            bus.ram_sent
                .lock()
                .unwrap()
                .iter()
                .filter(|&&frame| frame == global_torque_command(7))
                .count(),
            1
        );
        assert!(!bus
            .a2_sent
            .lock()
            .unwrap()
            .iter()
            .any(|&(limit, _)| limit == 7));
        assert_eq!(*bus.global_limit.lock().unwrap(), 0);
        assert_eq!(*bus.state.lock().unwrap(), 0x10);
        app.stop().await.unwrap();
    }

    #[tokio::test]
    async fn cancelling_during_a_global_limit_write_never_sends_the_new_motion() {
        let bus = Arc::new(ScriptBus::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let mut cfg = default_config();
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            cfg,
            Default::default(),
            stop.clone(),
            Default::default(),
        )
        .await
        .unwrap();
        wait_for_state(&app, |s| s.torque_limit_raw == 41).await;
        *bus.cancel_on_ram.lock().unwrap() = Some(stop);
        cfg.a2_speed.as_mut().unwrap().homing_raw = 80;
        app.set_config(cfg).unwrap();
        tokio::time::timeout(Duration::from_secs(2), async {
            while !app.task.as_ref().unwrap().is_finished() {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .unwrap();
        assert!(!bus
            .a2_sent
            .lock()
            .unwrap()
            .iter()
            .any(|&(limit, _)| limit == 80));
        app.stop().await.unwrap();
        assert_eq!(*bus.global_limit.lock().unwrap(), 0);
        assert_eq!(*bus.state.lock().unwrap(), 0x10);
    }

    #[tokio::test]
    async fn rehome_installs_the_slower_speed_before_increasing_global_torque() {
        let bus = Arc::new(ScriptBus::new(false));
        let mut cfg = default_config();
        let a2 = cfg.a2_speed.as_mut().unwrap();
        a2.max_raw = 600;
        a2.homing_raw = 400;
        a2.retract_raw = 200;
        let mut app = LingkongPull::start(
            bus.clone(),
            1,
            cfg,
            Default::default(),
            Default::default(),
            Default::default(),
        )
        .await
        .unwrap();
        wait_for_state(&app, |s| s.phase == Phase::Running).await;
        bus.feedback.lock().unwrap().speed_dps = 200;
        wait_for_state(&app, |s| s.global_torque_limit_raw == Some(200)).await;
        let before = bus.frames.lock().unwrap().len();
        app.rehome();
        wait_for_state(&app, |s| {
            s.phase == Phase::Homing && s.global_torque_limit_raw == Some(400)
        })
        .await;
        {
            let frames = bus.frames.lock().unwrap();
            let rehome = &frames[before..];
            let increase = rehome
                .iter()
                .position(|frame| *frame == global_torque_command(400))
                .unwrap();
            let speed = rehome[..increase]
                .iter()
                .rev()
                .find(|frame| frame[0] == 0xA2)
                .unwrap();
            assert_eq!(
                *speed,
                a2_command(200, speed_raw(cfg.homing_speed_m_s, &cfg))
            );
            assert!(rehome
                .iter()
                .filter(|frame| frame[0] == 0xC1)
                .all(|frame| frame[1] == 0x1E));
        }
        app.stop().await.unwrap();
    }
}
