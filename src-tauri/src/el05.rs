//! RobStride EDULITE 05 private protocol (EL05 使用说明书 260922, pp.29–44).
//! Classic CAN, 29-bit identifiers; packed motion words are big endian while
//! dictionary values are little endian. This session owns its control lifecycle.
use std::collections::{BTreeMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Result};
use can_transport::{CanBus, CanFilter, CanFrame, CanId, CanIoError, CanRx, FrameKind};
use serde::{Deserialize, Serialize};
use tokio::sync::{oneshot, Mutex};
use tokio::task::JoinHandle;

#[path = "el05_experiment.rs"]
pub mod experiment;
#[path = "el05_gravity.rs"]
pub mod gravity;
use experiment::{
    Experiment, ExperimentConfig, ExperimentData, ExperimentKind, ExperimentStatus, TelemetrySample,
};
use gravity::{
    GravityCompensation, GravityCompensationConfig, GravityCompensationStatus, GravityReference,
};

const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(750);
const CLIENT_TIMEOUT: Duration = Duration::from_secs(3);
const READ_TIMEOUT: Duration = Duration::from_millis(350);
const STEP: Duration = Duration::from_millis(20);
const MOTOR_TIMEOUT: u32 = 20_000; // 50 us ticks: one second.
const POLL: [u16; 5] = [0x7019, 0x701A, 0x701B, 0x701C, 0x7005];
// Use type-2/type-24 packed velocity directly, following the user's current
// direction measurements. Targets and dictionary mechVel remain separate.
const FEEDBACK_VELOCITY_SIGN: i8 = 1;

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
pub enum El05Mode {
    #[default]
    Mit,
    PositionPp,
    Velocity,
    Current,
    PositionCsp,
}
impl El05Mode {
    fn raw(self) -> u8 {
        match self {
            Self::Mit => 0,
            Self::PositionPp => 1,
            Self::Velocity => 2,
            Self::Current => 3,
            Self::PositionCsp => 5,
        }
    }
    fn from_raw(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Mit),
            1 => Some(Self::PositionPp),
            2 => Some(Self::Velocity),
            3 => Some(Self::Current),
            5 => Some(Self::PositionCsp),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(tag = "kind")]
pub enum El05Target {
    Mit {
        position_rad: f64,
        velocity_rad_s: f64,
        torque_nm: f64,
        kp: f64,
        kd: f64,
    },
    PositionPp {
        position_rad: f64,
        velocity_rad_s: f64,
        acceleration_rad_s2: f64,
        deceleration_rad_s2: Option<f64>,
        current_limit_a: f64,
    },
    Velocity {
        velocity_rad_s: f64,
        acceleration_rad_s2: f64,
        current_limit_a: f64,
    },
    Current {
        current_a: f64,
    },
    PositionCsp {
        position_rad: f64,
        velocity_rad_s: f64,
        current_limit_a: f64,
    },
}
impl El05Target {
    fn mode(self) -> El05Mode {
        match self {
            Self::Mit { .. } => El05Mode::Mit,
            Self::PositionPp { .. } => El05Mode::PositionPp,
            Self::Velocity { .. } => El05Mode::Velocity,
            Self::Current { .. } => El05Mode::Current,
            Self::PositionCsp { .. } => El05Mode::PositionCsp,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct El05Device {
    pub motor_id: u8,
    pub uid: Option<String>,
    pub online: bool,
    pub selected: bool,
    pub feedback_age_ms: Option<u64>,
    pub rx_count: u64,
}

#[derive(Clone, Debug, Serialize, PartialEq, Eq)]
pub struct El05Identity {
    pub motor_id: u8,
    /// MCU UID in wire byte order; this identifies the drive across ID changes.
    pub uid: String,
}

#[derive(Clone, Debug, Serialize, PartialEq, Eq)]
pub struct El05Version {
    pub motor_id: u8,
    /// Manual p.31 defines bytes 3..6, high byte first. It does not define a
    /// dotted firmware version, hardware revision, or the meaning of byte 7.
    pub value: u32,
    pub bytes: [u8; 4],
    pub raw: [u8; 8],
}

#[derive(Clone, Debug, Serialize)]
pub struct El05State {
    pub connected: bool,
    pub attached: bool,
    pub host_id: u8,
    pub motor_id: Option<u8>,
    pub identity: Option<El05Identity>,
    pub version: Option<El05Version>,
    pub mode: El05Mode,
    pub online: bool,
    pub enabled: bool,
    pub streaming: bool,
    pub reporting: bool,
    pub status_code: u8,
    pub position_rad: Option<f64>,
    pub mechanical_position_rad: Option<f64>,
    pub velocity_rad_s: Option<f64>,
    pub dictionary_velocity_rad_s: Option<f64>,
    pub dictionary_velocity_age_ms: Option<u64>,
    pub position_velocity_rad_s: Option<f64>,
    pub unwrapped_position_rad: Option<f64>,
    pub feedback_sequence: u64,
    pub feedback_time_s: Option<f64>,
    pub torque_nm: Option<f64>,
    pub current_a: Option<f64>,
    pub current_age_ms: Option<u64>,
    pub voltage_v: Option<f64>,
    pub temperature_c: Option<f64>,
    pub fault_bits: u8,
    pub fault_code: u32,
    pub warning_code: u32,
    pub faults: Vec<String>,
    pub feedback_age_ms: Option<u64>,
    pub feedback_rate_hz: Option<f64>,
    pub rx_count: u64,
    pub tx_count: u64,
    pub last_error: Option<String>,
    pub devices: Vec<El05Device>,
    pub experiment: ExperimentStatus,
    pub gravity_compensation: GravityCompensationStatus,
}
impl El05State {
    fn new(host_id: u8, connected: bool) -> Self {
        Self {
            connected,
            attached: false,
            host_id,
            motor_id: None,
            identity: None,
            version: None,
            mode: El05Mode::Mit,
            online: false,
            enabled: false,
            streaming: false,
            reporting: false,
            status_code: 0,
            position_rad: None,
            mechanical_position_rad: None,
            velocity_rad_s: None,
            dictionary_velocity_rad_s: None,
            dictionary_velocity_age_ms: None,
            position_velocity_rad_s: None,
            unwrapped_position_rad: None,
            feedback_sequence: 0,
            feedback_time_s: None,
            torque_nm: None,
            current_a: None,
            current_age_ms: None,
            voltage_v: None,
            temperature_c: None,
            fault_bits: 0,
            fault_code: 0,
            warning_code: 0,
            faults: Vec::new(),
            feedback_age_ms: None,
            feedback_rate_hz: None,
            rx_count: 0,
            tx_count: 0,
            last_error: None,
            devices: Vec::new(),
            experiment: ExperimentStatus::default(),
            gravity_compensation: GravityCompensationStatus::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
pub enum El05ParameterKind {
    Float,
    U8,
    U16,
    U32,
}
#[derive(Clone, Debug, Serialize)]
pub struct El05ParameterValue {
    pub index: u16,
    pub value: f64,
    pub kind: El05ParameterKind,
}

struct DeviceRecord {
    uid: Option<String>,
    seen: Instant,
    count: u64,
}
struct Shared {
    view: El05State,
    devices: BTreeMap<u8, DeviceRecord>,
    last_seen: Option<Instant>,
    feedback_times: VecDeque<Instant>,
    client_seen: Instant,
    target: Option<El05Target>,
    armed: bool,
    safety_hold: bool,
    /// Changes whenever a stop is requested, including while preparing a run.
    stop_revision: u64,
    /// Public Stop records cancellation before waiting for the CAN gate.
    stop_request_revision: u64,
    feedback_sequence: u64,
    mode_confirmed: bool,
    receive_error: Option<String>,
    pending: Option<(u8, u16, oneshot::Sender<Result<El05ParameterValue>>)>,
    pending_identity: Option<(u8, oneshot::Sender<El05Identity>)>,
    pending_version: Option<(u8, oneshot::Sender<El05Version>)>,
    scan_next: Option<u16>,
    epoch: Instant,
    current_seen: Option<Instant>,
    dictionary_velocity_seen: Option<Instant>,
    position_window: VecDeque<(Instant, f64)>,
    experiment: Experiment,
    gravity: GravityCompensation,
    last_sent_target: Option<El05Target>,
    last_target_sent: Option<Instant>,
    target_sequence: u64,
}
struct Inner {
    bus: Arc<dyn CanBus>,
    state: StdMutex<Shared>,
    gate: Mutex<()>,
    running: AtomicBool,
}
pub struct El05Session {
    inner: Arc<Inner>,
    tasks: StdMutex<Vec<JoinHandle<()>>>,
    start_gate: Mutex<()>,
}

impl El05Session {
    pub async fn start(bus: Arc<dyn CanBus>, host_id: u8) -> Result<Arc<Self>> {
        if host_id == 0xFE {
            bail!("EL05 host ID 254 is reserved for discovery replies");
        }
        let rx = bus
            .subscribe(CanFilter::pass_all_extended())
            .await
            .map_err(|e| anyhow!("subscribe EL05: {e}"))?;
        let inner = Arc::new(Inner {
            bus,
            state: StdMutex::new(Shared {
                view: El05State::new(host_id, true),
                devices: BTreeMap::new(),
                last_seen: None,
                feedback_times: VecDeque::new(),
                client_seen: Instant::now(),
                target: None,
                armed: false,
                safety_hold: false,
                stop_revision: 0,
                stop_request_revision: 0,
                feedback_sequence: 0,
                mode_confirmed: false,
                receive_error: None,
                pending: None,
                pending_identity: None,
                pending_version: None,
                scan_next: Some(0),
                epoch: Instant::now(),
                current_seen: None,
                dictionary_velocity_seen: None,
                position_window: VecDeque::new(),
                experiment: Experiment::default(),
                gravity: GravityCompensation::default(),
                last_sent_target: None,
                last_target_sent: None,
                target_sequence: 0,
            }),
            gate: Mutex::new(()),
            running: AtomicBool::new(true),
        });
        let session = Arc::new(Self {
            inner: inner.clone(),
            tasks: StdMutex::new(Vec::new()),
            start_gate: Mutex::new(()),
        });
        session.tasks.lock().unwrap().extend([
            tokio::spawn(receive_loop(inner.clone(), rx)),
            tokio::spawn(service_loop(inner)),
        ]);
        Ok(session)
    }

    pub fn touch_client(&self) {
        self.inner.state.lock().unwrap().client_seen = Instant::now();
    }
    pub fn snapshot(&self) -> El05State {
        let state = self.inner.state.lock().unwrap();
        let now = Instant::now();
        let mut view = state.view.clone();
        view.experiment = state.experiment.status.clone();
        view.gravity_compensation = state.gravity.status.clone();
        if view.gravity_compensation.active {
            view.gravity_compensation.elapsed_s = state.gravity.elapsed_s();
        }
        if view.experiment.active {
            view.experiment.elapsed_s = state.experiment.elapsed_s();
        }
        view.current_age_ms = state.current_seen.map(|t| t.elapsed().as_millis() as u64);
        view.dictionary_velocity_age_ms = state
            .dictionary_velocity_seen
            .map(|t| t.elapsed().as_millis() as u64);
        view.online = state
            .feedback_times
            .back()
            .is_some_and(|t| now.duration_since(*t) <= FEEDBACK_TIMEOUT);
        view.feedback_age_ms = state
            .feedback_times
            .back()
            .map(|t| now.duration_since(*t).as_millis() as u64);
        view.feedback_rate_hz = state
            .feedback_times
            .front()
            .zip(state.feedback_times.back())
            .and_then(|(first, last)| {
                let elapsed = last.duration_since(*first).as_secs_f64();
                (elapsed > 0.0 && now.duration_since(*last) <= FEEDBACK_TIMEOUT)
                    .then(|| (state.feedback_times.len() - 1) as f64 / elapsed)
            });
        view.streaming = state.target.is_some();
        view.devices = state
            .devices
            .iter()
            .map(|(&id, entry)| El05Device {
                motor_id: id,
                uid: entry.uid.clone(),
                online: now.duration_since(entry.seen) <= Duration::from_secs(10),
                selected: view.motor_id == Some(id),
                feedback_age_ms: Some(now.duration_since(entry.seen).as_millis() as u64),
                rx_count: entry.count,
            })
            .collect();
        view.faults = fault_names(view.fault_bits, view.fault_code, view.warning_code);
        view
    }
    pub fn devices(&self) -> Vec<El05Device> {
        self.snapshot().devices
    }
    pub async fn discover(&self) -> Result<()> {
        self.inner.check_running()?;
        self.inner.state.lock().unwrap().scan_next = Some(0);
        Ok(())
    }
    pub async fn attach(&self, motor_id: u8) -> Result<El05State> {
        let _guard = self.inner.gate.lock().await;
        self.inner.check_running()?;
        let host_id = self.inner.state.lock().unwrap().view.host_id;
        validate_motor_id(motor_id, host_id)?;
        if self.inner.state.lock().unwrap().view.motor_id.is_some() {
            self.inner.stop_locked(false).await?;
        }
        {
            let mut s = self.inner.state.lock().unwrap();
            let tx_count = s.view.tx_count;
            s.view = El05State::new(host_id, true);
            s.view.tx_count = tx_count;
            s.view.attached = true;
            s.view.motor_id = Some(motor_id);
            s.last_seen = None;
            s.feedback_times.clear();
            s.position_window.clear();
            s.current_seen = None;
            s.dictionary_velocity_seen = None;
            s.last_sent_target = None;
            s.last_target_sent = None;
            s.mode_confirmed = false;
            s.receive_error = None;
            s.gravity.invalidate_reference(
                "motor selection changed; capture the downward reference again",
            );
            s.client_seen = Instant::now();
        }
        self.inner.stop_locked(false).await?;
        let mode = self.inner.read_locked(0x7005).await?;
        let Some(mode) = El05Mode::from_raw(mode.value as u8) else {
            bail!("unsupported EL05 run_mode {}", mode.value);
        };
        {
            let mut s = self.inner.state.lock().unwrap();
            s.view.mode = mode;
            s.mode_confirmed = true;
        }
        self.inner.reporting_locked(true).await?;
        Ok(self.snapshot())
    }
    pub async fn set_mode(&self, mode: El05Mode) -> Result<El05State> {
        let _guard = self.inner.gate.lock().await;
        self.set_mode_locked(mode).await
    }
    async fn set_mode_locked(&self, mode: El05Mode) -> Result<El05State> {
        self.inner.selected()?;
        self.inner.stop_locked(false).await?;
        self.inner.state.lock().unwrap().mode_confirmed = false;
        tokio::time::sleep(Duration::from_millis(30)).await;
        self.inner
            .write_locked(0x7005, f64::from(mode.raw()))
            .await?;
        let readback = self.inner.read_locked(0x7005).await?;
        if readback.value != f64::from(mode.raw()) {
            bail!(
                "EL05 rejected mode {:?}: read back {}",
                mode,
                readback.value
            );
        }
        let mut s = self.inner.state.lock().unwrap();
        s.view.mode = mode;
        s.mode_confirmed = true;
        s.receive_error = None;
        s.view.last_error = None;
        drop(s);
        Ok(self.snapshot())
    }
    pub async fn enable(&self) -> Result<()> {
        let stop_request_revision = self.inner.state.lock().unwrap().stop_request_revision;
        let _guard = self.inner.gate.lock().await;
        self.inner.check_stop_request(Some(stop_request_revision))?;
        self.enable_with_cancellation_locked(Some(stop_request_revision))
            .await
    }
    async fn enable_with_cancellation_locked(
        &self,
        stop_request_revision: Option<u64>,
    ) -> Result<()> {
        let (id, host, mode) = self.inner.selected()?;
        {
            let s = self.inner.state.lock().unwrap();
            if s.experiment.status.active {
                bail!("stop the EL05 experiment before enabling");
            }
            if s.gravity.status.active {
                bail!("stop EL05 gravity compensation before enabling manual control");
            }
            if s.armed {
                bail!("EL05 is already enabled");
            }
            if s.safety_hold {
                bail!("confirm EL05 is stopped before enabling again");
            }
            if s.receive_error.is_some() {
                bail!("reconnect EL05 after a receive or mode synchronization error");
            }
            if !s.mode_confirmed {
                bail!("read or set the EL05 control mode before enabling");
            }
            if s.feedback_times
                .back()
                .is_none_or(|t| t.elapsed() > FEEDBACK_TIMEOUT)
            {
                bail!("EL05 is offline; select the motor again");
            }
            if s.view.fault_bits != 0 || s.view.fault_code != 0 {
                bail!("clear EL05 faults before enabling");
            }
        }
        // Always discard a previous goal before enabling. Position references
        // use the absolute dictionary position, never wrapped type-2 feedback.
        let neutral = match mode {
            El05Mode::Mit => El05Target::Mit {
                position_rad: 0.0,
                velocity_rad_s: 0.0,
                torque_nm: 0.0,
                kp: 0.0,
                kd: 0.0,
            },
            El05Mode::Current => El05Target::Current { current_a: 0.0 },
            El05Mode::Velocity => El05Target::Velocity {
                velocity_rad_s: 0.0,
                acceleration_rad_s2: 1.0,
                current_limit_a: 1.0,
            },
            El05Mode::PositionPp => El05Target::PositionPp {
                position_rad: self.inner.read_locked(0x7019).await?.value,
                velocity_rad_s: 0.5,
                acceleration_rad_s2: 1.0,
                deceleration_rad_s2: None,
                current_limit_a: 1.0,
            },
            El05Mode::PositionCsp => El05Target::PositionCsp {
                position_rad: self.inner.read_locked(0x7019).await?.value,
                velocity_rad_s: 0.5,
                current_limit_a: 1.0,
            },
        };
        self.inner
            .write_verified_locked(0x7028, f64::from(MOTOR_TIMEOUT))
            .await?;
        self.inner.check_stop_request(stop_request_revision)?;
        self.inner.target_locked(neutral, true).await?;
        self.inner.check_stop_request(stop_request_revision)?;
        let sequence = self.inner.state.lock().unwrap().feedback_sequence;
        let enabled = async {
            self.inner
                .send(frame(3, u16::from(host), id, [0; 8])?)
                .await?;
            self.inner.wait_status(sequence, 2, true).await
        }
        .await;
        if let Err(error) = enabled {
            let _ = self.inner.stop_locked(false).await;
            return Err(error);
        }
        if let Err(error) = self.inner.check_stop_request(stop_request_revision) {
            let _ = self.inner.stop_locked(false).await;
            return Err(error);
        }
        {
            let mut s = self.inner.state.lock().unwrap();
            s.armed = true;
            s.client_seen = Instant::now();
            s.view.last_error = None;
        }
        if let Err(error) = self.inner.target_locked(neutral, false).await {
            let _ = self.inner.stop_locked(false).await;
            return Err(error);
        }
        if mode == El05Mode::Mit {
            self.inner.state.lock().unwrap().target = Some(neutral);
        }
        Ok(())
    }
    pub async fn stop(&self) -> Result<()> {
        {
            let mut s = self.inner.state.lock().unwrap();
            s.stop_request_revision = s.stop_request_revision.wrapping_add(1);
            s.stop_revision = s.stop_revision.wrapping_add(1);
            s.target = None;
        }
        let _guard = self.inner.gate.lock().await;
        self.inner.stop_locked(false).await
    }
    pub async fn clear_fault(&self) -> Result<()> {
        let _guard = self.inner.gate.lock().await;
        self.inner.stop_locked(true).await?;
        let mut s = self.inner.state.lock().unwrap();
        s.view.fault_code = 0;
        s.view.warning_code = 0;
        s.view.last_error = None;
        Ok(())
    }
    pub async fn read_identity(&self) -> Result<El05Identity> {
        let _guard = self.inner.gate.lock().await;
        if {
            let s = self.inner.state.lock().unwrap();
            s.experiment.status.active || s.gravity.status.active
        } {
            bail!("stop the EL05 experiment or gravity compensation before querying identity");
        }
        let (id, host, _) = self.inner.selected()?;
        self.inner
            .identity_request_locked(id, frame(0, u16::from(host), id, [0; 8])?)
            .await?
            .ok_or_else(|| anyhow!("EL05 ID/UID query timed out for motor {id}"))
    }
    pub async fn read_version(&self) -> Result<El05Version> {
        let _guard = self.inner.gate.lock().await;
        let (id, host, _) = self.inner.selected()?;
        // This query is a special Stop frame, so end target streaming first.
        self.inner.stop_locked(false).await?;
        let (tx, rx) = oneshot::channel();
        {
            let mut s = self.inner.state.lock().unwrap();
            s.view.version = None;
            s.pending_version = Some((id, tx));
        }
        let result = async {
            self.inner
                .send(frame(4, u16::from(host), id, [0, 0xC4, 0, 0, 0, 0, 0, 0])?)
                .await?;
            match tokio::time::timeout(READ_TIMEOUT, rx).await {
                Ok(Ok(value)) => Ok(value),
                Ok(Err(_)) => bail!("EL05 version query cancelled"),
                Err(_) => bail!(
                    "EL05 motor version query timed out; firmware may not support this command"
                ),
            }
        }
        .await;
        self.inner.state.lock().unwrap().pending_version = None;
        result
    }
    pub async fn change_id(&self, new_id: u8) -> Result<El05Identity> {
        let _guard = self.inner.gate.lock().await;
        let (old_id, host, _) = self.inner.selected()?;
        // The 0x200A CAN_ID table explicitly limits configured IDs to 0..127.
        if new_id > 127 {
            bail!("EL05 new motor ID must be within 0..127 (manual CAN_ID range)");
        }
        validate_motor_id(new_id, host)?;
        if new_id == old_id {
            bail!("EL05 new motor ID is the same as the current ID");
        }
        self.inner.stop_locked(false).await?;
        self.inner
            .state
            .lock()
            .unwrap()
            .gravity
            .invalidate_reference("motor ID changed; capture the downward reference again");
        let original = self
            .inner
            .query_identity_locked(old_id, host)
            .await?
            .ok_or_else(|| {
                anyhow!("EL05 could not verify the current motor UID; ID was not changed")
            })?;
        if let Some(occupied) = self.inner.query_identity_locked(new_id, host).await? {
            bail!(
                "EL05 ID {new_id} is already occupied by MCU {}; ID was not changed",
                occupied.uid
            );
        }
        let reporting = {
            let mut s = self.inner.state.lock().unwrap();
            s.mode_confirmed = false;
            s.safety_hold = true;
            s.view.reporting
        };
        // Register before sending: type 7 replies immediately with a type-0
        // UID frame using the new address. A lost reply is recovered by query.
        let ack = self
            .inner
            .identity_request_locked(
                new_id,
                frame(
                    7,
                    (u16::from(new_id) << 8) | u16::from(host),
                    old_id,
                    [0; 8],
                )?,
            )
            .await;
        let verified = match ack {
            Ok(Some(identity)) if identity.uid == original.uid => Some(identity),
            _ => self
                .inner
                .query_identity_locked(new_id, host)
                .await
                .ok()
                .flatten()
                .filter(|identity| identity.uid == original.uid),
        };
        let Some(identity) = verified else {
            let still_at_old = self
                .inner
                .query_identity_locked(old_id, host)
                .await
                .ok()
                .flatten()
                .is_some_and(|identity| identity.uid == original.uid);
            let error = if still_at_old {
                format!("EL05 ID change was not confirmed; MCU {} still responds at ID {old_id}. Select it again before enabling", original.uid)
            } else {
                format!("EL05 ID change could not be verified at {old_id} or {new_id}; selection cleared. Scan and select MCU {} again", original.uid)
            };
            let mut s = self.inner.state.lock().unwrap();
            s.safety_hold = false;
            if !still_at_old {
                let tx_count = s.view.tx_count;
                s.view = El05State::new(host, true);
                s.view.tx_count = tx_count;
                s.last_seen = None;
                s.feedback_times.clear();
                s.devices.remove(&old_id);
                s.scan_next = Some(0);
            }
            s.view.last_error = Some(error.clone());
            bail!("{error}");
        };
        {
            let mut s = self.inner.state.lock().unwrap();
            let tx_count = s.view.tx_count;
            s.view = El05State::new(host, true);
            s.view.tx_count = tx_count;
            s.view.attached = true;
            s.view.motor_id = Some(new_id);
            s.view.identity = Some(identity.clone());
            s.last_seen = None;
            s.feedback_times.clear();
            s.devices.remove(&old_id);
            s.receive_error = None;
            s.client_seen = Instant::now();
        }
        // The address is now verified even if subsequent mode/feedback reads
        // fail. Keep the new selection and report that distinction to the UI.
        let synchronized = async {
            self.inner.stop_locked(false).await?;
            let mode = self.inner.read_locked(0x7005).await?;
            let mode = El05Mode::from_raw(mode.value as u8)
                .ok_or_else(|| anyhow!("unsupported EL05 run_mode {}", mode.value))?;
            self.inner.reporting_locked(reporting).await?;
            let mut s = self.inner.state.lock().unwrap();
            s.view.mode = mode;
            s.mode_confirmed = true;
            Ok::<_, anyhow::Error>(())
        }
        .await;
        if let Err(error) = synchronized {
            let error = format!("EL05 ID changed to {new_id}, but synchronization failed: {error}. Select ID {new_id} again before enabling");
            self.inner.state.lock().unwrap().view.last_error = Some(error.clone());
            bail!("{error}");
        }
        Ok(identity)
    }
    pub async fn set_zero(&self) -> Result<()> {
        let _guard = self.inner.gate.lock().await;
        let (id, host, mode) = self.inner.selected()?;
        if !matches!(mode, El05Mode::Mit | El05Mode::PositionCsp) {
            bail!("EL05 mechanical zero is supported in MIT and CSP modes; switch mode first");
        }
        if !self.inner.state.lock().unwrap().mode_confirmed {
            bail!("select the EL05 motor again before setting mechanical zero; its mode is not confirmed");
        }
        self.inner.stop_locked(false).await?;
        let sequence = {
            let mut s = self.inner.state.lock().unwrap();
            s.mode_confirmed = false;
            s.view.position_rad = None;
            s.view.mechanical_position_rad = None;
            s.view.unwrapped_position_rad = None;
            s.view.position_velocity_rad_s = None;
            s.position_window.clear();
            s.gravity.invalidate_reference(
                "mechanical zero changed; capture the downward reference again",
            );
            s.feedback_sequence
        };
        let result = async {
            self.inner
                .send(frame(6, u16::from(host), id, [1, 0, 0, 0, 0, 0, 0, 0])?)
                .await?;
            self.inner.wait_status(sequence, 0, true).await?;
            self.inner.read_locked(0x7019).await?;
            Ok::<_, anyhow::Error>(())
        }
        .await;
        if let Err(error) = result {
            let error = format!("EL05 mechanical zero/readback was not confirmed: {error}. Select the motor again to refresh its coordinates before enabling");
            self.inner.state.lock().unwrap().view.last_error = Some(error.clone());
            bail!("{error}");
        }
        let mut s = self.inner.state.lock().unwrap();
        s.mode_confirmed = true;
        s.view.last_error = None;
        Ok(())
    }
    pub async fn send_target(&self, target: El05Target, repeat: bool) -> Result<()> {
        let _guard = self.inner.gate.lock().await;
        let (_, _, mode) = self.inner.selected()?;
        if {
            let s = self.inner.state.lock().unwrap();
            s.experiment.status.active || s.gravity.status.active
        } {
            bail!("stop the EL05 experiment or gravity compensation before sending manual targets");
        }
        if mode != target.mode() {
            bail!("EL05 target mode does not match the selected mode");
        }
        if !self.inner.state.lock().unwrap().armed {
            bail!("enable EL05 before sending a target");
        }
        // Validate the complete sequence before sending its first limit write.
        validate_target(target)?;
        if let Err(error) = self.inner.target_locked(target, true).await {
            let _ = self.inner.stop_locked(false).await;
            return Err(error);
        }
        let mut s = self.inner.state.lock().unwrap();
        s.target = (repeat && mode != El05Mode::PositionPp).then_some(target);
        Ok(())
    }
    /// Stopping transmission alone leaves a drive's last target active. The UI
    /// action therefore sends the protocol Stop command as well.
    pub async fn stop_stream(&self) -> Result<()> {
        self.stop().await
    }
    pub async fn capture_gravity_reference(&self) -> Result<GravityReference> {
        let _start_guard = self.start_gate.lock().await;
        let _guard = self.inner.gate.lock().await;
        let (id, host, _) = self.inner.selected()?;
        {
            let s = self.inner.state.lock().unwrap();
            validate_gravity_capture(&s)?;
        }
        let identity = self
            .inner
            .query_identity_locked(id, host)
            .await?
            .ok_or_else(|| {
                anyhow!("EL05 UID must be verified before capturing the gravity reference")
            })?;
        let mut s = self.inner.state.lock().unwrap();
        validate_gravity_capture(&s)?;
        let position = s.view.unwrapped_position_rad.unwrap();
        let captured_at_s = s.epoch.elapsed().as_secs_f64();
        Ok(s.gravity.capture(id, identity.uid, position, captured_at_s))
    }

    pub async fn start_gravity_compensation(
        &self,
        config: GravityCompensationConfig,
    ) -> Result<()> {
        config.validate()?;
        let stop_request_revision = self.inner.state.lock().unwrap().stop_request_revision;
        let _start_guard = self.start_gate.lock().await;
        self.inner.check_stop_request(Some(stop_request_revision))?;
        {
            let s = self.inner.state.lock().unwrap();
            if s.experiment.status.active || s.gravity.status.active {
                bail!("stop the active EL05 experiment or gravity compensation before starting another mode");
            }
            validate_gravity_reference(&s, &config)?;
        }
        self.touch_client();
        let result = async {
            let stop_revision = {
                let _guard = self.inner.gate.lock().await;
                self.inner.check_stop_request(Some(stop_request_revision))?;
                let needs_stop = {
                    let s = self.inner.state.lock().unwrap();
                    s.armed || s.view.enabled || s.target.is_some() || s.safety_hold
                };
                if needs_stop {
                    self.inner.stop_locked(false).await?;
                }
                if self.inner.selected()?.2 != El05Mode::Mit {
                    self.set_mode_locked(El05Mode::Mit).await?;
                }
                self.inner.state.lock().unwrap().stop_revision
            };
            // Release the CAN gate between preparation and enabling so Stop
            // and device changes cancel preparation using the same revision.
            let _guard = self.inner.gate.lock().await;
            self.inner.check_stop_request(Some(stop_request_revision))?;
            {
                let s = self.inner.state.lock().unwrap();
                if s.stop_revision != stop_revision {
                    bail!("EL05 gravity compensation preparation cancelled by Stop");
                }
                validate_gravity_reference(&s, &config)?;
                if let Some(reason) = gravity_input_problem(&s, &config, false) {
                    bail!("{reason}");
                }
            }
            self.enable_with_cancellation_locked(Some(stop_request_revision))
                .await?;
            let mut s = self.inner.state.lock().unwrap();
            if s.stop_revision != stop_revision || !self.inner.running.load(Ordering::Acquire) {
                bail!("EL05 gravity compensation preparation cancelled by Stop");
            }
            validate_gravity_reference(&s, &config)?;
            if let Some(reason) = gravity_input_problem(&s, &config, true) {
                bail!("{reason}");
            }
            let position = s.view.unwrapped_position_rad.unwrap();
            let velocity = s.view.velocity_rad_s.unwrap();
            s.gravity.begin(config);
            s.target = Some(s.gravity.runtime_target(position, velocity));
            s.client_seen = Instant::now();
            Ok(())
        }
        .await;
        if result.is_err() {
            let _guard = self.inner.gate.lock().await;
            if self.inner.state.lock().unwrap().view.motor_id.is_some() {
                let _ = self.inner.stop_locked(false).await;
            }
        }
        result
    }

    pub async fn stop_gravity_compensation(&self) -> Result<()> {
        self.stop().await
    }

    pub async fn start_experiment(&self, mut config: ExperimentConfig) -> Result<()> {
        config.validate()?;
        let stop_request_revision = self.inner.state.lock().unwrap().stop_request_revision;
        // Serialize starts independently of the CAN gate so Stop can interrupt
        // slow preparation reads rather than waiting for the entire startup.
        let _start_guard = self.start_gate.lock().await;
        self.inner.check_stop_request(Some(stop_request_revision))?;
        {
            let s = self.inner.state.lock().unwrap();
            if s.experiment.status.active {
                bail!("an EL05 experiment is already active");
            }
            if s.gravity.status.active {
                bail!("stop EL05 gravity compensation before starting an experiment");
            }
        }
        self.touch_client();
        let result = async {
            // Stop an existing manual stream before any optional readbacks.
            // Those reads may time out; no motor target should remain active
            // while waiting, and the drive is enabled only after they finish.
            let stop_revision = {
                let _guard = self.inner.gate.lock().await;
                self.inner.check_stop_request(Some(stop_request_revision))?;
                let needs_stop = {
                    let s = self.inner.state.lock().unwrap();
                    s.armed || s.view.enabled || s.target.is_some() || s.safety_hold
                };
                if needs_stop {
                    self.inner.stop_locked(false).await?;
                }
                if config.kind != ExperimentKind::Record
                    && self.inner.selected()?.2 != El05Mode::Mit
                {
                    self.set_mode_locked(El05Mode::Mit).await?;
                }
                self.inner.state.lock().unwrap().stop_revision
            };

            let mut parameters = Vec::new();
            for index in [0x700B, 0x7026, 0x7028] {
                let _guard = self.inner.gate.lock().await;
                self.inner.check_stop_request(Some(stop_request_revision))?;
                if self.inner.state.lock().unwrap().stop_revision != stop_revision {
                    bail!("EL05 experiment preparation cancelled by Stop");
                }
                // Older firmware lacking an optional readback may still run.
                if let Ok(parameter) = self.inner.read_locked(index).await {
                    parameters.push(parameter);
                }
            }

            let _guard = self.inner.gate.lock().await;
            self.inner.check_stop_request(Some(stop_request_revision))?;
            {
                let s = self.inner.state.lock().unwrap();
                if s.stop_revision != stop_revision {
                    bail!("EL05 experiment preparation cancelled by Stop");
                }
                if s.receive_error.is_some()
                    || s.feedback_times
                        .back()
                        .is_none_or(|t| t.elapsed() > FEEDBACK_TIMEOUT)
                    || s.view.fault_bits != 0
                    || s.view.fault_code != 0
                {
                    bail!("EL05 experiment requires fresh feedback and no faults");
                }
                if config.max_temperature_c > 0.0
                    && s.view
                        .temperature_c
                        .is_some_and(|t| t >= config.max_temperature_c)
                {
                    bail!("EL05 temperature exceeds the experiment limit");
                }
            }
            if config.kind != ExperimentKind::Record {
                self.enable_with_cancellation_locked(Some(stop_request_revision))
                    .await?;
            }

            let (id, _, _) = self.inner.selected()?;
            let mut s = self.inner.state.lock().unwrap();
            if s.stop_revision != stop_revision || !self.inner.running.load(Ordering::Acquire) {
                bail!("EL05 experiment preparation cancelled by Stop");
            }
            let uid = s
                .view
                .identity
                .as_ref()
                .map(|identity| identity.uid.clone());
            config.origin_position_rad = s.view.unwrapped_position_rad.unwrap_or(0.0);
            s.target = config.target_with_feedback(
                0.0,
                s.view.position_rad.unwrap_or(0.0),
                config.origin_position_rad,
            );
            s.experiment.begin(config, id, uid);
            s.experiment.parameters = parameters;
            s.experiment.version = s.view.version.clone();
            s.client_seen = Instant::now();
            Ok(())
        }
        .await;
        if result.is_err() {
            let _guard = self.inner.gate.lock().await;
            let cleanup = {
                let mut s = self.inner.state.lock().unwrap();
                if s.experiment.status.active {
                    false
                } else {
                    s.target = None;
                    s.view.motor_id.is_some()
                }
            };
            if cleanup {
                let _ = self.inner.stop_locked(false).await;
            }
        }
        result
    }

    pub fn experiment_data(&self) -> ExperimentData {
        self.experiment_data_since(None)
    }

    pub fn experiment_data_since(&self, after_sequence: Option<u64>) -> ExperimentData {
        let s = self.inner.state.lock().unwrap();
        let cursor = after_sequence.unwrap_or(0);
        let command_cutoff = s
            .experiment
            .samples
            .iter()
            .rev()
            .find(|sample| sample.sequence <= cursor)
            .map(|sample| sample.t_s)
            .unwrap_or(-1.0);
        ExperimentData {
            feedback_velocity_sign: FEEDBACK_VELOCITY_SIGN,
            status: s.experiment.status.clone(),
            motor_id: s.experiment.motor_id,
            uid: s.experiment.uid.clone(),
            samples: s
                .experiment
                .samples
                .iter()
                .filter(|sample| sample.sequence > cursor)
                .cloned()
                .collect(),
            commands: s
                .experiment
                .commands
                .iter()
                .filter(|command| command.completed_t_s >= command_cutoff)
                .cloned()
                .collect(),
            parameters: s.experiment.parameters.clone(),
            version: s.experiment.version.clone(),
        }
    }
    pub async fn read_parameter(&self, index: u16) -> Result<El05ParameterValue> {
        let _guard = self.inner.gate.lock().await;
        if {
            let s = self.inner.state.lock().unwrap();
            s.experiment.status.active || s.gravity.status.active
        } {
            bail!("stop the EL05 experiment or gravity compensation before a synchronous parameter read");
        }
        self.inner.read_locked(index).await
    }
    pub async fn write_parameter(&self, index: u16, value: f64) -> Result<()> {
        if index == 0x7005 {
            let payload = encode_parameter(index, value)?;
            let mode =
                El05Mode::from_raw(payload[0]).ok_or_else(|| anyhow!("unsupported EL05 mode"))?;
            self.set_mode(mode).await?;
            return Ok(());
        }
        let _guard = self.inner.gate.lock().await;
        // General dictionary edits cannot bypass target validation or change
        // gains/limits underneath an active host stream.
        {
            let s = self.inner.state.lock().unwrap();
            if s.armed
                || s.safety_hold
                || s.view.enabled
                || s.experiment.status.active
                || s.gravity.status.active
            {
                bail!("confirm EL05 is stopped before editing dictionary parameters");
            }
        }
        self.inner.write_verified_locked(index, value).await
    }
    pub async fn shutdown(&self) -> Result<()> {
        let _guard = self.inner.gate.lock().await;
        if self.inner.state.lock().unwrap().view.motor_id.is_some() {
            self.inner.stop_locked(false).await?;
            self.inner.reporting_locked(false).await?;
        }
        self.force_stop();
        Ok(())
    }
    pub fn force_stop(&self) {
        self.inner.running.store(false, Ordering::Release);
        for task in self.tasks.lock().unwrap().drain(..) {
            task.abort();
        }
        let mut s = self.inner.state.lock().unwrap();
        s.stop_revision = s.stop_revision.wrapping_add(1);
        s.stop_request_revision = s.stop_request_revision.wrapping_add(1);
        s.target = None;
        s.experiment.finish("session closed");
        s.gravity
            .invalidate_reference("session closed; capture the downward reference again");
        s.armed = false;
        s.view.connected = false;
        s.view.attached = false;
        s.view.enabled = false;
    }
}
impl Drop for El05Session {
    fn drop(&mut self) {
        self.force_stop();
    }
}

impl Inner {
    fn check_stop_request(&self, expected: Option<u64>) -> Result<()> {
        self.check_running()?;
        if expected
            .is_some_and(|revision| self.state.lock().unwrap().stop_request_revision != revision)
        {
            bail!("EL05 preparation cancelled by Stop");
        }
        Ok(())
    }
    fn check_running(&self) -> Result<()> {
        if !self.running.load(Ordering::Acquire) {
            bail!("EL05 session is disconnected");
        }
        Ok(())
    }
    fn selected(&self) -> Result<(u8, u8, El05Mode)> {
        self.check_running()?;
        let s = self.state.lock().unwrap();
        Ok((
            s.view
                .motor_id
                .ok_or_else(|| anyhow!("select an EL05 motor first"))?,
            s.view.host_id,
            s.view.mode,
        ))
    }
    async fn send(&self, frame: CanFrame) -> Result<()> {
        self.bus
            .send(frame)
            .await
            .map_err(|e| anyhow!("EL05 CAN send: {e}"))?;
        let mut s = self.state.lock().unwrap();
        s.view.tx_count = s.view.tx_count.saturating_add(1);
        Ok(())
    }
    async fn wait_status(&self, sequence: u64, expected: u8, require_no_fault: bool) -> Result<()> {
        let deadline = Instant::now() + READ_TIMEOUT;
        loop {
            {
                let s = self.state.lock().unwrap();
                if s.feedback_sequence > sequence && s.view.status_code == expected {
                    if !require_no_fault || s.view.fault_bits == 0 {
                        return Ok(());
                    }
                    bail!("EL05 reported fault bits 0x{:02X}", s.view.fault_bits);
                }
                if expected != 0 {
                    if let Some(error) = &s.receive_error {
                        bail!("{error}");
                    }
                }
            }
            if Instant::now() >= deadline {
                bail!("EL05 did not confirm state {expected} before timeout");
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }
    async fn stop_locked(&self, clear: bool) -> Result<()> {
        let (id, host, _) = self.selected()?;
        let sequence = {
            let mut s = self.state.lock().unwrap();
            s.stop_revision = s.stop_revision.wrapping_add(1);
            s.target = None;
            s.experiment.finish("operator stop / mode or device change");
            s.gravity.finish("operator stop / mode or device change");
            s.armed = false;
            s.safety_hold = true;
            s.feedback_sequence
        };
        let result = async {
            self.send(frame(
                4,
                u16::from(host),
                id,
                [u8::from(clear), 0, 0, 0, 0, 0, 0, 0],
            )?)
            .await?;
            self.wait_status(sequence, 0, clear).await
        }
        .await;
        let mut s = self.state.lock().unwrap();
        if result.is_ok() {
            s.view.enabled = false;
            s.safety_hold = false;
        } else {
            s.view.last_error = Some(
                "EL05 stop was not confirmed; polling suspended so the drive watchdog can expire"
                    .into(),
            );
        }
        result
    }
    async fn reporting_locked(&self, enabled: bool) -> Result<()> {
        let (id, host, _) = self.selected()?;
        self.send(frame(
            24,
            u16::from(host),
            id,
            [1, 2, 3, 4, 5, 6, u8::from(enabled), 0],
        )?)
        .await?;
        self.state.lock().unwrap().view.reporting = enabled;
        Ok(())
    }
    async fn read_locked(&self, index: u16) -> Result<El05ParameterValue> {
        parameter_kind(index)?;
        let (id, host, _) = self.selected()?;
        let (tx, rx) = oneshot::channel();
        self.state.lock().unwrap().pending = Some((id, index, tx));
        let result = self.send(read_frame(id, host, index)?).await;
        if let Err(error) = result {
            self.state.lock().unwrap().pending = None;
            return Err(error);
        }
        let result = tokio::time::timeout(READ_TIMEOUT, rx).await;
        self.state.lock().unwrap().pending = None;
        match result {
            Ok(Ok(value)) => value,
            Ok(Err(_)) => bail!("EL05 parameter read cancelled"),
            Err(_) => bail!("EL05 parameter 0x{index:04X} read timed out"),
        }
    }
    async fn query_identity_locked(&self, id: u8, host: u8) -> Result<Option<El05Identity>> {
        self.identity_request_locked(id, frame(0, u16::from(host), id, [0; 8])?)
            .await
    }
    async fn identity_request_locked(
        &self,
        id: u8,
        request: CanFrame,
    ) -> Result<Option<El05Identity>> {
        let (tx, rx) = oneshot::channel();
        self.state.lock().unwrap().pending_identity = Some((id, tx));
        let result = async {
            self.send(request).await?;
            match tokio::time::timeout(READ_TIMEOUT, rx).await {
                Ok(Ok(identity)) => Ok(Some(identity)),
                Ok(Err(_)) => bail!("EL05 ID/UID query cancelled"),
                Err(_) => {
                    if let Some(error) = &self.state.lock().unwrap().receive_error {
                        bail!("EL05 ID/UID query failed: {error}");
                    }
                    Ok(None)
                }
            }
        }
        .await;
        self.state.lock().unwrap().pending_identity = None;
        result
    }
    async fn write_locked(&self, index: u16, value: f64) -> Result<()> {
        let (id, host, _) = self.selected()?;
        self.send(write_frame(id, host, index, value)?).await
    }
    async fn write_verified_locked(&self, index: u16, value: f64) -> Result<()> {
        let expected = encode_parameter(index, value)?;
        self.write_locked(index, value).await?;
        let actual = self.read_locked(index).await?;
        if encode_parameter(index, actual.value)? != expected {
            bail!("EL05 parameter 0x{index:04X} write verification failed: requested {value}, read back {}", actual.value);
        }
        Ok(())
    }
    async fn target_locked(&self, target: El05Target, limits: bool) -> Result<()> {
        let (id, host, mode) = self.selected()?;
        if mode != target.mode() {
            bail!("EL05 target mode mismatch");
        }
        if let El05Target::PositionPp {
            deceleration_rad_s2: Some(_),
            ..
        } = target
        {
            if limits {
                self.read_locked(0x702E).await?;
            }
        }
        for frame in target_frames(id, host, target, limits)? {
            let is_mit = frame.id().raw() >> 24 == 1;
            let can_id = frame.id().raw();
            let payload = frame.data().try_into().unwrap();
            let started = Instant::now();
            self.send(frame).await?;
            if is_mit {
                let completed = Instant::now();
                let mut s = self.state.lock().unwrap();
                s.target_sequence = s.target_sequence.saturating_add(1);
                let sequence = s.target_sequence;
                s.last_sent_target = Some(target);
                s.last_target_sent = Some(completed);
                s.experiment
                    .record_command(started, completed, sequence, target, can_id, payload);
            }
        }
        Ok(())
    }
}

fn validate_gravity_reference(s: &Shared, config: &GravityCompensationConfig) -> Result<()> {
    let motor_id = s
        .view
        .motor_id
        .ok_or_else(|| anyhow!("select an EL05 motor first"))?;
    if !s.gravity.reference_matches(
        &config.reference_id,
        motor_id,
        s.view
            .identity
            .as_ref()
            .map(|identity| identity.uid.as_str()),
    ) {
        bail!("EL05 downward reference is missing or belongs to an older motor/coordinate session; capture it again");
    }
    Ok(())
}

fn validate_gravity_capture(s: &Shared) -> Result<()> {
    if s.armed
        || s.view.enabled
        || s.safety_hold
        || s.target.is_some()
        || s.experiment.status.active
        || s.gravity.status.active
    {
        bail!("stop EL05 before capturing the pendulum downward reference");
    }
    if s.receive_error.is_some()
        || !s.mode_confirmed
        || s.feedback_times
            .back()
            .is_none_or(|t| t.elapsed() > gravity::FEEDBACK_TIMEOUT)
        || s.view.fault_bits != 0
        || s.view.fault_code != 0
        || s.view.unwrapped_position_rad.is_none_or(|q| !q.is_finite())
    {
        bail!("EL05 downward reference requires fresh feedback, a confirmed mode and no faults");
    }
    if s.view
        .velocity_rad_s
        .is_none_or(|v| !v.is_finite() || v.abs() > gravity::CAPTURE_MAX_SPEED)
        || s.view
            .position_velocity_rad_s
            .is_some_and(|v| !v.is_finite() || v.abs() > gravity::CAPTURE_MAX_SPEED)
    {
        bail!("hold the pendulum still vertically down (speed must be at most 0.1 rad/s), then capture its reference");
    }
    Ok(())
}

fn gravity_input_problem(
    s: &Shared,
    config: &GravityCompensationConfig,
    require_enabled: bool,
) -> Option<String> {
    if let Some(error) = &s.receive_error {
        return Some(error.clone());
    }
    let problem = if s
        .feedback_times
        .back()
        .is_none_or(|t| t.elapsed() > gravity::FEEDBACK_TIMEOUT)
    {
        Some("EL05 gravity compensation feedback timed out; capture the downward reference again")
    } else if s.view.fault_bits != 0 || s.view.fault_code != 0 {
        Some("EL05 gravity compensation stopped on a drive fault")
    } else if s.view.mode != El05Mode::Mit || !s.mode_confirmed {
        Some("EL05 gravity compensation requires confirmed MIT mode")
    } else if require_enabled && (!s.armed || !s.view.enabled) {
        Some("EL05 gravity compensation stopped because the drive left enabled state")
    } else if s.view.unwrapped_position_rad.is_none_or(|q| !q.is_finite())
        || s.view.velocity_rad_s.is_none_or(|v| !v.is_finite())
    {
        Some("EL05 gravity compensation requires finite position and velocity feedback")
    } else if s
        .view
        .velocity_rad_s
        .is_some_and(|v| v.abs() >= config.max_velocity_rad_s)
        || s.view
            .position_velocity_rad_s
            .is_some_and(|v| !v.is_finite() || v.abs() >= config.max_velocity_rad_s)
    {
        Some("EL05 gravity compensation speed limit exceeded; motor stopped")
    } else if s
        .view
        .temperature_c
        .is_none_or(|t| !t.is_finite() || t >= config.max_temperature_c)
    {
        Some("EL05 gravity compensation temperature limit exceeded or temperature unavailable; motor stopped")
    } else {
        None
    };
    problem.map(str::to_owned)
}

async fn service_loop(inner: Arc<Inner>) {
    let mut ticker = tokio::time::interval(STEP);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut ticks: usize = 0;
    while inner.running.load(Ordering::Acquire) {
        ticker.tick().await;
        let _guard = inner.gate.lock().await;
        if !inner.running.load(Ordering::Acquire) {
            break;
        }
        let gravity_end = {
            let mut s = inner.state.lock().unwrap();
            let reference_lost = if s.gravity.status.reference.is_some() {
                if s.feedback_times
                    .back()
                    .is_none_or(|t| t.elapsed() > gravity::FEEDBACK_TIMEOUT)
                {
                    Some(
                        "EL05 gravity feedback timed out; capture the downward reference again"
                            .to_string(),
                    )
                } else {
                    s.receive_error.clone()
                }
            } else {
                None
            };
            if let Some(reason) = reference_lost {
                let was_active = s.gravity.status.active;
                s.gravity.invalidate_reference(reason.clone());
                was_active.then_some(reason)
            } else if s.gravity.status.active {
                let config = s.gravity.status.config.as_ref().unwrap();
                let reason = if s.client_seen.elapsed() > CLIENT_TIMEOUT {
                    Some(
                        "EL05 gravity compensation UI heartbeat expired; motor stopped".to_string(),
                    )
                } else if let Err(error) = validate_gravity_reference(&s, config) {
                    Some(error.to_string())
                } else {
                    gravity_input_problem(&s, config, true)
                };
                if let Some(reason) = &reason {
                    s.gravity.finish(reason.clone());
                } else {
                    let position = s.view.unwrapped_position_rad.unwrap();
                    let velocity = s.view.velocity_rad_s.unwrap();
                    s.target = Some(s.gravity.runtime_target(position, velocity));
                }
                reason
            } else {
                None
            }
        };
        if let Some(reason) = gravity_end {
            let stopped = inner.stop_locked(false).await;
            inner.state.lock().unwrap().view.last_error = Some(match stopped {
                Ok(()) => reason,
                Err(error) => format!("{reason}; {error}"),
            });
        }
        let experiment_end = {
            let mut s = inner.state.lock().unwrap();
            if s.experiment.status.active {
                let reason = if s.client_seen.elapsed() > CLIENT_TIMEOUT {
                    Some("UI heartbeat expired".to_string())
                } else if s
                    .feedback_times
                    .back()
                    .is_none_or(|t| t.elapsed() > FEEDBACK_TIMEOUT)
                {
                    Some("experiment feedback timed out".to_string())
                } else if let Some(error) = &s.receive_error {
                    Some(error.clone())
                } else if s.view.fault_bits != 0 || s.view.fault_code != 0 {
                    Some("drive fault".to_string())
                } else if s
                    .experiment
                    .status
                    .config
                    .as_ref()
                    .is_some_and(|c| c.kind != ExperimentKind::Record)
                    && !s.armed
                {
                    Some("drive left enabled state".to_string())
                } else {
                    let temperature = s.view.temperature_c.unwrap_or(f64::NAN);
                    s.experiment.end_reason(temperature)
                };
                if let Some(reason) = &reason {
                    s.experiment.finish(reason.clone());
                } else {
                    let wrapped = s.view.position_rad.unwrap_or(0.0);
                    let unwrapped = s.view.unwrapped_position_rad.unwrap_or(wrapped);
                    let velocity = s.view.velocity_rad_s.unwrap_or(0.0);
                    s.target = s.experiment.runtime_target(wrapped, unwrapped, velocity);
                }
                reason
            } else {
                None
            }
        };
        if experiment_end.is_some() {
            if let Err(error) = inner.stop_locked(false).await {
                inner.state.lock().unwrap().view.last_error = Some(error.to_string());
            }
        }
        let reason = {
            let s = inner.state.lock().unwrap();
            if s.armed {
                if s.client_seen.elapsed() > CLIENT_TIMEOUT {
                    Some("EL05 UI heartbeat expired; motor stopped".to_string())
                } else if let Some(error) = &s.receive_error {
                    Some(error.clone())
                } else if s
                    .feedback_times
                    .back()
                    .is_none_or(|t| t.elapsed() > FEEDBACK_TIMEOUT)
                {
                    Some("EL05 feedback timed out; motor stopped".to_string())
                } else if s.view.fault_bits != 0 || s.view.fault_code != 0 {
                    Some("EL05 reported a drive fault; motor stopped".to_string())
                } else {
                    None
                }
            } else {
                None
            }
        };
        if let Some(reason) = reason {
            let stopped = inner.stop_locked(false).await;
            let mut s = inner.state.lock().unwrap();
            s.view.last_error = Some(match stopped {
                Ok(()) => reason,
                Err(error) => format!("{reason}; {error}"),
            });
        }
        // A failed Stop must not be kept alive by periodic dictionary reads.
        // Only an explicit retry may leave this hold; firmware times out too.
        if inner.state.lock().unwrap().safety_hold {
            continue;
        }
        let target = inner.state.lock().unwrap().target;
        if let Some(target) = target {
            if let Err(error) = inner.target_locked(target, false).await {
                let _ = inner.stop_locked(false).await;
                inner.state.lock().unwrap().view.last_error = Some(error.to_string());
                continue;
            }
        }
        if ticks % 5 == 0 {
            if let Ok((id, host, _)) = inner.selected() {
                // During experiments favor fresh Iq samples for static torque
                // regression, while still rotating the other dictionary reads.
                let active = inner.state.lock().unwrap().experiment.status.active;
                let index = if active && ticks % 25 != 0 {
                    0x701A
                } else {
                    POLL[(ticks / if active { 25 } else { 5 }) % POLL.len()]
                };
                if let Ok(frame) = read_frame(id, host, index) {
                    if let Err(error) = inner.send(frame).await {
                        inner.state.lock().unwrap().view.last_error = Some(error.to_string());
                    }
                }
            }
        }
        let probe = {
            let mut s = inner.state.lock().unwrap();
            s.scan_next.map(|next| {
                s.scan_next = (next < 255).then_some(next + 1);
                (next as u8, s.view.host_id)
            })
        };
        if let Some((id, host)) = probe {
            // Scan every eight-bit address, including ID zero. A UID reply is
            // distinguished from local echoes by its nonzero UID payload.
            if let Ok(frame) = frame(0, u16::from(host), id, [0; 8]) {
                if let Err(error) = inner.send(frame).await {
                    inner.state.lock().unwrap().view.last_error = Some(error.to_string());
                }
            }
        }
        ticks = ticks.wrapping_add(1);
    }
}

async fn receive_loop(inner: Arc<Inner>, mut rx: Box<dyn CanRx>) {
    while inner.running.load(Ordering::Acquire) {
        match rx.recv().await {
            Ok(frame) => observe_frame(&inner, frame),
            Err(CanIoError::Lagged { dropped }) => {
                let message = format!("EL05 receive queue dropped {dropped} frames");
                let mut s = inner.state.lock().unwrap();
                s.view.last_error = Some(message.clone());
                if s.experiment.status.active {
                    s.experiment.status.dropped_frames =
                        s.experiment.status.dropped_frames.saturating_add(dropped);
                    s.receive_error = Some(message.clone());
                }
                if s.gravity.status.reference.is_some() {
                    if s.gravity.status.active {
                        s.receive_error = Some(message.clone());
                    }
                    s.gravity.invalidate_reference(format!(
                        "{message}; capture the downward reference again"
                    ));
                }
            }
            Err(error) => {
                let message = format!("EL05 CAN receive failed: {error}");
                let mut s = inner.state.lock().unwrap();
                s.receive_error = Some(message.clone());
                s.view.last_error = Some(message);
                break;
            }
        }
    }
}

fn observe_frame(inner: &Inner, frame: CanFrame) {
    if frame.kind() != FrameKind::Data || frame.data().len() != 8 {
        return;
    }
    let CanId::Extended(id) = frame.id() else {
        return;
    };
    let ty = ((id >> 24) & 0x1F) as u8;
    let source = ((id >> 8) & 0xFF) as u8;
    let destination = (id & 0xFF) as u8;
    let data = frame.data();
    let now = Instant::now();
    let mut s = inner.state.lock().unwrap();
    let uid_reply = ty == 0 && destination == 0xFE && data.iter().any(|b| *b != 0);
    let addressed = destination == s.view.host_id && matches!(ty, 2 | 17 | 21 | 24);
    if !uid_reply && !addressed {
        return;
    }
    let entry = s.devices.entry(source).or_insert(DeviceRecord {
        uid: None,
        seen: now,
        count: 0,
    });
    entry.seen = now;
    entry.count = entry.count.saturating_add(1);
    if uid_reply {
        entry.uid = Some(hex::encode(data));
        let identity = El05Identity {
            motor_id: source,
            uid: hex::encode(data),
        };
        if s.view.motor_id == Some(source) {
            if s.view
                .identity
                .as_ref()
                .is_some_and(|previous| previous.uid != identity.uid)
                && s.gravity.status.reference.is_some()
            {
                s.gravity.invalidate_reference(
                    "EL05 motor UID changed; capture the downward reference again",
                );
                if s.armed {
                    s.receive_error = Some("EL05 motor UID changed; motor stopped".into());
                }
            }
            s.view.identity = Some(identity.clone());
        }
        // ID changes and collision probes can target an unselected address.
        if s.pending_identity
            .as_ref()
            .is_some_and(|(id, _)| *id == source)
        {
            if let Some((_, sender)) = s.pending_identity.take() {
                let _ = sender.send(identity);
            }
        }
    }
    if s.view.motor_id != Some(source) {
        return;
    }
    s.last_seen = Some(now);
    s.view.rx_count = s.view.rx_count.saturating_add(1);
    if uid_reply {
        return;
    }
    match ty {
        // EL05 firmware 0x0A050001 uses type 24 for periodic telemetry, with
        // the same payload/status layout as type 2 (verified on can3).
        // Host type-24 configuration frames have the opposite addresses and
        // are excluded by the source/destination checks above.
        2 | 24 => {
            // The version response shares type 2 but has no motion values.
            if data.starts_with(&[0x00, 0xC4, 0x56]) {
                if ty == 2 {
                    let bytes: [u8; 4] = data[3..7].try_into().unwrap();
                    let version = El05Version {
                        motor_id: source,
                        value: u32::from_be_bytes(bytes),
                        bytes,
                        raw: data.try_into().unwrap(),
                    };
                    s.view.version = Some(version.clone());
                    if s.pending_version
                        .as_ref()
                        .is_some_and(|(id, _)| *id == source)
                    {
                        if let Some((_, sender)) = s.pending_version.take() {
                            let _ = sender.send(version);
                        }
                    }
                }
                return;
            }
            if s.gravity.status.reference.is_some()
                && s.feedback_times
                    .back()
                    .is_some_and(|last| now.duration_since(*last) > gravity::FEEDBACK_TIMEOUT)
            {
                let active = s.gravity.status.active;
                s.gravity.invalidate_reference(
                    "EL05 gravity feedback gap; capture the downward reference again",
                );
                if active {
                    s.receive_error = Some("EL05 gravity feedback gap; motor stopped".into());
                }
            }
            s.view.fault_bits = ((id >> 16) & 0x3F) as u8;
            s.view.status_code = ((id >> 22) & 3) as u8;
            s.view.enabled = s.view.status_code == 2;
            s.feedback_sequence = s.feedback_sequence.saturating_add(1);
            s.view.position_rad = Some(unpack(data[0], data[1], -12.57, 12.57));
            s.view.velocity_rad_s =
                Some(f64::from(FEEDBACK_VELOCITY_SIGN) * unpack(data[2], data[3], -50.0, 50.0));
            s.view.torque_nm = Some(unpack(data[4], data[5], -6.0, 6.0));
            s.view.temperature_c = Some(f64::from(u16::from_be_bytes([data[6], data[7]])) / 10.0);
            let position = s.view.position_rad.unwrap();
            let unwrapped = match (s.view.unwrapped_position_rad, s.position_window.back()) {
                (Some(previous), Some((_, _))) => {
                    // Difference modulo the documented 16-bit position span.
                    let old_wrapped = (previous + 12.57).rem_euclid(25.14) - 12.57;
                    previous + (position - old_wrapped + 12.57).rem_euclid(25.14) - 12.57
                }
                _ => position,
            };
            if s.position_window
                .back()
                .is_some_and(|(t, _)| now.duration_since(*t) > Duration::from_millis(100))
            {
                s.position_window.clear();
            }
            s.position_window.push_back((now, unwrapped));
            while s.position_window.len() > 1
                && s.position_window
                    .front()
                    .is_some_and(|(t, _)| now.duration_since(*t) > Duration::from_millis(150))
            {
                s.position_window.pop_front();
            }
            s.view.position_velocity_rad_s = s.position_window.front().and_then(|(t, first)| {
                let dt = now.duration_since(*t).as_secs_f64();
                (dt >= 0.08).then(|| (unwrapped - first) / dt)
            });
            s.view.unwrapped_position_rad = Some(unwrapped);
            s.view.feedback_sequence = s.feedback_sequence;
            s.view.feedback_time_s = Some(now.duration_since(s.epoch).as_secs_f64());
            let sample = TelemetrySample {
                t_s: 0.0,
                sequence: s.feedback_sequence,
                can_id: id,
                payload: data.try_into().unwrap(),
                position_rad: position,
                unwrapped_position_rad: unwrapped,
                velocity_rad_s: s.view.velocity_rad_s.unwrap(),
                position_velocity_rad_s: s.view.position_velocity_rad_s,
                torque_nm: s.view.torque_nm.unwrap(),
                current_a: s.view.current_a,
                current_age_ms: s
                    .current_seen
                    .map(|t| now.duration_since(t).as_millis() as u64),
                dictionary_velocity_rad_s: s.view.dictionary_velocity_rad_s,
                dictionary_velocity_age_ms: s
                    .dictionary_velocity_seen
                    .map(|t| now.duration_since(t).as_millis() as u64),
                temperature_c: s.view.temperature_c.unwrap(),
                fault_bits: s.view.fault_bits,
                status_code: s.view.status_code,
                target: s.last_sent_target,
                target_sequence: s.target_sequence,
                target_age_ms: s
                    .last_target_sent
                    .map(|t| now.duration_since(t).as_millis() as u64),
            };
            s.experiment.push(sample, now);
            if s.experiment.status.static_friction.is_some() && s.experiment.status.active {
                // Apply a detected movement/angle guard to the next service
                // command, without waiting for a UI poll or waveform tick.
                let velocity = s.view.velocity_rad_s.unwrap();
                s.target = s.experiment.runtime_target(position, unwrapped, velocity);
            }
            s.feedback_times.push_back(now);
            while s.feedback_times.len() > 128
                || s.feedback_times
                    .front()
                    .is_some_and(|t| now.duration_since(*t) > Duration::from_secs(2))
            {
                s.feedback_times.pop_front();
            }
        }
        17 => {
            let index = u16::from_le_bytes([data[0], data[1]]);
            let result = if ((id >> 16) & 0xFF) != 0 {
                Err(anyhow!("EL05 parameter 0x{index:04X} read was rejected"))
            } else {
                decode_parameter(index, &data[4..8])
            };
            if let Ok(value) = &result {
                match index {
                    0x7005 => {
                        if let Some(mode) = El05Mode::from_raw(value.value as u8) {
                            // An external mode change invalidates our stream. The
                            // service loop stops on this mismatch before next send.
                            if mode != s.view.mode && s.armed {
                                s.receive_error = Some(
                                    "EL05 mode changed outside this session; motor stopped".into(),
                                );
                            }
                            s.view.mode = mode;
                        } else {
                            s.mode_confirmed = false;
                            s.receive_error = Some(format!(
                                "EL05 reported unsupported mode {}; motor stopped",
                                value.value
                            ));
                        }
                    }
                    0x7019 => s.view.mechanical_position_rad = Some(value.value),
                    0x701A => {
                        s.view.current_a = Some(value.value);
                        s.current_seen = Some(now);
                    }
                    0x701B => {
                        s.view.dictionary_velocity_rad_s = Some(value.value);
                        s.dictionary_velocity_seen = Some(now);
                    }
                    0x701C => s.view.voltage_v = Some(value.value),
                    _ => {}
                }
            }
            if s.pending
                .as_ref()
                .is_some_and(|(motor, idx, _)| *motor == source && *idx == index)
            {
                if let Some((_, _, sender)) = s.pending.take() {
                    let _ = sender.send(result);
                }
            }
        }
        21 => {
            s.view.fault_code = u32::from_le_bytes(data[0..4].try_into().unwrap());
            s.view.warning_code = u32::from_le_bytes(data[4..8].try_into().unwrap());
        }
        _ => {}
    }
}

fn validate_motor_id(id: u8, host: u8) -> Result<()> {
    if id == host {
        bail!("EL05 motor ID must differ from the host ID");
    }
    Ok(())
}
fn frame(ty: u8, data: u16, destination: u8, payload: [u8; 8]) -> Result<CanFrame> {
    let id = (u32::from(ty) << 24) | (u32::from(data) << 8) | u32::from(destination);
    CanFrame::new_data(CanId::Extended(id), &payload).map_err(|e| anyhow!("build EL05 frame: {e}"))
}
fn read_frame(id: u8, host: u8, index: u16) -> Result<CanFrame> {
    let mut data = [0; 8];
    data[..2].copy_from_slice(&index.to_le_bytes());
    frame(17, u16::from(host), id, data)
}
fn write_frame(id: u8, host: u8, index: u16, value: f64) -> Result<CanFrame> {
    let raw = encode_parameter(index, value)?;
    let mut data = [0; 8];
    data[..2].copy_from_slice(&index.to_le_bytes());
    data[4..].copy_from_slice(&raw);
    frame(18, u16::from(host), id, data)
}
fn check_range(value: f64, min: f64, max: f64, label: &str) -> Result<()> {
    if !value.is_finite() || !(min..=max).contains(&value) {
        bail!("EL05 {label} must be finite and within {min}..={max}");
    }
    Ok(())
}
fn parameter_kind(index: u16) -> Result<El05ParameterKind> {
    match index {
        0x7005 | 0x7029 | 0x702C | 0x702D => Ok(El05ParameterKind::U8),
        0x7026 => Ok(El05ParameterKind::U16),
        0x7028 => Ok(El05ParameterKind::U32),
        0x7006
        | 0x700A
        | 0x700B
        | 0x7010
        | 0x7011
        | 0x7014
        | 0x7016..=0x701C
        | 0x701E..=0x7022
        | 0x7024
        | 0x7025
        | 0x702B
        | 0x702E => Ok(El05ParameterKind::Float),
        _ => bail!("EL05 parameter 0x{index:04X} is not in the manual's supported dictionary"),
    }
}
fn encode_parameter(index: u16, value: f64) -> Result<[u8; 4]> {
    let kind = parameter_kind(index)?;
    if (0x7019..=0x701C).contains(&index) {
        bail!("EL05 parameter 0x{index:04X} is read-only");
    }
    let (min, max) = match index {
        0x7005 => (0.0, 5.0),
        0x7006 => (-11.0, 11.0),
        0x700A => (-50.0, 50.0),
        0x700B => (0.0, 6.0),
        0x7014 | 0x7021 => (0.0, 1.0),
        0x7017 | 0x7024 => (0.0, 50.0),
        0x7018 => (0.0, 11.0),
        0x7016 | 0x702B => (-(f32::MAX as f64), f32::MAX as f64),
        // EL05 0x0A050001 reports zero at power-on. Preserve/read back that
        // firmware value without guessing its undocumented timing semantics.
        0x7026 => (0.0, 65535.0),
        0x7028 => (0.0, u32::MAX as f64),
        0x7029 | 0x702C | 0x702D => (0.0, 1.0),
        _ => (0.0, f32::MAX as f64),
    };
    check_range(value, min, max, &format!("parameter 0x{index:04X}"))?;
    if kind != El05ParameterKind::Float && value.fract() != 0.0 {
        bail!("EL05 parameter 0x{index:04X} requires an integer");
    }
    if index == 0x7005 && El05Mode::from_raw(value as u8).is_none() {
        bail!("EL05 run_mode must be 0, 1, 2, 3, or 5");
    }
    Ok(match kind {
        El05ParameterKind::Float => (value as f32).to_le_bytes(),
        El05ParameterKind::U8 => [value as u8, 0, 0, 0],
        El05ParameterKind::U16 => {
            let b = (value as u16).to_le_bytes();
            [b[0], b[1], 0, 0]
        }
        El05ParameterKind::U32 => (value as u32).to_le_bytes(),
    })
}
fn decode_parameter(index: u16, data: &[u8]) -> Result<El05ParameterValue> {
    let kind = parameter_kind(index)?;
    let value = match kind {
        El05ParameterKind::Float => f64::from(f32::from_le_bytes(data.try_into()?)),
        El05ParameterKind::U8 => f64::from(data[0]),
        El05ParameterKind::U16 => f64::from(u16::from_le_bytes([data[0], data[1]])),
        El05ParameterKind::U32 => f64::from(u32::from_le_bytes(data.try_into()?)),
    };
    if !value.is_finite() {
        bail!("EL05 parameter 0x{index:04X} returned a non-finite value");
    }
    Ok(El05ParameterValue { index, value, kind })
}
fn validate_target(target: El05Target) -> Result<()> {
    for (index, value) in target_parameters(target, true) {
        encode_parameter(index, value)?;
    }
    match target {
        El05Target::Mit {
            position_rad,
            velocity_rad_s,
            torque_nm,
            kp,
            kd,
        } => {
            for (value, min, max, label) in [
                (position_rad, -12.57, 12.57, "position (rad)"),
                (velocity_rad_s, -50.0, 50.0, "velocity (rad/s)"),
                (torque_nm, -6.0, 6.0, "torque (Nm)"),
                (kp, 0.0, 500.0, "Kp"),
                (kd, 0.0, 5.0, "Kd"),
            ] {
                check_range(value, min, max, label)?;
            }
        }
        El05Target::PositionPp {
            velocity_rad_s,
            acceleration_rad_s2,
            deceleration_rad_s2,
            ..
        } => {
            if velocity_rad_s <= 0.0
                || acceleration_rad_s2 <= 0.0
                || deceleration_rad_s2.is_some_and(|value| value <= 0.0)
            {
                bail!("EL05 PP velocity, acceleration and deceleration must be positive");
            }
        }
        El05Target::PositionCsp { velocity_rad_s, .. } => {
            if velocity_rad_s <= 0.0 {
                bail!("EL05 CSP velocity limit must be positive");
            }
        }
        El05Target::Velocity {
            acceleration_rad_s2,
            ..
        } => {
            if acceleration_rad_s2 <= 0.0 {
                bail!("EL05 velocity acceleration must be positive");
            }
        }
        _ => {}
    }
    Ok(())
}
fn target_parameters(target: El05Target, limits: bool) -> Vec<(u16, f64)> {
    let mut out = Vec::new();
    match target {
        El05Target::Mit { .. } => {}
        El05Target::Current { current_a } => out.push((0x7006, current_a)),
        El05Target::Velocity {
            velocity_rad_s,
            acceleration_rad_s2,
            current_limit_a,
        } => {
            if limits {
                out.extend([(0x7018, current_limit_a), (0x7022, acceleration_rad_s2)]);
            }
            out.push((0x700A, velocity_rad_s));
        }
        El05Target::PositionPp {
            position_rad,
            velocity_rad_s,
            acceleration_rad_s2,
            deceleration_rad_s2,
            current_limit_a,
        } => {
            if limits {
                out.extend([
                    (0x7018, current_limit_a),
                    (0x7024, velocity_rad_s),
                    (0x7025, acceleration_rad_s2),
                ]);
                if let Some(value) = deceleration_rad_s2 {
                    out.push((0x702E, value));
                }
            }
            out.push((0x7016, position_rad));
        }
        El05Target::PositionCsp {
            position_rad,
            velocity_rad_s,
            current_limit_a,
        } => {
            if limits {
                out.extend([(0x7018, current_limit_a), (0x7017, velocity_rad_s)]);
            }
            out.push((0x7016, position_rad));
        }
    }
    out
}
fn target_frames(id: u8, host: u8, target: El05Target, limits: bool) -> Result<Vec<CanFrame>> {
    validate_target(target)?;
    if let El05Target::Mit {
        position_rad,
        velocity_rad_s,
        torque_nm,
        kp,
        kd,
    } = target
    {
        let mut data = [0; 8];
        for (chunk, (value, min, max)) in data.chunks_exact_mut(2).zip([
            (position_rad, -12.57, 12.57),
            (velocity_rad_s, -50.0, 50.0),
            (kp, 0.0, 500.0),
            (kd, 0.0, 5.0),
        ]) {
            chunk.copy_from_slice(&pack(value, min, max).to_be_bytes());
        }
        return Ok(vec![frame(1, pack(torque_nm, -6.0, 6.0), id, data)?]);
    }
    target_parameters(target, limits)
        .into_iter()
        .map(|(index, value)| write_frame(id, host, index, value))
        .collect()
}
fn pack(value: f64, min: f64, max: f64) -> u16 {
    ((value - min) * 65535.0 / (max - min)) as u16
}
fn unpack(high: u8, low: u8, min: f64, max: f64) -> f64 {
    f64::from(u16::from_be_bytes([high, low])) * (max - min) / 65535.0 + min
}
fn fault_names(bits: u8, code: u32, warnings: u32) -> Vec<String> {
    let mut names = Vec::new();
    for (bit, name) in [
        (0, "欠压"),
        (1, "驱动故障"),
        (2, "过温"),
        (3, "磁编码器故障"),
        (4, "堵转过载"),
        (5, "编码器未标定"),
    ] {
        if bits & (1 << bit) != 0 {
            names.push(name.into());
        }
    }
    for (bit, name) in [
        (0, "电机过温"),
        (1, "驱动芯片故障"),
        (2, "母线欠压"),
        (3, "母线过压"),
        (4, "B相过流"),
        (5, "C相过流"),
        (7, "编码器未标定"),
        (8, "硬件识别故障"),
        (9, "位置初始化故障"),
        (14, "堵转过载保护"),
        (16, "A相过流"),
    ] {
        if code & (1 << bit) != 0 && !names.iter().any(|n| n == name) {
            names.push(name.into());
        }
    }
    if warnings & 1 != 0 {
        names.push("电机过温预警".into());
    }
    names
}

#[cfg(test)]
mod tests {
    use super::*;
    use can_transport::CanCapabilities;
    use std::sync::atomic::AtomicU8;
    use tokio::sync::broadcast;

    struct MockBus {
        tx: broadcast::Sender<CanFrame>,
        sent: StdMutex<Vec<CanFrame>>,
        values: StdMutex<BTreeMap<u16, [u8; 4]>>,
        enabled: AtomicBool,
        suppress_stop_ack: AtomicBool,
        suppress_enable_ack: AtomicBool,
        skip_read_replies: StdMutex<BTreeMap<u16, usize>>,
        suppress_motion_feedback: AtomicBool,
        reject_writes: AtomicBool,
        motor_id: AtomicU8,
        occupied_id: AtomicU8,
        suppress_id_ack: AtomicBool,
        reject_id_change: AtomicBool,
        hide_after_id_change: AtomicBool,
        wrong_uid_after_id_change: AtomicBool,
        suppress_version_reply: AtomicBool,
        suppress_zero_ack: AtomicBool,
        suppress_stop_after_id_change: AtomicBool,
    }
    impl MockBus {
        fn new() -> Arc<Self> {
            let (tx, _) = broadcast::channel(256);
            Arc::new(Self {
                tx,
                sent: StdMutex::new(Vec::new()),
                values: StdMutex::new(BTreeMap::from([
                    (0x7005, [0; 4]),
                    (0x7019, 1.5f32.to_le_bytes()),
                ])),
                enabled: AtomicBool::new(false),
                suppress_stop_ack: AtomicBool::new(false),
                suppress_enable_ack: AtomicBool::new(false),
                skip_read_replies: StdMutex::new(BTreeMap::new()),
                suppress_motion_feedback: AtomicBool::new(false),
                reject_writes: AtomicBool::new(false),
                motor_id: AtomicU8::new(1),
                occupied_id: AtomicU8::new(255),
                suppress_id_ack: AtomicBool::new(false),
                reject_id_change: AtomicBool::new(false),
                hide_after_id_change: AtomicBool::new(false),
                wrong_uid_after_id_change: AtomicBool::new(false),
                suppress_version_reply: AtomicBool::new(false),
                suppress_zero_ack: AtomicBool::new(false),
                suppress_stop_after_id_change: AtomicBool::new(false),
            })
        }
        fn feedback(&self) {
            let state = if self.enabled.load(Ordering::Acquire) {
                2
            } else {
                0
            };
            let mut bytes = [0; 8];
            let position = f32::from_le_bytes(self.values.lock().unwrap()[&0x7019]);
            let velocity = f32::from_le_bytes(
                self.values
                    .lock()
                    .unwrap()
                    .get(&0x701B)
                    .copied()
                    .unwrap_or([0; 4]),
            );
            bytes[..2].copy_from_slice(&pack(f64::from(position), -12.57, 12.57).to_be_bytes());
            // Model the selected shared feedback direction without a sign flip.
            bytes[2..4].copy_from_slice(&pack(f64::from(velocity), -50.0, 50.0).to_be_bytes());
            bytes[4..6].copy_from_slice(&pack(0.0, -6.0, 6.0).to_be_bytes());
            bytes[6..].copy_from_slice(&250u16.to_be_bytes());
            let _ = self.tx.send(
                frame(
                    2,
                    (state << 14) | u16::from(self.motor_id.load(Ordering::Acquire)),
                    253,
                    bytes,
                )
                .unwrap(),
            );
        }
        fn identity(&self) {
            let id = self.motor_id.load(Ordering::Acquire);
            if id != 1 && self.hide_after_id_change.load(Ordering::Acquire) {
                return;
            }
            let uid = if id != 1 && self.wrong_uid_after_id_change.load(Ordering::Acquire) {
                [9; 8]
            } else {
                [1, 2, 3, 4, 5, 6, 7, 8]
            };
            let _ = self.tx.send(frame(0, u16::from(id), 254, uid).unwrap());
        }
    }
    struct MockRx(broadcast::Receiver<CanFrame>);
    #[async_trait::async_trait]
    impl CanRx for MockRx {
        async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
            self.0.recv().await.map_err(|error| match error {
                broadcast::error::RecvError::Lagged(dropped) => CanIoError::Lagged { dropped },
                broadcast::error::RecvError::Closed => CanIoError::Disconnected,
            })
        }
        fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
            match self.0.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(broadcast::error::TryRecvError::Empty) => Ok(None),
                Err(_) => Err(CanIoError::Disconnected),
            }
        }
    }
    #[async_trait::async_trait]
    impl CanBus for MockBus {
        async fn send(&self, request: CanFrame) -> std::result::Result<(), CanIoError> {
            self.sent.lock().unwrap().push(request.clone());
            let id = request.id().raw();
            let ty = (id >> 24) & 0x1F;
            if ty == 0 && id as u8 == self.occupied_id.load(Ordering::Acquire) {
                let _ = self
                    .tx
                    .send(frame(0, u16::from(id as u8), 254, [9; 8]).unwrap());
            }
            let motor_id = self.motor_id.load(Ordering::Acquire);
            if id as u8 != motor_id {
                return Ok(());
            }
            let data = request.data();
            let index = u16::from_le_bytes([data[0], data[1]]);
            match ty {
                0 => self.identity(),
                3 => {
                    self.enabled.store(true, Ordering::Release);
                    if !self.suppress_enable_ack.load(Ordering::Acquire) {
                        self.feedback();
                    }
                }
                4 if data[1] == 0xC4 => {
                    if !self.suppress_version_reply.load(Ordering::Acquire) {
                        let _ = self.tx.send(
                            frame(2, u16::from(motor_id), 253, [0, 0xC4, 0x56, 10, 5, 0, 1, 7])
                                .unwrap(),
                        );
                    }
                }
                4 => {
                    if !self.suppress_stop_ack.load(Ordering::Acquire)
                        && !(motor_id != 1
                            && self.suppress_stop_after_id_change.load(Ordering::Acquire))
                    {
                        self.enabled.store(false, Ordering::Release);
                        self.feedback();
                    }
                }
                17 => {
                    let skip_reply = {
                        let mut skips = self.skip_read_replies.lock().unwrap();
                        if let Some(remaining) = skips.get_mut(&index) {
                            if *remaining > 0 {
                                *remaining -= 1;
                                true
                            } else {
                                false
                            }
                        } else {
                            false
                        }
                    };
                    if skip_reply {
                        return Ok(());
                    }
                    let value = self
                        .values
                        .lock()
                        .unwrap()
                        .get(&index)
                        .copied()
                        .unwrap_or([0; 4]);
                    let mut payload = [0; 8];
                    payload[..2].copy_from_slice(&index.to_le_bytes());
                    payload[4..].copy_from_slice(&value);
                    let _ = self
                        .tx
                        .send(frame(17, u16::from(motor_id), 253, payload).unwrap());
                }
                18 => {
                    if !self.reject_writes.load(Ordering::Acquire) {
                        self.values
                            .lock()
                            .unwrap()
                            .insert(index, data[4..8].try_into().unwrap());
                    }
                    self.feedback();
                }
                6 => {
                    self.values
                        .lock()
                        .unwrap()
                        .insert(0x7019, 0.0f32.to_le_bytes());
                    if !self.suppress_zero_ack.load(Ordering::Acquire) {
                        self.feedback();
                    }
                }
                7 => {
                    if !self.reject_id_change.load(Ordering::Acquire) {
                        self.motor_id.store((id >> 16) as u8, Ordering::Release);
                        if !self.suppress_id_ack.load(Ordering::Acquire) {
                            self.identity();
                        }
                    }
                }
                1 | 24 => {
                    if !self.suppress_motion_feedback.load(Ordering::Acquire) {
                        self.feedback();
                    }
                }
                _ => {}
            }
            Ok(())
        }
        async fn subscribe(
            &self,
            _filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            Ok(Box::new(MockRx(self.tx.subscribe())))
        }
        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: false,
                max_dlen: 8,
            }
        }
    }
    async fn attached() -> (Arc<MockBus>, Arc<El05Session>) {
        let bus = MockBus::new();
        let app = El05Session::start(bus.clone(), 253).await.unwrap();
        app.attach(1).await.unwrap();
        (bus, app)
    }

    fn experiment_config(kind: ExperimentKind) -> ExperimentConfig {
        ExperimentConfig {
            kind,
            duration_s: 2.0,
            velocity_rad_s: 0.3,
            torque_nm: 0.3,
            kd: 0.5,
            max_temperature_c: 60.0,
            pendulum: None,
            ..Default::default()
        }
    }

    async fn gravity_config(app: &El05Session) -> GravityCompensationConfig {
        let reference = app.capture_gravity_reference().await.unwrap();
        GravityCompensationConfig {
            reference_id: reference.id,
            mass_kg: 0.5,
            length_m: 0.18,
            arm_mass_kg: 0.0,
            compensation_scale: 1.0,
            damping_nm_s_rad: 0.1,
            torque_limit_nm: 1.0,
            max_velocity_rad_s: 3.0,
            max_temperature_c: 70.0,
            ramp_s: 1.0,
        }
    }

    #[tokio::test]
    async fn gravity_reference_requires_stopped_stationary_feedback() {
        let (bus, app) = attached().await;
        let first = gravity_config(&app).await;
        app.inner.state.lock().unwrap().view.velocity_rad_s = Some(0.5);
        assert!(app
            .capture_gravity_reference()
            .await
            .unwrap_err()
            .to_string()
            .contains("still"));
        bus.feedback();
        tokio::time::sleep(Duration::from_millis(5)).await;
        let second = gravity_config(&app).await;
        assert_ne!(first.reference_id, second.reference_id);
        assert!(app.start_gravity_compensation(first).await.is_err());
        app.enable().await.unwrap();
        assert!(app
            .capture_gravity_reference()
            .await
            .unwrap_err()
            .to_string()
            .contains("stop"));
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn gravity_mode_excludes_experiments_manual_targets_and_synchronous_reads() {
        let (_, app) = attached().await;
        let config = gravity_config(&app).await;
        app.start_gravity_compensation(config.clone())
            .await
            .unwrap();
        let state = app.snapshot();
        assert!(state.enabled && state.streaming && state.gravity_compensation.active);
        assert!(!state.experiment.active);
        assert!(app
            .start_experiment(experiment_config(ExperimentKind::Record))
            .await
            .is_err());
        assert!(app
            .start_gravity_compensation(config.clone())
            .await
            .is_err());
        assert!(app.enable().await.is_err());
        assert!(app.read_parameter(0x7019).await.is_err());
        assert!(app.write_parameter(0x700B, 0.1).await.is_err());
        assert!(app
            .send_target(
                El05Target::Mit {
                    position_rad: 0.0,
                    velocity_rad_s: 0.0,
                    torque_nm: 1.0,
                    kp: 0.0,
                    kd: 0.0,
                },
                true
            )
            .await
            .is_err());
        app.stop_gravity_compensation().await.unwrap();
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming && !state.gravity_compensation.active);
        assert_eq!(state.gravity_compensation.command_torque_nm, 0.0);
        // An ordinary stop preserves the stationary encoder coordinate session.
        app.start_gravity_compensation(config).await.unwrap();
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn gravity_reference_is_invalid_after_zero_selection_and_feedback_gap() {
        let (bus, app) = attached().await;
        let before_zero = gravity_config(&app).await;
        app.set_zero().await.unwrap();
        assert!(app.snapshot().gravity_compensation.reference.is_none());
        assert!(app.start_gravity_compensation(before_zero).await.is_err());
        let before_selection = gravity_config(&app).await;
        app.attach(1).await.unwrap();
        assert!(app.snapshot().gravity_compensation.reference.is_none());
        assert!(app
            .start_gravity_compensation(before_selection)
            .await
            .is_err());
        let before_gap = gravity_config(&app).await;
        app.start_gravity_compensation(before_gap.clone())
            .await
            .unwrap();
        {
            let _guard = app.inner.gate.lock().await;
            let mut s = app.inner.state.lock().unwrap();
            s.feedback_times.clear();
            s.feedback_times
                .push_back(Instant::now() - Duration::from_secs(1));
        }
        tokio::time::sleep(Duration::from_millis(60)).await;
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming && !state.gravity_compensation.active);
        assert!(state.gravity_compensation.reference.is_none());
        bus.feedback();
        tokio::time::sleep(Duration::from_millis(5)).await;
        assert!(app.start_gravity_compensation(before_gap).await.is_err());
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn gravity_safety_limits_stop_the_drive() {
        for condition in ["speed", "temperature", "fault", "heartbeat", "disabled"] {
            let (bus, app) = attached().await;
            let config = gravity_config(&app).await;
            app.start_gravity_compensation(config).await.unwrap();
            bus.suppress_motion_feedback.store(true, Ordering::Release);
            tokio::time::sleep(Duration::from_millis(5)).await;
            {
                let _guard = app.inner.gate.lock().await;
                let mut s = app.inner.state.lock().unwrap();
                match condition {
                    "speed" => s.view.velocity_rad_s = Some(4.0),
                    "temperature" => s.view.temperature_c = Some(85.0),
                    "fault" => s.view.fault_code = 1,
                    "heartbeat" => s.client_seen = Instant::now() - Duration::from_secs(4),
                    "disabled" => s.view.enabled = false,
                    _ => unreachable!(),
                }
            }
            tokio::time::sleep(Duration::from_millis(60)).await;
            let state = app.snapshot();
            assert!(
                !state.enabled && !state.streaming && !state.gravity_compensation.active,
                "{condition}"
            );
            assert_eq!(state.gravity_compensation.command_torque_nm, 0.0);
            assert!(state
                .gravity_compensation
                .reason
                .as_ref()
                .is_some_and(|reason| !reason.is_empty()));
            app.shutdown().await.unwrap();
        }
    }

    #[tokio::test]
    async fn gravity_uid_change_invalidates_reference_and_stops_motion() {
        let (_, app) = attached().await;
        let config = gravity_config(&app).await;
        app.start_gravity_compensation(config).await.unwrap();
        observe_frame(&app.inner, frame(0, 1, 254, [9; 8]).unwrap());
        tokio::time::sleep(Duration::from_millis(60)).await;
        let state = app.snapshot();
        assert!(state.gravity_compensation.reference.is_none());
        assert!(!state.enabled && !state.streaming && !state.gravity_compensation.active);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn gravity_and_experiment_requests_queued_before_stop_cannot_restart() {
        let (bus, app) = attached().await;
        let config = gravity_config(&app).await;
        let start_guard = app.start_gate.lock().await;
        let gravity_app = app.clone();
        let gravity_start =
            tokio::spawn(async move { gravity_app.start_gravity_compensation(config).await });
        let experiment_app = app.clone();
        let experiment_start = tokio::spawn(async move {
            experiment_app
                .start_experiment(experiment_config(ExperimentKind::TorqueRamp))
                .await
        });
        tokio::time::sleep(Duration::from_millis(5)).await;
        app.stop().await.unwrap();
        drop(start_guard);
        assert!(gravity_start
            .await
            .unwrap()
            .unwrap_err()
            .to_string()
            .contains("cancelled by Stop"));
        assert!(experiment_start
            .await
            .unwrap()
            .unwrap_err()
            .to_string()
            .contains("cancelled by Stop"));
        assert!(!bus
            .sent
            .lock()
            .unwrap()
            .iter()
            .any(|frame| frame.id().raw() >> 24 == 3));
        assert!(!app.snapshot().gravity_compensation.active);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn gravity_stop_during_watchdog_readback_cancels_before_enable_frame() {
        let (bus, app) = attached().await;
        let config = gravity_config(&app).await;
        bus.skip_read_replies.lock().unwrap().insert(0x7028, 1);
        let start_app = app.clone();
        let start = tokio::spawn(async move { start_app.start_gravity_compensation(config).await });
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if bus.sent.lock().unwrap().iter().any(|frame| {
                    frame.id().raw() >> 24 == 17 && frame.data().starts_with(&[0x28, 0x70])
                }) {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .unwrap();
        let stop_app = app.clone();
        let stop = tokio::spawn(async move { stop_app.stop().await });
        tokio::time::sleep(Duration::from_millis(5)).await;
        let mut payload = [0; 8];
        payload[..2].copy_from_slice(&0x7028u16.to_le_bytes());
        payload[4..].copy_from_slice(&MOTOR_TIMEOUT.to_le_bytes());
        bus.tx.send(frame(17, 1, 253, payload).unwrap()).unwrap();
        assert!(start
            .await
            .unwrap()
            .unwrap_err()
            .to_string()
            .contains("cancelled by Stop"));
        stop.await.unwrap().unwrap();
        assert!(!bus
            .sent
            .lock()
            .unwrap()
            .iter()
            .any(|frame| frame.id().raw() >> 24 == 3));
        assert!(!app.snapshot().enabled && !app.snapshot().streaming);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn gravity_receive_lag_invalidates_idle_reference_and_stops_active_mode() {
        for active in [false, true] {
            let (bus, app) = attached().await;
            let config = gravity_config(&app).await;
            if active {
                app.start_gravity_compensation(config).await.unwrap();
            }
            // Fill the mock receive queue synchronously so the receiver sees
            // the transport's actual Lagged error on its next poll.
            for _ in 0..300 {
                bus.tx.send(frame(0, 2, 254, [2; 8]).unwrap()).unwrap();
            }
            tokio::time::sleep(Duration::from_millis(60)).await;
            let state = app.snapshot();
            assert!(state.gravity_compensation.reference.is_none());
            assert!(!state.gravity_compensation.active && !state.enabled && !state.streaming);
            assert!(state
                .gravity_compensation
                .reason
                .as_ref()
                .unwrap()
                .contains("dropped"));
            app.shutdown().await.unwrap();
        }
    }

    #[tokio::test]
    async fn dictionary_velocity_never_overwrites_fast_feedback() {
        let (_bus, app) = attached().await;
        let before = app.snapshot();
        let mut payload = [0; 8];
        payload[..2].copy_from_slice(&0x701Bu16.to_le_bytes());
        payload[4..].copy_from_slice(&7.5f32.to_le_bytes());
        observe_frame(&app.inner, frame(17, 1, 253, payload).unwrap());
        let after = app.snapshot();
        assert_eq!(after.velocity_rad_s, before.velocity_rad_s);
        assert_eq!(after.feedback_sequence, before.feedback_sequence);
        assert_eq!(after.dictionary_velocity_rad_s, Some(7.5));
        assert!(after.dictionary_velocity_age_ms.unwrap() < 100);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn reporting_control_frames_cannot_change_motion_or_confirm_status() {
        let (_bus, app) = attached().await;
        let before = app.snapshot();
        observe_frame(
            &app.inner,
            frame(24, 253, 1, [1, 2, 3, 4, 5, 6, 1, 0]).unwrap(),
        );
        let after = app.snapshot();
        assert_eq!(after.feedback_sequence, before.feedback_sequence);
        assert_eq!(after.position_rad, before.position_rad);
        assert_eq!(after.velocity_rad_s, before.velocity_rad_s);
        assert_eq!(after.enabled, before.enabled);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn captured_el05_periodic_type24_has_the_same_values_as_type2() {
        let (_bus, app) = attached().await;
        let _guard = app.inner.gate.lock().await;
        // Captured from UID bb4c30211831b608, firmware 0x0A050001 on can3.
        let payload = [0xB9, 0x3D, 0x7F, 0xFF, 0x7F, 0xFF, 0x01, 0x18];
        observe_frame(&app.inner, frame(2, 1, 253, payload).unwrap());
        let direct = app.snapshot();
        observe_frame(&app.inner, frame(24, 1, 253, payload).unwrap());
        let periodic = app.snapshot();
        assert_eq!(periodic.position_rad, direct.position_rad);
        assert_eq!(periodic.velocity_rad_s, direct.velocity_rad_s);
        assert_eq!(periodic.torque_nm, direct.torque_nm);
        assert_eq!(periodic.status_code, direct.status_code);
        assert_eq!(periodic.feedback_sequence, direct.feedback_sequence + 1);
        drop(_guard);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn packed_velocity_matches_position_direction_and_is_recorded_once() {
        let (_bus, app) = attached().await;
        let guard = app.inner.gate.lock().await;
        {
            let mut s = app.inner.state.lock().unwrap();
            s.experiment
                .begin(experiment_config(ExperimentKind::Record), 1, None);
            s.experiment.status.recording_interval_s = 0.0;
        }
        for ty in [2, 24] {
            for velocity in [-2.5, 2.5] {
                let now = Instant::now();
                {
                    let mut s = app.inner.state.lock().unwrap();
                    s.view.unwrapped_position_rad = Some(1.0 + velocity * 0.08);
                    s.position_window.clear();
                    s.position_window
                        .push_back((now - Duration::from_millis(120), 1.0));
                    s.position_window
                        .push_back((now - Duration::from_millis(40), 1.0 + velocity * 0.08));
                }
                let mut dictionary = [0; 8];
                dictionary[..2].copy_from_slice(&0x701Bu16.to_le_bytes());
                dictionary[4..].copy_from_slice(&(velocity as f32).to_le_bytes());
                observe_frame(&app.inner, frame(17, 1, 253, dictionary).unwrap());
                let mut payload = [0; 8];
                payload[..2]
                    .copy_from_slice(&pack(1.0 + velocity * 0.12, -12.57, 12.57).to_be_bytes());
                payload[2..4].copy_from_slice(&pack(velocity, -50.0, 50.0).to_be_bytes());
                payload[4..6].copy_from_slice(&pack(0.0, -6.0, 6.0).to_be_bytes());
                payload[6..].copy_from_slice(&250u16.to_be_bytes());
                observe_frame(&app.inner, frame(ty, 1, 253, payload).unwrap());

                let view = app.snapshot();
                assert!((view.velocity_rad_s.unwrap() - velocity).abs() < 0.002);
                assert!((view.position_velocity_rad_s.unwrap() - velocity).abs() < 0.02);
                assert_eq!(view.dictionary_velocity_rad_s, Some(velocity));
                let data = app.experiment_data();
                assert_eq!(data.feedback_velocity_sign, 1);
                let sample = data.samples.last().unwrap();
                assert_eq!(sample.velocity_rad_s, view.velocity_rad_s.unwrap());
                assert_eq!(sample.dictionary_velocity_rad_s, Some(velocity));
                assert_eq!(sample.payload, payload);
                assert_eq!(sample.can_id >> 24, u32::from(ty));
            }
        }
        assert_eq!(app.experiment_data().samples.len(), 4);
        drop(guard);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn shared_frame_velocity_drives_experiment_samples_and_gravity_damping() {
        let (_, app) = attached().await;
        let mut config = gravity_config(&app).await;
        let downward_position = app
            .snapshot()
            .gravity_compensation
            .reference
            .unwrap()
            .position_rad;
        // Exercise both consumers of the shared decode while retaining the
        // session's mutual exclusion between experimental and gravity modes.
        let mut gravity = GravityCompensation::default();
        config.reference_id = gravity
            .capture(1, "test-uid".into(), downward_position, 0.0)
            .id;
        let damping = config.damping_nm_s_rad;
        gravity.begin(config);
        let guard = app.inner.gate.lock().await;
        {
            let mut s = app.inner.state.lock().unwrap();
            s.experiment
                .begin(experiment_config(ExperimentKind::Record), 1, None);
            s.experiment.status.recording_interval_s = 0.0;
        }
        for ty in [2, 24] {
            for wire_velocity in [-2.5, 2.5] {
                let mut payload = [0; 8];
                payload[..2].copy_from_slice(&pack(1.5, -12.57, 12.57).to_be_bytes());
                payload[2..4].copy_from_slice(&pack(wire_velocity, -50.0, 50.0).to_be_bytes());
                payload[4..6].copy_from_slice(&pack(0.0, -6.0, 6.0).to_be_bytes());
                payload[6..].copy_from_slice(&250u16.to_be_bytes());
                observe_frame(&app.inner, frame(ty, 1, 253, payload).unwrap());

                let view = app.snapshot();
                let velocity = view.velocity_rad_s.unwrap();
                assert!((velocity - wire_velocity).abs() < 0.002);
                let data = app.experiment_data();
                let sample = data.samples.last().unwrap();
                assert_eq!(data.feedback_velocity_sign, 1);
                assert_eq!(sample.velocity_rad_s, velocity);
                assert_eq!(sample.can_id >> 24, u32::from(ty));

                let target = gravity.runtime_target(view.unwrapped_position_rad.unwrap(), velocity);
                assert!(gravity.status.gravity_torque_nm.abs() < 1e-12);
                let El05Target::Mit {
                    torque_nm, kp, kd, ..
                } = target
                else {
                    panic!("MIT target required")
                };
                assert_eq!(kp, 0.0);
                assert_eq!(kd, 0.0);
                assert!((torque_nm + damping * velocity).abs() < 1e-12);
                assert!(
                    torque_nm * velocity < 0.0,
                    "damping must oppose both feedback directions"
                );
                assert!(!gravity.status.limited);
            }
        }
        assert_eq!(app.experiment_data().samples.len(), 4);
        drop(guard);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn position_derivative_handles_wrapping_without_a_velocity_spike() {
        let (_bus, app) = attached().await;
        let _guard = app.inner.gate.lock().await;
        {
            let mut s = app.inner.state.lock().unwrap();
            s.view.unwrapped_position_rad = Some(12.57);
            s.position_window.clear();
            s.position_window
                .push_back((Instant::now() - Duration::from_millis(100), 12.56));
            s.position_window
                .push_back((Instant::now() - Duration::from_millis(50), 12.57));
        }
        let mut bytes = [0; 8];
        bytes[..2].copy_from_slice(&pack(-12.56, -12.57, 12.57).to_be_bytes());
        bytes[2..4].copy_from_slice(&pack(0.2, -50.0, 50.0).to_be_bytes());
        observe_frame(&app.inner, frame(2, 1, 253, bytes).unwrap());
        let view = app.snapshot();
        assert!((view.unwrapped_position_rad.unwrap() - 12.58).abs() < 0.001);
        assert!((view.velocity_rad_s.unwrap() - 0.2).abs() < 0.002);
        assert!((view.position_velocity_rad_s.unwrap() - 0.2).abs() < 0.02);
        drop(_guard);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn experiment_owns_targets_and_retains_data_after_heartbeat_stop() {
        let (bus, app) = attached().await;
        app.start_experiment(experiment_config(ExperimentKind::VelocitySteps))
            .await
            .unwrap();
        assert!(app.snapshot().enabled);
        assert!(app
            .start_experiment(experiment_config(ExperimentKind::VelocitySteps))
            .await
            .is_err());
        assert!(app
            .send_target(
                El05Target::Mit {
                    position_rad: 0.0,
                    velocity_rad_s: 1.0,
                    torque_nm: 0.0,
                    kp: 0.0,
                    kd: 0.1
                },
                true
            )
            .await
            .is_err());
        assert!(app.read_parameter(0x7019).await.is_err());
        bus.feedback();
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(app.snapshot().experiment.samples > 0);
        let recording = app.experiment_data();
        assert!(!recording.commands.is_empty());
        assert_eq!(recording.parameters.len(), 3);
        assert!(recording
            .commands
            .iter()
            .all(|command| command.can_id >> 24 == 1 && command.completed_t_s >= command.t_s));
        assert!(recording
            .samples
            .iter()
            .all(|sample| sample.target_sequence <= recording.commands.last().unwrap().sequence));
        let cursor = recording.samples.last().unwrap().sequence;
        bus.feedback();
        tokio::time::sleep(Duration::from_millis(25)).await;
        let delta = app.experiment_data_since(Some(cursor));
        assert!(!delta.samples.is_empty());
        assert!(delta.samples.iter().all(|sample| sample.sequence > cursor));
        assert_eq!(delta.status.run_id, recording.status.run_id);
        app.inner.state.lock().unwrap().client_seen =
            Instant::now() - CLIENT_TIMEOUT - Duration::from_millis(1);
        tokio::time::sleep(Duration::from_millis(60)).await;
        let view = app.snapshot();
        assert!(!view.enabled && !view.streaming && !view.experiment.active);
        assert_eq!(
            view.experiment.reason.as_deref(),
            Some("UI heartbeat expired")
        );
        assert!(!app.experiment_data().samples.is_empty());
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn motion_does_not_end_a_timed_torque_ramp_early() {
        let (bus, app) = attached().await;
        // Starting while moving and exceeding both former limits must not
        // terminate a run. The same receive/stream service still stops on time.
        bus.values
            .lock()
            .unwrap()
            .insert(0x701B, 20.0f32.to_le_bytes());
        app.enable().await.unwrap();
        assert!(app.snapshot().velocity_rad_s.unwrap() > 19.0);
        app.start_experiment(experiment_config(ExperimentKind::TorqueRamp))
            .await
            .unwrap();
        bus.values
            .lock()
            .unwrap()
            .insert(0x7019, 9.0f32.to_le_bytes());
        bus.feedback();
        tokio::time::sleep(Duration::from_millis(50)).await;
        let state = app.snapshot();
        assert!(state.enabled && state.streaming && state.experiment.active);
        assert!(state.experiment.reason.is_none());
        tokio::time::sleep(Duration::from_millis(2100)).await;
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming && !state.experiment.active);
        assert_eq!(
            state.experiment.reason.as_deref(),
            Some("duration completed")
        );
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn a_receive_queue_gap_aborts_the_experiment_and_is_recorded() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        app.start_experiment(experiment_config(ExperimentKind::VelocitySteps))
            .await
            .unwrap();
        // Overflow the actual broadcast receiver before yielding to its task.
        for _ in 0..600 {
            bus.feedback();
        }
        tokio::time::sleep(Duration::from_millis(70)).await;
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming && !state.experiment.active);
        assert!(state.experiment.dropped_frames > 0);
        assert!(state
            .experiment
            .reason
            .unwrap()
            .contains("receive queue dropped"));
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn passive_recording_stops_the_drive_automatically_and_never_sends_motion() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        let count = bus.sent.lock().unwrap().len();
        app.start_experiment(experiment_config(ExperimentKind::Record))
            .await
            .unwrap();
        assert!(!app.snapshot().enabled);
        assert!(app.enable().await.is_err());
        assert!(app.write_parameter(0x700B, 1.0).await.is_err());
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(bus.sent.lock().unwrap()[count..]
            .iter()
            .all(|f| f.id().raw() >> 24 != 1));
        app.stop().await.unwrap();
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn position_test_selects_mit_and_freezes_the_starting_coordinates() {
        let (_bus, app) = attached().await;
        app.set_mode(El05Mode::Velocity).await.unwrap();
        let mut config = experiment_config(ExperimentKind::PositionSteps);
        config.kp = 8.0;
        app.start_experiment(config).await.unwrap();
        let state = app.snapshot();
        assert_eq!(state.mode, El05Mode::Mit);
        assert!(state.enabled && state.experiment.active);
        let frozen = state.experiment.config.unwrap();
        assert!((frozen.origin_position_rad - state.unwrapped_position_rad.unwrap()).abs() < 0.01);
        app.stop().await.unwrap();
        let first_run = app.experiment_data().status.run_id;
        app.start_experiment(experiment_config(ExperimentKind::Record))
            .await
            .unwrap();
        assert_ne!(app.experiment_data().status.run_id, first_run);
        app.shutdown().await.unwrap();
    }

    async fn wait_for_parameter_read(app: &El05Session, index: u16) {
        tokio::time::timeout(READ_TIMEOUT * 5, async {
            loop {
                let pending = app
                    .inner
                    .state
                    .lock()
                    .unwrap()
                    .pending
                    .as_ref()
                    .is_some_and(|(_, current, _)| *current == index);
                if pending {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .expect("expected parameter read to become pending");
    }

    #[tokio::test]
    async fn optional_read_timeouts_finish_before_the_drive_is_enabled() {
        let (bus, app) = attached().await;
        *bus.skip_read_replies.lock().unwrap() =
            BTreeMap::from([(0x700B, 1), (0x7026, 1), (0x7028, 1)]);
        // Periodic telemetry continues independently of requested readbacks on
        // the drive; simulate that during the three optional timeout windows.
        let feedback_bus = bus.clone();
        let feedback = tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(100));
            loop {
                interval.tick().await;
                feedback_bus.feedback();
            }
        });
        let count = bus.sent.lock().unwrap().len();
        let start = Instant::now();
        let starter = app.clone();
        let task = tokio::spawn(async move {
            starter
                .start_experiment(experiment_config(ExperimentKind::PositionSteps))
                .await
        });
        wait_for_parameter_read(&app, 0x7028).await;
        assert!(start.elapsed() >= READ_TIMEOUT * 2);
        assert!(!bus.enabled.load(Ordering::Acquire));
        assert!(bus.sent.lock().unwrap()[count..]
            .iter()
            .all(|request| request.id().raw() >> 24 != 3));
        task.await.unwrap().unwrap();
        assert!(start.elapsed() >= READ_TIMEOUT * 3);
        assert!(app.snapshot().enabled && app.snapshot().experiment.active);
        // The verification read during enable still succeeds after the first
        // optional 0x7028 reply was intentionally omitted.
        assert!(app.experiment_data().parameters.is_empty());
        feedback.abort();
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn failed_preparation_stops_a_previous_manual_target_and_cleans_failed_enable() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        app.send_target(
            El05Target::Mit {
                position_rad: 0.0,
                velocity_rad_s: 0.0,
                torque_nm: 0.4,
                kp: 0.0,
                kd: 0.0,
            },
            true,
        )
        .await
        .unwrap();
        let count = bus.sent.lock().unwrap().len();
        let mut config = experiment_config(ExperimentKind::PositionSteps);
        config.max_temperature_c = 20.0;
        assert!(app.start_experiment(config).await.is_err());
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming && !state.experiment.active);
        assert!(bus.sent.lock().unwrap()[count..]
            .iter()
            .all(|request| request.id().raw() >> 24 != 3));

        bus.suppress_enable_ack.store(true, Ordering::Release);
        assert!(app
            .start_experiment(experiment_config(ExperimentKind::PositionSteps))
            .await
            .is_err());
        assert!(!bus.enabled.load(Ordering::Acquire));
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming && !state.experiment.active);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn stop_during_a_preparation_read_cancels_start_without_later_reenabling() {
        let (bus, app) = attached().await;
        bus.skip_read_replies.lock().unwrap().insert(0x700B, 1);
        let count = bus.sent.lock().unwrap().len();
        let starter = app.clone();
        let task = tokio::spawn(async move {
            starter
                .start_experiment(experiment_config(ExperimentKind::PositionSteps))
                .await
        });
        wait_for_parameter_read(&app, 0x700B).await;
        app.stop().await.unwrap();
        let error = task.await.unwrap().unwrap_err().to_string();
        assert!(error.contains("cancelled by Stop"));
        assert!(bus.sent.lock().unwrap()[count..]
            .iter()
            .all(|request| request.id().raw() >> 24 != 3));
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming && !state.experiment.active);
        app.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn concurrent_start_requests_do_not_stop_the_successful_run() {
        let (bus, app) = attached().await;
        bus.skip_read_replies.lock().unwrap().insert(0x700B, 1);
        let first = app.clone();
        let first = tokio::spawn(async move {
            first
                .start_experiment(experiment_config(ExperimentKind::PositionSteps))
                .await
        });
        wait_for_parameter_read(&app, 0x700B).await;
        let second = app.clone();
        let second = tokio::spawn(async move {
            second
                .start_experiment(experiment_config(ExperimentKind::PositionSteps))
                .await
        });
        first.await.unwrap().unwrap();
        assert!(second
            .await
            .unwrap()
            .unwrap_err()
            .to_string()
            .contains("already active"));
        assert!(app.snapshot().enabled && app.snapshot().experiment.active);
        app.shutdown().await.unwrap();
    }

    #[test]
    fn private_protocol_uses_extended_ids_and_distinct_endianness() {
        // Actual firmware initially reads 0; allow verified restoration.
        assert_eq!(encode_parameter(0x7026, 0.0).unwrap(), [0; 4]);
        let target = El05Target::Mit {
            position_rad: 12.57,
            velocity_rad_s: -50.0,
            torque_nm: 6.0,
            kp: 500.0,
            kd: 5.0,
        };
        let frames = target_frames(1, 253, target, true).unwrap();
        assert_eq!(frames[0].id(), CanId::Extended(0x01FFFF01));
        assert_eq!(frames[0].data(), &[255, 255, 0, 0, 255, 255, 255, 255]);
        let write = write_frame(0x7F, 0xFD, 0x701E, 30.0).unwrap();
        assert_eq!(write.id(), CanId::Extended(0x1200FD7F));
        assert_eq!(write.data(), &[0x1E, 0x70, 0, 0, 0, 0, 0xF0, 0x41]);
        assert_eq!(
            read_frame(0x7F, 0xFD, 0x701E).unwrap().id(),
            CanId::Extended(0x1100FD7F)
        );
    }
    #[test]
    fn target_validation_rejects_nonfinite_out_of_range_and_invalid_modes() {
        for value in [f64::NAN, f64::INFINITY, -11.001, 11.001] {
            assert!(validate_target(El05Target::Current { current_a: value }).is_err());
        }
        assert!(encode_parameter(0x7005, 4.0).is_err());
        assert!(encode_parameter(0x7028, 1.5).is_err());
        assert!(encode_parameter(0x7019, 0.0).is_err());
        assert!(validate_target(El05Target::Mit {
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_nm: 0.0,
            kp: 0.0,
            kd: 5.1
        })
        .is_err());
    }
    #[test]
    fn all_five_modes_use_correct_dictionary_sequences() {
        let targets = [
            El05Target::Current { current_a: 0.1 },
            El05Target::Velocity {
                velocity_rad_s: 1.0,
                acceleration_rad_s2: 2.0,
                current_limit_a: 0.5,
            },
            El05Target::PositionCsp {
                position_rad: 20.0,
                velocity_rad_s: 1.0,
                current_limit_a: 0.5,
            },
            El05Target::PositionPp {
                position_rad: 20.0,
                velocity_rad_s: 1.0,
                acceleration_rad_s2: 2.0,
                deceleration_rad_s2: None,
                current_limit_a: 0.5,
            },
        ];
        let expected: [&[u16]; 4] = [
            &[0x7006],
            &[0x7018, 0x7022, 0x700A],
            &[0x7018, 0x7017, 0x7016],
            &[0x7018, 0x7024, 0x7025, 0x7016],
        ];
        for (target, expected) in targets.into_iter().zip(expected) {
            let frames = target_frames(1, 253, target, true).unwrap();
            let indices: Vec<_> = frames
                .iter()
                .map(|f| u16::from_le_bytes([f.data()[0], f.data()[1]]))
                .collect();
            assert_eq!(indices, expected);
        }
        assert_eq!(
            [
                El05Mode::Mit.raw(),
                El05Mode::PositionPp.raw(),
                El05Mode::Velocity.raw(),
                El05Mode::Current.raw(),
                El05Mode::PositionCsp.raw()
            ],
            [0, 1, 2, 3, 5]
        );
    }
    #[tokio::test]
    async fn identity_is_queried_and_version_read_stops_the_stream() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        let identity = app.read_identity().await.unwrap();
        assert_eq!(identity.motor_id, 1);
        assert_eq!(identity.uid, "0102030405060708");
        assert!(
            app.snapshot().enabled,
            "identity reads must not stop a running motor"
        );
        let version = app.read_version().await.unwrap();
        assert_eq!(version.value, 0x0A050001);
        assert_eq!(version.bytes, [10, 5, 0, 1]);
        assert_eq!(version.raw, [0, 0xC4, 0x56, 10, 5, 0, 1, 7]);
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming);
        assert_eq!(state.version, Some(version));
        let sent = bus.sent.lock().unwrap();
        let query = sent
            .iter()
            .position(|f| f.data() == [0, 0xC4, 0, 0, 0, 0, 0, 0])
            .unwrap();
        assert_eq!(sent[query].id(), CanId::Extended(0x0400FD01));
        assert!(sent[..query]
            .iter()
            .any(|f| f.id().raw() == 0x0400FD01 && f.data() == [0; 8]));
        drop(sent);
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn version_timeout_does_not_reuse_a_cached_version_or_motion_frame() {
        let (bus, app) = attached().await;
        app.read_version().await.unwrap();
        bus.suppress_version_reply.store(true, Ordering::Release);
        assert!(app
            .read_version()
            .await
            .unwrap_err()
            .to_string()
            .contains("timed out"));
        assert!(app.snapshot().version.is_none());
        assert!(!app.snapshot().enabled);
        assert!(app.inner.state.lock().unwrap().pending_version.is_none());
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn id_change_verifies_uid_and_moves_the_stopped_selection() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        app.read_version().await.unwrap();
        let identity = app.change_id(2).await.unwrap();
        assert_eq!(identity.motor_id, 2);
        assert_eq!(identity.uid, "0102030405060708");
        let state = app.snapshot();
        assert_eq!(state.motor_id, Some(2));
        assert_eq!(state.identity, Some(identity));
        assert!(state.version.is_none());
        assert!(!state.enabled && !state.streaming);
        assert!(!state.devices.iter().any(|d| d.motor_id == 1));
        assert!(bus
            .sent
            .lock()
            .unwrap()
            .iter()
            .any(|f| f.id().raw() == 0x0702FD01 && f.data() == [0; 8]));
        app.enable().await.unwrap();
        assert!(app.snapshot().enabled);
        assert_eq!(app.change_id(1).await.unwrap().motor_id, 1);
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn id_change_recovers_a_lost_ack_by_querying_the_new_address() {
        let (bus, app) = attached().await;
        bus.suppress_id_ack.store(true, Ordering::Release);
        assert_eq!(app.change_id(0).await.unwrap().motor_id, 0);
        assert!(bus
            .sent
            .lock()
            .unwrap()
            .iter()
            .any(|f| f.id().raw() == 0x0000FD00));
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn occupied_or_invalid_ids_never_send_an_id_write() {
        let (bus, app) = attached().await;
        for id in [1, 128, 253, 255] {
            assert!(app.change_id(id).await.is_err());
        }
        bus.occupied_id.store(2, Ordering::Release);
        assert!(app
            .change_id(2)
            .await
            .unwrap_err()
            .to_string()
            .contains("occupied"));
        assert!(bus
            .sent
            .lock()
            .unwrap()
            .iter()
            .all(|f| f.id().raw() >> 24 != 7));
        assert_eq!(app.snapshot().motor_id, Some(1));
        assert!(validate_motor_id(5, 5).is_err());
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn rejected_id_change_keeps_the_verified_old_address_and_requires_reselection() {
        let (bus, app) = attached().await;
        bus.reject_id_change.store(true, Ordering::Release);
        assert!(app
            .change_id(2)
            .await
            .unwrap_err()
            .to_string()
            .contains("still responds at ID 1"));
        assert_eq!(app.snapshot().motor_id, Some(1));
        assert!(!app.snapshot().streaming);
        assert!(app.enable().await.is_err());
        app.attach(1).await.unwrap();
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn unverified_id_change_clears_selection_instead_of_controlling_another_uid() {
        for wrong_uid in [false, true] {
            let (bus, app) = attached().await;
            bus.hide_after_id_change
                .store(!wrong_uid, Ordering::Release);
            bus.wrong_uid_after_id_change
                .store(wrong_uid, Ordering::Release);
            assert!(app
                .change_id(2)
                .await
                .unwrap_err()
                .to_string()
                .contains("selection cleared"));
            let state = app.snapshot();
            assert_eq!(state.motor_id, None);
            assert!(state.identity.is_none() && state.version.is_none());
            assert!(!state.attached && !state.enabled && !state.streaming);
            assert!(app.enable().await.is_err());
            assert!(!bus.enabled.load(Ordering::Acquire));
            app.shutdown().await.unwrap();
        }
    }
    #[tokio::test]
    async fn id_change_retains_new_address_when_followup_synchronization_fails() {
        let (bus, app) = attached().await;
        bus.suppress_stop_after_id_change
            .store(true, Ordering::Release);
        assert!(app
            .change_id(2)
            .await
            .unwrap_err()
            .to_string()
            .contains("ID changed to 2"));
        assert_eq!(app.snapshot().motor_id, Some(2));
        assert_eq!(app.read_identity().await.unwrap().uid, "0102030405060708");
        assert!(app.enable().await.is_err());
        bus.suppress_stop_after_id_change
            .store(false, Ordering::Release);
        app.attach(2).await.unwrap();
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn zero_timeout_blocks_enable_until_coordinates_are_refreshed() {
        let (bus, app) = attached().await;
        bus.suppress_zero_ack.store(true, Ordering::Release);
        assert!(app
            .set_zero()
            .await
            .unwrap_err()
            .to_string()
            .contains("not confirmed"));
        assert!(!app.snapshot().enabled);
        assert!(app.enable().await.is_err());
        assert!(app.set_zero().await.is_err());
        assert!(app.snapshot().last_error.unwrap().contains("coordinates"));
        app.attach(1).await.unwrap();
        assert!(app.snapshot().position_rad.unwrap().abs() < 0.001);
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn zero_is_stopped_and_read_back_only_in_documented_modes() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        app.set_zero().await.unwrap();
        let state = app.snapshot();
        assert!(!state.enabled && !state.streaming);
        assert_eq!(state.mechanical_position_rad, Some(0.0));
        assert!(state.position_rad.unwrap().abs() < 0.001);
        assert!(bus
            .sent
            .lock()
            .unwrap()
            .iter()
            .any(|f| f.id().raw() == 0x0600FD01 && f.data() == [1, 0, 0, 0, 0, 0, 0, 0]));
        app.set_mode(El05Mode::PositionCsp).await.unwrap();
        app.set_zero().await.unwrap();
        for mode in [El05Mode::PositionPp, El05Mode::Velocity, El05Mode::Current] {
            app.set_mode(mode).await.unwrap();
            assert!(app.set_zero().await.is_err());
        }
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn metadata_replies_match_source_destination_and_do_not_confirm_motion_status() {
        let (_bus, app) = attached().await;
        app.inner.state.lock().unwrap().scan_next = None;
        let _guard = app.inner.gate.lock().await;
        let version = [0, 0xC4, 0x56, 10, 5, 0, 1, 7];
        let (tx, mut rx) = oneshot::channel();
        app.inner.state.lock().unwrap().pending_version = Some((1, tx));
        observe_frame(&app.inner, frame(2, 2, 253, version).unwrap());
        observe_frame(&app.inner, frame(2, 1, 252, version).unwrap());
        assert!(matches!(
            rx.try_recv(),
            Err(oneshot::error::TryRecvError::Empty)
        ));
        assert!(app.snapshot().version.is_none());
        let sequence = app.inner.state.lock().unwrap().feedback_sequence;
        observe_frame(&app.inner, frame(2, (2 << 14) | 1, 253, version).unwrap());
        assert_eq!(rx.try_recv().unwrap().value, 0x0A050001);
        assert_eq!(app.inner.state.lock().unwrap().feedback_sequence, sequence);
        assert!(!app.snapshot().enabled);
        let (tx, mut rx) = oneshot::channel();
        app.inner.state.lock().unwrap().pending_identity = Some((2, tx));
        observe_frame(&app.inner, frame(0, 1, 254, [3; 8]).unwrap());
        observe_frame(&app.inner, frame(0, 2, 253, [3; 8]).unwrap());
        observe_frame(&app.inner, frame(0, 2, 254, [0; 8]).unwrap());
        assert!(matches!(
            rx.try_recv(),
            Err(oneshot::error::TryRecvError::Empty)
        ));
        observe_frame(&app.inner, frame(0, 2, 254, [4; 8]).unwrap());
        assert_eq!(rx.try_recv().unwrap().motor_id, 2);
        drop(_guard);
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn uid_version_and_fault_frames_do_not_corrupt_motion_feedback() {
        let (_bus, app) = attached().await;
        let before = app.snapshot().position_rad;
        observe_frame(
            &app.inner,
            frame(0, 1, 254, [0xBB, 0x4C, 0x30, 0x21, 0x18, 0x31, 0xB6, 0x08]).unwrap(),
        );
        assert_eq!(
            app.devices()
                .iter()
                .find(|d| d.motor_id == 1)
                .unwrap()
                .uid
                .as_deref(),
            Some("bb4c30211831b608")
        );
        observe_frame(
            &app.inner,
            frame(2, 1, 253, [0, 0xC4, 0x56, 10, 5, 0, 1, 7]).unwrap(),
        );
        assert_eq!(app.snapshot().position_rad, before);
        observe_frame(
            &app.inner,
            frame(21, 1, 253, [0, 0, 1, 0, 1, 0, 0, 0]).unwrap(),
        );
        let state = app.snapshot();
        assert_eq!(state.fault_code, 1 << 16);
        assert_eq!(state.warning_code, 1);
        assert!(state.faults.iter().any(|s| s == "A相过流"));
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn mode_switch_stops_and_never_restarts_the_old_stream() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        app.set_mode(El05Mode::Velocity).await.unwrap();
        let count = bus.sent.lock().unwrap().len();
        tokio::time::sleep(Duration::from_millis(60)).await;
        assert!(!app.snapshot().enabled);
        assert!(!app.snapshot().streaming);
        assert!(bus.sent.lock().unwrap()[count..]
            .iter()
            .all(|f| f.id().raw() >> 24 != 1));
        let writes = bus.sent.lock().unwrap();
        let change = writes
            .iter()
            .position(|f| f.id().raw() >> 24 == 18 && f.data()[..2] == [5, 0x70])
            .unwrap();
        assert!(writes[..change]
            .iter()
            .rev()
            .any(|f| f.id().raw() >> 24 == 4));
        drop(writes);
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn unconfirmed_stop_suspends_all_traffic_and_is_retryable() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        bus.suppress_stop_ack.store(true, Ordering::Release);
        assert!(app.stop().await.is_err());
        assert!(
            app.snapshot().enabled,
            "a transmitted stop must not masquerade as a confirmed stop"
        );
        assert!(!app.snapshot().streaming);
        let sent = bus.sent.lock().unwrap().len();
        tokio::time::sleep(Duration::from_millis(120)).await;
        assert_eq!(
            bus.sent.lock().unwrap().len(),
            sent,
            "polling must not keep an unstopped drive's watchdog alive"
        );
        assert!(app.enable().await.is_err());
        assert!(app.write_parameter(0x7006, 0.1).await.is_err());
        bus.suppress_stop_ack.store(false, Ordering::Release);
        app.stop().await.unwrap();
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn expired_client_lease_stops_and_pp_is_never_streamed() {
        let (_bus, app) = attached().await;
        app.set_mode(El05Mode::PositionPp).await.unwrap();
        app.enable().await.unwrap();
        app.send_target(
            El05Target::PositionPp {
                position_rad: 1.6,
                velocity_rad_s: 0.5,
                acceleration_rad_s2: 1.0,
                deceleration_rad_s2: None,
                current_limit_a: 0.3,
            },
            true,
        )
        .await
        .unwrap();
        assert!(!app.snapshot().streaming);
        app.inner.state.lock().unwrap().client_seen =
            Instant::now() - CLIENT_TIMEOUT - Duration::from_millis(1);
        tokio::time::sleep(Duration::from_millis(80)).await;
        let state = app.snapshot();
        assert!(!state.enabled);
        assert!(state.last_error.unwrap().contains("heartbeat"));
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn generic_writes_need_readback_and_cannot_bypass_active_target_checks() {
        let (bus, app) = attached().await;
        app.enable().await.unwrap();
        assert!(app.write_parameter(0x7006, 0.1).await.is_err());
        app.stop().await.unwrap();
        bus.reject_writes.store(true, Ordering::Release);
        assert!(app.write_parameter(0x7018, 0.5).await.is_err());
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn stop_and_mode_recovery_wait_for_ack_despite_old_sync_error() {
        let (_bus, app) = attached().await;
        app.enable().await.unwrap();
        app.inner.state.lock().unwrap().receive_error = Some("external mode change".into());
        app.stop().await.unwrap();
        app.set_mode(El05Mode::Current).await.unwrap();
        assert!(app.inner.state.lock().unwrap().receive_error.is_none());
        app.enable().await.unwrap();
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn parameter_responses_cannot_hide_missing_drive_status() {
        let (bus, app) = attached().await;
        // The final neutral MIT command in enable() normally queues another
        // status frame. Suppress it before enabling so it cannot race with
        // aging the status window below; command acknowledgements still work.
        bus.suppress_motion_feedback.store(true, Ordering::Release);
        app.enable().await.unwrap();
        {
            let mut s = app.inner.state.lock().unwrap();
            s.target = None;
            s.last_seen = Some(Instant::now());
            s.feedback_times.clear();
            s.feedback_times
                .push_back(Instant::now() - FEEDBACK_TIMEOUT - Duration::from_millis(1));
        }
        tokio::time::sleep(Duration::from_millis(80)).await;
        assert!(!app.snapshot().enabled);
        assert!(app
            .snapshot()
            .last_error
            .unwrap()
            .contains("feedback timed out"));
        app.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn mechanical_zero_requires_new_ack_and_refreshes_absolute_position() {
        let (bus, app) = attached().await;
        app.set_zero().await.unwrap();
        assert_eq!(app.snapshot().mechanical_position_rad, Some(0.0));
        let sent = bus.sent.lock().unwrap();
        let zero = sent.iter().position(|f| f.id().raw() >> 24 == 6).unwrap();
        assert!(sent[..zero].iter().any(|f| f.id().raw() >> 24 == 4));
        assert!(sent[zero + 1..]
            .iter()
            .any(|f| f.id().raw() >> 24 == 17 && f.data()[..2] == [0x19, 0x70]));
        drop(sent);
        app.set_mode(El05Mode::PositionPp).await.unwrap();
        assert!(app.set_zero().await.is_err());
        app.shutdown().await.unwrap();
    }
}
