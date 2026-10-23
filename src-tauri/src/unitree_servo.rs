//! One serial owner, one request/response transaction at a time. Blocking USB
//! I/O runs on a dedicated thread; GUI state polling never reads the port.
use std::collections::{BTreeMap, BTreeSet};
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, ensure, Context, Result};
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;

use crate::unitree_boot::{enter_boot_packet, parse_boot_reply, BootReply, BootRequest};
use crate::unitree_protocol::{
    control_packet, hex_bytes, validate_id, FeedbackDecoder, ServoFeedback, ServoTarget, BAUD_RATE,
};

const ONLINE_AGE: Duration = Duration::from_millis(750);
const CLIENT_TIMEOUT: Duration = Duration::from_millis(1500);
const IDLE_PERIOD: Duration = Duration::from_millis(100);
const SINGLE_DURATION: Duration = Duration::from_millis(200);
const BOOT_SETTLE: Duration = Duration::from_millis(100);
const BOOT_TIMEOUT: Duration = Duration::from_millis(100);
const SAVE_TIMEOUT: Duration = Duration::from_millis(3000);
const MODE_TRANSITION_TIMEOUT: Duration = Duration::from_millis(250);

#[path = "unitree_servo_bus.rs"]
pub(crate) mod bus;
use bus::ActiveBusTest;
pub use bus::{BusTestReport, BusTestRequest};

#[derive(Debug)]
struct NoServoResponse(u8);
impl std::fmt::Display for NoServoResponse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "No response from servo ID {}; check servo power, wiring and adapter",
            self.0
        )
    }
}
impl std::error::Error for NoServoResponse {}

#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum IdChangePhase {
    Checking,
    EnteringBoot,
    Unlocking,
    Writing,
    Saving,
    Verifying,
    Restoring,
    Complete,
    Failed,
}

#[derive(Clone, Debug, Serialize)]
pub struct ServoIdChange {
    pub old_id: u8,
    pub new_id: u8,
    pub phase: IdChangePhase,
    pub write_attempted: bool,
    pub saved: bool,
    pub error: Option<String>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ServoSettings {
    pub command_hz: u16,
    pub response_timeout_ms: u16,
    pub timeout_protection: bool,
    pub position_min_rad: f64,
    pub position_max_rad: f64,
    pub velocity_limit_rad_s: f64,
    pub torque_limit_nm: f64,
}

impl Default for ServoSettings {
    fn default() -> Self {
        Self {
            command_hz: 100,
            response_timeout_ms: 20,
            timeout_protection: true,
            position_min_rad: -std::f64::consts::TAU,
            position_max_rad: std::f64::consts::TAU,
            velocity_limit_rad_s: 1.0,
            torque_limit_nm: 0.5,
        }
    }
}

impl ServoSettings {
    fn validate_target(&self, target: ServoTarget) -> Result<()> {
        target.validate()?;
        ensure!(
            target.position_rad >= self.position_min_rad
                && target.position_rad <= self.position_max_rad,
            "Target exceeds configured position limits"
        );
        ensure!(
            target.velocity_rad_s.abs() <= self.velocity_limit_rad_s,
            "Target exceeds configured velocity limit"
        );
        ensure!(
            target.torque_nm.abs() <= self.torque_limit_nm,
            "Target exceeds configured torque limit"
        );
        Ok(())
    }

    pub fn validate(self) -> Result<Self> {
        ensure!(
            (65..=500).contains(&self.command_hz),
            "Command frequency must be 65–500 Hz"
        );
        ensure!(
            (5..=100).contains(&self.response_timeout_ms),
            "Response timeout must be 5–100 ms"
        );
        ensure!(
            self.position_min_rad.is_finite()
                && self.position_max_rad.is_finite()
                && self.position_min_rad >= -1428.018898
                && self.position_max_rad <= 1428.018898
                && self.position_min_rad < self.position_max_rad,
            "Invalid position limits"
        );
        ensure!(
            self.velocity_limit_rad_s.is_finite()
                && self.velocity_limit_rad_s > 0.0
                && self.velocity_limit_rad_s <= 278.90143,
            "Velocity limit must be in (0, 278.90143] rad/s"
        );
        ensure!(
            self.torque_limit_nm.is_finite()
                && self.torque_limit_nm > 0.0
                && self.torque_limit_nm <= 36.908174,
            "Torque limit must be in (0, 36.908174] Nm"
        );
        Ok(self)
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct ServoPort {
    pub name: String,
    pub description: String,
}

pub fn list_ports() -> Result<Vec<ServoPort>> {
    let mut ports: Vec<_> = serialport::available_ports()?
        .into_iter()
        .map(|p| {
            let description = match p.port_type {
                serialport::SerialPortType::UsbPort(info) => format!(
                    "{} {:04X}:{:04X}",
                    info.product.unwrap_or_else(|| "USB serial".into()),
                    info.vid,
                    info.pid
                ),
                serialport::SerialPortType::BluetoothPort => "Bluetooth".into(),
                _ => "Serial".into(),
            };
            ServoPort {
                name: p.port_name,
                description,
            }
        })
        .collect();
    ports.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(ports)
}

#[derive(Clone, Debug, Serialize)]
pub struct ServoDevice {
    pub id: u8,
    pub online: bool,
    pub feedback: ServoFeedback,
}

#[derive(Clone, Debug, Serialize)]
pub struct ServoState {
    pub connected: bool,
    pub port: String,
    pub baud_rate: u32,
    pub selected_id: Option<u8>,
    pub online: bool,
    pub streaming: bool,
    pub motion_active: bool,
    pub stop_confirmed: bool,
    pub id_change: Option<ServoIdChange>,
    /// APP has not yet been confirmed at these addresses. No FOC while nonempty.
    pub bootloader_ids: Vec<u8>,
    pub bus_test: Option<BusTestReport>,
    pub pending_stop_ids: Vec<u8>,
    pub settings: ServoSettings,
    pub target: Option<ServoTarget>,
    pub feedback: Option<ServoFeedback>,
    pub feedback_age_ms: Option<u64>,
    pub feedback_sequence: u64,
    pub feedback_time_s: Option<f64>,
    pub devices: Vec<ServoDevice>,
    pub tx_count: u64,
    pub rx_count: u64,
    pub timeout_count: u64,
    pub crc_error_count: u64,
    pub last_tx_hex: Option<String>,
    pub last_rx_hex: Option<String>,
    pub last_error: Option<String>,
}

struct Shared {
    state: ServoState,
    feedback_at: Option<Instant>,
    client_at: Instant,
    started_at: Instant,
    devices: BTreeMap<u8, (Instant, ServoFeedback)>,
}

impl Shared {
    fn snapshot(&self) -> ServoState {
        let mut state = self.state.clone();
        state.feedback_age_ms = self.feedback_at.map(|t| t.elapsed().as_millis() as u64);
        state.online =
            state.connected && self.feedback_at.is_some_and(|t| t.elapsed() < ONLINE_AGE);
        if let Some(report) = state.bus_test.as_mut().filter(|report| report.running) {
            report.elapsed_s =
                (self.started_at.elapsed().as_secs_f64() - report.started_at_s).max(0.0);
        }
        state.devices = self
            .devices
            .iter()
            .map(|(id, (t, f))| ServoDevice {
                id: *id,
                online: state.connected && t.elapsed() < ONLINE_AGE,
                feedback: f.clone(),
            })
            .collect();
        state
    }
}

pub enum ServoOperation {
    Scan,
    Select(u8),
    Send { target: ServoTarget, repeat: bool },
    Stop,
    Configure(ServoSettings),
    ChangeId { old_id: u8, new_id: u8 },
    RestoreApp(u8),
    SetMode(u8),
    StartBusTest(BusTestRequest),
    Close { force: bool },
}

struct Job {
    operation: ServoOperation,
    epoch: u64,
    reply: oneshot::Sender<std::result::Result<ServoState, String>>,
}

trait SerialIo: Read + Write + Send {
    fn clear_input(&mut self) -> io::Result<()>;
}

struct UsbSerial(Box<dyn serialport::SerialPort>);
impl Read for UsbSerial {
    fn read(&mut self, data: &mut [u8]) -> io::Result<usize> {
        self.0.read(data)
    }
}
impl Write for UsbSerial {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.0.write(data)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}
impl SerialIo for UsbSerial {
    fn clear_input(&mut self) -> io::Result<()> {
        self.0
            .clear(serialport::ClearBuffer::Input)
            .map_err(Into::into)
    }
}

pub struct ServoSession {
    sender: mpsc::Sender<Job>,
    shared: Arc<Mutex<Shared>>,
    stop_epoch: Arc<AtomicU64>,
    closed: Arc<AtomicBool>,
}

impl ServoSession {
    pub fn open(port: String, id: u8, settings: ServoSettings) -> Result<Arc<Self>> {
        validate_id(id)?;
        settings.validate()?;
        ensure!(!port.trim().is_empty(), "Enter a serial port");
        let serial = serialport::new(port.trim(), BAUD_RATE)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(Duration::from_millis(5))
            .open().with_context(|| format!("Cannot open {port} at 6000000 bps (8N1); check permissions and adapter support"))?;
        ensure!(
            serial.baud_rate()? == BAUD_RATE,
            "Adapter did not accept 6000000 bps"
        );
        Self::start(port, id, settings, Box::new(UsbSerial(serial)))
    }

    fn start(
        port: String,
        id: u8,
        settings: ServoSettings,
        io: Box<dyn SerialIo>,
    ) -> Result<Arc<Self>> {
        let (sender, receiver) = mpsc::channel();
        let shared = Arc::new(Mutex::new(Shared {
            state: ServoState {
                connected: true,
                port,
                baud_rate: BAUD_RATE,
                selected_id: Some(id),
                online: false,
                streaming: false,
                motion_active: false,
                stop_confirmed: true,
                id_change: None,
                bootloader_ids: Vec::new(),
                bus_test: None,
                pending_stop_ids: Vec::new(),
                settings,
                target: None,
                feedback: None,
                feedback_age_ms: None,
                feedback_sequence: 0,
                feedback_time_s: None,
                devices: Vec::new(),
                tx_count: 0,
                rx_count: 0,
                timeout_count: 0,
                crc_error_count: 0,
                last_tx_hex: None,
                last_rx_hex: None,
                last_error: None,
            },
            feedback_at: None,
            client_at: Instant::now(),
            started_at: Instant::now(),
            devices: BTreeMap::new(),
        }));
        let stop_epoch = Arc::new(AtomicU64::new(0));
        let closed = Arc::new(AtomicBool::new(false));
        let session = Arc::new(Self {
            sender,
            shared: shared.clone(),
            stop_epoch: stop_epoch.clone(),
            closed: closed.clone(),
        });
        std::thread::Builder::new()
            .name("unitree-servo-serial".into())
            .spawn(move || {
                let mut worker = Worker {
                    io,
                    shared,
                    stop_epoch,
                    epoch: 0,
                    active: None,
                    single_until: None,
                    failures: 0,
                    bus_test: None,
                    pending_stop: BTreeSet::new(),
                };
                worker.run(receiver);
                let mut shared = worker.shared.lock().unwrap();
                shared.state.connected = false;
                shared.state.streaming = false;
                shared.state.motion_active = false;
                closed.store(true, Ordering::Release);
            })?;
        Ok(session)
    }

    pub fn snapshot(&self) -> ServoState {
        self.shared.lock().unwrap().snapshot()
    }

    pub fn touch_client(&self) {
        self.shared.lock().unwrap().client_at = Instant::now();
    }

    pub async fn execute(&self, operation: ServoOperation) -> Result<ServoState> {
        ensure!(
            !self.closed.load(Ordering::Acquire),
            "Serial session is closed"
        );
        self.touch_client();
        if matches!(
            operation,
            ServoOperation::Stop | ServoOperation::Close { .. }
        ) {
            // Interrupt even a long scan/response wait; invalidate queued motion.
            self.stop_epoch.fetch_add(1, Ordering::AcqRel);
        }
        let (reply, result) = oneshot::channel();
        self.sender
            .send(Job {
                operation,
                epoch: self.stop_epoch.load(Ordering::Acquire),
                reply,
            })
            .map_err(|_| anyhow!("Serial worker exited"))?;
        result
            .await
            .context("Serial worker exited before replying")?
            .map_err(anyhow::Error::msg)
    }

    pub async fn shutdown(&self, force: bool) -> Result<()> {
        if self.closed.load(Ordering::Acquire) {
            return Ok(());
        }
        self.execute(ServoOperation::Close { force }).await?;
        Ok(())
    }
}

impl Drop for ServoSession {
    fn drop(&mut self) {
        self.stop_epoch.fetch_add(1, Ordering::AcqRel);
        let (reply, _) = oneshot::channel();
        let _ = self.sender.send(Job {
            operation: ServoOperation::Close { force: true },
            epoch: self.stop_epoch.load(Ordering::Acquire),
            reply,
        });
    }
}

struct Worker {
    io: Box<dyn SerialIo>,
    shared: Arc<Mutex<Shared>>,
    stop_epoch: Arc<AtomicU64>,
    epoch: u64,
    active: Option<ServoTarget>,
    single_until: Option<Instant>,
    failures: u8,
    bus_test: Option<ActiveBusTest>,
    pending_stop: BTreeSet<u8>,
}

impl Worker {
    fn run(&mut self, receiver: mpsc::Receiver<Job>) {
        let mut next = Instant::now();
        loop {
            let epoch = self.stop_epoch.load(Ordering::Acquire);
            if epoch != self.epoch {
                self.epoch = epoch;
                if self.active.is_some() || self.bus_test.is_some() || !self.pending_stop.is_empty()
                {
                    let _ = self.stop();
                }
            }
            let client_stale = self.shared.lock().unwrap().client_at.elapsed() >= CLIENT_TIMEOUT;
            if self.bus_test.is_some() && client_stale {
                self.finish_bus_test(
                    "failed",
                    Some("Operator heartbeat expired; bus test stopped".into()),
                );
            } else if self.active.is_some() && client_stale {
                let _ = self.stop();
                self.set_error("Operator heartbeat expired; motion stopped".into());
            }
            match receiver.recv_timeout(
                next.saturating_duration_since(Instant::now())
                    .min(Duration::from_millis(20)),
            ) {
                Ok(job) => {
                    let force_close =
                        matches!(job.operation, ServoOperation::Close { force: true });
                    let closing = matches!(job.operation, ServoOperation::Close { .. });
                    let stale = job.epoch != self.stop_epoch.load(Ordering::Acquire);
                    let result =
                        if stale && !closing && !matches!(job.operation, ServoOperation::Stop) {
                            Err(anyhow!(
                                "Operation cancelled by Stop or servo reconfiguration"
                            ))
                        } else {
                            self.epoch = self.stop_epoch.load(Ordering::Acquire);
                            self.handle(job.operation)
                        };
                    let success = result.is_ok();
                    if let Err(error) = &result {
                        self.set_error(format!("{error:#}"));
                    }
                    let reply = result
                        .map(|_| self.shared.lock().unwrap().snapshot())
                        .map_err(|e| format!("{e:#}"));
                    let _ = job.reply.send(reply);
                    if closing && (success || force_close) {
                        break;
                    }
                    next = Instant::now() + self.period();
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    let _ = self.stop();
                    break;
                }
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    if Instant::now() >= next {
                        let tick_started = Instant::now();
                        self.tick();
                        // Never catch up by bursting commands after slow USB I/O.
                        next = (tick_started + self.period()).max(Instant::now());
                    }
                }
            }
        }
    }

    fn period(&self) -> Duration {
        if self.active.is_some() || self.bus_test.is_some() {
            Duration::from_secs_f64(
                1.0 / self.shared.lock().unwrap().state.settings.command_hz as f64,
            )
        } else {
            IDLE_PERIOD
        }
    }

    fn handle(&mut self, operation: ServoOperation) -> Result<()> {
        match operation {
            ServoOperation::Scan => {
                self.require_idle()?;
                self.shared.lock().unwrap().devices.clear();
                let mut ids = Vec::new();
                for id in 0..=14 {
                    ensure!(
                        self.stop_epoch.load(Ordering::Acquire) == self.epoch,
                        "Scan cancelled by Stop"
                    );
                    match self.exchange(id, 0, false, ServoTarget::default()) {
                        Ok(_) => ids.push(id),
                        Err(error) => {
                            if self.stop_epoch.load(Ordering::Acquire) != self.epoch {
                                return Err(error);
                            }
                        }
                    }
                }
                if ids.len() == 1 {
                    self.select(ids[0])?;
                }
                ensure!(
                    !ids.is_empty(),
                    "No servo replied at IDs 0–14; check servo power, wiring and adapter"
                );
                self.shared.lock().unwrap().state.last_error = None;
            }
            ServoOperation::Select(id) => {
                if let Some(test) = &self.bus_test {
                    ensure!(
                        test.request.nodes.iter().any(|node| node.id == id),
                        "Select an ID participating in the running bus test"
                    );
                    let mut shared = self.shared.lock().unwrap();
                    shared.state.selected_id = Some(id);
                    shared.state.feedback = shared.devices.get(&id).map(|(_, f)| f.clone());
                    shared.feedback_at = shared.devices.get(&id).map(|(at, _)| *at);
                    shared.state.feedback_time_s = shared.devices.get(&id).map(|(at, _)| {
                        at.saturating_duration_since(shared.started_at)
                            .as_secs_f64()
                    });
                    shared.state.feedback_sequence += 1;
                } else {
                    self.require_idle()?;
                    self.select(id)?;
                }
            }
            ServoOperation::Configure(settings) => {
                self.require_idle()?;
                self.shared.lock().unwrap().state.settings = settings.validate()?;
            }
            ServoOperation::ChangeId { old_id, new_id } => self.change_id(old_id, new_id)?,
            ServoOperation::RestoreApp(id) => {
                ensure!(
                    self.active.is_none()
                        && self.bus_test.is_none()
                        && self.pending_stop.is_empty(),
                    "Stop all motion before restoring APP"
                );
                let result = self.restore_app(id, true);
                self.invalidate_pending();
                result?;
                self.shared.lock().unwrap().state.last_error = None;
            }
            ServoOperation::StartBusTest(request) => self.start_bus_test(request)?,
            ServoOperation::SetMode(mode) => {
                ensure!(mode <= 1, "Select lock (0) or FOC (1)");
                ensure!(
                    self.bus_test.is_none(),
                    "Stop the bus test before changing single-servo mode"
                );
                if mode == 0 {
                    self.stop()?;
                } else {
                    let state = self.shared.lock().unwrap().snapshot();
                    let position_rad = state
                        .feedback
                        .as_ref()
                        .map(|feedback| feedback.position_rad)
                        .unwrap_or(0.0)
                        .clamp(
                            state.settings.position_min_rad,
                            state.settings.position_max_rad,
                        );
                    self.handle(ServoOperation::Send {
                        target: ServoTarget {
                            position_rad,
                            ..ServoTarget::default()
                        },
                        repeat: true,
                    })?;
                }
            }
            ServoOperation::Send { target, repeat } => {
                ensure!(
                    self.active.is_some() || self.pending_stop.is_empty(),
                    "Confirm stop for all pending servo IDs before enabling motion"
                );
                ensure!(
                    self.bus_test.is_none(),
                    "Stop the bus test before sending a single-servo target"
                );
                target.validate()?;
                let snapshot = self.shared.lock().unwrap().snapshot();
                ensure!(
                    snapshot.bootloader_ids.is_empty(),
                    "Restore APP before enabling motion"
                );
                ensure!(
                    target.position_rad >= snapshot.settings.position_min_rad
                        && target.position_rad <= snapshot.settings.position_max_rad,
                    "Target exceeds configured position limits"
                );
                ensure!(
                    target.velocity_rad_s.abs() <= snapshot.settings.velocity_limit_rad_s,
                    "Target exceeds configured velocity limit"
                );
                ensure!(
                    target.torque_nm.abs() <= snapshot.settings.torque_limit_nm,
                    "Target exceeds configured torque limit"
                );
                ensure!(
                    snapshot.online,
                    "Servo has not replied recently; motion is unavailable"
                );
                let feedback = snapshot
                    .feedback
                    .as_ref()
                    .context("Read servo feedback first")?;
                ensure!(
                    feedback.error_bits == 0,
                    "Servo reports a fault; stop and inspect the error code"
                );
                ensure!(
                    !feedback.timed_out,
                    "Timeout protection is latched; use Stop / clear timeout first"
                );
                let id = snapshot.selected_id.context("Select a servo first")?;
                let needs_enable = self.active.is_none() || feedback.mode != 1;
                self.pending_stop.insert(id);
                {
                    let mut shared = self.shared.lock().unwrap();
                    shared.state.stop_confirmed = false;
                    shared.state.motion_active = true;
                    shared.state.streaming = repeat;
                    shared.state.target = Some(target);
                    shared.state.last_error = None;
                    shared.state.pending_stop_ids = self.pending_stop.iter().copied().collect();
                }
                self.active = Some(target);
                self.single_until = if repeat {
                    None
                } else {
                    Some(Instant::now() + SINGLE_DURATION)
                };
                self.failures = 0;
                let result = (|| {
                    // Real J288 feedback reports the preceding command's mode.
                    // Establish FOC with zero effort before applying any target.
                    if needs_enable {
                        self.transition_mode(id, 1, snapshot.settings.timeout_protection)?;
                    }
                    self.exchange(id, 1, snapshot.settings.timeout_protection, target)
                        .and_then(Self::check_motion_feedback)
                })();
                if let Err(error) = result {
                    let stop_error = self.stop().err();
                    bail!(
                        "Motion command failed: {error:#}{}",
                        stop_error
                            .map(|e| format!("; stop unconfirmed: {e:#}"))
                            .unwrap_or_default()
                    );
                }
                if !repeat {
                    self.single_until = Some(Instant::now() + SINGLE_DURATION);
                }
            }
            ServoOperation::Stop => self.stop()?,
            ServoOperation::Close { force } => {
                let needs_stop = !self.shared.lock().unwrap().state.stop_confirmed;
                if needs_stop
                    || self.active.is_some()
                    || self.bus_test.is_some()
                    || !self.pending_stop.is_empty()
                {
                    let result = self.stop();
                    if !force {
                        result?;
                    }
                }
                self.shared.lock().unwrap().state.connected = false;
            }
        }
        Ok(())
    }

    fn require_idle(&self) -> Result<()> {
        ensure!(
            self.active.is_none() && self.bus_test.is_none() && self.pending_stop.is_empty(),
            "Stop motion before scanning, changing servo or editing settings"
        );
        ensure!(
            self.shared.lock().unwrap().state.stop_confirmed,
            "Servo stop has not been confirmed; retry Stop"
        );
        ensure!(
            self.shared.lock().unwrap().state.bootloader_ids.is_empty(),
            "Restore APP before continuing"
        );
        Ok(())
    }

    fn id_phase(&self, phase: IdChangePhase) {
        if let Some(change) = self.shared.lock().unwrap().state.id_change.as_mut() {
            change.phase = phase;
        }
    }

    fn invalidate_pending(&mut self) {
        // A target queued for the old selection must not start the newly
        // configured servo. Stop/close remain eligible regardless of epoch.
        self.epoch = self.stop_epoch.fetch_add(1, Ordering::AcqRel) + 1;
    }

    fn change_id(&mut self, old_id: u8, new_id: u8) -> Result<()> {
        validate_id(old_id)?;
        validate_id(new_id)?;
        ensure!(old_id != new_id, "New ID must differ from current ID");
        self.require_idle()?;
        let snapshot = self.shared.lock().unwrap().snapshot();
        ensure!(
            snapshot.selected_id == Some(old_id),
            "Selected servo changed; read the current ID again"
        );
        ensure!(
            snapshot.online,
            "Servo is offline; ID change is unavailable"
        );
        self.shared.lock().unwrap().state.id_change = Some(ServoIdChange {
            old_id,
            new_id,
            phase: IdChangePhase::Checking,
            write_attempted: false,
            saved: false,
            error: None,
        });
        let result = self.change_id_inner(old_id, new_id);
        if let Err(error) = result {
            let candidates = self.shared.lock().unwrap().state.bootloader_ids.clone();
            // Do not retry a write/save with an unknown outcome. Return to APP
            // and locate the current ID, retaining the exact persistence result.
            // The new address is tried first after a potentially applied write.
            let mut recovered = false;
            for id in candidates.iter().rev() {
                if self.restore_app(*id, true).is_ok() {
                    recovered = true;
                    break;
                }
            }
            let mut shared = self.shared.lock().unwrap();
            let selected = shared.state.selected_id;
            let change = shared.state.id_change.as_mut().unwrap();
            change.phase = IdChangePhase::Failed;
            let detail = format!(
                "{error:#}. {}{}",
                if change.saved {
                    "Flash save was acknowledged. "
                } else if change.write_attempted {
                    "Flash persistence is UNCONFIRMED; do not assume the ID survives power loss. "
                } else {
                    "No ID write was sent. "
                },
                if recovered {
                    format!("APP recovered at ID {}.", selected.unwrap())
                } else if !candidates.is_empty() {
                    format!(
                        "Restore APP at ID {old_id} or {new_id}; if necessary power-cycle first."
                    )
                } else {
                    String::new()
                }
            );
            change.error = Some(detail.clone());
            drop(shared);
            self.invalidate_pending();
            bail!("{detail}");
        }
        let mut shared = self.shared.lock().unwrap();
        let change = shared.state.id_change.as_mut().unwrap();
        change.phase = IdChangePhase::Complete;
        shared.state.last_error = None;
        drop(shared);
        self.invalidate_pending();
        Ok(())
    }

    fn change_id_inner(&mut self, old_id: u8, new_id: u8) -> Result<()> {
        // Only a genuine timeout means the new address is free. Serial errors
        // or a Stop cancellation must never be mistaken for an unused ID.
        match self.exchange(new_id, 0, false, ServoTarget::default()) {
            Ok(_) => bail!("ID {new_id} is already occupied; choose another ID"),
            Err(error) if error.downcast_ref::<NoServoResponse>().is_some() => {}
            Err(error) => return Err(error),
        }
        self.stop()?; // fresh lock acknowledgment from the explicitly selected ID
        ensure!(
            self.shared
                .lock()
                .unwrap()
                .state
                .feedback
                .as_ref()
                .is_some_and(|f| f.error_bits == 0),
            "Resolve servo faults before writing ID"
        );
        ensure!(
            self.stop_epoch.load(Ordering::Acquire) == self.epoch,
            "ID change cancelled by Stop"
        );
        self.id_phase(IdChangePhase::EnteringBoot);
        {
            let mut shared = self.shared.lock().unwrap();
            shared.state.bootloader_ids = vec![old_id];
            shared.state.stop_confirmed = false;
            shared.feedback_at = None;
        }
        // From Boot entry through save and APP recovery the serial transaction
        // finishes atomically, even if Stop/close queues a request. No FOC is sent.
        self.enter_boot(old_id)?;
        self.id_phase(IdChangePhase::Unlocking);
        let challenge = self.boot_exchange(BootRequest::challenge(old_id), BOOT_TIMEOUT)?;
        self.boot_exchange(BootRequest::unlock(old_id, &challenge.data)?, BOOT_TIMEOUT)
            .context("Flash unlock failed")?;
        self.id_phase(IdChangePhase::Writing);
        {
            let mut shared = self.shared.lock().unwrap();
            shared.state.bootloader_ids.push(new_id);
            shared.state.id_change.as_mut().unwrap().write_attempted = true;
        }
        let written = self
            .boot_exchange(BootRequest::write_id(old_id, new_id)?, BOOT_TIMEOUT)
            .context("ID write was not acknowledged")?;
        ensure!(
            written.data[0] == new_id,
            "ID write echo does not match requested ID {new_id}"
        );
        self.id_phase(IdChangePhase::Saving);
        let seconds = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_secs() as u32;
        // SDK save metadata: client identity (0x19 + 24-bit client tag), then
        // Unix seconds. It is metadata, not the Flash unlock challenge.
        let mut stamp = [0x19, b'h', b'e', b'x', 0, 0, 0, 0];
        stamp[4..].copy_from_slice(&seconds.to_le_bytes());
        self.boot_exchange(BootRequest::save(new_id, stamp), SAVE_TIMEOUT)
            .context("Flash save was not acknowledged")?;
        self.shared
            .lock()
            .unwrap()
            .state
            .id_change
            .as_mut()
            .unwrap()
            .saved = true;
        self.id_phase(IdChangePhase::Verifying);
        let readback = self
            .boot_exchange(BootRequest::read_id(new_id), BOOT_TIMEOUT)
            .context("Saved ID readback failed")?;
        ensure!(readback.data[0] == new_id, "Saved ID readback mismatch");
        self.id_phase(IdChangePhase::Restoring);
        self.restore_app(new_id, false)
            .context("ID saved, but APP recovery failed")?;
        self.shared.lock().unwrap().devices.remove(&old_id);
        Ok(())
    }

    fn enter_boot(&mut self, id: u8) -> Result<()> {
        self.write_configuration_packet(&enter_boot_packet(id)?)?;
        // The SDK accepts a missing transition reply: an already running Boot
        // ignores this APP packet. The following challenge is mandatory.
        std::thread::sleep(BOOT_SETTLE);
        Ok(())
    }

    fn restore_app(&mut self, id: u8, enter_boot: bool) -> Result<()> {
        validate_id(id)?;
        if enter_boot {
            {
                let mut shared = self.shared.lock().unwrap();
                if !shared.state.bootloader_ids.contains(&id) {
                    shared.state.bootloader_ids.push(id);
                }
                shared.state.stop_confirmed = false;
                shared.feedback_at = None;
            }
            self.enter_boot(id)?;
        }
        // APP may start before its Boot acknowledgment reaches USB. Only a
        // fresh, CRC-valid mode-0 response at this ID can confirm recovery.
        let _ack = self.boot_exchange(BootRequest::restore_app(id), BOOT_TIMEOUT);
        self.epoch = self.stop_epoch.load(Ordering::Acquire);
        let deadline = Instant::now() + Duration::from_millis(600);
        loop {
            self.epoch = self.stop_epoch.load(Ordering::Acquire);
            match self.exchange(id, 0, false, ServoTarget::default()) {
                Ok(feedback) if feedback.mode == 0 => {
                    let mut shared = self.shared.lock().unwrap();
                    for other in shared.state.bootloader_ids.clone() {
                        if other != id {
                            shared.devices.remove(&other);
                        }
                    }
                    shared.state.selected_id = Some(id);
                    shared.state.feedback = Some(feedback);
                    shared.feedback_at = Some(Instant::now());
                    shared.state.feedback_sequence += 1;
                    shared.state.feedback_time_s = Some(shared.started_at.elapsed().as_secs_f64());
                    shared.state.bootloader_ids.clear();
                    shared.state.stop_confirmed = true;
                    return Ok(());
                }
                _ if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(20)),
                _ => bail!("No lock-mode APP response from ID {id}"),
            }
        }
    }

    fn write_configuration_packet(&mut self, packet: &[u8]) -> Result<()> {
        self.io.clear_input().context("Cannot clear serial input")?;
        self.io
            .write_all(packet)
            .context("Configuration serial write failed")?;
        let mut shared = self.shared.lock().unwrap();
        shared.state.tx_count += 1;
        shared.state.last_tx_hex = Some(hex_bytes(packet));
        Ok(())
    }

    fn boot_exchange(&mut self, request: BootRequest, timeout: Duration) -> Result<BootReply> {
        self.write_configuration_packet(&request.packet()?)?;
        let deadline = Instant::now() + timeout;
        let mut buffered = Vec::<u8>::new();
        let mut bytes = [0; 256];
        while Instant::now() < deadline {
            match self.io.read(&mut bytes) {
                Ok(0) => bail!("Configuration serial port disconnected"),
                Ok(n) => buffered.extend_from_slice(&bytes[..n]),
                Err(error)
                    if matches!(
                        error.kind(),
                        io::ErrorKind::TimedOut
                            | io::ErrorKind::WouldBlock
                            | io::ErrorKind::Interrupted
                    ) =>
                {
                    continue
                }
                Err(error) => return Err(error).context("Configuration serial read failed"),
            }
            while buffered.len() >= 4 {
                if buffered[..2] != [0xfc, 0xee] {
                    buffered.remove(0);
                    continue;
                }
                // Ignore TX echoes and late APP feedback; Boot uses mode 7.
                let code = buffered[3] >> 4;
                let size = if buffered[2] & 0x70 != 0x70 {
                    26
                } else if code == 0 {
                    26
                } else if code == 8 {
                    34
                } else {
                    buffered.remove(0);
                    continue;
                };
                if buffered.len() < size {
                    break;
                }
                let candidate = &buffered[..size];
                if candidate[2] & 0x70 != 0x70 {
                    buffered.drain(..size);
                    continue;
                }
                match parse_boot_reply(candidate) {
                    Ok(reply) => {
                        let mut shared = self.shared.lock().unwrap();
                        shared.state.rx_count += 1;
                        shared.state.last_rx_hex = Some(hex_bytes(candidate));
                        drop(shared);
                        if request.matches(&reply) {
                            ensure!(
                                reply.acknowledged,
                                "Boot rejected command {} at address 0x{:04X} (ID {})",
                                request.command,
                                request.address,
                                request.id
                            );
                            return Ok(reply);
                        }
                        buffered.drain(..size);
                    }
                    Err(_) => {
                        self.shared.lock().unwrap().state.crc_error_count += 1;
                        buffered.remove(0);
                    }
                }
            }
        }
        self.shared.lock().unwrap().state.timeout_count += 1;
        bail!(
            "Boot response timeout: ID {}, command {}, address 0x{:04X}",
            request.id,
            request.command,
            request.address
        )
    }

    fn select(&mut self, id: u8) -> Result<()> {
        validate_id(id)?;
        {
            let mut shared = self.shared.lock().unwrap();
            shared.state.selected_id = Some(id);
            shared.state.feedback = None;
            shared.state.feedback_time_s = None;
            shared.feedback_at = None;
        }
        self.exchange(id, 0, false, ServoTarget::default())?;
        self.shared.lock().unwrap().state.last_error = None;
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        self.active = None;
        self.single_until = None;
        let test = self.bus_test.take();
        if test.is_some() {
            self.invalidate_pending();
        }
        let mut ids = self.pending_stop.clone();
        if let Some(test) = &test {
            ids.extend(test.request.nodes.iter().map(|node| node.id));
        }
        let selected = {
            let mut shared = self.shared.lock().unwrap();
            shared.state.streaming = false;
            shared.state.motion_active = false;
            shared.state.target = None;
            shared.state.selected_id
        };
        ensure!(
            self.shared.lock().unwrap().state.bootloader_ids.is_empty(),
            "Servo may be in Boot; use Restore APP before Stop"
        );
        if ids.is_empty() {
            ids.extend(selected);
        }
        self.epoch = self.stop_epoch.load(Ordering::Acquire);
        let mut errors = Vec::new();
        for id in ids {
            // Do not stop at the first missing member: every potentially
            // enabled servo must receive its own lock/clear-timeout request.
            let result = self.transition_mode(id, 0, false);
            if result.is_ok() {
                self.pending_stop.remove(&id);
            }
            if let Some(report) = self.shared.lock().unwrap().state.bus_test.as_mut() {
                if let Some(node) = report.nodes.iter_mut().find(|node| node.id == id) {
                    node.stop_confirmed = result.is_ok();
                    if let Ok(feedback) = &result {
                        node.feedback = Some(feedback.clone());
                    }
                }
            }
            if let Err(error) = result {
                errors.push(format!("ID {id}: {error:#}"));
            }
        }
        {
            let mut shared = self.shared.lock().unwrap();
            shared.state.stop_confirmed = self.pending_stop.is_empty();
            shared.state.pending_stop_ids = self.pending_stop.iter().copied().collect();
            if let Some(test) = test {
                if let Some(report) = shared.state.bus_test.as_mut() {
                    report.running = false;
                    report.elapsed_s = test.started.elapsed().as_secs_f64();
                    report.phase = if errors.is_empty() {
                        "stopped".into()
                    } else {
                        "failed".into()
                    };
                    if !errors.is_empty() {
                        report.error = Some(errors.join("; "));
                    }
                }
            }
            if errors.is_empty() {
                shared.state.last_error = None;
            }
        }
        ensure!(errors.is_empty(), "Stop unconfirmed: {}", errors.join("; "));
        Ok(())
    }

    fn transition_mode(&mut self, id: u8, mode: u8, timeout: bool) -> Result<ServoFeedback> {
        let deadline = Instant::now() + MODE_TRANSITION_TIMEOUT;
        let mut observed = None;
        // Some firmware reports the preceding request's mode in its reply.
        // Retry only this bounded transition, with all effort terms zero.
        for _ in 0..8 {
            let feedback = self.exchange(id, mode, timeout, ServoTarget::default())?;
            if mode == 1 {
                Self::check_servo_health(&feedback)?;
            }
            observed = Some(feedback.mode);
            if feedback.mode == mode && (timeout || !feedback.timed_out) {
                return Ok(feedback);
            }
            ensure!(
                feedback.mode <= 1,
                "Servo ID {id} reports unsupported mode {} while selecting {mode}",
                feedback.mode
            );
            if Instant::now() >= deadline {
                break;
            }
            std::thread::sleep(Duration::from_millis(3));
        }
        bail!("Servo ID {id} mode transition to {mode} was not confirmed; last feedback mode {observed:?}")
    }

    fn tick(&mut self) {
        if self.bus_test.is_some() {
            self.tick_bus_test();
            return;
        }
        if !self.pending_stop.is_empty() && self.active.is_none() {
            if let Err(error) = self.stop() {
                self.set_error(format!("{error:#}"));
            }
            return;
        }
        let snapshot = self.shared.lock().unwrap().snapshot();
        if !snapshot.bootloader_ids.is_empty() {
            return;
        }
        let Some(id) = snapshot.selected_id else {
            return;
        };
        if let Some(target) = self.active {
            if self
                .single_until
                .is_some_and(|until| Instant::now() >= until)
            {
                if let Err(error) = self.stop() {
                    self.set_error(format!("Single command ended; stop unconfirmed: {error:#}"));
                }
                return;
            }
            if self.single_until.is_some() {
                return;
            }
            match self
                .exchange(id, 1, snapshot.settings.timeout_protection, target)
                .and_then(Self::check_motion_feedback)
            {
                Ok(()) => self.failures = 0,
                Err(error) => {
                    self.failures = self.failures.saturating_add(1);
                    // Fault feedback stops immediately; missing replies have a
                    // short bounded retry budget, independently of GUI polling.
                    if self.failures >= 3
                        || self
                            .shared
                            .lock()
                            .unwrap()
                            .state
                            .feedback
                            .as_ref()
                            .is_some_and(|f| f.error_bits != 0 || f.timed_out || f.mode != 1)
                    {
                        let stop_error = self.stop().err();
                        self.set_error(format!(
                            "Motion stopped: {error:#}{}",
                            stop_error
                                .map(|e| format!("; stop unconfirmed: {e:#}"))
                                .unwrap_or_default()
                        ));
                    }
                }
            }
        } else {
            let mut ids: Vec<_> = self
                .shared
                .lock()
                .unwrap()
                .devices
                .keys()
                .copied()
                .collect();
            if !ids.contains(&id) {
                ids.push(id);
            }
            for id in ids {
                match self.transition_mode(id, 0, false) {
                    Ok(feedback) if feedback.mode == 0 => {
                        let mut shared = self.shared.lock().unwrap();
                        shared.state.stop_confirmed = self.pending_stop.is_empty();
                        // Preserve the reason a previous motion was interrupted.
                        if shared.state.last_error.as_ref().is_some_and(|s| {
                            s.starts_with("No response from") || s.starts_with("No servo replied")
                        }) {
                            shared.state.last_error = None;
                        }
                    }
                    Ok(_) => self.set_error("Servo has not acknowledged lock mode".into()),
                    Err(error) => self.set_error(format!("{error:#}")),
                }
            }
        }
    }

    fn check_motion_feedback(feedback: ServoFeedback) -> Result<()> {
        Self::check_servo_health(&feedback)?;
        ensure!(
            feedback.mode == 1,
            "Servo did not acknowledge FOC mode (1); received mode {}",
            feedback.mode
        );
        Ok(())
    }

    fn check_servo_health(feedback: &ServoFeedback) -> Result<()> {
        ensure!(
            feedback.error_bits == 0,
            "Servo fault 0x{:08X}: {}",
            feedback.error_bits,
            feedback.errors.join(", ")
        );
        ensure!(!feedback.timed_out, "Servo timeout protection triggered");
        Ok(())
    }

    fn set_error(&self, error: String) {
        self.shared.lock().unwrap().state.last_error = Some(error);
    }

    fn exchange(
        &mut self,
        id: u8,
        mode: u8,
        timeout: bool,
        target: ServoTarget,
    ) -> Result<ServoFeedback> {
        let packet = control_packet(id, mode, timeout, target)?;
        self.io.clear_input().context("Cannot clear serial input")?;
        ensure!(
            self.stop_epoch.load(Ordering::Acquire) == self.epoch,
            "Operation cancelled by Stop"
        );
        // Do not use tcdrain/flush: some USB drivers block there indefinitely.
        self.io.write_all(&packet).context("Serial write failed")?;
        let timeout_ms = {
            let mut shared = self.shared.lock().unwrap();
            shared.state.tx_count += 1;
            shared.state.last_tx_hex = Some(hex_bytes(&packet));
            shared.state.settings.response_timeout_ms
        };
        let deadline = Instant::now() + Duration::from_millis(timeout_ms as u64);
        let mut decoder = FeedbackDecoder::default();
        let mut bytes = [0u8; 256];
        loop {
            ensure!(
                self.stop_epoch.load(Ordering::Acquire) == self.epoch,
                "Operation cancelled by Stop"
            );
            if Instant::now() >= deadline {
                self.shared.lock().unwrap().state.timeout_count += 1;
                return Err(NoServoResponse(id).into());
            }
            match self.io.read(&mut bytes) {
                Ok(0) => bail!("Serial port disconnected (end of stream)"),
                Ok(count) => {
                    decoder.push(&bytes[..count]);
                    while let Some((feedback, raw)) = decoder.next() {
                        let mut shared = self.shared.lock().unwrap();
                        shared.state.rx_count += 1;
                        shared
                            .devices
                            .insert(feedback.id, (Instant::now(), feedback.clone()));
                        if feedback.id != id {
                            continue;
                        }
                        shared.state.last_rx_hex = Some(hex_bytes(&raw));
                        if shared.state.selected_id == Some(id) {
                            shared.state.feedback = Some(feedback.clone());
                            shared.feedback_at = Some(Instant::now());
                            shared.state.feedback_sequence += 1;
                            shared.state.feedback_time_s =
                                Some(shared.started_at.elapsed().as_secs_f64());
                        }
                        shared.state.crc_error_count += decoder.crc_errors;
                        return Ok(feedback);
                    }
                    self.shared.lock().unwrap().state.crc_error_count += decoder.crc_errors;
                    decoder.crc_errors = 0;
                }
                Err(error)
                    if matches!(
                        error.kind(),
                        io::ErrorKind::TimedOut
                            | io::ErrorKind::WouldBlock
                            | io::ErrorKind::Interrupted
                    ) => {}
                Err(error) => return Err(error).context("Serial read failed"),
            }
        }
    }
}

#[cfg(test)]
#[path = "unitree_servo_id_tests.rs"]
mod id_tests;

#[cfg(test)]
#[path = "unitree_servo_bus_tests.rs"]
mod bus_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::unitree_protocol::crc32;
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicU32, AtomicU8};

    struct SimControl {
        id: AtomicU8,
        respond: AtomicBool,
        fault: AtomicU32,
        writes: Mutex<Vec<[u8; 20]>>,
    }
    struct SimSerial {
        control: Arc<SimControl>,
        input: VecDeque<u8>,
    }
    impl Read for SimSerial {
        fn read(&mut self, data: &mut [u8]) -> io::Result<usize> {
            let count = data.len().min(7).min(self.input.len());
            if count == 0 {
                std::thread::sleep(Duration::from_millis(1));
                return Err(io::ErrorKind::TimedOut.into());
            }
            for byte in data.iter_mut().take(count) {
                *byte = self.input.pop_front().unwrap();
            }
            Ok(count)
        }
    }
    impl Write for SimSerial {
        fn write(&mut self, data: &[u8]) -> io::Result<usize> {
            let packet: [u8; 20] = data.try_into().unwrap();
            assert_eq!(
                crc32(&packet[..16]),
                u32::from_le_bytes(packet[16..].try_into().unwrap())
            );
            self.control.writes.lock().unwrap().push(packet);
            if self.control.respond.load(Ordering::Acquire)
                && packet[2] & 15 == self.control.id.load(Ordering::Acquire)
            {
                let mut feedback = [0u8; 26];
                feedback[..6].copy_from_slice(&[0xfc, 0xee, packet[2] & 0x7f, 25, 30, 24]);
                feedback[6..14].copy_from_slice(&packet[4..12]);
                feedback[14..18]
                    .copy_from_slice(&self.control.fault.load(Ordering::Acquire).to_le_bytes());
                let crc = crc32(&feedback[2..22]);
                feedback[22..].copy_from_slice(&crc.to_le_bytes());
                self.input.extend(packet); // USB TX echo must be ignored.
                self.input.extend(feedback);
            }
            Ok(data.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    impl SerialIo for SimSerial {
        fn clear_input(&mut self) -> io::Result<()> {
            self.input.clear();
            Ok(())
        }
    }
    fn simulation(respond: bool) -> (Arc<ServoSession>, Arc<SimControl>) {
        let control = Arc::new(SimControl {
            id: AtomicU8::new(3),
            respond: AtomicBool::new(respond),
            fault: AtomicU32::new(0),
            writes: Mutex::new(Vec::new()),
        });
        let app = ServoSession::start(
            "simulated".into(),
            3,
            ServoSettings {
                response_timeout_ms: 5,
                ..ServoSettings::default()
            },
            Box::new(SimSerial {
                control: control.clone(),
                input: VecDeque::new(),
            }),
        )
        .unwrap();
        (app, control)
    }
    fn target() -> ServoTarget {
        ServoTarget {
            velocity_rad_s: 0.1,
            kd: 0.1,
            ..ServoTarget::default()
        }
    }
    async fn wait_until(mut condition: impl FnMut() -> bool, budget: Duration) {
        let deadline = Instant::now() + budget;
        while !condition() {
            assert!(Instant::now() < deadline, "condition timed out");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    #[tokio::test]
    async fn scan_select_stream_and_stop_share_one_serial_owner() {
        let (app, control) = simulation(true);
        let found = app.execute(ServoOperation::Scan).await.unwrap();
        assert_eq!(
            found.devices.iter().map(|d| d.id).collect::<Vec<_>>(),
            vec![3]
        );
        assert_eq!(found.selected_id, Some(3));
        assert!(found.online);
        app.execute(ServoOperation::Send {
            target: target(),
            repeat: true,
        })
        .await
        .unwrap();
        wait_until(|| app.snapshot().rx_count >= 5, Duration::from_secs(1)).await;
        assert!(app.snapshot().streaming);
        assert!(app.execute(ServoOperation::Select(4)).await.is_err());
        let stopped = app.execute(ServoOperation::Stop).await.unwrap();
        assert!(!stopped.motion_active && !stopped.streaming && stopped.stop_confirmed);
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert_eq!(control.writes.lock().unwrap().last().unwrap()[2] >> 4, 0);
        app.shutdown(false).await.unwrap();
    }

    #[tokio::test]
    async fn no_feedback_never_enables_motion_and_idle_disconnect_succeeds() {
        let (app, control) = simulation(false);
        assert!(app.execute(ServoOperation::Select(3)).await.is_err());
        assert!(!app.snapshot().online);
        assert!(app
            .execute(ServoOperation::Send {
                target: target(),
                repeat: true
            })
            .await
            .is_err());
        assert!(control
            .writes
            .lock()
            .unwrap()
            .iter()
            .all(|p| p[2] & 0x70 == 0));
        app.shutdown(false).await.unwrap();
    }

    #[tokio::test]
    async fn servo_can_power_on_after_an_empty_scan_without_reconnecting() {
        let (app, control) = simulation(false);
        assert!(app.execute(ServoOperation::Scan).await.is_err());
        assert!(!app.snapshot().online);
        control.respond.store(true, Ordering::Release);
        wait_until(|| app.snapshot().online, Duration::from_secs(1)).await;
        assert!(app.snapshot().last_error.is_none());
        assert!(control
            .writes
            .lock()
            .unwrap()
            .iter()
            .all(|p| p[2] & 0x70 == 0));
        app.shutdown(false).await.unwrap();
    }

    #[tokio::test]
    async fn backend_enforces_limits_independently_of_frontend() {
        let (app, control) = simulation(true);
        app.execute(ServoOperation::Select(3)).await.unwrap();
        app.execute(ServoOperation::Configure(ServoSettings {
            velocity_limit_rad_s: 0.05,
            ..ServoSettings::default()
        }))
        .await
        .unwrap();
        assert!(app
            .execute(ServoOperation::Send {
                target: target(),
                repeat: true
            })
            .await
            .is_err());
        assert!(app
            .execute(ServoOperation::Configure(ServoSettings {
                position_min_rad: f64::NAN,
                ..ServoSettings::default()
            }))
            .await
            .is_err());
        assert!(app.execute(ServoOperation::Select(15)).await.is_err());
        assert!(control
            .writes
            .lock()
            .unwrap()
            .iter()
            .all(|p| p[2] & 0x70 == 0));
        app.shutdown(false).await.unwrap();
    }

    #[tokio::test]
    async fn lost_replies_stop_stream_and_retain_unconfirmed_session_for_retry() {
        let (app, control) = simulation(true);
        app.execute(ServoOperation::Select(3)).await.unwrap();
        app.execute(ServoOperation::Send {
            target: target(),
            repeat: true,
        })
        .await
        .unwrap();
        control.respond.store(false, Ordering::Release);
        wait_until(|| !app.snapshot().motion_active, Duration::from_secs(1)).await;
        assert!(!app.snapshot().stop_confirmed);
        assert!(app.shutdown(false).await.is_err());
        assert!(app.snapshot().connected);
        control.respond.store(true, Ordering::Release);
        app.execute(ServoOperation::Stop).await.unwrap();
        app.shutdown(false).await.unwrap();
    }

    #[tokio::test]
    async fn servo_fault_interrupts_stream_and_prevents_restart() {
        let (app, control) = simulation(true);
        app.execute(ServoOperation::Select(3)).await.unwrap();
        app.execute(ServoOperation::Send {
            target: target(),
            repeat: true,
        })
        .await
        .unwrap();
        control.fault.store(8, Ordering::Release);
        wait_until(
            || !app.snapshot().motion_active && app.snapshot().stop_confirmed,
            Duration::from_secs(1),
        )
        .await;
        assert_eq!(app.snapshot().feedback.unwrap().error_bits, 8);
        assert!(app
            .execute(ServoOperation::Send {
                target: target(),
                repeat: true
            })
            .await
            .is_err());
        app.shutdown(false).await.unwrap();
    }

    #[tokio::test]
    async fn client_heartbeat_loss_stops_motion_in_backend() {
        let (app, _) = simulation(true);
        app.execute(ServoOperation::Select(3)).await.unwrap();
        app.execute(ServoOperation::Send {
            target: target(),
            repeat: true,
        })
        .await
        .unwrap();
        wait_until(|| !app.snapshot().motion_active, Duration::from_secs(3)).await;
        assert!(app.snapshot().last_error.unwrap().contains("heartbeat"));
        app.shutdown(false).await.unwrap();
    }

    #[tokio::test]
    async fn single_command_has_a_bounded_duration_and_then_locks() {
        let (app, control) = simulation(true);
        app.execute(ServoOperation::Select(3)).await.unwrap();
        app.execute(ServoOperation::Send {
            target: target(),
            repeat: false,
        })
        .await
        .unwrap();
        assert!(!app.snapshot().streaming && app.snapshot().motion_active);
        wait_until(
            || !app.snapshot().motion_active && app.snapshot().stop_confirmed,
            Duration::from_secs(1),
        )
        .await;
        assert_eq!(
            control
                .writes
                .lock()
                .unwrap()
                .iter()
                .filter(|p| p[2] & 0x70 == 0x10)
                .count(),
            2 // zero-output enable, then the one requested target
        );
        app.shutdown(false).await.unwrap();
    }

    #[tokio::test]
    async fn stop_preempts_a_scan_instead_of_waiting_for_all_ids() {
        let (app, _) = simulation(false);
        app.execute(ServoOperation::Configure(ServoSettings {
            response_timeout_ms: 100,
            ..ServoSettings::default()
        }))
        .await
        .unwrap();
        let scanner = app.clone();
        let scan = tokio::spawn(async move { scanner.execute(ServoOperation::Scan).await });
        tokio::time::sleep(Duration::from_millis(10)).await;
        let started = Instant::now();
        let _ = app.execute(ServoOperation::Stop).await;
        assert!(scan.await.unwrap().is_err());
        assert!(started.elapsed() < Duration::from_millis(350));
        app.shutdown(true).await.unwrap();
    }
}
