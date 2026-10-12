//! Shanghai Lingkong single-motor Classic CAN protocol V2.36.
//! One independent bus session avoids CANopen discovery traffic on this bus.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanFilter, CanFrame, CanIoError, FrameKind};
use serde::Serialize;
use tokio::sync::Mutex;

use crate::backend;
use crate::can_lease::CanLease;
use crate::host_can;
use crate::lingkong_params::{self, ParameterSpec, ParameterValue, PARAMETERS};

const SCAN_TIMEOUT: Duration = Duration::from_millis(90);
const REPLY_TIMEOUT: Duration = Duration::from_millis(300);
const READ_ATTEMPTS: usize = 3;
const TURNAROUND_DELAY: Duration = Duration::from_millis(2);

// V2.36 documents closed/running as 0x10/0x00. MHF5010 with DHF50 was
// observed to report 0x20/0x30 after Close/Run with no fault bits. Accept
// these explicit pairs; the protocol does not define other state values.
fn running_state_for_closed(state: u8) -> Option<u8> {
    match state {
        0x10 => Some(0x00),
        0x20 => Some(0x30),
        _ => None,
    }
}

fn is_closed_state(state: u8) -> bool {
    running_state_for_closed(state).is_some()
}

fn is_running_state(state: u8) -> bool {
    matches!(state, 0x00 | 0x30)
}

#[derive(Debug, Clone, Serialize)]
pub struct LingkongProbe {
    pub id: u8,
    pub motor_state: u8,
    pub error_state: u8,
}

#[derive(Debug, Clone, Serialize)]
pub struct LingkongSnapshot {
    pub id: u8,
    pub temperature_c: i8,
    pub voltage_v: f64,
    pub bus_current_a: f64,
    pub motor_state: u8,
    pub error_state: u8,
    pub iq_raw: i16,
    pub speed_dps: i16,
    pub encoder: u16,
    pub angle_deg: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct LingkongInitialization {
    pub snapshot: LingkongSnapshot,
    pub parameters: Vec<ParameterValue>,
    pub unavailable_parameters: Vec<String>,
    pub settings_dirty: bool,
    pub reboot_required: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct LingkongParameterReadResult {
    pub parameters: Vec<ParameterValue>,
    pub unavailable_parameters: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct LingkongRebootResult {
    pub next_id: Option<u8>,
    pub next_bitrate: u32,
    pub closed_after_reboot: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum Action {
    Run,
    Stop,
    Close,
    ClearFault,
    Torque { iq_raw: i16 },
    Velocity { speed_dps: f64, iq_limit: u16 },
    Position { angle_deg: f64, max_speed_dps: u16 },
}

impl Action {
    pub fn parse(kind: &str, value: f64, limit: u16) -> Result<Self, String> {
        if !value.is_finite() {
            return Err("target must be finite".into());
        }
        match kind {
            "run" => Ok(Self::Run),
            "stop" => Ok(Self::Stop),
            "close" => Ok(Self::Close),
            "clear_fault" => Ok(Self::ClearFault),
            "torque" if value.fract() == 0.0 && (-2048.0..=2048.0).contains(&value) => {
                Ok(Self::Torque {
                    iq_raw: value as i16,
                })
            }
            "velocity" if (-6000.0..=6000.0).contains(&value) && (1..=2048).contains(&limit) => {
                Ok(Self::Velocity {
                    speed_dps: value,
                    iq_limit: limit,
                })
            }
            "position" if (-21_474_836.0..=21_474_836.0).contains(&value) && limit > 0 => {
                Ok(Self::Position {
                    angle_deg: value,
                    max_speed_dps: limit,
                })
            }
            _ => Err("unknown action or target outside the V2.36 protocol range".into()),
        }
    }

    fn payload(self) -> [u8; 8] {
        let mut data = [0u8; 8];
        match self {
            Self::Run => data[0] = 0x88,
            Self::Stop => data[0] = 0x81,
            Self::Close => data[0] = 0x80,
            Self::ClearFault => data[0] = 0x9B,
            Self::Torque { iq_raw } => {
                data[0] = 0xA1;
                data[4..6].copy_from_slice(&iq_raw.to_le_bytes());
            }
            Self::Velocity {
                speed_dps,
                iq_limit,
            } => {
                data[0] = 0xA2;
                data[2..4].copy_from_slice(&iq_limit.to_le_bytes());
                data[4..8].copy_from_slice(&((speed_dps * 100.0).round() as i32).to_le_bytes());
            }
            Self::Position {
                angle_deg,
                max_speed_dps,
            } => {
                data[0] = 0xA4;
                data[2..4].copy_from_slice(&max_speed_dps.to_le_bytes());
                data[4..8].copy_from_slice(&((angle_deg * 100.0).round() as i32).to_le_bytes());
            }
        }
        data
    }
}

pub struct LingkongSession {
    bus: Arc<dyn CanBus>,
    bitrate: u32,
    operation: Mutex<()>,
    armed: Mutex<BTreeSet<u8>>,
    // Expected running state for each initialized drive's confirmed encoding.
    initialized: Mutex<BTreeMap<u8, u8>>,
    dirty_settings: Mutex<BTreeSet<u8>>,
    reboot_required: Mutex<BTreeSet<u8>>,
    active: AtomicBool,
}

impl LingkongSession {
    pub async fn start(spec: &str, bitrate: u32, lease: CanLease) -> Result<Arc<Self>, String> {
        if ![100_000, 125_000, 250_000, 500_000, 1_000_000].contains(&bitrate) {
            return Err("unsupported Lingkong CAN bitrate".into());
        }
        if spec.to_ascii_lowercase().starts_with("gs_usb")
            || spec.to_ascii_lowercase().starts_with("gsusb")
        {
            if bitrate != 1_000_000 {
                return Err("gs_usb currently supports Classic CAN 1 Mbps only".into());
            }
        } else {
            let iface = spec.strip_prefix("socketcan:").unwrap_or(spec).to_owned();
            let host = tauri::async_runtime::spawn_blocking(move || host_can::inspect(&iface))
                .await
                .map_err(|error| error.to_string())??;
            if !host.up || host.fd_enabled || host.bitrate != Some(bitrate) {
                return Err(format!(
                    "{} must be up in Classic CAN mode at {} bit/s before connecting",
                    host.iface, bitrate
                ));
            }
        }
        let bus = backend::open_classic_1m_bus(spec, lease)
            .await
            .map_err(|error| error.to_string())?;
        Ok(Arc::new(Self {
            bus: backend::protect_feature_bus(bus),
            bitrate,
            operation: Mutex::new(()),
            armed: Mutex::new(BTreeSet::new()),
            initialized: Mutex::new(BTreeMap::new()),
            dirty_settings: Mutex::new(BTreeSet::new()),
            reboot_required: Mutex::new(BTreeSet::new()),
            active: AtomicBool::new(true),
        }))
    }

    pub async fn scan(&self) -> Result<Vec<LingkongProbe>, String> {
        let _operation = self.operation.lock().await;
        self.check_active()?;
        let mut found = Vec::new();
        for id in 1..=32 {
            if let Some(data) = self
                .exchange(id, [0x9A, 0, 0, 0, 0, 0, 0, 0], SCAN_TIMEOUT)
                .await?
            {
                found.push(LingkongProbe {
                    id,
                    motor_state: data[6],
                    error_state: data[7],
                });
            }
        }
        Ok(found)
    }

    pub async fn snapshot(&self, id: u8) -> Result<LingkongSnapshot, String> {
        validate_id(id)?;
        let _operation = self.operation.lock().await;
        self.check_active()?;
        self.snapshot_locked(id).await
    }

    /// A selected drive is left closed. Parameter reads establish the actual
    /// configuration before Run can be requested from the UI.
    pub async fn initialize(&self, id: u8) -> Result<LingkongInitialization, String> {
        validate_id(id)?;
        let _operation = self.operation.lock().await;
        self.check_active()?;
        self.initialized.lock().await.remove(&id);
        let first = self.required_reply(id, 0x9A).await?;
        if is_running_state(first[6]) {
            self.armed.lock().await.insert(id);
        } else if !is_closed_state(first[6]) {
            return Err(format!("motor {id} has unknown state 0x{:02X}", first[6]));
        }
        if !is_closed_state(first[6]) {
            self.close_locked(id).await?;
        }
        if first[7] != 0 {
            self.exchange(id, [0x9B, 0, 0, 0, 0, 0, 0, 0], REPLY_TIMEOUT)
                .await?
                .ok_or_else(|| format!("motor {id} did not acknowledge clear fault"))?;
        }
        let status = self.required_reply(id, 0x9A).await?;
        let expected_running = running_state_for_closed(status[6])
            .filter(|_| status[7] == 0)
            .ok_or_else(|| {
                format!(
                    "motor {id} is not closed and fault-free (state 0x{:02X}, faults 0x{:02X})",
                    status[6], status[7]
                )
            })?;
        let reads = self.read_parameters_locked(id).await?;
        for key in [
            "setting_driver_id",
            "setting_bus_type",
            "setting_can_baudrate",
        ] {
            if reads
                .unavailable_parameters
                .iter()
                .any(|value| value == key)
            {
                return Err(format!(
                    "motor {id} did not provide required parameter {key}"
                ));
            }
        }
        let configured_id = reads
            .parameters
            .iter()
            .find(|parameter| parameter.key == "setting_driver_id")
            .and_then(|parameter| parameter.values.first())
            .copied()
            .unwrap_or(0);
        let settings_dirty = self.dirty_settings.lock().await.contains(&id);
        let reboot_required = self.reboot_required.lock().await.contains(&id);
        if !settings_dirty
            && !reboot_required
            && configured_id != 0
            && configured_id != u32::from(id)
        {
            return Err(format!("motor {id} reports Driver ID {configured_id}; verify the effective ID after reboot"));
        }
        let snapshot = self.snapshot_locked(id).await?;
        if snapshot.motor_state != status[6] || snapshot.error_state != 0 {
            return Err(format!("motor {id} changed state during initialization"));
        }
        self.initialized.lock().await.insert(id, expected_running);
        Ok(LingkongInitialization {
            snapshot,
            parameters: reads.parameters,
            unavailable_parameters: reads.unavailable_parameters,
            settings_dirty,
            reboot_required,
        })
    }

    pub async fn read_parameters(&self, id: u8) -> Result<LingkongParameterReadResult, String> {
        validate_id(id)?;
        let _operation = self.operation.lock().await;
        self.check_initialized(id).await?;
        self.read_parameters_locked(id).await
    }

    async fn read_parameters_locked(&self, id: u8) -> Result<LingkongParameterReadResult, String> {
        let mut parameters = Vec::with_capacity(PARAMETERS.len());
        let mut unavailable_parameters = Vec::new();
        for spec in PARAMETERS {
            match self
                .read_parameter_if_available_locked(id, *spec)
                .await
                .map_err(|error| format!("reading {} failed: {error}", spec.key))?
            {
                Some(value) => parameters.push(value),
                None => unavailable_parameters.push(spec.key.to_string()),
            }
        }
        Ok(LingkongParameterReadResult {
            parameters,
            unavailable_parameters,
        })
    }

    pub async fn write_parameter(
        &self,
        id: u8,
        key: &str,
        values: Vec<u32>,
    ) -> Result<ParameterValue, String> {
        validate_id(id)?;
        let spec = lingkong_params::parameter(key)?;
        let request = spec.write_frame(&values)?;
        let _operation = self.operation.lock().await;
        self.check_initialized(id).await?;
        self.require_closed_and_clear(id).await?;
        // Do not write a parameter whose selector this firmware does not
        // return, even if a caller bypasses the disabled frontend control.
        self.read_parameter_locked(id, spec).await?;
        if spec.key == "setting_driver_id" {
            let next_id = values[0] as u8;
            if next_id != 0
                && next_id != id
                && self
                    .exchange(next_id, [0x9A, 0, 0, 0, 0, 0, 0, 0], SCAN_TIMEOUT)
                    .await?
                    .is_some()
            {
                return Err(format!("motor ID {next_id} already replies on this bus"));
            }
        }
        if spec.is_setting() {
            // Once the write is submitted, even a lost acknowledgement cannot
            // prove the staged setting was rejected. Keep Run blocked.
            self.dirty_settings.lock().await.insert(id);
        }
        let ack = self
            .exchange_parameter(id, request, spec)
            .await?
            .ok_or_else(|| format!("motor {id} returned an empty write reply for {}", spec.key))?;
        let acknowledged = spec.decode(ack, true)?;
        if acknowledged.values != values {
            return Err(format!(
                "{} write acknowledgement differs from requested value",
                spec.key
            ));
        }
        let readback = self.read_parameter_locked(id, spec).await?;
        if readback.values != values {
            return Err(format!(
                "{} read-back differs from requested value",
                spec.key
            ));
        }
        Ok(readback)
    }

    pub async fn save_settings(&self, id: u8) -> Result<(), String> {
        validate_id(id)?;
        let _operation = self.operation.lock().await;
        self.check_initialized(id).await?;
        self.require_closed_and_clear(id).await?;
        if !self.dirty_settings.lock().await.contains(&id) {
            return Err("no setting changes are waiting to be saved".into());
        }
        let reply = self
            .exchange(id, [0x44, 0x05, 0xFA, 0, 0, 0, 0, 0], REPLY_TIMEOUT)
            .await?
            .ok_or_else(|| format!("motor {id} did not confirm ROM save"))?;
        if reply[1] != 0x05 || reply[2] != 0x01 {
            return Err(format!("motor {id} reported ROM save failure"));
        }
        self.dirty_settings.lock().await.remove(&id);
        self.reboot_required.lock().await.insert(id);
        Ok(())
    }

    pub async fn reboot(&self, id: u8) -> Result<LingkongRebootResult, String> {
        validate_id(id)?;
        let _operation = self.operation.lock().await;
        self.check_initialized(id).await?;
        self.require_closed_and_clear(id).await?;
        if self.dirty_settings.lock().await.contains(&id) {
            return Err("save the pending settings before rebooting".into());
        }
        if !self.reboot_required.lock().await.contains(&id) {
            return Err("save setting changes before requesting a reboot".into());
        }
        let configured_id = self
            .read_parameter_locked(id, lingkong_params::parameter("setting_driver_id")?)
            .await?
            .values[0];
        let bus_type = self
            .read_parameter_locked(id, lingkong_params::parameter("setting_bus_type")?)
            .await?
            .values[0];
        let bitrate_code = self
            .read_parameter_locked(id, lingkong_params::parameter("setting_can_baudrate")?)
            .await?
            .values[0];
        let next_bitrate = [100_000, 125_000, 250_000, 500_000, 1_000_000]
            .get(bitrate_code as usize)
            .copied()
            .ok_or_else(|| "motor reported an invalid CAN bitrate code".to_string())?;
        let next_id = if bus_type == 2 {
            u8::try_from(configured_id)
                .ok()
                .filter(|value| (1..=32).contains(value))
        } else {
            None
        };
        // V2.36 explicitly specifies no reply for 0x07. The next session
        // must scan again because ID and bitrate may have changed.
        self.send(id, [0x07, 0, 0, 0, 0, 0, 0, 0]).await?;
        self.initialized.lock().await.remove(&id);
        self.armed.lock().await.remove(&id);
        self.reboot_required.lock().await.remove(&id);
        let mut closed_after_reboot = false;
        if next_bitrate == self.bitrate {
            if let Some(next_id) = next_id {
                for _ in 0..10 {
                    tokio::time::sleep(Duration::from_millis(250)).await;
                    let status = self
                        .exchange(next_id, [0x9A, 0, 0, 0, 0, 0, 0, 0], REPLY_TIMEOUT)
                        .await;
                    if let Ok(Some(_)) = status {
                        self.armed.lock().await.insert(next_id);
                        closed_after_reboot = self.close_locked(next_id).await.is_ok();
                        if closed_after_reboot {
                            break;
                        }
                    }
                }
            }
        }
        Ok(LingkongRebootResult {
            next_id,
            next_bitrate,
            closed_after_reboot,
        })
    }

    async fn read_parameter_locked(
        &self,
        id: u8,
        spec: ParameterSpec,
    ) -> Result<ParameterValue, String> {
        self.read_parameter_if_available_locked(id, spec)
            .await?
            .ok_or_else(|| {
                format!(
                    "motor {id} did not provide parameter {} (empty reply)",
                    spec.key
                )
            })
    }

    async fn read_parameter_if_available_locked(
        &self,
        id: u8,
        spec: ParameterSpec,
    ) -> Result<Option<ParameterValue>, String> {
        let reply = self.exchange_parameter(id, spec.read_frame(), spec).await?;
        reply.map(|frame| spec.decode(frame, false)).transpose()
    }

    async fn exchange_parameter(
        &self,
        id: u8,
        request: [u8; 8],
        spec: ParameterSpec,
    ) -> Result<Option<[u8; 8]>, String> {
        let matches = |reply: &[u8; 8]| {
            reply[0] == request[0]
                && (reply[1..].iter().all(|value| *value == 0)
                    || (reply[1] == request[1]
                        && (!spec.is_setting() || request[1] != 0x05 || reply[2] == request[2])))
        };
        let reply = if matches!(request[0], 0xC0 | 0x40) {
            self.exchange_read_matching(id, request, REPLY_TIMEOUT, matches)
                .await?
        } else {
            // A lost write acknowledgement never triggers another write.
            self.exchange_matching(id, request, REPLY_TIMEOUT, matches)
                .await?
        }
        .ok_or_else(|| format!("motor {id} did not reply to {}", spec.key))?;
        // This firmware returns just the opcode for unavailable selectors.
        // A valid zero value still includes the parameter selector/address.
        if reply[1..].iter().all(|value| *value == 0) {
            Ok(None)
        } else {
            Ok(Some(reply))
        }
    }

    async fn check_initialized(&self, id: u8) -> Result<u8, String> {
        self.check_active()?;
        self.initialized
            .lock()
            .await
            .get(&id)
            .copied()
            .ok_or_else(|| format!("initialize motor {id} before using controls or parameters"))
    }

    /// Returns the expected state after Run for this drive's closed state.
    async fn require_closed_and_clear(&self, id: u8) -> Result<u8, String> {
        let status = self.required_reply(id, 0x9A).await?;
        match running_state_for_closed(status[6]) {
            Some(running_state) if status[7] == 0 => Ok(running_state),
            _ => Err("close the motor and clear all faults before changing parameters".into()),
        }
    }

    pub async fn action(&self, id: u8, action: Action) -> Result<LingkongSnapshot, String> {
        validate_id(id)?;
        let _operation = self.operation.lock().await;
        self.check_active()?;
        let data = action.payload();
        match action {
            Action::Run => {
                let expected_running = self.check_initialized(id).await?;
                if self.dirty_settings.lock().await.contains(&id)
                    || self.reboot_required.lock().await.contains(&id)
                {
                    return Err("save settings and reboot the motor before Run".into());
                }
                if self.require_closed_and_clear(id).await? != expected_running {
                    return Err("motor state encoding changed; initialize the motor again".into());
                }
                // A lost reply does not prove the drive rejected Run. Keep the
                // ID in the safe-close set throughout command submission.
                self.armed.lock().await.insert(id);
                self.send_with_echo(id, data).await?;
                let status = self.required_reply(id, 0x9A).await?;
                if status[6] != expected_running || status[7] != 0 {
                    return Err(format!(
                        "motor did not confirm the running state (state 0x{:02X})",
                        status[6]
                    ));
                }
            }
            Action::Close => {
                self.close_locked(id).await?;
            }
            Action::Stop => {
                // Stop leaves the motor enabled; close it on disconnect.
                self.armed.lock().await.insert(id);
                self.send_with_echo(id, data).await?;
                self.required_reply(id, 0x9A).await?;
            }
            Action::ClearFault => {
                self.exchange(id, data, REPLY_TIMEOUT)
                    .await?
                    .ok_or_else(|| format!("motor {id} did not acknowledge clear fault"))?;
                self.initialized.lock().await.remove(&id);
                let status = self.required_reply(id, 0x9A).await?;
                if status[7] != 0 {
                    return Err(format!(
                        "motor {id} still reports faults 0x{:02X}",
                        status[7]
                    ));
                }
            }
            Action::Torque { .. } | Action::Velocity { .. } | Action::Position { .. } => {
                let expected_running = self.check_initialized(id).await?;
                if !self.armed.lock().await.contains(&id) {
                    return Err("press Run before sending a motion target".into());
                }
                let status = self.required_reply(id, 0x9A).await?;
                if status[6] != expected_running || status[7] != 0 {
                    return Err("motor is not running and fault-free".into());
                }
                self.exchange(id, data, REPLY_TIMEOUT)
                    .await?
                    .ok_or_else(|| format!("motor {id} did not acknowledge motion target"))?;
            }
        }
        self.snapshot_locked(id).await
    }

    pub async fn shutdown(&self) -> Result<(), String> {
        let _operation = self.operation.lock().await;
        let ids: Vec<u8> = self.armed.lock().await.iter().copied().collect();
        let mut failures = Vec::new();
        for id in ids {
            if let Err(error) = self.close_locked(id).await {
                failures.push(format!("ID {id}: {error}"));
            }
        }
        if failures.is_empty() {
            self.active.store(false, Ordering::Release);
            Ok(())
        } else {
            Err(failures.join("; "))
        }
    }

    pub async fn force_shutdown(&self) {
        let _operation = self.operation.lock().await;
        let ids: Vec<u8> = self.armed.lock().await.iter().copied().collect();
        for id in ids {
            if let Err(error) = self.send(id, Action::Close.payload()).await {
                log::warn!("Lingkong ID {id} close on window exit failed: {error}");
            }
        }
        self.active.store(false, Ordering::Release);
    }

    fn check_active(&self) -> Result<(), String> {
        if self.active.load(Ordering::Acquire) {
            Ok(())
        } else {
            Err("Lingkong session is closed".into())
        }
    }

    async fn close_locked(&self, id: u8) -> Result<(), String> {
        self.send_with_echo(id, Action::Close.payload()).await?;
        let status = self.required_reply(id, 0x9A).await?;
        if !is_closed_state(status[6]) {
            return Err("motor did not confirm the closed state".into());
        }
        self.armed.lock().await.remove(&id);
        Ok(())
    }

    async fn snapshot_locked(&self, id: u8) -> Result<LingkongSnapshot, String> {
        let status = self.required_reply(id, 0x9A).await?;
        let live = self.required_reply(id, 0x9C).await?;
        let angle = self
            .exchange(id, [0x92, 0, 0, 0, 0, 0, 0, 0], REPLY_TIMEOUT)
            .await?;
        Ok(decode_snapshot(id, status, live, angle))
    }

    async fn required_reply(&self, id: u8, opcode: u8) -> Result<[u8; 8], String> {
        self.exchange_read_matching(id, [opcode, 0, 0, 0, 0, 0, 0, 0], REPLY_TIMEOUT, |reply| {
            reply[0] == opcode
        })
        .await?
        .ok_or_else(|| format!("motor {id} did not reply to 0x{opcode:02X}"))
    }

    async fn send_with_echo(&self, id: u8, data: [u8; 8]) -> Result<(), String> {
        let reply = self
            .exchange_matching(id, data, REPLY_TIMEOUT, |reply| *reply == data)
            .await?;
        if reply.is_none() {
            // The caller must still confirm the resulting state with 0x9A.
            log::warn!(
                "Lingkong ID {id} missed 0x{:02X} echo; checking motor state",
                data[0]
            );
        }
        Ok(())
    }

    async fn exchange_read_matching(
        &self,
        id: u8,
        data: [u8; 8],
        timeout: Duration,
        matches: impl Fn(&[u8; 8]) -> bool,
    ) -> Result<Option<[u8; 8]>, String> {
        for _ in 0..READ_ATTEMPTS {
            if let Some(reply) = self.exchange_matching(id, data, timeout, &matches).await? {
                return Ok(Some(reply));
            }
        }
        Ok(None)
    }

    async fn send(&self, id: u8, data: [u8; 8]) -> Result<(), String> {
        let frame = CanFrame::new_data(0x140u16 + u16::from(id), &data)
            .map_err(|error| error.to_string())?;
        self.bus
            .send(frame)
            .await
            .map_err(|error| error.to_string())
    }

    async fn exchange(
        &self,
        id: u8,
        data: [u8; 8],
        timeout: Duration,
    ) -> Result<Option<[u8; 8]>, String> {
        self.exchange_matching(id, data, timeout, |reply| reply[0] == data[0])
            .await
    }

    async fn exchange_matching(
        &self,
        id: u8,
        data: [u8; 8],
        timeout: Duration,
        matches: impl Fn(&[u8; 8]) -> bool,
    ) -> Result<Option<[u8; 8]>, String> {
        let can_id = 0x140u16 + u16::from(id);
        let mut rx = self
            .bus
            .subscribe(CanFilter::exact_standard(can_id))
            .await
            .map_err(|error| error.to_string())?;
        // Requests and replies use the same CAN ID. Give the drive time to
        // finish its previous transmission before starting the next one.
        tokio::time::sleep(TURNAROUND_DELAY).await;
        self.send(id, data).await?;
        let deadline = Instant::now() + timeout;
        loop {
            let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
                return Ok(None);
            };
            match tokio::time::timeout(remaining, rx.recv()).await {
                Ok(Ok(frame)) if frame.kind() == FrameKind::Data && frame.data().len() == 8 => {
                    let received = frame.data();
                    let mut reply = [0u8; 8];
                    reply.copy_from_slice(received);
                    if matches(&reply) {
                        return Ok(Some(reply));
                    }
                }
                Ok(Ok(_)) | Ok(Err(CanIoError::Lagged { .. })) => continue,
                Ok(Err(error)) => return Err(error.to_string()),
                Err(_) => return Ok(None),
            }
        }
    }
}

fn validate_id(id: u8) -> Result<(), String> {
    if (1..=32).contains(&id) {
        Ok(())
    } else {
        Err("Lingkong motor ID must be 1–32".into())
    }
}

fn decode_snapshot(
    id: u8,
    status: [u8; 8],
    live: [u8; 8],
    angle: Option<[u8; 8]>,
) -> LingkongSnapshot {
    let angle_deg = angle.map(|data| {
        let sign = if data[7] & 0x80 != 0 { 0xFF } else { 0 };
        let raw = i64::from_le_bytes([
            data[1], data[2], data[3], data[4], data[5], data[6], data[7], sign,
        ]);
        raw as f64 / 100.0
    });
    LingkongSnapshot {
        id,
        temperature_c: status[1] as i8,
        voltage_v: u16::from_le_bytes([status[2], status[3]]) as f64 / 100.0,
        bus_current_a: i16::from_le_bytes([status[4], status[5]]) as f64 / 100.0,
        motor_state: status[6],
        error_state: status[7],
        iq_raw: i16::from_le_bytes([live[2], live[3]]),
        speed_dps: i16::from_le_bytes([live[4], live[5]]),
        encoder: u16::from_le_bytes([live[6], live[7]]),
        angle_deg,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use can_transport::{CanCapabilities, CanRx};
    use std::sync::atomic::AtomicUsize;
    use std::sync::Mutex as StdMutex;
    use tokio::sync::mpsc;

    struct MockRx(mpsc::UnboundedReceiver<CanFrame>);

    #[async_trait::async_trait]
    impl CanRx for MockRx {
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

    struct MockBus {
        state: StdMutex<u8>,
        faults: StdMutex<u8>,
        closed_state: u8,
        running_state: u8,
        sent: StdMutex<Vec<u8>>,
        replies: StdMutex<Vec<mpsc::UnboundedSender<CanFrame>>>,
        unavailable_settings: StdMutex<BTreeSet<u8>>,
        command_busy: Arc<AtomicBool>,
        early_queries: AtomicUsize,
        lost_status_replies: AtomicUsize,
        lost_parameter_replies: AtomicUsize,
        lose_close_echo: AtomicBool,
    }

    #[async_trait::async_trait]
    impl CanBus for MockBus {
        async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
            assert_eq!(frame.id().raw(), 0x141);
            let mut reply: [u8; 8] = frame.data().try_into().unwrap();
            self.sent.lock().unwrap().push(reply[0]);
            if self.command_busy.load(Ordering::Acquire) {
                self.early_queries.fetch_add(1, Ordering::AcqRel);
                return Ok(());
            }
            let loss_counter = match reply[0] {
                0x9A => Some(&self.lost_status_replies),
                0xC0 | 0x40 => Some(&self.lost_parameter_replies),
                _ => None,
            };
            if loss_counter.is_some_and(|counter| {
                counter
                    .fetch_update(Ordering::AcqRel, Ordering::Acquire, |value| {
                        value.checked_sub(1)
                    })
                    .is_ok()
            }) {
                return Ok(());
            }
            match reply[0] {
                0x80 => *self.state.lock().unwrap() = self.closed_state,
                0x88 => *self.state.lock().unwrap() = self.running_state,
                0x9A | 0x9B => {
                    reply[1] = 25;
                    reply[2..4].copy_from_slice(&2400u16.to_le_bytes());
                    reply[6] = *self.state.lock().unwrap();
                    reply[7] = *self.faults.lock().unwrap();
                }
                0x40 if reply[1] == 0x05
                    && self
                        .unavailable_settings
                        .lock()
                        .unwrap()
                        .contains(&reply[2]) =>
                {
                    reply[1..].fill(0);
                }
                _ => {}
            }
            let frame = CanFrame::new_data(0x141u16, &reply).unwrap();
            if matches!(reply[0], 0x80 | 0x81 | 0x88) {
                let senders = self.replies.lock().unwrap().clone();
                let busy = self.command_busy.clone();
                let drop_echo =
                    reply[0] == 0x80 && self.lose_close_echo.swap(false, Ordering::AcqRel);
                busy.store(true, Ordering::Release);
                tokio::spawn(async move {
                    tokio::time::sleep(Duration::from_millis(8)).await;
                    busy.store(false, Ordering::Release);
                    if !drop_echo {
                        for sender in senders {
                            let _ = sender.send(frame);
                        }
                    }
                });
                return Ok(());
            }
            self.replies
                .lock()
                .unwrap()
                .retain(|tx| tx.send(frame).is_ok());
            Ok(())
        }

        async fn subscribe(&self, _: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            self.replies.lock().unwrap().push(tx);
            Ok(Box::new(MockRx(rx)))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: false,
                max_dlen: 8,
            }
        }
    }

    fn test_session(initial: u8, closed: u8, running: u8) -> (Arc<MockBus>, LingkongSession) {
        let bus = Arc::new(MockBus {
            state: StdMutex::new(initial),
            faults: StdMutex::new(0),
            closed_state: closed,
            running_state: running,
            sent: StdMutex::new(Vec::new()),
            replies: StdMutex::new(Vec::new()),
            unavailable_settings: StdMutex::new(BTreeSet::new()),
            command_busy: Arc::new(AtomicBool::new(false)),
            early_queries: AtomicUsize::new(0),
            lost_status_replies: AtomicUsize::new(0),
            lost_parameter_replies: AtomicUsize::new(0),
            lose_close_echo: AtomicBool::new(false),
        });
        let session = LingkongSession {
            bus: bus.clone(),
            bitrate: 1_000_000,
            operation: Mutex::new(()),
            armed: Mutex::new(BTreeSet::new()),
            initialized: Mutex::new(BTreeMap::new()),
            dirty_settings: Mutex::new(BTreeSet::new()),
            reboot_required: Mutex::new(BTreeSet::new()),
            active: AtomicBool::new(true),
        };
        (bus, session)
    }

    #[tokio::test]
    async fn documented_and_mhf_states_support_initialization_motion_and_disconnect() {
        for (initial, closed, running) in [
            (0x00, 0x10, 0x00),
            (0x10, 0x10, 0x00),
            (0x20, 0x20, 0x30),
            (0x30, 0x20, 0x30),
        ] {
            let (bus, session) = test_session(initial, closed, running);
            let result = session.initialize(1).await.unwrap();
            assert_eq!(result.snapshot.motor_state, closed);
            assert_eq!(result.parameters.len(), PARAMETERS.len());
            assert_eq!(session.initialized.lock().await.get(&1), Some(&running));
            assert!(!session.armed.lock().await.contains(&1));
            assert!(!bus.sent.lock().unwrap().contains(&0x88));
            assert!(!bus.sent.lock().unwrap().contains(&0xA1));

            session
                .write_parameter(1, "ram_torque_limit", vec![0])
                .await
                .unwrap();
            let snapshot = session.action(1, Action::Run).await.unwrap();
            assert_eq!(snapshot.motor_state, running);
            for action in [
                Action::Torque { iq_raw: 0 },
                Action::Velocity {
                    speed_dps: 0.0,
                    iq_limit: 100,
                },
                Action::Position {
                    angle_deg: 0.0,
                    max_speed_dps: 120,
                },
            ] {
                let snapshot = session.action(1, action).await.unwrap();
                assert_eq!(snapshot.motor_state, running);
                assert!(bus.sent.lock().unwrap().contains(&action.payload()[0]));
            }
            assert_eq!(
                session.action(1, Action::Stop).await.unwrap().motor_state,
                running
            );
            assert_eq!(
                session.action(1, Action::Close).await.unwrap().motor_state,
                closed
            );
            assert!(!session.armed.lock().await.contains(&1));
            session.action(1, Action::Run).await.unwrap();
            session.shutdown().await.unwrap();
            assert_eq!(*bus.state.lock().unwrap(), closed);
            assert!(!session.active.load(Ordering::Acquire));
            assert!(session.armed.lock().await.is_empty());
            assert_eq!(bus.early_queries.load(Ordering::Acquire), 0);
        }
    }

    #[tokio::test]
    async fn empty_setting_replies_leave_parameters_unavailable_without_blocking_initialization() {
        let (bus, session) = test_session(0x30, 0x20, 0x30);
        bus.unavailable_settings
            .lock()
            .unwrap()
            .extend([0xE0, 0xE2, 0xE4, 0xEA, 0xEC]);
        let result = session.initialize(1).await.unwrap();
        assert_eq!(result.snapshot.motor_state, 0x20);
        assert_eq!(result.parameters.len(), 15);
        assert_eq!(
            result.unavailable_parameters,
            vec![
                "setting_max_power",
                "setting_max_speed",
                "setting_max_angle",
                "setting_current_ramp",
                "setting_speed_ramp",
            ]
        );
        assert!(result
            .parameters
            .iter()
            .all(|value| !result.unavailable_parameters.contains(&value.key)));
        // A zero value with the correct selector remains a readable value.
        assert_eq!(
            result
                .parameters
                .iter()
                .find(|value| value.key == "ram_angle_limit")
                .unwrap()
                .values,
            vec![0]
        );
        let refresh = session.read_parameters(1).await.unwrap();
        assert_eq!(
            refresh.unavailable_parameters,
            result.unavailable_parameters
        );
        assert_eq!(refresh.parameters.len(), 15);
        bus.sent.lock().unwrap().clear();
        assert!(session
            .write_parameter(1, "setting_max_power", vec![100])
            .await
            .is_err());
        assert!(!bus.sent.lock().unwrap().contains(&0x42));
        assert!(!session.dirty_settings.lock().await.contains(&1));
        assert_eq!(
            session.action(1, Action::Run).await.unwrap().motor_state,
            0x30
        );
        session.shutdown().await.unwrap();
        assert_eq!(bus.early_queries.load(Ordering::Acquire), 0);
    }

    #[tokio::test]
    async fn empty_required_identity_parameter_still_blocks_initialization() {
        let (bus, session) = test_session(0x20, 0x20, 0x30);
        bus.unavailable_settings.lock().unwrap().insert(0x0A);
        assert!(session
            .initialize(1)
            .await
            .unwrap_err()
            .contains("required parameter setting_driver_id"));
        assert!(session.initialized.lock().await.is_empty());
        assert!(!bus.sent.lock().unwrap().contains(&0x88));
    }

    #[tokio::test]
    async fn status_and_parameter_reads_retry_transient_reply_loss() {
        let (bus, session) = test_session(0x20, 0x20, 0x30);
        bus.lost_status_replies.store(2, Ordering::Release);
        bus.lost_parameter_replies.store(2, Ordering::Release);
        let result = session.initialize(1).await.unwrap();
        assert_eq!(result.parameters.len(), PARAMETERS.len());
        assert_eq!(
            bus.sent
                .lock()
                .unwrap()
                .iter()
                .filter(|code| **code == 0xC0)
                .count(),
            10
        );
        assert_eq!(bus.early_queries.load(Ordering::Acquire), 0);
    }

    #[tokio::test]
    async fn disconnect_failure_keeps_session_until_closed_state_can_be_confirmed() {
        let (bus, session) = test_session(0x20, 0x20, 0x30);
        session.initialize(1).await.unwrap();
        session.action(1, Action::Run).await.unwrap();
        bus.lost_status_replies
            .store(READ_ATTEMPTS, Ordering::Release);
        assert!(session.shutdown().await.is_err());
        assert!(session.active.load(Ordering::Acquire));
        assert!(session.armed.lock().await.contains(&1));
        session.shutdown().await.unwrap();
        assert!(!session.active.load(Ordering::Acquire));
        assert!(session.armed.lock().await.is_empty());
        assert_eq!(*bus.state.lock().unwrap(), 0x20);
    }

    #[tokio::test]
    async fn missing_close_echo_still_requires_confirmed_closed_state() {
        let (bus, session) = test_session(0x20, 0x20, 0x30);
        session.initialize(1).await.unwrap();
        session.action(1, Action::Run).await.unwrap();
        bus.lose_close_echo.store(true, Ordering::Release);
        session.shutdown().await.unwrap();
        assert_eq!(*bus.state.lock().unwrap(), 0x20);
        assert_eq!(bus.early_queries.load(Ordering::Acquire), 0);
    }

    #[tokio::test]
    async fn mhf_motion_and_parameter_writes_still_require_initialization_and_clear_faults() {
        let (bus, session) = test_session(0x30, 0x20, 0x30);
        let target = Action::Torque { iq_raw: 0 };
        assert!(session.action(1, target).await.is_err());
        assert!(bus.sent.lock().unwrap().is_empty());

        session.initialize(1).await.unwrap();
        assert!(session.action(1, target).await.is_err());
        assert!(!bus.sent.lock().unwrap().contains(&0xA1));

        session.action(1, Action::Run).await.unwrap();
        *bus.faults.lock().unwrap() = 0x01;
        assert!(session.action(1, target).await.is_err());
        assert!(!bus.sent.lock().unwrap().contains(&0xA1));
        session.action(1, Action::Close).await.unwrap();
        bus.sent.lock().unwrap().clear();
        assert!(session.action(1, Action::Run).await.is_err());
        assert!(session
            .write_parameter(1, "ram_torque_limit", vec![0])
            .await
            .is_err());
        assert!(!bus.sent.lock().unwrap().contains(&0x88));
        assert!(!bus.sent.lock().unwrap().contains(&0xC1));
        assert!(session.initialize(1).await.is_err());
        assert!(!session.initialized.lock().await.contains_key(&1));
    }

    #[tokio::test]
    async fn unknown_states_remain_blocked() {
        for state in [0x01, 0x11, 0x21, 0x31, 0xFF] {
            let (bus, session) = test_session(state, 0x20, 0x30);
            assert!(session.initialize(1).await.is_err());
            assert!(!session.initialized.lock().await.contains_key(&1));
            assert_eq!(*bus.sent.lock().unwrap(), vec![0x9A]);
        }
    }

    #[tokio::test]
    async fn run_rejects_a_mismatched_state_pair_and_keeps_motor_tracked_for_close() {
        for (closed, running) in [(0x10, 0x30), (0x20, 0x00)] {
            let (bus, session) = test_session(closed, closed, running);
            session.initialize(1).await.unwrap();
            assert!(session.action(1, Action::Run).await.is_err());
            assert!(session.armed.lock().await.contains(&1));
            assert!(session
                .action(1, Action::Torque { iq_raw: 0 })
                .await
                .is_err());
            assert!(!bus.sent.lock().unwrap().contains(&0xA1));
            session.shutdown().await.unwrap();
            assert_eq!(*bus.state.lock().unwrap(), closed);
            assert!(session.armed.lock().await.is_empty());
        }
    }

    #[test]
    fn v236_velocity_and_position_use_little_endian_scaled_values() {
        assert_eq!(
            Action::Velocity {
                speed_dps: 360.0,
                iq_limit: 100
            }
            .payload(),
            [0xA2, 0, 100, 0, 0xA0, 0x8C, 0, 0]
        );
        assert_eq!(
            Action::Position {
                angle_deg: -180.0,
                max_speed_dps: 120
            }
            .payload(),
            [0xA4, 0, 120, 0, 0xB0, 0xB9, 0xFF, 0xFF]
        );
    }

    #[test]
    fn status_and_signed_multiturn_angle_decode() {
        let snapshot = decode_snapshot(
            1,
            [0x9A, 25, 0xC4, 0x09, 0x9C, 0xFF, 0x10, 0x04],
            [0x9C, 25, 0xFE, 0xFF, 0x88, 0xFF, 0x34, 0x12],
            Some([0x92, 0xB0, 0xB9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
        );
        assert_eq!(snapshot.voltage_v, 25.0);
        assert_eq!(snapshot.bus_current_a, -1.0);
        assert_eq!(snapshot.speed_dps, -120);
        assert_eq!(snapshot.angle_deg, Some(-180.0));
        assert_eq!(snapshot.error_state, 0x04);
    }
}
