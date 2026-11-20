//! Desktop adapter for the pinned string_chassis MHF5010 controller.
//! One Classic CAN link is shared by the motor and optional direction encoder.
//! Its child adapter owns a separate CAN-FD link for HopeA3 chassis follow.
use std::path::{Path, PathBuf};
use std::sync::{
    atomic::{AtomicBool, AtomicU64, Ordering},
    Arc,
};
use std::time::Duration;

use can_transport::{CanBus, CanLinkConfig};
use hex_motor::canopen::sdo;
use magnetic_encoder::{EncoderConfig, EncoderMonitor, EncoderSnapshot};
use mhf_pull_cord::trajectory::{CordEndpoint, CordTrajectory, TrajectoryCursor, TrajectoryUpdate};
use mhf_pull_cord::{
    LingkongPull, Phase, PullCordConfig, PullCordEncoder, PullCordMode, PullCordState,
};
use serde::{Deserialize, Serialize};
use tauri::{Manager, State};
use tokio::sync::Mutex;

use crate::can_lease::CanTransportGate;
use crate::state::AppState;

#[path = "pull_chassis.rs"]
mod chassis;
pub use chassis::*;

#[derive(Default)]
pub struct PullCordApp {
    runtime: Mutex<Runtime>,
    cancel: Arc<AtomicBool>,
    stop_generation: AtomicU64,
    settings_lock: Mutex<()>,
    chassis_cancel: Arc<AtomicBool>,
    chassis_stop_generation: AtomicU64,
}

#[derive(Default)]
struct Runtime {
    bus: Option<Arc<dyn CanBus>>,
    iface: Option<String>,
    encoder: PullCordEncoder,
    control: Option<LingkongPull>,
    // Keep the completed trace available for inspection after a confirmed stop.
    trajectory: Option<CordTrajectory>,
    // A failed startup/stop must remain recoverable even without a live task.
    pending_motor: Option<(u8, PullCordMode)>,
    last_fault: Option<String>,
    chassis: chassis::ChassisConnection,
}

impl Runtime {
    async fn stop(&mut self) -> Result<(), String> {
        let chassis_result = self.chassis.stop().await;
        let result = if let Some(control) = self.control.as_mut() {
            control.stop().await
        } else if let (Some(bus), Some((node, mode))) = (&self.bus, self.pending_motor) {
            mhf_pull_cord::close_motor(bus.as_ref(), node, mode).await
        } else {
            Ok(())
        };
        let pull_result = match result {
            Ok(()) => {
                self.control = None;
                self.pending_motor = None;
                self.last_fault = None;
                Ok(())
            }
            Err(error) => {
                self.last_fault = Some(error.clone());
                Err(error)
            }
        };
        match (chassis_result, pull_result) {
            (Ok(()), Ok(())) => Ok(()),
            (Err(error), Ok(())) | (Ok(()), Err(error)) => Err(error),
            (Err(chassis), Err(cord)) => Err(format!("{chassis}; {cord}")),
        }
    }
}

impl PullCordApp {
    pub fn request_stop(&self) {
        self.stop_generation.fetch_add(1, Ordering::AcqRel);
        self.cancel.store(true, Ordering::Release);
        self.request_chassis_stop();
    }

    /// Keep the bus and failed motor handle until Close is confirmed.
    pub async fn disconnect(&self) -> Result<(), String> {
        self.request_stop();
        let mut runtime = self.runtime.lock().await;
        runtime.stop().await?;
        runtime.chassis.disconnect().await?;
        if let Some(mut encoder) = runtime.encoder.monitor.lock().await.take() {
            encoder.shutdown().await;
        }
        runtime.bus = None;
        runtime.iface = None;
        Ok(())
    }
}

#[derive(Serialize)]
pub struct PullCordStatus {
    connected: bool,
    iface: Option<String>,
    pending_node: Option<u8>,
    pending_mode: Option<PullCordMode>,
    fault: Option<String>,
    run: Option<PullCordState>,
    encoder: Option<EncoderStatus>,
    encoder_center_deg: f64,
    trajectory: Option<TrajectoryUpdate>,
    chassis: chassis::ChassisStatus,
}

#[derive(Serialize)]
struct EncoderStatus {
    node_id: u8,
    angle_degrees: Option<f64>,
    position_counts: Option<u32>,
    angle_usable: bool,
    sensor_status: Option<String>,
    communication: String,
    position_age_ms: Option<u64>,
    status_age_ms: Option<u64>,
    position_frames: u64,
    invalid_frames: u64,
    bus_state: Option<String>,
    receive_error: Option<String>,
}

impl From<EncoderSnapshot> for EncoderStatus {
    fn from(value: EncoderSnapshot) -> Self {
        Self {
            node_id: value.node_id,
            angle_degrees: value.position.map(|p| p.angle_degrees),
            position_counts: value.position.map(|p| p.position_counts),
            angle_usable: value.angle_is_usable(),
            sensor_status: value.sensor_status().map(|s| format!("{s:?}")),
            communication: format!("{:?}", value.communication),
            position_age_ms: value.position_age.map(|age| age.as_millis() as u64),
            status_age_ms: value.status_age.map(|age| age.as_millis() as u64),
            position_frames: value.position_frames,
            invalid_frames: value.invalid_frames,
            bus_state: value.bus_health.state.map(|s| format!("{s:?}")),
            receive_error: value.receive_error.or(value.last_decode_error),
        }
    }
}

fn validate_classic_link(link: Option<CanLinkConfig>) -> Result<(), String> {
    let link = link.ok_or("cannot verify CAN timing; configure Classic CAN at 1 Mbps first")?;
    if link.fd_enabled != Some(false)
        || link.nominal.and_then(|timing| timing.bitrate) != Some(1_000_000)
    {
        return Err("MHF5010 and encoder require Classic CAN at 1 Mbps (CAN FD disabled); configure the selected interface first".into());
    }
    Ok(())
}

#[tauri::command]
pub async fn pull_cord_connect(
    state: State<'_, AppState>,
    gate: State<'_, CanTransportGate>,
    iface: String,
    encoder_node: u8,
    encoder_center_deg: f64,
) -> Result<(), String> {
    if !encoder_center_deg.is_finite() || !(0.0..360.0).contains(&encoder_center_deg) {
        return Err("encoder centre must be within 0..360 degrees".into());
    }
    let _operation = state.extension_op.lock().await;
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("application is closing".into());
    }
    let encoder_config = EncoderConfig::for_node(encoder_node).map_err(|e| e.to_string())?;
    let mut runtime = state.pull_cord.runtime.lock().await;
    if runtime.bus.is_some() {
        return Err("pull-cord CAN is already connected".into());
    }
    chassis::check_distinct_interfaces(runtime.chassis.iface.as_deref(), &iface)?;
    let lease = gate.try_acquire_pull_channel()?;
    // Native close must select the extended cleanup even while open is pending.
    state
        .extension_runtime_active
        .store(true, Ordering::Release);
    let bus = crate::backend::open_classic_1m_bus(iface.trim(), lease)
        .await
        .map_err(|e| e.to_string())?;
    validate_classic_link(bus.link_config().await.map_err(|e| e.to_string())?)?;
    let bus = crate::backend::protect_feature_bus(bus);
    let encoder = EncoderMonitor::start(bus.clone(), encoder_config)
        .await
        .map_err(|e| e.to_string())?;
    *runtime.encoder.monitor.lock().await = Some(encoder);
    *runtime.encoder.center_deg.lock().unwrap() = encoder_center_deg;
    runtime.bus = Some(bus);
    runtime.iface = Some(iface.trim().to_owned());
    runtime.trajectory = None;
    Ok(())
}

#[tauri::command]
pub async fn pull_cord_disconnect(state: State<'_, AppState>) -> Result<(), String> {
    state.pull_cord.request_stop();
    let _operation = state.extension_op.lock().await;
    state.pull_cord.disconnect().await
}

#[tauri::command]
pub async fn pull_cord_status(
    state: State<'_, AppState>,
    trajectory_after: Option<TrajectoryCursor>,
) -> Result<PullCordStatus, String> {
    let runtime = state.pull_cord.runtime.lock().await;
    let encoder = runtime
        .encoder
        .monitor
        .lock()
        .await
        .as_ref()
        .map(|monitor| monitor.snapshot().into());
    let encoder_center_deg = *runtime.encoder.center_deg.lock().unwrap();
    Ok(PullCordStatus {
        connected: runtime.bus.is_some(),
        iface: runtime.iface.clone(),
        pending_node: runtime.pending_motor.map(|(node, _)| node),
        pending_mode: runtime.pending_motor.map(|(_, mode)| mode),
        fault: runtime.last_fault.clone(),
        run: runtime.control.as_ref().map(LingkongPull::snapshot),
        encoder,
        encoder_center_deg,
        trajectory: runtime
            .trajectory
            .as_ref()
            .map(|trace| trace.since(trajectory_after)),
        chassis: runtime.chassis.status(),
    })
}

/// Lightweight read-only endpoint feedback for consumers that do not need the plot.
#[tauri::command]
pub async fn pull_cord_endpoint(state: State<'_, AppState>) -> Result<Option<CordEndpoint>, String> {
    let runtime = state.pull_cord.runtime.lock().await;
    Ok(runtime.trajectory.as_ref().and_then(CordTrajectory::latest))
}

#[tauri::command]
pub async fn pull_cord_clear_trajectory(
    state: State<'_, AppState>,
    cursor: TrajectoryCursor,
) -> Result<Option<TrajectoryCursor>, String> {
    let runtime = state.pull_cord.runtime.lock().await;
    Ok(runtime
        .trajectory
        .as_ref()
        .and_then(|trajectory| trajectory.clear_history(cursor)))
}

#[tauri::command]
pub async fn pull_cord_start(
    state: State<'_, AppState>,
    node_id: u8,
    config: PullCordConfig,
) -> Result<PullCordState, String> {
    let generation = state.pull_cord.stop_generation.load(Ordering::Acquire);
    let config = mhf_pull_cord::validate_config(config)?;
    if !(1..=32).contains(&node_id) {
        return Err("Lingkong motor ID must be 1..32".into());
    }
    let _operation = state.extension_op.lock().await;
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("application is closing".into());
    }
    let mut runtime = state.pull_cord.runtime.lock().await;
    if runtime.pending_motor.is_some() {
        return Err("stop and confirm the previous motor is closed before restarting".into());
    }
    let bus = runtime.bus.clone().ok_or("connect pull-cord CAN first")?;
    state.pull_cord.cancel.store(false, Ordering::Release);
    // Reset first, then fence against every stop received while either lock
    // was pending. A later stop leaves the flag set for upstream startup.
    if generation != state.pull_cord.stop_generation.load(Ordering::Acquire) {
        state.pull_cord.cancel.store(true, Ordering::Release);
        return Err("pull-cord start cancelled by a stop request".into());
    }
    runtime.pending_motor = Some((node_id, config.control_mode));
    runtime.last_fault = None;
    match mhf_pull_cord::start(
        bus,
        node_id,
        config,
        state.pull_cord.cancel.clone(),
        runtime.encoder.clone(),
    )
    .await
    {
        Ok(control) => {
            let snapshot = control.snapshot();
            runtime.trajectory = Some(control.trajectory());
            runtime.control = Some(control);
            Ok(snapshot)
        }
        Err(error) => {
            // Preserve the selected node even if upstream startup's Close failed.
            runtime.last_fault = Some(error.clone());
            Err(error)
        }
    }
}

#[tauri::command]
pub async fn pull_cord_stop(state: State<'_, AppState>) -> Result<(), String> {
    // Interrupt configuration before waiting for the lifecycle mutex.
    state.pull_cord.request_stop();
    let _operation = state.extension_op.lock().await;
    state.pull_cord.runtime.lock().await.stop().await
}

#[tauri::command]
pub async fn pull_cord_rehome(state: State<'_, AppState>) -> Result<(), String> {
    state.pull_cord.request_chassis_stop();
    let _operation = state.extension_op.lock().await;
    let mut runtime = state.pull_cord.runtime.lock().await;
    runtime.chassis.stop().await?;
    let control = runtime
        .control
        .as_ref()
        .ok_or("pull-cord control is not running")?;
    if control.snapshot().phase == Phase::Fault {
        return Err("stop the faulted session before restarting".into());
    }
    control.rehome();
    Ok(())
}

#[tauri::command]
pub async fn pull_cord_set_config(
    state: State<'_, AppState>,
    config: PullCordConfig,
) -> Result<(), String> {
    let runtime = state.pull_cord.runtime.lock().await;
    runtime
        .control
        .as_ref()
        .ok_or("pull-cord control is not running")?
        .set_config(config)
}

#[tauri::command]
pub fn pull_cord_default_config() -> PullCordConfig {
    let config = serde_json::from_str(include_str!("../defaults/pull-cord.json"))
        .expect("embedded HopeA3 pull-cord configuration");
    mhf_pull_cord::validate_config(config).expect("valid embedded HopeA3 pull-cord configuration")
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SavedSettings {
    schema_version: u32,
    pull_cord: PullCordConfig,
}

fn settings_path(app: &tauri::AppHandle) -> Result<PathBuf, String> {
    app.path()
        .app_config_dir()
        .map(|dir| dir.join("pull-cord.json"))
        .map_err(|e| e.to_string())
}

fn load_settings(path: &Path) -> Result<PullCordConfig, String> {
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            return Ok(pull_cord_default_config())
        }
        Err(error) => return Err(error.to_string()),
    };
    let saved: SavedSettings =
        serde_json::from_slice(&bytes).map_err(|e| format!("invalid pull-cord settings: {e}"))?;
    if saved.schema_version != 1 {
        return Err("unsupported pull-cord settings version".into());
    }
    mhf_pull_cord::validate_config(saved.pull_cord)
}

fn save_settings(path: &Path, config: PullCordConfig) -> Result<PullCordConfig, String> {
    use std::io::Write;
    let config = mhf_pull_cord::validate_config(config)?;
    let bytes = serde_json::to_vec_pretty(&SavedSettings {
        schema_version: 1,
        pull_cord: config,
    })
    .map_err(|e| e.to_string())?;
    std::fs::create_dir_all(path.parent().ok_or("missing settings directory")?)
        .map_err(|e| e.to_string())?;
    let temporary = path.with_extension("json.tmp");
    let mut file = std::fs::File::create(&temporary).map_err(|e| e.to_string())?;
    file.write_all(&bytes)
        .and_then(|_| file.sync_all())
        .map_err(|e| e.to_string())?;
    drop(file);
    std::fs::rename(temporary, path).map_err(|e| e.to_string())?;
    Ok(config)
}

#[tauri::command]
pub async fn pull_cord_settings_get(
    app: tauri::AppHandle,
    state: State<'_, AppState>,
) -> Result<PullCordConfig, String> {
    let _guard = state.pull_cord.settings_lock.lock().await;
    load_settings(&settings_path(&app)?)
}

#[tauri::command]
pub async fn pull_cord_settings_save(
    app: tauri::AppHandle,
    state: State<'_, AppState>,
    config: PullCordConfig,
) -> Result<PullCordConfig, String> {
    let _guard = state.pull_cord.settings_lock.lock().await;
    save_settings(&settings_path(&app)?, config)
}

#[derive(Serialize)]
pub struct EncoderZeroResult {
    zero_counts: u32,
    applied_live: bool,
}

/// Match upstream's identity check, raw-position capture, write/readback and
/// fresh TPDO check. This is the encoder's persistent firmware zero.
#[tauri::command]
pub async fn pull_cord_encoder_zero(
    state: State<'_, AppState>,
) -> Result<EncoderZeroResult, String> {
    use magnetic_encoder::{
        EXPECTED_PRODUCT_CODE, RAW_POSITION_INDEX, RAW_POSITION_SUBINDEX, ZERO_POSITION_INDEX,
        ZERO_POSITION_SUBINDEX,
    };
    let _operation = state.extension_op.lock().await;
    let _settings = state.device_settings_operation.acquire().await;
    let runtime = state.pull_cord.runtime.lock().await;
    if runtime.pending_motor.is_some() {
        return Err("stop the pull-cord motor before changing encoder zero".into());
    }
    let snapshot = runtime
        .encoder
        .monitor
        .lock()
        .await
        .as_ref()
        .ok_or("encoder is not connected")?
        .snapshot();
    if !snapshot.angle_is_usable() {
        return Err("encoder angle is not usable; check the magnet and feedback".into());
    }
    let bus = runtime.bus.as_ref().ok_or("connect pull-cord CAN first")?;
    let node = snapshot.node_id;
    let timeout = Some(Duration::from_millis(500));
    let product = sdo::upload_u32(bus.as_ref(), node, 0x1018, 2, timeout)
        .await
        .map_err(|e| e.to_string())?;
    if product != EXPECTED_PRODUCT_CODE {
        return Err(format!(
            "node 0x{node:02X} product 0x{product:08X} is not the expected magnetic encoder"
        ));
    }
    let _previous = sdo::upload_u32(
        bus.as_ref(),
        node,
        ZERO_POSITION_INDEX,
        ZERO_POSITION_SUBINDEX,
        timeout,
    )
    .await
    .map_err(|e| e.to_string())?;
    let raw = sdo::upload_u32(
        bus.as_ref(),
        node,
        RAW_POSITION_INDEX,
        RAW_POSITION_SUBINDEX,
        timeout,
    )
    .await
    .map_err(|e| e.to_string())?;
    if raw >= snapshot.counts_per_turn {
        return Err("encoder raw position is outside one turn".into());
    }
    sdo::download_u32(
        bus.as_ref(),
        node,
        ZERO_POSITION_INDEX,
        ZERO_POSITION_SUBINDEX,
        raw,
        timeout,
    )
    .await
    .map_err(|e| e.to_string())?;
    let zero = sdo::upload_u32(
        bus.as_ref(),
        node,
        ZERO_POSITION_INDEX,
        ZERO_POSITION_SUBINDEX,
        timeout,
    )
    .await
    .map_err(|e| e.to_string())?;
    if zero != raw {
        return Err(format!(
            "encoder zero verification failed: wrote {raw}, read {zero}"
        ));
    }
    let mut applied_live = false;
    for _ in 0..25 {
        tokio::time::sleep(Duration::from_millis(20)).await;
        let next = runtime
            .encoder
            .monitor
            .lock()
            .await
            .as_ref()
            .unwrap()
            .snapshot();
        if next.position_frames > snapshot.position_frames && next.angle_is_usable() {
            if let Some(position) = next.position {
                if position
                    .position_counts
                    .min(next.counts_per_turn - position.position_counts)
                    <= 16
                {
                    applied_live = true;
                    break;
                }
            }
        }
    }
    Ok(EncoderZeroResult {
        zero_counts: zero,
        applied_live,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use can_transport::{CanCapabilities, CanFilter, CanFrame, CanIoError, CanRx};
    use std::sync::Mutex as StdMutex;
    use tokio::sync::mpsc;

    #[derive(Default)]
    struct CloseBus {
        fail: AtomicBool,
        receiver: StdMutex<Option<mpsc::UnboundedSender<CanFrame>>>,
        sent: StdMutex<Vec<u8>>,
    }

    struct CloseRx(mpsc::UnboundedReceiver<CanFrame>);

    #[async_trait::async_trait]
    impl CanRx for CloseRx {
        async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
            self.0.recv().await.ok_or(CanIoError::Disconnected)
        }
        fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
            Ok(self.0.try_recv().ok())
        }
    }

    #[async_trait::async_trait]
    impl CanBus for CloseBus {
        async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
            if self.fail.load(Ordering::Acquire) {
                return Err(CanIoError::Disconnected);
            }
            let mut reply = frame.data().to_vec();
            self.sent.lock().unwrap().push(reply[0]);
            if reply[0] == 0x9A {
                reply[6] = 0x10;
            }
            if let Some(tx) = self.receiver.lock().unwrap().as_ref() {
                let _ = tx.send(CanFrame::new_data(frame.id(), &reply).unwrap());
            }
            Ok(())
        }
        async fn subscribe(&self, _: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            *self.receiver.lock().unwrap() = Some(tx);
            Ok(Box::new(CloseRx(rx)))
        }
        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: false,
                max_dlen: 8,
            }
        }
    }

    #[tokio::test]
    async fn failed_close_keeps_the_node_and_bus_until_a_confirmed_retry() {
        let app = PullCordApp::default();
        let bus = Arc::new(CloseBus::default());
        bus.fail.store(true, Ordering::Release);
        {
            let mut runtime = app.runtime.lock().await;
            runtime.bus = Some(bus.clone());
            runtime.iface = Some("test".into());
            runtime.pending_motor = Some((1, PullCordMode::A1Current)); // failed startup without a handle
        }
        assert!(app.disconnect().await.is_err());
        assert!(app.cancel.load(Ordering::Acquire));
        {
            let runtime = app.runtime.lock().await;
            assert!(runtime.bus.is_some());
            assert_eq!(runtime.pending_motor, Some((1, PullCordMode::A1Current)));
            assert!(runtime.last_fault.is_some());
        }
        bus.fail.store(false, Ordering::Release);
        app.disconnect().await.unwrap();
        assert_eq!(*bus.sent.lock().unwrap(), [0xA1, 0x81, 0x80, 0x9A]);
        let runtime = app.runtime.lock().await;
        assert!(runtime.bus.is_none());
        assert!(runtime.pending_motor.is_none());
        assert!(runtime.last_fault.is_none());
    }

    #[test]
    fn classic_link_rejects_fd_wrong_speed_and_unknown_timing() {
        assert!(validate_classic_link(None).is_err());
        for (fd, bitrate, accepted) in [
            (false, 1_000_000, true),
            (true, 1_000_000, false),
            (false, 500_000, false),
        ] {
            assert_eq!(
                validate_classic_link(Some(CanLinkConfig {
                    fd_enabled: Some(fd),
                    nominal: Some(can_transport::CanBitTiming {
                        bitrate: Some(bitrate),
                        ..Default::default()
                    }),
                    ..Default::default()
                }))
                .is_ok(),
                accepted
            );
        }
    }

    #[test]
    fn settings_round_trip_and_invalid_save_preserves_previous_values() {
        let dir =
            std::env::temp_dir().join(format!("hex-gui-pull-cord-test-{}", std::process::id()));
        let path = dir.join("pull-cord.json");
        assert_eq!(load_settings(&path).unwrap(), pull_cord_default_config());
        let mut config = pull_cord_default_config();
        config.a2_speed.as_mut().unwrap().max_raw = 275;
        save_settings(&path, config).unwrap();
        let mut legacy: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        legacy["pull_cord"]
            .as_object_mut()
            .unwrap()
            .remove("control_mode");
        std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
        assert_eq!(config.control_mode, PullCordMode::A2Speed);
        assert_eq!(load_settings(&path).unwrap(), config);
        let mut bad = config;
        bad.a2_speed.as_mut().unwrap().max_raw = 2001;
        assert!(save_settings(&path, bad).is_err());
        assert_eq!(load_settings(&path).unwrap(), config);
        config.control_mode = PullCordMode::A1Current;
        config.a1_current.as_mut().unwrap().near_raw = 117;
        save_settings(&path, config).unwrap();
        assert_eq!(load_settings(&path).unwrap(), config);
        bad = config;
        bad.a1_current.as_mut().unwrap().max_raw = 2049;
        assert!(save_settings(&path, bad).is_err());
        assert_eq!(load_settings(&path).unwrap(), config);

        let mut legacy: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        let old_current = legacy["pull_cord"]["a1_current"].as_object_mut().unwrap();
        old_current.remove("payout_current_raw");
        old_current.remove("retract_current_raw");
        old_current.insert("retract_raw".into(), 1800.into()); // ignored by the previous release
        std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
        let a1 = config.a1_current.as_mut().unwrap();
        a1.payout_current_raw = 58; // half of the saved stationary value of 117
        a1.retract_current_raw = 117;
        assert_eq!(load_settings(&path).unwrap(), config);
        std::fs::write(&path, b"invalid").unwrap();
        assert!(load_settings(&path).is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }
}
