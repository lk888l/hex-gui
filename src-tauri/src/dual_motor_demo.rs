//! Independent dual-CAN MIT position pendulum. Motion and teardown
//! run in Rust, so UI redraws cannot interrupt the command stream.
use crate::{
    backend,
    can_lease::{CanOwner, CanTransportGate},
    damiao::{DamiaoConfig, DamiaoDiscovery, DamiaoMode, DamiaoSession, DamiaoTarget},
};
use anyhow::{bail, Context, Result};
use hex_motor::cia402::PdoProfile;
use hex_motor::meow_motor::{
    MeowMitTarget, MeowMotorLogic, MeowMotorManager, MeowMotorManagerOptions, MeowMotorMode,
    MeowMotorTarget,
};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};
use tokio::sync::Mutex;

const TAU: f64 = std::f64::consts::TAU;
const TICK: Duration = Duration::from_millis(40);

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DemoConnection {
    pub meow_iface: String,
    pub meow_bitrate: u32,
    pub dm_iface: String,
    pub dm_bitrate: u32,
    pub host_node_id: u8,
    pub meow_node_id: Option<u8>,
    pub dm_motor_id: Option<u16>,
}

impl Default for DemoConnection {
    fn default() -> Self {
        Self {
            meow_iface: if cfg!(target_os = "linux") {
                "can0"
            } else {
                "gs_usb0"
            }
            .into(),
            meow_bitrate: 5_000_000,
            dm_iface: if cfg!(target_os = "linux") {
                "can1"
            } else {
                "gs_usb1"
            }
            .into(),
            dm_bitrate: 1_000_000,
            host_node_id: 10,
            meow_node_id: None,
            dm_motor_id: None,
        }
    }
}

impl DemoConnection {
    fn validate(&self) -> Result<()> {
        if self.meow_iface.trim().is_empty() || self.dm_iface.trim().is_empty() {
            bail!("CAN interface must not be empty");
        }
        if !(1..=127).contains(&self.host_node_id) {
            bail!("Host Node-ID must be 1..127");
        }
        if self
            .meow_node_id
            .is_some_and(|n| n == 0 || n > 127 || n == self.host_node_id)
        {
            bail!("Motor Node-ID must be 1..127 and different from host");
        }
        if self.dm_motor_id.is_some_and(|n| n > 0x7ff) {
            bail!("DAMIAO ID must be 0..2047");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DemoMotion {
    pub amplitude_deg: f64,
    pub period_s: f64,
    pub speed_variation: f64,
    pub meow_kp: f64,
    pub meow_kd: f64,
    pub dm_kp: f64,
    pub dm_kd: f64,
    pub dm_reverse: bool,
}

impl Default for DemoMotion {
    fn default() -> Self {
        Self {
            amplitude_deg: 180.0,
            period_s: 5.0,
            speed_variation: 0.35,
            meow_kp: 3.0,
            meow_kd: 0.15,
            dm_kp: 3.0,
            dm_kd: 0.15,
            dm_reverse: false,
        }
    }
}

impl DemoMotion {
    fn validate(&self) -> Result<()> {
        if !(1.0..=180.0).contains(&self.amplitude_deg)
            || !(2.0..=30.0).contains(&self.period_s)
            || !(0.0..=0.8).contains(&self.speed_variation)
            || !(0.1..=30.0).contains(&self.meow_kp)
            || !(0.01..=5.0).contains(&self.meow_kd)
            || !(0.1..=30.0).contains(&self.dm_kp)
            || !(0.01..=5.0).contains(&self.dm_kd)
        {
            bail!("MIT pendulum requires amplitude 1..180°, period 2..30 s, speed variation 0..0.8, Kp 0.1..30 and Kd 0.01..5");
        }
        Ok(())
    }

    /// Phase speed varies over four swings. Return position AND its analytic
    /// derivative for MIT feed-forward; a C1 startup envelope avoids a jump.
    fn target_at(&self, elapsed: f64) -> (f64, f64) {
        let omega = TAU / self.period_s;
        let mod_omega = omega / 4.0;
        let phase = omega * elapsed + 4.0 * self.speed_variation * (mod_omega * elapsed).sin();
        let phase_rate = omega * (1.0 + self.speed_variation * (mod_omega * elapsed).cos());
        let u = elapsed.clamp(0.0, 1.0);
        let envelope = u * u * (3.0 - 2.0 * u);
        let envelope_rate = if elapsed < 1.0 {
            6.0 * u * (1.0 - u)
        } else {
            0.0
        };
        let amplitude = self.amplitude_deg.to_radians();
        (
            amplitude * envelope * phase.sin(),
            amplitude * (envelope_rate * phase.sin() + envelope * phase.cos() * phase_rate),
        )
    }
}

fn meow_gain(si: f64, factor: f32) -> Result<u16> {
    if !factor.is_finite() || factor <= 0.0 {
        bail!("hexmeow MIT gain factor is unavailable");
    }
    let raw = (si * TAU / f64::from(factor)).round();
    if !raw.is_finite() || !(0.0..=65535.0).contains(&raw) {
        bail!("MIT gain exceeds hexmeow's encoding range");
    }
    Ok(raw as u16)
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct MotorView {
    pub node_id: Option<u16>,
    pub online: bool,
    pub enabled: bool,
    pub target_rpm: f64,
    pub actual_rpm: Option<f64>,
    pub center_rad: Option<f64>,
    pub target_deg: f64,
    pub actual_deg: Option<f64>,
    pub temperature_c: Option<f64>,
    pub feedback_age_ms: Option<u64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct DemoView {
    pub connected: bool,
    pub running: bool,
    pub phase: String,
    pub elapsed_s: f64,
    pub connection: Option<DemoConnection>,
    pub motion: DemoMotion,
    pub meow: MotorView,
    pub dm: MotorView,
    pub error: Option<String>,
    pub stop_error: Option<String>,
}

impl Default for DemoView {
    fn default() -> Self {
        Self {
            connected: false,
            running: false,
            phase: "disconnected".into(),
            elapsed_s: 0.0,
            connection: None,
            motion: DemoMotion::default(),
            meow: MotorView::default(),
            dm: MotorView::default(),
            error: None,
            stop_error: None,
        }
    }
}

struct Session {
    meow: Arc<MeowMotorManager>,
    nid: u8,
    dm: Arc<DamiaoSession>,
    original_dm_pmax: Mutex<Option<f32>>,
}

impl Session {
    fn telemetry(&self, view: &mut DemoView) {
        if let Ok(state) = self.meow.status(self.nid) {
            view.meow.node_id = Some(u16::from(self.nid));
            view.meow.online = state.connection.online;
            view.meow.enabled = matches!(state.logic, Some(MeowMotorLogic::Enabled(_)));
            view.meow.actual_rpm = state.measurements.velocity_rev_per_s.map(|v| v * 60.0);
            view.meow.actual_deg = state
                .measurements
                .position
                .zip(view.meow.center_rad)
                .map(|(p, center)| (p.to_revolutions() * TAU - center).to_degrees());
            view.meow.temperature_c = state.measurements.motor_temp_c.map(f64::from);
            view.meow.feedback_age_ms = state
                .connection
                .last_tpdo1
                .map(|t| t.elapsed().as_millis() as u64);
        }
        let dm = self.dm.snapshot();
        view.dm.node_id = Some(dm.motor_id);
        view.dm.online = dm.online;
        view.dm.enabled = dm.enabled;
        view.dm.actual_rpm = dm.velocity_rad_s.map(|v| f64::from(v) * 60.0 / TAU);
        // Use the fast MIT feedback only when the configured mapping covers
        // the true multi-turn position. Otherwise fall back to float p_m.
        let position = dm
            .position_rad
            .filter(|_| dm.position_read_age_ms.is_some_and(|age| age < 750));
        let position = if position.is_some_and(|p| p.abs() < dm.p_max) {
            dm.feedback_position_rad.or(position)
        } else {
            position
        };
        view.dm.actual_deg = position
            .zip(view.dm.center_rad)
            .map(|(p, center)| (f64::from(p) - center).to_degrees());
        view.dm.temperature_c = dm.rotor_temp_c.map(f64::from);
        view.dm.feedback_age_ms = dm.feedback_age_ms;
    }

    async fn disable(&self) -> Result<()> {
        // Always attempt both, including when only the first enable succeeded.
        self.dm.stop_stream();
        let (a, b) = tokio::join!(self.meow.disable(self.nid), self.dm.disable());
        let mut errors = Vec::new();
        if let Err(e) = a {
            errors.push(format!("hexmeow stop: {e}"));
        }
        if let Err(e) = b {
            errors.push(format!("DAMIAO stop: {e}"));
        } else {
            // PMAX is a volatile protocol scale, not a zero preset. Restore
            // after confirmed disable, without saving to flash or moving.
            let mut original = self.original_dm_pmax.lock().await;
            if let Some(pmax) = *original {
                match self.dm.write_register(0x15, f64::from(pmax)).await {
                    Ok(_) => {
                        *original = None;
                    }
                    Err(e) => errors.push(format!("Restore DAMIAO PMAX: {e}")),
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            bail!("{}", errors.join("; "))
        }
    }
}

#[derive(Default)]
struct Runtime {
    session: Option<Arc<Session>>,
    job: Option<tokio::task::JoinHandle<()>>,
}

#[derive(Default)]
pub struct DualMotorDemo {
    runtime: Mutex<Runtime>,
    view: Arc<StdMutex<DemoView>>,
    cancel: Arc<AtomicBool>,
    closing: AtomicBool,
}

impl DualMotorDemo {
    pub async fn snapshot(&self) -> DemoView {
        let runtime = self.runtime.lock().await;
        let mut view = self.view.lock().unwrap();
        if let Some(session) = runtime.session.as_ref() {
            session.telemetry(&mut view);
        }
        view.clone()
    }

    pub async fn connect(
        &self,
        gate: &CanTransportGate,
        mut config: DemoConnection,
    ) -> Result<DemoView> {
        config.meow_iface = config.meow_iface.trim().into();
        config.dm_iface = config.dm_iface.trim().into();
        config.validate()?;
        let mut runtime = self.runtime.lock().await;
        if self.closing.load(Ordering::Acquire) {
            bail!("Application is closing");
        }
        if runtime.session.is_some() {
            bail!("Disconnect the demo before changing CAN settings");
        }
        let lease = gate
            .try_acquire(CanOwner::DualMotorDemo)
            .map_err(anyhow::Error::msg)?;
        let [meow_bus, dm_bus] = backend::open_demo_buses(
            &config.meow_iface,
            config.meow_bitrate,
            &config.dm_iface,
            config.dm_bitrate,
            lease,
        )
        .await?;
        let meow = Arc::new(MeowMotorManager::new(
            meow_bus,
            MeowMotorManagerOptions {
                heartbeat_node_id: config.host_node_id,
                auto_identify: true,
                sdo_timeout: Duration::from_millis(500),
                ..Default::default()
            },
        )?);
        let discovery = DamiaoDiscovery::start(dm_bus.clone()).await?;
        let found = async {
            let deadline = Instant::now() + Duration::from_secs(6);
            loop {
                let nodes: Vec<_> = meow.list().into_iter().filter(|m| m.online && m.identity.as_ref().is_some_and(|id|
                    id.vendor_id == crate::device_registry::MEOW_MOTOR_VENDOR_ID && id.product_code == crate::device_registry::MEOW_MOTOR_4310_PRODUCT_CODE))
                    .filter(|m| config.meow_node_id.is_none_or(|nid| m.node_id == nid)).collect();
                let motors: Vec<_> = discovery.snapshot().into_iter().filter(|m| m.online)
                    .filter(|m| config.dm_motor_id.is_none_or(|id| m.motor_id == id)).collect();
                if nodes.len() > 1 || (config.dm_motor_id.is_none() && motors.len() > 1) {
                    bail!("Multiple motors found; enter the desired Node-ID / motor ID");
                }
                if let Some(node) = nodes.first() {
                    if node.node_id == config.host_node_id { bail!("Host and motor Node-ID collide"); }
                    if let Some(id) = config.dm_motor_id.or_else(|| motors.first().map(|m| m.motor_id)) {
                        return Ok((node.node_id, id));
                    }
                }
                if Instant::now() >= deadline { bail!("Discovery timed out: hexmeow-4310 on {} ({} bit/s), DAMIAO on {} ({} bit/s). Check power, CAN channel and motor ID.", config.meow_iface, config.meow_bitrate, config.dm_iface, config.dm_bitrate); }
                if self.closing.load(Ordering::Acquire) { bail!("Application is closing"); }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        }.await;
        discovery.stop_and_wait().await;
        let (nid, dm_id) = found?;
        let dm = DamiaoSession::start(
            dm_bus,
            DamiaoConfig {
                motor_id: dm_id,
                master_id: 0,
                mode: DamiaoMode::Mit,
                p_max: 12.5,
                v_max: 30.0,
                t_max: 10.0,
            },
        )
        .await?;
        let session = Arc::new(Session {
            meow,
            nid,
            dm,
            original_dm_pmax: Mutex::new(None),
        });
        let mut view = self.view.lock().unwrap();
        *view = DemoView {
            connected: true,
            phase: "ready".into(),
            connection: Some(config),
            ..DemoView::default()
        };
        session.telemetry(&mut view);
        runtime.session = Some(session);
        Ok(view.clone())
    }

    pub async fn start(&self, motion: DemoMotion) -> Result<DemoView> {
        motion.validate()?;
        let mut runtime = self.runtime.lock().await;
        if self.closing.load(Ordering::Acquire) {
            bail!("Application is closing");
        }
        if runtime.job.as_ref().is_some_and(|j| !j.is_finished()) {
            bail!("Demo is already running");
        }
        let session = runtime
            .session
            .clone()
            .context("Connect both CAN channels first")?;
        if self.view.lock().unwrap().stop_error.is_some() {
            bail!("Retry Stop before restarting; a motor has not confirmed disable");
        }
        self.cancel.store(false, Ordering::Release);
        let cancel = self.cancel.clone();
        let shared = self.view.clone();
        {
            let mut v = shared.lock().unwrap();
            v.running = true;
            v.phase = "starting".into();
            v.motion = motion;
            v.elapsed_s = 0.0;
            v.error = None;
            v.stop_error = None;
        }
        runtime.job = Some(tokio::spawn(async move {
            let result = run_motion(&session, motion, &cancel, &shared).await;
            shared.lock().unwrap().phase = "stopping".into();
            let stop = session.disable().await;
            let mut v = shared.lock().unwrap();
            session.telemetry(&mut v);
            v.meow.target_rpm = 0.0;
            v.dm.target_rpm = 0.0;
            v.running = false;
            v.error = result.err().map(|e| format!("{e:#}"));
            v.stop_error = stop.err().map(|e| format!("{e:#}"));
            v.phase = if v.error.is_some() || v.stop_error.is_some() {
                "error"
            } else {
                "ready"
            }
            .into();
        }));
        Ok(self.view.lock().unwrap().clone())
    }

    pub async fn stop(&self) -> Result<DemoView> {
        self.cancel.store(true, Ordering::Release);
        let mut runtime = self.runtime.lock().await;
        self.cancel.store(true, Ordering::Release);
        if let Some(job) = runtime.job.take() {
            if let Err(error) = job.await {
                // A worker panic must not skip either motor's disable.
                self.view.lock().unwrap().error = Some(format!("Demo worker failed: {error}"));
            }
        }
        self.view.lock().unwrap().running = false;
        if let Some(session) = &runtime.session {
            let result = session.disable().await;
            let mut v = self.view.lock().unwrap();
            session.telemetry(&mut v);
            v.meow.target_rpm = 0.0;
            v.dm.target_rpm = 0.0;
            v.stop_error = result.as_ref().err().map(|e| e.to_string());
            v.phase = if result.is_ok() { "ready" } else { "error" }.into();
            result?;
        }
        Ok(self.view.lock().unwrap().clone())
    }

    pub async fn disconnect(&self) -> Result<()> {
        self.cancel.store(true, Ordering::Release);
        // Keep this lock through stop AND release; a concurrent Start cannot
        // insert a new worker after disable and before dropping the sessions.
        let mut runtime = self.runtime.lock().await;
        self.cancel.store(true, Ordering::Release);
        if let Some(job) = runtime.job.take() {
            if let Err(error) = job.await {
                log::error!("Demo worker failed before disconnect: {error}");
            }
        }
        self.view.lock().unwrap().running = false;
        if let Some(session) = &runtime.session {
            if let Err(e) = session.disable().await {
                self.view.lock().unwrap().stop_error = Some(e.to_string());
                return Err(e);
            }
            session.dm.shutdown().await?;
        }
        runtime.session.take();
        *self.view.lock().unwrap() = DemoView::default();
        Ok(())
    }

    pub async fn close(&self) {
        self.closing.store(true, Ordering::Release);
        if let Err(e) = self.disconnect().await {
            log::error!("Dual motor demo close: {e:#}");
        }
    }
}

async fn run_motion(
    session: &Session,
    motion: DemoMotion,
    cancel: &AtomicBool,
    view: &StdMutex<DemoView>,
) -> Result<()> {
    if cancel.load(Ordering::Acquire) {
        return Ok(());
    }
    if !session.meow.status(session.nid)?.lifecycle.is_ready() {
        session
            .meow
            .initialize(session.nid, PdoProfile::from_event_timer_ms(10)?)
            .await
            .context("Initialize hexmeow")?;
    }
    if cancel.load(Ordering::Acquire) {
        return Ok(());
    }
    session.meow.set_max_torque(session.nid, 150).await?;
    let meow_position = session
        .meow
        .status(session.nid)?
        .measurements
        .position
        .context("hexmeow position feedback is unavailable")?
        .to_revolutions()
        * TAU;
    let factor = session
        .meow
        .list()
        .into_iter()
        .find(|m| m.node_id == session.nid)
        .and_then(|m| m.mit_kp_kd_factor)
        .context("hexmeow MIT gain factor was not read")?;
    let kp = meow_gain(motion.meow_kp, factor)?;
    let kd = meow_gain(motion.meow_kd, factor)?;
    session.dm.disable().await?;
    let dm = session.dm.read_positions().await?;
    let dm_position = f64::from(
        dm.position_rad
            .context("DAMIAO precise position is unavailable")?,
    );
    if !dm_position.is_finite() {
        bail!("DAMIAO position is not finite");
    }
    // Continuous rotation may have accumulated far beyond the 16-bit MIT
    // mapping. Expand that volatile scale to include the current center;
    // never reinterpret its wrapped feedback as the true absolute position.
    let required_pmax = (dm_position.abs() + motion.amplitude_deg.to_radians() + 1.0).ceil();
    if required_pmax > f64::from(dm.p_max) {
        if required_pmax > 10_000.0 {
            bail!("DAMIAO multi-turn position is too large for this demo");
        }
        *session.original_dm_pmax.lock().await = Some(dm.p_max);
        session
            .dm
            .write_register(0x15, required_pmax)
            .await
            .context("Set temporary DAMIAO MIT position range")?;
    }
    {
        let mut v = view.lock().unwrap();
        v.meow.center_rad = Some(meow_position);
        v.dm.center_rad = Some(dm_position);
        v.meow.target_deg = 0.0;
        v.dm.target_deg = 0.0;
    }
    let meow_target = |position: f64, velocity: f64| {
        MeowMotorTarget::Mit(MeowMitTarget {
            position_rev: ((meow_position + position) / TAU) as f32,
            velocity_rev_per_s: (velocity / TAU) as f32,
            torque_nm: 0.0,
            kp,
            kd,
            kp_kd_limit_permille: 150,
        })
    };
    if cancel.load(Ordering::Acquire) {
        return Ok(());
    }
    session
        .dm
        .enable()
        .await
        .context("Enable DAMIAO MIT mode")?;
    if cancel.load(Ordering::Acquire) {
        return Ok(());
    }
    session
        .meow
        .set_mode_sdo(session.nid, meow_target(0.0, 0.0))
        .await
        .context("Enable hexmeow MIT mode")?;
    let started = Instant::now();
    let mut tick = tokio::time::interval(TICK);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    while !cancel.load(Ordering::Acquire) {
        tick.tick().await;
        if cancel.load(Ordering::Acquire) {
            break;
        }
        let elapsed = started.elapsed().as_secs_f64();
        let (position, velocity) = motion.target_at(elapsed);
        let direction = if motion.dm_reverse { -1.0 } else { 1.0 };
        let meow = session.meow.status(session.nid)?;
        let dm = session.dm.snapshot();
        if !meow.connection.online
            || !meow.lifecycle.is_ready()
            || meow.logic != Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit))
            || meow
                .connection
                .last_tpdo1
                .is_none_or(|t| t.elapsed() > Duration::from_millis(500))
        {
            bail!(
                "hexmeow feedback lost or motor fault: {:?} / {:?}",
                meow.lifecycle,
                meow.logic
            );
        }
        if !dm.online || !dm.enabled || dm.status_code != 1 || dm.mode != DamiaoMode::Mit {
            bail!("DAMIAO feedback lost or motor fault: {}", dm.status);
        }
        let (a, b) = tokio::join!(
            session
                .meow
                .set_target_sdo(session.nid, meow_target(position, velocity)),
            session.dm.send_target(
                DamiaoTarget::Mit {
                    position_rad: (dm_position + direction * position) as f32,
                    velocity_rad_s: (direction * velocity) as f32,
                    torque_nm: 0.0,
                    kp: motion.dm_kp as f32,
                    kd: motion.dm_kd as f32,
                },
                false
            )
        );
        a.context("hexmeow target")?;
        b.context("DAMIAO target")?;
        let mut v = view.lock().unwrap();
        v.phase = "running".into();
        v.elapsed_s = elapsed;
        v.meow.target_rpm = velocity * 60.0 / TAU;
        v.dm.target_rpm = direction * velocity * 60.0 / TAU;
        v.meow.target_deg = position.to_degrees();
        v.dm.target_deg = direction * position.to_degrees();
        session.telemetry(&mut v);
    }
    Ok(())
}

#[tauri::command]
pub async fn dual_demo_connect(
    state: tauri::State<'_, crate::state::AppState>,
    gate: tauri::State<'_, CanTransportGate>,
    config: DemoConnection,
) -> Result<DemoView, String> {
    state
        .dual_demo
        .connect(&gate, config)
        .await
        .map_err(|e| format!("{e:#}"))
}
#[tauri::command]
pub async fn dual_demo_start(
    state: tauri::State<'_, crate::state::AppState>,
    motion: DemoMotion,
) -> Result<DemoView, String> {
    state
        .dual_demo
        .start(motion)
        .await
        .map_err(|e| format!("{e:#}"))
}
#[tauri::command]
pub async fn dual_demo_get(
    state: tauri::State<'_, crate::state::AppState>,
) -> Result<DemoView, String> {
    Ok(state.dual_demo.snapshot().await)
}
#[tauri::command]
pub async fn dual_demo_stop(
    state: tauri::State<'_, crate::state::AppState>,
) -> Result<DemoView, String> {
    state.dual_demo.stop().await.map_err(|e| format!("{e:#}"))
}
#[tauri::command]
pub async fn dual_demo_disconnect(
    state: tauri::State<'_, crate::state::AppState>,
) -> Result<(), String> {
    state
        .dual_demo
        .disconnect()
        .await
        .map_err(|e| format!("{e:#}"))
}

/// Attended hardware check using the exact GUI runtime; always stops both
/// motors after the bounded run, including startup and telemetry errors.
pub async fn run_bench(seconds: u64, mirror: bool) -> Result<()> {
    let app = DualMotorDemo::default();
    let gate = CanTransportGate::default();
    println!(
        "{}",
        serde_json::to_string(&app.connect(&gate, DemoConnection::default()).await?)?
    );
    let result = async {
        app.start(DemoMotion {
            dm_reverse: mirror,
            ..DemoMotion::default()
        })
        .await?;
        let deadline = Instant::now() + Duration::from_secs(seconds.min(120));
        loop {
            tokio::time::sleep(Duration::from_millis(500)).await;
            let view = app.snapshot().await;
            println!("{}", serde_json::to_string(&view)?);
            if let Some(e) = view.error {
                bail!("{e}");
            }
            if Instant::now() >= deadline {
                break;
            }
        }
        Ok(())
    }
    .await;
    let stop = app.stop().await;
    tokio::time::sleep(Duration::from_millis(500)).await;
    println!("STOP {}", serde_json::to_string(&app.snapshot().await)?);
    let disconnect = app.disconnect().await;
    result?;
    stop?;
    disconnect?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pendulum_starts_at_rest_stays_in_range_and_has_consistent_velocity() {
        let m = DemoMotion::default();
        assert_eq!(m.target_at(0.0), (0.0, 0.0));
        let mut low = 0.0f64;
        let mut high = 0.0f64;
        for i in 1..4000 {
            let t = f64::from(i) * 0.01;
            let (p, v) = m.target_at(t);
            assert!(p.abs() <= m.amplitude_deg.to_radians() + 1e-8);
            let derivative = (m.target_at(t + 1e-5).0 - m.target_at(t - 1e-5).0) / 2e-5;
            assert!((derivative - v).abs() < 1e-4);
            low = low.min(p);
            high = high.max(p);
        }
        assert!(low.to_degrees() < -179.9 && high.to_degrees() > 179.9);
    }
    #[test]
    fn invalid_settings_are_rejected_before_any_io() {
        assert!(DemoMotion {
            amplitude_deg: f64::NAN,
            ..DemoMotion::default()
        }
        .validate()
        .is_err());
        assert!(DemoMotion {
            amplitude_deg: 181.0,
            ..DemoMotion::default()
        }
        .validate()
        .is_err());
        assert!(DemoMotion {
            period_s: 0.0,
            ..DemoMotion::default()
        }
        .validate()
        .is_err());
        assert!(DemoConnection {
            meow_node_id: Some(10),
            ..DemoConnection::default()
        }
        .validate()
        .is_err());
    }

    #[test]
    fn si_gains_use_the_motor_factor_and_reject_overflow() {
        assert_eq!(meow_gain(3.0, 0.01).unwrap(), 1885);
        assert_eq!(meow_gain(0.15, 0.01).unwrap(), 94);
        assert!(meow_gain(3.0, 0.0).is_err());
        assert!(meow_gain(3.0, 0.00001).is_err());
    }

    #[tokio::test]
    async fn failed_worker_does_not_skip_disconnect_cleanup() {
        let app = DualMotorDemo::default();
        app.runtime.lock().await.job =
            Some(tokio::spawn(async { panic!("simulated worker failure") }));
        app.view.lock().unwrap().running = true;
        app.disconnect().await.unwrap();
        assert!(!app.snapshot().await.running);
        assert!(app.runtime.lock().await.job.is_none());
    }
    #[tokio::test]
    async fn disconnected_start_is_rejected_and_stop_is_idempotent() {
        let app = DualMotorDemo::default();
        assert!(app.start(DemoMotion::default()).await.is_err());
        assert!(!app.stop().await.unwrap().running);
        app.disconnect().await.unwrap();
        app.close().await;
        assert!(app
            .connect(&CanTransportGate::default(), DemoConnection::default())
            .await
            .is_err());
    }
}
