//! string_chassis main follow state machine driving the real HopeA3 HexMeow base.
//! The UI never supplies live sensor values or decides when motion is permitted.
use super::*;
use std::sync::Mutex as StdMutex;
use std::time::Instant;

use hex_motor::meow_motor::{MeowMotorManager, MeowMotorManagerOptions};
use mhf_pull_cord::chassis_model::{
    ChassisConfig, CordExitSide, FollowInput, FollowParameters, FollowStatus,
    InputSource, WalkMode,
};
use mhf_pull_cord::motion::{read_motion, CordMotionReceiver};
use tokio::task::JoinHandle;

use crate::hopea3::{Hopea3, Hopea3State};

#[path = "pull_chassis_control.rs"]
mod control;
use control::ChassisFollowController;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PaceMotionMode {
    #[default]
    Steering,
    Omni,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PullChassisConfig {
    pub follow: FollowParameters,
    pub mode: WalkMode,
    #[serde(default)]
    pub pace_motion: PaceMotionMode,
    pub source: InputSource,
    pub cord_exit_side: CordExitSide,
    pub direction_inverted: bool,
    pub max_linear_mps: f64,
    pub max_angular_rad_s: f64,
    pub acceleration_mps2: f64,
    /// None follows the pull-cord run's applied retraction target.
    #[serde(default)]
    pub retract_brake_speed_mps: Option<f64>,
    /// Body yaw per unit travel speed and sine of cord deflection (1/m).
    pub turn_gain: f64,
    pub kd_si: [f64; 3],
    pub max_torque_permille: [u16; 3],
}

impl Default for PullChassisConfig {
    fn default() -> Self {
        let config: Self = serde_json::from_str(include_str!("../defaults/pull-chassis.json"))
            .expect("embedded HopeA3 chassis configuration");
        config
            .validate()
            .expect("valid embedded HopeA3 chassis configuration");
        config
    }
}

impl PullChassisConfig {
    fn model(&self) -> ChassisConfig {
        // These defaults supply only model validation; no DAMIAO command or
        // motor ID from the upstream single-steering hardware is executed.
        ChassisConfig {
            follow: self.follow,
            mode: self.mode,
            source: self.source,
            cord_exit_side: self.cord_exit_side,
            wheel_diameter_m: 0.154,
            wheelbase_m: 0.486,
            gear_ratio: 1.0,
            max_wheel_speed_mps: self.max_linear_mps,
            acceleration_mps2: self.acceleration_mps2,
            ..ChassisConfig::default()
        }
    }

    fn validate(&self) -> Result<(), String> {
        self.model().validate()?;
        if self
            .retract_brake_speed_mps
            .is_some_and(|speed| !speed.is_finite() || speed <= 0.0)
        {
            return Err("回收制动速度必须为正数，或使用收线目标速度".into());
        }
        if !self.max_angular_rad_s.is_finite()
            || !(0.0..=3.0).contains(&self.max_angular_rad_s)
            || !self.turn_gain.is_finite()
            || !(0.0..=10.0).contains(&self.turn_gain)
            || self.kd_si.iter().any(|v| !v.is_finite() || *v < 0.0)
            || self.max_torque_permille.iter().any(|v| *v > 1000)
        {
            return Err("invalid HopeA3 steering, KD or torque limits".into());
        }
        Ok(())
    }

    fn max_deceleration_mps2(&self) -> f64 {
        let behind = self.follow.behind_speed;
        [
            self.follow.pace.deceleration_mps2,
            behind.deceleration_mps2,
            behind.stop_deceleration_mps2,
            behind.switch_deceleration_mps2,
            behind.adaptive_deceleration_mps2,
            behind.payout_stop_deceleration_mps2,
            behind.pull_back_deceleration_mps2,
        ]
        .into_iter()
        .fold(0.0, f64::max)
    }

    fn retract_brake_limit(&self, applied_retract_target_mps: f64) -> f64 {
        self.retract_brake_speed_mps.unwrap_or(applied_retract_target_mps)
    }

    fn structural_change(&self, next: &Self) -> bool {
        self.source != next.source
            || self.mode != next.mode
            || self.pace_motion != next.pace_motion
            || self.cord_exit_side != next.cord_exit_side
            || self.direction_inverted != next.direction_inverted
    }
}

/// Positive encoder deflection is clockwise; HopeA3 +wz is counterclockwise.
/// Keep the upstream longitudinal speed, turn the body, and command no strafe.
fn body_command(speed: f64, encoder_error_deg: f64, config: &PullChassisConfig) -> (f64, f64, f64) {
    let angle = if encoder_error_deg.abs() <= config.follow.direction_deadband_deg {
        0.0
    } else {
        encoder_error_deg.to_radians()
    };
    let polarity = if config.direction_inverted { 1.0 } else { -1.0 };
    let yaw = (polarity * speed * angle.sin() * config.turn_gain)
        .clamp(-config.max_angular_rad_s, config.max_angular_rad_s);
    (speed, 0.0, yaw)
}

#[derive(Clone, Serialize)]
pub struct PullChassisRunState {
    pub running: bool,
    pub fault: Option<String>,
    pub stop_confirmed: bool,
    pub config: PullChassisConfig,
    pub input: FollowInput,
    pub manual_enabled: bool,
    pub follow_status: FollowStatus,
    pub lateral_pace_active: bool,
    pub retract_braking: bool,
    pub retract_brake_speed_mps: Option<f64>,
    pub retract_brake_deceleration_mps2: f64,
    pub chassis: Hopea3State,
}

#[derive(Serialize)]
pub struct ChassisStatus {
    pub connected: bool,
    pub iface: Option<String>,
    pub needs_stop: bool,
    pub run: Option<PullChassisRunState>,
}

#[derive(Default)]
pub(super) struct ChassisConnection {
    pub(super) iface: Option<String>,
    bus: Option<Arc<dyn CanBus>>,
    manager: Option<Arc<MeowMotorManager>>,
    run: Option<FollowRun>,
    last_state: Option<PullChassisRunState>,
    pending_start: bool,
}

impl ChassisConnection {
    pub(super) fn status(&self) -> ChassisStatus {
        ChassisStatus {
            connected: self.bus.is_some(),
            iface: self.iface.clone(),
            needs_stop: self.run.is_some() || self.pending_start,
            run: self
                .run
                .as_ref()
                .map(FollowRun::snapshot)
                .or_else(|| self.last_state.clone()),
        }
    }

    pub(super) async fn stop(&mut self) -> Result<(), String> {
        if self.pending_start {
            let manager = self.manager.as_ref().ok_or("底盘启动恢复缺少 CAN 会话")?;
            let nodes: Vec<u8> = manager
                .list()
                .iter()
                .filter(|info| {
                    crate::hopea3::NODE_IDS.contains(&info.node_id) && info.model.is_some()
                })
                .map(|info| info.node_id)
                .collect();
            crate::hopea3::disable_motors(manager, &nodes)
                .await
                .map_err(|e| format!("{e:#}"))?;
            self.pending_start = false;
            if let Some(state) = &mut self.last_state {
                state.stop_confirmed = true;
            }
        }
        if let Some(run) = self.run.as_mut() {
            run.stop().await?;
            self.last_state = Some(run.snapshot());
            self.run = None;
        }
        Ok(())
    }

    pub(super) async fn disconnect(&mut self) -> Result<(), String> {
        self.stop().await?;
        self.manager = None;
        self.bus = None;
        self.iface = None;
        Ok(())
    }
}

pub(super) fn check_distinct_interfaces(
    existing: Option<&str>,
    requested: &str,
) -> Result<(), String> {
    fn key(value: &str) -> String {
        if let Some(channel) = crate::backend::gs_usb_channel(value) {
            return format!("gs_usb{channel}");
        }
        let value = value
            .trim()
            .strip_prefix("socketcan:")
            .unwrap_or(value.trim());
        value.to_string()
    }
    if requested.trim().is_empty() || existing.is_some_and(|value| key(value) == key(requested)) {
        return Err("拉线经典 CAN 与 HopeA3 CAN-FD 必须使用不同接口".into());
    }
    Ok(())
}

struct LiveInput {
    motion: CordMotionReceiver,
    trajectory: CordTrajectory,
    epoch: u64,
    encoder: PullCordEncoder,
}

impl LiveInput {
    async fn sample(&self) -> Result<(FollowInput, f64), String> {
        let now = Instant::now();
        let motion = read_motion(&self.motion, now)?;
        if motion.invalidation_epoch != self.epoch {
            return Err("拉线会话已中断或重新归零，请重新启动底盘跟随".into());
        }
        let encoder = self
            .encoder
            .monitor
            .lock()
            .await
            .as_ref()
            .map(EncoderMonitor::snapshot)
            .ok_or("方向编码器已断开")?;
        if !encoder.angle_is_usable()
            || encoder
                .position_age
                .is_none_or(|age| age > Duration::from_millis(150))
        {
            return Err("方向编码器磁铁状态异常或角度反馈过期".into());
        }
        let input = FollowInput {
            cord_length_m: motion.length_m,
            cord_speed_mps: motion.speed_m_s,
            pull_back_speed_mps: motion.speed_m_s.map(|speed| {
                if motion.retract_positive {
                    -speed
                } else {
                    speed
                }
            }),
            encoder_angle_deg: encoder.position.map(|position| position.angle_degrees),
            encoder_sample_time_s: encoder
                .position_age
                .map(|age| encoder.monitor_age.saturating_sub(age).as_secs_f64()),
        };
        Ok((input, motion.retract_target_speed_m_s))
    }
}

struct FollowRun {
    driver: Arc<Mutex<Hopea3>>,
    config: Arc<StdMutex<PullChassisConfig>>,
    manual: Arc<StdMutex<(bool, FollowInput)>>,
    state: Arc<StdMutex<PullChassisRunState>>,
    cancel: Arc<AtomicBool>,
    task: Option<JoinHandle<()>>,
}

impl FollowRun {
    fn snapshot(&self) -> PullChassisRunState {
        self.state.lock().unwrap().clone()
    }

    async fn set_config(&self, next: PullChassisConfig) -> Result<(), String> {
        next.validate()?;
        if self.config.lock().unwrap().structural_change(&next) {
            return Err("请先停机，再更改输入来源、跟随模式、步速同步方式、出线侧或方向反相".into());
        }
        if !self.snapshot().running {
            return Err("底盘已停止，请重新启动".into());
        }
        let driver = self.driver.lock().await;
        apply_limits(&driver, &next)?;
        *self.config.lock().unwrap() = next.clone();
        self.state.lock().unwrap().config = next;
        Ok(())
    }

    fn set_manual(&self, enabled: bool, input: FollowInput) -> Result<(), String> {
        if self.config.lock().unwrap().source != InputSource::Manual || !self.snapshot().running {
            return Err("请先使能手动台架".into());
        }
        if !input.valid() || input.cord_length_m.is_some_and(|length| length < 0.0) {
            return Err("手动线长、线速或角度无效".into());
        }
        *self.manual.lock().unwrap() = (enabled, input);
        Ok(())
    }

    async fn stop(&mut self) -> Result<(), String> {
        self.cancel.store(true, Ordering::Release);
        if let Some(task) = self.task.take() {
            if let Err(error) = task.await {
                self.state.lock().unwrap().fault = Some(format!("follow task failed: {error}"));
            }
        }
        if self.snapshot().stop_confirmed {
            return Ok(());
        }
        let result = self
            .driver
            .lock()
            .await
            .stop()
            .await
            .map_err(|error| format!("{error:#}"));
        let mut state = self.state.lock().unwrap();
        state.running = false;
        state.stop_confirmed = result.is_ok();
        if let Err(error) = &result {
            state.fault = Some(error.clone());
        }
        result
    }
}

impl Drop for FollowRun {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Release);
    }
}

fn apply_limits(driver: &Hopea3, config: &PullChassisConfig) -> Result<(), String> {
    driver
        .set_kd(config.kd_si)
        .map_err(|error| error.to_string())?;
    driver.set_max_torque(config.max_torque_permille);
    driver.set_limits(config.max_linear_mps, config.max_angular_rad_s);
    // The follow controller ramps translation, including diagonal motion and braking.
    driver.set_accel_limits(0.0, 6.0);
    Ok(())
}

async fn follow_loop(
    driver: Arc<Mutex<Hopea3>>,
    config: Arc<StdMutex<PullChassisConfig>>,
    manual: Arc<StdMutex<(bool, FollowInput)>>,
    state: Arc<StdMutex<PullChassisRunState>>,
    cancel: Arc<AtomicBool>,
    live: Option<LiveInput>,
) {
    let mut controller = ChassisFollowController::default();
    let mut last_tick = Instant::now();
    let started = last_tick;
    let mut tick = tokio::time::interval(Duration::from_millis(10));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let result: Result<(), String> = async {
        loop {
            tick.tick().await;
            if cancel.load(Ordering::Acquire) {
                break;
            }
            let now = Instant::now();
            let dt = now.duration_since(last_tick).as_secs_f64().max(1e-6);
            last_tick = now;
            if dt > 0.15 {
                return Err("底盘跟随循环超过 150 ms 未更新".into());
            }
            let (enabled, input, retract_target) = if let Some(live) = &live {
                let (input, retract_target) = live.sample().await?;
                (true, input, Some(retract_target))
            } else {
                let (enabled, input) = *manual.lock().unwrap();
                (enabled, input, None)
            };
            let driver = driver.lock().await;
            let config = config.lock().unwrap().clone();
            let retract_brake_speed_mps =
                retract_target.map(|speed| config.retract_brake_limit(speed));
            let chassis = driver.state();
            if let Some(fault) = &chassis.fault {
                return Err(fault.clone());
            }
            if !chassis.running {
                return Err("HopeA3 电机控制已停止".into());
            }
            if chassis.motors.len() != 3 {
                if started.elapsed() > Duration::from_millis(150) {
                    return Err("HopeA3 尚未提供完整电机反馈".into());
                }
                driver.set_follow_cmd(0.0, 0.0, 0.0);
                continue;
            }
            let command = if enabled {
                let endpoint = live
                    .as_ref()
                    .filter(|_| config.pace_motion == PaceMotionMode::Omni)
                    .and_then(|live| live.trajectory.latest());
                controller.step(input, endpoint, &chassis, dt, &config, retract_brake_speed_mps)
            } else {
                controller.reset();
                (0.0, 0.0, 0.0)
            };
            if cancel.load(Ordering::Acquire) {
                break;
            }
            // Recheck the exact cord publication after awaits, before handing
            // targets to the 500 Hz wheel loop.
            if let Some(live) = &live {
                let motion = read_motion(&live.motion, Instant::now())?;
                if motion.invalidation_epoch != live.epoch {
                    return Err("拉线会话已中断".into());
                }
            }
            driver.set_follow_cmd(command.0, command.1, command.2);
            let mut state = state.lock().unwrap();
            state.input = input;
            state.manual_enabled = enabled && config.source == InputSource::Manual;
            state.follow_status = controller.status();
            state.lateral_pace_active = controller.lateral_pace_active();
            state.retract_braking = controller.retract_braking;
            state.retract_brake_speed_mps = retract_brake_speed_mps;
            state.retract_brake_deceleration_mps2 = config.max_deceleration_mps2();
            state.chassis = chassis;
        }
        Ok(())
    }
    .await;
    let mut driver = driver.lock().await;
    driver.set_follow_cmd(0.0, 0.0, 0.0);
    let stopped = driver.stop().await;
    let mut state = state.lock().unwrap();
    state.running = false;
    state.lateral_pace_active = false;
    state.manual_enabled = false;
    state.chassis = driver.state();
    state.stop_confirmed = stopped.is_ok();
    state.fault = match (result, stopped) {
        (Ok(()), Ok(())) => state.fault.clone(),
        (Err(error), Ok(())) => Some(error),
        (Ok(()), Err(error)) => Some(format!("停机未确认: {error:#}")),
        (Err(error), Err(stop)) => Some(format!("{error}; 停机未确认: {stop:#}")),
    };
}

impl PullCordApp {
    pub(super) fn request_chassis_stop(&self) {
        self.chassis_stop_generation.fetch_add(1, Ordering::AcqRel);
        self.chassis_cancel.store(true, Ordering::Release);
    }
}

#[tauri::command]
pub async fn pull_chassis_connect(
    state: State<'_, AppState>,
    gate: State<'_, CanTransportGate>,
    iface: String,
    data_bitrate: u32,
) -> Result<(), String> {
    let _operation = state.extension_op.lock().await;
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("application is closing".into());
    }
    let mut runtime = state.pull_cord.runtime.lock().await;
    if runtime.chassis.bus.is_some() {
        return Err("底盘 CAN 已连接".into());
    }
    check_distinct_interfaces(runtime.iface.as_deref(), &iface)?;
    state
        .extension_runtime_active
        .store(true, Ordering::Release);
    let lease = gate.try_acquire_pull_channel()?;
    let (bus, _) = crate::backend::open_bus(iface.trim(), data_bitrate, false, lease)
        .await
        .map_err(|error| error.to_string())?;
    let bus = crate::backend::protect_feature_bus(bus);
    if let Some(link) = bus.link_config().await.map_err(|e| e.to_string())? {
        if link.fd_enabled == Some(false)
            || link
                .nominal
                .and_then(|timing| timing.bitrate)
                .is_some_and(|rate| rate != 1_000_000)
            || link
                .data
                .and_then(|timing| timing.bitrate)
                .is_some_and(|rate| rate != data_bitrate)
        {
            return Err(
                "HopeA3 需要 CAN-FD：仲裁段 1 Mbps，数据段与界面选择一致；请先配置接口".into(),
            );
        }
    }
    let manager = Arc::new(
        MeowMotorManager::new(
            bus.clone(),
            MeowMotorManagerOptions {
                heartbeat_node_id: 10,
                broadcast_heartbeat: true,
                auto_identify: false,
                sdo_timeout: Duration::from_millis(500),
                ..Default::default()
            },
        )
        .map_err(|error| error.to_string())?,
    );
    runtime.chassis.bus = Some(bus);
    runtime.chassis.manager = Some(manager);
    runtime.chassis.iface = Some(iface.trim().to_string());
    Ok(())
}

#[tauri::command]
pub async fn pull_chassis_disconnect(state: State<'_, AppState>) -> Result<(), String> {
    state.pull_cord.request_chassis_stop();
    let _operation = state.extension_op.lock().await;
    state
        .pull_cord
        .runtime
        .lock()
        .await
        .chassis
        .disconnect()
        .await
}

#[tauri::command]
pub async fn pull_chassis_start(
    state: State<'_, AppState>,
    config: PullChassisConfig,
) -> Result<(), String> {
    config.validate()?;
    let generation = state
        .pull_cord
        .chassis_stop_generation
        .load(Ordering::Acquire);
    let _operation = state.extension_op.lock().await;
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("application is closing".into());
    }
    let mut runtime = state.pull_cord.runtime.lock().await;
    if runtime.chassis.run.is_some() || runtime.chassis.pending_start {
        return Err("请先停止并确认上一底盘会话".into());
    }
    let manager = runtime
        .chassis
        .manager
        .clone()
        .ok_or("请先连接底盘 CAN-FD")?;
    let bus = runtime.chassis.bus.clone().ok_or("请先连接底盘 CAN-FD")?;
    let live = if config.source == InputSource::Live {
        let cord = runtime.control.as_ref().ok_or("请先启动拉线并完成归零")?;
        if !cord.snapshot().feedback_ready_for_chassis() {
            return Err("拉线尚未完成归零或存在故障".into());
        }
        let motion = cord.subscribe_motion();
        let epoch = read_motion(&motion, Instant::now())?.invalidation_epoch;
        let node = runtime
            .encoder
            .monitor
            .lock()
            .await
            .as_ref()
            .map(|m| m.snapshot().node_id)
            .ok_or("方向编码器未连接")?;
        let product = sdo::upload_u32(
            runtime.bus.as_ref().ok_or("拉线 CAN 未连接")?.as_ref(),
            node,
            0x1018,
            2,
            Some(Duration::from_millis(500)),
        )
        .await
        .map_err(|e| e.to_string())?;
        if product != magnetic_encoder::EXPECTED_PRODUCT_CODE {
            return Err("方向编码器产品身份不匹配".into());
        }
        let live = LiveInput {
            motion,
            trajectory: cord.trajectory(),
            epoch,
            encoder: runtime.encoder.clone(),
        };
        live.sample().await?;
        Some(live)
    } else {
        None
    };
    state
        .pull_cord
        .chassis_cancel
        .store(false, Ordering::Release);
    if generation
        != state
            .pull_cord
            .chassis_stop_generation
            .load(Ordering::Acquire)
    {
        state
            .pull_cord
            .chassis_cancel
            .store(true, Ordering::Release);
        return Err("底盘启动已取消".into());
    }
    runtime.chassis.pending_start = true;
    let driver = match Hopea3::start(
        manager,
        bus,
        Some(Duration::from_millis(500)),
        &state.hopea3_init,
        &state.pull_cord.chassis_cancel,
    )
    .await
    {
        Ok(driver) => driver,
        Err(error) => {
            let error = format!("{error:#}");
            runtime.chassis.last_state = Some(PullChassisRunState {
                running: false,
                fault: Some(error.clone()),
                stop_confirmed: false,
                retract_braking: false,
                retract_brake_speed_mps: None,
                retract_brake_deceleration_mps2: config.max_deceleration_mps2(),
                config,
                input: FollowInput::default(),
                manual_enabled: false,
                follow_status: FollowStatus::default(),
                lateral_pace_active: false,
                chassis: Hopea3State::default(),
            });
            return Err(error);
        }
    };
    runtime.chassis.pending_start = false;
    let initial = if let Some(live) = &live {
        live.sample().await.map(|(input, _)| input)
    } else {
        Ok(FollowInput {
            cord_length_m: Some(0.0),
            cord_speed_mps: Some(0.0),
            encoder_angle_deg: Some(config.follow.encoder_center_deg),
            ..Default::default()
        })
    };
    // Always install the owner before returning an error so failed disable
    // remains retryable. The worker immediately handles a cancelled startup.
    let initial_fault = initial
        .as_ref()
        .err()
        .cloned()
        .or_else(|| apply_limits(&driver, &config).err());
    let input = initial.unwrap_or_default();
    driver.set_follow_cmd(0.0, 0.0, 0.0);
    let state_view = Arc::new(StdMutex::new(PullChassisRunState {
        running: true,
        fault: initial_fault.clone(),
        stop_confirmed: false,
        config: config.clone(),
        input,
        manual_enabled: false,
        follow_status: FollowStatus::default(),
        lateral_pace_active: false,
        retract_braking: false,
        retract_brake_speed_mps: None,
        retract_brake_deceleration_mps2: config.max_deceleration_mps2(),
        chassis: driver.state(),
    }));
    if initial_fault.is_some() {
        state
            .pull_cord
            .chassis_cancel
            .store(true, Ordering::Release);
    }
    let driver = Arc::new(Mutex::new(driver));
    let config = Arc::new(StdMutex::new(config));
    let manual = Arc::new(StdMutex::new((false, input)));
    let cancel = state.pull_cord.chassis_cancel.clone();
    let task = tokio::spawn(follow_loop(
        driver.clone(),
        config.clone(),
        manual.clone(),
        state_view.clone(),
        cancel.clone(),
        live,
    ));
    *runtime.encoder.center_deg.lock().unwrap() = config.lock().unwrap().follow.encoder_center_deg;
    runtime.chassis.last_state = None;
    runtime.chassis.run = Some(FollowRun {
        driver,
        config,
        manual,
        state: state_view,
        cancel,
        task: Some(task),
    });
    if let Some(error) = initial_fault {
        return Err(error);
    }
    if state.pull_cord.chassis_cancel.load(Ordering::Acquire) {
        return Err("底盘启动已取消，请确认停机".into());
    }
    Ok(())
}

#[tauri::command]
pub async fn pull_chassis_stop(state: State<'_, AppState>) -> Result<(), String> {
    state.pull_cord.request_chassis_stop();
    let _operation = state.extension_op.lock().await;
    state.pull_cord.runtime.lock().await.chassis.stop().await
}

#[tauri::command]
pub async fn pull_chassis_set_manual(
    state: State<'_, AppState>,
    enabled: bool,
    input: FollowInput,
) -> Result<(), String> {
    let runtime = state.pull_cord.runtime.lock().await;
    runtime
        .chassis
        .run
        .as_ref()
        .ok_or("底盘尚未启动")?
        .set_manual(enabled, input)
}

#[tauri::command]
pub async fn pull_chassis_set_config(
    state: State<'_, AppState>,
    config: PullChassisConfig,
) -> Result<(), String> {
    let runtime = state.pull_cord.runtime.lock().await;
    runtime
        .chassis
        .run
        .as_ref()
        .ok_or("底盘尚未启动")?
        .set_config(config.clone())
        .await?;
    *runtime.encoder.center_deg.lock().unwrap() = config.follow.encoder_center_deg;
    Ok(())
}

#[tauri::command]
pub fn pull_chassis_default_config() -> PullChassisConfig {
    PullChassisConfig::default()
}

#[tauri::command]
pub async fn pull_chassis_settings_get(
    app: tauri::AppHandle,
    state: State<'_, AppState>,
) -> Result<PullChassisConfig, String> {
    let _guard = state.pull_cord.settings_lock.lock().await;
    let path = app
        .path()
        .app_config_dir()
        .map_err(|e| e.to_string())?
        .join("pull-chassis.json");
    load_chassis_settings(&path)
}

fn load_chassis_settings(path: &Path) -> Result<PullChassisConfig, String> {
    let config = match std::fs::read(path) {
        Ok(bytes) => {
            serde_json::from_slice(&bytes).map_err(|e| format!("invalid chassis settings: {e}"))?
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => PullChassisConfig::default(),
        Err(error) => return Err(error.to_string()),
    };
    config.validate()?;
    Ok(config)
}

#[tauri::command]
pub async fn pull_chassis_settings_save(
    app: tauri::AppHandle,
    state: State<'_, AppState>,
    config: PullChassisConfig,
) -> Result<(), String> {
    use std::io::Write;
    config.validate()?;
    let _guard = state.pull_cord.settings_lock.lock().await;
    let path = app
        .path()
        .app_config_dir()
        .map_err(|e| e.to_string())?
        .join("pull-chassis.json");
    std::fs::create_dir_all(path.parent().unwrap()).map_err(|e| e.to_string())?;
    let temporary = path.with_extension("json.tmp");
    let bytes = serde_json::to_vec_pretty(&config).map_err(|e| e.to_string())?;
    let mut file = std::fs::File::create(&temporary).map_err(|e| e.to_string())?;
    file.write_all(&bytes)
        .and_then(|_| file.sync_all())
        .map_err(|e| e.to_string())?;
    drop(file);
    std::fs::rename(temporary, path).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chassis_settings_use_build_defaults_only_without_local_settings() {
        let dir =
            std::env::temp_dir().join(format!("hex-gui-chassis-defaults-{}", std::process::id()));
        let path = dir.join("pull-chassis.json");
        let config = load_chassis_settings(&path).unwrap();
        let expected: serde_json::Value =
            serde_json::from_str(include_str!("../defaults/pull-chassis.json")).unwrap();
        assert_eq!(serde_json::to_value(&config).unwrap(), expected);
        let mut saved = config;
        saved.follow.pace.target_length_m = 1.1;
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(&path, serde_json::to_vec(&saved).unwrap()).unwrap();
        assert_eq!(
            load_chassis_settings(&path)
                .unwrap()
                .follow
                .pace
                .target_length_m,
            1.1
        );
        std::fs::write(&path, b"invalid").unwrap();
        assert!(load_chassis_settings(&path).is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn pull_chassis_interface_aliases_cannot_open_one_channel_twice() {
        assert!(check_distinct_interfaces(Some("can0"), "socketcan:can0").is_err());
        assert!(check_distinct_interfaces(Some("gs_usb"), "gs_usb0").is_err());
        assert!(check_distinct_interfaces(Some("gsusb1"), "GS_USB:1").is_err());
        assert!(check_distinct_interfaces(Some("gs_usb0"), "gs_usb1").is_ok());
    }

    #[test]
    fn legacy_settings_keep_steering_and_omni_is_a_persisted_startup_choice() {
        let defaults = PullChassisConfig::default();
        let mut legacy = serde_json::to_value(&defaults).unwrap();
        legacy.as_object_mut().unwrap().remove("pace_motion");
        let mut config: PullChassisConfig = serde_json::from_value(legacy).unwrap();
        assert_eq!(config.pace_motion, PaceMotionMode::Steering);
        assert!(!config.structural_change(&defaults));
        config.pace_motion = PaceMotionMode::Omni;
        assert!(defaults.structural_change(&config));
        let saved = serde_json::to_vec(&config).unwrap();
        let restored: PullChassisConfig = serde_json::from_slice(&saved).unwrap();
        assert_eq!(restored.pace_motion, PaceMotionMode::Omni);
    }

    #[test]
    fn retract_brake_settings_migrate_validate_and_follow_the_applied_target() {
        let mut legacy = serde_json::to_value(PullChassisConfig::default()).unwrap();
        legacy.as_object_mut().unwrap().remove("retract_brake_speed_mps");
        let mut config: PullChassisConfig = serde_json::from_value(legacy).unwrap();
        config.validate().unwrap();
        assert_eq!(config.retract_brake_speed_mps, None);
        assert_eq!(config.retract_brake_limit(2.0), 2.0);
        assert_eq!(config.retract_brake_limit(1.5), 1.5);
        config.retract_brake_speed_mps = Some(0.8);
        config.validate().unwrap();
        assert_eq!(config.retract_brake_limit(2.0), 0.8);
        let restored: PullChassisConfig =
            serde_json::from_slice(&serde_json::to_vec(&config).unwrap()).unwrap();
        assert_eq!(restored.retract_brake_speed_mps, Some(0.8));
        assert!(!config.structural_change(&PullChassisConfig::default()));
        for speed in [0.0, -0.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            config.retract_brake_speed_mps = Some(speed);
            assert!(config.validate().is_err(), "invalid threshold {speed}");
        }
    }

    #[test]
    fn retract_brake_uses_the_largest_configured_deceleration() {
        let mut config = PullChassisConfig::default();
        assert_eq!(config.max_deceleration_mps2(), 3.0);
        // Even inactive legacy tuning remains part of the app's maximum.
        config.follow.behind_speed.payout_stop_enabled = false;
        config.follow.behind_speed.adaptive_enabled = false;
        config.acceleration_mps2 = 4.0;
        assert_eq!(config.max_deceleration_mps2(), 3.0);
        for index in 0..7 {
            let mut tuning = config.clone();
            let behind = &mut tuning.follow.behind_speed;
            let fields = [
                &mut tuning.follow.pace.deceleration_mps2,
                &mut behind.deceleration_mps2,
                &mut behind.stop_deceleration_mps2,
                &mut behind.switch_deceleration_mps2,
                &mut behind.adaptive_deceleration_mps2,
                &mut behind.payout_stop_deceleration_mps2,
                &mut behind.pull_back_deceleration_mps2,
            ];
            *fields.into_iter().nth(index).unwrap() = 3.5;
            assert_eq!(tuning.max_deceleration_mps2(), 3.5, "field {index}");
        }
    }

    #[test]
    fn pull_chassis_preserves_forward_and_reverse_without_strafing() {
        let config = PullChassisConfig::default();
        config.validate().unwrap();
        let forward = body_command(0.3, 30.0, &config);
        let reverse = body_command(-0.3, 30.0, &config);
        assert_eq!((forward.0, forward.1), (0.3, 0.0));
        assert!(forward.2 < 0.0 && reverse.2 > 0.0);
        assert_eq!(body_command(0.0, 90.0, &config), (0.0, 0.0, 0.0));
        assert!(body_command(3.0, 90.0, &config).2.abs() <= config.max_angular_rad_s);
        assert!(config.structural_change(&PullChassisConfig {
            source: InputSource::Manual,
            ..config.clone()
        }));
    }
}
