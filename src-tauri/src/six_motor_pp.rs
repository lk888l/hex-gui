//! Six-axis arm repeatability test using the Meow Motor Profile Position mode.
//!
//! Every motor is initialized and enabled at its freshly measured encoder
//! position so the arm is held without a startup step. During the test only one
//! motor receives a new PP target. The next motor is not commanded until fresh
//! position and velocity feedback for the active motor remain inside the
//! configured arrival window for the full settle time.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex as StdMutex, MutexGuard as StdMutexGuard};
use std::time::{Duration, Instant};

use hex_motor::cia402::PdoProfile;
use hex_motor::meow_motor::{
    MeowMotorLifecycle, MeowMotorLiveState, MeowMotorLogic, MeowMotorManager, MeowMotorMode,
    MeowMotorTarget, MeowProfileLimits, SignedQ8_24,
};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;
use tokio::task::JoinHandle;

pub const NODE_IDS: [u8; 6] = [1, 2, 3, 4, 5, 6];

const MOTOR_COUNT: usize = NODE_IDS.len();
const WAYPOINT_COUNT: usize = 4;
const LOOP_PERIOD: Duration = Duration::from_millis(20);
const POSITION_MIN_REV: f64 = -128.0;
const POSITION_MAX_REV_EXCLUSIVE: f64 = 128.0;

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SixMotorPpProfile {
    pub velocity_rev_per_s: f32,
    pub acceleration_rev_per_s2: f32,
    pub deceleration_rev_per_s2: f32,
}

impl From<SixMotorPpProfile> for MeowProfileLimits {
    fn from(profile: SixMotorPpProfile) -> Self {
        Self {
            velocity_rev_per_s: profile.velocity_rev_per_s,
            acceleration_rev_per_s2: profile.acceleration_rev_per_s2,
            deceleration_rev_per_s2: profile.deceleration_rev_per_s2,
        }
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SixMotorPpRequest {
    pub waypoints: Vec<[f64; MOTOR_COUNT]>,
    pub tpdo_event_timer_ms: u16,
    pub max_torque_permille: u16,
    pub profile: SixMotorPpProfile,
    pub arrival_position_tolerance_rev: f64,
    pub arrival_velocity_tolerance_rev_per_s: f64,
    pub settle_time_ms: u32,
    pub movement_timeout_ms: u32,
    pub waypoint_dwell_ms: u32,
}

impl SixMotorPpRequest {
    pub fn validate(&self) -> Result<(), String> {
        if self.waypoints.len() != WAYPOINT_COUNT {
            return Err(format!(
                "six-motor PP test requires exactly {WAYPOINT_COUNT} waypoints"
            ));
        }
        for (waypoint_index, waypoint) in self.waypoints.iter().enumerate() {
            for (motor_index, position) in waypoint.iter().enumerate() {
                if !position.is_finite()
                    || *position < POSITION_MIN_REV
                    || *position >= POSITION_MAX_REV_EXCLUSIVE
                {
                    return Err(format!(
                        "waypoint {}, motor {} position must be finite and in [{POSITION_MIN_REV}, {POSITION_MAX_REV_EXCLUSIVE}) Rev",
                        waypoint_index + 1,
                        NODE_IDS[motor_index]
                    ));
                }
            }
        }
        if !(1..=100).contains(&self.tpdo_event_timer_ms) {
            return Err("TPDO event timer must be 1..=100 ms".into());
        }
        if self.max_torque_permille > 1_000 {
            return Err("maximum torque must be 0..=1000 permille".into());
        }
        validate_positive_bounded(
            self.profile.velocity_rev_per_s,
            5.0,
            "profile velocity",
            "Rev/s",
        )?;
        validate_positive_bounded(
            self.profile.acceleration_rev_per_s2,
            50.0,
            "profile acceleration",
            "Rev/s^2",
        )?;
        validate_positive_bounded(
            self.profile.deceleration_rev_per_s2,
            50.0,
            "profile deceleration",
            "Rev/s^2",
        )?;
        validate_positive_bounded_f64(
            self.arrival_position_tolerance_rev,
            1.0,
            "arrival position tolerance",
            "Rev",
        )?;
        validate_positive_bounded_f64(
            self.arrival_velocity_tolerance_rev_per_s,
            5.0,
            "arrival velocity tolerance",
            "Rev/s",
        )?;
        if !(20..=5_000).contains(&self.settle_time_ms) {
            return Err("settle time must be 20..=5000 ms".into());
        }
        if !(1_000..=600_000).contains(&self.movement_timeout_ms) {
            return Err("per-motor movement timeout must be 1000..=600000 ms".into());
        }
        if self.waypoint_dwell_ms > 60_000 {
            return Err("waypoint dwell must be 0..=60000 ms".into());
        }
        Ok(())
    }
}

fn validate_positive_bounded(value: f32, max: f32, label: &str, unit: &str) -> Result<(), String> {
    if !value.is_finite() || value <= 0.0 || value > max {
        return Err(format!("{label} must be finite and in (0, {max}] {unit}"));
    }
    Ok(())
}

fn validate_positive_bounded_f64(
    value: f64,
    max: f64,
    label: &str,
    unit: &str,
) -> Result<(), String> {
    if !value.is_finite() || value <= 0.0 || value > max {
        return Err(format!("{label} must be finite and in (0, {max}] {unit}"));
    }
    Ok(())
}

#[derive(Debug, Clone, Serialize)]
pub struct SixMotorPpMotorView {
    pub node_id: u8,
    pub online: bool,
    pub enabled: bool,
    pub active: bool,
    pub sequence_state: String,
    pub target_position_rev: f64,
    pub actual_position_rev: Option<f64>,
    pub actual_velocity_rev_per_s: Option<f64>,
    pub position_error_rev: Option<f64>,
    pub feedback_age_ms: Option<u64>,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SixMotorPpView {
    pub running: bool,
    pub phase: String,
    pub active_waypoint_index: usize,
    pub active_motor_index: Option<usize>,
    pub completed_cycles: u64,
    pub pdo_rate_hz: f64,
    pub motors: Vec<SixMotorPpMotorView>,
    pub last_error: Option<String>,
}

impl SixMotorPpView {
    pub fn idle() -> Self {
        Self {
            running: false,
            phase: "idle".into(),
            active_waypoint_index: 0,
            active_motor_index: None,
            completed_cycles: 0,
            pdo_rate_hz: 0.0,
            motors: Vec::new(),
            last_error: None,
        }
    }
}

pub struct SixMotorPp {
    manager: Arc<MeowMotorManager>,
    request: SixMotorPpRequest,
    view: Arc<StdMutex<SixMotorPpView>>,
    running: Arc<AtomicBool>,
    task: Mutex<Option<JoinHandle<()>>>,
    shutdown_gate: Mutex<()>,
}

impl SixMotorPp {
    pub async fn start(
        manager: Arc<MeowMotorManager>,
        request: SixMotorPpRequest,
        shutdown_requested: &AtomicBool,
    ) -> anyhow::Result<Arc<Self>> {
        request.validate().map_err(anyhow::Error::msg)?;
        let profile = PdoProfile::from_event_timer_ms(request.tpdo_event_timer_ms)
            .map_err(|error| anyhow::anyhow!("build PP PDO profile: {error}"))?;
        let initial_positions = match setup_all(&manager, profile, &request, shutdown_requested)
            .await
        {
            Ok(positions) => positions,
            Err(error) => {
                let failures = disable_startup_candidates(&manager).await;
                if failures.is_empty() {
                    return Err(error);
                }
                anyhow::bail!(
                    "{error}; startup rollback could not confirm disable: {}. Remove motor power before handling the arm",
                    failures.join("; ")
                );
            }
        };

        let motors = NODE_IDS
            .iter()
            .enumerate()
            .map(|(index, &node_id)| SixMotorPpMotorView {
                node_id,
                online: true,
                enabled: true,
                active: index == 0,
                sequence_state: if index == 0 { "moving" } else { "waiting" }.into(),
                target_position_rev: request.waypoints[0][index],
                actual_position_rev: Some(initial_positions[index]),
                actual_velocity_rev_per_s: None,
                position_error_rev: Some(initial_positions[index] - request.waypoints[0][index]),
                feedback_age_ms: Some(0),
                error: None,
            })
            .collect();
        let view = SixMotorPpView {
            running: true,
            phase: "moving".into(),
            active_waypoint_index: 0,
            active_motor_index: Some(0),
            completed_cycles: 0,
            pdo_rate_hz: 1_000.0 / f64::from(request.tpdo_event_timer_ms),
            motors,
            last_error: None,
        };
        let app = Arc::new(Self {
            manager,
            request,
            view: Arc::new(StdMutex::new(view)),
            running: Arc::new(AtomicBool::new(true)),
            task: Mutex::new(None),
            shutdown_gate: Mutex::new(()),
        });
        let task_app = Arc::clone(&app);
        let task = tokio::spawn(async move {
            task_app.control_loop().await;
        });
        *app.task.lock().await = Some(task);
        Ok(app)
    }

    pub fn snapshot(&self) -> SixMotorPpView {
        lock_unpoisoned(&self.view).clone()
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Acquire)
    }

    pub fn begin_shutdown(&self) {
        self.running.store(false, Ordering::Release);
        let mut view = lock_unpoisoned(&self.view);
        view.running = false;
        if view.phase != "error" && view.phase != "stopped" {
            view.phase = "stopping".into();
        }
    }

    pub async fn shutdown(&self) -> anyhow::Result<()> {
        let _shutdown = self.shutdown_gate.lock().await;
        self.begin_shutdown();
        let task_failure = if let Some(task) = self.task.lock().await.take() {
            task.await
                .err()
                .map(|error| format!("PP control task failed: {error}"))
        } else {
            None
        };
        let mut failures = disable_all(&self.manager).await;
        if let Some(error) = task_failure {
            failures.push(error);
        }
        let mut view = lock_unpoisoned(&self.view);
        view.running = false;
        view.active_motor_index = None;
        for motor in &mut view.motors {
            motor.active = false;
            motor.enabled = failures
                .iter()
                .any(|failure| failure.starts_with(&format!("ID {}:", motor.node_id)));
            motor.sequence_state = if motor.enabled {
                "disable-failed"
            } else {
                "disabled"
            }
            .into();
        }
        if failures.is_empty() {
            if view.phase != "error" {
                view.phase = "stopped".into();
            }
            Ok(())
        } else {
            let detail = failures.join("; ");
            view.phase = "error".into();
            view.last_error = Some(detail.clone());
            Err(anyhow::anyhow!(detail))
        }
    }

    async fn control_loop(self: Arc<Self>) {
        let mut waypoint_index = 0usize;
        let mut motor_index = 0usize;
        let mut completed_cycles = 0u64;
        let mut command_pending = true;
        let mut movement_started = Instant::now();
        let mut settle_started: Option<Instant> = None;
        let mut dwell_until: Option<Instant> = None;
        let feedback_stale_timeout =
            Duration::from_millis((u64::from(self.request.tpdo_event_timer_ms) * 4).max(250));
        let mut ticker = tokio::time::interval(LOOP_PERIOD);
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut failure: Option<String> = None;

        while self.running.load(Ordering::Acquire) {
            ticker.tick().await;
            let now = Instant::now();
            let mut feedback = Vec::with_capacity(MOTOR_COUNT);
            for &node_id in &NODE_IDS {
                let state = match self.manager.status(node_id) {
                    Ok(state) => state,
                    Err(error) => {
                        failure = Some(format!("motor {node_id} status unavailable: {error}"));
                        break;
                    }
                };
                let age = state.connection.last_tpdo1.map(|instant| instant.elapsed());
                let enabled = matches!(
                    state.logic,
                    Some(MeowMotorLogic::Enabled(MeowMotorMode::ProfilePosition))
                );
                let motor_error = match state.logic {
                    Some(MeowMotorLogic::Enabled(MeowMotorMode::ProfilePosition)) => None,
                    Some(MeowMotorLogic::Error {
                        mode_display,
                        detailed_error,
                    }) => Some(format!(
                        "mode 0x{mode_display:02X}, detail 0x{detailed_error:04X}"
                    )),
                    Some(other) => Some(format!("unexpected control state {other:?}")),
                    None => Some("PP control state unavailable".into()),
                };
                let position = meow_position_rev(&state);
                let velocity = meow_velocity_rev_per_s(&state);
                if !state.connection.online {
                    failure = Some(format!("motor {node_id} went offline"));
                } else if age.is_none_or(|value| value > feedback_stale_timeout) {
                    failure = Some(format!("motor {node_id} TPDO1 feedback is stale"));
                } else if !enabled {
                    failure = Some(format!(
                        "motor {node_id} left enabled PP mode: {}",
                        motor_error
                            .clone()
                            .unwrap_or_else(|| "unknown state".into())
                    ));
                } else if position.is_none() || velocity.is_none() {
                    failure = Some(format!(
                        "motor {node_id} position or velocity feedback is unavailable"
                    ));
                }
                feedback.push((state, age, motor_error, position, velocity));
            }
            if failure.is_some() {
                break;
            }

            if let Some(until) = dwell_until {
                if now >= until {
                    waypoint_index = (waypoint_index + 1) % self.request.waypoints.len();
                    if waypoint_index == 0 {
                        completed_cycles = completed_cycles.saturating_add(1);
                    }
                    motor_index = 0;
                    command_pending = true;
                    settle_started = None;
                    dwell_until = None;
                }
            } else if command_pending {
                let target = self.request.waypoints[waypoint_index][motor_index];
                let position = match SignedQ8_24::from_revolutions(target) {
                    Ok(position) => position,
                    Err(error) => {
                        failure = Some(format!(
                            "motor {} target at waypoint {} is invalid: {error}",
                            NODE_IDS[motor_index],
                            waypoint_index + 1
                        ));
                        break;
                    }
                };
                if let Err(error) = self
                    .manager
                    .set_target_sdo(
                        NODE_IDS[motor_index],
                        MeowMotorTarget::ProfilePosition { position },
                    )
                    .await
                {
                    failure = Some(format!(
                        "motor {} PP target at waypoint {} failed: {error}",
                        NODE_IDS[motor_index],
                        waypoint_index + 1
                    ));
                    break;
                }
                command_pending = false;
                movement_started = Instant::now();
                settle_started = None;
            } else {
                let target = self.request.waypoints[waypoint_index][motor_index];
                let position = feedback[motor_index]
                    .3
                    .expect("validated finite PP position feedback");
                let velocity = feedback[motor_index]
                    .4
                    .expect("validated finite PP velocity feedback");
                if now.saturating_duration_since(movement_started)
                    > Duration::from_millis(self.request.movement_timeout_ms as u64)
                {
                    failure = Some(format!(
                        "motor {} did not settle at waypoint {} within {} ms",
                        NODE_IDS[motor_index],
                        waypoint_index + 1,
                        self.request.movement_timeout_ms
                    ));
                    break;
                }
                let at_target = (position - target).abs()
                    <= self.request.arrival_position_tolerance_rev
                    && velocity.abs() <= self.request.arrival_velocity_tolerance_rev_per_s;
                if at_target {
                    let since = settle_started.get_or_insert(now);
                    if now.saturating_duration_since(*since)
                        >= Duration::from_millis(self.request.settle_time_ms as u64)
                    {
                        settle_started = None;
                        if motor_index + 1 < MOTOR_COUNT {
                            motor_index += 1;
                            command_pending = true;
                        } else {
                            dwell_until = Some(
                                now + Duration::from_millis(self.request.waypoint_dwell_ms as u64),
                            );
                        }
                    }
                } else {
                    settle_started = None;
                }
            }

            let in_dwell = dwell_until.is_some();
            let mut view = lock_unpoisoned(&self.view);
            view.running = true;
            view.phase = if in_dwell { "dwell" } else { "moving" }.into();
            view.active_waypoint_index = waypoint_index;
            view.active_motor_index = (!in_dwell).then_some(motor_index);
            view.completed_cycles = completed_cycles;
            for index in 0..MOTOR_COUNT {
                let (state, age, motor_error, actual, velocity) = &feedback[index];
                let motor = &mut view.motors[index];
                let target = self.request.waypoints[waypoint_index][index];
                motor.online = state.connection.online;
                motor.enabled = matches!(
                    state.logic,
                    Some(MeowMotorLogic::Enabled(MeowMotorMode::ProfilePosition))
                );
                motor.active = !in_dwell && index == motor_index;
                motor.sequence_state = if in_dwell {
                    "holding"
                } else if index < motor_index {
                    "reached"
                } else if index == motor_index {
                    if command_pending {
                        "commanding"
                    } else if settle_started.is_some() {
                        "settling"
                    } else {
                        "moving"
                    }
                } else {
                    "waiting"
                }
                .into();
                motor.target_position_rev = target;
                motor.actual_position_rev = *actual;
                motor.actual_velocity_rev_per_s = *velocity;
                motor.position_error_rev = actual.map(|position| position - target);
                motor.feedback_age_ms = age.map(|value| value.as_millis() as u64);
                motor.error = motor_error.clone();
            }
        }

        self.running.store(false, Ordering::Release);
        if let Some(detail) = failure {
            {
                let mut view = lock_unpoisoned(&self.view);
                view.running = false;
                view.phase = "error".into();
                view.active_motor_index = None;
                view.last_error = Some(detail.clone());
                for motor in &mut view.motors {
                    motor.active = false;
                    if motor.error.is_none() {
                        motor.error = Some(detail.clone());
                    }
                }
            }
            log::error!("six-motor PP safety stop: {detail}");
            let failures = disable_all(&self.manager).await;
            let mut view = lock_unpoisoned(&self.view);
            for motor in &mut view.motors {
                motor.enabled = failures
                    .iter()
                    .any(|failure| failure.starts_with(&format!("ID {}:", motor.node_id)));
                motor.sequence_state = if motor.enabled {
                    "disable-failed"
                } else {
                    "disabled"
                }
                .into();
            }
            if !failures.is_empty() {
                view.last_error = Some(format!(
                    "{detail}; disable failures: {}",
                    failures.join("; ")
                ));
            }
        } else {
            lock_unpoisoned(&self.view).running = false;
        }
        log::info!("six-motor sequential PP control loop stopped");
    }
}

async fn setup_all(
    manager: &MeowMotorManager,
    pdo_profile: PdoProfile,
    request: &SixMotorPpRequest,
    shutdown_requested: &AtomicBool,
) -> anyhow::Result<[f64; MOTOR_COUNT]> {
    for &node_id in &NODE_IDS {
        check_startup_cancel(shutdown_requested)?;
        manager
            .identify(node_id)
            .await
            .map_err(|error| anyhow::anyhow!("motor {node_id} identify failed: {error}"))?;
    }
    ensure_six_identified_online(manager)?;

    for &node_id in &NODE_IDS {
        check_startup_cancel(shutdown_requested)?;
        manager
            .initialize(node_id, pdo_profile)
            .await
            .map_err(|error| anyhow::anyhow!("motor {node_id} initialize failed: {error}"))?;
        manager
            .set_max_torque(node_id, request.max_torque_permille)
            .await
            .map_err(|error| anyhow::anyhow!("motor {node_id} maximum torque failed: {error}"))?;
        manager
            .set_profile_limits(node_id, request.profile.into())
            .await
            .map_err(|error| anyhow::anyhow!("motor {node_id} PP profile failed: {error}"))?;
    }

    let mut positions = [0.0f64; MOTOR_COUNT];
    for (index, &node_id) in NODE_IDS.iter().enumerate() {
        check_startup_cancel(shutdown_requested)?;
        let position_rev = wait_for_fresh_position(manager, node_id).await?;
        let position = SignedQ8_24::from_revolutions(position_rev)
            .map_err(|error| anyhow::anyhow!("motor {node_id} startup hold target: {error}"))?;
        manager
            .set_mode_sdo(node_id, MeowMotorTarget::ProfilePosition { position })
            .await
            .map_err(|error| anyhow::anyhow!("motor {node_id} PP enable failed: {error}"))?;
        positions[index] = position_rev;
    }
    Ok(positions)
}

fn check_startup_cancel(shutdown_requested: &AtomicBool) -> anyhow::Result<()> {
    if shutdown_requested.load(Ordering::Acquire) {
        anyhow::bail!("six-motor PP startup cancelled by application shutdown");
    }
    Ok(())
}

fn ensure_six_identified_online(manager: &MeowMotorManager) -> anyhow::Result<()> {
    let devices = manager.list();
    for &node_id in &NODE_IDS {
        let device = devices
            .iter()
            .find(|device| device.node_id == node_id)
            .ok_or_else(|| anyhow::anyhow!("motor ID {node_id} has not been discovered"))?;
        if !device.online {
            anyhow::bail!("motor ID {node_id} is offline");
        }
        if device.identity.is_none() || device.model.is_none() {
            anyhow::bail!(
                "node ID {node_id} is not an exact supported Meow Motor identity; refusing PP motion"
            );
        }
    }
    Ok(())
}

async fn wait_for_fresh_position(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<f64> {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let state = manager
            .status(node_id)
            .map_err(|error| anyhow::anyhow!("motor {node_id} status unavailable: {error}"))?;
        if state.connection.online
            && state
                .connection
                .last_tpdo1
                .is_some_and(|instant| instant.elapsed() <= Duration::from_millis(250))
        {
            if let Some(position) = meow_position_rev(&state) {
                return Ok(position);
            }
        }
        if Instant::now() >= deadline {
            anyhow::bail!("motor {node_id} did not provide a fresh position within 2 seconds");
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

fn meow_position_rev(state: &MeowMotorLiveState) -> Option<f64> {
    state
        .measurements
        .position
        .map(|position| position.to_revolutions())
        .filter(|position| position.is_finite())
}

fn meow_velocity_rev_per_s(state: &MeowMotorLiveState) -> Option<f64> {
    state
        .measurements
        .velocity_rev_per_s
        .filter(|velocity| velocity.is_finite())
}

async fn disable_all(manager: &MeowMotorManager) -> Vec<String> {
    let mut failures = Vec::new();
    for &node_id in &NODE_IDS {
        if let Err(error) = manager.disable(node_id).await {
            failures.push(format!(
                "ID {node_id}: disable confirmation failed: {error}"
            ));
        }
    }
    failures
}

async fn disable_startup_candidates(manager: &MeowMotorManager) -> Vec<String> {
    let devices = manager.list();
    let mut failures = Vec::new();
    for &node_id in &NODE_IDS {
        let should_disable = devices.iter().any(|device| {
            device.node_id == node_id
                && (matches!(device.lifecycle, MeowMotorLifecycle::Initialized)
                    || matches!(device.logic, Some(MeowMotorLogic::Enabled(_))))
        });
        if should_disable {
            if let Err(error) = manager.disable(node_id).await {
                failures.push(format!(
                    "ID {node_id}: disable confirmation failed: {error}"
                ));
            }
        }
    }
    failures
}

fn lock_unpoisoned<T>(mutex: &StdMutex<T>) -> StdMutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request() -> SixMotorPpRequest {
        SixMotorPpRequest {
            waypoints: vec![[0.0; MOTOR_COUNT]; WAYPOINT_COUNT],
            tpdo_event_timer_ms: 10,
            max_torque_permille: 200,
            profile: SixMotorPpProfile {
                velocity_rev_per_s: 0.2,
                acceleration_rev_per_s2: 0.2,
                deceleration_rev_per_s2: 0.2,
            },
            arrival_position_tolerance_rev: 0.003,
            arrival_velocity_tolerance_rev_per_s: 0.02,
            settle_time_ms: 200,
            movement_timeout_ms: 15_000,
            waypoint_dwell_ms: 2_500,
        }
    }

    #[test]
    fn request_accepts_four_complete_waypoints() {
        assert!(request().validate().is_ok());
    }

    #[test]
    fn request_rejects_missing_axes_and_unsafe_limits() {
        let mut bad = request();
        bad.waypoints[2][4] = 128.0;
        assert!(bad.validate().unwrap_err().contains("motor 5"));
        let mut bad = request();
        bad.tpdo_event_timer_ms = 0;
        assert!(bad.validate().unwrap_err().contains("TPDO"));
        let mut bad = request();
        bad.movement_timeout_ms = 999;
        assert!(bad.validate().unwrap_err().contains("timeout"));
    }
}
