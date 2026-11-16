//! Motor-referenced tests of the unmodified on-chip SFLP attitude.
//! All timing, freshness checks and cleanup run natively, independent of UI polling.
use super::{Live, Session};
use anyhow::{ensure, Context, Result};
use async_trait::async_trait;
use hex_motor::cia402::PdoProfile;
use hex_motor::meow_motor::{
    MeowMotorInitializeOptions, MeowMotorLogic, MeowMotorManager, MeowMotorManagerOptions,
    MeowMotorMode, MeowMotorTarget, MeowProfileLimits, SignedQ8_24,
};
use serde::{Deserialize, Serialize};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::time::{Duration, Instant};
use tokio::{sync::watch, task::JoinHandle};

const FRESH: Duration = Duration::from_millis(250);
const TICK: Duration = Duration::from_millis(10);
type Quaternion = [f64; 4];

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Config {
    pub imu_node: u8,
    pub motor_node: u8,
    pub host_node: u8,
    pub angle_deg: f64,
    pub repeats: u32,
    /// Sensor-native X=0, Y=1, Z=2, aligned with the motor shaft.
    pub axis: usize,
    pub direction: i8,
    pub zero_each_round: bool,
    pub speed_deg_s: f64,
    pub settle_ms: u64,
    pub samples: usize,
    pub tolerance_deg: f64,
    pub timeout_s: u64,
}

impl Config {
    fn validate(&self) -> Result<()> {
        for node in [self.imu_node, self.motor_node, self.host_node] {
            crate::can_node_policy::check(node).map_err(anyhow::Error::msg)?;
        }
        ensure!(
            self.imu_node != self.motor_node
                && self.host_node != self.motor_node
                && self.host_node != self.imu_node,
            "IMU, motor and host Node-IDs must be different"
        );
        ensure!(
            self.angle_deg.is_finite() && (0.1..=360.0).contains(&self.angle_deg.abs()),
            "Rotation must be between ±0.1° and ±360°"
        );
        ensure!(
            (1..=10_000).contains(&self.repeats),
            "Repeat count must be 1..10000"
        );
        ensure!(
            self.axis <= 2 && [-1, 1].contains(&self.direction),
            "Select X/Y/Z and direction ±1"
        );
        ensure!(
            (1.0..=180.0).contains(&self.speed_deg_s),
            "Speed must be 1..180 °/s"
        );
        ensure!(
            (100..=10_000).contains(&self.settle_ms),
            "Settling time must be 100..10000 ms"
        );
        ensure!(
            (1..=200).contains(&self.samples),
            "Sample count must be 1..200"
        );
        ensure!(
            (0.001..=1.0).contains(&self.tolerance_deg),
            "Motor tolerance must be 0.001..1°"
        );
        ensure!(
            (2..=600).contains(&self.timeout_s),
            "Motion timeout must be 2..600 s"
        );
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct Row {
    pub round: u32,
    pub target_deg: f64,
    pub motor_deg: f64,
    pub home_raw_deg: f64,
    pub raw_deg: f64,
    pub error_deg: f64,
    pub error_percent: f64,
    pub absolute_error_percent: f64,
    pub return_raw_deg: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct TestSnapshot {
    pub active: bool,
    pub phase: String,
    pub completed: u32,
    pub config: Config,
    pub motor_model: Option<String>,
    pub motor_position_deg: Option<f64>,
    pub imu_angle_deg: Option<f64>,
    pub stop_confirmed: bool,
    pub error: Option<String>,
    pub rows: Vec<Row>,
}

#[derive(Clone, Copy)]
struct Attitude {
    q: Quaternion,
    counter: u32,
    received: Instant,
}

/// Require a coherent quaternion/counter/flags/age PDO. A remapping across
/// separate asynchronous PDOs is not a synchronized measurement for this test.
fn attitude(live: &Live) -> Result<Attitude> {
    let now = Instant::now();
    ensure!(
        live.error.is_none(),
        "IMU receive error: {}",
        live.error.as_deref().unwrap_or("")
    );
    let get = |index, sub| -> Result<&super::TimedValue> {
        let v = live
            .values
            .get(&(index, sub))
            .context("Missing SFLP fields; initialize default PDOs first")?;
        ensure!(
            v.value.is_finite() && now.duration_since(v.received) < FRESH,
            "IMU data is stale"
        );
        Ok(v)
    };
    let count = get(0x3008, 1)?;
    ensure!(
        now.duration_since(count.changed) < FRESH,
        "IMU sample counter stopped"
    );
    let age = get(0x3008, 2)?;
    let flags = get(0x3008, 5)?;
    ensure!(
        flags.value as u32 & 1 != 0
            && age.value >= 0.0
            && age.value + now.duration_since(age.received).as_secs_f64() * 1000.0 < 250.0,
        "IMU SFLP attitude is invalid or stale"
    );
    let mut q = [0.0; 4];
    for (i, component) in q.iter_mut().enumerate() {
        let v = get(0x3000, i as u8 + 1)?;
        ensure!(
            v.received == count.received,
            "Map quaternion and SFLP counter in the same PDO"
        );
        *component = v.value;
    }
    ensure!(
        age.received == count.received && flags.received == count.received,
        "Map SFLP age, status and quaternion in the same PDO"
    );
    let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    ensure!((0.5..=1.5).contains(&norm), "Invalid IMU quaternion norm");
    Ok(Attitude {
        q: q.map(|v| v / norm),
        counter: count.value as u32,
        received: count.received,
    })
}

fn relative(q: Quaternion, reference: Quaternion) -> Quaternion {
    let [a, b, c, d] = [reference[0], -reference[1], -reference[2], -reference[3]];
    let [w, x, y, z] = q;
    [
        a * w - b * x - c * y - d * z,
        a * x + b * w + c * z - d * y,
        a * y - b * z + c * w + d * x,
        a * z + b * y - c * x + d * w,
    ]
}

struct AngleTracker {
    reference: Quaternion,
    axis: usize,
    direction: f64,
    previous: f64,
    angle: f64,
    counter: u32,
    received: Instant,
}

impl AngleTracker {
    fn new(sample: Attitude, config: &Config) -> Self {
        Self {
            reference: sample.q,
            axis: config.axis,
            direction: config.direction as f64,
            previous: 0.0,
            angle: 0.0,
            counter: sample.counter,
            received: sample.received,
        }
    }

    fn update(&mut self, sample: Attitude) -> Result<bool> {
        ensure!(
            sample.received.duration_since(self.received) < FRESH,
            "Gap in IMU test samples"
        );
        if sample.counter == self.counter {
            return Ok(false);
        }
        ensure!(
            sample.counter.wrapping_sub(self.counter) < (1 << 31),
            "IMU sample counter reset"
        );
        let q = relative(sample.q, self.reference);
        // Swing/twist projection avoids Euler pitch folding at 90° and does
        // not hide overshoot at 180°. q and -q represent the same orientation.
        ensure!(
            q[0].hypot(q[self.axis + 1]) > 1e-6,
            "Rotation is perpendicular to the selected sensor axis; check fixture alignment"
        );
        let phase = 2.0 * q[self.axis + 1].atan2(q[0]).to_degrees() * self.direction;
        let delta = (phase - self.previous + 180.0).rem_euclid(360.0) - 180.0;
        self.angle += delta;
        self.previous = phase;
        self.counter = sample.counter;
        self.received = sample.received;
        Ok(true)
    }
}

struct Feedback {
    position: f64,
    velocity: f64,
    received: Instant,
}

#[async_trait]
trait Rig: Send + Sync {
    async fn initialize(&self, config: &Config) -> Result<String>;
    async fn command(&self, degrees: f64) -> Result<()>;
    fn feedback(&self) -> Result<Feedback>;
    fn attitude(&self) -> Result<Attitude>;
    async fn disable(&self) -> Result<()>;
}

struct HardwareRig {
    manager: MeowMotorManager,
    live: Arc<Mutex<Live>>,
    node: u8,
    needs_disable: AtomicBool,
}

#[async_trait]
impl Rig for HardwareRig {
    async fn initialize(&self, config: &Config) -> Result<String> {
        let model = self.manager.identify(self.node).await?;
        // This manager was just attached to an already-open IMU bus. SDO
        // identity may complete before its first heartbeat subscription fires.
        tokio::time::timeout(Duration::from_secs(2), async {
            while !self.manager.status(self.node)?.connection.online {
                tokio::time::sleep(TICK).await;
            }
            Ok::<_, anyhow::Error>(())
        })
        .await
        .context("Motor heartbeat not received after identification")??;
        self.needs_disable.store(true, Ordering::Release);
        self.manager
            .initialize_with_options(
                self.node,
                MeowMotorInitializeOptions::new(PdoProfile::from_event_timer_ms(10)?),
            )
            .await?;
        self.manager
            .set_profile_limits(
                self.node,
                MeowProfileLimits {
                    velocity_rev_per_s: (config.speed_deg_s / 360.0) as f32,
                    acceleration_rev_per_s2: (config.speed_deg_s / 180.0) as f32,
                    deceleration_rev_per_s2: (config.speed_deg_s / 180.0) as f32,
                },
            )
            .await?;
        Ok(model.display_name().into())
    }

    async fn command(&self, degrees: f64) -> Result<()> {
        let target = MeowMotorTarget::ProfilePosition {
            position: SignedQ8_24::from_revolutions(degrees / 360.0)?,
        };
        if self.manager.status(self.node)?.logic
            == Some(MeowMotorLogic::Enabled(MeowMotorMode::ProfilePosition))
        {
            self.manager.set_target_sdo(self.node, target).await?;
        } else {
            self.manager.set_mode_sdo(self.node, target).await?;
        }
        Ok(())
    }

    fn feedback(&self) -> Result<Feedback> {
        let s = self.manager.status(self.node)?;
        ensure!(
            s.lifecycle.is_ready() && s.connection.online,
            "Motor offline or needs reinitialization"
        );
        ensure!(
            s.logic == Some(MeowMotorLogic::Enabled(MeowMotorMode::ProfilePosition)),
            "Motor left position mode: {:?}",
            s.logic
        );
        let received = s
            .connection
            .last_tpdo1
            .context("No motor position feedback")?;
        ensure!(
            received.elapsed() < FRESH
                && s.connection.last_tpdo2.is_some_and(|t| t.elapsed() < FRESH),
            "Motor feedback is stale"
        );
        Ok(Feedback {
            position: s
                .measurements
                .position
                .context("No motor position")?
                .to_revolutions()
                * 360.0,
            velocity: s
                .measurements
                .velocity_rev_per_s
                .context("No motor velocity")?
                * 360.0,
            received,
        })
    }

    fn attitude(&self) -> Result<Attitude> {
        attitude(&self.live.lock().unwrap())
    }

    async fn disable(&self) -> Result<()> {
        if self.needs_disable.load(Ordering::Acquire) {
            self.manager
                .disable(self.node)
                .await
                .context("Motor disable was not confirmed; retry Stop")?;
            self.needs_disable.store(false, Ordering::Release);
        }
        Ok(())
    }
}

type Shared = Arc<Mutex<TestSnapshot>>;

/// An entire leg has one deadline, including arrival, settling and new samples.
async fn measure(
    rig: &dyn Rig,
    config: &Config,
    target: f64,
    tracker: &mut Option<AngleTracker>,
    shared: &Shared,
) -> Result<(f64, f64, Attitude)> {
    let command_at = Instant::now();
    rig.command(target).await?;
    let mut stable_since = None;
    let mut sum = 0.0;
    let mut motor_sum = 0.0;
    let mut samples = 0;
    let mut quaternion_sum = [0.0; 4];
    let mut last_sample = None;
    loop {
        ensure!(
            command_at.elapsed() < Duration::from_secs(config.timeout_s),
            "Motor arrival / IMU sampling timed out at {target}°"
        );
        let motor = rig.feedback()?;
        let imu = rig.attitude()?;
        let changed = match tracker.as_mut() {
            Some(t) => t.update(imu)?,
            None => last_sample != Some(imu.counter),
        };
        let angle = tracker.as_ref().map_or(0.0, |t| t.angle);
        {
            let mut s = shared.lock().unwrap();
            s.motor_position_deg = Some(motor.position);
            s.imu_angle_deg = tracker.as_ref().map(|t| t.angle);
        }
        // Never accept pre-command feedback, or a transient pass through target.
        if motor.received > command_at
            && (motor.position - target).abs() <= config.tolerance_deg
            && motor.velocity.abs() <= 0.5
        {
            let stable = *stable_since.get_or_insert_with(Instant::now);
            if stable.elapsed() >= Duration::from_millis(config.settle_ms)
                && imu.received > stable + Duration::from_millis(config.settle_ms)
                && changed
            {
                sum += angle;
                motor_sum += motor.position;
                let sign = if quaternion_sum
                    .iter()
                    .zip(imu.q)
                    .map(|(a, b)| a * b)
                    .sum::<f64>()
                    < 0.0
                {
                    -1.0
                } else {
                    1.0
                };
                for (sum, component) in quaternion_sum.iter_mut().zip(imu.q) {
                    *sum += component * sign;
                }
                samples += 1;
                if samples == config.samples {
                    let norm = quaternion_sum.iter().map(|v| v * v).sum::<f64>().sqrt();
                    return Ok((
                        sum / samples as f64,
                        motor_sum / samples as f64,
                        Attitude {
                            q: quaternion_sum.map(|v| v / norm),
                            ..imu
                        },
                    ));
                }
            }
        } else {
            stable_since = None;
            sum = 0.0;
            motor_sum = 0.0;
            samples = 0;
            quaternion_sum = [0.0; 4];
        }
        last_sample = Some(imu.counter);
        tokio::time::sleep(TICK).await;
    }
}

fn phase(shared: &Shared, name: &str) {
    shared.lock().unwrap().phase = name.into();
}

async fn execute(rig: &dyn Rig, config: &Config, shared: &Shared) -> Result<()> {
    let model = rig.initialize(config).await?;
    shared.lock().unwrap().motor_model = Some(model);
    // Check IMU again after initialization before issuing any motion.
    rig.attitude()?;
    phase(shared, "homing");
    let mut tracker = None;
    let (_, _, mut home_sample) = measure(rig, config, 0.0, &mut tracker, shared).await?;
    tracker = Some(AngleTracker::new(home_sample, config));
    let mut home = 0.0;
    for round in 1..=config.repeats {
        if config.zero_each_round {
            tracker = Some(AngleTracker::new(home_sample, config));
        }
        phase(shared, "rotating");
        let (raw, motor, _) = measure(rig, config, config.angle_deg, &mut tracker, shared).await?;
        let error = raw - config.angle_deg;
        shared.lock().unwrap().rows.push(Row {
            round,
            target_deg: config.angle_deg,
            motor_deg: motor,
            home_raw_deg: home,
            raw_deg: raw,
            error_deg: error,
            error_percent: error / config.angle_deg.abs() * 100.0,
            absolute_error_percent: error.abs() / config.angle_deg.abs() * 100.0,
            return_raw_deg: None,
        });
        phase(shared, "returning");
        let (returned, _, sample) = measure(rig, config, 0.0, &mut tracker, shared).await?;
        home_sample = sample;
        home = returned;
        let mut s = shared.lock().unwrap();
        s.rows.last_mut().unwrap().return_raw_deg = Some(returned);
        s.completed = round;
    }
    Ok(())
}

async fn drive(rig: &dyn Rig, config: &Config, shared: &Shared, mut cancel: watch::Receiver<bool>) {
    let outcome = tokio::select! {
        biased;
        _ = async { if !*cancel.borrow() { let _ = cancel.changed().await; } } => None,
        result = execute(rig, config, shared) => Some(result),
    };
    phase(shared, "stopping");
    // Cleanup is outside the cancellation branch; Stop cannot interrupt disable.
    let stopped = rig.disable().await;
    let mut s = shared.lock().unwrap();
    s.active = false;
    s.stop_confirmed = stopped.is_ok();
    s.phase = match &outcome {
        None => "cancelled",
        Some(Ok(())) => "completed",
        Some(Err(_)) => "failed",
    }
    .into();
    s.error = outcome.and_then(Result::err).map(|e| format!("{e:#}"));
    if let Err(e) = stopped {
        s.phase = "failed".into();
        s.error = Some(format!(
            "{}{}",
            s.error.as_ref().map_or(String::new(), |e| format!("{e}; ")),
            e
        ));
    }
}

pub struct Run {
    shared: Shared,
    rig: Arc<HardwareRig>,
    cancel: watch::Sender<bool>,
    task: Option<JoinHandle<()>>,
}

impl Run {
    fn validate_start(session: &Session, config: &Config, previous_host: Option<u8>) -> Result<()> {
        config.validate()?;
        {
            let live = session.live.lock().unwrap();
            ensure!(
                live.identity
                    .as_ref()
                    .is_some_and(|i| i.node == config.imu_node),
                "Initialize the selected IMU first"
            );
            attitude(&live)?;
            ensure!(
                previous_host == Some(config.host_node)
                    || !live
                        .heartbeat
                        .get(&config.host_node)
                        .is_some_and(|(_, t)| t.elapsed() < Duration::from_secs(2)),
                "Host Node-ID is already in use; choose a free ID"
            );
            // The driver configures standard motor TPDO1/2. Reject custom IMU
            // mappings that would alias either feedback stream or SDO replies.
            let motor_ids =
                [0x180, 0x280, 0x580, 0x600, 0x700].map(|base| base + config.motor_node as u32);
            ensure!(
                !live
                    .pdos
                    .iter()
                    .any(|p| p.cob_id & super::DISABLED == 0
                        && motor_ids.contains(&(p.cob_id & 0x7ff))),
                "IMU PDO COB-ID conflicts with the motor; restore default IMU mappings"
            );
        }
        Ok(())
    }

    fn start(session: &Session, config: Config) -> Result<Self> {
        let rig = Arc::new(HardwareRig {
            manager: MeowMotorManager::new(
                session.bus.clone(),
                MeowMotorManagerOptions {
                    heartbeat_node_id: config.host_node,
                    broadcast_heartbeat: true,
                    auto_identify: false,
                    sdo_timeout: Duration::from_millis(500),
                    ..Default::default()
                },
            )?,
            live: session.live.clone(),
            node: config.motor_node,
            needs_disable: AtomicBool::new(false),
        });
        let shared = Arc::new(Mutex::new(TestSnapshot {
            active: true,
            phase: "initializing".into(),
            completed: 0,
            config: config.clone(),
            motor_model: None,
            motor_position_deg: None,
            imu_angle_deg: None,
            stop_confirmed: false,
            error: None,
            rows: vec![],
        }));
        let (cancel, rx) = watch::channel(false);
        let (task_rig, task_shared) = (rig.clone(), shared.clone());
        let task = tokio::spawn(async move {
            drive(&*task_rig, &config, &task_shared, rx).await;
        });
        Ok(Self {
            shared,
            rig,
            cancel,
            task: Some(task),
        })
    }

    pub fn snapshot(&self) -> TestSnapshot {
        self.shared.lock().unwrap().clone()
    }
    pub fn blocks_changes(&self) -> bool {
        let s = self.shared.lock().unwrap();
        s.active || !s.stop_confirmed
    }
    pub async fn stop(&mut self) -> Result<()> {
        let _ = self.cancel.send(true);
        if let Some(task) = self.task.take() {
            if let Err(e) = task.await {
                let mut s = self.shared.lock().unwrap();
                s.active = false;
                s.phase = "failed".into();
                s.error = Some(format!("Test task: {e}"));
            }
        }
        if !self.shared.lock().unwrap().stop_confirmed {
            self.rig.disable().await?;
            self.shared.lock().unwrap().stop_confirmed = true;
        }
        Ok(())
    }
}

impl Drop for Run {
    fn drop(&mut self) {
        // Do not abort the worker: it must run its disable cleanup on cancellation.
        let _ = self.cancel.send(true);
    }
}

#[tauri::command]
pub async fn hpm_imu_test_start(
    state: tauri::State<'_, crate::state::AppState>,
    config: Config,
) -> Result<TestSnapshot, String> {
    config.validate().map_err(|e| e.to_string())?;
    ensure_not_shutting_down(&state)?;
    // Choose the extended close path before any await, as in other motor apps.
    state
        .extension_runtime_active
        .store(true, Ordering::Release);
    let _operation = state.extension_op.lock().await;
    let mut guard = state.hpm_imu.lock().await;
    ensure_not_shutting_down(&state)?;
    let session = guard.as_mut().ok_or("HPM IMU not connected")?;
    session.ensure_test_idle().map_err(|e| e.to_string())?;
    let previous_host = session
        .angle_test
        .as_ref()
        .map(|run| run.snapshot().config.host_node);
    // A failed preflight must preserve the previous measurement report.
    Run::validate_start(session, &config, previous_host).map_err(|e| format!("{e:#}"))?;
    // Drop the previous test's heartbeat producer before creating another one.
    if let Some(mut old) = session.angle_test.take() {
        old.stop().await.map_err(|e| format!("{e:#}"))?;
    }
    let run = Run::start(session, config).map_err(|e| format!("{e:#}"))?;
    let snapshot = run.snapshot();
    session.angle_test = Some(run);
    Ok(snapshot)
}

fn ensure_not_shutting_down(state: &crate::state::AppState) -> Result<(), String> {
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("Application is shutting down".into());
    }
    Ok(())
}

#[tauri::command]
pub async fn hpm_imu_test_stop(
    state: tauri::State<'_, crate::state::AppState>,
) -> Result<(), String> {
    let mut guard = state.hpm_imu.lock().await;
    if let Some(run) = guard.as_mut().and_then(|s| s.angle_test.as_mut()) {
        run.stop().await.map_err(|e| format!("{e:#}"))?;
    }
    Ok(())
}

#[cfg(test)]
#[path = "hpm_imu_test_tests.rs"]
mod tests;
