//! HopeA3 — first **Robot Application**: an inverted-triangle 3-wheel omni base.
//!
//! Unlike Direct Control (discover unknown motors, drive each over SDO), a
//! Robot Application has a *fixed* motor count and layout, so it can use PDO
//! for control. HopeA3 drives all three **hex-meow** motors in **uncompressed
//! MIT** mode used as a direct velocity loop: with KP=0 and PDES unused, the
//! motor's torque law reduces to `τ = KD·(VDES − v) + TFF`, clamped by max
//! torque — no profile ramp (which is why this replaced PV).
//!
//! RPDO1 consumes an 8-byte lane of one shared 24-byte CAN-FD frame:
//! velocity f32 (`0x4102:02`), KD u16 (`0x4102:05`), max torque u16
//! (`0x4572`). RPDO2 consumes the motor's mode byte (`0x4401`). Initialization
//! explicitly disables compressed MIT (`0x4103:01`), zeros position/KP/TFF,
//! sets the Kp/Kd result limit, and reads each motor's gain factor (`0x4102:07`).
//!
//! Feedback / odometry reuse the meow driver's existing Device TPDO1 stream (it
//! already parses position / host-filtered velocity / torque per motor); we read
//! `mgr.status(nid)` and run forward kinematics + dead-reckoning at the control
//! rate.
//!
//! Geometry, drivetrain and the installed motor type are **compile-time
//! constants** below (the chassis is not reconfigurable at runtime — only the
//! velocity/torque limits and the command are).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::Duration;

use can_transport::{CanBus, CanFrame};
use hex_motor::canopen::rpdo_config::{build_rpdo_config_writes, RpdoRecipe};
use hex_motor::canopen::sdo;
use hex_motor::canopen::tpdo_config::{SdoWrite, TpdoEntry};
use hex_motor::cia402::PdoProfile;
use hex_motor::meow_motor::{
    build_target_sdo_writes, mode_rpdo2_recipe, MeowMitTarget, MeowMotorInitializeOptions,
    MeowMotorLifecycle, MeowMotorLogic, MeowMotorManager, MeowMotorMode, MeowMotorTarget,
};
use serde::Serialize;
use tokio::sync::Mutex;
use tokio::task::JoinHandle;

// ───────────────────────── compile-time chassis spec ─────────────────────────

// NOTE: the 4310/4342 motors report **and** accept velocity at the *output*
// (post-gearbox) shaft — i.e. the wheel shaft — so the reduction ratio cancels
// out of the kinematics entirely. The uncompressed-MIT velocity target
// (`0x4102:02`) and feedback velocity are both wheel rev/s; no gear factor is
// applied anywhere below.

/// Wheel radius (m). Wheel diameter is 0.154 m.
const WHEEL_RADIUS_M: f64 = 0.077;

/// Distance from each wheel's ground-contact point to the chassis centre (m).
/// The wheel-centre circle diameter is 0.648 m.
const WHEEL_CENTER_DISTANCE_M: f64 = 0.324;

/// Global CANopen host allocation: decimal 10 (`0x0A`).
const HOST_NODE_ID: u8 = 10;

/// The three motors' Node-IDs, indexed the same as [`CONTACTS_M`]:
/// `[motor1 (top-left), motor2 (bottom), motor3 (top-right)]`.
const NODE_IDS: [u8; 3] = [21, 22, 23];

/// Wheel ground-contact points in the chassis frame (ROS convention: +X
/// forward toward the "head" = the motor-1↔motor-3 edge, +Y left, +Z up),
/// metres, with the origin at **motor 2's contact** (as given). Indexed
/// `[motor1, motor2, motor3]`.
///
/// The three contacts form an equilateral triangle on a 648 mm diameter circle.
/// Relative to motor 2, motors 1/3 are 486 mm forward (X) and approximately
/// 280.593 mm left/right (Y).
const CONTACTS_M: [[f64; 2]; 3] = [
    [
        1.5 * WHEEL_CENTER_DISTANCE_M,
        0.866_025_403_784_438_6 * WHEEL_CENTER_DISTANCE_M,
    ], // motor 1, top-left
    [0.0, 0.0], // motor 2, bottom (given zero point)
    [
        1.5 * WHEEL_CENTER_DISTANCE_M,
        -0.866_025_403_784_438_6 * WHEEL_CENTER_DISTANCE_M,
    ], // motor 3, top-right
];

/// Extra offset (m, chassis frame) applied **on top of the contact centroid**
/// to pick the control/odometry reference point. The body origin is
/// `centroid(CONTACTS_M) + BODY_OFFSET_M`. Leave at zero for "centre of the
/// three contacts"; tweak to shift the reference (e.g. to a payload centre).
const BODY_OFFSET_M: [f64; 2] = [0.0, 0.0];

/// Per-wheel direction sign. All `+1` because "all motors +rotation ⇒ chassis
/// rotates CCW" already matches the tangential-CCW drive directions derived
/// from the geometry. Flip an entry to `-1.0` only if a motor turns out wired
/// the other way on hardware.
const WHEEL_SIGN: [f64; 3] = [1.0, 1.0, 1.0];

/// Shared CAN-FD COB-ID that all three motors listen to as RPDO1 (uncompressed-MIT
/// targets). Uses the global host-10 TPDO1 convention (`0x180 + 0x0A = 0x18A`),
/// matching the meow driver's [`host_tpdo_cob_id`]`(10, 0)`.
const HOST_TPDO1_COB_ID: u16 = 0x180 + HOST_NODE_ID as u16;

/// Shared CAN-FD COB-ID for the mode-command lane (RPDO2): `0x280 + 0x0A = 0x28A`,
/// matching the meow driver's [`host_tpdo_cob_id`]`(10, 1)`.
const HOST_TPDO2_COB_ID: u16 = 0x280 + HOST_NODE_ID as u16;

/// Control / odometry loop rate.
const CONTROL_HZ: u64 = 500;

/// Bytes per motor in the shared frame: velocity f32 + KD u16 + max torque u16 = 8 bytes.
const SLICE_LEN: usize = 8;

/// MIT mode command code for the shared mode lane (`0x4401`).
const MODE_MIT: u8 = MeowMotorMode::Mit.command_code();

/// Default limits / torque (all runtime-adjustable except being clamped, never
/// errored, when the UI sends more).
pub const DEFAULT_MAX_TORQUE_PERMILLE: u16 = 800;
pub const DEFAULT_MAX_LINEAR_MPS: f64 = 20.0;
pub const DEFAULT_MAX_ANGULAR_RPS: f64 = 3.0;
/// Default MIT velocity gain (Nm·s/rad). Tune live.
pub const DEFAULT_KD_SI: f64 = 1.0;

/// Default chassis acceleration limits (slew-rate limiting of the commanded
/// twist). `0` = unlimited (instant). Linear bounds the velocity-*vector*
/// increment (so heading changes are limited too); angular bounds |Δωz|.
pub const DEFAULT_MAX_LIN_ACC: f64 = 2.0; // m/s²
pub const DEFAULT_MAX_ANG_ACC: f64 = 6.0; // rad/s²

// ───────────────────────────── shared state ─────────────────────────────────

#[derive(Clone, Copy)]
struct Command {
    /// Body-frame twist (m/s, m/s, rad/s). Stored already clamped.
    vx: f64,
    vy: f64,
    wz: f64,
    /// Per-motor max torque (‰ of peak), indexed like [`NODE_IDS`].
    max_torque: [u16; 3],
    /// Per-motor MIT velocity gain KD in **SI units (Nm·s/rad)**, indexed like
    /// [`NODE_IDS`]. Converted to u16 using `KD × τ / 0x4102:07` (as in Motor Control).
    kd_si: [f64; 3],
    /// Adjustable velocity limits.
    max_linear: f64,
    max_angular: f64,
    /// Adjustable acceleration (slew-rate) limits. `0` = unlimited.
    max_lin_acc: f64,
    max_ang_acc: f64,
}

impl Default for Command {
    fn default() -> Self {
        Self {
            vx: 0.0,
            vy: 0.0,
            wz: 0.0,
            max_torque: [DEFAULT_MAX_TORQUE_PERMILLE; 3],
            kd_si: [DEFAULT_KD_SI; 3],
            max_linear: DEFAULT_MAX_LINEAR_MPS,
            max_angular: DEFAULT_MAX_ANGULAR_RPS,
            max_lin_acc: DEFAULT_MAX_LIN_ACC,
            max_ang_acc: DEFAULT_MAX_ANG_ACC,
        }
    }
}

/// Snapshot handed to the frontend each poll.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Hopea3State {
    /// World pose from dead-reckoning (m, m, rad).
    pub pose_x: f64,
    pub pose_y: f64,
    pub pose_theta: f64,
    /// Measured body twist from wheel feedback.
    pub meas_vx: f64,
    pub meas_vy: f64,
    pub meas_wz: f64,
    /// Applied body twist after clamping and acceleration ramping.
    pub cmd_vx: f64,
    pub cmd_vy: f64,
    pub cmd_wz: f64,
    /// Peak wheel-derived chassis speed since start/reset (not ground truth).
    pub peak_speed_mps: f64,
    pub feedback_valid: bool,
    pub max_linear: f64,
    pub max_angular: f64,
    pub motors: Vec<Hopea3Motor>,
    pub running: bool,
}

/// Init progress, polled by the UI while `hopea3_start` runs.
#[derive(Debug, Clone, Default, Serialize)]
pub struct InitProgress {
    /// `true` while an init is in flight.
    pub active: bool,
    /// 1-based index of the motor currently being initialized (0 = not started).
    pub current: u8,
    /// Total motors to initialize.
    pub total: u8,
    /// Current attempt number (1..=INIT_ATTEMPTS) for `current`.
    pub attempt: u8,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Hopea3Motor {
    pub node_id: u8,
    pub online: bool,
    pub enabled: bool,
    /// Commanded motor shaft velocity this tick (rev/s).
    pub target_rev_per_s: f32,
    /// Measured motor shaft velocity (rev/s, host-filtered by the manager).
    pub velocity_rev_per_s: Option<f32>,
    pub torque_nm: Option<f32>,
    pub max_torque_permille: u16,
    pub torque_permille: Option<i16>,
    /// Estimated MIT torque demand using host velocity feedback, after limits.
    /// Same raw torque domain as 0x4577; not a measurement of firmware demand.
    pub estimated_torque_permille: Option<f64>,
    pub applied_kd_si: f64,
    pub max_kd_si: f64,
    pub driver_temp_c: Option<f32>,
    pub motor_temp_c: Option<f32>,
    /// `Some(reason)` if the motor is in a fault state.
    pub error: Option<String>,
}

// ───────────────────────────── kinematics ───────────────────────────────────

/// Precomputed inverse/forward kinematics for the fixed geometry.
struct Kinematics {
    /// Inverse-kinematics matrix J (3×3): `wheel_speed = J · [vx,vy,wz]`.
    /// Row i = `[cosθ_i, sinθ_i, rx_i·sinθ_i − ry_i·cosθ_i]` (units: wheel
    /// contact linear speed m/s), with the per-wheel sign folded in.
    j: [[f64; 3]; 3],
    /// `J⁻¹` for odometry: `[vx,vy,wz] = J⁻¹ · wheel_speed`.
    j_inv: [[f64; 3]; 3],
}

impl Kinematics {
    fn new() -> Self {
        let centroid = [
            (CONTACTS_M[0][0] + CONTACTS_M[1][0] + CONTACTS_M[2][0]) / 3.0,
            (CONTACTS_M[0][1] + CONTACTS_M[1][1] + CONTACTS_M[2][1]) / 3.0,
        ];
        let origin = [
            centroid[0] + BODY_OFFSET_M[0],
            centroid[1] + BODY_OFFSET_M[1],
        ];

        let mut j = [[0.0f64; 3]; 3];
        for i in 0..3 {
            // Wheel position relative to the *body origin* (used for the moment arm).
            let rx = CONTACTS_M[i][0] - origin[0];
            let ry = CONTACTS_M[i][1] - origin[1];
            // Drive direction: tangential CCW about the *centroid* (a physical
            // mounting property, independent of where we put the body origin).
            let ang = (CONTACTS_M[i][1] - centroid[1]).atan2(CONTACTS_M[i][0] - centroid[0])
                + std::f64::consts::FRAC_PI_2;
            let (ct, st) = (ang.cos(), ang.sin());
            let s = WHEEL_SIGN[i];
            j[i] = [s * ct, s * st, s * (rx * st - ry * ct)];
        }
        let j_inv = invert3(&j).expect("HopeA3 kinematics matrix must be invertible");
        Self { j, j_inv }
    }

    /// Body twist → per-wheel velocity (output-shaft rev/s).
    fn twist_to_motor_rev_s(&self, vx: f64, vy: f64, wz: f64) -> [f64; 3] {
        let mut out = [0.0; 3];
        for i in 0..3 {
            let wheel_mps = self.j[i][0] * vx + self.j[i][1] * vy + self.j[i][2] * wz;
            out[i] = wheel_mps / (2.0 * std::f64::consts::PI * WHEEL_RADIUS_M);
        }
        out
    }

    /// Per-wheel velocity (output-shaft rev/s, from feedback) → body twist.
    fn motor_rev_s_to_twist(&self, motor_rev_s: [f64; 3]) -> (f64, f64, f64) {
        let mut wheel_mps = [0.0; 3];
        for i in 0..3 {
            wheel_mps[i] = motor_rev_s[i] * 2.0 * std::f64::consts::PI * WHEEL_RADIUS_M;
        }
        let vx = self.j_inv[0][0] * wheel_mps[0]
            + self.j_inv[0][1] * wheel_mps[1]
            + self.j_inv[0][2] * wheel_mps[2];
        let vy = self.j_inv[1][0] * wheel_mps[0]
            + self.j_inv[1][1] * wheel_mps[1]
            + self.j_inv[1][2] * wheel_mps[2];
        let wz = self.j_inv[2][0] * wheel_mps[0]
            + self.j_inv[2][1] * wheel_mps[1]
            + self.j_inv[2][2] * wheel_mps[2];
        (vx, vy, wz)
    }
}

/// Invert a 3×3 matrix (cofactor method). `None` if (near-)singular.
fn invert3(m: &[[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let mut out = [[0.0; 3]; 3];
    out[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
    out[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
    out[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
    Some(out)
}

// ───────────────────────────── the driver ───────────────────────────────────

/// A running HopeA3 chassis: owns the 500 Hz control/odom task.
pub struct Hopea3 {
    cmd: Arc<StdMutex<Command>>,
    state: Arc<StdMutex<Hopea3State>>,
    running: Arc<AtomicBool>,
    task: JoinHandle<()>,
    mgr: Arc<MeowMotorManager>,
    bus: Arc<dyn CanBus>,
    sdo_timeout: Option<Duration>,
    runtime: Arc<Mutex<Runtime>>,
}

#[derive(Clone, Copy)]
struct MotorConfig {
    peak_torque_nm: Option<f32>,
    kd_factor: f32,
    session_epoch: u64,
}

struct Runtime {
    motors: [Option<MotorConfig>; 3],
    mode_sent: [u8; 3],
    reset_motion: bool,
}

/// How many times to attempt each motor's init before giving up. Motor init
/// (especially the firmware's flaky heartbeat-fault clear) can fail
/// intermittently, so we retry per motor.
const INIT_ATTEMPTS: u8 = 3;

impl Hopea3 {
    /// Configure the three motors (identify + uncompressed-MIT shared host PDO +
    /// max torque) and start the 500 Hz control/odometry loop. The meow manager
    /// must already exist (created by `connect`); `bus` is the shared CAN
    /// transport the loop streams the shared frames on. `progress` is updated in
    /// place so the UI can show which motor is being initialized.
    pub async fn start(
        mgr: Arc<MeowMotorManager>,
        bus: Arc<dyn CanBus>,
        sdo_timeout: Option<Duration>,
        progress: &StdMutex<InitProgress>,
    ) -> anyhow::Result<Self> {
        let kin = Kinematics::new();
        let cmd = Arc::new(StdMutex::new(Command::default()));
        let state = Arc::new(StdMutex::new(Hopea3State::default()));

        let default_torque = cmd.lock().unwrap().max_torque;

        *progress.lock().unwrap() = InitProgress {
            active: true,
            current: 0,
            total: NODE_IDS.len() as u8,
            attempt: 0,
        };

        // Per-motor init, each retried up to INIT_ATTEMPTS times. Between
        // attempts we clear faults and wait, since the most common failure is
        // the firmware's phase-dependent heartbeat-fault clear.
        let mut motor_configs = [None; 3];
        for (slice, &nid) in NODE_IDS.iter().enumerate() {
            let mut last_err = None;
            for attempt in 1..=INIT_ATTEMPTS {
                {
                    let mut p = progress.lock().unwrap();
                    p.current = (slice + 1) as u8;
                    p.attempt = attempt;
                }
                match init_one_motor(&mgr, &bus, sdo_timeout, slice, nid, default_torque[slice])
                    .await
                {
                    Ok(config) => {
                        log::info!(
                            "HopeA3: motor 0x{nid:02X} ready, uncompressed MIT, KD factor {}",
                            config.kd_factor
                        );
                        motor_configs[slice] = Some(config);
                        last_err = None;
                        break;
                    }
                    Err(e) => {
                        log::warn!(
                            "HopeA3: motor 0x{nid:02X} init attempt {attempt}/{INIT_ATTEMPTS} failed: {e}"
                        );
                        last_err = Some(e);
                        let _ = mgr.clear_error(nid).await;
                        tokio::time::sleep(Duration::from_millis(300)).await;
                    }
                }
            }
            if let Some(e) = last_err {
                progress.lock().unwrap().active = false;
                return Err(e.context(format!(
                    "motor 0x{nid:02X} failed after {INIT_ATTEMPTS} attempts"
                )));
            }
        }
        progress.lock().unwrap().active = false;

        // 2) Control + odometry loop.
        let runtime = Arc::new(Mutex::new(Runtime {
            motors: motor_configs,
            mode_sent: [0; 3],
            reset_motion: false,
        }));
        let running = Arc::new(AtomicBool::new(true));
        let task = {
            let mgr = mgr.clone();
            let bus = bus.clone();
            let runtime = runtime.clone();
            let cmd = cmd.clone();
            let state = state.clone();
            let running = running.clone();
            tokio::spawn(async move {
                control_loop(mgr, bus, kin, runtime, cmd, state, running).await;
            })
        };

        Ok(Self {
            cmd,
            state,
            running,
            task,
            mgr,
            bus,
            sdo_timeout,
            runtime,
        })
    }

    /// Pause the chassis and reconfigure one motor. Resume only with a new
    /// drive command; shared frames cannot race with RPDO remapping.
    pub async fn reinit_motor(&self, nid: u8) -> anyhow::Result<()> {
        let slice = NODE_IDS
            .iter()
            .position(|&n| n == nid)
            .ok_or_else(|| anyhow::anyhow!("nid 0x{nid:02X} is not a HopeA3 motor"))?;
        let mut runtime = self.runtime.lock().await;
        self.set_cmd(0.0, 0.0, 0.0);
        runtime.reset_motion = true;
        runtime.motors[slice] = None;
        self.bus
            .send(CanFrame::new_fd(
                HOST_TPDO1_COB_ID,
                &[0; SLICE_LEN * 3],
                true,
            )?)
            .await?;
        self.bus
            .send(CanFrame::new_fd(HOST_TPDO2_COB_ID, &[0; 3], true)?)
            .await?;
        runtime.mode_sent = [0; 3];
        let torque = self.cmd.lock().unwrap().max_torque[slice];
        let mut last_err = None;
        for attempt in 1..=INIT_ATTEMPTS {
            match init_one_motor(&self.mgr, &self.bus, self.sdo_timeout, slice, nid, torque).await {
                Ok(config) => {
                    runtime.motors[slice] = Some(config);
                    return Ok(());
                }
                Err(e) => {
                    log::warn!("HopeA3: re-init 0x{nid:02X} attempt {attempt}: {e}");
                    last_err = Some(e);
                    let _ = self.mgr.clear_error(nid).await;
                    tokio::time::sleep(Duration::from_millis(300)).await;
                }
            }
        }
        Err(last_err
            .unwrap()
            .context(format!("re-init 0x{nid:02X} failed")))
    }

    /// Update the commanded twist (clamped to the current limits, never errored).
    pub fn set_cmd(&self, vx: f64, vy: f64, wz: f64) {
        let mut c = self.cmd.lock().unwrap();
        let (vx, vy, wz) = clamp_twist(vx, vy, wz, c.max_linear, c.max_angular);
        c.vx = vx;
        c.vy = vy;
        c.wz = wz;
    }

    /// Max torque is carried in each motor's PDO lane on the next tick.
    pub fn set_max_torque(&self, permille: [u16; 3]) {
        self.cmd.lock().unwrap().max_torque = permille.map(|p| p.min(1000));
    }

    /// Reject unrepresentable gains instead of silently clipping them.
    pub async fn set_kd(&self, kd_si: [f64; 3]) -> anyhow::Result<()> {
        let runtime = self.runtime.lock().await;
        for (i, kd) in kd_si.iter().enumerate() {
            let config = runtime.motors[i]
                .ok_or_else(|| anyhow::anyhow!("motor {} needs reinitialization", NODE_IDS[i]))?;
            kd_to_raw(*kd, config.kd_factor)?;
        }
        self.cmd.lock().unwrap().kd_si = kd_si;
        Ok(())
    }

    /// Update the acceleration (slew-rate) limits. `0` disables limiting for
    /// that axis (instant changes). Linear is m/s², angular rad/s².
    pub fn set_accel_limits(&self, max_lin_acc: f64, max_ang_acc: f64) {
        let mut c = self.cmd.lock().unwrap();
        c.max_lin_acc = max_lin_acc.max(0.0);
        c.max_ang_acc = max_ang_acc.max(0.0);
    }

    /// Update the velocity limits (re-clamps the current command).
    pub fn set_limits(&self, max_linear: f64, max_angular: f64) {
        let mut c = self.cmd.lock().unwrap();
        c.max_linear = max_linear.max(0.0);
        c.max_angular = max_angular.max(0.0);
        let (vx, vy, wz) = clamp_twist(c.vx, c.vy, c.wz, c.max_linear, c.max_angular);
        c.vx = vx;
        c.vy = vy;
        c.wz = wz;
    }

    /// Reset the dead-reckoned pose to the origin.
    pub fn reset_odom(&self) {
        let mut s = self.state.lock().unwrap();
        s.pose_x = 0.0;
        s.pose_y = 0.0;
        s.pose_theta = 0.0;
        s.peak_speed_mps = 0.0;
    }

    pub fn state(&self) -> Hopea3State {
        self.state.lock().unwrap().clone()
    }

    /// Stop the loop. The loop's exit path streams zero targets and disables all
    /// motors via the shared mode lane before returning.
    pub async fn stop(self) {
        self.running.store(false, Ordering::SeqCst);
        let _ = self.task.await;
    }
}

/// Configure the same noncompressed MIT objects as Motor Control, while
/// Disabled. Only the subsequent control loop may issue the MIT mode command.
async fn init_one_motor(
    mgr: &MeowMotorManager,
    bus: &Arc<dyn CanBus>,
    timeout: Option<Duration>,
    slice: usize,
    nid: u8,
    max_torque: u16,
) -> anyhow::Result<MotorConfig> {
    mgr.identify(nid).await?;
    mgr.initialize_with_options(
        nid,
        MeowMotorInitializeOptions {
            pdo_profile: PdoProfile::from_event_timer_ms(1)?,
            configure_consumer_heartbeat: true,
            shared_host_pdo: None,
        },
    )
    .await?;
    let info = mgr
        .list()
        .into_iter()
        .find(|i| i.node_id == nid)
        .ok_or_else(|| anyhow::anyhow!("motor 0x{nid:02X} identity missing"))?;
    let factor = info
        .mit_kp_kd_factor
        .filter(|v| v.is_finite() && *v > 0.0)
        .ok_or_else(|| {
            anyhow::anyhow!("motor 0x{nid:02X}: invalid/missing MIT gain factor 0x4102:07")
        })?;
    kd_to_raw(DEFAULT_KD_SI, factor)?;
    for w in uncompressed_setup_writes(slice)? {
        ensure_config_session(mgr, nid, info.session_epoch)?;
        sdo::download(&**bus, nid, w.index, w.subindex, &w.data, timeout)
            .await
            .map_err(|e| {
                anyhow::anyhow!("0x{nid:02X} write {:04X}:{:02X}: {e}", w.index, w.subindex)
            })?;
        tokio::time::sleep(Duration::from_millis(4)).await;
    }
    ensure_config_session(mgr, nid, info.session_epoch)?;
    anyhow::ensure!(
        sdo::upload_u8(&**bus, nid, 0x4103, 1, timeout).await? == 0,
        "motor 0x{nid:02X} still has compressed MIT enabled"
    );
    mgr.set_max_torque(nid, max_torque).await?;
    ensure_config_session(mgr, nid, info.session_epoch)?;
    Ok(MotorConfig {
        peak_torque_nm: info.peak_torque_nm.filter(|p| p.is_finite() && *p > 0.0),
        kd_factor: factor,
        session_epoch: info.session_epoch,
    })
}

fn ensure_config_session(mgr: &MeowMotorManager, nid: u8, epoch: u64) -> anyhow::Result<()> {
    let status = mgr.status(nid)?;
    anyhow::ensure!(
        status.session_epoch == epoch
            && status.lifecycle == MeowMotorLifecycle::Initialized
            && status.connection.online
            && status.logic == Some(MeowMotorLogic::Disabled),
        "motor 0x{nid:02X} left its initialized, disabled session"
    );
    Ok(())
}

fn velocity_rpdo_recipe(slice: usize) -> RpdoRecipe {
    let mut entries = Vec::new();
    for slot in 0..3 {
        if slot == slice {
            entries.extend([
                TpdoEntry {
                    index: 0x4102,
                    subindex: 2,
                    bit_len: 32,
                },
                TpdoEntry {
                    index: 0x4102,
                    subindex: 5,
                    bit_len: 16,
                },
                TpdoEntry {
                    index: 0x4572,
                    subindex: 0,
                    bit_len: 16,
                },
            ]);
        } else {
            entries.extend(
                [TpdoEntry {
                    index: 0x3000,
                    subindex: 3,
                    bit_len: 32,
                }; 2],
            );
        }
    }
    RpdoRecipe {
        rpdo_index: 0,
        cob_id: HOST_TPDO1_COB_ID,
        entries,
        transmission_type: 255,
    }
}

fn uncompressed_setup_writes(slice: usize) -> anyhow::Result<Vec<SdoWrite>> {
    let mut writes = vec![SdoWrite::u8(0x4103, 1, 0)];
    writes.extend(build_target_sdo_writes(MeowMotorTarget::Mit(
        MeowMitTarget {
            position_rev: 0.0,
            velocity_rev_per_s: 0.0,
            torque_nm: 0.0,
            kp: 0,
            kd: 0,
            kp_kd_limit_permille: 1000,
        },
    ))?);
    writes.extend(build_rpdo_config_writes(&velocity_rpdo_recipe(slice))?);
    writes.extend(build_rpdo_config_writes(&mode_rpdo2_recipe(
        slice as u8,
        3,
        HOST_NODE_ID,
    )?)?);
    Ok(writes)
}

fn kd_to_raw(kd_si: f64, factor: f32) -> anyhow::Result<u16> {
    anyhow::ensure!(
        factor.is_finite() && factor > 0.0,
        "invalid MIT gain factor"
    );
    anyhow::ensure!(
        kd_si.is_finite() && kd_si >= 0.0,
        "KD must be finite and nonnegative"
    );
    let raw = (kd_si * std::f64::consts::TAU / f64::from(factor)).round();
    anyhow::ensure!(
        raw <= u16::MAX as f64,
        "KD exceeds motor limit {:.4} Nm·s/rad",
        u16::MAX as f64 * f64::from(factor) / std::f64::consts::TAU
    );
    Ok(raw as u16)
}

fn pack_velocity_frame(
    velocities: [f64; 3],
    kd: [u16; 3],
    torque: [u16; 3],
) -> [u8; SLICE_LEN * 3] {
    let mut bytes = [0; SLICE_LEN * 3];
    for i in 0..3 {
        let lane = &mut bytes[i * SLICE_LEN..(i + 1) * SLICE_LEN];
        lane[..4].copy_from_slice(&(velocities[i] as f32).to_le_bytes());
        lane[4..6].copy_from_slice(&kd[i].to_le_bytes());
        lane[6..8].copy_from_slice(&torque[i].to_le_bytes());
    }
    bytes
}

fn estimated_torque_permille(
    target_rev_s: f32,
    velocity_rev_s: Option<f32>,
    kd_si: f64,
    peak_torque_nm: Option<f32>,
    max_torque_permille: u16,
) -> Option<f64> {
    let velocity = f64::from(velocity_rev_s?);
    let peak = f64::from(peak_torque_nm?);
    if !velocity.is_finite()
        || !target_rev_s.is_finite()
        || !kd_si.is_finite()
        || kd_si < 0.0
        || !peak.is_finite()
        || peak <= 0.0
    {
        return None;
    }
    let demand = kd_si * std::f64::consts::TAU * (f64::from(target_rev_s) - velocity);
    let limit = f64::from(max_torque_permille.min(1000));
    Some((demand / peak * 1000.0).clamp(-limit, limit))
}

/// Best-effort fault clear on all three motors. Useful before starting if a
/// previous run left them in a heartbeat-lost / fault state (motors that were
/// never identified in this session are skipped; `start` clears their faults
/// during init anyway).
pub async fn clear_errors(mgr: &MeowMotorManager) {
    for &nid in &NODE_IDS {
        if let Err(e) = mgr.clear_error(nid).await {
            log::warn!("HopeA3: clear_error 0x{nid:02X}: {e}");
        }
    }
}

fn clamp_twist(vx: f64, vy: f64, wz: f64, max_linear: f64, max_angular: f64) -> (f64, f64, f64) {
    let mag = (vx * vx + vy * vy).sqrt();
    let (vx, vy) = if mag > max_linear {
        let s = max_linear / mag;
        (vx * s, vy * s)
    } else {
        (vx, vy)
    };
    let wz = wz.clamp(-max_angular, max_angular);
    (vx, vy, wz)
}

async fn control_loop(
    mgr: Arc<MeowMotorManager>,
    bus: Arc<dyn CanBus>,
    kin: Kinematics,
    runtime: Arc<Mutex<Runtime>>,
    cmd: Arc<StdMutex<Command>>,
    state: Arc<StdMutex<Hopea3State>>,
    running: Arc<AtomicBool>,
) {
    let period = Duration::from_micros(1_000_000 / CONTROL_HZ);
    let dt = 1.0 / CONTROL_HZ as f64;
    let mut tick = tokio::time::interval(period);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    // Applied (rate-limited) twist, ramped toward the target each tick.
    let (mut ax, mut ay, mut aw) = (0.0f64, 0.0f64, 0.0f64);

    while running.load(Ordering::SeqCst) {
        tick.tick().await;

        let mut runtime = runtime.lock().await;
        if runtime.reset_motion {
            (ax, ay, aw) = (0.0, 0.0, 0.0);
            runtime.reset_motion = false;
        }
        let c = *cmd.lock().unwrap();
        let mut kd_raw = [0; 3];

        // ── read wheel feedback first (so a fault can gate this tick's targets) ──
        let mut motor_rev_s = [0.0f64; 3];
        let mut motors = Vec::with_capacity(3);
        let mut any_error = false;
        let mut feedback_valid = true;
        for (i, &nid) in NODE_IDS.iter().enumerate() {
            let mut torque_permille = None;
            let mut session_error = None;
            let (online, vel, torque_nm, driver_temp, motor_temp, enabled, mut error) = match mgr
                .status(nid)
            {
                Ok(ls) => {
                    let m = &ls.measurements;
                    let vel = m.velocity_rev_per_s;
                    torque_permille = m.torque_permille;
                    let torque_nm = m.torque_permille.and_then(|p| {
                        runtime.motors[i]?
                            .peak_torque_nm
                            .map(|peak| p as f32 / 1000.0 * peak)
                    });
                    if !ls.connection.online
                        || !ls.lifecycle.is_ready()
                        || !runtime.motors[i]
                            .is_some_and(|config| config.session_epoch == ls.session_epoch)
                    {
                        any_error = true;
                        session_error = Some("Motor offline or needs reinitialization".to_owned());
                    }
                    let (enabled, error) = match ls.logic {
                        Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit)) => (true, None),
                        Some(MeowMotorLogic::Error {
                            mode_display,
                            detailed_error,
                        }) => {
                            any_error = true;
                            (
                                false,
                                Some(format!("0x{mode_display:02X}:0x{detailed_error:04X}")),
                            )
                        }
                        Some(MeowMotorLogic::Disabled) => (false, None),
                        _ => {
                            any_error = true;
                            (false, Some("Unexpected motor mode".to_owned()))
                        }
                    };
                    (
                        ls.connection.online,
                        vel,
                        torque_nm,
                        m.driver_temp_c,
                        m.motor_temp_c,
                        enabled,
                        error,
                    )
                }
                Err(e) => {
                    log::debug!("HopeA3: status 0x{nid:02X}: {e}");
                    (false, None, None, None, None, false, Some(e.to_string()))
                }
            };
            error = error.or(session_error);
            if !online || error.is_some() || vel.is_none_or(|v| !v.is_finite()) {
                any_error = true;
                feedback_valid = false;
            }
            let factor = runtime.motors[i].map(|config| config.kd_factor);
            match factor.and_then(|factor| kd_to_raw(c.kd_si[i], factor).ok()) {
                Some(raw) => kd_raw[i] = raw,
                None => any_error = true,
            }
            motor_rev_s[i] = WHEEL_SIGN[i] * vel.filter(|v| v.is_finite()).unwrap_or(0.0);
            motors.push(Hopea3Motor {
                node_id: nid,
                online,
                enabled,
                target_rev_per_s: 0.0, // filled after kinematics below
                velocity_rev_per_s: vel.map(|v| v as f32),
                torque_nm,
                max_torque_permille: c.max_torque[i],
                torque_permille,
                estimated_torque_permille: None,
                applied_kd_si: factor
                    .map_or(0.0, |f| kd_raw[i] as f64 * f as f64 / std::f64::consts::TAU),
                max_kd_si: factor
                    .map_or(0.0, |f| u16::MAX as f64 * f as f64 / std::f64::consts::TAU),
                driver_temp_c: driver_temp,
                motor_temp_c: motor_temp,
                error,
            });
        }

        // ── chassis acceleration limiting (slew-rate) ──
        // If ANY motor is faulted, stop the whole chassis: zero the applied
        // twist so every wheel (including the healthy ones) gets VDES=0. This
        // also avoids ramping back up the instant the fault clears.
        if any_error {
            ax = 0.0;
            ay = 0.0;
            aw = 0.0;
            let mut requested = cmd.lock().unwrap();
            (requested.vx, requested.vy, requested.wz) = (0.0, 0.0, 0.0);
        } else {
            // Linear: bound the velocity-*vector* step to max_lin_acc·dt, which
            // limits speed and heading change together. Angular: bound |Δωz|.
            // A zero limit means "instant" (snap to target).
            let (tx, ty, tw) = (c.vx, c.vy, c.wz);
            if c.max_lin_acc > 0.0 {
                let (dx, dy) = (tx - ax, ty - ay);
                let dist = (dx * dx + dy * dy).sqrt();
                let step = c.max_lin_acc * dt;
                if dist > step {
                    let s = step / dist;
                    ax += dx * s;
                    ay += dy * s;
                } else {
                    ax = tx;
                    ay = ty;
                }
            } else {
                ax = tx;
                ay = ty;
            }
            if c.max_ang_acc > 0.0 {
                let dw = tw - aw;
                let step = c.max_ang_acc * dt;
                aw += dw.clamp(-step, step);
            } else {
                aw = tw;
            }
        }

        // Uncompressed velocity avoids the former ±10 rev/s quantizer clamp.
        let targets = kin.twist_to_motor_rev_s(ax, ay, aw);
        for i in 0..3 {
            motors[i].target_rev_per_s = targets[i] as f32;
            motors[i].estimated_torque_permille = if feedback_valid && !any_error {
                estimated_torque_permille(
                    motors[i].target_rev_per_s,
                    motors[i].velocity_rev_per_s,
                    motors[i].applied_kd_si,
                    runtime.motors[i].and_then(|config| config.peak_torque_nm),
                    c.max_torque[i],
                )
            } else {
                None
            };
        }
        let data = pack_velocity_frame(targets, kd_raw, c.max_torque);
        let mut target_sent = false;
        match CanFrame::new_fd(HOST_TPDO1_COB_ID, &data, true) {
            Ok(frame) => {
                if let Err(e) = bus.send(frame).await {
                    log::warn!("HopeA3: host TPDO1 send failed: {e}");
                } else {
                    target_sent = true;
                }
            }
            Err(e) => log::error!("HopeA3: build host TPDO1 frame: {e}"),
        }

        // ── mode lane: switch all three into MIT on change ──
        let desired_mode = if any_error || !target_sent {
            [0; 3]
        } else {
            [MODE_MIT; 3]
        };
        let mode_changed = runtime.mode_sent != desired_mode;
        if mode_changed {
            match CanFrame::new_fd(HOST_TPDO2_COB_ID, &desired_mode, true) {
                Ok(frame) => {
                    if let Err(e) = bus.send(frame).await {
                        log::warn!("HopeA3: host TPDO2 send failed: {e}");
                    } else {
                        runtime.mode_sent = desired_mode;
                    }
                }
                Err(e) => log::error!("HopeA3: build host TPDO2 frame: {e}"),
            }
        }

        // ── forward kinematics from wheel feedback → twist + odom ──
        let (mvx, mvy, mwz) = kin.motor_rev_s_to_twist(motor_rev_s);

        {
            let mut s = state.lock().unwrap();
            // Dead-reckon in the world frame (Euler integration of body twist).
            let th = s.pose_theta;
            s.pose_x += (mvx * th.cos() - mvy * th.sin()) * dt;
            s.pose_y += (mvx * th.sin() + mvy * th.cos()) * dt;
            s.pose_theta = wrap_pi(s.pose_theta + mwz * dt);
            s.meas_vx = mvx;
            s.meas_vy = mvy;
            s.meas_wz = mwz;
            // Report the applied (rate-limited) twist that's actually driving.
            s.cmd_vx = ax;
            s.cmd_vy = ay;
            s.cmd_wz = aw;
            s.feedback_valid = feedback_valid && !any_error;
            if s.feedback_valid {
                s.peak_speed_mps = s.peak_speed_mps.max(mvx.hypot(mvy));
            }
            s.max_linear = c.max_linear;
            s.max_angular = c.max_angular;
            s.motors = motors;
            s.running = true;
        }
    }

    // Final safety: zero every lane and disable every motor before exiting.
    let zero = [0; SLICE_LEN * 3];
    if let Ok(frame) = CanFrame::new_fd(HOST_TPDO1_COB_ID, &zero, true) {
        let _ = bus.send(frame).await;
    }
    if let Ok(frame) = CanFrame::new_fd(HOST_TPDO2_COB_ID, &[0u8, 0, 0], true) {
        let _ = bus.send(frame).await;
    }

    state.lock().unwrap().running = false;
    log::info!("HopeA3: control loop stopped");
}

fn wrap_pi(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let mut a = a % tau;
    if a > std::f64::consts::PI {
        a -= tau;
    } else if a < -std::f64::consts::PI {
        a += tau;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use hex_motor::meow_motor::host_tpdo_cob_id;

    #[test]
    fn canopen_allocation_matches_global_contract() {
        assert_eq!(HOST_NODE_ID, 10);
        assert_eq!(NODE_IDS, [21, 22, 23]);
        assert_eq!(HOST_TPDO1_COB_ID, 0x18A);
        assert_eq!(HOST_TPDO2_COB_ID, 0x28A);
        // The shared-frame COB-IDs must match the meow driver's derivation.
        assert_eq!(host_tpdo_cob_id(10, 0).unwrap(), HOST_TPDO1_COB_ID);
        assert_eq!(host_tpdo_cob_id(10, 1).unwrap(), HOST_TPDO2_COB_ID);
    }

    #[test]
    fn wheel_diameter_is_154_mm() {
        assert!((2.0 * WHEEL_RADIUS_M - 0.154).abs() < f64::EPSILON);
    }

    #[test]
    fn wheel_centre_circle_diameter_is_648_mm() {
        let centroid = [
            CONTACTS_M.iter().map(|point| point[0]).sum::<f64>() / 3.0,
            CONTACTS_M.iter().map(|point| point[1]).sum::<f64>() / 3.0,
        ];

        for point in CONTACTS_M {
            let distance =
                ((point[0] - centroid[0]).powi(2) + (point[1] - centroid[1]).powi(2)).sqrt();
            assert!((2.0 * distance - 0.648).abs() < 1e-12);
        }
    }

    #[test]
    fn noncompressed_setup_clears_old_mode_and_targets_before_mapping() {
        let writes = uncompressed_setup_writes(1).unwrap();
        assert_eq!(writes[0], SdoWrite::u8(0x4103, 1, 0));
        assert_eq!(writes[1], SdoWrite::f32(0x4102, 1, 0.0));
        assert_eq!(writes[2], SdoWrite::f32(0x4102, 2, 0.0));
        assert_eq!(writes[3], SdoWrite::f32(0x4102, 3, 0.0));
        assert_eq!(writes[4], SdoWrite::u16(0x4102, 4, 0));
        assert_eq!(writes[5], SdoWrite::u16(0x4102, 5, 0));
        assert_eq!(writes[6], SdoWrite::u16(0x4102, 6, 1000));
        assert!(!writes.iter().any(|w| w.index == 0x4401)); // enable is deferred
    }

    #[test]
    fn high_speed_wire_payload_matches_each_motors_mapping() {
        let velocities = [-41.3389, 0.0, 41.3389];
        let kd = [10, 200, 3000];
        let torque = [100, 500, 800];
        let data = pack_velocity_frame(velocities, kd, torque);
        for i in 0..3 {
            let recipe = velocity_rpdo_recipe(i);
            recipe.validate().unwrap();
            assert_eq!(recipe.total_bytes(), data.len());
            let mut offset = 0;
            for entry in recipe.entries {
                let length = entry.bit_len as usize / 8;
                let value = &data[offset..offset + length];
                match (entry.index, entry.subindex) {
                    (0x4102, 2) => assert_eq!(
                        f32::from_le_bytes(value.try_into().unwrap()),
                        velocities[i] as f32
                    ),
                    (0x4102, 5) => assert_eq!(u16::from_le_bytes(value.try_into().unwrap()), kd[i]),
                    (0x4572, 0) => {
                        assert_eq!(u16::from_le_bytes(value.try_into().unwrap()), torque[i])
                    }
                    (0x3000, 3) => {}
                    _ => panic!("unexpected mapping"),
                }
                offset += length;
            }
        }
    }

    #[test]
    fn kd_uses_each_motors_scale_and_rejects_overflow() {
        assert_eq!(kd_to_raw(1.0, 0.01).unwrap(), 628);
        assert_eq!(kd_to_raw(1.0, 0.02).unwrap(), 314);
        assert!(kd_to_raw(4.0, 0.01).unwrap() > 2000); // beyond old compressed cap
        for factor in [0.0, -1.0, f32::NAN] {
            assert!(kd_to_raw(1.0, factor).is_err());
        }
        for kd in [f64::NAN, f64::INFINITY, -1.0, 1e9] {
            assert!(kd_to_raw(kd, 0.01).is_err());
        }
    }

    #[test]
    fn twenty_mps_forward_requires_about_35_8_wheel_revs_per_second() {
        let kin = Kinematics::new();
        let targets = kin.twist_to_motor_rev_s(20.0, 0.0, 0.0);
        assert!((targets[0] + 35.8005776248).abs() < 1e-6);
        assert!(targets[1].abs() < 1e-10);
        assert!((targets[2] - 35.8005776248).abs() < 1e-6);
        for twist in [(20.0, 0.0, 0.0), (0.0, 20.0, 0.0), (2.0, -1.0, 3.0)] {
            let back =
                kin.motor_rev_s_to_twist(kin.twist_to_motor_rev_s(twist.0, twist.1, twist.2));
            assert!((back.0 - twist.0).abs() < 1e-10);
            assert!((back.1 - twist.1).abs() < 1e-10);
            assert!((back.2 - twist.2).abs() < 1e-10);
        }
    }

    #[test]
    fn zero_linear_limit_stops_translation() {
        assert_eq!(clamp_twist(2.0, -3.0, 0.0, 0.0, 1.0), (0.0, -0.0, 0.0));
    }

    #[test]
    fn default_linear_limit_accepts_twenty_mps_without_reducing_wheel_targets() {
        let defaults = Command::default();
        assert_eq!(
            clamp_twist(20.0, 0.0, 0.0, defaults.max_linear, defaults.max_angular),
            (20.0, 0.0, 0.0)
        );
    }

    #[test]
    fn estimated_mit_demand_distinguishes_small_error_from_output_saturation() {
        let saturated = estimated_torque_permille(38.0, Some(3.8), 1.0, Some(8.658), 800);
        assert_eq!(saturated, Some(800.0));
        assert_eq!(
            estimated_torque_permille(-38.0, Some(-3.8), 1.0, Some(8.658), 800),
            Some(-800.0)
        );
        let small = estimated_torque_permille(4.0, Some(3.8), 1.0, Some(8.658), 800).unwrap();
        assert!((small - 145.137).abs() < 0.01);
        assert_eq!(
            estimated_torque_permille(38.0, Some(3.8), 0.0, Some(8.658), 800),
            Some(0.0)
        );
        assert_eq!(
            estimated_torque_permille(38.0, None, 1.0, Some(8.658), 800),
            None
        );
        assert_eq!(
            estimated_torque_permille(38.0, Some(3.8), 1.0, Some(0.0), 800),
            None
        );
    }
}
