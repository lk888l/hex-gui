//! HopeA3 — first **Robot Application**: an inverted-triangle 3-wheel omni base.
//!
//! Unlike Direct Control (discover unknown motors, drive each over SDO), a
//! Robot Application has a *fixed* motor count and layout, so it can use PDO
//! for control. HopeA3 drives all three HexMeow motors in **uncompressed MIT** mode
//! used as a direct velocity loop: with KP=0 and PDES unused, the motor's torque
//! law reduces to `τ = KD·(VDES − v) + TFF`, clamped by max torque — no profile
//! ramp (which is why this replaced PV). Each motor's **RPDO1** maps its MIT
//! velocity `0x4102:02` (f32 rev/s) + KD `0x4102:05` (u16) + max torque `0x4572`
//! (u16, ‰). The master streams targets at **500 Hz** as **one shared CAN-FD
//! frame** that all three motors receive — each reads its own 8-byte slice, the
//! other slices are consumed by placeholder mappings. This mirrors the
//! single-frame technique from a proven reference implementation.
//!
//! Feedback / odometry reuse the manager's existing TPDO1 stream (it already
//! parses position / host-filtered velocity / torque per motor); we read
//! `mgr.status(nid)` and run forward kinematics + dead-reckoning at the control
//! rate.
//!
//! Geometry, drivetrain and the installed motor type are **compile-time
//! constants** below (the chassis is not reconfigurable at runtime — only the
//! velocity/torque limits and the command are).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context};
use can_transport::{CanBus, CanFrame};
use hex_motor::canopen::rpdo_config::{build_rpdo_config_writes, RpdoRecipe};
use hex_motor::canopen::sdo;
use hex_motor::canopen::tpdo_config::TpdoEntry;
use hex_motor::cia402::PdoProfile;
use hex_motor::meow_motor::{
    MeowMitTarget, MeowMotorCanSettingsStatus, MeowMotorLifecycle, MeowMotorLiveState,
    MeowMotorLogic, MeowMotorManager, MeowMotorMode, MeowMotorTarget,
};
use serde::Serialize;
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
pub(crate) const NODE_IDS: [u8; 3] = [21, 22, 23];

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

/// Shared CAN-FD COB-ID that all three motors listen to as RPDO1.
/// Uses the global host-10 TPDO1 convention (`0x180 + 0x0A = 0x18A`).
const SHARED_RPDO_COB_ID: u16 = 0x180 + HOST_NODE_ID as u16;

/// Control / odometry loop rate.
const CONTROL_HZ: u64 = 500;

/// Bytes per motor in the shared frame: uncompressed-MIT velocity
/// `0x4102:02`(4) + KD `0x4102:05`(2) + max torque `0x4572`(2) = 8.
const SLICE_LEN: usize = 8;

/// Uncompressed-MIT control parameter object (`0x4102`). With KP=0 and PDES
/// unused, torque = KD·(VDES − v) + TFF, clamped by max torque (`0x4572`) — a
/// direct velocity loop with no profile ramp (unlike PV).
const OD_MIT: u16 = 0x4102;
const MIT_SUB_VDES: u8 = 0x02; // f32 Rev/s    (velocity target, streamed)
const MIT_SUB_KD: u8 = 0x05; // u16 0..=65535 (velocity gain, streamed)
const OD_MAX_TORQUE: u16 = 0x4572; // u16 ‰ of raw peak torque
const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(200);

/// Placeholder mapping object for the bytes belonging to *other* motors. Using
/// the same vendor filler as the SDK's HexMeow shared Host PDO (`0x3000:03`,
/// 32-bit).
/// One per 4 bytes ⇒ two per 8-byte slice.
const PAD_ENTRY: TpdoEntry = TpdoEntry {
    index: 0x3000,
    subindex: 3,
    bit_len: 32,
};

/// Default limits / torque (all runtime-adjustable except being clamped, never
/// errored, when the UI sends more).
pub const DEFAULT_MAX_TORQUE_PERMILLE: u16 = 800;
pub const DEFAULT_MAX_LINEAR_MPS: f64 = 3.0;
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
    /// Follow sessions refresh this in the native loop; standalone manual
    /// control leaves it unset. An expired producer cannot retain wheel speed.
    deadline: Option<Instant>,
    /// Body-frame twist (m/s, m/s, rad/s). Stored already clamped.
    vx: f64,
    vy: f64,
    wz: f64,
    /// Per-motor max torque (‰ of peak), indexed like [`NODE_IDS`].
    max_torque: [u16; 3],
    /// Per-motor MIT velocity gain KD in **SI units (Nm·s/rad)**, indexed like
    /// [`NODE_IDS`]. Converted to the wire u16 (Nm·s/Rev ÷ factor) in the loop.
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
            deadline: None,
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
    /// Commanded (clamped) body twist.
    pub cmd_vx: f64,
    pub cmd_vy: f64,
    pub cmd_wz: f64,
    pub max_linear: f64,
    pub max_angular: f64,
    pub motors: Vec<Hopea3Motor>,
    pub running: bool,
    /// Motion is inhibited while feedback/mode is invalid or CAN sending fails.
    pub fault: Option<String>,
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

    /// Body twist → per-wheel velocity (output-shaft rev/s, `0x4102:02`).
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
    task: Option<JoinHandle<()>>,
    mgr: Arc<MeowMotorManager>,
    bus: Arc<dyn CanBus>,
    sdo_timeout: Option<Duration>,
    parameters: Arc<StdMutex<[MotorParameters; 3]>>,
    reinitializing: Arc<AtomicBool>,
}

#[derive(Debug, Clone, Copy, Default)]
struct MotorParameters {
    kd_factor: f32,
    peak_torque_nm: Option<f32>,
    torque_factor: f64,
    session_epoch: u64,
    nmt_revision: u64,
    brs: bool,
}

/// How many times to attempt each motor's init before giving up. Motor init
/// (especially the firmware's flaky heartbeat-fault clear) can fail
/// intermittently, so we retry per motor.
const INIT_ATTEMPTS: u8 = 3;

impl Hopea3 {
    /// Configure the three HexMeow motors (init + RPDO mapping + zero MIT target) and
    /// start the 500 Hz control/odometry loop. The shared bus must already have
    /// host-10 heartbeat broadcast on. `progress` is updated in place so
    /// the UI can show which motor is being initialized.
    pub async fn start(
        mgr: Arc<MeowMotorManager>,
        bus: Arc<dyn CanBus>,
        sdo_timeout: Option<Duration>,
        progress: &StdMutex<InitProgress>,
        shutdown: &AtomicBool,
    ) -> anyhow::Result<Self> {
        anyhow::ensure!(
            bus.capabilities().fd && bus.capabilities().max_dlen >= 24,
            "HopeA3 requires CAN-FD with a 24-byte shared PDO"
        );
        let kin = Kinematics::new();
        let cmd = Arc::new(StdMutex::new(Command::default()));
        let state = Arc::new(StdMutex::new(Hopea3State::default()));

        *progress.lock().unwrap() = InitProgress {
            active: true,
            current: 0,
            total: NODE_IDS.len() as u8,
            attempt: 0,
        };

        // Retry transient per-motor failures. Each attempt uses SDK initialization
        // to clear existing errors and confirm Disabled before configuring MIT.
        let initialized = async {
            let mut parameters = [MotorParameters::default(); 3];
            for (slice, &nid) in NODE_IDS.iter().enumerate() {
                parameters[slice] = init_with_retry(
                    &mgr,
                    &bus,
                    sdo_timeout,
                    slice,
                    DEFAULT_KD_SI,
                    shutdown,
                    Some(progress),
                )
                .await
                .with_context(|| format!("HexMeow motor 0x{nid:02X} initialization failed"))?;
            }
            check_shutdown(shutdown)?;
            Ok::<_, anyhow::Error>(parameters)
        }
        .await;
        progress.lock().unwrap().active = false;
        let parameters = match initialized {
            Ok(parameters) => parameters,
            Err(error) => {
                // Every newly enabled motor still has KD=0, TFF=0 and max
                // torque=0. Also disable all exact-identified chassis nodes.
                let known: Vec<u8> = mgr
                    .list()
                    .iter()
                    .filter(|info| NODE_IDS.contains(&info.node_id) && info.model.is_some())
                    .map(|info| info.node_id)
                    .collect();
                if let Err(cleanup) = disable_motors(&mgr, &known).await {
                    return Err(error.context(format!("startup rollback also failed: {cleanup}")));
                }
                return Err(error);
            }
        };

        // 2) Control + odometry loop.
        let parameters = Arc::new(StdMutex::new(parameters));
        let reinitializing = Arc::new(AtomicBool::new(false));
        let running = Arc::new(AtomicBool::new(true));
        state.lock().unwrap().running = true;
        let task = {
            let mgr = mgr.clone();
            let bus = bus.clone();
            let parameters = parameters.clone();
            let reinitializing = reinitializing.clone();
            let cmd = cmd.clone();
            let state = state.clone();
            let running = running.clone();
            tokio::spawn(async move {
                control_loop(
                    mgr,
                    bus,
                    kin,
                    parameters,
                    cmd,
                    state,
                    running,
                    reinitializing,
                )
                .await;
            })
        };

        Ok(Self {
            cmd,
            state,
            running,
            task: Some(task),
            mgr,
            bus,
            sdo_timeout,
            parameters,
            reinitializing,
        })
    }

    /// Reinitialize one motor with all three wheels held at zero output.
    /// Refresh calibration/session fences and discard the old motion command.
    pub async fn reinit_motor(&self, nid: u8, shutdown: &AtomicBool) -> anyhow::Result<()> {
        let slice = NODE_IDS
            .iter()
            .position(|&n| n == nid)
            .ok_or_else(|| anyhow::anyhow!("nid 0x{nid:02X} is not a HopeA3 motor"))?;
        anyhow::ensure!(
            self.running.load(Ordering::SeqCst),
            "stop HopeA3 before restarting"
        );
        self.reinitializing.store(true, Ordering::SeqCst);
        let kd = {
            let mut c = self.cmd.lock().unwrap();
            c.vx = 0.0;
            c.vy = 0.0;
            c.wz = 0.0;
            c.kd_si[slice]
        };
        let result = init_with_retry(
            &self.mgr,
            &self.bus,
            self.sdo_timeout,
            slice,
            kd,
            shutdown,
            None,
        )
        .await;
        let result = match result {
            Ok(parameters) => {
                self.parameters.lock().unwrap()[slice] = parameters;
                Ok(())
            }
            Err(error) => match self.mgr.disable(nid).await {
                Ok(()) => Err(error),
                Err(cleanup) => {
                    Err(error.context(format!("disable after failed re-init: {cleanup}")))
                }
            },
        };
        self.reinitializing.store(false, Ordering::SeqCst);
        result
    }

    /// Update the commanded twist (clamped to the current limits, never errored).
    pub fn set_cmd(&self, vx: f64, vy: f64, wz: f64) {
        self.update_command(vx, vy, wz, false);
    }

    pub fn set_follow_cmd(&self, vx: f64, vy: f64, wz: f64) {
        self.update_command(vx, vy, wz, true);
    }

    fn update_command(&self, vx: f64, vy: f64, wz: f64, follow: bool) {
        let mut c = self.cmd.lock().unwrap();
        if follow {
            c.deadline = Some(Instant::now() + Duration::from_millis(150));
        }
        let (vx, vy, wz) = if self.reinitializing.load(Ordering::SeqCst)
            || ![vx, vy, wz].iter().all(|v| v.is_finite())
        {
            (0.0, 0.0, 0.0)
        } else {
            (vx, vy, wz)
        };
        let (vx, vy, wz) = clamp_twist(vx, vy, wz, c.max_linear, c.max_angular);
        c.vx = vx;
        c.vy = vy;
        c.wz = wz;
    }

    /// Update per-motor max torque (‰), pushed to the stream on the next tick.
    pub fn set_max_torque(&self, permille: [u16; 3]) {
        let mut c = self.cmd.lock().unwrap();
        c.max_torque = permille.map(|p| p.min(1000));
    }

    /// Update per-motor MIT velocity gain KD (SI, Nm·s/rad), streamed next tick.
    pub fn set_kd(&self, kd_si: [f64; 3]) -> anyhow::Result<()> {
        let parameters = *self.parameters.lock().unwrap();
        for i in 0..3 {
            kd_to_raw(kd_si[i], parameters[i].kd_factor)?;
        }
        let mut c = self.cmd.lock().unwrap();
        c.kd_si = kd_si;
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
    }

    pub fn state(&self) -> Hopea3State {
        self.state.lock().unwrap().clone()
    }

    /// Stop the loop, zero the targets and disable all motors.
    pub async fn stop(&mut self) -> anyhow::Result<()> {
        self.running.store(false, Ordering::SeqCst);
        if let Some(task) = self.task.take() {
            if let Err(error) = task.await {
                log::warn!("HopeA3: control task ended unexpectedly: {error}");
            }
        }
        let brs = self.parameters.lock().unwrap().iter().all(|p| p.brs);
        // Even if the task panicked or a previous stop failed, retry a zero
        // frame before SDK-confirmed 0x4401=0 on every motor.
        let _ = self
            .bus
            .send(shared_frame([0.0; 3], [0; 3], [0; 3], brs))
            .await;
        let result = disable_motors(&self.mgr, &NODE_IDS).await;
        let mut state = self.state.lock().unwrap();
        state.running = false;
        state.cmd_vx = 0.0;
        state.cmd_vy = 0.0;
        state.cmd_wz = 0.0;
        for motor in &mut state.motors {
            motor.target_rev_per_s = 0.0;
            if let Ok(live) = self.mgr.status(motor.node_id) {
                motor.enabled = live.logic == Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit));
            }
        }
        state.fault = result
            .as_ref()
            .err()
            .map(|error| format!("Stop failed: {error}; retry Stop"));
        result
    }
}

impl Drop for Hopea3 {
    fn drop(&mut self) {
        // Never leave a detached task streaming motion after its owner goes away.
        self.running.store(false, Ordering::SeqCst);
    }
}

fn check_shutdown(shutdown: &AtomicBool) -> anyhow::Result<()> {
    anyhow::ensure!(
        !shutdown.load(Ordering::SeqCst),
        "HopeA3 initialization cancelled by shutdown"
    );
    Ok(())
}

async fn init_with_retry(
    mgr: &MeowMotorManager,
    bus: &Arc<dyn CanBus>,
    sdo_timeout: Option<Duration>,
    slice: usize,
    kd_si: f64,
    shutdown: &AtomicBool,
    progress: Option<&StdMutex<InitProgress>>,
) -> anyhow::Result<MotorParameters> {
    let nid = NODE_IDS[slice];
    for attempt in 1..=INIT_ATTEMPTS {
        check_shutdown(shutdown)?;
        if let Some(progress) = progress {
            let mut p = progress.lock().unwrap();
            p.current = (slice + 1) as u8;
            p.attempt = attempt;
        }
        match init_one_motor(mgr, bus, sdo_timeout, slice, kd_si, shutdown).await {
            Ok(parameters) => return Ok(parameters),
            Err(error) => {
                log::warn!("HopeA3: HexMeow 0x{nid:02X} init {attempt}/{INIT_ATTEMPTS}: {error:#}");
                if attempt == INIT_ATTEMPTS || shutdown.load(Ordering::SeqCst) {
                    return Err(error);
                }
                // initialize() itself clears errors and confirms Disabled.
                tokio::time::sleep(Duration::from_millis(300)).await;
            }
        }
    }
    unreachable!()
}

/// SDK identity/TPDO initialization stays shared with Motor Control. Only the
/// application-specific RPDO1 layout and explicit uncompressed selector differ.
async fn init_one_motor(
    mgr: &MeowMotorManager,
    bus: &Arc<dyn CanBus>,
    sdo_timeout: Option<Duration>,
    slice: usize,
    kd_si: f64,
    shutdown: &AtomicBool,
) -> anyhow::Result<MotorParameters> {
    let nid = NODE_IDS[slice];
    mgr.identify(nid)
        .await
        .context("identify exact HexMeow 0x1018 identity")?;
    check_shutdown(shutdown)?;
    mgr.initialize(
        nid,
        PdoProfile::from_event_timer_ms((1000 / CONTROL_HZ) as u16)?,
    )
    .await
    .context("HexMeow initialize")?;

    let info = mgr
        .list()
        .into_iter()
        .find(|info| info.node_id == nid)
        .context("initialized HexMeow node disappeared")?;
    let kd_factor = info
        .mit_kp_kd_factor
        .context("missing HexMeow 0x4102:07 Kp/Kd factor")?;
    kd_to_raw(kd_si, kd_factor)?;
    let calibration = crate::meow_calibration::read(mgr, nid).await;
    let torque_factor = calibration
        .applied_factor()
        .with_context(|| format!("factory torque factor unavailable: {calibration:?}"))?;
    let brs = match info.can_settings {
        MeowMotorCanSettingsStatus::Available(settings) => settings.transmit_pdo_brs,
        _ => bail!("HexMeow CAN settings unavailable"),
    };

    let recipe = RpdoRecipe {
        rpdo_index: 0,
        cob_id: SHARED_RPDO_COB_ID,
        entries: rpdo_entries_for_slice(slice),
        transmission_type: 255,
    };
    let writes =
        build_rpdo_config_writes(&recipe).map_err(|e| anyhow::anyhow!("rpdo recipe: {e}"))?;
    for w in &writes {
        sdo::download(&**bus, nid, w.index, w.subindex, &w.data, sdo_timeout)
            .await
            .map_err(|e| anyhow::anyhow!("rpdo write {:04X}:{}: {e}", w.index, w.subindex))?;
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    // A previous compressed-MIT application may have left this selector at 1.
    sdo::download(&**bus, nid, 0x4103, 1, &[0], sdo_timeout)
        .await
        .context("select uncompressed MIT")?;
    mgr.set_max_torque(nid, 0)
        .await
        .context("zero initial max torque")?;
    check_shutdown(shutdown)?;
    mgr.set_mode_sdo(
        nid,
        MeowMotorTarget::Mit(MeowMitTarget {
            position_rev: 0.0,
            velocity_rev_per_s: 0.0,
            torque_nm: 0.0,
            kp: 0,
            kd: 0,
            // The live global 0x4572 limit remains the per-wheel output limit.
            kp_kd_limit_permille: 1000,
        }),
    )
    .await
    .context("enable HexMeow MIT with a zero target")?;
    let live = mgr.status(nid)?;
    anyhow::ensure!(
        live.session_epoch == info.session_epoch,
        "HexMeow session changed during init"
    );
    Ok(MotorParameters {
        kd_factor,
        peak_torque_nm: info.peak_torque_nm,
        torque_factor,
        session_epoch: live.session_epoch,
        nmt_revision: live.nmt_revision,
        brs,
    })
}

/// Exact identity precedes the HexMeow 0x4401 clear-error transaction.
pub async fn clear_errors(mgr: &MeowMotorManager) -> anyhow::Result<()> {
    let mut failures = Vec::new();
    for &nid in &NODE_IDS {
        let result = async {
            mgr.identify(nid).await?;
            mgr.clear_error(nid).await
        }
        .await;
        if let Err(error) = result {
            failures.push(format!("0x{nid:02X}: {error}"));
        }
    }
    anyhow::ensure!(
        failures.is_empty(),
        "HexMeow clear errors failed: {}",
        failures.join("; ")
    );
    Ok(())
}

pub(crate) async fn disable_motors(mgr: &MeowMotorManager, nodes: &[u8]) -> anyhow::Result<()> {
    let mut failures = Vec::new();
    for &nid in nodes {
        if let Err(error) = mgr.disable(nid).await {
            failures.push(format!("0x{nid:02X}: {error}"));
        }
    }
    anyhow::ensure!(
        failures.is_empty(),
        "HexMeow disable failed: {}",
        failures.join("; ")
    );
    Ok(())
}

/// RPDO1 mapping entries for the motor occupying `slice` (0..3) of the shared
/// frame: its own `[0x4102:02/32, 0x4102:05/16, 0x4572:00/16]`, and two
/// 32-bit placeholders for every other motor's 8-byte slice.
fn rpdo_entries_for_slice(slice: usize) -> Vec<TpdoEntry> {
    let mut entries = Vec::with_capacity(3 + 4);
    for j in 0..3 {
        if j == slice {
            entries.push(TpdoEntry {
                index: OD_MIT,
                subindex: MIT_SUB_VDES,
                bit_len: 32,
            }); // velocity
            entries.push(TpdoEntry {
                index: OD_MIT,
                subindex: MIT_SUB_KD,
                bit_len: 16,
            }); // KD
            entries.push(TpdoEntry {
                index: OD_MAX_TORQUE,
                subindex: 0,
                bit_len: 16,
            }); // max torque
        } else {
            entries.push(PAD_ENTRY);
            entries.push(PAD_ENTRY);
        }
    }
    entries
}

/// Same SI-to-Rev/u16 conversion as Motor Control. Factory torque calibration
/// does not rescale KP/KD or the raw-domain maximum-torque limit.
fn kd_to_raw(kd_si: f64, factor: f32) -> anyhow::Result<u16> {
    anyhow::ensure!(
        factor.is_finite() && factor > 0.0,
        "invalid HexMeow 0x4102:07 Kp/Kd factor: {factor}"
    );
    anyhow::ensure!(
        kd_si.is_finite() && kd_si >= 0.0,
        "KD must be finite and non-negative"
    );
    let raw = (kd_si * std::f64::consts::TAU / f64::from(factor)).round();
    anyhow::ensure!(
        raw <= f64::from(u16::MAX),
        "KD exceeds this HexMeow motor's u16 range"
    );
    Ok(raw as u16)
}

fn shared_frame(targets: [f64; 3], kd: [u16; 3], max_torque: [u16; 3], brs: bool) -> CanFrame {
    let mut data = [0u8; SLICE_LEN * 3];
    for slice in 0..3 {
        let off = slice * SLICE_LEN;
        data[off..off + 4].copy_from_slice(&(targets[slice] as f32).to_le_bytes());
        data[off + 4..off + 6].copy_from_slice(&kd[slice].to_le_bytes());
        data[off + 6..off + 8].copy_from_slice(&max_torque[slice].to_le_bytes());
    }
    CanFrame::new_fd(SHARED_RPDO_COB_ID, &data, brs).expect("fixed 24-byte HopeA3 CAN-FD frame")
}

fn feedback_error(live: &MeowMotorLiveState, parameters: MotorParameters) -> Option<String> {
    if !live.connection.online {
        return Some("HexMeow offline".into());
    }
    if live.session_epoch != parameters.session_epoch
        || live.nmt_revision != parameters.nmt_revision
    {
        return Some("HexMeow session changed; reinitialize motor".into());
    }
    if live.lifecycle != MeowMotorLifecycle::Initialized {
        return Some(format!("HexMeow {:?}; reinitialize motor", live.lifecycle));
    }
    let now = Instant::now();
    if [live.connection.last_tpdo1, live.connection.last_tpdo2]
        .iter()
        .any(|last| last.is_none_or(|last| now.saturating_duration_since(last) > FEEDBACK_TIMEOUT))
    {
        return Some("HexMeow feedback stale; reinitialize motor".into());
    }
    match live.logic {
        Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit)) => {
            if live
                .measurements
                .velocity_rev_per_s
                .is_some_and(f64::is_finite)
            {
                None
            } else {
                Some("Waiting for HexMeow velocity feedback".into())
            }
        }
        Some(MeowMotorLogic::Error {
            mode_display,
            detailed_error,
        }) => Some(format!(
            "HexMeow mode 0x{mode_display:02X}, error 0x{detailed_error:04X}"
        )),
        _ => Some("HexMeow is not in MIT mode; reinitialize motor".into()),
    }
}

fn clamp_twist(vx: f64, vy: f64, wz: f64, max_linear: f64, max_angular: f64) -> (f64, f64, f64) {
    let mag = vx.hypot(vy);
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
    parameters: Arc<StdMutex<[MotorParameters; 3]>>,
    cmd: Arc<StdMutex<Command>>,
    state: Arc<StdMutex<Hopea3State>>,
    running: Arc<AtomicBool>,
    reinitializing: Arc<AtomicBool>,
) {
    let period = Duration::from_micros(1_000_000 / CONTROL_HZ);
    let dt = 1.0 / CONTROL_HZ as f64;
    let mut tick = tokio::time::interval(period);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    // Applied (rate-limited) twist, ramped toward the target each tick.
    let (mut ax, mut ay, mut aw) = (0.0f64, 0.0f64, 0.0f64);
    let mut send_fault: Option<String> = None;
    let mut follow_expired = false;

    while running.load(Ordering::SeqCst) {
        tick.tick().await;
        if !running.load(Ordering::SeqCst) {
            break;
        }

        let c = *cmd.lock().unwrap();
        let parameters = *parameters.lock().unwrap();
        let brs = parameters.iter().all(|p| p.brs);

        // ── read wheel feedback first (so a fault can gate this tick's targets) ──
        let mut motor_rev_s = [0.0f64; 3];
        let mut motors = Vec::with_capacity(3);
        let mut fault = send_fault.clone().or_else(|| {
            reinitializing
                .load(Ordering::SeqCst)
                .then(|| "Reinitializing HexMeow motor".into())
        });
        follow_expired |= c.deadline.is_some_and(|deadline| Instant::now() > deadline);
        if follow_expired {
            fault = Some("Follow controller expired; stop and restart HopeA3".into());
        }
        for (i, &nid) in NODE_IDS.iter().enumerate() {
            let ls = match mgr.status(nid) {
                Ok(live) => live,
                Err(error) => {
                    let error = format!("HexMeow 0x{nid:02X}: {error}");
                    fault.get_or_insert(error.clone());
                    motors.push(Hopea3Motor {
                        node_id: nid,
                        error: Some(error),
                        max_torque_permille: c.max_torque[i],
                        ..Default::default()
                    });
                    continue;
                }
            };
            let m = &ls.measurements;
            let error = feedback_error(&ls, parameters[i]);
            if let Some(error) = &error {
                fault.get_or_insert_with(|| format!("0x{nid:02X}: {error}"));
            }
            let valid = error.is_none();
            let vel = m.velocity_rev_per_s.filter(|v| valid && v.is_finite());
            motor_rev_s[i] = vel.unwrap_or(0.0);
            motors.push(Hopea3Motor {
                node_id: nid,
                online: ls.connection.online,
                enabled: ls.logic == Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit)),
                target_rev_per_s: 0.0, // filled after kinematics below
                velocity_rev_per_s: vel.map(|v| v as f32),
                torque_nm: if valid {
                    crate::dto::physical_torque_nm(
                        m.torque_permille,
                        parameters[i].peak_torque_nm,
                        Some(parameters[i].torque_factor),
                    )
                    .map(|torque| torque as f32)
                } else {
                    None
                },
                max_torque_permille: c.max_torque[i],
                driver_temp_c: m.driver_temp_c,
                motor_temp_c: m.motor_temp_c,
                error,
            });
        }

        // ── chassis acceleration limiting (slew-rate) ──
        // Offline/stale/changed-session/disabled motors are just as important
        // as firmware errors: inhibit every wheel and discard the old command.
        if fault.is_some() {
            ax = 0.0;
            ay = 0.0;
            aw = 0.0;
            let mut command = cmd.lock().unwrap();
            command.vx = 0.0;
            command.vy = 0.0;
            command.wz = 0.0;
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

        // ── inverse kinematics → send shared RPDO frame ──
        // Per motor: VDES (f32 Rev/s) + KD (u16) + max torque (u16). KD is given
        // in SI (Nm·s/rad); convert to the motor's wire int: Rev units = ×τ,
        // then ÷ the per-motor 0x4102:07 factor (full u16 range).
        let targets = kin.twist_to_motor_rev_s(ax, ay, aw);
        let mut kd = [0u16; 3];
        for slice in 0..3 {
            if fault.is_none() {
                kd[slice] = kd_to_raw(c.kd_si[slice], parameters[slice].kd_factor)
                    .expect("KD validated before init or command update");
            }
            motors[slice].target_rev_per_s = targets[slice] as f32;
        }
        let max_torque = if fault.is_none() {
            c.max_torque
        } else {
            [0; 3]
        };
        if let Err(error) = bus.send(shared_frame(targets, kd, max_torque, brs)).await {
            if send_fault.is_none() {
                log::warn!("HopeA3: RPDO send failed: {error}");
                send_fault = Some(format!("CAN send failed: {error}; stop and restart HopeA3"));
            }
            fault = send_fault.clone();
            (ax, ay, aw) = (0.0, 0.0, 0.0);
        }

        // ── forward kinematics from wheel feedback → twist + odom ──
        // A partial wheel vector cannot produce meaningful chassis odometry.
        let (mvx, mvy, mwz) = if motors
            .iter()
            .all(|motor| motor.velocity_rev_per_s.is_some())
        {
            kin.motor_rev_s_to_twist(motor_rev_s)
        } else {
            (0.0, 0.0, 0.0)
        };

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
            s.max_linear = c.max_linear;
            s.max_angular = c.max_angular;
            s.motors = motors;
            s.running = true;
            s.fault = fault;
        }
    }

    let brs = parameters.lock().unwrap().iter().all(|p| p.brs);
    let _ = bus.send(shared_frame([0.0; 3], [0; 3], [0; 3], brs)).await;
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

    #[test]
    fn hexmeow_gain_uses_full_u16_range_and_rejects_unknown_scaling() {
        let gain = 50_000.0 * f64::from(0.001f32) / std::f64::consts::TAU;
        assert_eq!(kd_to_raw(gain, 0.001).unwrap(), 50_000);
        assert!(kd_to_raw(1.0, 0.0).is_err());
        assert!(kd_to_raw(1.0, f32::NAN).is_err());
        assert!(kd_to_raw(-1.0, 0.001).is_err());
        assert!(kd_to_raw(f64::INFINITY, 0.001).is_err());
        assert!(kd_to_raw(100.0, 0.001).is_err());
    }

    #[test]
    fn shared_wire_layout_has_three_independent_hexmeow_lanes() {
        let frame = shared_frame([1.5, -2.5, 3.0], [1, 32768, 65535], [0, 800, 1000], false);
        assert_eq!(frame.id().raw(), 0x18A);
        assert_eq!(frame.data().len(), 24);
        for slot in 0..3 {
            let entries = rpdo_entries_for_slice(slot);
            let mut offset = 0;
            let mut actual = Vec::new();
            for entry in entries {
                if entry.index != 0x3000 {
                    actual.push((offset, entry.packed()));
                }
                offset += entry.bit_len as usize / 8;
            }
            assert_eq!(offset, 24);
            assert_eq!(
                actual,
                vec![
                    (slot * 8, 0x4102_0220),
                    (slot * 8 + 4, 0x4102_0510),
                    (slot * 8 + 6, 0x4572_0010),
                ]
            );
        }
        assert_eq!(&frame.data()[8..16], &[0, 0, 32, 192, 0, 128, 32, 3]);
        assert_eq!(&frame.data()[16..24], &[0, 0, 64, 64, 255, 255, 232, 3]);
    }

    #[test]
    fn feedback_gate_requires_current_session_freshness_and_mit_mode() {
        let now = Instant::now();
        let mut live = MeowMotorLiveState::empty(now);
        live.connection.online = true;
        live.connection.last_tpdo1 = Some(now);
        live.connection.last_tpdo2 = Some(now);
        live.lifecycle = MeowMotorLifecycle::Initialized;
        live.logic = Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit));
        live.measurements.velocity_rev_per_s = Some(0.0);
        let parameters = MotorParameters::default();
        assert!(feedback_error(&live, parameters).is_none());
        live.session_epoch += 1;
        assert!(feedback_error(&live, parameters)
            .unwrap()
            .contains("session"));
        live.session_epoch = 0;
        live.logic = Some(MeowMotorLogic::Enabled(MeowMotorMode::ProfileVelocity));
        assert!(feedback_error(&live, parameters).unwrap().contains("MIT"));
        live.logic = Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit));
        live.connection.last_tpdo2 = Some(now - Duration::from_secs(1));
        assert!(feedback_error(&live, parameters).unwrap().contains("stale"));
    }

    #[test]
    fn canopen_allocation_matches_global_contract() {
        assert_eq!(HOST_NODE_ID, 10);
        assert_eq!(NODE_IDS, [21, 22, 23]);
        assert_eq!(SHARED_RPDO_COB_ID, 0x18A);
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
}

#[cfg(test)]
#[path = "hopea3_protocol_tests.rs"]
mod protocol_tests;
