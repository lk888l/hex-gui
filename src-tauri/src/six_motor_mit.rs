//! Six-axis arm repeatability test using the current Meow Motor protocol's
//! canonical, uncompressed MIT object (`0x4102`).
//!
//! The values that change every control tick are mapped into two shared RPDOs:
//! `Pdes` (`0x4102:01`, REAL32 Rev) and `Vdes` (`0x4102:02`, REAL32 Rev/s) use a
//! 48-byte CAN-FD frame, while the six `Tff` values (`0x4102:03`, REAL32 Nm)
//! use a 24-byte frame. Both frames run at 500 Hz. Splitting them is necessary
//! because six complete uncompressed MIT lanes would require 72 bytes, beyond
//! the 64-byte CAN-FD payload limit. `Kp`, `Kd`, the Kp/Kd output limit and the
//! global torque limit stay outside PDO and use verified SDO transactions.
//!
//! Motion is serialized by axis. Motor 1 follows the active waypoint while the
//! other five hold their last commanded positions; only after measured position
//! and velocity have stayed inside the arrival window does motor 2 begin, and
//! so on through motor 6.

use std::f32::consts::TAU as TAU_F32;
use std::f64::consts::TAU as TAU_F64;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex as StdMutex, MutexGuard as StdMutexGuard};
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanFrame};
use hex_arm_dynamics::ArmDynamics;
use hex_motor::canopen::rpdo_config::{build_rpdo_config_writes, RpdoRecipe};
use hex_motor::canopen::sdo;
use hex_motor::canopen::tpdo_config::TpdoEntry;
use hex_motor::cia402::PdoProfile;
use hex_motor::meow_motor::{
    MeowMitTarget, MeowMotorLifecycle, MeowMotorLiveState, MeowMotorLogic, MeowMotorManager,
    MeowMotorMode, MeowMotorTarget,
};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;
use tokio::task::JoinHandle;

pub const NODE_IDS: [u8; 6] = [1, 2, 3, 4, 5, 6];
pub const CONTROL_HZ: u64 = 500;

const MOTOR_COUNT: usize = NODE_IDS.len();
const WAYPOINT_COUNT: usize = 4;
const POSITION_VELOCITY_LANE_BYTES: usize = 8;
const POSITION_VELOCITY_FRAME_BYTES: usize = MOTOR_COUNT * POSITION_VELOCITY_LANE_BYTES;
const TFF_LANE_BYTES: usize = 4;
const TFF_FRAME_BYTES: usize = MOTOR_COUNT * TFF_LANE_BYTES;
const CONTROL_PERIOD: Duration = Duration::from_micros(1_000_000 / CONTROL_HZ);
const FEEDBACK_STALE_TIMEOUT: Duration = Duration::from_millis(100);
const CONTROL_JITTER_TRIP: Duration = Duration::from_millis(50);
const SDO_TIMEOUT: Option<Duration> = Some(Duration::from_millis(500));
const SDO_SETTLE: Duration = Duration::from_millis(5);
const UPDATE_BRAKE_TIMEOUT_MAX: Duration = Duration::from_secs(10);
const UPDATE_HOLD_MARGIN: Duration = Duration::from_secs(2);
const HOLD_POSITION_TOLERANCE_MAX_REV: f32 = 0.003;
const HOLD_VELOCITY_TOLERANCE_MAX_REV_PER_S: f32 = 0.02;
const TFF_TRANSITION_EPSILON_NM: f32 = 1.0e-3;
const POSITION_MIN_REV: f32 = -128.0;
const POSITION_MAX_REV_EXCLUSIVE: f32 = 128.0;
const GRAVITY_MODEL: &str = "firefly_y6 v1.1.0 · hex-arm-dynamics";
const ARM_URDF: &str = include_str!(
    "../../public/urdf/packages/xpkg_urdf_firefly_y6/1.1.0/7a14c706b9ad102d/urdf/xpkg_urdf_firefly_y6.urdf"
);
const JOINT_MIN_RAD: [f32; MOTOR_COUNT] = [-2.86, -1.57, 0.0, -1.57, -1.54, -2.79];
const JOINT_MAX_RAD: [f32; MOTOR_COUNT] = [2.86, 2.09, 3.14, 1.57, 1.54, 2.79];
const JOINT_LIMIT_TOLERANCE_RAD: f32 = 0.1;
const PREFLIGHT_SAMPLES_PER_AXIS: usize = 32;
const TORQUE_HEADROOM: f32 = 1.15;
/// The current protocol consumes a shared Host TPDO1, matching the Motor
/// Control driver's proven `host_tpdo_cob_id(host, 0)` convention.
const RPDO1_COB_ID_BASE: u16 = 0x180;
const RPDO2_COB_ID_BASE: u16 = 0x280;

const OD_MIT: u16 = 0x4102;
const MIT_SUB_PDES: u8 = 0x01;
const MIT_SUB_VDES: u8 = 0x02;
const MIT_SUB_TFF: u8 = 0x03;
const MIT_SUB_KP: u8 = 0x04;
const MIT_SUB_KD: u8 = 0x05;
const MIT_SUB_KP_KD_LIMIT: u8 = 0x06;
const MIT_SUB_FACTOR: u8 = 0x07;

/// `0x4103:01 = 0` selects canonical/uncompressed `0x4102`; `1` selects the
/// quantized eight-byte compressed object. This test always writes and verifies
/// zero before enabling a motor and again before a live SDO tuning update.
const OD_COMPRESSED_MIT: u16 = 0x4103;
const COMPRESSED_MIT_SUB_ENABLE: u8 = 0x01;
const UNCOMPRESSED_MIT: u8 = 0;

const OD_MAX_TORQUE: u16 = 0x4572;

/// Vendor-provided 32-bit PDO padding object used by the existing, proven
/// multi-motor shared-frame implementations in this application.
const PAD_ENTRY: TpdoEntry = TpdoEntry {
    index: 0x3000,
    subindex: 3,
    bit_len: 32,
};

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SixMotorMitTuning {
    /// Constant physical feed-forward torque, streamed through `0x4102:03`.
    pub tff_nm: f32,
    /// Motor encoder direction relative to the corresponding URDF joint.
    /// This is also the joint-torque to motor-torque sign.
    pub gravity_direction: i8,
    /// Motor encoder position corresponding to `gravity_park_angle_rad`.
    pub gravity_park_position_rev: f32,
    /// URDF joint angle at the physical parked/folded reference pose.
    pub gravity_park_angle_rad: f32,
    /// Per-axis multiplier applied to the coupled URDF gravity torque.
    pub gravity_scale: f32,
    /// Absolute host-side clamp for this axis' gravity contribution.
    pub gravity_max_torque_nm: f32,
    /// Operator-facing SI gain. Converted to the motor's u16 field using the
    /// read-only `0x4102:07` factor.
    pub kp_nm_per_rad: f64,
    /// Operator-facing SI damping gain. Converted like Kp.
    pub kd_nm_s_per_rad: f64,
    /// `0x4102:06`, permille of peak torque.
    pub kp_kd_limit_permille: u16,
    /// `0x4572`, permille of peak torque.
    pub max_torque_permille: u16,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SixMotorMitRequest {
    pub waypoints: Vec<[f32; MOTOR_COUNT]>,
    pub gravity_enabled: bool,
    /// At start, bind each configured URDF park angle to the last fresh
    /// encoder sample taken immediately before enable.
    pub capture_gravity_park_on_start: bool,
    pub gravity_vector_m_per_s2: [f32; 3],
    pub tff_slew_rate_nm_per_s: f32,
    pub payload_mass_kg: f32,
    pub payload_center_of_mass_m: [f32; 3],
    pub max_velocity_rev_per_s: f32,
    pub acceleration_rev_per_s2: f32,
    pub deceleration_rev_per_s2: f32,
    pub arrival_position_tolerance_rev: f32,
    pub arrival_velocity_tolerance_rev_per_s: f32,
    pub settle_time_ms: u32,
    pub movement_timeout_ms: u32,
    pub waypoint_dwell_ms: u32,
    pub tuning: [SixMotorMitTuning; MOTOR_COUNT],
}

impl SixMotorMitRequest {
    pub fn validate(&self) -> Result<(), String> {
        if self.waypoints.len() != WAYPOINT_COUNT {
            return Err(format!(
                "six-motor MIT test requires exactly {WAYPOINT_COUNT} waypoints"
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
        validate_positive_bounded(
            self.max_velocity_rev_per_s,
            5.0,
            "maximum MIT velocity",
            "Rev/s",
        )?;
        validate_positive_bounded(
            self.acceleration_rev_per_s2,
            50.0,
            "MIT acceleration",
            "Rev/s^2",
        )?;
        validate_positive_bounded(
            self.deceleration_rev_per_s2,
            50.0,
            "MIT deceleration",
            "Rev/s^2",
        )?;
        validate_positive_bounded(
            self.arrival_position_tolerance_rev,
            1.0,
            "arrival position tolerance",
            "Rev",
        )?;
        validate_positive_bounded(
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
        for (axis, component) in self.gravity_vector_m_per_s2.iter().enumerate() {
            if !component.is_finite() || component.abs() > 20.0 {
                return Err(format!(
                    "gravity vector component {axis} must be finite and within +/-20 m/s^2"
                ));
            }
        }
        if self.gravity_enabled {
            let gravity_magnitude = self
                .gravity_vector_m_per_s2
                .iter()
                .map(|value| value * value)
                .sum::<f32>()
                .sqrt();
            if !(8.0..=12.0).contains(&gravity_magnitude) {
                return Err(
                    "gravity vector magnitude must be in [8, 12] m/s^2 when enabled".into(),
                );
            }
        }
        validate_positive_bounded(
            self.tff_slew_rate_nm_per_s,
            1_000.0,
            "Tff slew rate",
            "Nm/s",
        )?;
        if !self.payload_mass_kg.is_finite() || !(0.0..=100.0).contains(&self.payload_mass_kg) {
            return Err("payload mass must be finite and in [0, 100] kg".into());
        }
        for (axis, component) in self.payload_center_of_mass_m.iter().enumerate() {
            if !component.is_finite() || component.abs() > 10.0 {
                return Err(format!(
                    "payload center-of-mass component {axis} must be finite and within +/-10 m"
                ));
            }
        }
        for (index, tuning) in self.tuning.iter().enumerate() {
            if !tuning.tff_nm.is_finite() || tuning.tff_nm.abs() > 1_000.0 {
                return Err(format!(
                    "motor {} Tff must be finite and within +/-1000 Nm",
                    NODE_IDS[index]
                ));
            }
            if tuning.gravity_direction != -1 && tuning.gravity_direction != 1 {
                return Err(format!(
                    "motor {} gravity direction must be -1 or +1",
                    NODE_IDS[index]
                ));
            }
            if !tuning.gravity_park_position_rev.is_finite()
                || tuning.gravity_park_position_rev < POSITION_MIN_REV
                || tuning.gravity_park_position_rev >= POSITION_MAX_REV_EXCLUSIVE
            {
                return Err(format!(
                    "motor {} gravity park position must be finite and in [{POSITION_MIN_REV}, {POSITION_MAX_REV_EXCLUSIVE}) Rev",
                    NODE_IDS[index]
                ));
            }
            if !tuning.gravity_park_angle_rad.is_finite()
                || !(-TAU_F32..=TAU_F32).contains(&tuning.gravity_park_angle_rad)
            {
                return Err(format!(
                    "motor {} gravity park angle must be finite and in [-2pi, 2pi] rad",
                    NODE_IDS[index]
                ));
            }
            if !tuning.gravity_scale.is_finite() || !(0.0..=1.5).contains(&tuning.gravity_scale) {
                return Err(format!(
                    "motor {} gravity scale must be finite and in [0, 1.5]",
                    NODE_IDS[index]
                ));
            }
            if !tuning.gravity_max_torque_nm.is_finite()
                || !(0.0..=1_000.0).contains(&tuning.gravity_max_torque_nm)
            {
                return Err(format!(
                    "motor {} gravity maximum torque must be finite and in [0, 1000] Nm",
                    NODE_IDS[index]
                ));
            }
            if tuning.tff_nm.abs() + tuning.gravity_max_torque_nm.abs() > 1_000.0 {
                return Err(format!(
                    "motor {} constant Tff plus maximum gravity torque must stay within +/-1000 Nm",
                    NODE_IDS[index]
                ));
            }
            for (name, value) in [("Kp", tuning.kp_nm_per_rad), ("Kd", tuning.kd_nm_s_per_rad)] {
                if !value.is_finite() || value < 0.0 {
                    return Err(format!(
                        "motor {} {name} must be finite and non-negative",
                        NODE_IDS[index]
                    ));
                }
            }
            if tuning.kp_kd_limit_permille > 1_000 {
                return Err(format!(
                    "motor {} Kp/Kd output limit must be 0..=1000 permille",
                    NODE_IDS[index]
                ));
            }
            if tuning.max_torque_permille > 1_000 {
                return Err(format!(
                    "motor {} maximum torque must be 0..=1000 permille",
                    NODE_IDS[index]
                ));
            }
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

#[derive(Debug, Clone, Copy, Default)]
struct WireTuning {
    kp: u16,
    kd: u16,
}

#[derive(Debug, Clone, Serialize)]
pub struct SixMotorMitMotorView {
    pub node_id: u8,
    pub online: bool,
    pub enabled: bool,
    pub active: bool,
    pub sequence_state: String,
    pub target_position_rev: f32,
    pub command_position_rev: f32,
    pub command_velocity_rev_per_s: f32,
    pub actual_position_rev: Option<f32>,
    pub actual_velocity_rev_per_s: Option<f32>,
    pub position_error_rev: Option<f32>,
    pub tff_nm: f32,
    pub gravity_direction: i8,
    pub gravity_park_position_rev: f32,
    pub gravity_park_angle_rad: f32,
    pub gravity_scale: f32,
    pub gravity_max_torque_nm: f32,
    pub joint_position_rad: f32,
    pub model_gravity_torque_nm: f32,
    pub gravity_target_torque_nm: f32,
    pub gravity_limited: bool,
    pub gravity_torque_nm: f32,
    pub command_tff_nm: f32,
    pub wire_tff_nm: f32,
    pub tff_limit_nm: f32,
    pub tff_utilization_percent: f32,
    pub tff_limited: bool,
    pub peak_torque_nm: f32,
    pub recommended_min_torque_permille: u16,
    pub kp_nm_per_rad: f64,
    pub kd_nm_s_per_rad: f64,
    pub kp_raw: u16,
    pub kd_raw: u16,
    pub kp_kd_limit_permille: u16,
    pub max_torque_permille: u16,
    pub kp_kd_factor: f32,
    pub feedback_age_ms: Option<u64>,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SixMotorMitView {
    pub running: bool,
    pub phase: String,
    pub gravity_model: String,
    pub gravity_enabled: bool,
    pub active_waypoint_index: usize,
    pub active_motor_index: Option<usize>,
    pub completed_cycles: u64,
    pub pdo_rate_hz: u64,
    pub rpdo_cob_id: u16,
    pub tff_rpdo_cob_id: u16,
    pub pdo_fields: Vec<String>,
    pub sdo_fields: Vec<String>,
    pub motors: Vec<SixMotorMitMotorView>,
    pub last_error: Option<String>,
}

impl SixMotorMitView {
    pub fn idle() -> Self {
        Self {
            running: false,
            phase: "idle".into(),
            gravity_model: GRAVITY_MODEL.into(),
            gravity_enabled: false,
            active_waypoint_index: 0,
            active_motor_index: None,
            completed_cycles: 0,
            pdo_rate_hz: CONTROL_HZ,
            rpdo_cob_id: 0,
            tff_rpdo_cob_id: 0,
            pdo_fields: vec![
                "RPDO1 0x4102:01 Pdes".into(),
                "RPDO1 0x4102:02 Vdes".into(),
                "RPDO2 0x4102:03 Tff (constant + coupled URDF gravity)".into(),
            ],
            sdo_fields: vec![
                "0x4102:04 Kp".into(),
                "0x4102:05 Kd".into(),
                "0x4102:06 Kp/Kd limit".into(),
                "0x4102:07 factor (read-only)".into(),
                "0x4572 max torque".into(),
                "0x4103:01 uncompressed selector".into(),
            ],
            motors: Vec::new(),
            last_error: None,
        }
    }
}

struct RuntimeConfig {
    request: SixMotorMitRequest,
    dynamics: Arc<ArmDynamics>,
    wire_tuning: [WireTuning; MOTOR_COUNT],
    recommended_min_torque_permille: [u16; MOTOR_COUNT],
    revision: u64,
}

#[derive(Clone, Copy, Default)]
struct UpdateHoldFence {
    generation: u64,
    after: [Option<Instant>; MOTOR_COUNT],
}

#[derive(Clone, Copy)]
struct LastSentCommand {
    positions: [f32; MOTOR_COUNT],
    tff_commands: [f32; MOTOR_COUNT],
}

struct PersistentDisableHold {
    running: Arc<AtomicBool>,
    healthy: Arc<AtomicBool>,
    task: JoinHandle<()>,
}

struct SetupResult {
    effective_request: SixMotorMitRequest,
    initial_command_positions: [f32; MOTOR_COUNT],
    initial_sensed_positions: [f32; MOTOR_COUNT],
    initial_gravity: GravityComputation,
    factors: [f32; MOTOR_COUNT],
    peak_torque_nm: [f32; MOTOR_COUNT],
    recommended_min_torque_permille: [u16; MOTOR_COUNT],
    wire_tuning: [WireTuning; MOTOR_COUNT],
    position_velocity_cob_id: u16,
    tff_cob_id: u16,
}

struct StartupRecovery {
    setup: SetupResult,
    detail: String,
    remaining_enabled: [bool; MOTOR_COUNT],
    hold: PersistentDisableHold,
}

enum SetupOutcome {
    Ready(SetupResult),
    Recovery(StartupRecovery),
}

/// Startup can fail after one or more drives have entered MIT mode.  In that
/// case the error carries a stopped recovery session whose 500 Hz hold remains
/// live and which the application must retain so a later Stop can retry the
/// confirmed disable operation.
pub struct SixMotorMitStartError {
    detail: String,
    recovery: Option<Arc<SixMotorMit>>,
}

impl SixMotorMitStartError {
    fn plain(error: impl std::fmt::Display) -> Self {
        Self {
            detail: error.to_string(),
            recovery: None,
        }
    }

    fn with_recovery(detail: String, recovery: Arc<SixMotorMit>) -> Self {
        Self {
            detail,
            recovery: Some(recovery),
        }
    }

    pub fn into_parts(self) -> (String, Option<Arc<SixMotorMit>>) {
        (self.detail, self.recovery)
    }
}

impl std::fmt::Debug for SixMotorMitStartError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("SixMotorMitStartError")
            .field("detail", &self.detail)
            .field("has_recovery", &self.recovery.is_some())
            .finish()
    }
}

impl std::fmt::Display for SixMotorMitStartError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.detail)
    }
}

impl std::error::Error for SixMotorMitStartError {}

impl From<anyhow::Error> for SixMotorMitStartError {
    fn from(error: anyhow::Error) -> Self {
        Self::plain(error)
    }
}

fn initial_session_view(
    setup: &SetupResult,
    request: &SixMotorMitRequest,
    torque_factors: &[f32; MOTOR_COUNT],
    running: bool,
    phase: &str,
    enabled: [bool; MOTOR_COUNT],
    last_error: Option<String>,
) -> SixMotorMitView {
    let applied_physical_tff = [0.0; MOTOR_COUNT];
    let wire_tff = wire_tff_states(
        request,
        &applied_physical_tff,
        torque_factors,
        &setup.peak_torque_nm,
    );
    let motors = NODE_IDS
        .iter()
        .enumerate()
        .map(|(index, &node_id)| SixMotorMitMotorView {
            node_id,
            online: true,
            enabled: enabled[index],
            active: false,
            sequence_state: if running {
                "holding"
            } else if enabled[index] {
                "disable-failed"
            } else {
                "disabled"
            }
            .into(),
            target_position_rev: request.waypoints[0][index],
            command_position_rev: setup.initial_command_positions[index],
            command_velocity_rev_per_s: 0.0,
            actual_position_rev: Some(setup.initial_sensed_positions[index]),
            actual_velocity_rev_per_s: None,
            position_error_rev: Some(
                setup.initial_sensed_positions[index] - request.waypoints[0][index],
            ),
            tff_nm: request.tuning[index].tff_nm,
            gravity_direction: request.tuning[index].gravity_direction,
            gravity_park_position_rev: request.tuning[index].gravity_park_position_rev,
            gravity_park_angle_rad: request.tuning[index].gravity_park_angle_rad,
            gravity_scale: request.tuning[index].gravity_scale,
            gravity_max_torque_nm: request.tuning[index].gravity_max_torque_nm,
            joint_position_rad: setup.initial_gravity.joint_position_rad[index],
            model_gravity_torque_nm: setup.initial_gravity.model_torque_nm[index],
            gravity_target_torque_nm: setup.initial_gravity.target_motor_torque_nm[index],
            gravity_limited: setup.initial_gravity.limited[index],
            // Both constant trim and gravity start at zero; startup holds the
            // captured Pdes until the transition has converged.
            gravity_torque_nm: 0.0,
            command_tff_nm: applied_physical_tff[index],
            wire_tff_nm: wire_tff[index].command_nm,
            tff_limit_nm: wire_tff[index].limit_nm,
            tff_utilization_percent: wire_tff[index].utilization_percent,
            tff_limited: wire_tff[index].limited,
            peak_torque_nm: setup.peak_torque_nm[index],
            recommended_min_torque_permille: setup.recommended_min_torque_permille[index],
            kp_nm_per_rad: request.tuning[index].kp_nm_per_rad,
            kd_nm_s_per_rad: request.tuning[index].kd_nm_s_per_rad,
            kp_raw: setup.wire_tuning[index].kp,
            kd_raw: setup.wire_tuning[index].kd,
            kp_kd_limit_permille: request.tuning[index].kp_kd_limit_permille,
            max_torque_permille: request.tuning[index].max_torque_permille,
            kp_kd_factor: setup.factors[index],
            feedback_age_ms: Some(0),
            error: (!running && enabled[index])
                .then(|| "startup rollback could not confirm disable".into()),
        })
        .collect();
    SixMotorMitView {
        running,
        phase: phase.into(),
        gravity_model: GRAVITY_MODEL.into(),
        gravity_enabled: request.gravity_enabled,
        active_waypoint_index: 0,
        active_motor_index: None,
        completed_cycles: 0,
        pdo_rate_hz: CONTROL_HZ,
        rpdo_cob_id: setup.position_velocity_cob_id,
        tff_rpdo_cob_id: setup.tff_cob_id,
        pdo_fields: SixMotorMitView::idle().pdo_fields,
        sdo_fields: SixMotorMitView::idle().sdo_fields,
        motors,
        last_error,
    }
}

/// One Rust-owned test session. The application state retains the `Arc` until
/// confirmed disable succeeds, so a failed stop remains retryable.
pub struct SixMotorMit {
    manager: Arc<MeowMotorManager>,
    bus: Arc<dyn CanBus>,
    position_velocity_cob_id: u16,
    tff_cob_id: u16,
    factors: [f32; MOTOR_COUNT],
    torque_factors: [f32; MOTOR_COUNT],
    peak_torque_nm: [f32; MOTOR_COUNT],
    base_dynamics: Arc<ArmDynamics>,
    config: Arc<StdMutex<Arc<RuntimeConfig>>>,
    view: Arc<StdMutex<SixMotorMitView>>,
    running: Arc<AtomicBool>,
    updating: AtomicBool,
    update_hold_confirmed_generation: AtomicU64,
    update_hold_fence: StdMutex<UpdateHoldFence>,
    last_sent_command: StdMutex<LastSentCommand>,
    task: Mutex<Option<JoinHandle<()>>>,
    disable_hold: Mutex<Option<PersistentDisableHold>>,
    pdo_send_gate: Mutex<()>,
    sdo_gate: Mutex<()>,
    shutdown_gate: Mutex<()>,
}

impl SixMotorMit {
    pub async fn start(
        manager: Arc<MeowMotorManager>,
        bus: Arc<dyn CanBus>,
        host_node_id: u8,
        torque_factors: [f32; MOTOR_COUNT],
        request: SixMotorMitRequest,
        shutdown_requested: &AtomicBool,
    ) -> Result<Arc<Self>, SixMotorMitStartError> {
        request.validate().map_err(anyhow::Error::msg)?;
        validate_torque_factors(&torque_factors)?;
        let base_dynamics = ArmDynamics::from_urdf_string(ARM_URDF)
            .map_err(|error| anyhow::anyhow!("load bundled {GRAVITY_MODEL} URDF: {error}"))?;
        if base_dynamics.dof() != MOTOR_COUNT {
            return Err(SixMotorMitStartError::plain(format!(
                "bundled gravity model has {} joints, expected {MOTOR_COUNT}",
                base_dynamics.dof()
            )));
        }
        let base_dynamics = Arc::new(base_dynamics);
        let configured_dynamics = Arc::new(configured_dynamics(&base_dynamics, &request));
        let capabilities = bus.capabilities();
        if !capabilities.fd || capabilities.max_dlen < POSITION_VELOCITY_FRAME_BYTES {
            return Err(SixMotorMitStartError::plain(format!(
                "six-motor MIT requires CAN-FD with at least {POSITION_VELOCITY_FRAME_BYTES} payload bytes; adapter reports fd={}, max_dlen={}",
                capabilities.fd,
                capabilities.max_dlen
            )));
        }
        let pdo_profile = PdoProfile::from_event_timer_ms(2)
            .map_err(|error| anyhow::anyhow!("build 500 Hz PDO profile: {error}"))?;
        let setup = match setup_all(
            &manager,
            &bus,
            host_node_id,
            pdo_profile,
            &torque_factors,
            &request,
            &configured_dynamics,
            shutdown_requested,
        )
        .await
        {
            Ok(SetupOutcome::Ready(setup)) => setup,
            Ok(SetupOutcome::Recovery(recovery)) => {
                let app = Self::from_startup_recovery(
                    Arc::clone(&manager),
                    Arc::clone(&bus),
                    torque_factors,
                    Arc::clone(&base_dynamics),
                    configured_dynamics,
                    recovery,
                )
                .await;
                let detail = app
                    .snapshot()
                    .last_error
                    .unwrap_or_else(|| "six-motor MIT startup recovery is active".into());
                return Err(SixMotorMitStartError::with_recovery(detail, app));
            }
            Err(error) => {
                let failures = disable_startup_candidates(&manager).await;
                if failures.is_empty() {
                    return Err(SixMotorMitStartError::plain(error));
                }
                return Err(SixMotorMitStartError::plain(format!(
                    "{error}; startup rollback could not confirm disable: {}. Remove motor power before handling the arm",
                    failures.join("; ")
                )));
            }
        };

        let request = setup.effective_request.clone();
        let config = Arc::new(RuntimeConfig {
            request: request.clone(),
            dynamics: configured_dynamics,
            wire_tuning: setup.wire_tuning,
            recommended_min_torque_permille: setup.recommended_min_torque_permille,
            revision: 0,
        });
        let view = initial_session_view(
            &setup,
            &request,
            &torque_factors,
            true,
            "compensation-ramp",
            [true; MOTOR_COUNT],
            None,
        );

        let app = Arc::new(Self {
            manager,
            bus,
            position_velocity_cob_id: setup.position_velocity_cob_id,
            tff_cob_id: setup.tff_cob_id,
            factors: setup.factors,
            torque_factors,
            peak_torque_nm: setup.peak_torque_nm,
            base_dynamics,
            config: Arc::new(StdMutex::new(config)),
            view: Arc::new(StdMutex::new(view)),
            running: Arc::new(AtomicBool::new(true)),
            updating: AtomicBool::new(false),
            update_hold_confirmed_generation: AtomicU64::new(0),
            update_hold_fence: StdMutex::new(UpdateHoldFence::default()),
            last_sent_command: StdMutex::new(LastSentCommand {
                positions: setup.initial_command_positions,
                tff_commands: [0.0; MOTOR_COUNT],
            }),
            task: Mutex::new(None),
            disable_hold: Mutex::new(None),
            pdo_send_gate: Mutex::new(()),
            sdo_gate: Mutex::new(()),
            shutdown_gate: Mutex::new(()),
        });

        let task_app = Arc::clone(&app);
        let task = tokio::spawn(async move {
            task_app.control_loop(setup.initial_command_positions).await;
        });
        *app.task.lock().await = Some(task);
        Ok(app)
    }

    async fn from_startup_recovery(
        manager: Arc<MeowMotorManager>,
        bus: Arc<dyn CanBus>,
        torque_factors: [f32; MOTOR_COUNT],
        base_dynamics: Arc<ArmDynamics>,
        configured_dynamics: Arc<ArmDynamics>,
        recovery: StartupRecovery,
    ) -> Arc<Self> {
        let StartupRecovery {
            setup,
            mut detail,
            remaining_enabled,
            hold,
        } = recovery;
        let request = setup.effective_request.clone();
        let config = Arc::new(RuntimeConfig {
            request: request.clone(),
            dynamics: configured_dynamics,
            wire_tuning: setup.wire_tuning,
            recommended_min_torque_permille: setup.recommended_min_torque_permille,
            revision: 0,
        });
        let view = initial_session_view(
            &setup,
            &request,
            &torque_factors,
            false,
            "error",
            remaining_enabled,
            Some(detail.clone()),
        );
        let last_sent = LastSentCommand {
            positions: setup.initial_command_positions,
            tff_commands: [0.0; MOTOR_COUNT],
        };
        let app = Arc::new(Self {
            manager,
            bus,
            position_velocity_cob_id: setup.position_velocity_cob_id,
            tff_cob_id: setup.tff_cob_id,
            factors: setup.factors,
            torque_factors,
            peak_torque_nm: setup.peak_torque_nm,
            base_dynamics,
            config: Arc::new(StdMutex::new(config)),
            view: Arc::new(StdMutex::new(view)),
            running: Arc::new(AtomicBool::new(false)),
            updating: AtomicBool::new(false),
            update_hold_confirmed_generation: AtomicU64::new(0),
            update_hold_fence: StdMutex::new(UpdateHoldFence::default()),
            last_sent_command: StdMutex::new(last_sent),
            task: Mutex::new(None),
            disable_hold: Mutex::new(Some(hold)),
            pdo_send_gate: Mutex::new(()),
            sdo_gate: Mutex::new(()),
            shutdown_gate: Mutex::new(()),
        });

        // A send error may have ended the transferred task. Try to replace it
        // immediately; if the bus is still unavailable, retain the new task so
        // Stop can retry and keep the safety failure visible to the operator.
        let hold_failures = app.ensure_persistent_disable_hold(last_sent).await;
        if !hold_failures.is_empty() {
            detail.push_str(&format!(
                "; startup recovery hold failures: {}. Immediately remove motor power and do not touch the arm",
                hold_failures.join("; ")
            ));
            lock_unpoisoned(&app.view).last_error = Some(detail);
        }
        app
    }

    pub fn snapshot(&self) -> SixMotorMitView {
        lock_unpoisoned(&self.view).clone()
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Acquire)
    }

    /// Apply every runtime-configurable value. Pdes/Vdes/Tff remain on the
    /// 500 Hz streams; the other MIT fields are written and verified through SDO.
    pub async fn update(&self, request: SixMotorMitRequest) -> anyhow::Result<SixMotorMitView> {
        request.validate().map_err(anyhow::Error::msg)?;
        if !self.is_running() {
            anyhow::bail!("six-motor MIT test is not running");
        }
        let dynamics = Arc::new(configured_dynamics(&self.base_dynamics, &request));
        let wire_tuning = convert_all_tuning(&request.tuning, &self.factors)?;
        let sdo_guard = self.sdo_gate.lock().await;
        if !self.is_running() {
            anyhow::bail!("six-motor MIT test stopped before the SDO update began");
        }
        let old_config = Arc::clone(&lock_unpoisoned(&self.config));
        let sdo_changed: [bool; MOTOR_COUNT] = std::array::from_fn(|index| {
            sdo_tuning_changed(old_config.request.tuning[index], request.tuning[index])
        });
        let has_sdo_changes = sdo_changed.iter().any(|changed| *changed);
        let lowers_torque_limit = (0..MOTOR_COUNT).any(|index| {
            request.tuning[index].max_torque_permille
                < old_config.request.tuning[index].max_torque_permille
        });
        // Every Apply first brings the trajectory to a controlled stop, then
        // latches one fresh measured Pdes and waits for all axes to settle.
        // Besides protecting an SDO gain change from a Vdes/Kd step, this also
        // makes a gravity-only revision restart from the same state preflight
        // checked (fresh position -> waypoint 1, motor 1 first).
        let initial_baselines = tpdo1_baselines(&self.manager)?;
        let initial_hold_generation = {
            let mut fence = lock_unpoisoned(&self.update_hold_fence);
            fence.generation = fence.generation.wrapping_add(1);
            fence.after = initial_baselines;
            fence.generation
        };
        self.updating.store(true, Ordering::Release);
        let _updating = AtomicFlagReset(&self.updating);
        self.wait_for_update_hold_confirmation(&old_config.request, initial_hold_generation)
            .await?;

        // Braking changes the real starting pose. Run the path preflight only
        // after the fresh-Pdes hold has settled, so current -> W1 is the path
        // that the subsequent revision will actually execute.
        let sensed_positions = current_position_snapshot(&self.manager)?;
        let command_positions = {
            let view = lock_unpoisoned(&self.view);
            current_command_snapshot(&view)?
        };
        let mut recommended_min_torque_permille = preflight_sensed_and_command_starts(
            &request,
            &dynamics,
            &sensed_positions,
            &command_positions,
            &self.torque_factors,
            &self.peak_torque_nm,
        )?;

        // If a maximum-torque setting is lowered, publish the lower host-side
        // ceiling before changing the drive's SDO. Reset the hold ack and wait
        // for one successfully sent 500 Hz frame under that new ceiling.
        if lowers_torque_limit {
            let mut interim_request = old_config.request.clone();
            for index in 0..MOTOR_COUNT {
                interim_request.tuning[index].max_torque_permille = interim_request.tuning[index]
                    .max_torque_permille
                    .min(request.tuning[index].max_torque_permille);
            }
            let ceiling_baselines = tpdo1_baselines(&self.manager)?;
            let ceiling_hold_generation = {
                // The control loop takes these locks in this same order and
                // therefore cannot pair the new generation with the old host
                // torque ceiling (or vice versa).
                let mut config = lock_unpoisoned(&self.config);
                let mut fence = lock_unpoisoned(&self.update_hold_fence);
                *config = Arc::new(RuntimeConfig {
                    request: interim_request,
                    dynamics: Arc::clone(&old_config.dynamics),
                    wire_tuning: old_config.wire_tuning,
                    recommended_min_torque_permille: old_config.recommended_min_torque_permille,
                    revision: old_config.revision,
                });
                fence.generation = fence.generation.wrapping_add(1);
                fence.after = ceiling_baselines;
                fence.generation
            };
            self.wait_for_update_hold_confirmation(&old_config.request, ceiling_hold_generation)
                .await?;
        }

        let sdo_update_result: anyhow::Result<Option<[u16; MOTOR_COUNT]>> = async {
            if !has_sdo_changes {
                return Ok(None);
            }
            for index in 0..MOTOR_COUNT {
                if !sdo_changed[index] {
                    continue;
                }
                if !self.is_running() {
                    anyhow::bail!("six-motor MIT test stopped during the SDO update");
                }
                write_and_verify_runtime_tuning(
                    &self.bus,
                    NODE_IDS[index],
                    old_config.request.tuning[index],
                    request.tuning[index],
                    old_config.wire_tuning[index],
                    wire_tuning[index],
                )
                .await
                .map_err(|error| {
                    anyhow::anyhow!(
                        "motor {} runtime SDO update failed: {error}",
                        NODE_IDS[index]
                    )
                })?;
            }
            if !self.is_running() {
                anyhow::bail!("six-motor MIT test stopped before the SDO update completed");
            }

            // Gains/limits can alter the held pose. Start a new generation so
            // the control loop repeats close+still, one-time fresh rebase, and
            // post-rebase settle under the *new* drive parameters.
            let post_sdo_baselines = tpdo1_baselines(&self.manager)?;
            let post_sdo_hold_generation = {
                let mut fence = lock_unpoisoned(&self.update_hold_fence);
                fence.generation = fence.generation.wrapping_add(1);
                fence.after = post_sdo_baselines;
                fence.generation
            };
            self.wait_for_update_hold_confirmation(&old_config.request, post_sdo_hold_generation)
                .await?;
            let final_sensed_positions = current_position_snapshot(&self.manager)?;
            let final_command_positions = {
                let view = lock_unpoisoned(&self.view);
                current_command_snapshot(&view)?
            };
            let final_recommendation = preflight_sensed_and_command_starts(
                &request,
                &dynamics,
                &final_sensed_positions,
                &final_command_positions,
                &self.torque_factors,
                &self.peak_torque_nm,
            )?;
            Ok(Some(final_recommendation))
        }
        .await;
        match sdo_update_result {
            Ok(Some(final_recommendation)) => {
                recommended_min_torque_permille = final_recommendation;
            }
            Ok(None) => {}
            Err(error) => {
                // A partially completed multi-axis SDO sequence cannot safely
                // resume under the old software config. Stop, release the SDO
                // gate, and confirm-disable while preserving the last PDO hold.
                drop(sdo_guard);
                let primary = error.to_string();
                return match self.shutdown_inner(Some(primary.clone())).await {
                    Err(stop_error) => Err(stop_error),
                    Ok(()) => Err(anyhow::anyhow!(primary)),
                };
            }
        }

        let config = Arc::new(RuntimeConfig {
            request: request.clone(),
            dynamics,
            wire_tuning,
            recommended_min_torque_permille,
            // Every Apply restarts at waypoint 1, motor 1 after the common
            // controlled-stop hold, making preflight and execution identical.
            revision: old_config.revision.wrapping_add(1),
        });
        *lock_unpoisoned(&self.config) = config;
        {
            let mut view = lock_unpoisoned(&self.view);
            view.gravity_enabled = request.gravity_enabled;
            update_view_tuning(
                &mut view,
                &request,
                &wire_tuning,
                &recommended_min_torque_permille,
            );
        }
        Ok(self.snapshot())
    }

    async fn wait_for_update_hold_confirmation(
        &self,
        request: &SixMotorMitRequest,
        expected_generation: u64,
    ) -> anyhow::Result<()> {
        let braking_time = Duration::from_secs_f32(
            request.max_velocity_rev_per_s / request.deceleration_rev_per_s2,
        )
        .min(UPDATE_BRAKE_TIMEOUT_MAX);
        let settle_time = Duration::from_millis(request.settle_time_ms as u64);
        let timeout = braking_time
            .saturating_add(settle_time)
            .saturating_add(settle_time)
            .saturating_add(UPDATE_HOLD_MARGIN);
        let deadline = Instant::now() + timeout;
        loop {
            if self
                .update_hold_confirmed_generation
                .load(Ordering::Acquire)
                == expected_generation
            {
                return Ok(());
            }
            if !self.is_running() {
                anyhow::bail!("six-motor MIT test stopped before fresh-Pdes hold confirmation");
            }
            if Instant::now() >= deadline {
                anyhow::bail!(
                    "500 Hz MIT loop did not confirm a fresh, settled Pdes hold within {} ms",
                    timeout.as_millis()
                );
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    }

    pub fn begin_shutdown(&self) {
        self.running.store(false, Ordering::Release);
        let mut view = lock_unpoisoned(&self.view);
        if view.phase != "error" && view.phase != "stopped" {
            view.phase = "stopping".into();
        }
        view.running = false;
    }

    /// Stop the 500 Hz loop, send a zero-velocity hold frame, then request
    /// confirmed-safe new-protocol disable on all six axes. Idempotent and retryable.
    pub async fn shutdown(&self) -> anyhow::Result<()> {
        self.shutdown_inner(None).await
    }

    async fn ensure_persistent_disable_hold(&self, command: LastSentCommand) -> Vec<String> {
        let stale = {
            let mut slot = self.disable_hold.lock().await;
            if slot.as_ref().is_some_and(|hold| {
                hold.healthy.load(Ordering::Acquire) && !hold.task.is_finished()
            }) {
                return Vec::new();
            }
            slot.take()
        };
        let mut failures = Vec::new();
        if let Some(stale) = stale {
            stale.running.store(false, Ordering::Release);
            if let Err(error) = stale.task.await {
                failures.push(format!("previous disable-hold task failed: {error}"));
            }
        }
        if let Err(error) = send_control_frames(
            &self.bus,
            self.position_velocity_cob_id,
            self.tff_cob_id,
            command.positions,
            [0.0; MOTOR_COUNT],
            command.tff_commands,
        )
        .await
        {
            failures.push(format!("initial disable-hold frame failed: {error}"));
        }
        let running = Arc::new(AtomicBool::new(true));
        let healthy = Arc::new(AtomicBool::new(true));
        let task = {
            let bus = Arc::clone(&self.bus);
            let positions = Arc::new(StdMutex::new(command.positions));
            let task_running = Arc::clone(&running);
            let task_healthy = Arc::clone(&healthy);
            let position_velocity_cob_id = self.position_velocity_cob_id;
            let tff_cob_id = self.tff_cob_id;
            tokio::spawn(async move {
                startup_hold_loop(
                    bus,
                    position_velocity_cob_id,
                    tff_cob_id,
                    positions,
                    command.tff_commands,
                    task_running,
                    task_healthy,
                )
                .await;
            })
        };
        *self.disable_hold.lock().await = Some(PersistentDisableHold {
            running,
            healthy,
            task,
        });
        failures
    }

    async fn stop_persistent_disable_hold(&self) -> Vec<String> {
        let Some(hold) = self.disable_hold.lock().await.take() else {
            return Vec::new();
        };
        hold.running.store(false, Ordering::Release);
        let mut failures = Vec::new();
        if let Err(error) = hold.task.await {
            failures.push(format!("500 Hz hold task failed during disable: {error}"));
        }
        if !hold.healthy.load(Ordering::Acquire) {
            failures.push("500 Hz hold stream failed during disable".into());
        }
        failures
    }

    async fn persistent_disable_hold_health_error(&self) -> Option<String> {
        let slot = self.disable_hold.lock().await;
        slot.as_ref().and_then(|hold| {
            (!hold.healthy.load(Ordering::Acquire) || hold.task.is_finished())
                .then(|| "500 Hz hold stream is not running after disable failure".into())
        })
    }

    async fn shutdown_inner(&self, primary_error: Option<String>) -> anyhow::Result<()> {
        let _shutdown = self.shutdown_gate.lock().await;
        if let Some(primary) = &primary_error {
            let mut view = lock_unpoisoned(&self.view);
            view.phase = "error".into();
            view.last_error = Some(primary.clone());
        }
        // Stop the single control writer while holding its send gate, and
        // install the persistent hold from the exact last successful frame
        // before releasing that gate. A queued control tick rechecks running
        // after it acquires the gate and therefore cannot write afterward.
        let pdo_send = self.pdo_send_gate.lock().await;
        let initially_enabled = {
            let view = lock_unpoisoned(&self.view);
            view.motors.iter().any(|motor| motor.enabled)
        };
        let mut failures = Vec::new();
        if initially_enabled {
            let last_sent = *lock_unpoisoned(&self.last_sent_command);
            failures.extend(self.ensure_persistent_disable_hold(last_sent).await);
        }
        self.begin_shutdown();
        drop(pdo_send);

        // The dedicated stream is installed before waiting on an in-flight
        // Apply. Fence that SDO transaction, release the gate, then join.
        let sdo_fence = self.sdo_gate.lock().await;
        drop(sdo_fence);
        let task_failure = if let Some(task) = self.task.lock().await.take() {
            task.await
                .err()
                .map(|error| format!("control task failed: {error}"))
        } else {
            None
        };

        let needs_disable = {
            let view = lock_unpoisoned(&self.view);
            view.motors.iter().any(|motor| motor.enabled)
        };
        let disable_failures = if needs_disable {
            let _sdo = self.sdo_gate.lock().await;
            disable_all(&self.manager).await
        } else {
            Vec::new()
        };
        if disable_failures.is_empty() {
            failures.extend(self.stop_persistent_disable_hold().await);
        } else if let Some(error) = self.persistent_disable_hold_health_error().await {
            failures.push(error);
        }
        failures.extend(disable_failures.iter().cloned());
        if let Some(error) = task_failure {
            failures.push(error);
        }
        let mut view = lock_unpoisoned(&self.view);
        view.running = false;
        view.active_motor_index = None;
        for motor in &mut view.motors {
            motor.active = false;
            motor.command_velocity_rev_per_s = 0.0;
            motor.enabled = disable_failures
                .iter()
                .any(|failure| failure.starts_with(&format!("ID {}:", motor.node_id)));
            motor.sequence_state = if motor.enabled {
                "disable-failed"
            } else {
                "disabled"
            }
            .into();
        }
        if failures.is_empty() && primary_error.is_none() {
            if view.phase != "error" {
                view.phase = "stopped".into();
            }
            Ok(())
        } else {
            let mut detail = primary_error.unwrap_or_default();
            if !failures.is_empty() {
                if !detail.is_empty() {
                    detail.push_str("; ");
                }
                detail.push_str(&format!("safe-stop failures: {}", failures.join("; ")));
                detail.push_str(". Immediately remove motor power and do not touch the arm");
            } else if !detail.is_empty() {
                detail.push_str("; all motors were disabled after the failed runtime update");
            }
            view.phase = "error".into();
            view.last_error = Some(detail.clone());
            Err(anyhow::anyhow!(detail))
        }
    }

    async fn control_loop(self: Arc<Self>, mut command_positions: [f32; MOTOR_COUNT]) {
        let mut command_velocities = [0.0f32; MOTOR_COUNT];
        let mut applied_constant_tff = [0.0f32; MOTOR_COUNT];
        let mut applied_gravity_tff = [0.0f32; MOTOR_COUNT];
        let mut waypoint_index = 0usize;
        let mut motor_index = 0usize;
        let mut config_revision = 0u64;
        let mut completed_cycles = 0u64;
        let mut movement_started = Instant::now();
        let mut settle_started: Option<Instant> = None;
        let mut dwell_until: Option<Instant> = None;
        let mut ticker = tokio::time::interval(CONTROL_PERIOD);
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut last_tick = Instant::now();
        let mut failure: Option<String> = None;
        let mut compensation_transition = true;
        let mut compensation_settle_started: Option<Instant> = None;
        let mut update_hold_latched = false;
        let mut update_pre_latch_settle_started: Option<Instant> = None;
        let mut update_settle_started: Option<Instant> = None;
        let mut was_parameters_updating = false;
        let mut observed_update_hold_generation = 0u64;

        while self.running.load(Ordering::Acquire) {
            ticker.tick().await;
            let now = Instant::now();
            let elapsed = now.saturating_duration_since(last_tick);
            last_tick = now;
            if elapsed > CONTROL_JITTER_TRIP {
                failure = Some(format!(
                    "500 Hz MIT control loop was not scheduled for {:.1} ms",
                    elapsed.as_secs_f64() * 1_000.0
                ));
                break;
            }

            let (config, update_hold_fence) = {
                // Keep the host-ceiling config and hold generation/freshness
                // fence as one logical snapshot. The updater uses this lock
                // order when it lowers 0x4572.
                let config = lock_unpoisoned(&self.config);
                let fence = lock_unpoisoned(&self.update_hold_fence);
                (Arc::clone(&config), *fence)
            };
            let parameters_updating = self.updating.load(Ordering::Acquire);
            if parameters_updating {
                let hold_generation = update_hold_fence.generation;
                if !was_parameters_updating || hold_generation != observed_update_hold_generation {
                    update_hold_latched = false;
                    update_pre_latch_settle_started = None;
                    update_settle_started = None;
                    observed_update_hold_generation = hold_generation;
                }
            } else {
                update_hold_latched = false;
                update_pre_latch_settle_started = None;
                update_settle_started = None;
            }
            was_parameters_updating = parameters_updating;
            if config.revision != config_revision {
                // The updater publishes the new immutable config while the
                // settled hold is still active. Mark the torque transition as
                // soon as that revision becomes visible so no scheduling race
                // can apply a changed trim/gravity target without blending.
                compensation_transition = true;
                compensation_settle_started = None;
            }
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
                let mode_age = state.connection.last_tpdo2.map(|instant| instant.elapsed());
                let enabled = matches!(
                    state.logic,
                    Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit))
                );
                let motor_error = match state.logic {
                    Some(MeowMotorLogic::Error {
                        mode_display,
                        detailed_error,
                    }) => Some(format!(
                        "mode 0x{mode_display:02X}, detail 0x{detailed_error:04X}"
                    )),
                    Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit)) => None,
                    Some(other) => Some(format!("unexpected control state {other:?}")),
                    None => Some("MIT control state unavailable".into()),
                };
                let actual_position = meow_position_rev(&state);
                let actual_velocity = meow_velocity_rev_per_s(&state);
                if !state.connection.online {
                    failure = Some(format!("motor {node_id} went offline"));
                } else if age.is_none_or(|value| value > FEEDBACK_STALE_TIMEOUT) {
                    failure = Some(format!("motor {node_id} TPDO1 feedback is stale"));
                } else if mode_age.is_none_or(|value| value > FEEDBACK_STALE_TIMEOUT) {
                    failure = Some(format!(
                        "motor {node_id} TPDO2 mode/fault feedback is stale"
                    ));
                } else if !enabled {
                    failure = Some(format!(
                        "motor {node_id} left enabled MIT mode: {}",
                        motor_error
                            .clone()
                            .unwrap_or_else(|| "unknown state".into())
                    ));
                } else if actual_position.is_none() {
                    failure = Some(format!(
                        "motor {node_id} position feedback is unavailable or non-finite"
                    ));
                } else if actual_velocity.is_none() {
                    failure = Some(format!(
                        "motor {node_id} velocity feedback is unavailable or non-finite"
                    ));
                }
                feedback.push((state, age, motor_error, actual_position, actual_velocity));
            }
            if failure.is_some() {
                break;
            }

            let mut update_hold_ready = false;
            if parameters_updating {
                // First decelerate the commanded trajectory normally. A
                // one-tick Vdes=0 step can itself create a Kd torque jerk.
                if !update_hold_latched {
                    let max_velocity_delta =
                        config.request.deceleration_rev_per_s2 * CONTROL_PERIOD.as_secs_f32();
                    for index in 0..MOTOR_COUNT {
                        command_velocities[index] =
                            approach(command_velocities[index], 0.0, max_velocity_delta);
                        command_positions[index] +=
                            command_velocities[index] * CONTROL_PERIOD.as_secs_f32();
                    }
                    let trajectory_stopped = command_velocities
                        .iter()
                        .all(|velocity| velocity.abs() <= f32::EPSILON);
                    if trajectory_stopped {
                        let baselines = update_hold_fence.after;
                        let all_newer = (0..MOTOR_COUNT).all(|index| {
                            feedback[index]
                                .0
                                .connection
                                .last_tpdo1
                                .is_some_and(|instant| {
                                    baselines[index].is_none_or(|baseline| instant > baseline)
                                })
                        });
                        if all_newer {
                            let position_tolerance = config
                                .request
                                .arrival_position_tolerance_rev
                                .min(HOLD_POSITION_TOLERANCE_MAX_REV);
                            let velocity_tolerance = config
                                .request
                                .arrival_velocity_tolerance_rev_per_s
                                .min(HOLD_VELOCITY_TOLERANCE_MAX_REV_PER_S);
                            let close_and_still = (0..MOTOR_COUNT).all(|index| {
                                let actual = feedback[index]
                                    .3
                                    .expect("validated finite position feedback");
                                let velocity = feedback[index]
                                    .4
                                    .expect("validated finite velocity feedback");
                                (actual - command_positions[index]).abs() <= position_tolerance
                                    && velocity.abs() <= velocity_tolerance
                            });
                            if close_and_still {
                                let since = update_pre_latch_settle_started.get_or_insert(now);
                                if now.saturating_duration_since(*since)
                                    >= Duration::from_millis(config.request.settle_time_ms as u64)
                                {
                                    // Rebase by at most the conservative
                                    // position tolerance, exactly once.
                                    for index in 0..MOTOR_COUNT {
                                        command_positions[index] = feedback[index]
                                            .3
                                            .expect("validated finite position feedback");
                                    }
                                    command_velocities.fill(0.0);
                                    update_hold_latched = true;
                                    update_pre_latch_settle_started = None;
                                    update_settle_started = None;
                                }
                            } else {
                                update_pre_latch_settle_started = None;
                            }
                        }
                    } else {
                        update_pre_latch_settle_started = None;
                    }
                }
                if update_hold_latched {
                    command_velocities.fill(0.0);
                    let position_tolerance = config
                        .request
                        .arrival_position_tolerance_rev
                        .min(HOLD_POSITION_TOLERANCE_MAX_REV);
                    let velocity_tolerance = config
                        .request
                        .arrival_velocity_tolerance_rev_per_s
                        .min(HOLD_VELOCITY_TOLERANCE_MAX_REV_PER_S);
                    let all_axes_still = (0..MOTOR_COUNT).all(|index| {
                        let actual = feedback[index]
                            .3
                            .expect("validated finite position feedback");
                        let velocity = feedback[index]
                            .4
                            .expect("validated finite velocity feedback");
                        (actual - command_positions[index]).abs() <= position_tolerance
                            && velocity.abs() <= velocity_tolerance
                    });
                    if all_axes_still {
                        let since = update_settle_started.get_or_insert(now);
                        update_hold_ready = now.saturating_duration_since(*since)
                            >= Duration::from_millis(config.request.settle_time_ms as u64);
                    } else {
                        update_settle_started = None;
                    }
                } else {
                    update_settle_started = None;
                }
                movement_started = now;
                settle_started = None;
                dwell_until = None;
            } else if config.revision != config_revision {
                config_revision = config.revision;
                waypoint_index = 0;
                motor_index = 0;
                movement_started = now;
                settle_started = None;
                dwell_until = None;
                command_velocities.fill(0.0);
                // Hold this fresh snapshot until constant trim and any changed
                // gravity configuration have blended to the new target.
                compensation_transition = true;
                compensation_settle_started = None;
            } else if compensation_transition {
                command_velocities.fill(0.0);
                movement_started = now;
                settle_started = None;
                dwell_until = None;
            } else if let Some(until) = dwell_until {
                if now >= until {
                    waypoint_index = (waypoint_index + 1) % config.request.waypoints.len();
                    if waypoint_index == 0 {
                        completed_cycles = completed_cycles.saturating_add(1);
                    }
                    motor_index = 0;
                    movement_started = now;
                    settle_started = None;
                    dwell_until = None;
                }
            } else {
                let target = config.request.waypoints[waypoint_index][motor_index];
                let (next_position, next_velocity) = step_trajectory(
                    command_positions[motor_index],
                    command_velocities[motor_index],
                    target,
                    config.request.max_velocity_rev_per_s,
                    config.request.acceleration_rev_per_s2,
                    config.request.deceleration_rev_per_s2,
                    CONTROL_PERIOD.as_secs_f32(),
                );
                command_positions[motor_index] = next_position;
                command_velocities[motor_index] = next_velocity;

                let actual_position = feedback[motor_index].3;
                let actual_velocity = feedback[motor_index].4;
                let trajectory_at_target = (next_position - target).abs() <= f32::EPSILON
                    && next_velocity.abs() <= f32::EPSILON;
                let feedback_at_target = actual_position.is_some_and(|position| {
                    (position - target).abs() <= config.request.arrival_position_tolerance_rev
                }) && actual_velocity.is_some_and(|velocity| {
                    velocity.abs() <= config.request.arrival_velocity_tolerance_rev_per_s
                });
                if trajectory_at_target && feedback_at_target {
                    let since = settle_started.get_or_insert(now);
                    if now.saturating_duration_since(*since)
                        >= Duration::from_millis(config.request.settle_time_ms as u64)
                    {
                        command_positions[motor_index] = target;
                        command_velocities[motor_index] = 0.0;
                        settle_started = None;
                        if motor_index + 1 < MOTOR_COUNT {
                            motor_index += 1;
                            movement_started = now;
                        } else {
                            dwell_until = Some(
                                now + Duration::from_millis(
                                    config.request.waypoint_dwell_ms as u64,
                                ),
                            );
                        }
                    }
                } else {
                    settle_started = None;
                }
                if now.saturating_duration_since(movement_started)
                    > Duration::from_millis(config.request.movement_timeout_ms as u64)
                {
                    failure = Some(format!(
                        "motor {} did not settle at waypoint {} within {} ms",
                        NODE_IDS[motor_index],
                        waypoint_index + 1,
                        config.request.movement_timeout_ms
                    ));
                    break;
                }
            }

            let sensed_positions = std::array::from_fn(|index| {
                feedback[index]
                    .3
                    .expect("validated finite position feedback")
            });
            let gravity =
                match gravity_compensation(&config.dynamics, &config.request, &sensed_positions) {
                    Ok(gravity) => gravity,
                    Err(error) => {
                        failure = Some(format!("gravity compensation failed: {error}"));
                        break;
                    }
                };
            if let Err(error) = validate_joint_limits(&gravity.joint_position_rad, "live feedback")
            {
                failure = Some(format!("MIT joint-limit safety stop: {error}"));
                break;
            }
            let max_tff_delta =
                config.request.tff_slew_rate_nm_per_s * CONTROL_PERIOD.as_secs_f32();
            let mut applied_physical_tff = [0.0f32; MOTOR_COUNT];
            for index in 0..MOTOR_COUNT {
                let tuning = config.request.tuning[index];
                if compensation_transition {
                    slew_tff_components(
                        &mut applied_constant_tff[index],
                        &mut applied_gravity_tff[index],
                        tuning.tff_nm,
                        gravity.target_motor_torque_nm[index],
                        tuning.gravity_max_torque_nm,
                        max_tff_delta,
                    );
                } else {
                    // Once a configuration transition has converged, G(q)
                    // follows fresh feedback every 500 Hz tick. Applying the
                    // transition slew to changing q would lag gravity and make
                    // a correctly configured loaded joint feel weak.
                    applied_constant_tff[index] = tuning.tff_nm;
                    applied_gravity_tff[index] = gravity.target_motor_torque_nm[index]
                        .clamp(-tuning.gravity_max_torque_nm, tuning.gravity_max_torque_nm);
                }
                applied_physical_tff[index] =
                    applied_constant_tff[index] + applied_gravity_tff[index];
            }
            let compensation_torque_converged = (0..MOTOR_COUNT).all(|index| {
                let tuning = config.request.tuning[index];
                (applied_constant_tff[index] - tuning.tff_nm).abs() <= TFF_TRANSITION_EPSILON_NM
                    && (applied_gravity_tff[index] - gravity.target_motor_torque_nm[index]).abs()
                        <= TFF_TRANSITION_EPSILON_NM
            });
            if compensation_transition {
                let position_tolerance = config
                    .request
                    .arrival_position_tolerance_rev
                    .min(HOLD_POSITION_TOLERANCE_MAX_REV);
                let velocity_tolerance = config
                    .request
                    .arrival_velocity_tolerance_rev_per_s
                    .min(HOLD_VELOCITY_TOLERANCE_MAX_REV_PER_S);
                let all_axes_still = (0..MOTOR_COUNT).all(|index| {
                    let actual = feedback[index]
                        .3
                        .expect("validated finite position feedback");
                    let velocity = feedback[index]
                        .4
                        .expect("validated finite velocity feedback");
                    (actual - command_positions[index]).abs() <= position_tolerance
                        && velocity.abs() <= velocity_tolerance
                });
                if compensation_torque_converged && all_axes_still {
                    let since = compensation_settle_started.get_or_insert(now);
                    if now.saturating_duration_since(*since)
                        >= Duration::from_millis(config.request.settle_time_ms as u64)
                    {
                        compensation_transition = false;
                        compensation_settle_started = None;
                        movement_started = now;
                    }
                } else {
                    compensation_settle_started = None;
                }
            }
            let wire_tff = wire_tff_states(
                &config.request,
                &applied_physical_tff,
                &self.torque_factors,
                &self.peak_torque_nm,
            );
            let raw_tff = std::array::from_fn(|index| wire_tff[index].command_nm);
            let _pdo_send = self.pdo_send_gate.lock().await;
            if !self.running.load(Ordering::Acquire) {
                break;
            }
            if let Err(error) = send_control_frames(
                &self.bus,
                self.position_velocity_cob_id,
                self.tff_cob_id,
                command_positions,
                command_velocities,
                raw_tff,
            )
            .await
            {
                failure = Some(format!("MIT RPDO send failed: {error}"));
                break;
            }
            *lock_unpoisoned(&self.last_sent_command) = LastSentCommand {
                positions: command_positions,
                tff_commands: raw_tff,
            };
            let in_dwell = dwell_until.is_some();
            let in_compensation_transition = compensation_transition;
            let mut view = lock_unpoisoned(&self.view);
            view.running = true;
            view.gravity_enabled = config.request.gravity_enabled;
            view.phase = if parameters_updating {
                "updating"
            } else if in_compensation_transition {
                "compensation-ramp"
            } else if in_dwell {
                "dwell"
            } else {
                "moving"
            }
            .into();
            view.active_waypoint_index = waypoint_index;
            view.active_motor_index =
                (!parameters_updating && !in_compensation_transition && !in_dwell)
                    .then_some(motor_index);
            view.completed_cycles = completed_cycles;
            for index in 0..MOTOR_COUNT {
                let (state, age, motor_error, actual, actual_velocity) = &feedback[index];
                let target = config.request.waypoints[waypoint_index][index];
                let motor = &mut view.motors[index];
                motor.online = state.connection.online;
                motor.enabled = matches!(
                    state.logic,
                    Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit))
                );
                motor.active = !parameters_updating
                    && !in_compensation_transition
                    && !in_dwell
                    && index == motor_index;
                motor.sequence_state =
                    if parameters_updating || in_compensation_transition || in_dwell {
                        "holding"
                    } else if index < motor_index {
                        "reached"
                    } else if index == motor_index {
                        if settle_started.is_some() {
                            "settling"
                        } else {
                            "moving"
                        }
                    } else {
                        "waiting"
                    }
                    .into();
                motor.target_position_rev = target;
                motor.command_position_rev = command_positions[index];
                motor.command_velocity_rev_per_s = command_velocities[index];
                motor.actual_position_rev = *actual;
                motor.actual_velocity_rev_per_s = *actual_velocity;
                motor.position_error_rev = actual.map(|position| position - target);
                let tuning = config.request.tuning[index];
                motor.tff_nm = tuning.tff_nm;
                motor.gravity_direction = tuning.gravity_direction;
                motor.gravity_park_position_rev = tuning.gravity_park_position_rev;
                motor.gravity_park_angle_rad = tuning.gravity_park_angle_rad;
                motor.gravity_scale = tuning.gravity_scale;
                motor.gravity_max_torque_nm = tuning.gravity_max_torque_nm;
                motor.joint_position_rad = gravity.joint_position_rad[index];
                motor.model_gravity_torque_nm = gravity.model_torque_nm[index];
                motor.gravity_target_torque_nm = gravity.target_motor_torque_nm[index];
                motor.gravity_limited = gravity.limited[index];
                motor.gravity_torque_nm = applied_gravity_tff[index];
                motor.command_tff_nm = applied_physical_tff[index];
                motor.wire_tff_nm = wire_tff[index].command_nm;
                motor.tff_limit_nm = wire_tff[index].limit_nm;
                motor.tff_utilization_percent = wire_tff[index].utilization_percent;
                motor.tff_limited = wire_tff[index].limited;
                motor.peak_torque_nm = self.peak_torque_nm[index];
                motor.recommended_min_torque_permille =
                    config.recommended_min_torque_permille[index];
                motor.kp_nm_per_rad = tuning.kp_nm_per_rad;
                motor.kd_nm_s_per_rad = tuning.kd_nm_s_per_rad;
                motor.kp_raw = config.wire_tuning[index].kp;
                motor.kd_raw = config.wire_tuning[index].kd;
                motor.kp_kd_limit_permille = tuning.kp_kd_limit_permille;
                motor.max_torque_permille = tuning.max_torque_permille;
                motor.feedback_age_ms = age.map(|value| value.as_millis() as u64);
                motor.error = motor_error.clone();
            }
            if parameters_updating && update_hold_latched && update_hold_ready {
                // Publish the generation only after both PDO frames succeeded
                // and the view contains that exact held command snapshot.
                self.update_hold_confirmed_generation
                    .store(observed_update_hold_generation, Ordering::Release);
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
                    motor.command_velocity_rev_per_s = 0.0;
                    if motor.error.is_none() {
                        motor.error = Some(detail.clone());
                    }
                }
            }
            log::error!("six-motor MIT safety stop: {detail}");
            // Whichever path acquires shutdown_gate owns cleanup. If an
            // external/update shutdown already installed its hold stream, do
            // not wait for the gate (this is the task it is joining) and do
            // not launch a competing stream with a different snapshot.
            if let Ok(_cleanup_owner) = self.shutdown_gate.try_lock() {
                let last_sent = *lock_unpoisoned(&self.last_sent_command);
                let mut hold_failures = self.ensure_persistent_disable_hold(last_sent).await;
                let _sdo = self.sdo_gate.lock().await;
                let disable_failures = disable_all(&self.manager).await;
                if disable_failures.is_empty() {
                    hold_failures.extend(self.stop_persistent_disable_hold().await);
                } else if let Some(error) = self.persistent_disable_hold_health_error().await {
                    hold_failures.push(error);
                }
                hold_failures.extend(disable_failures.iter().cloned());
                let mut view = lock_unpoisoned(&self.view);
                for motor in &mut view.motors {
                    motor.enabled = disable_failures
                        .iter()
                        .any(|failure| failure.starts_with(&format!("ID {}:", motor.node_id)));
                    motor.sequence_state = if motor.enabled {
                        "disable-failed"
                    } else {
                        "disabled"
                    }
                    .into();
                }
                if !hold_failures.is_empty() {
                    let combined = format!(
                            "{detail}; disable failures: {}. Immediately remove motor power and do not touch the arm",
                            hold_failures.join("; ")
                        );
                    view.last_error = Some(combined);
                }
            }
        } else {
            let mut view = lock_unpoisoned(&self.view);
            view.running = false;
        }
        log::info!("six-motor MIT 500 Hz control loop stopped");
    }
}

fn lock_unpoisoned<T>(mutex: &StdMutex<T>) -> StdMutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn meow_position_rev(state: &MeowMotorLiveState) -> Option<f32> {
    state
        .measurements
        .position
        .map(|position| position.to_revolutions() as f32)
        .filter(|position| position.is_finite())
}

fn meow_velocity_rev_per_s(state: &MeowMotorLiveState) -> Option<f32> {
    state.measurements.velocity_rev_per_s.and_then(|velocity| {
        let velocity = velocity as f32;
        velocity.is_finite().then_some(velocity)
    })
}

/// Cancellation-safe scope guard for the live-SDO hold flag.
struct AtomicFlagReset<'a>(&'a AtomicBool);

impl Drop for AtomicFlagReset<'_> {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
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
        device.identity.as_ref().ok_or_else(|| {
            anyhow::anyhow!("motor ID {node_id} has not completed identity discovery")
        })?;
        if device.model.is_none() {
            anyhow::bail!(
                "node ID {node_id} is not an exact supported Meow Motor identity; refusing 0x4102 writes"
            );
        }
    }
    Ok(())
}

async fn setup_all(
    manager: &MeowMotorManager,
    bus: &Arc<dyn CanBus>,
    host_node_id: u8,
    pdo_profile: PdoProfile,
    torque_factors: &[f32; MOTOR_COUNT],
    request: &SixMotorMitRequest,
    dynamics: &Arc<ArmDynamics>,
    shutdown_requested: &AtomicBool,
) -> anyhow::Result<SetupOutcome> {
    if !(1..=127).contains(&host_node_id) {
        anyhow::bail!("host node ID must be 1..=127, got {host_node_id}");
    }
    if NODE_IDS.contains(&host_node_id) {
        anyhow::bail!(
            "host node ID {host_node_id} conflicts with motor IDs 1..=6; reconnect with a different host node ID"
        );
    }
    let position_velocity_cob_id = RPDO1_COB_ID_BASE + host_node_id as u16;
    let tff_cob_id = RPDO2_COB_ID_BASE + host_node_id as u16;
    let mut positions = [0.0f32; MOTOR_COUNT];
    let mut factors = [0.0f32; MOTOR_COUNT];
    let mut peak_torque_nm = [0.0f32; MOTOR_COUNT];

    // Match the Motor Control window: identify every current-protocol motor
    // through MeowMotorManager before making the first configuration write.
    for &node_id in &NODE_IDS {
        if shutdown_requested.load(Ordering::Acquire) {
            anyhow::bail!("six-motor MIT startup cancelled by application shutdown");
        }
        manager
            .identify(node_id)
            .await
            .map_err(|error| anyhow::anyhow!("motor {node_id} identify failed: {error}"))?;
    }
    ensure_six_identified_online(manager)?;

    for (index, &node_id) in NODE_IDS.iter().enumerate() {
        if shutdown_requested.load(Ordering::Acquire) {
            anyhow::bail!("six-motor MIT startup cancelled by application shutdown");
        }
        manager
            .initialize(node_id, pdo_profile)
            .await
            .map_err(|error| anyhow::anyhow!("motor {node_id} initialize failed: {error}"))?;
        configure_uncompressed_rpdos(bus, node_id, index, position_velocity_cob_id, tff_cob_id)
            .await?;
        let (position, factor) = wait_for_position_and_factor(manager, bus, node_id).await?;
        positions[index] = position;
        factors[index] = factor;
        peak_torque_nm[index] = initialized_peak_torque(manager, node_id)?;
    }

    let wire_tuning = convert_all_tuning(&request.tuning, &factors)?;
    for index in 0..MOTOR_COUNT {
        write_initial_target_and_tuning(
            bus,
            NODE_IDS[index],
            positions[index],
            request.tuning[index],
            wire_tuning[index],
        )
        .await?;
    }

    // Configuration can take seconds, during which a gravity-loaded disabled
    // joint may drift. Re-snapshot every encoder immediately before electrical
    // enable and keep a 500 Hz zero-velocity hold stream alive throughout the
    // complete mode-enable ramp. This removes the stale-start-target jump.
    for (index, &node_id) in NODE_IDS.iter().enumerate() {
        positions[index] = wait_for_fresh_position(manager, node_id).await?;
    }
    let mut effective_request = request.clone();
    if effective_request.capture_gravity_park_on_start {
        for index in 0..MOTOR_COUNT {
            effective_request.tuning[index].gravity_park_position_rev = positions[index];
        }
    }
    let initial_recommended_min_torque_permille = preflight_request(
        &effective_request,
        dynamics,
        &positions,
        torque_factors,
        &peak_torque_nm,
    )?;

    // These are deliberately two different states: the hold positions latch
    // at each axis' final pre-enable encoder sample and never follow a loaded
    // joint if it droops; fresh sensed positions are consumed only by the
    // coupled G(q) calculation after the control loop starts.
    let hold_command_positions = Arc::new(StdMutex::new(positions));
    // Both constant trim and coupled gravity start at zero. The control loop
    // slews their sum only after all six drives are enabled and re-snapshotted.
    let initial_tff = [0.0f32; MOTOR_COUNT];
    send_control_frames(
        bus,
        position_velocity_cob_id,
        tff_cob_id,
        positions,
        [0.0; MOTOR_COUNT],
        initial_tff,
    )
    .await
    .map_err(|error| anyhow::anyhow!("initial safe-hold RPDO failed: {error}"))?;
    let hold_running = Arc::new(AtomicBool::new(true));
    let hold_healthy = Arc::new(AtomicBool::new(true));
    let hold_task = {
        let bus = bus.clone();
        let hold_command_positions = hold_command_positions.clone();
        let hold_running = hold_running.clone();
        let hold_healthy = hold_healthy.clone();
        let initial_tff = initial_tff;
        tokio::spawn(async move {
            startup_hold_loop(
                bus,
                position_velocity_cob_id,
                tff_cob_id,
                hold_command_positions,
                initial_tff,
                hold_running,
                hold_healthy,
            )
            .await;
        })
    };
    let enable_result: anyhow::Result<(
        [f32; MOTOR_COUNT],
        [u16; MOTOR_COUNT],
        GravityComputation,
    )> = async {
        for (index, &node_id) in NODE_IDS.iter().enumerate() {
            if shutdown_requested.load(Ordering::Acquire) {
                anyhow::bail!("six-motor MIT startup cancelled before enable");
            }
            let fresh = wait_for_fresh_position(manager, node_id).await?;
            if !hold_healthy.load(Ordering::Acquire) {
                anyhow::bail!("500 Hz safe-hold stream failed before motor {node_id} enable");
            }
            let safe_positions = {
                let mut targets = lock_unpoisoned(&hold_command_positions);
                targets[index] = fresh;
                *targets
            };
            // Do not rely only on the background tick: synchronously put this
            // motor's latest encoder position into its RPDO lane before the
            // new-protocol mode command reaches Enabled(Mit).
            send_control_frames(
                bus,
                position_velocity_cob_id,
                tff_cob_id,
                safe_positions,
                [0.0; MOTOR_COUNT],
                initial_tff,
            )
            .await
            .map_err(|error| {
                anyhow::anyhow!("motor {node_id} pre-enable safe-hold RPDO failed: {error}")
            })?;
            let tuning = effective_request.tuning[index];
            let wire = wire_tuning[index];
            manager
                .set_mode_sdo(
                    node_id,
                    MeowMotorTarget::Mit(MeowMitTarget {
                        position_rev: fresh,
                        velocity_rev_per_s: 0.0,
                        torque_nm: initial_tff[index],
                        kp: wire.kp,
                        kd: wire.kd,
                        kp_kd_limit_permille: tuning.kp_kd_limit_permille,
                    }),
                )
                .await
                .map_err(|error| anyhow::anyhow!("motor {node_id} MIT enable failed: {error}"))?;
            if !hold_healthy.load(Ordering::Acquire) {
                anyhow::bail!("500 Hz safe-hold stream failed while enabling motor {node_id}");
            }
        }

        // Enabling six drives is not instantaneous and a loaded disabled axis
        // can move meanwhile. Fence and collect a second complete snapshot
        // after *all* drives report enabled. This sensed pose drives G(q) and
        // the second preflight, while each Pdes lane deliberately keeps the
        // fresh value latched immediately before that axis was enabled. That
        // preserves any position-loop support until gravity has ramped in.
        let mut final_positions = [0.0f32; MOTOR_COUNT];
        for (index, &node_id) in NODE_IDS.iter().enumerate() {
            final_positions[index] = wait_for_fresh_position(manager, node_id).await?;
            if !hold_healthy.load(Ordering::Acquire) {
                anyhow::bail!("500 Hz safe-hold stream failed during final position snapshot");
            }
        }
        for &node_id in &NODE_IDS {
            wait_for_fresh_enabled_mit(manager, node_id).await?;
        }
        let final_command_positions = *lock_unpoisoned(&hold_command_positions);
        let final_recommended_min_torque_permille = preflight_sensed_and_command_starts(
            &effective_request,
            dynamics,
            &final_positions,
            &final_command_positions,
            torque_factors,
            &peak_torque_nm,
        )?;
        if !hold_healthy.load(Ordering::Acquire) {
            anyhow::bail!("500 Hz safe-hold stream failed after final position snapshot");
        }
        let final_gravity = gravity_compensation(dynamics, &effective_request, &final_positions)?;
        Ok((
            final_positions,
            final_recommended_min_torque_permille,
            final_gravity,
        ))
    }
    .await;
    // If any enable/final-preflight step failed, keep the live 500 Hz hold
    // running while rolling back every axis that may have reached MIT mode.
    let final_command_positions = *lock_unpoisoned(&hold_command_positions);
    let mut startup_hold = PersistentDisableHold {
        running: hold_running,
        healthy: hold_healthy,
        task: hold_task,
    };
    let mut hold_replacement_failures = Vec::new();
    if enable_result.is_err() {
        (startup_hold, hold_replacement_failures) = ensure_live_startup_hold(
            startup_hold,
            bus,
            position_velocity_cob_id,
            tff_cob_id,
            final_command_positions,
            initial_tff,
        )
        .await;
    }
    let rollback_failures = if enable_result.is_err() {
        disable_startup_candidates(manager).await
    } else {
        Vec::new()
    };
    let (positions, recommended_min_torque_permille, initial_gravity) = match enable_result {
        Ok(result) => result,
        Err(error) => {
            let mut detail = error.to_string();
            if !hold_replacement_failures.is_empty() {
                detail.push_str(&format!(
                    "; startup hold replacement diagnostics: {}",
                    hold_replacement_failures.join("; ")
                ));
            }
            if !rollback_failures.is_empty() {
                detail.push_str(&format!(
                    "; startup rollback disable failures: {}. A 500 Hz hold remains active; use Stop to retry disable and immediately remove motor power before touching the arm",
                    rollback_failures.join("; ")
                ));
                if !startup_hold.healthy.load(Ordering::Acquire) {
                    detail.push_str(
                        "; replacement startup hold reported a send failure; Stop will retry it",
                    );
                }
                let remaining_enabled = std::array::from_fn(|index| {
                    let prefix = format!("ID {}:", NODE_IDS[index]);
                    rollback_failures
                        .iter()
                        .any(|failure| failure.starts_with(&prefix))
                });
                let recovery_sensed_positions =
                    current_position_snapshot(manager).unwrap_or(positions);
                let recovery_gravity =
                    gravity_compensation(dynamics, &effective_request, &recovery_sensed_positions)
                        .unwrap_or(GravityComputation {
                            joint_position_rad: joint_positions_rad(
                                &effective_request,
                                &recovery_sensed_positions,
                            )
                            .unwrap_or([0.0; MOTOR_COUNT]),
                            model_torque_nm: [0.0; MOTOR_COUNT],
                            target_motor_torque_nm: [0.0; MOTOR_COUNT],
                            limited: [false; MOTOR_COUNT],
                        });
                return Ok(SetupOutcome::Recovery(StartupRecovery {
                    setup: SetupResult {
                        effective_request,
                        initial_command_positions: final_command_positions,
                        initial_sensed_positions: recovery_sensed_positions,
                        initial_gravity: recovery_gravity,
                        factors,
                        peak_torque_nm,
                        recommended_min_torque_permille: initial_recommended_min_torque_permille,
                        wire_tuning,
                        position_velocity_cob_id,
                        tff_cob_id,
                    },
                    detail,
                    remaining_enabled,
                    hold: startup_hold,
                }));
            }

            // Every candidate was confirmed disabled, so retaining a live PDO
            // writer is unnecessary and would interfere with other bus owners.
            let stop_failures = stop_owned_hold(startup_hold).await;
            if !stop_failures.is_empty() {
                detail.push_str(&format!(
                    "; startup hold shutdown diagnostics: {}",
                    stop_failures.join("; ")
                ));
            }
            anyhow::bail!(detail);
        }
    };
    let hold_stop_failures = stop_owned_hold(startup_hold).await;
    if !hold_stop_failures.is_empty() {
        let mut detail = "500 Hz safe-hold stream failed after all motors were enabled".to_string();
        detail.push_str(&format!("; {}", hold_stop_failures.join("; ")));

        // The original writer has stopped, so immediately replace it before
        // attempting rollback. If rollback cannot confirm every disable, this
        // replacement is transferred into a recovery session for Stop retry.
        if let Err(error) = send_control_frames(
            bus,
            position_velocity_cob_id,
            tff_cob_id,
            final_command_positions,
            [0.0; MOTOR_COUNT],
            initial_tff,
        )
        .await
        {
            detail.push_str(&format!("; replacement hold frame failed: {error}"));
        }
        let recovery_hold = spawn_static_hold(
            bus,
            position_velocity_cob_id,
            tff_cob_id,
            final_command_positions,
            initial_tff,
        );
        let rollback_failures = disable_startup_candidates(manager).await;
        if !rollback_failures.is_empty() {
            detail.push_str(&format!(
                "; startup rollback disable failures: {}. A 500 Hz hold remains active; use Stop to retry disable and immediately remove motor power before touching the arm",
                rollback_failures.join("; ")
            ));
            let remaining_enabled = std::array::from_fn(|index| {
                let prefix = format!("ID {}:", NODE_IDS[index]);
                rollback_failures
                    .iter()
                    .any(|failure| failure.starts_with(&prefix))
            });
            return Ok(SetupOutcome::Recovery(StartupRecovery {
                setup: SetupResult {
                    effective_request,
                    initial_command_positions: final_command_positions,
                    initial_sensed_positions: positions,
                    initial_gravity,
                    factors,
                    peak_torque_nm,
                    recommended_min_torque_permille,
                    wire_tuning,
                    position_velocity_cob_id,
                    tff_cob_id,
                },
                detail,
                remaining_enabled,
                hold: recovery_hold,
            }));
        }
        let replacement_stop_failures = stop_owned_hold(recovery_hold).await;
        if !replacement_stop_failures.is_empty() {
            detail.push_str(&format!(
                "; replacement hold shutdown diagnostics: {}",
                replacement_stop_failures.join("; ")
            ));
        }
        anyhow::bail!(detail);
    }
    send_hold_frames(
        bus,
        position_velocity_cob_id,
        tff_cob_id,
        final_command_positions,
        initial_tff,
    )
    .await;

    Ok(SetupOutcome::Ready(SetupResult {
        effective_request,
        initial_command_positions: final_command_positions,
        initial_sensed_positions: positions,
        initial_gravity,
        factors,
        peak_torque_nm,
        recommended_min_torque_permille,
        wire_tuning,
        position_velocity_cob_id,
        tff_cob_id,
    }))
}

async fn startup_hold_loop(
    bus: Arc<dyn CanBus>,
    position_velocity_cob_id: u16,
    tff_cob_id: u16,
    positions: Arc<StdMutex<[f32; MOTOR_COUNT]>>,
    tff_commands: [f32; MOTOR_COUNT],
    running: Arc<AtomicBool>,
    healthy: Arc<AtomicBool>,
) {
    let mut ticker = tokio::time::interval(CONTROL_PERIOD);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    while running.load(Ordering::Acquire) {
        ticker.tick().await;
        let target = *lock_unpoisoned(&positions);
        if let Err(error) = send_control_frames(
            &bus,
            position_velocity_cob_id,
            tff_cob_id,
            target,
            [0.0; MOTOR_COUNT],
            tff_commands,
        )
        .await
        {
            log::error!("six-motor MIT 500 Hz hold frame send failed: {error}");
            healthy.store(false, Ordering::Release);
            break;
        }
    }
}

fn spawn_static_hold(
    bus: &Arc<dyn CanBus>,
    position_velocity_cob_id: u16,
    tff_cob_id: u16,
    positions: [f32; MOTOR_COUNT],
    tff_commands: [f32; MOTOR_COUNT],
) -> PersistentDisableHold {
    let running = Arc::new(AtomicBool::new(true));
    let healthy = Arc::new(AtomicBool::new(true));
    let task = {
        let bus = Arc::clone(bus);
        let positions = Arc::new(StdMutex::new(positions));
        let task_running = Arc::clone(&running);
        let task_healthy = Arc::clone(&healthy);
        tokio::spawn(async move {
            startup_hold_loop(
                bus,
                position_velocity_cob_id,
                tff_cob_id,
                positions,
                tff_commands,
                task_running,
                task_healthy,
            )
            .await;
        })
    };
    PersistentDisableHold {
        running,
        healthy,
        task,
    }
}

async fn stop_owned_hold(hold: PersistentDisableHold) -> Vec<String> {
    hold.running.store(false, Ordering::Release);
    let was_healthy = Arc::clone(&hold.healthy);
    let mut failures = Vec::new();
    if let Err(error) = hold.task.await {
        failures.push(format!("500 Hz hold task failed: {error}"));
    }
    if !was_healthy.load(Ordering::Acquire) {
        failures.push("500 Hz hold stream reported a send failure".into());
    }
    failures
}

/// Ensure rollback never spends an SDO transaction interval without a PDO
/// writer. A task that already reported a send error or exited is replaced
/// before the first disable request is issued.
async fn ensure_live_startup_hold(
    hold: PersistentDisableHold,
    bus: &Arc<dyn CanBus>,
    position_velocity_cob_id: u16,
    tff_cob_id: u16,
    positions: [f32; MOTOR_COUNT],
    tff_commands: [f32; MOTOR_COUNT],
) -> (PersistentDisableHold, Vec<String>) {
    if hold.healthy.load(Ordering::Acquire) && !hold.task.is_finished() {
        return (hold, Vec::new());
    }

    let mut failures = stop_owned_hold(hold).await;
    if let Err(error) = send_control_frames(
        bus,
        position_velocity_cob_id,
        tff_cob_id,
        positions,
        [0.0; MOTOR_COUNT],
        tff_commands,
    )
    .await
    {
        failures.push(format!("replacement hold frame failed: {error}"));
    }
    (
        spawn_static_hold(
            bus,
            position_velocity_cob_id,
            tff_cob_id,
            positions,
            tff_commands,
        ),
        failures,
    )
}

async fn configure_uncompressed_rpdos(
    bus: &Arc<dyn CanBus>,
    node_id: u8,
    motor_index: usize,
    position_velocity_cob_id: u16,
    tff_cob_id: u16,
) -> anyhow::Result<()> {
    sdo::download_u8(
        &**bus,
        node_id,
        OD_COMPRESSED_MIT,
        COMPRESSED_MIT_SUB_ENABLE,
        UNCOMPRESSED_MIT,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("motor {node_id} select uncompressed MIT: {error}"))?;
    let selector = sdo::upload_u8(
        &**bus,
        node_id,
        OD_COMPRESSED_MIT,
        COMPRESSED_MIT_SUB_ENABLE,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("motor {node_id} read MIT selector: {error}"))?;
    if selector != UNCOMPRESSED_MIT {
        anyhow::bail!("motor {node_id} rejected uncompressed MIT: 0x4103:01 read back {selector}");
    }

    configure_rpdo(
        bus,
        node_id,
        0,
        position_velocity_cob_id,
        position_velocity_rpdo_entries_for_motor(motor_index),
    )
    .await?;
    configure_rpdo(
        bus,
        node_id,
        1,
        tff_cob_id,
        tff_rpdo_entries_for_motor(motor_index),
    )
    .await
}

async fn configure_rpdo(
    bus: &Arc<dyn CanBus>,
    node_id: u8,
    rpdo_index: u8,
    cob_id: u16,
    entries: Vec<TpdoEntry>,
) -> anyhow::Result<()> {
    let expected_mapping: Vec<u32> = entries.iter().map(TpdoEntry::packed).collect();
    let recipe = RpdoRecipe {
        rpdo_index,
        cob_id,
        entries,
        transmission_type: 255,
    };
    let writes = build_rpdo_config_writes(&recipe)
        .map_err(|error| anyhow::anyhow!("motor {node_id} RPDO recipe: {error}"))?;
    for write in writes {
        sdo::download(
            &**bus,
            node_id,
            write.index,
            write.subindex,
            &write.data,
            SDO_TIMEOUT,
        )
        .await
        .map_err(|error| {
            anyhow::anyhow!(
                "motor {node_id} RPDO write 0x{:04X}:{:02X}: {error}",
                write.index,
                write.subindex
            )
        })?;
        tokio::time::sleep(SDO_SETTLE).await;
    }
    let map_index = 0x1600 + u16::from(rpdo_index);
    let comm_index = 0x1400 + u16::from(rpdo_index);
    let count = sdo::upload_u8(&**bus, node_id, map_index, 0, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("motor {node_id} verify RPDO map count: {error}"))?;
    let mapped_cob_id = sdo::upload_u32(&**bus, node_id, comm_index, 1, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("motor {node_id} verify RPDO COB-ID: {error}"))?;
    let transmission_type = sdo::upload_u8(&**bus, node_id, comm_index, 2, SDO_TIMEOUT)
        .await
        .map_err(|error| {
            anyhow::anyhow!("motor {node_id} verify RPDO transmission type: {error}")
        })?;
    if count as usize != expected_mapping.len()
        || mapped_cob_id != cob_id as u32
        || transmission_type != 255
    {
        anyhow::bail!(
                "motor {node_id} RPDO{} verification failed: count={count}, COB-ID=0x{mapped_cob_id:08X}, transmission={transmission_type}",
                rpdo_index + 1
        );
    }
    for (index, expected) in expected_mapping.into_iter().enumerate() {
        let subindex = (index + 1) as u8;
        let actual = sdo::upload_u32(&**bus, node_id, map_index, subindex, SDO_TIMEOUT)
            .await
            .map_err(|error| {
                anyhow::anyhow!(
                    "motor {node_id} verify RPDO{} mapping 0x{map_index:04X}:{subindex:02X}: {error}",
                    rpdo_index + 1
                )
            })?;
        if actual != expected {
            anyhow::bail!(
                "motor {node_id} RPDO{} mapping 0x{map_index:04X}:{subindex:02X} read back 0x{actual:08X}, expected 0x{expected:08X}",
                rpdo_index + 1
            );
        }
    }
    Ok(())
}

fn position_velocity_rpdo_entries_for_motor(motor_index: usize) -> Vec<TpdoEntry> {
    let mut entries = Vec::with_capacity(MOTOR_COUNT * 2);
    for index in 0..MOTOR_COUNT {
        if index == motor_index {
            entries.push(TpdoEntry {
                index: OD_MIT,
                subindex: MIT_SUB_PDES,
                bit_len: 32,
            });
            entries.push(TpdoEntry {
                index: OD_MIT,
                subindex: MIT_SUB_VDES,
                bit_len: 32,
            });
        } else {
            entries.push(PAD_ENTRY);
            entries.push(PAD_ENTRY);
        }
    }
    entries
}

fn tff_rpdo_entries_for_motor(motor_index: usize) -> Vec<TpdoEntry> {
    (0..MOTOR_COUNT)
        .map(|index| {
            if index == motor_index {
                TpdoEntry {
                    index: OD_MIT,
                    subindex: MIT_SUB_TFF,
                    bit_len: 32,
                }
            } else {
                PAD_ENTRY
            }
        })
        .collect()
}

async fn wait_for_position_and_factor(
    manager: &MeowMotorManager,
    bus: &Arc<dyn CanBus>,
    node_id: u8,
) -> anyhow::Result<(f32, f32)> {
    let position = wait_for_fresh_position(manager, node_id).await?;
    let factor = sdo::upload_f32(&**bus, node_id, OD_MIT, MIT_SUB_FACTOR, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("motor {node_id} read 0x4102:07 factor: {error}"))?;
    if !factor.is_finite() || factor <= f32::EPSILON {
        anyhow::bail!("motor {node_id} returned invalid 0x4102:07 factor {factor}");
    }
    Ok((position, factor))
}

fn initialized_peak_torque(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<f32> {
    let peak = manager
        .list()
        .into_iter()
        .find(|motor| motor.node_id == node_id)
        .and_then(|motor| motor.peak_torque_nm)
        .ok_or_else(|| {
            anyhow::anyhow!(
                "motor {node_id} did not expose peak torque after initialization; refusing gravity feed-forward"
            )
        })?;
    if !peak.is_finite() || peak <= f32::EPSILON {
        anyhow::bail!("motor {node_id} returned invalid peak torque {peak} Nm");
    }
    Ok(peak)
}

async fn wait_for_fresh_position(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<f32> {
    let baseline = manager
        .status(node_id)
        .map_err(|error| anyhow::anyhow!("motor {node_id} status unavailable: {error}"))?
        .connection
        .last_tpdo1;
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let state = manager
            .status(node_id)
            .map_err(|error| anyhow::anyhow!("motor {node_id} status unavailable: {error}"))?;
        let fresh = state.connection.last_tpdo1.is_some_and(|instant| {
            baseline.is_none_or(|previous| instant > previous)
                && instant.elapsed() <= FEEDBACK_STALE_TIMEOUT
        });
        if state.connection.online && fresh {
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

async fn wait_for_fresh_enabled_mit(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<()> {
    let baseline = manager
        .status(node_id)
        .map_err(|error| anyhow::anyhow!("motor {node_id} status unavailable: {error}"))?
        .connection
        .last_tpdo2;
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let state = manager
            .status(node_id)
            .map_err(|error| anyhow::anyhow!("motor {node_id} status unavailable: {error}"))?;
        let fresh = state.connection.last_tpdo2.is_some_and(|instant| {
            baseline.is_none_or(|previous| instant > previous)
                && instant.elapsed() <= FEEDBACK_STALE_TIMEOUT
        });
        if state.connection.online && fresh {
            match state.logic {
                Some(MeowMotorLogic::Enabled(MeowMotorMode::Mit)) => return Ok(()),
                Some(other) => {
                    anyhow::bail!("motor {node_id} left enabled MIT mode during startup: {other:?}")
                }
                None => anyhow::bail!(
                    "motor {node_id} returned fresh TPDO2 without a decoded MIT state"
                ),
            }
        }
        if Instant::now() >= deadline {
            anyhow::bail!(
                "motor {node_id} did not confirm enabled MIT mode on a fresh TPDO2 within 2 seconds"
            );
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

fn convert_all_tuning(
    tuning: &[SixMotorMitTuning; MOTOR_COUNT],
    factors: &[f32; MOTOR_COUNT],
) -> anyhow::Result<[WireTuning; MOTOR_COUNT]> {
    let mut wire = [WireTuning::default(); MOTOR_COUNT];
    for index in 0..MOTOR_COUNT {
        wire[index] = WireTuning {
            kp: gain_to_raw(tuning[index].kp_nm_per_rad, factors[index])
                .map_err(|error| anyhow::anyhow!("motor {} Kp: {error}", NODE_IDS[index]))?,
            kd: gain_to_raw(tuning[index].kd_nm_s_per_rad, factors[index])
                .map_err(|error| anyhow::anyhow!("motor {} Kd: {error}", NODE_IDS[index]))?,
        };
    }
    Ok(wire)
}

fn validate_torque_factors(factors: &[f32; MOTOR_COUNT]) -> anyhow::Result<()> {
    for (index, factor) in factors.iter().copied().enumerate() {
        if !factor.is_finite() || factor <= 0.0 || factor > f32::MAX / 1_000.0 {
            anyhow::bail!(
                "motor {} factory torque factor must be finite and positive, got {factor}",
                NODE_IDS[index]
            );
        }
    }
    Ok(())
}

fn configured_dynamics(base: &ArmDynamics, request: &SixMotorMitRequest) -> ArmDynamics {
    base.with_tip_payload(request.payload_mass_kg, request.payload_center_of_mass_m)
}

fn current_position_snapshot(manager: &MeowMotorManager) -> anyhow::Result<[f32; MOTOR_COUNT]> {
    let mut positions = [0.0f32; MOTOR_COUNT];
    for (index, &node_id) in NODE_IDS.iter().enumerate() {
        let state = manager
            .status(node_id)
            .map_err(|error| anyhow::anyhow!("motor {node_id} status unavailable: {error}"))?;
        if !state.connection.online {
            anyhow::bail!("motor {node_id} is offline during MIT preflight");
        }
        if state
            .connection
            .last_tpdo1
            .is_none_or(|instant| instant.elapsed() > FEEDBACK_STALE_TIMEOUT)
        {
            anyhow::bail!("motor {node_id} TPDO1 is stale during MIT preflight");
        }
        positions[index] = meow_position_rev(&state).ok_or_else(|| {
            anyhow::anyhow!("motor {node_id} position is unavailable during MIT preflight")
        })?;
    }
    Ok(positions)
}

fn current_command_snapshot(view: &SixMotorMitView) -> anyhow::Result<[f32; MOTOR_COUNT]> {
    if view.motors.len() != MOTOR_COUNT {
        anyhow::bail!(
            "MIT view has {} motors, expected {MOTOR_COUNT}",
            view.motors.len()
        );
    }
    let positions = std::array::from_fn(|index| view.motors[index].command_position_rev);
    for (index, position) in positions.iter().enumerate() {
        if !position.is_finite() {
            anyhow::bail!(
                "motor {} command position is non-finite during MIT preflight",
                NODE_IDS[index]
            );
        }
    }
    Ok(positions)
}

fn tpdo1_baselines(manager: &MeowMotorManager) -> anyhow::Result<[Option<Instant>; MOTOR_COUNT]> {
    let mut baselines = [None; MOTOR_COUNT];
    for (index, &node_id) in NODE_IDS.iter().enumerate() {
        baselines[index] = manager
            .status(node_id)
            .map_err(|error| anyhow::anyhow!("motor {node_id} status unavailable: {error}"))?
            .connection
            .last_tpdo1;
    }
    Ok(baselines)
}

#[derive(Clone, Copy)]
struct GravityComputation {
    joint_position_rad: [f32; MOTOR_COUNT],
    model_torque_nm: [f32; MOTOR_COUNT],
    target_motor_torque_nm: [f32; MOTOR_COUNT],
    limited: [bool; MOTOR_COUNT],
}

fn joint_positions_rad(
    request: &SixMotorMitRequest,
    positions: &[f32; MOTOR_COUNT],
) -> anyhow::Result<[f32; MOTOR_COUNT]> {
    let joint_positions = std::array::from_fn(|index| {
        let tuning = request.tuning[index];
        tuning.gravity_park_angle_rad
            + f32::from(tuning.gravity_direction)
                * TAU_F32
                * (positions[index] - tuning.gravity_park_position_rev)
    });
    for (index, position) in joint_positions.iter().enumerate() {
        if !position.is_finite() {
            anyhow::bail!(
                "motor {} maps to a non-finite URDF joint angle",
                NODE_IDS[index]
            );
        }
    }
    Ok(joint_positions)
}

fn validate_joint_limits(
    joint_positions_rad: &[f32; MOTOR_COUNT],
    sample_label: &str,
) -> anyhow::Result<()> {
    for (index, &joint) in joint_positions_rad.iter().enumerate() {
        let minimum = JOINT_MIN_RAD[index] - JOINT_LIMIT_TOLERANCE_RAD;
        let maximum = JOINT_MAX_RAD[index] + JOINT_LIMIT_TOLERANCE_RAD;
        if !(minimum..=maximum).contains(&joint) {
            anyhow::bail!(
                "{sample_label} maps motor {} to joint_{}={joint:.4} rad, outside URDF [{:.4}, {:.4}] rad (plus {:.3} rad tolerance); verify park pose, encoder direction and waypoints",
                NODE_IDS[index],
                index + 1,
                JOINT_MIN_RAD[index],
                JOINT_MAX_RAD[index],
                JOINT_LIMIT_TOLERANCE_RAD
            );
        }
    }
    Ok(())
}

fn gravity_compensation(
    dynamics: &ArmDynamics,
    request: &SixMotorMitRequest,
    positions: &[f32; MOTOR_COUNT],
) -> anyhow::Result<GravityComputation> {
    let joint_position_rad = joint_positions_rad(request, positions)?;
    let model = dynamics.gravity_torque_with(&joint_position_rad, request.gravity_vector_m_per_s2);
    if model.len() != MOTOR_COUNT {
        anyhow::bail!(
            "gravity model returned {} torques, expected {MOTOR_COUNT}",
            model.len()
        );
    }
    let mut model_torque_nm = [0.0f32; MOTOR_COUNT];
    let mut target_motor_torque_nm = [0.0f32; MOTOR_COUNT];
    let mut limited = [false; MOTOR_COUNT];
    for index in 0..MOTOR_COUNT {
        let model_torque = model[index];
        if !model_torque.is_finite() {
            anyhow::bail!(
                "gravity model returned non-finite torque for motor {}",
                NODE_IDS[index]
            );
        }
        model_torque_nm[index] = model_torque;
        if request.gravity_enabled {
            let tuning = request.tuning[index];
            let requested =
                f32::from(tuning.gravity_direction) * tuning.gravity_scale * model_torque;
            let clamped =
                requested.clamp(-tuning.gravity_max_torque_nm, tuning.gravity_max_torque_nm);
            target_motor_torque_nm[index] = clamped;
            limited[index] = !float_nearly_equal(requested, clamped);
        }
    }
    Ok(GravityComputation {
        joint_position_rad,
        model_torque_nm,
        target_motor_torque_nm,
        limited,
    })
}

#[derive(Clone, Copy, Default)]
struct WireTffState {
    command_nm: f32,
    limit_nm: f32,
    utilization_percent: f32,
    limited: bool,
}

fn slew_tff_components(
    applied_constant_nm: &mut f32,
    applied_gravity_nm: &mut f32,
    target_constant_nm: f32,
    target_gravity_nm: f32,
    gravity_hard_limit_nm: f32,
    total_delta_budget_nm: f32,
) {
    // A reduced gravity clamp is a hard safety limit and therefore takes
    // effect immediately. Preserve the preceding *total* while reclassifying
    // the excess as transition/constant torque; otherwise opposing constant
    // and gravity terms could turn a gravity-clamp reduction into a large
    // one-frame output step. The reclassified part then slews to the requested
    // constant target under the same shared budget.
    let preceding_total = *applied_constant_nm + *applied_gravity_nm;
    *applied_gravity_nm =
        (*applied_gravity_nm).clamp(-gravity_hard_limit_nm, gravity_hard_limit_nm);
    *applied_constant_nm = preceding_total - *applied_gravity_nm;
    let constant_delta = target_constant_nm - *applied_constant_nm;
    let gravity_delta = target_gravity_nm - *applied_gravity_nm;
    let total_distance = constant_delta.abs() + gravity_delta.abs();
    if total_distance <= total_delta_budget_nm {
        *applied_constant_nm = target_constant_nm;
        *applied_gravity_nm = target_gravity_nm;
    } else if total_distance > f32::EPSILON {
        let fraction = total_delta_budget_nm / total_distance;
        *applied_constant_nm += constant_delta * fraction;
        *applied_gravity_nm += gravity_delta * fraction;
    }
    *applied_gravity_nm =
        (*applied_gravity_nm).clamp(-gravity_hard_limit_nm, gravity_hard_limit_nm);
}

fn wire_tff_state(
    tuning: SixMotorMitTuning,
    applied_physical_tff_nm: f32,
    torque_factor: f32,
    peak_torque_nm: f32,
) -> WireTffState {
    let requested_wire_nm = applied_physical_tff_nm * torque_factor;
    let limit_nm = peak_torque_nm * f32::from(tuning.max_torque_permille) / 1_000.0;
    let command_nm = requested_wire_nm.clamp(-limit_nm, limit_nm);
    let utilization_percent = if limit_nm > f32::EPSILON {
        command_nm.abs() / limit_nm * 100.0
    } else {
        0.0
    };
    WireTffState {
        command_nm,
        limit_nm,
        utilization_percent,
        limited: !float_nearly_equal(requested_wire_nm, command_nm),
    }
}

fn wire_tff_states(
    request: &SixMotorMitRequest,
    applied_physical_tff_nm: &[f32; MOTOR_COUNT],
    torque_factors: &[f32; MOTOR_COUNT],
    peak_torque_nm: &[f32; MOTOR_COUNT],
) -> [WireTffState; MOTOR_COUNT] {
    std::array::from_fn(|index| {
        wire_tff_state(
            request.tuning[index],
            applied_physical_tff_nm[index],
            torque_factors[index],
            peak_torque_nm[index],
        )
    })
}

fn preflight_request(
    request: &SixMotorMitRequest,
    dynamics: &ArmDynamics,
    current_positions: &[f32; MOTOR_COUNT],
    torque_factors: &[f32; MOTOR_COUNT],
    peak_torque_nm: &[f32; MOTOR_COUNT],
) -> anyhow::Result<[u16; MOTOR_COUNT]> {
    let mut maximum_required_wire_nm = [0.0f32; MOTOR_COUNT];
    evaluate_preflight_sample(
        request,
        dynamics,
        current_positions,
        torque_factors,
        &mut maximum_required_wire_nm,
        "current position",
    )?;

    // Sample the actual serialized path: current -> W1 -> W2 -> W3 -> W4,
    // one axis at a time, plus W4 -> W1 for the repeating cycle. This catches
    // coupled-gravity maxima that do not occur at waypoint endpoints.
    let mut cursor = *current_positions;
    for leg in 0..=request.waypoints.len() {
        let waypoint_index = leg % request.waypoints.len();
        let target = request.waypoints[waypoint_index];
        for motor_index in 0..MOTOR_COUNT {
            let start = cursor[motor_index];
            for sample_index in 1..=PREFLIGHT_SAMPLES_PER_AXIS {
                let fraction = sample_index as f32 / PREFLIGHT_SAMPLES_PER_AXIS as f32;
                let mut sample = cursor;
                sample[motor_index] = start + (target[motor_index] - start) * fraction;
                evaluate_preflight_sample(
                    request,
                    dynamics,
                    &sample,
                    torque_factors,
                    &mut maximum_required_wire_nm,
                    &format!(
                        "path to waypoint {}, motor {}, sample {}",
                        waypoint_index + 1,
                        NODE_IDS[motor_index],
                        sample_index
                    ),
                )?;
            }
            cursor[motor_index] = target[motor_index];
        }
    }

    let mut recommended = [0u16; MOTOR_COUNT];
    for index in 0..MOTOR_COUNT {
        let required_with_headroom = maximum_required_wire_nm[index] * TORQUE_HEADROOM;
        let tff_permille = (required_with_headroom / peak_torque_nm[index] * 1_000.0).ceil();
        if !tff_permille.is_finite() || tff_permille > 1_000.0 {
            anyhow::bail!(
                "motor {} needs {:.3} Nm raw Tff including {:.0}% headroom, beyond its {:.3} Nm peak; lower Tff/gravity scale or payload",
                NODE_IDS[index],
                required_with_headroom,
                (TORQUE_HEADROOM - 1.0) * 100.0,
                peak_torque_nm[index]
            );
        }
        let combined_permille =
            tff_permille.max(0.0) as u32 + u32::from(request.tuning[index].kp_kd_limit_permille);
        if combined_permille > 1_000 {
            anyhow::bail!(
                "motor {} needs {} permille for Tff (including {:.0}% headroom) plus {} permille for Kp/Kd, beyond the 1000 permille drive limit",
                NODE_IDS[index],
                tff_permille.max(0.0) as u16,
                (TORQUE_HEADROOM - 1.0) * 100.0,
                request.tuning[index].kp_kd_limit_permille
            );
        }
        recommended[index] = combined_permille as u16;
        if request.tuning[index].max_torque_permille < recommended[index] {
            anyhow::bail!(
                "motor {} maximum torque is {} permille, but this path needs at least {} permille (peak {:.3} Nm, Tff includes {:.0}% headroom plus {} permille Kp/Kd reserve)",
                NODE_IDS[index],
                request.tuning[index].max_torque_permille,
                recommended[index],
                peak_torque_nm[index],
                (TORQUE_HEADROOM - 1.0) * 100.0,
                request.tuning[index].kp_kd_limit_permille
            );
        }
    }
    Ok(recommended)
}

fn preflight_sensed_and_command_starts(
    request: &SixMotorMitRequest,
    dynamics: &ArmDynamics,
    sensed_positions: &[f32; MOTOR_COUNT],
    command_positions: &[f32; MOTOR_COUNT],
    torque_factors: &[f32; MOTOR_COUNT],
    peak_torque_nm: &[f32; MOTOR_COUNT],
) -> anyhow::Result<[u16; MOTOR_COUNT]> {
    let sensed = preflight_request(
        request,
        dynamics,
        sensed_positions,
        torque_factors,
        peak_torque_nm,
    )?;
    let commanded = preflight_request(
        request,
        dynamics,
        command_positions,
        torque_factors,
        peak_torque_nm,
    )?;
    Ok(std::array::from_fn(|index| {
        sensed[index].max(commanded[index])
    }))
}

fn evaluate_preflight_sample(
    request: &SixMotorMitRequest,
    dynamics: &ArmDynamics,
    positions: &[f32; MOTOR_COUNT],
    torque_factors: &[f32; MOTOR_COUNT],
    maximum_required_wire_nm: &mut [f32; MOTOR_COUNT],
    sample_label: &str,
) -> anyhow::Result<()> {
    let gravity = gravity_compensation(dynamics, request, positions)?;
    validate_joint_limits(&gravity.joint_position_rad, sample_label)?;
    for index in 0..MOTOR_COUNT {
        let physical = request.tuning[index].tff_nm + gravity.target_motor_torque_nm[index];
        let required_wire = (physical * torque_factors[index]).abs();
        let startup_constant_wire = (request.tuning[index].tff_nm * torque_factors[index]).abs();
        maximum_required_wire_nm[index] = maximum_required_wire_nm[index]
            .max(required_wire)
            .max(startup_constant_wire);
    }
    Ok(())
}

fn sdo_tuning_changed(old: SixMotorMitTuning, new: SixMotorMitTuning) -> bool {
    old.kp_nm_per_rad.to_bits() != new.kp_nm_per_rad.to_bits()
        || old.kd_nm_s_per_rad.to_bits() != new.kd_nm_s_per_rad.to_bits()
        || old.kp_kd_limit_permille != new.kp_kd_limit_permille
        || old.max_torque_permille != new.max_torque_permille
}

fn update_view_tuning(
    view: &mut SixMotorMitView,
    request: &SixMotorMitRequest,
    wire_tuning: &[WireTuning; MOTOR_COUNT],
    recommended_min_torque_permille: &[u16; MOTOR_COUNT],
) {
    for index in 0..MOTOR_COUNT {
        if let Some(motor) = view.motors.get_mut(index) {
            let tuning = request.tuning[index];
            motor.tff_nm = tuning.tff_nm;
            motor.gravity_direction = tuning.gravity_direction;
            motor.gravity_park_position_rev = tuning.gravity_park_position_rev;
            motor.gravity_park_angle_rad = tuning.gravity_park_angle_rad;
            motor.gravity_scale = tuning.gravity_scale;
            motor.gravity_max_torque_nm = tuning.gravity_max_torque_nm;
            motor.kp_nm_per_rad = tuning.kp_nm_per_rad;
            motor.kd_nm_s_per_rad = tuning.kd_nm_s_per_rad;
            motor.kp_raw = wire_tuning[index].kp;
            motor.kd_raw = wire_tuning[index].kd;
            motor.kp_kd_limit_permille = tuning.kp_kd_limit_permille;
            motor.max_torque_permille = tuning.max_torque_permille;
            motor.recommended_min_torque_permille = recommended_min_torque_permille[index];
        }
    }
}

fn float_nearly_equal(left: f32, right: f32) -> bool {
    (left - right).abs() <= (1.0e-6f32).max(left.abs().max(right.abs()) * 1.0e-5)
}

fn gain_to_raw(gain_si: f64, factor_nm_per_rev: f32) -> Result<u16, String> {
    if !gain_si.is_finite() || gain_si < 0.0 {
        return Err("gain must be finite and non-negative".into());
    }
    if !factor_nm_per_rev.is_finite() || factor_nm_per_rev <= f32::EPSILON {
        return Err("0x4102:07 factor must be finite and positive".into());
    }
    let raw = (gain_si * TAU_F64 / factor_nm_per_rev as f64).round();
    if raw > u16::MAX as f64 {
        return Err(format!(
            "{gain_si} SI converts to {raw}, exceeding the 0x4102 u16 range"
        ));
    }
    Ok(raw as u16)
}

async fn write_initial_target_and_tuning(
    bus: &Arc<dyn CanBus>,
    node_id: u8,
    position: f32,
    tuning: SixMotorMitTuning,
    wire: WireTuning,
) -> anyhow::Result<()> {
    sdo::download_f32(&**bus, node_id, OD_MIT, MIT_SUB_PDES, position, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("motor {node_id} initialize Pdes: {error}"))?;
    sdo::download_f32(&**bus, node_id, OD_MIT, MIT_SUB_VDES, 0.0, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("motor {node_id} initialize Vdes: {error}"))?;
    // Never step either constant trim or gravity into a disabled/enabling
    // drive. The complete physical Tff ramps from zero in the control loop.
    let initial_tff = 0.0;
    sdo::download_f32(
        &**bus,
        node_id,
        OD_MIT,
        MIT_SUB_TFF,
        initial_tff,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("motor {node_id} initialize Tff: {error}"))?;
    write_and_verify_tuning(bus, node_id, tuning, wire).await?;
    let observed_tff = sdo::upload_f32(&**bus, node_id, OD_MIT, MIT_SUB_TFF, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("motor {node_id} verify initial Tff: {error}"))?;
    if !float_readback_matches(observed_tff, initial_tff) {
        anyhow::bail!(
            "motor {node_id} initial Tff readback mismatch: expected {initial_tff}, got {observed_tff}"
        );
    }
    Ok(())
}

async fn write_and_verify_runtime_tuning(
    bus: &Arc<dyn CanBus>,
    node_id: u8,
    old: SixMotorMitTuning,
    new: SixMotorMitTuning,
    old_wire: WireTuning,
    new_wire: WireTuning,
) -> anyhow::Result<()> {
    sdo::download_u8(
        &**bus,
        node_id,
        OD_COMPRESSED_MIT,
        COMPRESSED_MIT_SUB_ENABLE,
        UNCOMPRESSED_MIT,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("select uncompressed MIT: {error}"))?;
    tokio::time::sleep(SDO_SETTLE).await;
    let selector = sdo::upload_u8(
        &**bus,
        node_id,
        OD_COMPRESSED_MIT,
        COMPRESSED_MIT_SUB_ENABLE,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("read uncompressed selector: {error}"))?;
    if selector != UNCOMPRESSED_MIT {
        anyhow::bail!("uncompressed selector read back {selector}");
    }

    // Lower the global drive ceiling before any feedback authority can grow.
    if new.max_torque_permille < old.max_torque_permille {
        write_and_verify_u16(
            bus,
            node_id,
            OD_MAX_TORQUE,
            0,
            new.max_torque_permille,
            "max torque",
        )
        .await?;
    }
    if new.kp_kd_limit_permille < old.kp_kd_limit_permille {
        write_and_verify_u16(
            bus,
            node_id,
            OD_MIT,
            MIT_SUB_KP_KD_LIMIT,
            new.kp_kd_limit_permille,
            "Kp/Kd output limit",
        )
        .await?;
    }
    if new_wire.kp != old_wire.kp {
        write_and_verify_u16(bus, node_id, OD_MIT, MIT_SUB_KP, new_wire.kp, "Kp").await?;
    }
    if new_wire.kd != old_wire.kd {
        write_and_verify_u16(bus, node_id, OD_MIT, MIT_SUB_KD, new_wire.kd, "Kd").await?;
    }
    if new.kp_kd_limit_permille > old.kp_kd_limit_permille {
        write_and_verify_u16(
            bus,
            node_id,
            OD_MIT,
            MIT_SUB_KP_KD_LIMIT,
            new.kp_kd_limit_permille,
            "Kp/Kd output limit",
        )
        .await?;
    }
    // Raise the global ceiling only after the requested gains and feedback
    // limit are active against a confirmed zero-error Pdes hold.
    if new.max_torque_permille > old.max_torque_permille {
        write_and_verify_u16(
            bus,
            node_id,
            OD_MAX_TORQUE,
            0,
            new.max_torque_permille,
            "max torque",
        )
        .await?;
    }
    Ok(())
}

async fn write_and_verify_u16(
    bus: &Arc<dyn CanBus>,
    node_id: u8,
    index: u16,
    subindex: u8,
    expected: u16,
    label: &str,
) -> anyhow::Result<()> {
    sdo::download_u16(&**bus, node_id, index, subindex, expected, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("write {label}: {error}"))?;
    tokio::time::sleep(SDO_SETTLE).await;
    let actual = sdo::upload_u16(&**bus, node_id, index, subindex, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("read {label}: {error}"))?;
    if actual != expected {
        anyhow::bail!("{label} readback mismatch: expected {expected}, got {actual}");
    }
    Ok(())
}

async fn write_and_verify_tuning(
    bus: &Arc<dyn CanBus>,
    node_id: u8,
    tuning: SixMotorMitTuning,
    wire: WireTuning,
) -> anyhow::Result<()> {
    sdo::download_u8(
        &**bus,
        node_id,
        OD_COMPRESSED_MIT,
        COMPRESSED_MIT_SUB_ENABLE,
        UNCOMPRESSED_MIT,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("select uncompressed MIT: {error}"))?;
    tokio::time::sleep(SDO_SETTLE).await;
    sdo::download_u16(&**bus, node_id, OD_MIT, MIT_SUB_KP, wire.kp, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("write Kp: {error}"))?;
    tokio::time::sleep(SDO_SETTLE).await;
    sdo::download_u16(&**bus, node_id, OD_MIT, MIT_SUB_KD, wire.kd, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("write Kd: {error}"))?;
    tokio::time::sleep(SDO_SETTLE).await;
    sdo::download_u16(
        &**bus,
        node_id,
        OD_MIT,
        MIT_SUB_KP_KD_LIMIT,
        tuning.kp_kd_limit_permille,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("write Kp/Kd output limit: {error}"))?;
    tokio::time::sleep(SDO_SETTLE).await;
    sdo::download_u16(
        &**bus,
        node_id,
        OD_MAX_TORQUE,
        0,
        tuning.max_torque_permille,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("write max torque: {error}"))?;

    let selector = sdo::upload_u8(
        &**bus,
        node_id,
        OD_COMPRESSED_MIT,
        COMPRESSED_MIT_SUB_ENABLE,
        SDO_TIMEOUT,
    )
    .await
    .map_err(|error| anyhow::anyhow!("read uncompressed selector: {error}"))?;
    let kp = sdo::upload_u16(&**bus, node_id, OD_MIT, MIT_SUB_KP, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("read Kp: {error}"))?;
    let kd = sdo::upload_u16(&**bus, node_id, OD_MIT, MIT_SUB_KD, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("read Kd: {error}"))?;
    let limit = sdo::upload_u16(&**bus, node_id, OD_MIT, MIT_SUB_KP_KD_LIMIT, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("read Kp/Kd output limit: {error}"))?;
    let max_torque = sdo::upload_u16(&**bus, node_id, OD_MAX_TORQUE, 0, SDO_TIMEOUT)
        .await
        .map_err(|error| anyhow::anyhow!("read max torque: {error}"))?;

    if selector != UNCOMPRESSED_MIT
        || kp != wire.kp
        || kd != wire.kd
        || limit != tuning.kp_kd_limit_permille
        || max_torque != tuning.max_torque_permille
    {
        anyhow::bail!(
            "SDO readback mismatch: selector={selector}, Kp={kp}, Kd={kd}, limit={limit}, maxTorque={max_torque}"
        );
    }
    Ok(())
}

fn float_readback_matches(actual: f32, expected: f32) -> bool {
    actual.is_finite() && (actual - expected).abs() <= (1.0e-6f32).max(expected.abs() * 1.0e-5)
}

fn build_position_velocity_frame(
    cob_id: u16,
    positions: [f32; MOTOR_COUNT],
    velocities: [f32; MOTOR_COUNT],
) -> Result<CanFrame, can_transport::CanIoError> {
    let mut data = [0u8; POSITION_VELOCITY_FRAME_BYTES];
    for index in 0..MOTOR_COUNT {
        let offset = index * POSITION_VELOCITY_LANE_BYTES;
        data[offset..offset + 4].copy_from_slice(&positions[index].to_le_bytes());
        data[offset + 4..offset + 8].copy_from_slice(&velocities[index].to_le_bytes());
    }
    CanFrame::new_fd(cob_id, &data, true)
}

fn build_tff_frame(
    cob_id: u16,
    tff_commands: [f32; MOTOR_COUNT],
) -> Result<CanFrame, can_transport::CanIoError> {
    let mut data = [0u8; TFF_FRAME_BYTES];
    for index in 0..MOTOR_COUNT {
        let offset = index * TFF_LANE_BYTES;
        data[offset..offset + TFF_LANE_BYTES].copy_from_slice(&tff_commands[index].to_le_bytes());
    }
    CanFrame::new_fd(cob_id, &data, true)
}

async fn send_control_frames(
    bus: &Arc<dyn CanBus>,
    position_velocity_cob_id: u16,
    tff_cob_id: u16,
    positions: [f32; MOTOR_COUNT],
    velocities: [f32; MOTOR_COUNT],
    tff_commands: [f32; MOTOR_COUNT],
) -> anyhow::Result<()> {
    let tff_frame = build_tff_frame(tff_cob_id, tff_commands)
        .map_err(|error| anyhow::anyhow!("build Tff RPDO2: {error}"))?;
    let position_velocity_frame =
        build_position_velocity_frame(position_velocity_cob_id, positions, velocities)
            .map_err(|error| anyhow::anyhow!("build Pdes/Vdes RPDO1: {error}"))?;
    bus.send(tff_frame)
        .await
        .map_err(|error| anyhow::anyhow!("send Tff RPDO2: {error}"))?;
    bus.send(position_velocity_frame)
        .await
        .map_err(|error| anyhow::anyhow!("send Pdes/Vdes RPDO1: {error}"))?;
    Ok(())
}

async fn send_hold_frames(
    bus: &Arc<dyn CanBus>,
    position_velocity_cob_id: u16,
    tff_cob_id: u16,
    positions: [f32; MOTOR_COUNT],
    tff_commands: [f32; MOTOR_COUNT],
) {
    for _ in 0..3 {
        if let Err(error) = send_control_frames(
            bus,
            position_velocity_cob_id,
            tff_cob_id,
            positions,
            [0.0; MOTOR_COUNT],
            tff_commands,
        )
        .await
        {
            log::warn!("six-motor MIT safe hold frame failed: {error}");
            break;
        }
        tokio::time::sleep(CONTROL_PERIOD).await;
    }
}

async fn disable_all(manager: &MeowMotorManager) -> Vec<String> {
    let mut failures = Vec::new();
    for &node_id in &NODE_IDS {
        if let Err(error) = disable_and_confirm(manager, node_id).await {
            failures.push(format!("ID {node_id}: {error}"));
        }
    }
    failures
}

/// During an early startup failure, nodes not yet initialized cannot accept a
/// manager-level disable and have never reached electrical enable. Avoid
/// reporting those expected `NotReady` results as unsafe rollback failures,
/// while still trying any initialized or observed-enabled node.
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
            if let Err(error) = disable_and_confirm(manager, node_id).await {
                failures.push(format!("ID {node_id}: {error}"));
            }
        }
    }
    failures
}

async fn disable_and_confirm(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<()> {
    // MeowMotorManager::disable uses the same confirmation path as Motor
    // Control: it fences on a newer TPDO2 (or SDO mode-display while not yet
    // initialized) and returns only after Disabled has been observed.
    manager
        .disable(node_id)
        .await
        .map_err(|error| anyhow::anyhow!("disable confirmation failed: {error}"))
}

/// One deterministic 500 Hz trapezoidal position step. The stopping-distance
/// speed cap makes the command decelerate before the target instead of changing
/// Pdes for all axes in one discontinuous jump.
fn step_trajectory(
    position: f32,
    velocity: f32,
    target: f32,
    max_velocity: f32,
    acceleration: f32,
    deceleration: f32,
    dt: f32,
) -> (f32, f32) {
    let error = target - position;
    if error.abs() <= f32::EPSILON && velocity.abs() <= f32::EPSILON {
        return (target, 0.0);
    }
    let direction = if error >= 0.0 { 1.0 } else { -1.0 };
    let stopping_speed = (2.0 * deceleration * error.abs()).sqrt();
    let desired_velocity = direction * max_velocity.min(stopping_speed);
    let changing_direction = velocity != 0.0 && velocity.signum() != desired_velocity.signum();
    let slowing = changing_direction || desired_velocity.abs() < velocity.abs();
    let max_delta = if slowing { deceleration } else { acceleration } * dt;
    let next_velocity = approach(velocity, desired_velocity, max_delta);
    let next_position = position + next_velocity * dt;
    let crossed = (target - position).signum() != (target - next_position).signum();
    if crossed || (target - next_position).abs() <= 1.0e-7 {
        (target, 0.0)
    } else {
        (next_position, next_velocity)
    }
}

fn approach(current: f32, target: f32, max_delta: f32) -> f32 {
    if (target - current).abs() <= max_delta {
        target
    } else {
        current + (target - current).signum() * max_delta
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_request() -> SixMotorMitRequest {
        let park_positions = [0.0, 0.25, 0.25, -0.01, 0.0, 0.0];
        let park_angles = [0.0, -1.570, 3.140, 0.0, 0.0, 0.0];
        let directions = [-1, -1, 1, 1, 1, 1];
        let tuning = std::array::from_fn(|index| SixMotorMitTuning {
            tff_nm: 0.0,
            gravity_direction: directions[index],
            gravity_park_position_rev: park_positions[index],
            gravity_park_angle_rad: park_angles[index],
            gravity_scale: 1.0,
            gravity_max_torque_nm: 20.0,
            kp_nm_per_rad: 12.0,
            kd_nm_s_per_rad: 2.0,
            kp_kd_limit_permille: 200,
            max_torque_permille: 1_000,
        });
        SixMotorMitRequest {
            waypoints: vec![park_positions; WAYPOINT_COUNT],
            gravity_enabled: true,
            capture_gravity_park_on_start: false,
            gravity_vector_m_per_s2: [0.0, 0.0, -9.81],
            tff_slew_rate_nm_per_s: 5.0,
            payload_mass_kg: 0.0,
            payload_center_of_mass_m: [0.0; 3],
            max_velocity_rev_per_s: 0.2,
            acceleration_rev_per_s2: 0.2,
            deceleration_rev_per_s2: 0.2,
            arrival_position_tolerance_rev: 0.003,
            arrival_velocity_tolerance_rev_per_s: 0.02,
            settle_time_ms: 200,
            movement_timeout_ms: 15_000,
            waypoint_dwell_ms: 500,
            tuning,
        }
    }

    #[test]
    fn six_motor_frame_is_uncompressed_pdes_then_vdes_per_lane() {
        let positions = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let velocities = [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0];
        let frame = build_position_velocity_frame(0x18A, positions, velocities).unwrap();
        assert_eq!(frame.id().raw(), 0x18A);
        assert!(frame.is_fd());
        assert!(frame.brs());
        assert_eq!(frame.data().len(), 48);
        for index in 0..MOTOR_COUNT {
            let offset = index * POSITION_VELOCITY_LANE_BYTES;
            assert_eq!(
                f32::from_le_bytes(frame.data()[offset..offset + 4].try_into().unwrap()),
                positions[index]
            );
            assert_eq!(
                f32::from_le_bytes(frame.data()[offset + 4..offset + 8].try_into().unwrap()),
                velocities[index]
            );
        }
    }

    #[test]
    fn tff_frame_contains_one_real32_per_motor() {
        let commands = [-3.0, -2.0, -1.0, 1.0, 2.0, 3.0];
        let frame = build_tff_frame(0x28A, commands).unwrap();
        assert_eq!(frame.id().raw(), 0x28A);
        assert_eq!(frame.data().len(), TFF_FRAME_BYTES);
        for (index, command) in commands.into_iter().enumerate() {
            let offset = index * TFF_LANE_BYTES;
            assert_eq!(
                f32::from_le_bytes(
                    frame.data()[offset..offset + TFF_LANE_BYTES]
                        .try_into()
                        .unwrap()
                ),
                command
            );
        }
    }

    #[test]
    fn each_rpdo_maps_only_its_own_uncompressed_mit_lanes() {
        for motor_index in 0..MOTOR_COUNT {
            let entries = position_velocity_rpdo_entries_for_motor(motor_index);
            assert_eq!(entries.len(), MOTOR_COUNT * 2);
            for lane in 0..MOTOR_COUNT {
                let pair = &entries[lane * 2..lane * 2 + 2];
                if lane == motor_index {
                    assert_eq!(pair[0].index, OD_MIT);
                    assert_eq!(pair[0].subindex, MIT_SUB_PDES);
                    assert_eq!(pair[1].index, OD_MIT);
                    assert_eq!(pair[1].subindex, MIT_SUB_VDES);
                } else {
                    assert_eq!(pair, [PAD_ENTRY, PAD_ENTRY]);
                }
            }
            let tff_entries = tff_rpdo_entries_for_motor(motor_index);
            assert_eq!(tff_entries.len(), MOTOR_COUNT);
            for (lane, entry) in tff_entries.into_iter().enumerate() {
                if lane == motor_index {
                    assert_eq!(entry.index, OD_MIT);
                    assert_eq!(entry.subindex, MIT_SUB_TFF);
                    assert_eq!(entry.bit_len, 32);
                } else {
                    assert_eq!(entry, PAD_ENTRY);
                }
            }
        }
    }

    #[test]
    fn coupled_urdf_gravity_uses_park_mapping_and_motor_direction() {
        let request = test_request();
        let dynamics =
            configured_dynamics(&ArmDynamics::from_urdf_string(ARM_URDF).unwrap(), &request);
        let positions = request.waypoints[0];
        let gravity = gravity_compensation(&dynamics, &request, &positions).unwrap();
        for index in 0..MOTOR_COUNT {
            assert!(
                (gravity.joint_position_rad[index] - request.tuning[index].gravity_park_angle_rad)
                    .abs()
                    < 1.0e-6
            );
        }
        assert!((gravity.model_torque_nm[1] - 2.3795).abs() < 0.02);
        assert!((gravity.model_torque_nm[2] + 4.3564).abs() < 0.02);
        assert!((gravity.model_torque_nm[3] + 0.5625).abs() < 0.02);
        assert!((gravity.target_motor_torque_nm[1] + 2.3795).abs() < 0.02);

        let mut moved = positions;
        moved[1] += 0.25;
        let moved_gravity = gravity_compensation(&dynamics, &request, &moved).unwrap();
        assert!(
            (moved_gravity.joint_position_rad[1]
                - (request.tuning[1].gravity_park_angle_rad - TAU_F32 * 0.25))
                .abs()
                < 1.0e-5
        );
        assert!(
            (moved_gravity.model_torque_nm[2] - gravity.model_torque_nm[2]).abs() > 0.1,
            "joint 3 gravity must respond to upstream joint 2"
        );
    }

    #[test]
    fn tff_slew_and_raw_ceiling_are_independent_safety_layers() {
        let request = test_request();
        let mut constant = 0.0;
        let mut gravity = 0.0;
        slew_tff_components(
            &mut constant,
            &mut gravity,
            2.0,
            -1.0,
            20.0,
            request.tff_slew_rate_nm_per_s * 0.002,
        );
        assert!(constant.abs() + gravity.abs() <= 0.010_001);

        // Reducing the hard gravity clamp must not turn cancellation into a
        // one-frame total torque step; excess is blended out as transition
        // torque under the same total slew budget.
        constant = 10.0;
        gravity = -10.0;
        let preceding_total = constant + gravity;
        slew_tff_components(&mut constant, &mut gravity, 0.0, -1.0, 1.0, 0.01);
        assert!(gravity.abs() <= 1.0);
        assert!((constant + gravity - preceding_total).abs() <= 0.010_001);

        let mut tuning = request.tuning[1];
        tuning.max_torque_permille = 200;
        let wire = wire_tff_state(tuning, 3.0, 1.5, 10.0);
        assert_eq!(wire.limit_nm, 2.0);
        assert_eq!(wire.command_nm, 2.0);
        assert_eq!(wire.utilization_percent, 100.0);
        assert!(wire.limited);
    }

    #[test]
    fn gravity_only_runtime_changes_do_not_require_sdo_writes() {
        let request = test_request();
        let tuning = request.tuning[1];
        let mut gravity_change = tuning;
        gravity_change.gravity_scale = 0.75;
        gravity_change.gravity_park_position_rev += 0.01;
        assert!(!sdo_tuning_changed(tuning, gravity_change));

        let mut drive_change = tuning;
        drive_change.max_torque_permille -= 1;
        assert!(sdo_tuning_changed(tuning, drive_change));
    }

    #[test]
    fn preflight_recommends_headroom_and_rejects_a_smaller_drive_limit() {
        let mut request = test_request();
        let dynamics =
            configured_dynamics(&ArmDynamics::from_urdf_string(ARM_URDF).unwrap(), &request);
        let peaks = [31.6; MOTOR_COUNT];
        let factors = [1.0; MOTOR_COUNT];
        let positions = request.waypoints[0];
        let recommended =
            preflight_request(&request, &dynamics, &positions, &factors, &peaks).unwrap();
        assert!(recommended[1] > 0);
        assert!(recommended[2] > 0);
        request.tuning[1].max_torque_permille = recommended[1].saturating_sub(1);
        assert!(
            preflight_request(&request, &dynamics, &positions, &factors, &peaks)
                .unwrap_err()
                .to_string()
                .contains("motor 2 maximum torque")
        );
    }

    #[test]
    fn recommended_gui_defaults_cover_the_serialized_test_path() {
        let mut request = test_request();
        request.waypoints = vec![
            [0.0, 0.25, 0.25, -0.01, 0.0, 0.0],
            [0.0, 0.18, 0.18, -0.07, 0.01, 0.0],
            [-0.14, 0.11, 0.08, 0.06, 0.03, 0.0],
            [0.12, 0.1, 0.2, -0.14, 0.1, 0.0],
        ];
        let scales = [0.0, 1.0, 1.0, 1.0, 0.0, 0.0];
        let gravity_limits = [0.2, 12.0, 5.0, 1.0, 0.2, 0.1];
        let drive_limits = [200, 650, 400, 250, 200, 200];
        for index in 0..MOTOR_COUNT {
            request.tuning[index].gravity_scale = scales[index];
            request.tuning[index].gravity_max_torque_nm = gravity_limits[index];
            request.tuning[index].max_torque_permille = drive_limits[index];
        }
        let dynamics =
            configured_dynamics(&ArmDynamics::from_urdf_string(ARM_URDF).unwrap(), &request);
        let recommendations = preflight_request(
            &request,
            &dynamics,
            &request.waypoints[0],
            &[1.0; MOTOR_COUNT],
            &[31.6; MOTOR_COUNT],
        )
        .unwrap();
        assert_eq!(recommendations, [200, 328, 367, 224, 200, 200]);
        for index in 0..MOTOR_COUNT {
            assert!(recommendations[index] <= drive_limits[index]);
        }
    }

    #[test]
    fn trajectory_accelerates_and_lands_without_overshoot() {
        let mut position = 0.0;
        let mut velocity = 0.0;
        let target = 0.25;
        for _ in 0..20_000 {
            (position, velocity) =
                step_trajectory(position, velocity, target, 0.2, 0.2, 0.2, 0.002);
            assert!(position <= target + f32::EPSILON);
            if position == target && velocity == 0.0 {
                break;
            }
        }
        assert_eq!(position, target);
        assert_eq!(velocity, 0.0);
    }

    #[test]
    fn gain_conversion_uses_read_only_motor_factor_and_full_u16_range() {
        let factor = 0.01;
        let raw = gain_to_raw(12.0, factor).unwrap();
        assert_eq!(raw, (12.0 * TAU_F64 / factor as f64).round() as u16);
        assert!(gain_to_raw(10_000.0, factor).is_err());
    }

    #[test]
    fn request_requires_all_four_complete_waypoints_and_safe_limits() {
        let request = test_request();
        assert!(request.validate().is_ok());
        let mut bad = request.clone();
        bad.waypoints[1][4] = 128.0;
        assert!(bad.validate().unwrap_err().contains("motor 5"));
    }
}
