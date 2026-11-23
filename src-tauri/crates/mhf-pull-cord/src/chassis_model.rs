//! Single steering wheel: positive drive toward the driven end, configurable steering polarity.
//! The actual cord exits toward the driven axle (+X).
//! The simulator's -45° scene rotation is presentation only.
use serde::{Deserialize, Deserializer, Serialize};

mod pace;
use pace::PaceObserver;
pub use pace::{MeasuredMotion, PaceFollowParameters};

/// Maximum motor output-shaft speed: 200 RPM, expressed in rad/s.
pub const MOTOR_MAX_OUTPUT_RAD_S: f64 = 200.0 * std::f64::consts::TAU / 60.0;
/// The module is installed 180 degrees from the existing calibrated motor zero.
pub const STEERING_CENTER_DEG: f64 = 180.0;
pub const STEERING_MIN_SPEED_RAD_S: f64 = 0.01;
pub const STEERING_MAX_SPEED_RAD_S: f64 = MOTOR_MAX_OUTPUT_RAD_S;
pub const STEERING_MAX_FULL_SPEED_RAD_S: f64 = MOTOR_MAX_OUTPUT_RAD_S;
/// DM-J4310 output-shaft limits; Nm values are converted using each motor's calibration.
pub const CHASSIS_MIN_TORQUE_NM: f64 = 0.05;
pub const CHASSIS_MAX_TORQUE_NM: f64 = 7.0;
pub const MIN_OVERCURRENT_RATIO: f64 = 0.01;
pub const MAX_OVERCURRENT_RATIO: f64 = 0.98;
pub const MAX_IMAX_OVERRIDE_A: f64 = 100.0;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum WalkMode {
    Behind,
    Ahead,
}

/// Cord speed is positive for payout and negative for retraction.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(default, rename_all = "camelCase")]
pub struct BehindSpeedParameters {
    pub enabled: bool,
    pub payout_speed_ratio: f64,
    pub neutral_boost_ratio: f64,
    pub retract_gain: f64,
    pub speed_deadband_mps: f64,
    pub payout_blend_mps: f64,
    pub filter_ms: f64,
    pub payout_acceleration_mps2: f64,
    pub response_acceleration_mps2: f64,
    /// Ordinary Behind braking, independent of the launch acceleration.
    pub deceleration_mps2: f64,
    /// Fixed braking rate after reaching the long-cord stop line.
    pub stop_deceleration_mps2: f64,
    /// Braking before Ahead takeover; never weaker than the stop-line rate.
    pub switch_deceleration_mps2: f64,
    /// Adapt ordinary Behind ramps to cord motion and commanded speed error.
    pub adaptive_enabled: bool,
    /// Additional m/s² per m/s of cord motion or longitudinal speed error.
    pub adaptive_gain: f64,
    pub adaptive_acceleration_mps2: f64,
    pub adaptive_deceleration_mps2: f64,
    /// A confirmed fast payout latches a stop in legacy cruise, not pace following.
    pub payout_stop_enabled: bool,
    pub payout_stop_speed_mps: f64,
    pub payout_stop_confirm_ms: f64,
    pub payout_stop_min_payout_m: f64,
    pub payout_stop_deceleration_mps2: f64,
    /// Dedicated ceiling for forward resumption after stopped retraction.
    pub pull_back_acceleration_mps2: f64,
    /// Additional acceleration per m/s of normalized retraction speed.
    pub pull_back_acceleration_gain: f64,
    /// Require real net shortening as well as the 80 ms speed confirmation.
    pub pull_back_min_retract_m: f64,
    /// Speed ceiling per metre retracted during this uninterrupted gesture.
    pub pull_back_length_gain: f64,
    /// Dedicated braking rate when the restart gesture ends or slows down.
    pub pull_back_deceleration_mps2: f64,
}

impl Default for BehindSpeedParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            payout_speed_ratio: 0.6,
            neutral_boost_ratio: 0.15,
            retract_gain: 0.5,
            speed_deadband_mps: 0.02,
            payout_blend_mps: 0.1,
            filter_ms: 120.0,
            payout_acceleration_mps2: 0.1,
            response_acceleration_mps2: 0.3,
            deceleration_mps2: 0.3,
            stop_deceleration_mps2: 1.5,
            switch_deceleration_mps2: 3.0,
            adaptive_enabled: true,
            adaptive_gain: 2.0,
            adaptive_acceleration_mps2: 1.2,
            adaptive_deceleration_mps2: 1.2,
            payout_stop_enabled: true,
            payout_stop_speed_mps: 0.5,
            payout_stop_confirm_ms: 40.0,
            payout_stop_min_payout_m: 0.02,
            payout_stop_deceleration_mps2: 1.2,
            pull_back_acceleration_mps2: 1.2,
            pull_back_acceleration_gain: 2.0,
            pull_back_min_retract_m: 0.01,
            pull_back_length_gain: 2.0,
            pull_back_deceleration_mps2: 0.8,
        }
    }
}

impl BehindSpeedParameters {
    fn payout_weight(self, cord_speed_mps: f64) -> f64 {
        smoothstep((cord_speed_mps - self.speed_deadband_mps) / self.payout_blend_mps)
    }

    fn speed(self, cord_speed_mps: f64, max_speed_mps: f64) -> f64 {
        if !self.enabled {
            return max_speed_mps;
        }
        let retract = (-cord_speed_mps - self.speed_deadband_mps).max(0.0);
        (max_speed_mps
            * (self.payout_speed_ratio
                + self.neutral_boost_ratio * (1.0 - self.payout_weight(cord_speed_mps)))
            + self.retract_gain * retract)
            .clamp(0.0, max_speed_mps)
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct FollowParameters {
    /// Ahead-mode payout threshold; the behind mode has its own four points.
    pub start_cord_length_m: f64,
    /// Initial Behind launch requires this payout; pace keeps a near-origin guard after launch.
    pub behind_start_cord_length_m: f64,
    /// Behind mode keeps the full speed envelope through this desired payout.
    pub behind_target_cord_length_m: f64,
    /// Behind-follow speed reaches zero at this payout.
    pub behind_stop_cord_length_m: f64,
    /// At or above this payout, Behind (including running pace) brakes into Ahead motion.
    pub behind_ahead_switch_cord_length_m: f64,
    pub behind_speed: BehindSpeedParameters,
    pub pace: PaceFollowParameters,
    pub length_gain_per_s: f64,
    pub cord_speed_feedforward: f64,
    pub max_speed_mps: f64,
    pub allow_reverse: bool,
    pub max_reverse_speed_mps: f64,
    pub encoder_center_deg: f64,
    pub direction_deadband_deg: f64,
    pub smoothing_ms: f64,
}

fn default_behind_start_cord_length_m() -> f64 {
    0.75
}

fn default_behind_target_cord_length_m() -> f64 {
    1.2
}

fn default_behind_stop_cord_length_m() -> f64 {
    1.8
}

fn default_behind_ahead_switch_cord_length_m() -> f64 {
    2.2
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct FollowParametersWire {
    start_cord_length_m: f64,
    #[serde(default)]
    behind_start_cord_length_m: Option<f64>,
    #[serde(default)]
    behind_target_cord_length_m: Option<f64>,
    #[serde(default)]
    behind_stop_cord_length_m: Option<f64>,
    #[serde(default)]
    behind_ahead_switch_cord_length_m: Option<f64>,
    #[serde(default)]
    behind_speed: BehindSpeedParameters,
    #[serde(default)]
    pace: PaceFollowParameters,
    length_gain_per_s: f64,
    cord_speed_feedforward: f64,
    max_speed_mps: f64,
    allow_reverse: bool,
    max_reverse_speed_mps: f64,
    encoder_center_deg: f64,
    direction_deadband_deg: f64,
    smoothing_ms: f64,
}

impl<'de> Deserialize<'de> for FollowParameters {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = FollowParametersWire::deserialize(deserializer)?;
        let previous_leash_defaults = wire.behind_start_cord_length_m == Some(0.75)
            && wire.behind_target_cord_length_m == Some(1.2)
            && wire.behind_stop_cord_length_m == Some(2.3)
            && wire.behind_ahead_switch_cord_length_m.is_none();
        let behind_start_cord_length_m = wire
            .behind_start_cord_length_m
            .unwrap_or_else(default_behind_start_cord_length_m);
        let behind_target_cord_length_m = wire
            .behind_target_cord_length_m
            .unwrap_or_else(default_behind_target_cord_length_m);
        let mut behind_stop_cord_length_m = wire
            .behind_stop_cord_length_m
            .unwrap_or_else(default_behind_stop_cord_length_m);
        let behind_ahead_switch_cord_length_m =
            if let Some(value) = wire.behind_ahead_switch_cord_length_m {
                value
            } else if previous_leash_defaults {
                // Migrate the immediately preceding 0.75/1.2/2.3 leash
                // configuration to the new stop band without breaking saved
                // controller settings at startup.
                behind_stop_cord_length_m = default_behind_stop_cord_length_m();
                default_behind_ahead_switch_cord_length_m()
            } else if behind_stop_cord_length_m >= default_behind_ahead_switch_cord_length_m() {
                // Preserve a customised old stop and place the newly added
                // switch 0.4 m beyond it.
                behind_stop_cord_length_m + 0.4
            } else {
                default_behind_ahead_switch_cord_length_m()
            };
        Ok(Self {
            start_cord_length_m: wire.start_cord_length_m,
            behind_start_cord_length_m,
            behind_target_cord_length_m,
            behind_stop_cord_length_m,
            behind_ahead_switch_cord_length_m,
            behind_speed: wire.behind_speed,
            pace: wire.pace,
            length_gain_per_s: wire.length_gain_per_s,
            cord_speed_feedforward: wire.cord_speed_feedforward,
            max_speed_mps: wire.max_speed_mps,
            allow_reverse: wire.allow_reverse,
            max_reverse_speed_mps: wire.max_reverse_speed_mps,
            encoder_center_deg: wire.encoder_center_deg,
            direction_deadband_deg: wire.direction_deadband_deg,
            smoothing_ms: wire.smoothing_ms,
        })
    }
}

impl Default for FollowParameters {
    fn default() -> Self {
        Self {
            start_cord_length_m: 1.0,
            behind_start_cord_length_m: default_behind_start_cord_length_m(),
            behind_target_cord_length_m: default_behind_target_cord_length_m(),
            behind_stop_cord_length_m: default_behind_stop_cord_length_m(),
            behind_ahead_switch_cord_length_m: default_behind_ahead_switch_cord_length_m(),
            behind_speed: BehindSpeedParameters::default(),
            pace: PaceFollowParameters::default(),
            length_gain_per_s: 0.9,
            cord_speed_feedforward: 0.45,
            max_speed_mps: 0.2,
            allow_reverse: false,
            max_reverse_speed_mps: 0.1,
            encoder_center_deg: 0.0,
            direction_deadband_deg: 0.0,
            smoothing_ms: 180.0,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FollowInput {
    /// Positive shaft rotation, converted to m/s at the drum. Unlike cord
    /// speed, this does not change sign with the configured winding polarity.
    /// Kept for telemetry compatibility; resumption uses cord_speed_mps.
    #[serde(default)]
    pub pull_back_speed_mps: Option<f64>,
    pub cord_length_m: Option<f64>,
    pub cord_speed_mps: Option<f64>,
    pub encoder_angle_deg: Option<f64>,
    /// Monotonic encoder acquisition time, not the wheel loop or UI time.
    /// Legacy/simulation callers may omit it and supply fresh samples each step.
    #[serde(default)]
    pub encoder_sample_time_s: Option<f64>,
}
impl FollowInput {
    pub fn valid(self) -> bool {
        [
            self.cord_length_m,
            self.cord_speed_mps,
            self.encoder_angle_deg,
        ]
        .iter()
        .all(|x| x.is_some_and(f64::is_finite))
            && self.pull_back_speed_mps.is_none_or(f64::is_finite)
            && self
                .encoder_sample_time_s
                .is_none_or(|t| t.is_finite() && t >= 0.0)
    }

    pub fn pull_back_speed(self) -> f64 {
        self.pull_back_speed_mps
            .unwrap_or(-self.cord_speed_mps.unwrap_or(0.0))
    }

    /// Positive while the cord returns, regardless of shaft winding polarity.
    pub fn retraction_speed(self) -> f64 {
        -self.cord_speed_mps.unwrap_or(0.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputSource {
    Live,
    Manual,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CordExitSide {
    PassiveAxle,
    #[default]
    DrivenAxle,
}
impl CordExitSide {
    /// Outlet +X points forwards at the driven (front) axle and aft at the passive axle.
    pub fn sign(self) -> f64 {
        match self {
            Self::PassiveAxle => -1.0,
            Self::DrivenAxle => 1.0,
        }
    }
}

/// Optional RAM-only tuning for the steering motor's internal mode-4 loops.
/// Speeds and accelerations use the geared output shaft; DEC remains negative.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SteeringDriverConfig {
    pub max_speed_rad_s: f64,
    pub acceleration_rad_s2: f64,
    pub deceleration_rad_s2: f64,
    pub position_kp: f64,
    pub position_ki: f64,
    pub velocity_kp: f64,
    pub velocity_ki: f64,
}
impl SteeringDriverConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !self.max_speed_rad_s.is_finite()
            || !(STEERING_MIN_SPEED_RAD_S..=MOTOR_MAX_OUTPUT_RAD_S).contains(&self.max_speed_rad_s)
        {
            return Err(format!(
                "steering driver max speed must be {STEERING_MIN_SPEED_RAD_S}..{MOTOR_MAX_OUTPUT_RAD_S} rad/s"
            ));
        }
        if !self.acceleration_rad_s2.is_finite() || self.acceleration_rad_s2 <= 0.0 {
            return Err("steering driver acceleration must be finite and positive".into());
        }
        if !self.deceleration_rad_s2.is_finite() || self.deceleration_rad_s2 >= 0.0 {
            return Err("steering driver deceleration must be finite and negative".into());
        }
        for (name, value) in [
            ("position_kp", self.position_kp),
            ("position_ki", self.position_ki),
            ("velocity_kp", self.velocity_kp),
            ("velocity_ki", self.velocity_ki),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!(
                    "steering driver {name} must be finite and nonnegative"
                ));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct ChassisConfig {
    pub follow: FollowParameters,
    pub mode: WalkMode,
    pub source: InputSource,
    #[serde(default)]
    pub cord_exit_side: CordExitSide,
    pub drive_node_id: u8,
    pub steering_node_id: u8,
    pub drive_inverted: bool,
    /// Reverse encoder deflections around the 180-degree installation center.
    pub steering_inverted: bool,
    pub wheel_diameter_m: f64,
    pub wheelbase_m: f64,
    /// External output-shaft revolutions per wheel revolution, excluding motor gearing.
    pub gear_ratio: f64,
    pub max_wheel_speed_mps: f64,
    pub acceleration_mps2: f64,
    /// Steering speed ceiling at zero travel speed, in output-shaft rad/s.
    pub steering_speed_at_rest_rad_s: f64,
    /// Steering speed ceiling at the effective maximum travel speed, in rad/s.
    pub steering_speed_at_full_speed_rad_s: f64,
    /// None leaves existing internal driver tuning unchanged at startup.
    pub steering_driver: Option<SteeringDriverConfig>,
    /// Estimated output-shaft torque corresponding to the drive's current protection threshold.
    pub drive_max_torque_nm: f64,
    /// Estimated output-shaft torque ceiling sent in each force-position command.
    pub steering_max_torque_nm: f64,
    pub drive_overcurrent_ratio: f64,
    pub steering_overcurrent_ratio: f64,
    /// Optional actual driver sampling range; 0 permits ratio control if CAN Imax is unavailable.
    pub drive_imax_a: f64,
    pub steering_imax_a: f64,
}
impl Default for ChassisConfig {
    fn default() -> Self {
        Self {
            follow: FollowParameters::default(),
            mode: WalkMode::Behind,
            source: InputSource::Live,
            cord_exit_side: CordExitSide::DrivenAxle,
            drive_node_id: 0x01,
            steering_node_id: 0x02,
            drive_inverted: false,
            steering_inverted: false,
            wheel_diameter_m: 0.120,
            wheelbase_m: 0.49917,
            gear_ratio: 1.0,
            max_wheel_speed_mps: 0.2,
            acceleration_mps2: 0.3,
            steering_speed_at_rest_rad_s: 60.0_f64.to_radians(),
            steering_speed_at_full_speed_rad_s: 60.0_f64.to_radians(),
            steering_driver: None,
            drive_max_torque_nm: 3.0,
            steering_max_torque_nm: 3.0,
            drive_overcurrent_ratio: 0.8,
            steering_overcurrent_ratio: 0.8,
            drive_imax_a: 0.0,
            steering_imax_a: 0.0,
        }
    }
}
impl ChassisConfig {
    pub fn validate(&self) -> Result<(), String> {
        if let Some(driver) = &self.steering_driver {
            driver.validate()?;
        }
        // Validate geometry before converting the common output-shaft limit
        // into the linear speeds used by the follow controller.
        for (name, value, min, max) in [
            ("wheel diameter", self.wheel_diameter_m, 0.02, 1.0),
            ("gear ratio", self.gear_ratio, 0.1, 100.0),
        ] {
            if !value.is_finite() || !(min..=max).contains(&value) {
                return Err(format!("{name} must be {min}..{max}"));
            }
        }
        self.follow.pace.validate()?;
        let p = &self.follow;
        let output_speed_limit_mps = self.max_output_speed_mps();
        for (name, value, min, max) in [
            ("start length", p.start_cord_length_m, 0.0, 40.0),
            (
                "behind start length",
                p.behind_start_cord_length_m,
                0.0,
                80.0,
            ),
            (
                "behind target length",
                p.behind_target_cord_length_m,
                0.0,
                80.0,
            ),
            ("behind stop length", p.behind_stop_cord_length_m, 0.0, 80.0),
            (
                "behind-to-ahead switch length",
                p.behind_ahead_switch_cord_length_m,
                0.0,
                80.0,
            ),
            ("length gain", p.length_gain_per_s, 0.0, 20.0),
            ("feedforward", p.cord_speed_feedforward, 0.0, 10.0),
            (
                "behind payout speed ratio",
                p.behind_speed.payout_speed_ratio,
                0.0,
                1.0,
            ),
            (
                "behind neutral boost ratio",
                p.behind_speed.neutral_boost_ratio,
                0.0,
                1.0,
            ),
            (
                "behind retract gain",
                p.behind_speed.retract_gain,
                0.0,
                10.0,
            ),
            (
                "behind speed deadband",
                p.behind_speed.speed_deadband_mps,
                0.0,
                3.0,
            ),
            (
                "behind payout blend",
                p.behind_speed.payout_blend_mps,
                0.001,
                3.0,
            ),
            ("behind speed filter", p.behind_speed.filter_ms, 0.0, 5000.0),
            (
                "payout stop speed",
                p.behind_speed.payout_stop_speed_mps,
                0.01,
                3.0,
            ),
            (
                "payout stop confirmation",
                p.behind_speed.payout_stop_confirm_ms,
                10.0,
                1000.0,
            ),
            (
                "payout stop minimum stroke",
                p.behind_speed.payout_stop_min_payout_m,
                0.001,
                0.5,
            ),
            (
                "payout stop deceleration",
                p.behind_speed.payout_stop_deceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "behind payout acceleration",
                p.behind_speed.payout_acceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "behind response acceleration",
                p.behind_speed.response_acceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "behind deceleration",
                p.behind_speed.deceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "behind stop deceleration",
                p.behind_speed.stop_deceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "behind-to-ahead switch deceleration",
                p.behind_speed.switch_deceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "behind adaptive gain",
                p.behind_speed.adaptive_gain,
                0.0,
                10.0,
            ),
            (
                "behind adaptive acceleration",
                p.behind_speed.adaptive_acceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "behind adaptive deceleration",
                p.behind_speed.adaptive_deceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "maximum speed",
                p.max_speed_mps,
                0.0,
                output_speed_limit_mps,
            ),
            (
                "reverse speed",
                p.max_reverse_speed_mps,
                0.0,
                output_speed_limit_mps,
            ),
            ("encoder centre", p.encoder_center_deg, 0.0, 360.0),
            ("angle deadband", p.direction_deadband_deg, 0.0, 180.0),
            ("display smoothing", p.smoothing_ms, 0.0, 5000.0),
            ("wheelbase", self.wheelbase_m, 0.05, 3.0),
            (
                "wheel speed limit",
                self.max_wheel_speed_mps,
                0.0,
                output_speed_limit_mps,
            ),
            ("acceleration", self.acceleration_mps2, 0.01, 4.0),
            (
                "pull-back acceleration",
                p.behind_speed.pull_back_acceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "pull-back acceleration gain",
                p.behind_speed.pull_back_acceleration_gain,
                0.0,
                10.0,
            ),
            (
                "restart minimum retraction",
                p.behind_speed.pull_back_min_retract_m,
                0.001,
                0.5,
            ),
            (
                "restart length gain",
                p.behind_speed.pull_back_length_gain,
                0.0,
                10.0,
            ),
            (
                "restart deceleration",
                p.behind_speed.pull_back_deceleration_mps2,
                0.01,
                4.0,
            ),
            (
                "steering speed at rest",
                self.steering_speed_at_rest_rad_s,
                STEERING_MIN_SPEED_RAD_S,
                STEERING_MAX_SPEED_RAD_S,
            ),
            (
                "steering speed at full speed",
                self.steering_speed_at_full_speed_rad_s,
                STEERING_MIN_SPEED_RAD_S,
                STEERING_MAX_FULL_SPEED_RAD_S,
            ),
            (
                "drive torque protection limit",
                self.drive_max_torque_nm,
                CHASSIS_MIN_TORQUE_NM,
                CHASSIS_MAX_TORQUE_NM,
            ),
            (
                "steering torque limit",
                self.steering_max_torque_nm,
                CHASSIS_MIN_TORQUE_NM,
                CHASSIS_MAX_TORQUE_NM,
            ),
            (
                "drive overcurrent ratio",
                self.drive_overcurrent_ratio,
                MIN_OVERCURRENT_RATIO,
                MAX_OVERCURRENT_RATIO,
            ),
            (
                "steering overcurrent ratio",
                self.steering_overcurrent_ratio,
                MIN_OVERCURRENT_RATIO,
                MAX_OVERCURRENT_RATIO,
            ),
            (
                "drive Imax override",
                self.drive_imax_a,
                0.0,
                MAX_IMAX_OVERRIDE_A,
            ),
            (
                "steering Imax override",
                self.steering_imax_a,
                0.0,
                MAX_IMAX_OVERRIDE_A,
            ),
        ] {
            if !value.is_finite() || !(min..=max).contains(&value) {
                return Err(format!("{name} must be {min}..{max}"));
            }
        }
        if !(p.behind_start_cord_length_m < p.behind_target_cord_length_m
            && p.behind_target_cord_length_m < p.behind_stop_cord_length_m
            && p.behind_stop_cord_length_m < p.behind_ahead_switch_cord_length_m)
        {
            return Err(
                "behind cord lengths must satisfy start < target < stop < ahead switch".into(),
            );
        }
        for nid in [self.drive_node_id, self.steering_node_id] {
            if !(1..=15).contains(&nid) {
                return Err(
                    "DAMIAO IDs must be 1..15; feedback reserves the high nibble for status".into(),
                );
            }
        }
        if self.drive_node_id == self.steering_node_id {
            return Err("drive and steering motor IDs must differ".into());
        }
        Ok(())
    }
    fn pace_selected(&self) -> bool {
        self.mode == WalkMode::Behind
            && self.follow.pace.enabled
            && self.source == InputSource::Live
    }

    pub fn motor_rps(&self, speed: f64, inverted: bool) -> f64 {
        speed / (std::f64::consts::PI * self.wheel_diameter_m)
            * self.gear_ratio
            * if inverted { -1.0 } else { 1.0 }
    }
    pub fn wheel_mps(&self, motor_rps: f64, inverted: bool) -> f64 {
        motor_rps * std::f64::consts::PI * self.wheel_diameter_m / self.gear_ratio
            * if inverted { -1.0 } else { 1.0 }
    }

    /// Linear speed equivalent of 200 RPM at the module output shaft.
    /// Internal motor gearing is already handled by the motor protocol;
    /// gear_ratio describes only the external transmission to the wheel.
    pub fn max_output_speed_mps(&self) -> f64 {
        MOTOR_MAX_OUTPUT_RAD_S * self.wheel_diameter_m / (2.0 * self.gear_ratio)
    }

    /// Full travel speed respects the follow setting, wheel ceiling and 200 RPM rating.
    pub fn steering_full_speed_mps(&self) -> f64 {
        self.follow
            .max_speed_mps
            .min(self.max_wheel_speed_mps)
            .min(self.max_output_speed_mps())
    }

    pub fn steering_speed_slope(&self) -> f64 {
        let full_speed = self.steering_full_speed_mps();
        if full_speed > 0.0 {
            (self.steering_speed_at_full_speed_rad_s - self.steering_speed_at_rest_rad_s)
                / full_speed
        } else {
            0.0
        }
    }

    /// Interpolate using measured travel speed magnitude, including reverse motion.
    pub fn steering_speed_rad_s(&self, travel_mps: f64) -> f64 {
        let full_speed = self.steering_full_speed_mps();
        if full_speed <= 0.0 || travel_mps == 0.0 {
            self.steering_speed_at_rest_rad_s
        } else if travel_mps.abs() >= full_speed {
            self.steering_speed_at_full_speed_rad_s
        } else {
            self.steering_speed_at_rest_rad_s + self.steering_speed_slope() * travel_mps.abs()
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct ChassisCommand {
    pub speed_mps: f64,
    pub encoder_error_deg: f64,
    pub linear_mps: f64,
    /// Absolute steering motor output-shaft target, including configured polarity.
    pub steering_angle_deg: f64,
}
pub fn signed_degrees(v: f64) -> f64 {
    (v + 180.0).rem_euclid(360.0) - 180.0
}
fn smoothstep(value: f64) -> f64 {
    let t = value.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[derive(Debug, Clone, Copy)]
struct LongitudinalRequest {
    speed_mps: f64,
    /// The motion direction can switch to Ahead in the far section of a
    /// selected Behind mode without mutating the configured operator mode.
    travel_mode: WalkMode,
}

fn ahead_longitudinal_request(
    input: FollowInput,
    p: &FollowParameters,
    start_cord_length_m: f64,
) -> LongitudinalRequest {
    let length_error = (input.cord_length_m.unwrap() - start_cord_length_m).max(0.0);
    let speed_mps = if length_error > 0.0 {
        (length_error * p.length_gain_per_s
            + input.cord_speed_mps.unwrap() * p.cord_speed_feedforward)
            .clamp(
                if p.allow_reverse {
                    -p.max_reverse_speed_mps.min(p.max_speed_mps)
                } else {
                    0.0
                },
                p.max_speed_mps,
            )
    } else {
        0.0
    };
    LongitudinalRequest {
        speed_mps,
        travel_mode: WalkMode::Ahead,
    }
}

fn longitudinal_request(input: FollowInput, c: &ChassisConfig) -> LongitudinalRequest {
    let p = &c.follow;
    let length = input.cord_length_m.unwrap();
    if c.mode == WalkMode::Behind {
        let range_valid = p.behind_start_cord_length_m < p.behind_target_cord_length_m
            && p.behind_target_cord_length_m < p.behind_stop_cord_length_m
            && p.behind_stop_cord_length_m < p.behind_ahead_switch_cord_length_m;
        if !range_valid {
            return LongitudinalRequest {
                speed_mps: 0.0,
                travel_mode: WalkMode::Behind,
            };
        }

        // The far section of Behind mode deliberately reuses the existing
        // Ahead law, but starts its length error at the configurable switch.
        // This leaves a stopped band between the two directions.
        if length >= p.behind_ahead_switch_cord_length_m {
            return ahead_longitudinal_request(input, p, p.behind_ahead_switch_cord_length_m);
        }

        let start_margin = length - p.behind_start_cord_length_m;
        let stop_margin = p.behind_stop_cord_length_m - length;
        if start_margin <= 0.0 || stop_margin <= 0.0 || p.max_speed_mps <= 0.0 {
            return LongitudinalRequest {
                speed_mps: 0.0,
                travel_mode: WalkMode::Behind,
            };
        }

        // Apply the length envelope AFTER the cord-speed response, including
        // its boost, so even fast retraction fades to zero at the far stop.
        let speed_scale = if length <= p.behind_target_cord_length_m {
            1.0
        } else {
            (stop_margin / (p.behind_stop_cord_length_m - p.behind_target_cord_length_m))
                .clamp(0.0, 1.0)
        };
        return LongitudinalRequest {
            speed_mps: p
                .behind_speed
                .speed(input.cord_speed_mps.unwrap(), p.max_speed_mps)
                * speed_scale,
            travel_mode: WalkMode::Behind,
        };
    }

    ahead_longitudinal_request(input, p, p.start_cord_length_m)
}

pub fn compute(input: FollowInput, c: &ChassisConfig) -> ChassisCommand {
    if !input.valid() {
        return ChassisCommand::default();
    }
    let request = longitudinal_request(input, c);
    compute_request(input, c, request)
}

/// Map encoder deflections around the rotated installation center, with polarity.
/// Position follows at rest and in either travel mode. Never reset the motor zero.
pub fn steering_angle(input: FollowInput, c: &ChassisConfig) -> f64 {
    let Some(angle) = input.encoder_angle_deg.filter(|v| v.is_finite()) else {
        return 0.0;
    };
    let error = signed_degrees(angle - c.follow.encoder_center_deg);
    let polarity = if c.steering_inverted { -1.0 } else { 1.0 };
    STEERING_CENTER_DEG + error * polarity
}

/// Pick the equivalent multi-turn target closest to the measured motor position.
/// Slip rings permit crossing zero and full turns without a mechanical stop.
pub fn nearest_steering_angle(target_deg: f64, measured_deg: f64) -> f64 {
    target_deg + ((measured_deg - target_deg) / 360.0).round() * 360.0
}

fn compute_request(
    input: FollowInput,
    c: &ChassisConfig,
    request: LongitudinalRequest,
) -> ChassisCommand {
    let p = &c.follow;
    let speed = request.speed_mps;
    let error = signed_degrees(input.encoder_angle_deg.unwrap() - p.encoder_center_deg);
    let direction = if request.travel_mode == WalkMode::Behind {
        1.0
    } else {
        -1.0
    };
    let linear = (c.cord_exit_side.sign() * direction * speed)
        .clamp(-c.max_wheel_speed_mps, c.max_wheel_speed_mps);
    ChassisCommand {
        speed_mps: speed,
        encoder_error_deg: error,
        linear_mps: linear,
        steering_angle_deg: steering_angle(input, c),
    }
}

/// Longitudinal ramp for the single drive motor. Steering uses a position target.
pub fn slew_motion(previous: f64, target: ChassisCommand, dt: f64, c: &ChassisConfig) -> f64 {
    slew_motion_with_acceleration(previous, target, dt, c, c.acceleration_mps2)
}

fn slew_motion_with_acceleration(
    previous: f64,
    target: ChassisCommand,
    dt: f64,
    c: &ChassisConfig,
    acceleration_mps2: f64,
) -> f64 {
    // Live reductions of hard limits take priority over smooth deceleration.
    let v0 = previous.clamp(-c.max_wheel_speed_mps, c.max_wheel_speed_mps);
    let dv = acceleration_mps2 * dt.clamp(0.0, 0.03);
    let v_min = (v0 - dv).max(-c.max_wheel_speed_mps);
    let v_max = (v0 + dv).min(c.max_wheel_speed_mps);
    target.linear_mps.clamp(v_min, v_max)
}

/// Runtime state belongs to one run. Length gates always use the raw sample;
/// Measured grip speed, legacy cord speed, and timestamped encoder rate are
/// filtered independently. Stateless `compute` previews the legacy pedal law.
#[derive(Default)]
pub struct FollowController {
    pace: PaceObserver,
    filtered_speed_mps: Option<f64>,
    payout_origin_m: Option<f64>,
    payout_hold_s: f64,
    payout_stop_resume_length_m: Option<f64>,
    stop_armed: bool,
    pull_back_hold_s: f64,
    retract_resume: bool,
    retract_origin_m: Option<f64>,
    retract_braking: bool,
    braking: Option<BehindBraking>,
    status: FollowStatus,
}

/// A longitudinal stop that must finish before another motion request wins.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BehindBraking {
    StopLine,
    AheadSwitch,
}

#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct FollowStatus {
    pub travel_mode: Option<WalkMode>,
    pub braking: Option<BehindBraking>,
    pub payout_stop: bool,
    pub payout_stop_resume_length_m: Option<f64>,
    pub waiting_for_pull_back: bool,
    /// Legacy field, always false. Retraction no longer switches to Ahead.
    pub pull_back_ahead: bool,
    pub retract_resume: bool,
    /// Net shortening in the current gesture, not integrated negative noise.
    pub retract_distance_m: f64,
    pub retract_braking: bool,
    pub acceleration_mps2: f64,
    pub adaptive_response: bool,
    /// Normal live Behind cruise is using measured pace feedback.
    pub pace_active: bool,
    /// A launched pace session, including its protective stop/restart phase.
    pub pace_tracking: bool,
    pub pace_reference_length_m: Option<f64>,
    pub pace_stop_length_m: Option<f64>,
    /// Signed grip ground velocity along the forward reference; person behind cart.
    pub estimated_user_speed_mps: Option<f64>,
    /// Signed drive speed minus cord speed, independent of steering angle.
    pub pace_forward_speed_mps: Option<f64>,
    /// A stopped chassis rejects apparent retraction from passive-exit rotation.
    pub pace_rotation_guard: bool,
    /// Compatibility marker for a longitudinal target; always 1 when present.
    pub pace_turn_scale: Option<f64>,
    pub pace_correction_mps: f64,
    pub pace_target_mps: Option<f64>,
    /// Active start/near-origin threshold, for a matching runtime preview.
    pub pace_min_length_m: Option<f64>,
}

impl FollowController {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn status(&self) -> FollowStatus {
        self.status
    }

    fn clear_retract_restart(&mut self) {
        self.stop_armed = false;
        self.retract_resume = false;
        self.pull_back_hold_s = 0.0;
        self.retract_origin_m = None;
        self.retract_braking = false;
    }

    fn update_payout_stop(
        &mut self,
        input: FollowInput,
        v0: f64,
        stopped: bool,
        dt: f64,
        c: &ChassisConfig,
    ) {
        let p = &c.follow;
        let tuning = p.behind_speed;
        let length = input.cord_length_m.unwrap();
        let speed = input.cord_speed_mps.unwrap();
        // In pace following, payout means the person is falling behind.
        // Handle it as a pace reduction, not a separate intentional tug.
        // Keep an already latched legacy stop until its normal release, but
        // never start a new tug gesture while pace following is selected.
        if c.mode != WalkMode::Behind
            || !tuning.payout_stop_enabled
            || p.max_speed_mps <= 0.0
            || (length >= p.behind_ahead_switch_cord_length_m && stopped && v0.abs() <= 1e-9)
        {
            self.payout_stop_resume_length_m = None;
        } else if let Some(resume_length) = self.payout_stop_resume_length_m {
            let resume_length = resume_length.min(p.behind_target_cord_length_m);
            self.payout_stop_resume_length_m = Some(resume_length);
            // Keep braking even if the hand returns before the chassis stops.
            // The gesture's origin prevents immediate restart for a tug made
            // entirely inside the full-speed length band.
            if stopped
                && v0.abs() <= 1e-9
                && length <= resume_length
                && speed <= tuning.speed_deadband_mps
            {
                self.payout_stop_resume_length_m = None;
            }
        } else if !c.pace_selected()
            && !self.stop_armed
            && !self.retract_resume
            && v0 * c.cord_exit_side.sign() > 0.01
            && length > p.behind_start_cord_length_m
            && length < p.behind_stop_cord_length_m
            && speed >= tuning.payout_stop_speed_mps
            && speed > tuning.speed_deadband_mps
        {
            let origin = *self.payout_origin_m.get_or_insert(length);
            self.payout_hold_s += dt.clamp(0.0, 0.03);
            if self.payout_hold_s >= tuning.payout_stop_confirm_ms / 1000.0
                && length - origin >= tuning.payout_stop_min_payout_m
            {
                self.payout_stop_resume_length_m = Some(origin.min(p.behind_target_cord_length_m));
            } else {
                return;
            }
        }
        // Neither isolated spikes nor separate short pulls accumulate.
        self.payout_origin_m = None;
        self.payout_hold_s = 0.0;
    }

    fn update_braking(
        &mut self,
        length: f64,
        v0: f64,
        behind_motion: bool,
        stopped: bool,
        c: &ChassisConfig,
        pace_tracking: bool,
    ) {
        if c.mode != WalkMode::Behind || (stopped && v0.abs() <= 1e-9) {
            self.braking = None;
            return;
        }
        let p = &c.follow;
        let stop_length = if pace_tracking {
            p.pace.stop_length(c)
        } else {
            p.behind_stop_cord_length_m
        };
        // The Ahead threshold remains active after pace launch, even when
        // the running stop guard is at or beyond it. Finish braking first.
        if length >= p.behind_ahead_switch_cord_length_m
            && (behind_motion
                || self.braking.is_some()
                || self.payout_stop_resume_length_m.is_some())
        {
            self.braking = Some(BehindBraking::AheadSwitch);
        } else if self.braking.is_none()
            && length >= stop_length
            && length < p.behind_ahead_switch_cord_length_m
            && !self.retract_resume
            && !self.retract_braking
        {
            self.braking = Some(BehindBraking::StopLine);
        }
        // Keep the selected rate across cord bounce and return to the cruise
        // band. Only a zero command plus fresh, near-zero feedback releases it.
    }

    /// Simulation convenience. Hardware MUST supply fresh measured velocity
    /// through step_with_feedback, never infer a physical stop from a target.
    pub fn step(
        &mut self,
        input: FollowInput,
        previous: f64,
        dt: f64,
        c: &ChassisConfig,
    ) -> (ChassisCommand, f64) {
        self.step_with_motion(
            input,
            previous,
            Some(MeasuredMotion::straight(previous)),
            dt,
            c,
        )
    }

    pub fn step_with_feedback(
        &mut self,
        input: FollowInput,
        previous: f64,
        measured_linear_mps: Option<f64>,
        dt: f64,
        c: &ChassisConfig,
    ) -> (ChassisCommand, f64) {
        // The steering module provides one drive speed; no body yaw sensor
        // is available. Never estimate yaw from a pair of wheel commands.
        self.step_with_motion(
            input,
            previous,
            measured_linear_mps.map(|linear_mps| MeasuredMotion {
                linear_mps,
                yaw_rate_rad_s: 0.0,
            }),
            dt,
            c,
        )
    }

    pub fn step_with_motion(
        &mut self,
        input: FollowInput,
        previous: f64,
        measured_motion: Option<MeasuredMotion>,
        dt: f64,
        c: &ChassisConfig,
    ) -> (ChassisCommand, f64) {
        self.step_with_planar_motion(input, previous, measured_motion, true, dt, c)
    }

    /// An omni base also confirms its lateral command and wheel feedback have
    /// settled before releasing a brake or authorizing a direction change.
    pub fn step_with_planar_motion(
        &mut self,
        mut input: FollowInput,
        previous: f64,
        measured_motion: Option<MeasuredMotion>,
        lateral_stopped: bool,
        dt: f64,
        c: &ChassisConfig,
    ) -> (ChassisCommand, f64) {
        if !input.valid() || !dt.is_finite() || dt <= 0.0 {
            self.reset();
            return (ChassisCommand::default(), 0.0);
        }
        let p = &c.follow;
        let measured_linear_mps = measured_motion.map(|v| v.linear_mps);
        let tuning = p.behind_speed;
        let length = input.cord_length_m.unwrap();
        let raw_input = input;
        let retract_speed = input.retraction_speed();
        let v0 = previous;
        let stopped = lateral_stopped
            && v0.abs() <= 0.01
            && measured_linear_mps.is_some_and(|v| v.is_finite() && v.abs() <= 0.02);
        let pace_selected = c.pace_selected();
        let pace_tracking = pace_selected && self.pace.tracking(c);
        let stop_length = if pace_tracking {
            p.pace.stop_length(c)
        } else {
            p.behind_stop_cord_length_m
        };
        let target_length = if pace_tracking {
            p.pace.target_length_m
        } else {
            p.behind_target_cord_length_m
        };
        let behind_region = length < p.behind_ahead_switch_cord_length_m;
        self.update_payout_stop(input, v0, stopped, dt, c);
        // A sideways-only Behind run must brake before handing over to Ahead.
        let behind_motion = v0 * c.cord_exit_side.sign() > 0.0
            || measured_linear_mps
                .is_none_or(|v| !v.is_finite() || v * c.cord_exit_side.sign() > 0.02)
            || (!lateral_stopped && self.status.travel_mode != Some(WalkMode::Ahead));
        self.update_braking(length, v0, behind_motion, stopped, c, pace_tracking);
        let payout_stop = self.payout_stop_resume_length_m.is_some();
        let pace_can_take_over = pace_selected && behind_region && length < stop_length;
        let retracting = retract_speed > tuning.speed_deadband_mps.max(0.02);
        if c.mode != WalkMode::Behind
            || length <= target_length
            || p.max_speed_mps <= 0.0
            || payout_stop
        {
            self.clear_retract_restart();
        } else if self.braking.is_some() || !behind_region {
            // Braking takes priority over restart confirmation. Returning to
            // the stop band requires a fresh stopped gesture.
            self.stop_armed = true;
            self.retract_resume = false;
            self.pull_back_hold_s = 0.0;
            self.retract_origin_m = None;
            self.retract_braking = false;
        } else {
            self.stop_armed |= length >= stop_length;
            if stopped && v0.abs() <= 1e-9 {
                self.retract_braking = false;
            }
            // Outside the normal pace band, release still cancels the
            // gesture. A confirmed restart inside the band transfers below,
            // even if relative speed has already reached zero on this sample.
            if self.retract_resume && !retracting && !pace_can_take_over {
                self.retract_resume = false;
                self.retract_braking = true;
                self.retract_origin_m = None;
                self.pull_back_hold_s = 0.0;
            }
            if self.stop_armed && !self.retract_resume && stopped && retracting {
                let origin = *self.retract_origin_m.get_or_insert(length);
                self.pull_back_hold_s += dt.clamp(0.0, 0.03);
                if self.pull_back_hold_s >= 0.08
                    && origin - length >= tuning.pull_back_min_retract_m
                {
                    self.retract_resume = true;
                    self.retract_braking = false;
                    self.filtered_speed_mps = None;
                }
            } else if !self.retract_resume {
                self.pull_back_hold_s = 0.0;
                self.retract_origin_m = None;
            }
        }
        // A confirmed launch has served its purpose once back inside the
        // stop line. Do not require continuous retraction down to the target:
        // matched human/cart velocity naturally produces zero cord speed.
        // Unfinished braking and unconfirmed gestures never set this flag.
        if pace_can_take_over && self.retract_resume {
            self.clear_retract_restart();
        }
        let retract_distance = self
            .retract_origin_m
            .map_or(0.0, |origin| (origin - length).max(0.0));
        let near = c.mode == WalkMode::Behind
            && tuning.enabled
            && behind_region
            && !self.retract_resume
            && !payout_stop
            && self.braking.is_none();
        let pace_min_length = self.pace.minimum_length(c);
        let pace_active = pace_selected
            && behind_region
            && length > pace_min_length
            && length < stop_length
            && p.max_speed_mps > 0.0
            && !self.stop_armed
            && !self.retract_resume
            && !self.retract_braking
            && !payout_stop
            && self.braking.is_none();
        let adaptive = near && tuning.adaptive_enabled && !pace_selected;
        let active = near
            && length > p.behind_start_cord_length_m
            && length < stop_length
            && p.max_speed_mps > 0.0;
        if active && !pace_selected {
            let raw = input.cord_speed_mps.unwrap();
            let alpha = if tuning.filter_ms <= 0.0 {
                1.0
            } else {
                1.0 - (-dt.clamp(0.0, 0.03) / (tuning.filter_ms / 1000.0)).exp()
            };
            let filtered = self
                .filtered_speed_mps
                .map_or(raw, |old| old + alpha * (raw - old));
            self.filtered_speed_mps = Some(filtered);
            input.cord_speed_mps = Some(filtered);
        } else {
            self.filtered_speed_mps = None;
        }
        let mut request = if pace_tracking && behind_region {
            LongitudinalRequest {
                speed_mps: 0.0,
                travel_mode: WalkMode::Behind,
            }
        } else {
            longitudinal_request(input, c)
        };
        let pace_request = if pace_active {
            let pace = self.pace.update(raw_input, measured_motion, dt, c);
            request.speed_mps = pace.speed_mps;
            pace
        } else {
            if pace_tracking
                && (self.braking.is_some()
                    || (behind_region && length > pace_min_length && p.max_speed_mps > 0.0))
            {
                self.pace.suspend();
            } else {
                // Ahead takeover discards the old pace estimate. During
                // braking, retain tracking so cord bounce keeps the run's
                // stop guard and cannot weaken an already active brake.
                self.pace.reset();
            }
            Default::default()
        };
        if adaptive && active {
            // A lower raw request may brake immediately; filtering only
            // delays increases. In particular, releasing fast retraction
            // must not keep accelerating on the old negative speed sample.
            // This is a bounded target reduction, never a derivative of noise.
            request.speed_mps = request
                .speed_mps
                .min(longitudinal_request(raw_input, c).speed_mps);
        }
        if payout_stop || self.braking.is_some() {
            // Hold translation until the latched brake completes. A tug also
            // requires returning to its recorded length before near cruise.
            request.speed_mps = 0.0;
        } else if self.retract_resume {
            // A brief fast gesture must not unlock the entire long-cord
            // position error. Speed is bounded independently by actual net
            // shortening AND current retraction speed; neither can add a
            // persistent cruise request after release.
            request = LongitudinalRequest {
                speed_mps: ((retract_distance - tuning.pull_back_min_retract_m).max(0.0)
                    * tuning.pull_back_length_gain)
                    .min(
                        (retract_speed - tuning.speed_deadband_mps).max(0.0)
                            * p.cord_speed_feedforward,
                    )
                    .min(p.max_speed_mps),
                travel_mode: WalkMode::Behind,
            };
        } else if self.stop_armed && behind_region {
            // Retraction can shorten the cord while the chassis decelerates.
            // Stay stopped instead of re-entering Behind during confirmation.
            request.speed_mps = 0.0;
        }
        let command = compute_request(input, c, request);
        let decelerating = command.linear_mps * v0 < 0.0 || command.linear_mps.abs() < v0.abs();
        let restart_control = self.retract_resume || self.retract_braking;
        let mut acceleration = if let Some(braking) = self.braking {
            let rate = match braking {
                BehindBraking::StopLine if pace_tracking => {
                    tuning.stop_deceleration_mps2.max(p.pace.deceleration_mps2)
                }
                BehindBraking::StopLine => tuning.stop_deceleration_mps2,
                BehindBraking::AheadSwitch => {
                    let rate = tuning
                        .switch_deceleration_mps2
                        .max(tuning.stop_deceleration_mps2);
                    if pace_tracking {
                        rate.max(p.pace.deceleration_mps2)
                    } else {
                        rate
                    }
                }
            };
            if payout_stop {
                rate.max(tuning.payout_stop_deceleration_mps2)
            } else {
                rate
            }
        } else if payout_stop {
            tuning.payout_stop_deceleration_mps2
        } else if restart_control && decelerating {
            tuning.pull_back_deceleration_mps2
        } else if self.retract_resume {
            // This is an explicit, separately configured exception to the
            // ordinary longitudinal ramp, not an increase in hard speed caps.
            (c.acceleration_mps2.max(tuning.response_acceleration_mps2)
                + tuning.pull_back_acceleration_gain
                    * (retract_speed - tuning.speed_deadband_mps).max(0.0))
            .min(tuning.pull_back_acceleration_mps2)
        } else if pace_active {
            if decelerating {
                p.pace.deceleration_mps2
            } else {
                p.pace.acceleration_mps2
            }
        } else if decelerating && c.mode == WalkMode::Behind && behind_region {
            if pace_selected {
                tuning.deceleration_mps2.max(p.pace.deceleration_mps2)
            } else {
                tuning.deceleration_mps2
            }
        } else if near {
            let payout = tuning.payout_weight(input.cord_speed_mps.unwrap());
            (tuning.payout_acceleration_mps2 * payout
                + tuning.response_acceleration_mps2 * (1.0 - payout))
                .min(c.acceleration_mps2)
        } else {
            c.acceleration_mps2
        };
        if adaptive && !restart_control {
            // Cord speed is relative motion, not a person's ground speed.
            // Use it to adjust responsiveness while retaining the length /
            // speed pedal law. Speed error also brakes promptly when the cord
            // stops moving or crosses a translation gate.
            let cord_speed = if decelerating {
                raw_input.cord_speed_mps.unwrap().abs()
            } else {
                input.cord_speed_mps.unwrap().abs()
            };
            let motion = (cord_speed - tuning.speed_deadband_mps).max(0.0);
            // During a reversal only the distance to zero is braking demand.
            let speed_error = if command.linear_mps * v0 < 0.0 {
                v0.abs()
            } else {
                (command.linear_mps - v0).abs()
            };
            let ceiling = if decelerating {
                // Adaptation may add braking, never weaken the base brake.
                tuning.adaptive_deceleration_mps2.max(acceleration)
            } else {
                tuning.adaptive_acceleration_mps2
            };
            // A high cruise setting alone must not turn a gentle start into
            // a hard launch. Speed error strengthens braking only; launch
            // response follows the cord gesture, including a soft neutral.
            let response = if decelerating {
                motion.max(speed_error)
            } else {
                motion
            };
            acceleration = (acceleration + tuning.adaptive_gain * response).min(ceiling);
        }
        self.status = FollowStatus {
            travel_mode: Some(request.travel_mode),
            braking: self.braking,
            payout_stop,
            payout_stop_resume_length_m: self.payout_stop_resume_length_m,
            waiting_for_pull_back: self.stop_armed && !self.retract_resume && behind_region,
            pull_back_ahead: false,
            retract_resume: self.retract_resume,
            retract_distance_m: retract_distance,
            retract_braking: self.retract_braking,
            acceleration_mps2: acceleration,
            adaptive_response: adaptive && !restart_control,
            pace_active,
            pace_tracking: pace_selected && self.pace.tracking(c),
            pace_reference_length_m: pace_selected.then_some(p.pace.target_length_m),
            pace_stop_length_m: pace_selected.then_some(p.pace.stop_length(c)),
            estimated_user_speed_mps: pace_request.estimated_speed_mps,
            pace_forward_speed_mps: pace_request.forward_speed_mps,
            pace_rotation_guard: pace_request.rotation_guard,
            pace_turn_scale: pace_request.forward_speed_mps.map(|_| 1.0),
            pace_correction_mps: pace_request.correction_mps,
            pace_target_mps: pace_active.then_some(request.speed_mps),
            pace_min_length_m: pace_selected.then_some(self.pace.minimum_length(c)),
        };
        // A stronger brake must not become a stronger reverse launch within
        // the same tick. Re-evaluate the acceleration ceiling after zero.
        let ramp_target = if command.linear_mps * v0 < 0.0 {
            ChassisCommand {
                linear_mps: 0.0,
                ..command
            }
        } else {
            command
        };
        let wheels = slew_motion_with_acceleration(previous, ramp_target, dt, c, acceleration);
        (command, wheels)
    }
}

#[cfg(test)]
fn legacy_follow_config() -> ChassisConfig {
    let mut c = ChassisConfig::default();
    c.follow.pace.enabled = false;
    c
}

#[cfg(test)]
#[path = "pace_tests.rs"]
mod pace_tests;

#[cfg(test)]
#[path = "pace_length_tests.rs"]
mod pace_length_tests;

#[cfg(test)]
#[path = "pace_switch_tests.rs"]
mod pace_switch_tests;

#[cfg(test)]
#[path = "single_steering_tests.rs"]
mod single_steering_tests;

#[cfg(test)]
#[path = "behind_speed_tests.rs"]
mod behind_speed_tests;

#[cfg(test)]
#[path = "behind_braking_tests.rs"]
mod behind_braking_tests;

#[cfg(test)]
#[path = "payout_stop_tests.rs"]
mod payout_stop_tests;

#[cfg(test)]
#[path = "adaptive_response_tests.rs"]
mod adaptive_response_tests;

#[cfg(test)]
#[path = "pull_back_tests.rs"]
mod pull_back_tests;

#[cfg(test)]
mod tests {
    use super::*;
    fn steering_driver_config() -> SteeringDriverConfig {
        SteeringDriverConfig {
            max_speed_rad_s: MOTOR_MAX_OUTPUT_RAD_S,
            acceleration_rad_s2: 200.0,
            deceleration_rad_s2: -200.0,
            position_kp: 54.0,
            position_ki: 0.0,
            velocity_kp: 0.00372,
            velocity_ki: 0.002,
        }
    }

    #[test]
    fn steering_driver_requires_finite_speed_signed_acceleration_and_nonnegative_gains() {
        let config = steering_driver_config();
        config.validate().unwrap();
        for field in 0..7 {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut invalid = config.clone();
                match field {
                    0 => invalid.max_speed_rad_s = value,
                    1 => invalid.acceleration_rad_s2 = value,
                    2 => invalid.deceleration_rad_s2 = value,
                    3 => invalid.position_kp = value,
                    4 => invalid.position_ki = value,
                    5 => invalid.velocity_kp = value,
                    _ => invalid.velocity_ki = value,
                }
                assert!(invalid.validate().is_err(), "field {field}, value {value}");
            }
        }
        for value in [0.0, -1.0, MOTOR_MAX_OUTPUT_RAD_S + 1e-9] {
            let mut invalid = config.clone();
            invalid.max_speed_rad_s = value;
            assert!(invalid.validate().is_err());
        }
        for value in [0.0, -1.0] {
            let mut invalid = config.clone();
            invalid.acceleration_rad_s2 = value;
            assert!(invalid.validate().is_err());
            invalid = config.clone();
            invalid.deceleration_rad_s2 = -value;
            assert!(invalid.validate().is_err());
        }
        for field in 0..4 {
            let mut invalid = config.clone();
            match field {
                0 => invalid.position_kp = -0.01,
                1 => invalid.position_ki = -0.01,
                2 => invalid.velocity_kp = -0.01,
                _ => invalid.velocity_ki = -0.01,
            }
            assert!(invalid.validate().is_err());
        }
    }

    #[test]
    fn optional_steering_driver_preserves_old_config_and_validates_opt_in() {
        let mut old = serde_json::to_value(ChassisConfig::default()).unwrap();
        old.as_object_mut().unwrap().remove("steering_driver");
        let mut config: ChassisConfig = serde_json::from_value(old).unwrap();
        assert_eq!(config.steering_driver, None);
        config.validate().unwrap();
        config.steering_driver = Some(steering_driver_config());
        config.validate().unwrap();
        let restored: ChassisConfig =
            serde_json::from_value(serde_json::to_value(&config).unwrap()).unwrap();
        assert_eq!(restored.steering_driver, config.steering_driver);
        config.steering_driver.as_mut().unwrap().deceleration_rad_s2 = 200.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn shared_frontend_vectors_match_backend() {
        // JSON.stringify emits integral floats as integers (60 rather than
        // 60.0). Compare numeric values while preserving every key and type.
        fn normalize_numbers(value: &mut serde_json::Value) {
            use serde_json::Value;
            match value {
                Value::Number(n) => *value = serde_json::json!(n.as_f64().unwrap()),
                Value::Array(a) => a.iter_mut().for_each(normalize_numbers),
                Value::Object(o) => o.values_mut().for_each(normalize_numbers),
                _ => {}
            }
        }
        let fixture: serde_json::Value =
            serde_json::from_str(include_str!("../test/fixtures/chassisVectors.json"))
                .unwrap();
        let mut expected_defaults = fixture["defaults"].clone();
        let mut actual_defaults = serde_json::to_value(ChassisConfig::default()).unwrap();
        normalize_numbers(&mut expected_defaults);
        normalize_numbers(&mut actual_defaults);
        assert_eq!(expected_defaults, actual_defaults);
        for case in fixture["cases"].as_array().unwrap() {
            let mut config = ChassisConfig::default();
            config.mode = serde_json::from_value(case["mode"].clone()).unwrap();
            config.follow.allow_reverse = case["allowReverse"].as_bool().unwrap();
            let input = serde_json::from_value(case["input"].clone()).unwrap();
            let actual = serde_json::to_value(compute(input, &config)).unwrap();
            for (key, expected) in case["expected"].as_object().unwrap() {
                assert!(
                    (actual[key].as_f64().unwrap() - expected.as_f64().unwrap()).abs() < 1e-10,
                    "{key}: {case}"
                );
            }
        }
    }
    fn input(angle: f64) -> FollowInput {
        FollowInput {
            encoder_sample_time_s: None,
            pull_back_speed_mps: None,
            cord_length_m: Some(1.6),
            cord_speed_mps: Some(0.1),
            encoder_angle_deg: Some(angle),
        }
    }
    #[test]
    fn gate_modes_and_geometry() {
        let mut c = ChassisConfig::default();
        c.validate().unwrap();
        let mut i = input(0.0);
        i.cord_length_m = Some(c.follow.behind_start_cord_length_m);
        i.cord_speed_mps = Some(10.0);
        assert_eq!(compute(i, &c).linear_mps, 0.0);
        let straight = compute(input(0.0), &c);
        assert!(straight.linear_mps > 0.0); // follow toward the driven-side cord exit
        assert!((c.motor_rps(0.2, false) - 0.530516477).abs() < 1e-8);
        c.mode = WalkMode::Ahead;
        let ahead = compute(input(0.0), &c);
        assert!(ahead.linear_mps < 0.0); // guide opposite the driven-side cord exit
        assert!(straight.speed_mps < ahead.speed_mps); // stretched leash slows behind mode
    }

    #[test]
    fn behind_mode_keeps_near_following_stops_then_switches_to_ahead() {
        let mut c = ChassisConfig::default();
        assert_eq!(c.follow.behind_start_cord_length_m, 0.75);
        assert_eq!(c.follow.behind_target_cord_length_m, 1.2);
        assert_eq!(c.follow.behind_stop_cord_length_m, 1.8);
        assert_eq!(c.follow.behind_ahead_switch_cord_length_m, 2.2);

        let at = |config: &ChassisConfig, length, cord_speed, angle| {
            compute(
                FollowInput {
                    encoder_sample_time_s: None,
                    pull_back_speed_mps: None,
                    cord_length_m: Some(length),
                    cord_speed_mps: Some(cord_speed),
                    encoder_angle_deg: Some(angle),
                },
                config,
            )
        };
        for length in [0.0, 0.749, 0.75, 1.8, 2.0, 2.2] {
            let stopped = at(&c, length, -100.0, 45.0);
            assert_eq!(stopped.speed_mps, 0.0, "length={length}");
            assert_eq!(stopped.linear_mps, 0.0, "length={length}");
            assert_eq!(stopped.steering_angle_deg, STEERING_CENTER_DEG + 45.0);
        }

        let just_started = at(&c, 0.750001, 100.0, 0.0);
        let desired = at(&c, 1.2, -100.0, 0.0);
        let stretched = at(&c, 1.5, 0.0, 0.0);
        let farther = at(&c, 1.7, 0.0, 0.0);
        assert!((just_started.speed_mps - 0.12).abs() < 1e-12);
        assert_eq!(desired.speed_mps, c.follow.max_speed_mps);
        assert!((stretched.speed_mps - 0.075).abs() < 1e-12);
        assert!(desired.speed_mps > stretched.speed_mps);
        assert!(stretched.speed_mps > farther.speed_mps);
        assert!(farther.speed_mps > 0.0);

        // Retraction raises the near target, payout lowers it. Ahead-mode
        // feed-forward and reverse tuning cannot change this response.
        c.follow.allow_reverse = true;
        c.follow.max_reverse_speed_mps = c.follow.max_speed_mps;
        assert!((at(&c, 1.5, -100.0, 0.0).speed_mps - 0.1).abs() < 1e-12);
        assert!((at(&c, 1.5, 100.0, 0.0).speed_mps - 0.06).abs() < 1e-12);

        // Beyond the adjustable far switch, the same selected Behind mode
        // uses Ahead direction and its existing length/feed-forward law.
        c.follow.allow_reverse = false;
        let far_ahead = at(&c, 2.3, 0.0, 0.0);
        assert!((far_ahead.speed_mps - 0.09).abs() < 1e-12);
        assert!(far_ahead.linear_mps < 0.0);
        assert_eq!(at(&c, 2.3, 100.0, 0.0).speed_mps, c.follow.max_speed_mps);
        c.follow.allow_reverse = true;
        assert_eq!(at(&c, 2.3, -100.0, 0.0).speed_mps, -0.2);

        c.follow.behind_ahead_switch_cord_length_m = 2.5;
        assert_eq!(at(&c, 2.3, 100.0, 45.0).linear_mps, 0.0);
        assert!((at(&c, 2.6, 0.0, 0.0).speed_mps - 0.09).abs() < 1e-12);
        c.follow.behind_ahead_switch_cord_length_m = 2.2;

        // Existing acceleration limiting decelerates into the stopped band
        // and accelerates out of rest in the opposite travel direction.
        let stopping = slew_motion(0.2, at(&c, 1.8, 0.0, 0.0), 0.01, &c);
        assert!((stopping - 0.197).abs() < 1e-12);
        let starting_ahead = slew_motion(0.0, far_ahead, 0.01, &c);
        assert!((starting_ahead + 0.003).abs() < 1e-12);
    }
    #[test]
    fn exit_side_and_travel_mode_preserve_corrective_heading_feedback() {
        for mode in [WalkMode::Behind, WalkMode::Ahead] {
            let mut c = ChassisConfig::default();
            c.mode = mode;
            c.max_wheel_speed_mps = 0.4;
            c.cord_exit_side = CordExitSide::PassiveAxle;
            let passive = compute(input(45.0), &c);
            c.cord_exit_side = CordExitSide::DrivenAxle;
            let driven = compute(input(45.0), &c);
            assert!((passive.linear_mps + driven.linear_mps).abs() < 1e-10);
            assert!((passive.steering_angle_deg - driven.steering_angle_deg).abs() < 1e-10);
            // A positive cord angle requests positive steering in either mode.
            assert_eq!(driven.steering_angle_deg, STEERING_CENTER_DEG + 45.0);
        }
    }
    #[test]
    fn saved_configs_without_exit_side_use_the_driven_axle() {
        let mut saved = serde_json::to_value(ChassisConfig::default()).unwrap();
        saved.as_object_mut().unwrap().remove("cord_exit_side");
        let restored: ChassisConfig = serde_json::from_value(saved).unwrap();
        assert_eq!(restored.cord_exit_side, CordExitSide::DrivenAxle);
    }
    #[test]
    fn missing_motor_fields_receive_single_steering_defaults() {
        let mut saved = serde_json::to_value(ChassisConfig::default()).unwrap();
        for key in [
            "drive_node_id",
            "steering_node_id",
            "drive_inverted",
            "steering_inverted",
            "steering_speed_at_rest_rad_s",
            "steering_speed_at_full_speed_rad_s",
            "drive_max_torque_nm",
            "steering_max_torque_nm",
            "drive_overcurrent_ratio",
            "steering_overcurrent_ratio",
            "drive_imax_a",
            "steering_imax_a",
        ] {
            saved.as_object_mut().unwrap().remove(key);
        }
        let restored: ChassisConfig = serde_json::from_value(saved).unwrap();
        restored.validate().unwrap();
        assert_eq!(
            serde_json::to_value(restored).unwrap(),
            serde_json::to_value(ChassisConfig::default()).unwrap()
        );
    }
    #[test]
    fn old_configs_receive_behind_leash_defaults() {
        let mut saved = serde_json::to_value(ChassisConfig::default()).unwrap();
        for key in [
            "behindStartCordLengthM",
            "behindTargetCordLengthM",
            "behindStopCordLengthM",
            "behindAheadSwitchCordLengthM",
        ] {
            saved["follow"].as_object_mut().unwrap().remove(key);
        }
        let restored: ChassisConfig = serde_json::from_value(saved).unwrap();
        restored.validate().unwrap();
        assert_eq!(restored.follow.behind_start_cord_length_m, 0.75);
        assert_eq!(restored.follow.behind_target_cord_length_m, 1.2);
        assert_eq!(restored.follow.behind_stop_cord_length_m, 1.8);
        assert_eq!(restored.follow.behind_ahead_switch_cord_length_m, 2.2);
    }
    #[test]
    fn previous_leash_defaults_migrate_to_the_new_stop_and_ahead_switch() {
        let mut saved = serde_json::to_value(ChassisConfig::default()).unwrap();
        saved["follow"]["behindStopCordLengthM"] = 2.3.into();
        saved["follow"]
            .as_object_mut()
            .unwrap()
            .remove("behindAheadSwitchCordLengthM");
        let restored: ChassisConfig = serde_json::from_value(saved).unwrap();
        restored.validate().unwrap();
        assert_eq!(restored.follow.behind_stop_cord_length_m, 1.8);
        assert_eq!(restored.follow.behind_ahead_switch_cord_length_m, 2.2);
    }
    #[test]
    fn steering_wrap_and_saturation() {
        let c = ChassisConfig::default();
        let right = compute(input(45.0), &c);
        assert_eq!(right.steering_angle_deg, STEERING_CENTER_DEG + 45.0);
        for a in -360..720 {
            let w = compute(input(a as f64), &c);
            assert!(w.linear_mps.abs() <= c.max_wheel_speed_mps + 1e-12);
        }
        assert!(
            (compute(input(359.0), &c).steering_angle_deg
                - compute(input(-1.0), &c).steering_angle_deg)
                .abs()
                < 1e-12
        );
        assert_eq!(compute(FollowInput::default(), &c).linear_mps, 0.0);
    }
    #[test]
    fn expanded_motion_boundaries_accept_maximum_and_reject_overflow() {
        let drive_speed_limit_mps = ChassisConfig::default().max_output_speed_mps();
        for (section, field, limit) in [
            ("follow", "startCordLengthM", 40.0),
            ("follow", "behindAheadSwitchCordLengthM", 80.0),
            ("follow", "lengthGainPerS", 20.0),
            ("follow", "cordSpeedFeedforward", 10.0),
            ("follow", "maxSpeedMps", drive_speed_limit_mps),
            ("follow", "maxReverseSpeedMps", drive_speed_limit_mps),
            ("", "max_wheel_speed_mps", drive_speed_limit_mps),
            ("", "acceleration_mps2", 4.0),
            ("", "steering_speed_at_rest_rad_s", STEERING_MAX_SPEED_RAD_S),
            (
                "",
                "steering_speed_at_full_speed_rad_s",
                STEERING_MAX_FULL_SPEED_RAD_S,
            ),
        ] {
            for (value, accepted) in [(limit, true), (limit + 0.000001, false)] {
                let mut json = serde_json::to_value(ChassisConfig::default()).unwrap();
                if section.is_empty() {
                    json[field] = value.into();
                } else {
                    json[section][field] = value.into();
                }
                let config: ChassisConfig = serde_json::from_value(json).unwrap();
                assert_eq!(
                    config.validate().is_ok(),
                    accepted,
                    "{section}.{field}={value}"
                );
            }
        }
        let mut config = ChassisConfig::default();
        config.drive_node_id = 16;
        assert!(config.validate().is_err());
    }

    #[test]
    fn behind_length_limits_accept_values_above_forty_and_reject_over_eighty() {
        let mut config = ChassisConfig::default();
        config.follow.behind_start_cord_length_m = 79.7;
        config.follow.behind_target_cord_length_m = 79.8;
        config.follow.behind_stop_cord_length_m = 79.9;
        config.follow.behind_ahead_switch_cord_length_m = 80.0;
        config.validate().unwrap();

        let valid = serde_json::to_value(config).unwrap();
        for (field, name) in [
            ("behindStartCordLengthM", "behind start length"),
            ("behindTargetCordLengthM", "behind target length"),
            ("behindStopCordLengthM", "behind stop length"),
            (
                "behindAheadSwitchCordLengthM",
                "behind-to-ahead switch length",
            ),
        ] {
            let mut over_limit = valid.clone();
            over_limit["follow"][field] = 80.000001.into();
            let over_limit: ChassisConfig = serde_json::from_value(over_limit).unwrap();
            assert_eq!(
                over_limit.validate().unwrap_err(),
                format!("{name} must be 0..80"),
                "{field}"
            );
        }
    }

    #[test]
    fn drive_speed_limits_match_200_rpm_for_every_supported_geometry() {
        for (diameter, ratio) in [(0.120, 1.0), (0.20, 2.0), (0.02, 100.0), (1.0, 0.1)] {
            let mut c = ChassisConfig::default();
            c.wheel_diameter_m = diameter;
            c.gear_ratio = ratio;
            let limit = c.max_output_speed_mps();
            c.follow.max_speed_mps = limit;
            c.follow.max_reverse_speed_mps = limit;
            c.follow.allow_reverse = true;
            c.max_wheel_speed_mps = limit;
            c.validate().unwrap();
            for inverted in [false, true] {
                let output_rad_s = c.motor_rps(limit, inverted).abs() * std::f64::consts::TAU;
                assert!((output_rad_s - MOTOR_MAX_OUTPUT_RAD_S).abs() < 1e-12);
            }
            c.mode = WalkMode::Ahead;
            for sign in [-1.0, 1.0] {
                let target = compute(
                    FollowInput {
                        cord_speed_mps: Some(sign * 1000.0),
                        ..input(0.0)
                    },
                    &c,
                );
                assert_eq!(target.speed_mps, sign * limit);
                assert_eq!(target.linear_mps.abs(), limit);
                assert_eq!(target.steering_angle_deg, STEERING_CENTER_DEG);
            }
            for invalid in [limit + 1e-6, -0.001, f64::INFINITY, f64::NAN] {
                for field in ["maximum speed", "reverse speed", "wheel speed limit"] {
                    let mut bad = c.clone();
                    match field {
                        "maximum speed" => bad.follow.max_speed_mps = invalid,
                        "reverse speed" => bad.follow.max_reverse_speed_mps = invalid,
                        _ => bad.max_wheel_speed_mps = invalid,
                    }
                    assert!(bad.validate().unwrap_err().contains(field));
                }
            }
            // Zero is a deliberate drive stop; steering remains available.
            c.max_wheel_speed_mps = 0.0;
            c.follow.max_speed_mps = 0.0;
            c.follow.max_reverse_speed_mps = 0.0;
            c.validate().unwrap();
            let stopped = compute(input(45.0), &c);
            assert_eq!(stopped.linear_mps, 0.0);
            assert_eq!(stopped.steering_angle_deg, STEERING_CENTER_DEG + 45.0);
        }
    }

    #[test]
    fn invalid_config_and_acceleration() {
        let mut c = ChassisConfig::default();
        c.steering_speed_at_rest_rad_s = f64::NAN;
        assert!(c.validate().is_err());
        c = ChassisConfig::default();
        c.steering_node_id = c.drive_node_id;
        assert!(c.validate().is_err());
        c = ChassisConfig::default();
        c.source = InputSource::Manual;
        assert!(c.validate().is_ok());
        c = ChassisConfig::default();
        c.follow.behind_target_cord_length_m = c.follow.behind_start_cord_length_m;
        assert!(c.validate().is_err());
        c = ChassisConfig::default();
        c.follow.behind_stop_cord_length_m = c.follow.behind_target_cord_length_m;
        assert!(c.validate().is_err());
        c = ChassisConfig::default();
        c.follow.behind_ahead_switch_cord_length_m = c.follow.behind_stop_cord_length_m;
        assert!(c.validate().is_err());
        c = ChassisConfig::default();
        c.steering_speed_at_rest_rad_s = -0.1;
        assert!(c.validate().is_err());
        c = ChassisConfig::default();
        c.steering_speed_at_rest_rad_s = f64::NAN;
        assert!(c.validate().is_err());
        c = ChassisConfig::default();
        c.max_wheel_speed_mps = 0.05;
        let reduced = slew_motion(0.2, ChassisCommand::default(), 0.01, &c);
        assert!(reduced.abs() <= 0.05);
    }
}
