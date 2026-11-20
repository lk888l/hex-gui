//! Pull-cord configuration, spring/friction/haptic models and feedback snapshots.
use serde::{Deserialize, Serialize};
use std::time::Duration;

mod a1;
mod a2;
mod friction;
pub use a1::{A1CurrentConfig, A1_PROTOCOL_MAX_RAW};
pub use a2::{A2SpeedConfig, A2_PROTOCOL_MAX_RAW, A2_PROTOCOL_MAX_RPM};
pub use friction::{
    FrictionCalibration, FrictionCompensationConfig, FrictionMode, FrictionOutput, FrictionRuntime,
    FrictionSource,
};
/// Pull-cord update period: 500 Hz / 2 ms.
pub const CONTROL_HZ: u64 = 500;
pub const CONTROL_PERIOD_US: u64 = 1_000_000 / CONTROL_HZ;

/// Linear, drum-side spring model. The 38 mm spring outer diameter describes
/// packaging, not the cord lever arm. This assumes spring and drum rotate 1:1.
/// Real wound-strip springs have hysteresis; stiffness/gain need bench calibration.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SpringCompensationConfig {
    pub enabled: bool,
    pub stiffness_nm_per_rad: f64,
    pub preload_length_m: f64,
    pub gain: f64,
    /// Multiplier relative to global gain at and beyond the alert boundary.
    #[serde(alias = "working_scale")]
    pub alert_scale: f64,
    pub max_torque_nm: f64,
    pub slew_rate_nm_s: f64,
}

impl Default for SpringCompensationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            // E*b*t^3/(12*L), SI units; E is provisional until alloy is known.
            stiffness_nm_per_rad: 206e9 * 0.007 * 0.0003_f64.powi(3) / (12.0 * 2.6),
            preload_length_m: 0.52,
            gain: 1.0,
            alert_scale: 0.3,
            max_torque_nm: 0.20,
            slew_rate_nm_s: 0.5,
        }
    }
}

impl SpringCompensationConfig {
    fn validate(&self) -> Result<(), String> {
        for (name, value, max) in [
            ("stiffness_nm_per_rad", self.stiffness_nm_per_rad, 1.0),
            ("preload_length_m", self.preload_length_m, 10.0),
            ("gain", self.gain, 2.0),
            ("alert_scale", self.alert_scale, 1.0),
            ("max_torque_nm", self.max_torque_nm, 2.0),
            ("slew_rate_nm_s", self.slew_rate_nm_s, 10.0),
        ] {
            if !value.is_finite() || !(0.0..=max).contains(&value) {
                return Err(format!(
                    "spring_compensation.{name} must be finite within 0..={max}"
                ));
            }
        }
        if self.slew_rate_nm_s == 0.0 {
            return Err("spring_compensation.slew_rate_nm_s must be positive".into());
        }
        Ok(())
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PullCordMode {
    A1Current,
    #[default]
    A2Speed,
}

/// Everything the operator can tune, in cord-facing units.
///
/// MHF uses the selected raw current or speed profile, with command polarity
/// from `retract_positive`. Nm fields remain for legacy models/templates.
/// Lengths are metres of cord, speeds metres per second of cord, torques N·m
/// at the shaft. The GUI shows the equivalent cord force (`τ / R`) alongside
/// each torque, since that is what a hand actually feels. Motor-producing
/// values describe the desired base effect. Spring cancellation is an
/// independent additive feed-forward; it never rescales speed or MIT gains.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PullCordConfig {
    /// Templates saved before mode selection always ran A2.
    #[serde(default)]
    pub control_mode: PullCordMode,
    /// Drum radius in metres. Cord length is `turns · 2πR`; layer build-up on
    /// the drum is not modelled, so keep the cord in a single layer or
    /// calibrate this against a tape measure.
    pub drum_radius_m: f64,
    /// Usable physical cord length and the right edge of the zone ruler.
    pub cord_total_length_m: f64,
    /// `true` when the positive shaft direction winds the cord **in**.
    pub retract_positive: bool,
    /// Raw MHF A1 commands. Missing in legacy Nm templates; converted once
    /// when loaded by the MHF service. Other motor controllers ignore this.
    #[serde(default)]
    pub a1_current: Option<A1CurrentConfig>,
    /// MHF A2 speed-loop configuration. Missing in older templates; populated
    /// by the MHF service/settings migration without changing cord geometry.
    #[serde(default)]
    pub a2_speed: Option<A2SpeedConfig>,
    #[serde(default)]
    pub spring_compensation: SpringCompensationConfig,
    #[serde(default)]
    pub friction_compensation: FrictionCompensationConfig,

    // ── homing ──
    /// Torque cap while seeking the mechanical stop.
    pub homing_torque_nm: f64,
    /// Cord speed target while seeking the stop.
    pub homing_speed_m_s: f64,
    /// Below this cord speed the drum counts as stalled.
    pub homing_stall_speed_m_s: f64,
    /// How long the stall must hold before the origin is accepted.
    pub homing_stall_hold_ms: u64,
    /// Give up and fault after this long without a stall.
    pub homing_timeout_s: f64,

    // ── zone boundaries, in metres of cord ──
    /// End of the light-payout zone.
    pub zone_near_end_m: f64,
    /// Ramp endpoint and start of the end-of-cord alert. Legacy settings use
    /// the former detent-zone endpoint to preserve the warning distance.
    #[serde(alias = "zone_knob_end_m")]
    pub zone_alert_start_m: f64,

    // ── forces ──
    /// Light retract torque in the first zone, including during payout.
    pub torque_near_nm: f64,
    /// Base retract torque at the alert boundary.
    #[serde(alias = "torque_main_nm")]
    pub torque_alert_nm: f64,
    /// Available torque once the cord is moving inward. At rest and during
    /// payout the smaller zone torque still determines the pull effort.
    #[serde(default = "default_retract_boost_torque_nm")]
    pub retract_boost_torque_nm: f64,

    // ── zero-friction payout and ramp transition ──
    /// Manual friction estimate when factory data is absent or deselected.
    pub zero_friction_assist_nm: f64,

    // ── end-of-cord alert ──
    /// Alert wave frequency.
    pub alert_pulse_hz: f64,
    /// Fraction of each period the pulse is on. Kept low on purpose: the pulse
    /// rides *on top of* the retract torque, so a high duty is mostly extra
    /// heat rather than extra warning.
    pub alert_duty: f64,
    /// Extra feed-forward torque during the pulse, in the retract direction.
    pub alert_pulse_nm: f64,

    // ── MIT law ──
    /// Velocity gain, N·m per Rev/s of shaft error.
    pub kd_nm_per_rev_per_s: f64,
    /// Maximum free-retraction speed; payout follows measured speed with a
    /// torque-sized bias, keeping this target independent of the pull force.
    pub retract_speed_m_s: f64,

    /// Legacy ceiling in permille of peak torque. Migrated to `a1_current.max_raw`
    /// for MHF; the historical HEX4310 controller wrote `0x4572`.
    pub max_torque_permille: u16,
}

impl Default for PullCordConfig {
    fn default() -> Self {
        let cord_total_length_m = 3.0;
        let zone_near_end_m = 1.8;
        // Preserve the previous warning onset while using the whole middle
        // distance for a single gradual transition.
        let zone_alert_start_m =
            zone_near_end_m + 37.0 * (cord_total_length_m - zone_near_end_m) / 49.0;
        Self {
            control_mode: PullCordMode::default(),
            drum_radius_m: 0.0425,
            cord_total_length_m,
            retract_positive: true,
            a1_current: None,
            a2_speed: None,
            spring_compensation: SpringCompensationConfig::default(),
            friction_compensation: FrictionCompensationConfig::default(),

            homing_torque_nm: 0.25,
            homing_speed_m_s: 0.25,
            homing_stall_speed_m_s: 0.02,
            homing_stall_hold_ms: 400,
            homing_timeout_s: 20.0,

            zone_near_end_m,
            zone_alert_start_m,

            torque_near_nm: 0.025,
            torque_alert_nm: 0.04,
            retract_boost_torque_nm: 0.12,

            zero_friction_assist_nm: 0.045,

            alert_pulse_hz: 2.0,
            alert_duty: 0.35,
            alert_pulse_nm: 0.40,

            kd_nm_per_rev_per_s: 2.0,
            retract_speed_m_s: 2.6,

            max_torque_permille: 250,
        }
    }
}

fn default_retract_boost_torque_nm() -> f64 {
    PullCordConfig::default().retract_boost_torque_nm
}

impl PullCordConfig {
    pub fn validate(&self) -> Result<(), String> {
        if let Some(a1) = self
            .a1_current
            .filter(|_| self.control_mode == PullCordMode::A1Current)
        {
            a1.validate()?;
        }
        if let Some(a2) = self
            .a2_speed
            .filter(|_| self.control_mode == PullCordMode::A2Speed)
        {
            a2.validate()?;
            let maximum_speed_m_s = A2_PROTOCOL_MAX_RPM / 60.0 * self.metres_per_rev();
            for (name, speed) in [
                ("homing_speed_m_s", self.homing_speed_m_s),
                ("retract_speed_m_s", self.retract_speed_m_s),
            ] {
                if speed > maximum_speed_m_s {
                    return Err(format!(
                        "{name} must not exceed the MHF5010 {A2_PROTOCOL_MAX_RPM} rpm protocol limit ({maximum_speed_m_s:.3} m/s at this drum radius)"
                    ));
                }
            }
        }
        self.spring_compensation.validate()?;
        self.friction_compensation.validate()?;
        let positive = [
            ("drum_radius_m", self.drum_radius_m),
            ("cord_total_length_m", self.cord_total_length_m),
            ("homing_torque_nm", self.homing_torque_nm),
            ("homing_speed_m_s", self.homing_speed_m_s),
            ("homing_timeout_s", self.homing_timeout_s),
            ("zone_near_end_m", self.zone_near_end_m),
            ("zone_alert_start_m", self.zone_alert_start_m),
            ("torque_near_nm", self.torque_near_nm),
            ("torque_alert_nm", self.torque_alert_nm),
            ("retract_boost_torque_nm", self.retract_boost_torque_nm),
            ("alert_pulse_hz", self.alert_pulse_hz),
            ("kd_nm_per_rev_per_s", self.kd_nm_per_rev_per_s),
            ("retract_speed_m_s", self.retract_speed_m_s),
        ];
        for (field, value) in positive {
            if !value.is_finite() || value <= 0.0 {
                return Err(format!("{field} must be finite and greater than zero"));
            }
        }
        for (field, value) in [
            ("homing_stall_speed_m_s", self.homing_stall_speed_m_s),
            ("alert_pulse_nm", self.alert_pulse_nm),
            ("zero_friction_assist_nm", self.zero_friction_assist_nm),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!("{field} must be finite and non-negative"));
            }
        }
        if !(self.zone_near_end_m < self.zone_alert_start_m
            && self.zone_alert_start_m < self.cord_total_length_m)
        {
            return Err(
                "zone boundaries must satisfy near end < alert start < total length".into(),
            );
        }
        if !(0.0..=1.0).contains(&self.alert_duty) {
            return Err("alert duty must be within 0..=1".to_string());
        }
        if self.max_torque_permille > 1000 {
            return Err("max torque must be 0..=1000 permille".to_string());
        }
        Ok(())
    }

    /// Metres of cord per shaft revolution.
    pub fn metres_per_rev(&self) -> f64 {
        std::f64::consts::TAU * self.drum_radius_m
    }
}

/// Geometry/direction are setup-time values. A2 limits, reference and PID
/// tuning can change during a run; the legacy controllers retain their guards.
pub fn validate_live_config(current: &PullCordConfig, next: &PullCordConfig) -> Result<(), String> {
    if current.control_mode != next.control_mode
        || current.drum_radius_m != next.drum_radius_m
        || current.retract_positive != next.retract_positive
        || current.a2_speed.is_some() != next.a2_speed.is_some()
    {
        return Err(
            "stop the pull-cord before changing drum radius, direction or control mode".into(),
        );
    }
    if current.control_mode == PullCordMode::A2Speed && current.a2_speed.is_some() {
        return Ok(());
    }
    let current_a1 = current
        .a1_current
        .unwrap_or_else(|| A1CurrentConfig::from_legacy(current));
    let next_a1 = next
        .a1_current
        .unwrap_or_else(|| A1CurrentConfig::from_legacy(next));
    if current.max_torque_permille != next.max_torque_permille
        || current_a1.max_raw != next_a1.max_raw
        || current_a1.full_current_raw != next_a1.full_current_raw
    {
        return Err(
            "stop the pull-cord before changing drum radius, direction, current/torque ceiling or A1 calibration"
                .into(),
        );
    }
    Ok(())
}

/// Positive physical spring torque in the retract direction, before gain/limits.
pub fn spring_torque_nm(length_m: f64, config: &PullCordConfig) -> f64 {
    config.spring_compensation.stiffness_nm_per_rad
        * (config.spring_compensation.preload_length_m + length_m.max(0.0))
        / config.drum_radius_m
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SpringOutput {
    pub estimated_torque_nm: f64,
    pub tff_nm: f64,
    pub limited: bool,
}

#[derive(Default)]
pub struct SpringRuntime {
    /// Full-strength amplitude, before the length envelope is applied.
    tff_nm: f64,
    alert_scale: Option<f64>,
}

impl SpringRuntime {
    /// Apply the length envelope after smoothing so a low slew rate cannot
    /// carry the first-zone amplitude into the alert zone.
    /// Live edits of alert_scale are smoothed in torque units at fixed length.
    /// With no valid origin (homing/fault), cancellation is zero.
    pub fn update(
        &mut self,
        length_m: Option<f64>,
        config: &PullCordConfig,
        available_torque_nm: f64,
    ) -> SpringOutput {
        let Some(length) = length_m.filter(|v| v.is_finite()) else {
            self.tff_nm = 0.0;
            self.alert_scale = None;
            return SpringOutput::default();
        };
        let spring = config.spring_compensation;
        let estimated = spring_torque_nm(length, config);
        let requested = if spring.enabled {
            estimated * spring.gain
        } else {
            0.0
        };
        let cap = spring.max_torque_nm.min(available_torque_nm.max(0.0));
        let target = -requested.min(cap);
        let step = spring.slew_rate_nm_s / CONTROL_HZ as f64;
        self.tff_nm += (target - self.tff_nm).clamp(-step, step);
        // A reduced hardware/headroom limit takes precedence over smoothing.
        self.tff_nm = self.tff_nm.clamp(-cap, 0.0);
        let alert_scale = self.alert_scale.get_or_insert(spring.alert_scale);
        let scale_step = if self.tff_nm.abs() > step {
            step / self.tff_nm.abs()
        } else {
            1.0
        };
        *alert_scale += (spring.alert_scale - *alert_scale).clamp(-scale_step, scale_step);
        let scale = *alert_scale + (1.0 - *alert_scale) * payout_compensation_scale(length, config);
        SpringOutput {
            estimated_torque_nm: estimated,
            tff_nm: self.tff_nm * scale,
            limited: scale > 0.0 && requested > cap,
        }
    }
}

// ──────────────────────────────── live state ─────────────────────────────────

/// Which part of the cord the person is in right now.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Zone {
    /// First zone: light resting tension and payout assistance in preload mode.
    NearOrigin,
    /// Force ramps up with length.
    RampUp,
    /// Past the ramp: light retract force plus the alert wave.
    Alert,
}

/// Where the run is in its lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    /// Attached, motor enabled, not yet homed.
    Idle,
    /// Winding in against the stop to find the origin.
    Homing,
    /// Normal zoned operation.
    Running,
    /// Stopped on a motor fault or a failed homing; needs operator action.
    Fault,
}

#[derive(Debug, Clone, Serialize)]
pub struct PullCordState {
    pub running: bool,
    pub node_id: Option<u8>,
    pub phase: Phase,
    /// Non-null while `phase` is `Fault`, or after a failed start.
    pub fault: Option<String>,
    pub config: PullCordConfig,

    /// `null` until homing has established an origin.
    pub length_m: Option<f64>,
    /// Positive while the cord pays out.
    pub cord_speed_m_s: Option<f64>,
    pub zone: Option<Zone>,
    /// Base retract torque before any feed-forward (including spring cancellation).
    /// None for MHF A1 until the command-to-torque relation is calibrated.
    pub commanded_torque_nm: Option<f64>,
    pub spring_torque_nm: f64,
    /// Applied physical spring cancellation; negative is payout, both polarities.
    pub spring_tff_nm: f64,
    pub spring_compensation_limited: bool,
    /// Signed payout feed-forward in cord-facing coordinates. Negative points
    /// in the payout direction. It is full-strength in the first zone and
    /// fades to zero through the ramp.
    pub zero_friction_tff_nm: f64,
    /// Whether the selected strategy enables resting/payout assistance.
    pub zero_friction_active: bool,
    /// Raw command-domain, cord-facing contribution; never factor-scaled twice.
    pub friction_tff_raw_nm: f64,
    pub friction_source: FrictionSource,
    pub friction_preloading: bool,
    pub friction_compensation_limited: bool,
    /// Feed-forward torque currently added by the alert wave.
    pub alert_pulse_nm: f64,
    /// `true` on the on-phase of the alert wave, for a blinking indicator.
    pub alert_active: bool,
    /// Legacy shaft torque (`0x4577`, factor-corrected); None for uncalibrated MHF.
    pub measured_torque_nm: Option<f64>,
    /// Measured torque expressed as cord force, `τ / R`.
    pub measured_force_n: Option<f64>,
    /// Signed MHF5010 A1 current command; zero when A2 speed control is active.
    pub current_command_raw: i16,
    /// Effective nonnegative A2 speed-loop torque limit, in driver raw units.
    pub torque_limit_raw: u16,
    /// Last acknowledged live RAM Torque Limit (C1:1E); None if unconfirmed
    /// or the MHF controller is no longer active.
    pub global_torque_limit_raw: Option<u16>,
    /// Signed shaft velocity target streamed through A2, in degrees/second.
    pub speed_command_dps: f64,
    /// Diagnostic magnetic encoder offset; never used to adjust pull current.
    /// None if encoder feedback is unusable.
    pub encoder_offset_deg: Option<f64>,
    /// Stationary near-zone A1 target, before length blending and caps.
    pub stationary_current_raw: Option<f64>,
    pub measured_current_raw: Option<i16>,
    pub motor_speed_dps: Option<i16>,
    pub position_angle_deg: Option<f64>,
    pub bus_voltage_v: Option<f64>,
    /// Ages of the three independent MHF feedback streams at the last control tick.
    pub a1_feedback_age_ms: Option<u64>,
    /// A2 feedback age; the legacy A1 age mirrors this for older consumers.
    pub a2_feedback_age_ms: Option<u64>,
    pub angle_feedback_age_ms: Option<u64>,
    pub motor_status_age_ms: Option<u64>,
    /// Base requested retract force, `τ / R`, before feed-forward components.
    pub commanded_force_n: Option<f64>,
    /// Streamed velocity target, in shaft Rev/s.
    pub vdes_rev_per_s: f64,
    /// Streamed Kp/Kd output limit, in permille of peak torque.
    pub limit_permille: u16,

    pub motor_temp_c: Option<f32>,
    pub driver_temp_c: Option<f32>,
    pub peak_torque_nm: Option<f32>,
    pub torque_factor: Option<f64>,
    pub detailed_error: Option<u16>,
    pub online: bool,
    /// Homing progress, 0..1, for the progress bar.
    pub homing_progress: f64,
}

impl PullCordState {
    /// The chassis follows feedback only while the pull-cord run is healthy.
    pub fn feedback_ready_for_chassis(&self) -> bool {
        self.running && self.phase == Phase::Running && self.fault.is_none()
    }

    pub fn idle(config: PullCordConfig) -> Self {
        Self {
            running: false,
            node_id: None,
            phase: Phase::Idle,
            fault: None,
            config,
            length_m: None,
            cord_speed_m_s: None,
            zone: None,
            commanded_torque_nm: None,
            spring_torque_nm: 0.0,
            spring_tff_nm: 0.0,
            spring_compensation_limited: false,
            zero_friction_tff_nm: 0.0,
            zero_friction_active: false,
            friction_tff_raw_nm: 0.0,
            friction_source: FrictionSource::Off,
            friction_preloading: false,
            friction_compensation_limited: false,
            alert_pulse_nm: 0.0,
            alert_active: false,
            measured_torque_nm: None,
            measured_force_n: None,
            current_command_raw: 0,
            torque_limit_raw: 0,
            global_torque_limit_raw: None,
            speed_command_dps: 0.0,
            encoder_offset_deg: None,
            stationary_current_raw: None,
            measured_current_raw: None,
            motor_speed_dps: None,
            position_angle_deg: None,
            bus_voltage_v: None,
            a1_feedback_age_ms: None,
            a2_feedback_age_ms: None,
            angle_feedback_age_ms: None,
            motor_status_age_ms: None,
            commanded_force_n: None,
            vdes_rev_per_s: 0.0,
            limit_permille: 0,
            motor_temp_c: None,
            driver_temp_c: None,
            peak_torque_nm: None,
            torque_factor: None,
            detailed_error: None,
            online: false,
            homing_progress: 0.0,
        }
    }
}

// ─────────────────────────── pure control functions ──────────────────────────

/// Base retract torque without payout assistance, in N·m at the shaft.
///
/// Split out from the loop so the zone profile is testable without a motor.
pub fn zone_torque_nm(length_m: f64, config: &PullCordConfig) -> (Zone, f64) {
    zone_profile(
        length_m,
        config,
        config.torque_near_nm,
        config.torque_alert_nm,
    )
}

fn zone_profile(length_m: f64, config: &PullCordConfig, near: f64, alert: f64) -> (Zone, f64) {
    if !length_m.is_finite() {
        return (Zone::NearOrigin, near);
    }
    if length_m < config.zone_near_end_m {
        // Deliberately open-ended below zero: a slightly negative reading after
        // homing drift is still "nearly wound in", not an error.
        (Zone::NearOrigin, near)
    } else if length_m < config.zone_alert_start_m {
        let span = config.zone_alert_start_m - config.zone_near_end_m;
        let t = ((length_m - config.zone_near_end_m) / span).clamp(0.0, 1.0);
        (Zone::RampUp, near + t * (alert - near))
    } else {
        (Zone::Alert, alert)
    }
}

/// Apply the same end-zone adjustment in every direction. Measured cord speed
/// is positive during payout, regardless of the installed motor polarity.
fn directional_target_raw(
    stationary_at_length: f64,
    cord_speed_m_s: f64,
    payout_raw: u16,
    near_raw: u16,
    retract_raw: u16,
) -> f64 {
    const MOTION_THRESHOLD_M_S: f64 = 0.005;
    let base = if cord_speed_m_s.is_finite() && cord_speed_m_s > MOTION_THRESHOLD_M_S {
        payout_raw
    } else if cord_speed_m_s.is_finite() && cord_speed_m_s < -MOTION_THRESHOLD_M_S {
        retract_raw
    } else {
        near_raw
    };
    stationary_at_length + f64::from(base) - f64::from(near_raw)
}

/// Give a winding cord extra acceleration authority without making stationary
/// or outward pull harder. The blend is continuous at zero and reaches the
/// configured ceiling by 0.15 m/s of inward speed.
pub fn retract_torque_nm(length_m: f64, speed_m_s: f64, config: &PullCordConfig) -> f64 {
    let base = zone_torque_nm(length_m, config).1;
    retract_profile(base, config.retract_boost_torque_nm, speed_m_s)
}

fn retract_profile(base: f64, boost: f64, speed_m_s: f64) -> f64 {
    if !speed_m_s.is_finite() {
        return base;
    }
    let x = (-speed_m_s / 0.15).clamp(0.0, 1.0);
    let blend = x * x * (3.0 - 2.0 * x);
    base + (boost - base).max(0.0) * blend
}

/// Fraction of the full payout compensation to apply at this cord length.
///
/// Compensation stays at full strength throughout the zero-friction zone,
/// then fades linearly to zero across the ramp. Combined with
/// base retract torque, payout assistance fades continuously into the alert.
pub fn payout_compensation_scale(length_m: f64, config: &PullCordConfig) -> f64 {
    if !length_m.is_finite() {
        return 0.0;
    }
    if length_m <= config.zone_near_end_m {
        1.0
    } else if length_m >= config.zone_alert_start_m {
        0.0
    } else {
        let span = config.zone_alert_start_m - config.zone_near_end_m;
        ((config.zone_alert_start_m - length_m) / span).clamp(0.0, 1.0)
    }
}

/// Alert feed-forward torque at `elapsed` into the run.
///
/// Unipolar on purpose: the pulse rides on top of the retract torque, so it can
/// only ever tug harder, never push the cord back out.
pub fn alert_pulse_nm(elapsed: Duration, config: &PullCordConfig) -> f64 {
    if config.alert_pulse_nm <= 0.0 || config.alert_duty <= 0.0 {
        return 0.0;
    }
    let phase = (elapsed.as_secs_f64() * config.alert_pulse_hz).fract();
    if phase < config.alert_duty {
        config.alert_pulse_nm
    } else {
        0.0
    }
}

/// Track an outward pull with a small retract bias. This keeps normal pull
/// effort tied to the torque limit even when free retraction is configured fast.
/// Once the cord stops paying out, the full speed target is restored so loose
/// cord can be taken up promptly. The existing torque ceiling still applies.
#[derive(Debug, Default)]
pub struct PayoutFollowRuntime {
    active: bool,
}

impl PayoutFollowRuntime {
    pub fn deactivate(&mut self) {
        self.active = false;
    }

    /// Signed retract speed target in cord units.
    pub fn update(
        &mut self,
        cord_speed_m_s: Option<f64>,
        base_torque_nm: f64,
        free_retract_m_s: f64,
        config: &PullCordConfig,
    ) -> f64 {
        let Some(speed) = cord_speed_m_s.filter(|speed| speed.is_finite()) else {
            self.deactivate();
            return free_retract_m_s;
        };
        if self.active {
            if speed <= 0.005 {
                self.deactivate();
            }
        } else if speed >= 0.015 {
            self.active = true;
        }
        if !self.active {
            return free_retract_m_s;
        }
        let bias = (base_torque_nm / config.kd_nm_per_rev_per_s * config.metres_per_rev())
            .clamp(0.0, 0.02);
        (-speed + bias).clamp(-config.retract_speed_m_s, config.retract_speed_m_s)
    }
}

/// Physical N·m → the raw-domain permille the `0x4102:06` clamp expects.
pub fn torque_to_limit_permille(torque_nm: f64, peak_nm: f64, torque_factor: f64) -> u16 {
    if !peak_nm.is_finite() || peak_nm <= 0.0 {
        return 0;
    }
    let raw = torque_nm * torque_factor / peak_nm * 1000.0;
    raw.round().clamp(0.0, 1000.0) as u16
}

/// Shaft gain (`0x4102:05` raw u16) for a gain in N·m per Rev/s.
pub fn kd_to_raw(kd_nm_per_rev_per_s: f64, kp_kd_factor: f64) -> u16 {
    if !kp_kd_factor.is_finite() || kp_kd_factor <= 0.0 {
        return 0;
    }
    (kd_nm_per_rev_per_s / kp_kd_factor)
        .round()
        .clamp(0.0, f64::from(u16::MAX)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> PullCordConfig {
        PullCordConfig::default()
    }

    #[test]
    fn default_config_validates() {
        config().validate().expect("defaults must be valid");
    }

    #[test]
    fn default_config_matches_the_latest_bench_tuning_snapshot() {
        let cfg = config();
        assert_eq!(cfg.zero_friction_assist_nm, 0.045);
        assert_eq!(cfg.torque_near_nm, 0.025);
        assert_eq!(cfg.torque_alert_nm, 0.04);
    }

    #[test]
    fn spring_model_matches_the_supplied_strip_drum_and_preload() {
        let cfg = config();
        assert!(
            (cfg.spring_compensation.stiffness_nm_per_rad - 0.0012478846153846154).abs() < 1e-12
        );
        assert!((spring_torque_nm(0.0, &cfg) - 0.015268235294117645).abs() < 1e-12);
        assert!((spring_torque_nm(3.0, &cfg) - 0.10335420814479637).abs() < 1e-10);
        // Doubling the spring deflection doubles torque; slight negative drift
        // below the physical stop never reverses the spring compensation.
        assert!((spring_torque_nm(0.52, &cfg) - 2.0 * spring_torque_nm(0.0, &cfg)).abs() < 1e-12);
        assert_eq!(spring_torque_nm(-0.01, &cfg), spring_torque_nm(0.0, &cfg));
    }

    #[test]
    fn spring_compensation_ramps_limits_and_clears_without_an_origin() {
        let mut cfg = config();
        let mut runtime = SpringRuntime::default();
        let first = runtime.update(Some(1.0), &cfg, 1.0);
        assert!((first.tff_nm + 0.001).abs() < 1e-12);
        for _ in 0..200 {
            runtime.update(Some(1.0), &cfg, 1.0);
        }
        let full = runtime.update(Some(1.0), &cfg, 1.0);
        assert!((full.tff_nm + full.estimated_torque_nm).abs() < 1e-12);
        assert!(!full.limited);
        // A lowered motor ceiling takes effect immediately.
        let limited = runtime.update(Some(1.0), &cfg, 0.01);
        assert_eq!(limited.tff_nm, -0.01);
        assert!(limited.limited);
        assert_eq!(runtime.update(Some(1.0), &cfg, -0.01).tff_nm, 0.0);
        cfg.spring_compensation.max_torque_nm = 0.005;
        for _ in 0..10 {
            runtime.update(Some(1.0), &cfg, 1.0);
        }
        assert_eq!(runtime.update(Some(1.0), &cfg, 1.0).tff_nm, -0.005);
        cfg.spring_compensation.enabled = false;
        let fading = runtime.update(Some(1.0), &cfg, 1.0);
        assert!(fading.tff_nm > -0.005 && fading.tff_nm < 0.0);
        for _ in 0..10 {
            runtime.update(Some(1.0), &cfg, 1.0);
        }
        assert_eq!(runtime.tff_nm, 0.0);
        cfg.spring_compensation.enabled = true;
        runtime.update(Some(1.0), &cfg, 1.0);
        assert_eq!(runtime.update(None, &cfg, 1.0).tff_nm, 0.0);
        assert_eq!(runtime.update(Some(f64::NAN), &cfg, 1.0).tff_nm, 0.0);
    }

    #[test]
    fn spring_zone_coefficients_follow_current_model_and_are_continuous() {
        for working_scale in [0.0, 0.3, 1.0] {
            let mut cfg = config();
            cfg.spring_compensation.alert_scale = working_scale;
            cfg.spring_compensation.gain = 0.8;
            let start = cfg.zone_near_end_m;
            let end = cfg.zone_alert_start_m;
            let expected = |length: f64, scale: f64| {
                -spring_torque_nm(length, &cfg) * cfg.spring_compensation.gain * scale
            };
            let mut runtime = SpringRuntime::default();
            for (length, scale) in [
                (0.4, 1.0),
                (start, 1.0),
                ((start + end) / 2.0, (1.0 + working_scale) / 2.0),
                (end, working_scale),
                (cfg.cord_total_length_m, working_scale),
                ((start + end) / 2.0, (1.0 + working_scale) / 2.0),
                (start, 1.0),
            ] {
                for _ in 0..300 {
                    runtime.update(Some(length), &cfg, 1.0);
                }
                let out = runtime.update(Some(length), &cfg, 1.0);
                assert!((out.tff_nm - expected(length, scale)).abs() < 1e-12);
                assert!((out.estimated_torque_nm - spring_torque_nm(length, &cfg)).abs() < 1e-12);
            }
            for boundary in [start, end] {
                for _ in 0..300 {
                    runtime.update(Some(boundary), &cfg, 1.0);
                }
                let left = runtime.update(Some(boundary - 1e-8), &cfg, 1.0).tff_nm;
                let right = runtime.update(Some(boundary + 1e-8), &cfg, 1.0).tff_nm;
                assert!((left - right).abs() < 1e-8);
            }
        }
    }

    #[test]
    fn working_coefficient_live_edits_are_smooth_and_reach_both_endpoints() {
        let mut cfg = config();
        let length = cfg.zone_alert_start_m + 0.1;
        let mut runtime = SpringRuntime::default();
        for _ in 0..300 {
            runtime.update(Some(length), &cfg, 1.0);
        }
        let mut previous = runtime.update(Some(length), &cfg, 1.0).tff_nm;
        assert!(previous < 0.0);
        for target in [1.0, 0.0, 0.3] {
            cfg.spring_compensation.alert_scale = target;
            for _ in 0..300 {
                let out = runtime.update(Some(length), &cfg, 1.0);
                assert!(
                    (out.tff_nm - previous).abs()
                        <= cfg.spring_compensation.slew_rate_nm_s / CONTROL_HZ as f64 + 1e-12
                );
                previous = out.tff_nm;
            }
            assert!((previous + spring_torque_nm(length, &cfg) * target).abs() < 1e-12);
        }
    }

    #[test]
    fn spring_length_envelope_has_no_slew_tail_or_false_limit_in_working_zone() {
        let mut cfg = config();
        cfg.spring_compensation.alert_scale = 0.0;
        let mut runtime = SpringRuntime::default();
        for _ in 0..200 {
            runtime.update(Some(cfg.zone_near_end_m), &cfg, 1.0);
        }
        // Deliberately make a time-based fade too slow to reach zero in a tick.
        cfg.spring_compensation.slew_rate_nm_s = 0.0001;
        let mid = (cfg.zone_near_end_m + cfg.zone_alert_start_m) / 2.0;
        let before = runtime.update(Some(mid), &cfg, 1.0);
        assert!(before.tff_nm < -0.01);
        assert_eq!(
            runtime
                .update(Some(cfg.zone_alert_start_m), &cfg, 1.0)
                .tff_nm,
            0.0
        );
        let no_headroom = runtime.update(Some(cfg.zone_alert_start_m), &cfg, 0.0);
        assert_eq!(no_headroom.tff_nm, 0.0);
        assert!(!no_headroom.limited);
        // Zone edits use the same live boundaries as the rest of the controller.
        cfg.zone_near_end_m = 0.5;
        cfg.zone_alert_start_m = 1.0;
        assert_eq!(runtime.update(Some(1.0), &cfg, 1.0).tff_nm, 0.0);
    }

    #[test]
    fn old_pull_configs_deserialize_and_invalid_spring_configs_are_rejected() {
        let mut previous = serde_json::to_value(config()).unwrap();
        previous["spring_compensation"]
            .as_object_mut()
            .unwrap()
            .remove("alert_scale");
        previous["spring_compensation"]["gain"] = 0.7.into();
        let upgraded: PullCordConfig = serde_json::from_value(previous).unwrap();
        assert_eq!(upgraded.spring_compensation.alert_scale, 0.3);
        assert_eq!(upgraded.spring_compensation.gain, 0.7);
        for value in [0.0, 0.3, 1.0] {
            let mut cfg = config();
            cfg.spring_compensation.alert_scale = value;
            cfg.validate().unwrap();
            let reopened: PullCordConfig =
                serde_json::from_str(&serde_json::to_string(&cfg).unwrap()).unwrap();
            assert_eq!(reopened.spring_compensation.alert_scale, value);
        }
        for value in [f64::NAN, f64::INFINITY, -0.1, 1.01] {
            let mut cfg = config();
            cfg.spring_compensation.alert_scale = value;
            assert!(cfg.validate().is_err());
        }
        let mut json = serde_json::to_value(config()).unwrap();
        json.as_object_mut().unwrap().remove("spring_compensation");
        let legacy: PullCordConfig = serde_json::from_value(json).unwrap();
        assert_eq!(
            legacy.spring_compensation,
            SpringCompensationConfig::default()
        );
        for bad in [f64::NAN, f64::INFINITY, -0.01, 2.1] {
            let mut cfg = config();
            cfg.spring_compensation.gain = bad;
            assert!(cfg.validate().is_err());
        }
        let mut cfg = config();
        cfg.spring_compensation.slew_rate_nm_s = 0.0;
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn default_stage_layout_preserves_the_alert_distance() {
        let cfg = config();
        assert_eq!(cfg.zone_near_end_m, 1.8);
        assert_eq!(cfg.cord_total_length_m, 3.0);
        assert!((cfg.zone_alert_start_m - (1.8 + 37.0 * 1.2 / 49.0)).abs() < 1e-12);
        assert_eq!(zone_torque_nm(cfg.zone_near_end_m, &cfg).0, Zone::RampUp);
        assert_eq!(zone_torque_nm(cfg.zone_alert_start_m, &cfg).0, Zone::Alert);
    }

    #[test]
    fn zone_boundaries_must_increase() {
        let mut cfg = config();
        cfg.zone_alert_start_m = cfg.zone_near_end_m;
        assert!(cfg.validate().is_err());
        cfg.zone_alert_start_m = cfg.cord_total_length_m;
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn first_zone_extends_below_zero() {
        // Homing drift can leave a slightly negative reading; that is still the
        // weakest zone, never a jump to full retract force.
        let cfg = config();
        let (zone, torque) = zone_torque_nm(-0.05, &cfg);
        assert_eq!(zone, Zone::NearOrigin);
        assert_eq!(torque, cfg.torque_near_nm);
    }

    #[test]
    fn payout_follows_measured_speed_and_release_restores_fast_retraction() {
        let cfg = config();
        let mut follow = PayoutFollowRuntime::default();
        assert_eq!(retract_torque_nm(1.0, 0.2, &cfg), cfg.torque_near_nm);
        assert_eq!(retract_torque_nm(1.0, 0.0, &cfg), cfg.torque_near_nm);
        assert!(retract_torque_nm(1.0, -0.075, &cfg) > cfg.torque_near_nm);
        assert_eq!(
            retract_torque_nm(1.0, -0.15, &cfg),
            cfg.retract_boost_torque_nm
        );
        assert_eq!(
            follow.update(Some(0.0), cfg.torque_near_nm, cfg.retract_speed_m_s, &cfg),
            cfg.retract_speed_m_s
        );
        let speed = 0.2;
        let target = follow.update(Some(speed), cfg.torque_near_nm, cfg.retract_speed_m_s, &cfg);
        let bias = cfg.torque_near_nm / cfg.kd_nm_per_rev_per_s * cfg.metres_per_rev();
        assert!((target + speed - bias).abs() < 1e-12);
        assert_eq!(
            follow.update(Some(-0.1), cfg.torque_near_nm, cfg.retract_speed_m_s, &cfg),
            cfg.retract_speed_m_s
        );
        assert_eq!(
            follow.update(None, cfg.torque_near_nm, cfg.retract_speed_m_s, &cfg),
            cfg.retract_speed_m_s
        );
    }

    #[test]
    fn ramp_zone_interpolates_between_the_two_force_levels() {
        let cfg = config();
        let mid = (cfg.zone_near_end_m + cfg.zone_alert_start_m) / 2.0;
        let (zone, torque) = zone_torque_nm(mid, &cfg);
        assert_eq!(zone, Zone::RampUp);
        let expected = (cfg.torque_near_nm + cfg.torque_alert_nm) / 2.0;
        assert!((torque - expected).abs() < 1e-12, "{torque} vs {expected}");

        // The base retract profile itself is continuous at both ends.
        let (_, at_near) = zone_torque_nm(cfg.zone_near_end_m, &cfg);
        assert!((at_near - cfg.torque_near_nm).abs() < 1e-12);
        let (_, just_below_ramp_end) = zone_torque_nm(cfg.zone_alert_start_m - 1e-9, &cfg);
        assert!((just_below_ramp_end - cfg.torque_alert_nm).abs() < 1e-6);
    }

    #[test]
    fn payout_assistance_fades_continuously_into_alert() {
        let cfg = config();
        let effective = |length_m: f64| {
            zone_torque_nm(length_m, &cfg).1
                - cfg.zero_friction_assist_nm * payout_compensation_scale(length_m, &cfg)
        };
        let near = cfg.zone_near_end_m;
        let alert = cfg.zone_alert_start_m;
        assert_eq!(payout_compensation_scale(near, &cfg), 1.0);
        assert_eq!(payout_compensation_scale(alert, &cfg), 0.0);
        assert!((effective(near - 1e-9) - effective(near)).abs() < 1e-9);
        assert!((effective(alert - 1e-9) - effective(alert)).abs() < 1e-9);
        assert!((effective(alert) - cfg.torque_alert_nm).abs() < 1e-12);
    }

    #[test]
    fn alert_keeps_the_ramp_endpoint_base_force() {
        let cfg = config();
        let (zone, torque) = zone_torque_nm(cfg.zone_alert_start_m + 0.1, &cfg);
        assert_eq!(zone, Zone::Alert);
        assert_eq!(torque, cfg.torque_alert_nm);
    }

    #[test]
    fn alert_pulse_is_unipolar_and_respects_its_duty() {
        let cfg = config();
        // 2 Hz, 35% duty: on for the first 175 ms of each 500 ms period.
        let on = alert_pulse_nm(Duration::from_millis(50), &cfg);
        let off = alert_pulse_nm(Duration::from_millis(300), &cfg);
        assert_eq!(on, cfg.alert_pulse_nm);
        assert_eq!(off, 0.0);
        assert!(
            on >= 0.0 && off >= 0.0,
            "the pulse never pushes the cord out"
        );

        // Next period repeats.
        assert_eq!(alert_pulse_nm(Duration::from_millis(550), &cfg), on);
        assert_eq!(alert_pulse_nm(Duration::from_millis(800), &cfg), 0.0);
    }

    #[test]
    fn a_zero_amplitude_alert_never_pulses() {
        let mut cfg = config();
        cfg.alert_pulse_nm = 0.0;
        assert_eq!(alert_pulse_nm(Duration::from_millis(10), &cfg), 0.0);
    }

    #[test]
    fn limit_permille_scales_by_peak_and_factor_and_saturates() {
        // Half of peak with a neutral factor.
        assert_eq!(torque_to_limit_permille(4.0, 8.0, 1.0), 500);
        // The factory factor converts physical into raw command units.
        assert_eq!(torque_to_limit_permille(4.0, 8.0, 1.1), 550);
        // Asking for more than peak clamps instead of wrapping.
        assert_eq!(torque_to_limit_permille(99.0, 8.0, 1.0), 1000);
        // A nonsense peak yields no torque rather than a huge one.
        assert_eq!(torque_to_limit_permille(1.0, 0.0, 1.0), 0);
    }

    #[test]
    fn kd_raw_uses_the_motors_own_scale_factor() {
        // 0.0118 Nm/Rev per unit is the measured 4310 factor.
        assert_eq!(kd_to_raw(3.0, 0.011813799), 254);
        assert_eq!(kd_to_raw(3.0, 0.0), 0);
        // A gain past the u16 field saturates rather than wrapping to a small one.
        assert_eq!(kd_to_raw(1.0e9, 0.0118), u16::MAX);
    }

    #[test]
    fn metres_per_rev_follows_the_drum_radius() {
        let mut cfg = config();
        cfg.drum_radius_m = 0.05;
        assert!((cfg.metres_per_rev() - 0.314_159_265).abs() < 1e-6);
    }
}

#[cfg(test)]
mod mpu_tests {
    use super::*;
    #[test]
    fn live_edit_preserves_hardware_ceiling_and_geometry() {
        let initial = PullCordConfig::default();
        let mut next = initial;
        next.max_torque_permille += 1;
        assert!(validate_live_config(&initial, &next).is_err());
        next = initial;
        next.drum_radius_m *= 2.0;
        assert!(validate_live_config(&initial, &next).is_err());
        next = initial;
        next.retract_positive = !next.retract_positive;
        assert!(validate_live_config(&initial, &next).is_err());
        next = initial;
        next.torque_near_nm *= 0.9;
        assert!(validate_live_config(&initial, &next).is_ok());
    }
}

#[cfg(test)]
mod chassis_feedback_tests {
    use super::*;

    #[test]
    fn only_a_healthy_running_cord_provides_chassis_feedback() {
        let mut cord = PullCordState::idle(PullCordConfig::default());
        cord.running = true;
        cord.phase = Phase::Running;
        cord.length_m = Some(1.5);
        cord.cord_speed_m_s = Some(-0.05);
        assert!(cord.feedback_ready_for_chassis());

        cord.phase = Phase::Homing;
        assert!(!cord.feedback_ready_for_chassis());
        cord.phase = Phase::Running;
        cord.fault = Some("feedback expired".into());
        assert!(!cord.feedback_ready_for_chassis());
        cord.fault = None;
        cord.running = false;
        assert!(!cord.feedback_ready_for_chassis());
    }
}
