//! Timed MIT experiments and receive-time telemetry. Times are host monotonic
//! receive times, not drive timestamps. Torque feedback is the drive's estimate.
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};
use std::{
    f64::consts::TAU,
    sync::atomic::{AtomicU64, Ordering},
    time::{Instant, SystemTime, UNIX_EPOCH},
};

use super::{check_range, El05ParameterValue, El05Target, El05Version};

const MAX_SAMPLES: usize = 120_000;
const MIT_POSITION_LIMIT: f64 = 12.57;
const MIT_TORQUE_LIMIT: f64 = 6.0;
static NEXT_RUN_ID: AtomicU64 = AtomicU64::new(1);

fn next_run_id() -> u64 {
    // Epoch microseconds remain exactly representable by JavaScript numbers
    // for more than two centuries, and distinguish runs after an app restart.
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64;
    let previous = NEXT_RUN_ID
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |previous| {
            Some(now.max(previous.saturating_add(1)))
        })
        .unwrap();
    now.max(previous.saturating_add(1))
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
pub enum ExperimentKind {
    Record,
    VelocitySteps,
    TorqueRamp,
    StaticFriction,
    StaticTorque,
    PositionSteps,
    SineSweep,
    Impedance,
    Backdrive,
    ThermalHold,
    SlowTriangle,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct PendulumConfig {
    pub mass_kg: f64,
    pub length_m: f64,
    pub arm_mass_kg: f64,
    /// Unwrapped encoder position with the pendulum vertically down.
    pub downward_zero_rad: f64,
    /// False until the user measures or supplies the downward reference.
    pub zero_measured: bool,
    /// Inertia of fixtures and other rotating loads about the motor axis.
    #[serde(default)]
    pub extra_inertia_kg_m2: f64,
}

impl Default for PendulumConfig {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            length_m: 0.18,
            arm_mass_kg: 0.0,
            downward_zero_rad: 0.0,
            zero_measured: false,
            extra_inertia_kg_m2: 0.0,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ExperimentConfig {
    pub kind: ExperimentKind,
    pub duration_s: f64,
    pub velocity_rad_s: f64,
    /// Signed endpoint for the timed torque ramp; sign selects direction.
    pub torque_nm: f64,
    pub kd: f64,
    pub max_temperature_c: f64,
    #[serde(default)]
    pub pendulum: Option<PendulumConfig>,
    #[serde(default = "default_kp")]
    pub kp: f64,
    /// Offset from the measured downward reference, or from the run's start.
    #[serde(default)]
    pub center_angle_rad: f64,
    #[serde(default = "default_position_amplitude")]
    pub position_amplitude_rad: f64,
    #[serde(default = "default_frequency_start")]
    pub frequency_start_hz: f64,
    #[serde(default = "default_frequency_end")]
    pub frequency_end_hz: f64,
    #[serde(default)]
    pub gravity_compensation: bool,
    #[serde(default = "default_disturbance_torque")]
    pub disturbance_torque_nm: f64,
    /// Captured by the session when the run starts; always an unwrapped angle.
    #[serde(default)]
    pub origin_position_rad: f64,
    /// Single-direction static friction calibration ramp, in Nm/s.
    #[serde(default = "default_torque_ramp_rate")]
    pub torque_ramp_rate_nm_s: f64,
    #[serde(default = "default_max_excursion")]
    pub max_excursion_rad: f64,
    #[serde(default = "default_motion_threshold")]
    pub motion_threshold_rad: f64,
    #[serde(default = "default_motion_confirm")]
    pub motion_confirm_s: f64,
    #[serde(default = "default_settle_time")]
    pub settle_time_s: f64,
}

fn default_torque_ramp_rate() -> f64 {
    0.02
}
fn default_max_excursion() -> f64 {
    3.0_f64.to_radians()
}
fn default_motion_threshold() -> f64 {
    0.15_f64.to_radians()
}
fn default_motion_confirm() -> f64 {
    0.04
}
fn default_settle_time() -> f64 {
    0.5
}

fn default_kp() -> f64 {
    8.0
}
fn default_position_amplitude() -> f64 {
    0.25
}
fn default_frequency_start() -> f64 {
    0.1
}
fn default_frequency_end() -> f64 {
    2.0
}
fn default_disturbance_torque() -> f64 {
    0.15
}

impl Default for ExperimentConfig {
    fn default() -> Self {
        Self {
            kind: ExperimentKind::Record,
            duration_s: 20.0,
            velocity_rad_s: 0.3,
            torque_nm: 0.3,
            kd: 0.5,
            max_temperature_c: 80.0,
            pendulum: None,
            kp: default_kp(),
            center_angle_rad: 0.0,
            position_amplitude_rad: default_position_amplitude(),
            frequency_start_hz: default_frequency_start(),
            frequency_end_hz: default_frequency_end(),
            gravity_compensation: false,
            disturbance_torque_nm: default_disturbance_torque(),
            origin_position_rad: 0.0,
            torque_ramp_rate_nm_s: default_torque_ramp_rate(),
            max_excursion_rad: default_max_excursion(),
            motion_threshold_rad: default_motion_threshold(),
            motion_confirm_s: default_motion_confirm(),
            settle_time_s: default_settle_time(),
        }
    }
}

impl ExperimentConfig {
    pub fn validate(&self) -> Result<()> {
        for (value, min, max, label) in [
            (self.duration_s, 2.0, 3600.0, "experiment duration (s)"),
            (self.velocity_rad_s, 0.0, 50.0, "experiment speed (rad/s)"),
            (self.torque_nm, -6.0, 6.0, "experiment torque (Nm)"),
            (self.kp, 0.0, 500.0, "experiment Kp"),
            (self.kd, 0.0, 5.0, "experiment Kd"),
            (
                self.center_angle_rad,
                -1_000_000.0,
                1_000_000.0,
                "center angle (rad)",
            ),
            (
                self.origin_position_rad,
                -1_000_000.0,
                1_000_000.0,
                "origin angle (rad)",
            ),
            (
                self.position_amplitude_rad,
                0.0,
                12.57,
                "position amplitude (rad)",
            ),
            (self.frequency_start_hz, 0.0, 50.0, "start frequency (Hz)"),
            (self.frequency_end_hz, 0.0, 50.0, "end frequency (Hz)"),
            (
                self.disturbance_torque_nm,
                -6.0,
                6.0,
                "disturbance torque (Nm)",
            ),
        ] {
            check_range(value, min, max, label)?;
        }
        if !self.max_temperature_c.is_finite() || self.max_temperature_c > 135.0 {
            bail!(
                "experiment temperature bound must be finite and at most 135 °C (≤0 disables it)"
            );
        }
        if self.kind == ExperimentKind::SineSweep {
            // Commands run at 50 Hz. A 10 Hz bound leaves at least five
            // commands per cycle and stays below the command Nyquist limit.
            check_range(
                self.frequency_start_hz,
                0.001,
                10.0,
                "sweep start frequency (Hz)",
            )?;
            check_range(
                self.frequency_end_hz,
                0.001,
                10.0,
                "sweep end frequency (Hz)",
            )?;
            if TAU
                * self.position_amplitude_rad
                * self.frequency_start_hz.max(self.frequency_end_hz)
                > 50.0
            {
                bail!("sine sweep peak target velocity exceeds the MIT ±50 rad/s range");
            }
        }
        if self.kind == ExperimentKind::StaticFriction {
            for (value, min, max, label) in [
                (
                    self.torque_ramp_rate_nm_s,
                    0.000_001,
                    6.0,
                    "friction ramp rate (Nm/s)",
                ),
                (
                    self.max_excursion_rad,
                    0.3_f64.to_radians(),
                    30.0_f64.to_radians(),
                    "friction angle limit (rad)",
                ),
                (
                    self.motion_threshold_rad,
                    0.05_f64.to_radians(),
                    30.0_f64.to_radians(),
                    "friction motion threshold (rad)",
                ),
                (
                    self.motion_confirm_s,
                    0.01,
                    0.5,
                    "friction motion confirmation (s)",
                ),
                (
                    self.settle_time_s,
                    0.1,
                    10.0,
                    "friction settling interval (s)",
                ),
            ] {
                check_range(value, min, max, label)?;
            }
            if self.motion_threshold_rad >= self.max_excursion_rad {
                bail!("friction motion threshold must be below the angle limit");
            }
            if self.torque_nm.abs() < 0.000_001 || self.kp <= 0.0 || self.kd <= 0.0 {
                bail!("friction calibration requires a nonzero torque limit and positive return Kp/Kd");
            }
        }
        if let Some(pendulum) = &self.pendulum {
            for (value, min, max, label) in [
                (pendulum.mass_kg, 0.0, 20.0, "pendulum end mass (kg)"),
                (pendulum.length_m, 0.001, 2.0, "pendulum length (m)"),
                (pendulum.arm_mass_kg, 0.0, 20.0, "pendulum arm mass (kg)"),
                (
                    pendulum.extra_inertia_kg_m2,
                    0.0,
                    1000.0,
                    "extra load inertia (kg m²)",
                ),
                (
                    pendulum.downward_zero_rad,
                    -1_000_000.0,
                    1_000_000.0,
                    "pendulum downward reference (rad)",
                ),
            ] {
                check_range(value, min, max, label)?;
            }
        }
        Ok(())
    }

    /// Compatibility entry point without live feedback. Position references are
    /// still unwrapped here; the session must use `target_with_feedback` on CAN.
    pub fn target(&self, elapsed_s: f64) -> Option<El05Target> {
        self.waveform(elapsed_s)
    }

    /// Generate the MIT command in the drive's wrapped position coordinates.
    /// The representable position error at any instant is
    /// [-12.57 - wrapped_position, 12.57 - wrapped_position] rad. Clamping to that
    /// interval moves toward an unreachable reference without wrapping its
    /// direction or introducing a discontinuity at the encoder boundary.
    pub fn target_with_feedback(
        &self,
        elapsed_s: f64,
        wrapped_position: f64,
        unwrapped_position: f64,
    ) -> Option<El05Target> {
        let El05Target::Mit {
            position_rad,
            velocity_rad_s,
            mut torque_nm,
            kp,
            kd,
        } = self.waveform(elapsed_s)?
        else {
            unreachable!()
        };
        let position_rad = if kp > 0.0 {
            (wrapped_position + position_rad - unwrapped_position)
                .clamp(-MIT_POSITION_LIMIT, MIT_POSITION_LIMIT)
        } else {
            0.0
        };
        if self.gravity_compensation
            && !matches!(
                self.kind,
                ExperimentKind::Backdrive | ExperimentKind::StaticFriction
            )
        {
            if let Some(pendulum) = self.pendulum.as_ref().filter(|p| p.zero_measured) {
                let theta = unwrapped_position - pendulum.downward_zero_rad;
                torque_nm += 9.80665
                    * pendulum.length_m
                    * (pendulum.mass_kg + 0.5 * pendulum.arm_mass_kg)
                    * theta.sin();
            }
        }
        Some(El05Target::Mit {
            position_rad,
            velocity_rad_s,
            torque_nm: torque_nm.clamp(-MIT_TORQUE_LIMIT, MIT_TORQUE_LIMIT),
            kp,
            kd,
        })
    }

    fn waveform(&self, elapsed_s: f64) -> Option<El05Target> {
        let elapsed_s = elapsed_s.clamp(0.0, self.duration_s);
        let phase = elapsed_s / self.duration_s;
        let center = self
            .pendulum
            .as_ref()
            .filter(|p| p.zero_measured)
            .map_or(self.origin_position_rad, |p| p.downward_zero_rad)
            + self.center_angle_rad;
        let amplitude = self.position_amplitude_rad;
        let mut target = El05Target::Mit {
            position_rad: center,
            velocity_rad_s: 0.0,
            torque_nm: 0.0,
            kp: self.kp,
            kd: self.kd,
        };
        let El05Target::Mit {
            position_rad,
            velocity_rad_s,
            torque_nm,
            kp,
            kd,
        } = &mut target
        else {
            unreachable!()
        };
        match self.kind {
            ExperimentKind::Record => return None,
            ExperimentKind::VelocitySteps => {
                // Rest, positive plateau, rest, negative plateau, rest.
                *velocity_rad_s = if (0.1..0.4).contains(&phase) {
                    self.velocity_rad_s
                } else if (0.5..0.8).contains(&phase) {
                    -self.velocity_rad_s
                } else {
                    0.0
                };
                *position_rad = 0.0;
                *kp = 0.0;
            }
            ExperimentKind::TorqueRamp => {
                *position_rad = 0.0;
                *torque_nm = self.torque_nm * ((phase - 0.1) / 0.9).clamp(0.0, 1.0);
                *kp = 0.0;
                *kd = 0.0;
            }
            // A feedback-driven state machine owns this experiment. The
            // compatibility/config-only entry point must never start a ramp.
            ExperimentKind::StaticFriction => {
                *position_rad = 0.0;
                *kp = 0.0;
                *kd = 0.0;
            }
            ExperimentKind::StaticTorque => {
                // Identical intermediate angles are approached in both
                // directions so static friction and hysteresis can be compared.
                let offsets = [
                    0.0, 0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0, 1.0, 0.5, 0.0,
                ];
                let index = ((phase * offsets.len() as f64) as usize).min(offsets.len() - 1);
                *position_rad += amplitude * offsets[index];
            }
            ExperimentKind::PositionSteps => {
                *position_rad += if (0.1..0.4).contains(&phase) {
                    amplitude
                } else if (0.5..0.8).contains(&phase) {
                    -amplitude
                } else {
                    0.0
                };
            }
            ExperimentKind::SineSweep => {
                let slope = (self.frequency_end_hz - self.frequency_start_hz) / self.duration_s;
                let angle = TAU
                    * (self.frequency_start_hz * elapsed_s + 0.5 * slope * elapsed_s * elapsed_s);
                *position_rad += amplitude * angle.sin();
                *velocity_rad_s =
                    amplitude * TAU * (self.frequency_start_hz + slope * elapsed_s) * angle.cos();
            }
            ExperimentKind::Impedance => {
                *torque_nm = if (0.30..0.35).contains(&phase) {
                    self.disturbance_torque_nm
                } else if (0.65..0.70).contains(&phase) {
                    -self.disturbance_torque_nm
                } else {
                    0.0
                };
            }
            ExperimentKind::Backdrive => {
                *position_rad = 0.0;
                *kp = 0.0;
            }
            ExperimentKind::ThermalHold => {}
            ExperimentKind::SlowTriangle => {
                let (offset, direction) = if phase < 0.25 {
                    (4.0 * phase, 1.0)
                } else if phase < 0.75 {
                    (2.0 - 4.0 * phase, -1.0)
                } else {
                    (4.0 * phase - 4.0, 1.0)
                };
                *position_rad += amplitude * offset;
                *velocity_rad_s = direction * 4.0 * amplitude / self.duration_s;
            }
        }
        Some(target)
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct TelemetrySample {
    pub t_s: f64,
    pub sequence: u64,
    pub can_id: u32,
    pub payload: [u8; 8],
    pub position_rad: f64,
    pub unwrapped_position_rad: f64,
    pub velocity_rad_s: f64,
    pub position_velocity_rad_s: Option<f64>,
    pub torque_nm: f64,
    pub current_a: Option<f64>,
    pub current_age_ms: Option<u64>,
    pub dictionary_velocity_rad_s: Option<f64>,
    pub dictionary_velocity_age_ms: Option<u64>,
    pub temperature_c: f64,
    pub fault_bits: u8,
    pub status_code: u8,
    pub target: Option<El05Target>,
    pub target_sequence: u64,
    pub target_age_ms: Option<u64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct SentCommand {
    pub t_s: f64,
    pub completed_t_s: f64,
    pub sequence: u64,
    pub target: El05Target,
    pub can_id: u32,
    pub payload: [u8; 8],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum StaticFrictionPhase {
    Settling,
    Ramping,
    Returning,
    Complete,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum StaticFrictionOutcome {
    BreakawayDetected,
    AngleLimit,
    TorqueLimit,
    Timeout,
    NotSettled,
    Interrupted,
}

#[derive(Clone, Debug, Serialize)]
pub struct StaticFrictionStatus {
    pub phase: StaticFrictionPhase,
    pub outcome: Option<StaticFrictionOutcome>,
    pub reference_position_rad: Option<f64>,
    pub ramp_started_s: Option<f64>,
    pub last_stationary_sample: Option<TelemetrySample>,
    pub onset_sample: Option<TelemetrySample>,
    pub confirmation_sample: Option<TelemetrySample>,
    pub peak_excursion_rad: f64,
    pub return_started_s: Option<f64>,
    /// None until return finishes; false records its timeout without erasing
    /// valid breakaway evidence.
    pub return_settled: Option<bool>,
}

impl Default for StaticFrictionStatus {
    fn default() -> Self {
        Self {
            phase: StaticFrictionPhase::Settling,
            outcome: None,
            reference_position_rad: None,
            ramp_started_s: None,
            last_stationary_sample: None,
            onset_sample: None,
            confirmation_sample: None,
            peak_excursion_rad: 0.0,
            return_started_s: None,
            return_settled: None,
        }
    }
}

/// Detection runs on every received frame, before recording is decimated. All
/// angles are unwrapped; the mechanical downward reference remains untouched.
#[derive(Default)]
struct StaticFrictionRuntime {
    settling_anchor: Option<(f64, f64)>,
    frozen_torque_nm: Option<f64>,
    return_settled_since: Option<f64>,
}

const FRICTION_REST_VELOCITY: f64 = 0.015;
const FRICTION_RETURN_TIMEOUT_S: f64 = 3.0;

fn friction_ramp_torque(config: &ExperimentConfig, state: &StaticFrictionStatus, t_s: f64) -> f64 {
    config.torque_nm.signum()
        * (config.torque_ramp_rate_nm_s * (t_s - state.ramp_started_s.unwrap_or(t_s)).max(0.0))
            .min(config.torque_nm.abs())
}

fn friction_begin_return(
    state: &mut StaticFrictionStatus,
    runtime: &mut StaticFrictionRuntime,
    outcome: StaticFrictionOutcome,
    t_s: f64,
) {
    state.outcome = Some(outcome);
    state.phase = StaticFrictionPhase::Returning;
    state.return_started_s = Some(t_s);
    state.return_settled = None;
    runtime.frozen_torque_nm = None;
    runtime.return_settled_since = None;
}

fn friction_advance_time(
    config: &ExperimentConfig,
    state: &mut StaticFrictionStatus,
    runtime: &mut StaticFrictionRuntime,
    t_s: f64,
) {
    match state.phase {
        StaticFrictionPhase::Settling if t_s >= config.duration_s => {
            state.outcome = Some(StaticFrictionOutcome::NotSettled);
            state.phase = StaticFrictionPhase::Complete;
        }
        StaticFrictionPhase::Ramping if t_s >= config.duration_s => {
            friction_begin_return(state, runtime, StaticFrictionOutcome::Timeout, t_s);
        }
        StaticFrictionPhase::Returning
            if t_s - state.return_started_s.unwrap_or(t_s) >= FRICTION_RETURN_TIMEOUT_S =>
        {
            state.phase = StaticFrictionPhase::Complete;
            state.return_settled = Some(false);
        }
        _ => {}
    }
}

fn friction_observe(
    config: &ExperimentConfig,
    state: &mut StaticFrictionStatus,
    runtime: &mut StaticFrictionRuntime,
    sample: &TelemetrySample,
) {
    let t = sample.t_s;
    let q = sample.unwrapped_position_rad;
    let v = sample.velocity_rad_s;
    let reference = state
        .reference_position_rad
        .unwrap_or(config.origin_position_rad);
    let excursion = (q - reference).abs();
    state.peak_excursion_rad = state.peak_excursion_rad.max(excursion);
    // Stop applying the measurement torque before reaching the configured
    // boundary. This is an angle guard, not a guarantee of physical braking
    // distance; every later frame can still invalidate a detected breakaway.
    let measuring = matches!(
        state.phase,
        StaticFrictionPhase::Settling | StaticFrictionPhase::Ramping
    );
    let margin = (config.max_excursion_rad * 0.05).min(0.05_f64.to_radians());
    let approaching_limit =
        measuring && excursion + v.abs() * 0.06 >= config.max_excursion_rad - margin;
    if excursion >= config.max_excursion_rad || approaching_limit {
        if measuring {
            friction_begin_return(state, runtime, StaticFrictionOutcome::AngleLimit, t);
        } else {
            state.outcome = Some(StaticFrictionOutcome::AngleLimit);
        }
    }
    friction_advance_time(config, state, runtime, t);
    match state.phase {
        StaticFrictionPhase::Settling => {
            if v.abs() < FRICTION_REST_VELOCITY {
                let (since, anchor) = *runtime.settling_anchor.get_or_insert((t, q));
                if (q - anchor).abs() > config.motion_threshold_rad * 0.5 {
                    runtime.settling_anchor = Some((t, q));
                } else if t - since + 1e-9 >= config.settle_time_s {
                    state.reference_position_rad = Some(q);
                    state.ramp_started_s = Some(t);
                    state.last_stationary_sample = Some(sample.clone());
                    state.phase = StaticFrictionPhase::Ramping;
                }
            } else {
                runtime.settling_anchor = None;
            }
        }
        StaticFrictionPhase::Ramping => {
            let direction = config.torque_nm.signum();
            let moving =
                (q - reference) * direction >= config.motion_threshold_rad && v * direction > 0.0;
            if moving {
                if state.onset_sample.is_none() {
                    state.onset_sample = Some(sample.clone());
                    // Freeze at the command actually associated with the
                    // candidate frame, avoiding extra ramp torque while the
                    // movement confirmation window elapses.
                    runtime.frozen_torque_nm = Some(
                        match sample.target {
                            Some(El05Target::Mit { torque_nm, .. }) => torque_nm,
                            _ => friction_ramp_torque(config, state, t),
                        }
                        .clamp(-config.torque_nm.abs(), config.torque_nm.abs()),
                    );
                }
                if t - state.onset_sample.as_ref().unwrap().t_s + 1e-9 >= config.motion_confirm_s {
                    state.confirmation_sample = Some(sample.clone());
                    friction_begin_return(
                        state,
                        runtime,
                        StaticFrictionOutcome::BreakawayDetected,
                        t,
                    );
                }
            } else {
                state.onset_sample = None;
                runtime.frozen_torque_nm = None;
                if v.abs() < FRICTION_REST_VELOCITY && excursion < config.motion_threshold_rad * 0.5
                {
                    state.last_stationary_sample = Some(sample.clone());
                }
                if friction_ramp_torque(config, state, t).abs() >= config.torque_nm.abs() {
                    friction_begin_return(state, runtime, StaticFrictionOutcome::TorqueLimit, t);
                }
            }
        }
        StaticFrictionPhase::Returning => {
            if excursion < config.motion_threshold_rad * 0.5 && v.abs() < FRICTION_REST_VELOCITY {
                let since = *runtime.return_settled_since.get_or_insert(t);
                if t - since + 1e-9 >= 0.2 {
                    state.phase = StaticFrictionPhase::Complete;
                    state.return_settled = Some(true);
                }
            } else {
                runtime.return_settled_since = None;
            }
        }
        StaticFrictionPhase::Complete => {}
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct ExperimentStatus {
    #[serde(default)]
    pub run_id: u64,
    pub active: bool,
    pub config: Option<ExperimentConfig>,
    pub samples: usize,
    pub elapsed_s: f64,
    pub reason: Option<String>,
    pub dropped_frames: u64,
    /// Duration-dependent minimum interval for storing unmodified receive
    /// frames and sent commands. Retention is bounded while covering the run.
    #[serde(default)]
    pub recording_interval_s: f64,
    pub static_friction: Option<StaticFrictionStatus>,
}

#[derive(Clone, Debug, Serialize)]
pub struct ExperimentData {
    /// Multiplier already applied to packed feedback velocity. CAN payloads
    /// remain unchanged, allowing old exports to be normalized on import.
    pub feedback_velocity_sign: i8,
    pub status: ExperimentStatus,
    pub motor_id: Option<u8>,
    pub uid: Option<String>,
    pub samples: Vec<TelemetrySample>,
    pub commands: Vec<SentCommand>,
    pub parameters: Vec<El05ParameterValue>,
    pub version: Option<El05Version>,
}

#[derive(Default)]
pub struct Experiment {
    pub status: ExperimentStatus,
    pub samples: Vec<TelemetrySample>,
    pub motor_id: Option<u8>,
    pub uid: Option<String>,
    pub commands: Vec<SentCommand>,
    pub parameters: Vec<El05ParameterValue>,
    pub version: Option<El05Version>,
    start: Option<Instant>,
    static_friction_runtime: StaticFrictionRuntime,
}

impl Experiment {
    pub fn begin(&mut self, config: ExperimentConfig, motor_id: u8, uid: Option<String>) {
        *self = Self {
            status: ExperimentStatus {
                run_id: next_run_id(),
                active: true,
                recording_interval_s: config.duration_s / (MAX_SAMPLES - 1) as f64,
                static_friction: (config.kind == ExperimentKind::StaticFriction)
                    .then(StaticFrictionStatus::default),
                config: Some(config),
                ..Default::default()
            },
            start: Some(Instant::now()),
            motor_id: Some(motor_id),
            uid,
            ..Default::default()
        };
    }

    pub fn elapsed_s(&self) -> f64 {
        self.start
            .map_or(0.0, |start| start.elapsed().as_secs_f64())
    }

    pub fn finish(&mut self, reason: impl Into<String>) {
        if self.status.active {
            let reason = reason.into();
            if let Some(state) = self.status.static_friction.as_mut() {
                let completed_reason = format!("static friction calibration: {:?}", state.outcome);
                if state.phase != StaticFrictionPhase::Complete || reason != completed_reason {
                    state.phase = StaticFrictionPhase::Complete;
                    state.outcome = Some(StaticFrictionOutcome::Interrupted);
                }
            }
            self.status.elapsed_s = self.elapsed_s();
            self.status.active = false;
            self.status.reason = Some(reason);
        }
    }

    pub fn push(&mut self, mut sample: TelemetrySample, received: Instant) {
        if self.status.active {
            sample.t_s = received.duration_since(self.start.unwrap()).as_secs_f64();
            if let (Some(config), Some(state)) =
                (&self.status.config, &mut self.status.static_friction)
            {
                friction_observe(config, state, &mut self.static_friction_runtime, &sample);
            }
            if self.samples.len() == MAX_SAMPLES {
                // Preserve the latest observation even if a high-rate source
                // exhausts the bound (for example due to clock granularity).
                *self.samples.last_mut().unwrap() = sample;
                return;
            }
            if self.samples.last().is_some_and(|previous| {
                sample.t_s - previous.t_s + 1e-9 < self.status.recording_interval_s
            }) {
                return;
            }
            self.samples.push(sample);
            self.status.samples = self.samples.len();
        }
    }

    /// Stateful experiments use receive-time detection and host-side bounded
    /// torque control. Existing waveforms keep their original coordinate and
    /// gravity compensation behavior.
    pub fn runtime_target(
        &mut self,
        wrapped_position: f64,
        unwrapped_position: f64,
        velocity_rad_s: f64,
    ) -> Option<El05Target> {
        self.runtime_target_at(
            self.elapsed_s(),
            wrapped_position,
            unwrapped_position,
            velocity_rad_s,
        )
    }

    fn runtime_target_at(
        &mut self,
        elapsed_s: f64,
        wrapped_position: f64,
        unwrapped_position: f64,
        velocity_rad_s: f64,
    ) -> Option<El05Target> {
        let config = self.status.config.as_ref()?;
        let Some(state) = self.status.static_friction.as_mut() else {
            return config.target_with_feedback(elapsed_s, wrapped_position, unwrapped_position);
        };
        friction_advance_time(config, state, &mut self.static_friction_runtime, elapsed_s);
        let torque_nm = match state.phase {
            StaticFrictionPhase::Settling | StaticFrictionPhase::Complete => 0.0,
            StaticFrictionPhase::Ramping => self
                .static_friction_runtime
                .frozen_torque_nm
                .unwrap_or_else(|| friction_ramp_torque(config, state, elapsed_s)),
            StaticFrictionPhase::Returning => {
                let reference = state
                    .reference_position_rad
                    .unwrap_or(config.origin_position_rad);
                (-config.kp * (unwrapped_position - reference) - config.kd * velocity_rad_s)
                    .clamp(-config.torque_nm.abs(), config.torque_nm.abs())
            }
        };
        Some(El05Target::Mit {
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_nm,
            kp: 0.0,
            kd: 0.0,
        })
    }

    pub fn record_command(
        &mut self,
        started: Instant,
        completed: Instant,
        sequence: u64,
        target: El05Target,
        can_id: u32,
        payload: [u8; 8],
    ) {
        if self.status.active {
            let start = self.start.unwrap();
            let command = SentCommand {
                t_s: started.duration_since(start).as_secs_f64(),
                completed_t_s: completed.duration_since(start).as_secs_f64(),
                sequence,
                target,
                can_id,
                payload,
            };
            if self.commands.len() == MAX_SAMPLES {
                *self.commands.last_mut().unwrap() = command;
                return;
            }
            if self.commands.last().is_some_and(|previous| {
                command.t_s - previous.t_s + 1e-9 < self.status.recording_interval_s
            }) {
                return;
            }
            self.commands.push(command);
        }
    }

    /// Checked before every command, also for passive recording with the drive
    /// disabled. The service loop sends Stop when any condition ends a run.
    pub fn end_reason(&mut self, temperature: f64) -> Option<String> {
        if !self.status.active {
            return None;
        }
        if !temperature.is_finite() {
            return Some("invalid telemetry".into());
        }
        let elapsed = self.elapsed_s();
        self.status.elapsed_s = elapsed;
        let config = self.status.config.as_ref()?;
        if config.max_temperature_c > 0.0 && temperature >= config.max_temperature_c {
            return Some("temperature bound reached".into());
        }
        if let Some(state) = &mut self.status.static_friction {
            friction_advance_time(config, state, &mut self.static_friction_runtime, elapsed);
            return (state.phase == StaticFrictionPhase::Complete)
                .then(|| format!("static friction calibration: {:?}", state.outcome));
        }
        (elapsed >= config.duration_s).then(|| "duration completed".into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config(kind: ExperimentKind) -> ExperimentConfig {
        ExperimentConfig {
            kind,
            duration_s: 10.0,
            velocity_rad_s: 0.3,
            torque_nm: -0.3,
            kd: 0.5,
            max_temperature_c: 60.0,
            pendulum: None,
            ..Default::default()
        }
    }

    #[test]
    fn pendulum_metadata_is_optional_validated_and_frozen_with_the_run() {
        let mut config = config(ExperimentKind::Record);
        // Previous callers may omit the new field.
        let mut legacy = serde_json::to_value(&config).unwrap();
        legacy.as_object_mut().unwrap().remove("pendulum");
        let legacy: ExperimentConfig = serde_json::from_value(legacy).unwrap();
        assert!(legacy.pendulum.is_none());
        config.pendulum = Some(PendulumConfig {
            mass_kg: 1.0,
            length_m: 0.18,
            arm_mass_kg: 0.2,
            downward_zero_rad: 30.0,
            zero_measured: true,
            ..Default::default()
        });
        assert!(config.validate().is_ok());
        let mut run = Experiment::default();
        run.begin(config.clone(), 1, None);
        config.pendulum.as_mut().unwrap().downward_zero_rad = 40.0;
        let frozen = run
            .status
            .config
            .as_ref()
            .unwrap()
            .pendulum
            .as_ref()
            .unwrap();
        assert_eq!(frozen.downward_zero_rad, 30.0);
        assert!(frozen.zero_measured);
        for invalid in [f64::NAN, f64::INFINITY, -1.0, 21.0] {
            config.pendulum.as_mut().unwrap().mass_kg = invalid;
            assert!(config.validate().is_err());
        }
        config.pendulum.as_mut().unwrap().mass_kg = 1.0;
        config.pendulum.as_mut().unwrap().length_m = 0.0;
        assert!(config.validate().is_err());
        config.pendulum.as_mut().unwrap().length_m = 0.18;
        config.pendulum.as_mut().unwrap().arm_mass_kg = -0.01;
        assert!(config.validate().is_err());
        config.pendulum.as_mut().unwrap().arm_mass_kg = 0.0;
        config.pendulum.as_mut().unwrap().downward_zero_rad = f64::NAN;
        assert!(config.validate().is_err());
    }

    #[test]
    fn phases_cover_both_directions_and_end_at_rest() {
        let config = config(ExperimentKind::VelocitySteps);
        for (t, expected) in [(0.0, 0.0), (2.0, 0.3), (4.5, 0.0), (6.0, -0.3), (9.0, 0.0)] {
            let Some(El05Target::Mit {
                velocity_rad_s,
                kp,
                torque_nm,
                ..
            }) = config.target(t)
            else {
                panic!()
            };
            assert_eq!((velocity_rad_s, kp, torque_nm), (expected, 0.0, 0.0));
        }
    }

    #[test]
    fn ramp_is_signed_and_has_no_hidden_pd_torque() {
        let config = config(ExperimentKind::TorqueRamp);
        for (t, expected) in [(0.0, 0.0), (1.0, 0.0), (5.5, -0.15), (10.0, -0.3)] {
            let Some(El05Target::Mit {
                torque_nm,
                velocity_rad_s,
                kp,
                kd,
                ..
            }) = config.target(t)
            else {
                panic!()
            };
            assert!((torque_nm - expected).abs() < 1e-12);
            assert_eq!((velocity_rad_s, kp, kd), (0.0, 0.0, 0.0));
        }
        assert!(self::config(ExperimentKind::Record).target(1.0).is_none());
    }

    #[test]
    fn invalid_configuration_and_optional_temperature_limits_are_checked() {
        let mut config = config(ExperimentKind::VelocitySteps);
        config.kd = 0.0;
        assert!(config.validate().is_ok());
        config.kd = 0.5;
        config.duration_s = f64::NAN;
        assert!(config.validate().is_err());
        config.duration_s = 10.0;
        let mut run = Experiment::default();
        run.begin(config, 1, None);
        assert!(run.end_reason(25.0).is_none());
        assert_eq!(
            run.end_reason(60.0).as_deref(),
            Some("temperature bound reached")
        );
        run.status.config.as_mut().unwrap().max_temperature_c = 0.0;
        assert!(run.end_reason(130.0).is_none());
    }

    #[test]
    fn torque_ramp_runs_to_duration_and_preserves_the_original_stop_reason() {
        let mut run = Experiment::default();
        run.begin(config(ExperimentKind::TorqueRamp), 1, None);
        run.start = Some(Instant::now() - std::time::Duration::from_secs(2));
        assert!(run.end_reason(25.0).is_none());
        run.start = Some(Instant::now() - std::time::Duration::from_secs(11));
        let reason = run.end_reason(25.0).unwrap();
        assert_eq!(reason, "duration completed");
        run.finish(reason.clone());
        run.finish("operator stop");
        assert_eq!(run.status.reason.as_ref(), Some(&reason));
        run.begin(config(ExperimentKind::Record), 1, None);
        assert_eq!(
            run.end_reason(f64::NAN).as_deref(),
            Some("invalid telemetry")
        );
        run.start = Some(Instant::now() - std::time::Duration::from_secs(11));
        assert_eq!(run.end_reason(25.0).as_deref(), Some("duration completed"));
    }

    fn mit(target: Option<El05Target>) -> (f64, f64, f64, f64, f64) {
        let Some(El05Target::Mit {
            position_rad,
            velocity_rad_s,
            torque_nm,
            kp,
            kd,
        }) = target
        else {
            panic!("expected MIT command")
        };
        (position_rad, velocity_rad_s, torque_nm, kp, kd)
    }

    #[test]
    fn legacy_json_defaults_new_fields_without_changing_existing_runs() {
        let legacy = serde_json::json!({
            "kind":"TorqueRamp", "duration_s":10.0, "velocity_rad_s":0.3,
            "torque_nm":-0.3, "kd":0.5, "max_temperature_c":60.0,
            "pendulum": {"mass_kg":1.0,"length_m":0.2,"arm_mass_kg":0.1,
                "downward_zero_rad":12.0,"zero_measured":true}
        });
        let config: ExperimentConfig = serde_json::from_value(legacy).unwrap();
        assert_eq!(config.kp, 8.0);
        assert_eq!(config.position_amplitude_rad, 0.25);
        assert_eq!(
            (config.frequency_start_hz, config.frequency_end_hz),
            (0.1, 2.0)
        );
        assert_eq!(config.disturbance_torque_nm, 0.15);
        assert_eq!(config.origin_position_rad, 0.0);
        assert!(!config.gravity_compensation);
        assert_eq!(config.pendulum.as_ref().unwrap().extra_inertia_kg_m2, 0.0);
        assert!(config.validate().is_ok());
        assert_eq!(mit(config.target(10.0)), (0.0, 0.0, -0.3, 0.0, 0.0));
    }

    #[test]
    fn static_plateaus_revisit_angles_and_position_steps_have_rest_periods() {
        let mut config = config(ExperimentKind::StaticTorque);
        config.duration_s = 12.0;
        config.origin_position_rad = 30.0;
        config.center_angle_rad = 0.2;
        config.position_amplitude_rad = 0.4;
        let offsets = [
            0.0, 0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0, 1.0, 0.5, 0.0,
        ];
        for (index, offset) in offsets.into_iter().enumerate() {
            let (position, velocity, torque, kp, kd) = mit(config.target(index as f64 + 0.5));
            assert!((position - (30.2 + 0.4 * offset)).abs() < 1e-12);
            assert_eq!((velocity, torque, kp, kd), (0.0, 0.0, config.kp, config.kd));
        }
        config.kind = ExperimentKind::PositionSteps;
        config.duration_s = 10.0;
        for (time, offset) in [
            (0.0, 0.0),
            (1.0, 0.4),
            (3.99, 0.4),
            (4.0, 0.0),
            (5.0, -0.4),
            (7.99, -0.4),
            (8.0, 0.0),
            (10.0, 0.0),
        ] {
            assert!((mit(config.target(time)).0 - (30.2 + offset)).abs() < 1e-12);
        }
    }

    #[test]
    fn sweep_velocity_is_the_derivative_of_the_linear_chirp() {
        let mut config = config(ExperimentKind::SineSweep);
        config.position_amplitude_rad = 0.2;
        config.frequency_start_hz = 0.3;
        config.frequency_end_hz = 2.4;
        for time in [0.01, 0.6, 2.5, 5.0, 9.99] {
            let (position, velocity, _, _, _) = mit(config.target(time));
            let slope = (2.4 - 0.3) / config.duration_s;
            let phase = TAU * (0.3 * time + 0.5 * slope * time * time);
            assert!((position - 0.2 * phase.sin()).abs() < 1e-12);
            let h = 1e-6;
            let numerical_velocity =
                (mit(config.target(time + h)).0 - mit(config.target(time - h)).0) / (2.0 * h);
            assert!((velocity - numerical_velocity).abs() < 1e-7);
        }
        assert!(config.validate().is_ok());
        config.frequency_end_hz = 25.0;
        assert!(config.validate().is_err());
        config.frequency_end_hz = 10.0;
        config.position_amplitude_rad = 1.0;
        assert!(config.validate().is_err());
        config.kind = ExperimentKind::ThermalHold;
        // Sweep-only restrictions do not block other tests.
        assert!(config.validate().is_ok());
    }

    #[test]
    fn impedance_pulses_triangle_and_backdrive_follow_the_documented_profiles() {
        let mut config = config(ExperimentKind::Impedance);
        for (time, torque) in [
            (0.0, 0.0),
            (3.0, 0.15),
            (3.49, 0.15),
            (3.5, 0.0),
            (6.5, -0.15),
            (6.99, -0.15),
            (7.0, 0.0),
        ] {
            assert_eq!(mit(config.target(time)).2, torque);
        }
        config.kind = ExperimentKind::SlowTriangle;
        config.position_amplitude_rad = 0.4;
        for (time, position, velocity) in [
            (0.0, 0.0, 0.16),
            (2.5, 0.4, -0.16),
            (5.0, 0.0, -0.16),
            (7.5, -0.4, 0.16),
            (10.0, 0.0, 0.16),
        ] {
            let (actual_position, actual_velocity, _, _, _) = mit(config.target(time));
            assert!((actual_position - position).abs() < 1e-12);
            assert!((actual_velocity - velocity).abs() < 1e-12);
        }
        config.kind = ExperimentKind::Backdrive;
        assert_eq!(mit(config.target(5.0)), (0.0, 0.0, 0.0, 0.0, config.kd));
    }

    #[test]
    fn live_targets_use_unwrapped_errors_and_respect_the_mit_position_window() {
        let mut config = config(ExperimentKind::ThermalHold);
        config.origin_position_rad = 37.69;
        let before_wrap = mit(config.target_with_feedback(1.0, 12.56, 37.7));
        assert!((before_wrap.0 - 12.55).abs() < 1e-12);
        config.origin_position_rad = 37.72;
        assert_eq!(mit(config.target_with_feedback(1.0, 12.56, 37.7)).0, 12.57);
        let after_wrap = mit(config.target_with_feedback(1.0, -12.56, 37.72));
        assert!((after_wrap.0 + 12.56).abs() < 1e-12);
        config.origin_position_rad = -100.0;
        assert_eq!(
            mit(config.target_with_feedback(1.0, -12.56, 37.72)).0,
            -12.57
        );

        config.pendulum = Some(PendulumConfig {
            downward_zero_rad: 37.0,
            zero_measured: true,
            ..Default::default()
        });
        config.center_angle_rad = 0.3;
        assert!((mit(config.target(1.0)).0 - 37.3).abs() < 1e-12);
        config.pendulum.as_mut().unwrap().zero_measured = false;
        assert!((mit(config.target(1.0)).0 + 99.7).abs() < 1e-12);
    }

    #[test]
    fn gravity_uses_measured_angle_not_the_target_and_backdrive_has_zero_feedforward() {
        let mut config = config(ExperimentKind::ThermalHold);
        config.gravity_compensation = true;
        config.center_angle_rad = 1.0;
        config.pendulum = Some(PendulumConfig {
            mass_kg: 1.0,
            length_m: 0.2,
            arm_mass_kg: 0.2,
            downward_zero_rad: 20.0,
            zero_measured: true,
            ..Default::default()
        });
        let expected = 9.80665 * 0.2 * 1.1 * 0.3_f64.sin();
        assert!((mit(config.target_with_feedback(1.0, -4.84, 20.3)).2 - expected).abs() < 1e-12);
        config.pendulum.as_mut().unwrap().zero_measured = false;
        assert_eq!(mit(config.target_with_feedback(1.0, -4.84, 20.3)).2, 0.0);
        config.pendulum.as_mut().unwrap().zero_measured = true;
        config.pendulum.as_mut().unwrap().mass_kg = 20.0;
        assert_eq!(
            mit(config.target_with_feedback(1.0, 0.0, 20.0 + std::f64::consts::FRAC_PI_2)).2,
            6.0
        );
        config.kind = ExperimentKind::Backdrive;
        assert_eq!(
            mit(config.target_with_feedback(1.0, -4.84, 20.3)),
            (0.0, 0.0, 0.0, 0.0, config.kd)
        );
    }

    fn sample(sequence: u64) -> TelemetrySample {
        TelemetrySample {
            t_s: 0.0,
            sequence,
            can_id: 1,
            payload: [42; 8],
            position_rad: 0.0,
            unwrapped_position_rad: 0.0,
            velocity_rad_s: 0.0,
            position_velocity_rad_s: None,
            torque_nm: 0.0,
            current_a: None,
            current_age_ms: None,
            dictionary_velocity_rad_s: None,
            dictionary_velocity_age_ms: None,
            temperature_c: 25.0,
            fault_bits: 0,
            status_code: 0,
            target: None,
            target_sequence: 0,
            target_age_ms: None,
        }
    }

    #[test]
    fn hour_long_runs_keep_the_full_time_span_with_bounded_storage() {
        let config = ExperimentConfig {
            duration_s: 3600.0,
            ..Default::default()
        };
        assert!(config.validate().is_ok());
        let mut run = Experiment::default();
        run.begin(config, 1, None);
        let first_run_id = run.status.run_id;
        let start = run.start.unwrap();
        let target = El05Target::Mit {
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_nm: 0.0,
            kp: 0.0,
            kd: 0.0,
        };
        for sequence in 0..=360_000_u64 {
            let time = start + std::time::Duration::from_millis(sequence * 10);
            run.push(sample(sequence), time);
            run.record_command(time, time, sequence, target, 1, [42; 8]);
        }
        assert!(run.samples.len() <= MAX_SAMPLES);
        assert!(run.samples.len() >= 80_000);
        assert!(run.commands.len() <= MAX_SAMPLES);
        assert!(run.samples.first().unwrap().t_s < 0.01);
        assert!(run.samples.last().unwrap().t_s >= 3599.95);
        assert!(run.commands.last().unwrap().t_s >= 3599.95);
        assert_eq!(run.samples.last().unwrap().payload, [42; 8]);
        assert_eq!(run.status.samples, run.samples.len());
        assert!(run.end_reason(25.0).is_none());
        run.begin(ExperimentConfig::default(), 1, None);
        assert_ne!(run.status.run_id, first_run_id);
    }

    #[test]
    fn full_capacity_keeps_latest_observation_instead_of_ending_a_run() {
        let mut run = Experiment::default();
        run.begin(ExperimentConfig::default(), 1, None);
        run.samples.resize(MAX_SAMPLES, sample(0));
        run.status.samples = MAX_SAMPLES;
        let time = run.start.unwrap() + std::time::Duration::from_secs(19);
        run.push(sample(999), time);
        assert_eq!(run.samples.len(), MAX_SAMPLES);
        assert_eq!(run.samples.last().unwrap().sequence, 999);
        assert_eq!(run.samples.last().unwrap().t_s, 19.0);
        assert!(run.end_reason(25.0).is_none());
    }

    fn friction_run(direction: f64) -> Experiment {
        let mut run = Experiment::default();
        run.begin(
            ExperimentConfig {
                kind: ExperimentKind::StaticFriction,
                duration_s: 45.0,
                torque_nm: direction * 0.6,
                kd: 0.8,
                ..Default::default()
            },
            1,
            None,
        );
        run
    }

    fn observe(run: &mut Experiment, t_s: f64, position: f64, velocity: f64) {
        let mut sample = sample((t_s * 10_000.0).round() as u64);
        sample.unwrapped_position_rad = position;
        sample.position_rad = (position + 12.57).rem_euclid(25.14) - 12.57;
        sample.velocity_rad_s = velocity;
        sample.target = run.runtime_target_at(t_s, sample.position_rad, position, velocity);
        run.push(
            sample,
            run.start.unwrap() + std::time::Duration::from_secs_f64(t_s),
        );
    }

    fn friction_state(run: &Experiment) -> &StaticFrictionStatus {
        run.status.static_friction.as_ref().unwrap()
    }

    fn settle(run: &mut Experiment) {
        let origin = run.status.config.as_ref().unwrap().origin_position_rad;
        observe(run, 0.0, origin, 0.0);
        observe(run, 0.5, origin, 0.0);
        assert_eq!(friction_state(run).phase, StaticFrictionPhase::Ramping);
    }

    #[test]
    fn friction_waits_for_continuous_rest_and_preserves_the_downward_reference() {
        let mut run = friction_run(1.0);
        let config = run.status.config.as_mut().unwrap();
        config.origin_position_rad = 37.69;
        config.gravity_compensation = true;
        config.pendulum = Some(PendulumConfig {
            downward_zero_rad: 12.0,
            zero_measured: true,
            mass_kg: 1.0,
            ..Default::default()
        });
        observe(&mut run, 0.0, 37.69, 0.0);
        observe(&mut run, 0.3, 37.69, 0.02);
        observe(&mut run, 0.4, 37.69, 0.0);
        observe(&mut run, 0.8, 37.6902, 0.0);
        assert_eq!(friction_state(&run).phase, StaticFrictionPhase::Settling);
        assert_eq!(
            mit(run.runtime_target_at(0.85, 12.55, 37.6902, 0.0)),
            (0.0, 0.0, 0.0, 0.0, 0.0)
        );
        observe(&mut run, 0.9, 37.6901, 0.0);
        assert_eq!(friction_state(&run).reference_position_rad, Some(37.6901));
        assert_eq!(friction_state(&run).ramp_started_s, Some(0.9));
        assert!((mit(run.runtime_target_at(1.9, 12.55, 37.6901, 0.0)).2 - 0.02).abs() < 1e-12);
        let pendulum = run
            .status
            .config
            .as_ref()
            .unwrap()
            .pendulum
            .as_ref()
            .unwrap();
        assert_eq!(pendulum.downward_zero_rad, 12.0);
        assert!(pendulum.zero_measured);
    }

    #[test]
    fn friction_freezes_each_direction_at_onset_then_immediately_removes_the_ramp() {
        for direction in [1.0, -1.0] {
            let mut run = friction_run(direction);
            settle(&mut run);
            observe(&mut run, 1.9, direction * 0.001, 0.0);
            observe(&mut run, 2.0, direction * 0.003, direction * 0.02);
            let onset = friction_state(&run).onset_sample.as_ref().unwrap();
            assert_eq!(onset.t_s, 2.0);
            assert_eq!(
                friction_state(&run)
                    .last_stationary_sample
                    .as_ref()
                    .unwrap()
                    .t_s,
                1.9
            );
            assert_eq!(friction_state(&run).outcome, None);
            assert!(
                (mit(run.runtime_target_at(
                    2.03,
                    0.003 * direction,
                    0.003 * direction,
                    0.02 * direction
                ))
                .2 - 0.03 * direction)
                    .abs()
                    < 1e-12
            );
            observe(&mut run, 2.04, direction * 0.0038, direction * 0.02);
            assert_eq!(friction_state(&run).phase, StaticFrictionPhase::Returning);
            assert_eq!(
                friction_state(&run).outcome,
                Some(StaticFrictionOutcome::BreakawayDetected)
            );
            assert_eq!(
                friction_state(&run)
                    .confirmation_sample
                    .as_ref()
                    .unwrap()
                    .t_s,
                2.04
            );
            let (_, velocity, torque, kp, kd) = mit(run.runtime_target_at(
                2.04,
                0.0038 * direction,
                0.0038 * direction,
                0.02 * direction,
            ));
            assert!(torque * direction < 0.0);
            assert_eq!((velocity, kp, kd), (0.0, 0.0, 0.0));
            observe(&mut run, 2.2, 0.0001 * direction, 0.0);
            observe(&mut run, 2.4, 0.0001 * direction, 0.0);
            assert_eq!(friction_state(&run).phase, StaticFrictionPhase::Complete);
            assert_eq!(friction_state(&run).return_settled, Some(true));
            assert_eq!(mit(run.runtime_target_at(2.4, 0.0, 0.0, 0.0)).2, 0.0);
            let reason = run.end_reason(25.0).unwrap();
            run.finish(reason);
            assert!(!run.status.active);
            assert_eq!(
                friction_state(&run).outcome,
                Some(StaticFrictionOutcome::BreakawayDetected)
            );
        }
    }

    #[test]
    fn friction_noise_and_unconfirmed_movement_do_not_become_a_measurement() {
        let mut run = friction_run(1.0);
        settle(&mut run);
        for index in 0..100 {
            observe(
                &mut run,
                0.6 + f64::from(index) * 0.01,
                if index % 2 == 0 { 0.0004 } else { -0.0004 },
                0.01,
            );
        }
        assert!(friction_state(&run).onset_sample.is_none());
        observe(&mut run, 1.7, 0.003, 0.01);
        assert!(friction_state(&run).onset_sample.is_some());
        observe(&mut run, 1.72, 0.001, -0.01);
        assert!(friction_state(&run).onset_sample.is_none());
        assert!(friction_state(&run).confirmation_sample.is_none());
        assert_eq!(friction_state(&run).outcome, None);
        assert_eq!(friction_state(&run).phase, StaticFrictionPhase::Ramping);
        assert!((mit(run.runtime_target_at(2.0, 0.001, 0.001, 0.0)).2 - 0.03).abs() < 1e-12);
    }

    #[test]
    fn friction_last_stationary_evidence_rejects_moving_and_displaced_samples() {
        let mut run = friction_run(1.0);
        settle(&mut run);
        assert_eq!(
            friction_state(&run)
                .last_stationary_sample
                .as_ref()
                .unwrap()
                .t_s,
            0.5
        );
        // Below the onset threshold but already moving forward.
        observe(&mut run, 0.6, 0.0008, 0.03);
        // Motion in the opposite direction cannot become stationary evidence.
        observe(&mut run, 0.7, -0.003, -0.03);
        // A stopped sample away from the reference is not the initial rest.
        observe(&mut run, 0.8, 0.002, 0.0);
        assert_eq!(
            friction_state(&run)
                .last_stationary_sample
                .as_ref()
                .unwrap()
                .t_s,
            0.5
        );
        observe(&mut run, 0.9, 0.0005, 0.0);
        observe(&mut run, 1.0, 0.003, 0.02);
        assert_eq!(
            friction_state(&run)
                .last_stationary_sample
                .as_ref()
                .unwrap()
                .t_s,
            0.9
        );
        assert_eq!(friction_state(&run).onset_sample.as_ref().unwrap().t_s, 1.0);
    }

    #[test]
    fn friction_angle_guard_has_priority_and_also_protects_initial_settling() {
        for before_settling in [true, false] {
            let mut run = friction_run(1.0);
            run.status.config.as_mut().unwrap().origin_position_rad = 37.69;
            if !before_settling {
                settle(&mut run);
                observe(&mut run, 1.0, 37.693, 0.01);
            }
            // This would confirm the previous candidate, but its predicted
            // excursion reaches the limit first.
            observe(&mut run, 1.04, 37.70, 0.8);
            assert_eq!(
                friction_state(&run).outcome,
                Some(StaticFrictionOutcome::AngleLimit)
            );
            assert_eq!(friction_state(&run).phase, StaticFrictionPhase::Returning);
            assert!(friction_state(&run).confirmation_sample.is_none());
            assert!(mit(run.runtime_target_at(1.04, 12.56, 37.70, 0.8)).2 < 0.0);
        }
    }

    #[test]
    fn friction_actual_angle_overshoot_invalidates_a_confirmed_breakaway() {
        let mut run = friction_run(1.0);
        settle(&mut run);
        observe(&mut run, 1.0, 0.003, 0.02);
        observe(&mut run, 1.04, 0.004, 0.02);
        assert_eq!(
            friction_state(&run).outcome,
            Some(StaticFrictionOutcome::BreakawayDetected)
        );
        observe(&mut run, 1.1, 0.06, 0.2);
        assert_eq!(
            friction_state(&run).outcome,
            Some(StaticFrictionOutcome::AngleLimit)
        );
        assert!(friction_state(&run).confirmation_sample.is_some());
        assert_eq!(friction_state(&run).peak_excursion_rad, 0.06);
    }

    #[test]
    fn friction_torque_and_time_limits_never_report_success() {
        let mut torque_limited = friction_run(1.0);
        torque_limited.status.config.as_mut().unwrap().torque_nm = 0.01;
        settle(&mut torque_limited);
        observe(&mut torque_limited, 1.0, 0.0, 0.0);
        assert_eq!(
            friction_state(&torque_limited).outcome,
            Some(StaticFrictionOutcome::TorqueLimit)
        );
        assert_eq!(
            mit(torque_limited.runtime_target_at(1.0, 0.0, 0.0, 0.0)).2,
            0.0
        );

        let mut timed_out = friction_run(1.0);
        timed_out.status.config.as_mut().unwrap().duration_s = 2.0;
        settle(&mut timed_out);
        observe(&mut timed_out, 2.0, 0.0, 0.0);
        assert_eq!(
            friction_state(&timed_out).outcome,
            Some(StaticFrictionOutcome::Timeout)
        );
        assert_eq!(mit(timed_out.runtime_target_at(2.0, 0.0, 0.0, 0.0)).2, 0.0);

        let mut not_settled = friction_run(1.0);
        not_settled.status.config.as_mut().unwrap().duration_s = 2.0;
        observe(&mut not_settled, 0.0, 0.0, 0.02);
        observe(&mut not_settled, 2.0, 0.0, 0.02);
        assert_eq!(
            friction_state(&not_settled).phase,
            StaticFrictionPhase::Complete
        );
        assert_eq!(
            friction_state(&not_settled).outcome,
            Some(StaticFrictionOutcome::NotSettled)
        );
    }

    #[test]
    fn friction_return_is_host_pd_torque_capped_and_has_a_finite_timeout() {
        let mut run = friction_run(1.0);
        settle(&mut run);
        observe(&mut run, 1.0, 0.003, 0.02);
        observe(&mut run, 1.04, 0.004, 0.02);
        assert_eq!(
            mit(run.runtime_target_at(1.1, 12.56, 100.0, 50.0)),
            (0.0, 0.0, -0.6, 0.0, 0.0)
        );
        assert_eq!(
            mit(run.runtime_target_at(1.1, -12.56, -100.0, -50.0)),
            (0.0, 0.0, 0.6, 0.0, 0.0)
        );
        assert_eq!(mit(run.runtime_target_at(4.04, 0.004, 0.004, 0.02)).2, 0.0);
        assert_eq!(friction_state(&run).phase, StaticFrictionPhase::Complete);
        assert_eq!(friction_state(&run).return_settled, Some(false));
        assert_eq!(
            friction_state(&run).outcome,
            Some(StaticFrictionOutcome::BreakawayDetected)
        );
    }

    #[test]
    fn friction_operator_stop_and_feedback_failure_are_interrupted_even_after_onset() {
        for reason in [
            "operator stop / mode or device change",
            "experiment feedback timed out",
        ] {
            let mut run = friction_run(-1.0);
            settle(&mut run);
            observe(&mut run, 1.0, -0.003, -0.02);
            observe(&mut run, 1.04, -0.004, -0.02);
            assert_eq!(
                friction_state(&run).outcome,
                Some(StaticFrictionOutcome::BreakawayDetected)
            );
            run.finish(reason);
            assert_eq!(
                friction_state(&run).outcome,
                Some(StaticFrictionOutcome::Interrupted)
            );
            assert_eq!(friction_state(&run).phase, StaticFrictionPhase::Complete);
            assert_eq!(friction_state(&run).return_settled, None);
            assert!(friction_state(&run).onset_sample.is_some());
            assert!(friction_state(&run).confirmation_sample.is_some());
        }
    }

    #[test]
    fn friction_detection_processes_received_frames_before_storage_decimation() {
        let mut run = friction_run(1.0);
        run.status.recording_interval_s = 1.0;
        settle(&mut run);
        observe(&mut run, 0.6, 0.003, 0.02);
        observe(&mut run, 0.64, 0.004, 0.02);
        assert_eq!(run.samples.len(), 1);
        assert_eq!(
            friction_state(&run).outcome,
            Some(StaticFrictionOutcome::BreakawayDetected)
        );
        assert_eq!(friction_state(&run).onset_sample.as_ref().unwrap().t_s, 0.6);
        assert_eq!(
            friction_state(&run)
                .confirmation_sample
                .as_ref()
                .unwrap()
                .t_s,
            0.64
        );
    }

    #[test]
    fn friction_legacy_defaults_and_kind_specific_validation_are_supported() {
        let mut value = serde_json::to_value(config(ExperimentKind::StaticFriction)).unwrap();
        for key in [
            "torque_ramp_rate_nm_s",
            "max_excursion_rad",
            "motion_threshold_rad",
            "motion_confirm_s",
            "settle_time_s",
        ] {
            value.as_object_mut().unwrap().remove(key);
        }
        let mut config: ExperimentConfig = serde_json::from_value(value).unwrap();
        assert_eq!(config.torque_ramp_rate_nm_s, 0.02);
        assert_eq!(config.max_excursion_rad, 3.0_f64.to_radians());
        assert_eq!(config.motion_threshold_rad, 0.15_f64.to_radians());
        assert_eq!(config.motion_confirm_s, 0.04);
        assert_eq!(config.settle_time_s, 0.5);
        assert!(config.validate().is_ok());
        config.motion_threshold_rad = config.max_excursion_rad;
        assert!(config.validate().is_err());
        config.motion_threshold_rad = 0.01_f64.to_radians();
        assert!(config.validate().is_err());
        config.motion_threshold_rad = default_motion_threshold();
        config.torque_ramp_rate_nm_s = f64::NAN;
        assert!(config.validate().is_err());
        config.kind = ExperimentKind::TorqueRamp;
        assert!(config.validate().is_ok());
        config.kind = ExperimentKind::StaticFriction;
        config.torque_ramp_rate_nm_s = 0.02;
        config.max_excursion_rad = 0.1_f64.to_radians();
        assert!(config.validate().is_err());
        config.max_excursion_rad = default_max_excursion();
        config.torque_nm = 0.0;
        assert!(config.validate().is_err());
    }
}
