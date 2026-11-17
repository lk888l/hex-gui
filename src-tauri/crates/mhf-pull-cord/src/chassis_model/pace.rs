//! A person follows behind the cart: closing the gap requests more forward speed.
use super::{smoothstep, ChassisConfig, CordExitSide, FollowInput};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct PaceFollowParameters {
    pub enabled: bool,
    /// Desired running gap, independent of the existing launch gesture.
    pub target_length_m: f64,
    /// Extra cord beyond the desired gap before the running stop guard.
    pub stop_margin_m: f64,
    /// Filter the signed ground-speed estimate before applying forward limits.
    pub filter_ms: f64,
    pub length_gain_per_s: f64,
    pub length_deadband_m: f64,
    pub max_correction_mps: f64,
    pub acceleration_mps2: f64,
    pub deceleration_mps2: f64,
}

impl Default for PaceFollowParameters {
    fn default() -> Self {
        Self {
            enabled: true,
            target_length_m: 0.68,
            stop_margin_m: 0.5,
            filter_ms: 250.0,
            length_gain_per_s: 0.6,
            length_deadband_m: 0.04,
            max_correction_mps: 0.12,
            acceleration_mps2: 1.0,
            deceleration_mps2: 1.2,
        }
    }
}

impl PaceFollowParameters {
    pub(super) fn validate(self) -> Result<(), String> {
        for (name, value, min, max) in [
            ("pace target length", self.target_length_m, 0.1, 8.0),
            ("pace stop margin", self.stop_margin_m, 0.1, 4.0),
            ("pace filter", self.filter_ms, 20.0, 2000.0),
            ("pace length gain", self.length_gain_per_s, 0.0, 2.0),
            ("pace length deadband", self.length_deadband_m, 0.0, 0.5),
            ("pace length correction", self.max_correction_mps, 0.0, 0.3),
            ("pace acceleration", self.acceleration_mps2, 0.01, 4.0),
            ("pace deceleration", self.deceleration_mps2, 0.01, 4.0),
        ] {
            if !value.is_finite() || !(min..=max).contains(&value) {
                return Err(format!("{name} must be {min}..{max}"));
            }
        }
        if self.length_deadband_m > self.target_length_m.min(self.stop_margin_m) {
            return Err("pace length deadband must not exceed target length or stop margin".into());
        }
        Ok(())
    }

    pub fn stop_length(&self, c: &ChassisConfig) -> f64 {
        // Retain a deliberately longer existing guard. The old short stop
        // remains a launch threshold; running needs room on both sides of the target.
        c.follow
            .behind_stop_cord_length_m
            .max(self.target_length_m + self.stop_margin_m)
    }
}

/// Drive speed plus an optional external body-yaw measurement for models.
/// The single-module runtime uses drive feedback and supplies zero body yaw.
#[derive(Debug, Clone, Copy)]
pub struct MeasuredMotion {
    pub linear_mps: f64,
    pub yaw_rate_rad_s: f64,
}

impl MeasuredMotion {
    pub fn straight(speed_mps: f64) -> Self {
        Self {
            linear_mps: speed_mps,
            yaw_rate_rad_s: 0.0,
        }
    }

    fn valid(self) -> bool {
        self.linear_mps.is_finite() && self.yaw_rate_rad_s.is_finite()
    }
}

#[derive(Default)]
pub(super) struct PaceObserver {
    settings: Option<PaceObserverSettings>,
    grip_speed: SignedSpeedFilter,
    forward_speed: SignedSpeedFilter,
    tracking: bool,
}

#[derive(Clone, Copy, PartialEq)]
struct PaceObserverSettings {
    side: CordExitSide,
    geometry: [f64; 5],
    longitudinal: [f64; 5],
}

#[derive(Default)]
struct SignedSpeedFilter {
    first: Option<f64>,
    second: Option<f64>,
}

impl SignedSpeedFilter {
    fn update(&mut self, raw: f64, alpha: f64, tracking: bool) -> f64 {
        let first = self.first.map_or(raw, |old| old + alpha * (raw - old));
        // Keep the single-stage response for the first gentle launch.
        let second = if tracking {
            self.second.map_or(first, |old| old + alpha * (first - old))
        } else {
            first
        };
        self.first = Some(first);
        self.second = Some(second);
        second
    }
}

#[derive(Default)]
pub(super) struct PaceRequest {
    pub estimated_speed_mps: Option<f64>,
    pub forward_speed_mps: Option<f64>,
    pub correction_mps: f64,
    pub speed_mps: f64,
    pub rotation_guard: bool,
}

impl PaceObserver {
    pub(super) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(super) fn tracking(&self, c: &ChassisConfig) -> bool {
        self.tracking
            && self.settings.is_some_and(|settings| {
                settings.side == c.cord_exit_side
                    && settings.geometry[0] == c.wheelbase_m
                    && settings.geometry[1] == c.follow.encoder_center_deg
            })
    }

    /// Clear velocity history while retaining the running phase across a
    /// measured stop, a protective brake, or a missing wheel sample.
    pub(super) fn suspend(&mut self) {
        self.grip_speed = SignedSpeedFilter::default();
        self.forward_speed = SignedSpeedFilter::default();
    }

    /// Once a valid forward pace request starts, the launch threshold is
    /// not a stop line. Retain only the near-origin cord guard until reset.
    pub(super) fn minimum_length(&self, c: &ChassisConfig) -> f64 {
        if self.tracking(c) {
            c.follow.behind_start_cord_length_m.min(0.05)
        } else {
            c.follow.behind_start_cord_length_m
        }
    }

    fn settings(c: &ChassisConfig) -> PaceObserverSettings {
        let p = &c.follow;
        PaceObserverSettings {
            side: c.cord_exit_side,
            geometry: [
                c.wheelbase_m,
                p.encoder_center_deg,
                p.behind_start_cord_length_m,
                p.behind_target_cord_length_m,
                p.behind_stop_cord_length_m,
            ],
            longitudinal: [
                p.pace.target_length_m,
                p.pace.filter_ms,
                p.pace.length_gain_per_s,
                p.pace.length_deadband_m,
                p.pace.max_correction_mps,
            ],
        }
    }

    pub(super) fn update(
        &mut self,
        input: FollowInput,
        feedback: Option<MeasuredMotion>,
        dt: f64,
        c: &ChassisConfig,
    ) -> PaceRequest {
        let p = &c.follow;
        let tuning = p.pace;
        let settings = Self::settings(c);
        if self.settings != Some(settings) {
            let tracking = self.tracking(c);
            self.reset();
            self.settings = Some(settings);
            self.tracking = tracking;
        }
        let Some(feedback) = feedback.filter(|v| v.valid()) else {
            self.suspend();
            return PaceRequest::default();
        };
        if dt > 0.15 {
            self.suspend();
            return PaceRequest::default();
        }
        let angle = -super::signed_degrees(input.encoder_angle_deg.unwrap() - p.encoder_center_deg)
            .to_radians();
        let sign = c.cord_exit_side.sign();
        // The cart is ahead of the person. The forward reference points from
        // the grip towards the exit: Ldot = (v_exit - v_grip) dot forward.
        // Keep the established wheel/steering directions; change the relative
        // velocity sign, including the passive exit's motion during yaw.
        let exit_x = match c.cord_exit_side {
            CordExitSide::DrivenAxle => 0.0,
            CordExitSide::PassiveAxle => -c.wheelbase_m,
        };
        let exit_rotation_mps = sign * feedback.yaw_rate_rad_s * exit_x * angle.sin();
        let raw = sign * feedback.linear_mps * angle.cos() + exit_rotation_mps
            - input.cord_speed_mps.unwrap();
        if !raw.is_finite() {
            self.suspend();
            return PaceRequest::default();
        }
        // Keep the physical along-cord estimate for telemetry and to reject
        // apparent retraction caused by rotation of the passive exit. It is
        // not the longitudinal pace command.
        let alpha = 1.0 - (-dt / (tuning.filter_ms / 1000.0)).exp();
        let estimated = self.grip_speed.update(raw, alpha, self.tracking);

        // A stopped chassis must not launch from cord retraction produced by
        // rotating its passive exit. Once translating, projected grip speed
        // never gates pace, including sideways/backwards cord directions.
        let rotation_only = feedback.linear_mps.abs() <= 0.02
            && exit_rotation_mps < -0.01
            && input.cord_speed_mps.unwrap() < -0.01
            && raw <= 0.01;
        if rotation_only {
            self.forward_speed = SignedSpeedFilter::default();
            return PaceRequest {
                estimated_speed_mps: Some(estimated),
                forward_speed_mps: Some(0.0),
                rotation_guard: true,
                ..Default::default()
            };
        }

        // Longitudinal control uses the same drive-speed and line-speed law
        // at every steering angle. Dividing by cos(angle)
        // would amplify both acceleration and braking merely by steering.
        let raw_forward = sign * feedback.linear_mps - input.cord_speed_mps.unwrap();
        if !raw_forward.is_finite() {
            self.suspend();
            return PaceRequest::default();
        }
        // Both filters remain signed: hand swing must not rectify into creep.
        let forward = self.forward_speed.update(raw_forward, alpha, self.tracking);
        let length = input.cord_length_m.unwrap();
        let error = tuning.target_length_m - length;
        // Outer distance loop: open a short gap, close a long one. Inside the
        // tolerance band only measured walking pace remains. No integral term
        // can accumulate during a speed limit or a stop.
        let mut correction = (error.signum()
            * (error.abs() - tuning.length_deadband_m).max(0.0)
            * tuning.length_gain_per_s)
            .clamp(-tuning.max_correction_mps, tuning.max_correction_mps);
        // Let the existing retraction gesture launch first. Distance feedback
        // fades in with actual forward motion, rather than launching from a
        // position error. At a slow walking pace it cannot cancel all motion.
        correction = correction.max(-forward.max(0.0) * 0.5)
            * smoothstep(forward.max(0.0) / 0.1)
            * smoothstep(sign * feedback.linear_mps / 0.1);
        let speed = if forward <= 0.01 {
            0.0
        } else {
            (forward + correction).clamp(0.0, p.max_speed_mps)
        };
        // The stop line is enforced by the controller's independent brake.
        // The bounded distance correction is separate from that hard guard;
        // no length-proportional speed ceiling is applied to matched pace.
        if speed > 0.0 {
            self.tracking = true;
        }
        PaceRequest {
            estimated_speed_mps: Some(estimated),
            forward_speed_mps: Some(forward),
            correction_mps: correction,
            speed_mps: speed,
            rotation_guard: false,
        }
    }
}
