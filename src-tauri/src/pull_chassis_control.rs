//! HopeA3 body commands from the shared longitudinal controller and live cord feedback.
use super::{
    body_command, FollowInput, FollowStatus, Hopea3State, PaceMotionMode, PullChassisConfig,
};
use crate::hopea3::slew_translation;
use mhf_pull_cord::chassis_model::{FollowController, InputSource, MeasuredMotion, WalkMode};
use mhf_pull_cord::trajectory::CordEndpoint;

#[derive(Default)]
struct LateralPace {
    segment: Option<(u64, u64)>,
    reference_y_m: f64,
    filtered_speed_mps: f64,
}

impl LateralPace {
    fn target(
        &mut self,
        endpoint: Option<CordEndpoint>,
        measured: &Hopea3State,
        config: &PullChassisConfig,
        dt: f64,
    ) -> f64 {
        // A live centre edit can reach the chassis before the cord recorder
        // publishes its first sample in the new frame.
        let sample = endpoint
            .filter(|p| p.center_deg == config.follow.encoder_center_deg)
            .and_then(|p| p.velocity_m_s.map(|velocity| (p, velocity[1])));
        let Some((point, relative_vy)) = sample else {
            *self = Self::default();
            return 0.0;
        };
        let side = config.cord_exit_side.sign();
        let y_sign = if config.direction_inverted {
            -side
        } else {
            side
        };
        let y = y_sign * point.y_m;
        // In body coordinates v_grip = v_base + omega × r + d(r)/dt.
        // The outlet is the translation reference; omni cruise commands zero yaw.
        let grip_vy = measured.meas_vy + measured.meas_wz * side * point.x_m + y_sign * relative_vy;
        if !y.is_finite() || !grip_vy.is_finite() {
            *self = Self::default();
            return 0.0;
        }
        let segment = (point.frame_id, point.segment);
        if self.segment != Some(segment) {
            self.segment = Some(segment);
            self.reference_y_m = y;
            self.filtered_speed_mps = 0.0;
        }
        let pace = &config.follow.pace;
        let limit = config.max_linear_mps.min(config.follow.max_speed_mps);
        let alpha = 1.0 - (-dt / (pace.filter_ms / 1000.0)).exp();
        self.filtered_speed_mps += alpha * (grip_vy.clamp(-limit, limit) - self.filtered_speed_mps);
        let speed = if self.filtered_speed_mps.abs() <= 0.01 {
            0.0
        } else {
            self.filtered_speed_mps
        };
        let error = y - self.reference_y_m;
        let correction = (error.abs() - pace.length_deadband_m).max(0.0) * pace.length_gain_per_s;
        speed + error.signum() * correction.min(pace.max_correction_mps)
    }
}

#[derive(Default)]
pub(super) struct ChassisFollowController {
    follow: FollowController,
    previous_x: f64,
    previous_y: f64,
    lateral: LateralPace,
    pub(super) retract_braking: bool,
}

impl ChassisFollowController {
    pub(super) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(super) fn status(&self) -> FollowStatus {
        self.follow.status()
    }

    pub(super) fn lateral_pace_active(&self) -> bool {
        self.lateral.segment.is_some()
    }

    fn ramp_translation(
        &mut self,
        mut target: (f64, f64),
        limit: f64,
        acceleration: f64,
        deceleration: f64,
        dt: f64,
    ) -> (f64, f64, f64) {
        // Project both vectors when a hard limit is reduced during a run.
        let from = slew_translation((0.0, 0.0), (self.previous_x, self.previous_y), limit);
        // As in the longitudinal controller, finish braking a reversing axis
        // before accelerating it in the new direction.
        if from.0 * target.0 < 0.0 {
            target.0 = 0.0;
        }
        if from.1 * target.1 < 0.0 {
            target.1 = 0.0;
        }
        let to = slew_translation((0.0, 0.0), target, limit);
        let slowing = (to.0 - from.0) * from.0 + (to.1 - from.1) * from.1 < 0.0;
        let rate = if slowing { deceleration } else { acceleration };
        (self.previous_x, self.previous_y) = slew_translation(from, to, rate * dt.clamp(0.0, 0.03));
        (self.previous_x, self.previous_y, 0.0)
    }

    pub(super) fn step(
        &mut self,
        input: FollowInput,
        endpoint: Option<CordEndpoint>,
        measured: &Hopea3State,
        dt: f64,
        config: &PullChassisConfig,
        retract_brake_speed_mps: Option<f64>,
    ) -> (f64, f64, f64) {
        let omni = config.pace_motion == PaceMotionMode::Omni
            && config.source == InputSource::Live
            && config.mode == WalkMode::Behind;
        let pace_limit = config.max_linear_mps.min(config.follow.max_speed_mps);
        if let Some(limit) = retract_brake_speed_mps {
            // Raw normalized cord feedback: payout is positive, retraction negative.
            // This gate precedes every follow mode, length gate and speed filter.
            self.retract_braking |= input.cord_speed_mps.is_some_and(|speed| speed <= -limit);
            if self.retract_braking {
                let stopped = self.previous_x.abs() <= 1e-9
                    && self.previous_y.abs() <= 1e-9
                    && measured.meas_vx.abs() <= 0.02
                    && measured.meas_vy.abs() <= 0.02
                    && measured.meas_wz.abs() <= 0.02;
                // Finish braking even when the cord slows early. A 5% release
                // margin avoids toggling around the threshold after stopping.
                let released = input
                    .cord_speed_mps
                    .is_some_and(|speed| speed > -limit * 0.95);
                if !stopped || !released {
                    self.follow.reset();
                    self.lateral = LateralPace::default();
                    let rate = config.max_deceleration_mps2();
                    let speed_limit = if omni && config.follow.pace.enabled {
                        pace_limit
                    } else {
                        config.max_linear_mps
                    };
                    // Yaw also targets zero through HopeA3's angular slew limit.
                    return self.ramp_translation((0.0, 0.0), speed_limit, rate, rate, dt);
                }
                self.retract_braking = false;
            }
        }
        let lateral_stopped = !omni || (self.previous_y == 0.0 && measured.meas_vy.abs() <= 0.02);
        let (command, applied) = self.follow.step_with_planar_motion(
            input,
            self.previous_x,
            Some(MeasuredMotion {
                linear_mps: measured.meas_vx,
                yaw_rate_rad_s: measured.meas_wz,
            }),
            lateral_stopped,
            dt,
            &config.model(),
        );
        let status = self.follow.status();
        if status.travel_mode.is_none() {
            self.reset();
            return (0.0, 0.0, 0.0);
        }
        if omni && status.pace_active && status.pace_tracking && !status.pace_rotation_guard {
            let vy = self.lateral.target(endpoint, measured, config, dt);
            let pace = &config.follow.pace;
            return self.ramp_translation(
                (command.linear_mps, vy),
                pace_limit,
                pace.acceleration_mps2,
                pace.deceleration_mps2,
                dt,
            );
        }
        self.lateral = LateralPace::default();
        if self.previous_y != 0.0 || (omni && status.pace_tracking && status.braking.is_some()) {
            let rate = status.acceleration_mps2;
            return self.ramp_translation((command.linear_mps, 0.0), pace_limit, rate, rate, dt);
        }
        self.previous_x = applied;
        // Automatic Ahead takeover does not change the configured operator
        // mode. Reverse steering for actual Ahead guidance, after the shared
        // brake completes so residual Behind motion keeps its original yaw.
        let encoder_error_deg =
            if status.travel_mode == Some(WalkMode::Ahead) && status.braking.is_none() {
                -command.encoder_error_deg
            } else {
                command.encoder_error_deg
            };
        body_command(applied, encoder_error_deg, config)
    }
}

#[cfg(test)]
#[path = "pull_chassis_omni_tests.rs"]
mod omni_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use mhf_pull_cord::chassis_model::{BehindBraking, CordExitSide};

    fn input(config: &PullChassisConfig, speed: f64, length: f64) -> FollowInput {
        FollowInput {
            cord_speed_mps: Some(speed),
            cord_length_m: Some(length),
            encoder_angle_deg: Some(config.follow.encoder_center_deg + 30.0),
            ..Default::default()
        }
    }

    #[test]
    fn automatic_ahead_guidance_reverses_steering_only_after_switch_braking() {
        for (side, behind_sign) in [
            (CordExitSide::DrivenAxle, 1.0),
            (CordExitSide::PassiveAxle, -1.0),
        ] {
            for angle in [-30.0_f64, 30.0] {
                for inverted in [false, true] {
                    let mut config = PullChassisConfig::default();
                    config.mode = WalkMode::Behind;
                    config.source = InputSource::Live;
                    config.cord_exit_side = side;
                    config.direction_inverted = inverted;
                    config.follow.pace.enabled = true;
                    config.follow.behind_start_cord_length_m = 0.55;
                    config.follow.behind_target_cord_length_m = 0.65;
                    config.follow.behind_stop_cord_length_m = 0.9;
                    config.follow.behind_ahead_switch_cord_length_m = 1.0;
                    config.follow.pace.target_length_m = 0.8;
                    config.follow.pace.stop_margin_m = 0.2;
                    config.validate().unwrap();
                    let sample = |length| FollowInput {
                        encoder_angle_deg: Some(config.follow.encoder_center_deg + angle),
                        ..input(&config, 0.0, length)
                    };
                    let mut control = ChassisFollowController {
                        previous_x: 0.3 * behind_sign,
                        ..Default::default()
                    };
                    let mut measured = Hopea3State {
                        meas_vx: control.previous_x,
                        ..Default::default()
                    };
                    let behind =
                        control.step(sample(0.8), None, &measured, 0.01, &config, Some(2.0));
                    assert!(control.status().pace_active);
                    assert_eq!(control.status().travel_mode, Some(WalkMode::Behind));
                    assert_eq!(behind.1, 0.0);
                    assert_eq!(
                        behind.2.signum(),
                        if inverted { behind_sign } else { -behind_sign } * angle.signum()
                    );

                    let mut braked = false;
                    let mut guided = false;
                    for _ in 0..100 {
                        let command =
                            control.step(sample(1.05), None, &measured, 0.01, &config, Some(2.0));
                        assert_eq!(command.1, 0.0);
                        assert!(command.2.abs() <= config.max_angular_rad_s);
                        if control.status().braking.is_some() {
                            assert_eq!(control.status().braking, Some(BehindBraking::AheadSwitch));
                            assert!(command.0 * behind_sign >= 0.0);
                            braked = true;
                        } else if command.0 * behind_sign < 0.0 {
                            assert!(braked);
                            assert_eq!(control.status().travel_mode, Some(WalkMode::Ahead));
                            assert_eq!(config.mode, WalkMode::Behind);
                            guided = true;
                        }
                        // The corrected Ahead response has the same left/right
                        // yaw sign as Behind despite reversing longitudinal travel.
                        if command.0 != 0.0 {
                            assert_eq!(command.2.signum(), behind.2.signum());
                        } else {
                            assert_eq!(command.2, 0.0);
                        }
                        measured.meas_vx = command.0;
                    }
                    assert!(guided, "switch must finish and start Ahead guidance");
                }
            }
        }
    }

    #[test]
    fn explicit_ahead_guidance_keeps_deadband_limits_and_corrected_left_right_response() {
        let mut config = PullChassisConfig::default();
        config.mode = WalkMode::Ahead;
        config.cord_exit_side = CordExitSide::DrivenAxle;
        config.follow.direction_deadband_deg = 5.0;
        config.max_angular_rad_s = 0.001;
        for (angle, expected_yaw) in [
            (-30.0, 0.001),
            (-5.0, 0.0),
            (0.0, 0.0),
            (5.0, 0.0),
            (30.0, -0.001),
        ] {
            let mut control = ChassisFollowController::default();
            let command = control.step(
                FollowInput {
                    encoder_angle_deg: Some(config.follow.encoder_center_deg + angle),
                    ..input(&config, 0.0, config.follow.start_cord_length_m + 0.2)
                },
                None,
                &Hopea3State::default(),
                0.01,
                &config,
                Some(2.0),
            );
            assert_eq!(control.status().travel_mode, Some(WalkMode::Ahead));
            assert!(command.0 < 0.0);
            assert_eq!(command.1, 0.0);
            assert_eq!(command.2, expected_yaw);
        }
    }

    #[test]
    fn retract_brake_is_directional_and_triggers_at_the_exact_raw_threshold() {
        let config = PullChassisConfig::default();
        for speed in [2.5, 2.0, 0.0, -1.999, -2.0, -2.5] {
            let mut control = ChassisFollowController {
                previous_x: 1.0,
                ..Default::default()
            };
            let command = control.step(
                input(&config, speed, 0.8),
                None,
                &Hopea3State::default(),
                0.01,
                &config,
                Some(2.0),
            );
            assert_eq!(control.retract_braking, speed <= -2.0, "cord speed {speed}");
            if speed <= -2.0 {
                assert_eq!(command, (0.97, 0.0, 0.0));
            }
        }
    }

    #[test]
    fn retract_brake_overrides_all_follow_modes_and_never_reverses_direction() {
        for mode in [WalkMode::Behind, WalkMode::Ahead] {
            for pace in [false, true] {
                for previous in [-1.0, 1.0] {
                    for length in [0.0, 0.8, 1.4, 2.0] {
                        let mut config = PullChassisConfig::default();
                        config.mode = mode;
                        config.follow.pace.enabled = pace;
                        config.acceleration_mps2 = 0.01;
                        config.follow.behind_speed.filter_ms = 5000.0;
                        let mut control = ChassisFollowController {
                            previous_x: previous,
                            ..Default::default()
                        };
                        let mut measured = Hopea3State {
                            meas_vx: previous,
                            ..Default::default()
                        };
                        let first = control.step(
                            input(&config, -2.0, length),
                            None,
                            &measured,
                            0.01,
                            &config,
                            Some(2.0),
                        );
                        assert!((first.0 - previous.signum() * 0.97).abs() < 1e-12);
                        assert_eq!((first.1, first.2), (0.0, 0.0));
                        // A speed dip and a new length request cannot interrupt braking.
                        for _ in 0..40 {
                            let before = control.previous_x;
                            let command = control.step(
                                input(&config, 0.0, 2.0),
                                None,
                                &measured,
                                0.01,
                                &config,
                                Some(2.0),
                            );
                            assert!(command.0.abs() <= before.abs());
                            assert!(command.0 * previous >= 0.0);
                            assert!(control.retract_braking);
                        }
                        assert_eq!(control.previous_x, 0.0);
                        measured.meas_vx = 0.0;
                        let held = control.step(
                            input(&config, -2.0, length),
                            None,
                            &measured,
                            0.01,
                            &config,
                            Some(2.0),
                        );
                        assert_eq!(held, (0.0, 0.0, 0.0));
                        assert!(control.retract_braking);
                    }
                }
            }
        }
    }

    #[test]
    fn release_waits_for_translation_yaw_and_speed_margin_then_restarts_from_rest() {
        let mut config = PullChassisConfig::default();
        config.mode = WalkMode::Ahead;
        config.cord_exit_side = CordExitSide::DrivenAxle;
        let mut control = ChassisFollowController::default();
        control.step(
            input(&config, -2.0, 2.0),
            None,
            &Hopea3State::default(),
            0.01,
            &config,
            Some(2.0),
        );
        assert!(control.retract_braking);
        for measured in [
            Hopea3State {
                meas_vx: 0.03,
                ..Default::default()
            },
            Hopea3State {
                meas_vy: 0.03,
                ..Default::default()
            },
            Hopea3State {
                meas_wz: 0.03,
                ..Default::default()
            },
            Hopea3State {
                meas_vx: f64::NAN,
                ..Default::default()
            },
        ] {
            assert_eq!(
                control.step(
                    input(&config, 0.0, 2.0),
                    None,
                    &measured,
                    0.01,
                    &config,
                    Some(2.0)
                ),
                (0.0, 0.0, 0.0)
            );
            assert!(control.retract_braking);
        }
        for speed in [-2.01, -1.99, -1.9] {
            control.step(
                input(&config, speed, 2.0),
                None,
                &Hopea3State::default(),
                0.01,
                &config,
                Some(2.0),
            );
            assert!(control.retract_braking);
        }
        let resumed = control.step(
            input(&config, 0.0, 2.0),
            None,
            &Hopea3State::default(),
            0.01,
            &config,
            Some(2.0),
        );
        assert!(!control.retract_braking);
        assert!(resumed.0.abs() > 0.0);
        assert!(resumed.0.abs() <= config.acceleration_mps2 * 0.01 + 1e-12);
    }

    #[test]
    fn braking_uses_live_maximum_and_clamps_to_lowered_hard_speed_limit() {
        let mut config = PullChassisConfig::default();
        let mut control = ChassisFollowController {
            previous_x: 1.0,
            ..Default::default()
        };
        let measured = Hopea3State::default();
        control.step(
            input(&config, -2.0, 0.8),
            None,
            &measured,
            0.01,
            &config,
            Some(2.0),
        );
        config.follow.pace.deceleration_mps2 = 4.0;
        let command = control.step(
            input(&config, 0.0, 0.8),
            None,
            &measured,
            0.01,
            &config,
            Some(2.0),
        );
        assert!((command.0 - 0.93).abs() < 1e-12);
        config.max_linear_mps = 0.2;
        let command = control.step(
            input(&config, 0.0, 0.8),
            None,
            &measured,
            0.01,
            &config,
            Some(2.0),
        );
        assert!((command.0 - 0.16).abs() < 1e-12);
    }

    #[test]
    fn manual_bench_preserves_the_shared_follow_behavior() {
        let config = PullChassisConfig::default();
        let measured = Hopea3State::default();
        let input = input(&config, -2.5, 0.8);
        let mut control = ChassisFollowController {
            previous_x: 0.4,
            ..Default::default()
        };
        let mut reference = FollowController::default();
        let (target, applied) = reference.step_with_motion(
            input,
            0.4,
            Some(MeasuredMotion {
                linear_mps: 0.0,
                yaw_rate_rad_s: 0.0,
            }),
            0.01,
            &config.model(),
        );
        assert_eq!(
            control.step(input, None, &measured, 0.01, &config, None),
            body_command(applied, target.encoder_error_deg, &config)
        );
        assert!(!control.retract_braking);
    }
}
