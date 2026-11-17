//! HopeA3 live-cord protection around the shared follow state machine.
use super::{body_command, FollowInput, FollowStatus, Hopea3State, PullChassisConfig};
use mhf_pull_cord::chassis_model::{FollowController, MeasuredMotion, WalkMode};

#[derive(Default)]
pub(super) struct ChassisFollowController {
    follow: FollowController,
    previous: f64,
    pub(super) retract_braking: bool,
}

impl ChassisFollowController {
    pub(super) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(super) fn status(&self) -> FollowStatus {
        self.follow.status()
    }

    pub(super) fn step(
        &mut self,
        input: FollowInput,
        measured: &Hopea3State,
        dt: f64,
        config: &PullChassisConfig,
        retract_brake_speed_mps: Option<f64>,
    ) -> (f64, f64, f64) {
        if let Some(limit) = retract_brake_speed_mps {
            // Raw normalized cord feedback: payout is positive, retraction negative.
            // This gate precedes every follow mode, length gate and speed filter.
            self.retract_braking |= input.cord_speed_mps.is_some_and(|speed| speed <= -limit);
            if self.retract_braking {
                let stopped = self.previous.abs() <= 1e-9
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
                    let decrement = config.max_deceleration_mps2() * dt.clamp(0.0, 0.03);
                    let previous = self
                        .previous
                        .clamp(-config.max_linear_mps, config.max_linear_mps);
                    self.previous = previous.signum() * (previous.abs() - decrement).max(0.0);
                    // Yaw also targets zero through HopeA3's angular slew limit.
                    return (self.previous, 0.0, 0.0);
                }
                self.retract_braking = false;
            }
        }
        let (command, applied) = self.follow.step_with_motion(
            input,
            self.previous,
            Some(MeasuredMotion {
                linear_mps: measured.meas_vx,
                yaw_rate_rad_s: measured.meas_wz,
            }),
            dt,
            &config.model(),
        );
        self.previous = applied;
        let status = self.follow.status();
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
mod tests {
    use super::*;
    use mhf_pull_cord::chassis_model::{BehindBraking, CordExitSide, InputSource};

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
                        previous: 0.3 * behind_sign,
                        ..Default::default()
                    };
                    let mut measured = Hopea3State {
                        meas_vx: control.previous,
                        ..Default::default()
                    };
                    let behind = control.step(sample(0.8), &measured, 0.01, &config, Some(2.0));
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
                            control.step(sample(1.05), &measured, 0.01, &config, Some(2.0));
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
                previous: 1.0,
                ..Default::default()
            };
            let command = control.step(
                input(&config, speed, 0.8),
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
                            previous,
                            ..Default::default()
                        };
                        let mut measured = Hopea3State {
                            meas_vx: previous,
                            ..Default::default()
                        };
                        let first = control.step(
                            input(&config, -2.0, length),
                            &measured,
                            0.01,
                            &config,
                            Some(2.0),
                        );
                        assert!((first.0 - previous.signum() * 0.97).abs() < 1e-12);
                        assert_eq!((first.1, first.2), (0.0, 0.0));
                        // A speed dip and a new length request cannot interrupt braking.
                        for _ in 0..40 {
                            let before = control.previous;
                            let command = control.step(
                                input(&config, 0.0, 2.0),
                                &measured,
                                0.01,
                                &config,
                                Some(2.0),
                            );
                            assert!(command.0.abs() <= before.abs());
                            assert!(command.0 * previous >= 0.0);
                            assert!(control.retract_braking);
                        }
                        assert_eq!(control.previous, 0.0);
                        measured.meas_vx = 0.0;
                        let held = control.step(
                            input(&config, -2.0, length),
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
                &Hopea3State::default(),
                0.01,
                &config,
                Some(2.0),
            );
            assert!(control.retract_braking);
        }
        let resumed = control.step(
            input(&config, 0.0, 2.0),
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
            previous: 1.0,
            ..Default::default()
        };
        let measured = Hopea3State::default();
        control.step(
            input(&config, -2.0, 0.8),
            &measured,
            0.01,
            &config,
            Some(2.0),
        );
        config.follow.pace.deceleration_mps2 = 4.0;
        let command = control.step(
            input(&config, 0.0, 0.8),
            &measured,
            0.01,
            &config,
            Some(2.0),
        );
        assert!((command.0 - 0.93).abs() < 1e-12);
        config.max_linear_mps = 0.2;
        let command = control.step(
            input(&config, 0.0, 0.8),
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
            previous: 0.4,
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
            control.step(input, &measured, 0.01, &config, None),
            body_command(applied, target.encoder_error_deg, &config)
        );
        assert!(!control.retract_braking);
    }
}
