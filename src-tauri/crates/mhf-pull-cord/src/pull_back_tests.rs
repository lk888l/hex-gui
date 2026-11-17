use super::*;

#[test]
fn a_short_fast_retraction_requests_only_a_small_speed_and_stops_on_release() {
    let mut c = ChassisConfig::default();
    c.follow.max_speed_mps = 0.9;
    c.max_wheel_speed_mps = 1.1;
    c.follow.behind_speed.response_acceleration_mps2 = 0.6;
    let mut controller = FollowController::default();
    let mut wheels = 0.0;
    controller.step_with_feedback(input(1.95, 0.0), wheels, Some(0.0), 0.01, &c);
    for tick in 1..=10 {
        let (command, next) = controller.step_with_feedback(
            input(1.95 - tick as f64 * 0.003, 0.3),
            wheels,
            Some(wheels),
            0.01,
            &c,
        );
        assert!(
            command.speed_mps <= 0.06 + 1e-9,
            "3 cm of retraction must not request cruise speed"
        );
        wheels = next;
    }
    assert!(wheels > 0.0, "a real small gesture still moves forward");
    for _ in 0..200 {
        let (command, next) =
            controller.step_with_feedback(input(1.92, 0.0), wheels, Some(wheels), 0.01, &c);
        assert_eq!(
            command.linear_mps, 0.0,
            "release must remove the longitudinal request immediately"
        );
        wheels = next;
    }
    assert_eq!(wheels, 0.0);
}

#[test]
fn speed_without_enough_actual_shortening_cannot_restart() {
    let c = ChassisConfig::default();
    for stroke in [0.0, 0.004, 0.009] {
        let mut controller = FollowController::default();
        for tick in 0..=100 {
            let (command, wheels) = controller.step_with_feedback(
                input(1.95 - stroke * tick as f64 / 100.0, 0.5),
                0.0,
                Some(0.0),
                0.01,
                &c,
            );
            assert_eq!(command.linear_mps, 0.0);
            assert_eq!(wheels, 0.0);
            assert!(!controller.status().retract_resume);
        }
    }
}

#[test]
fn repeated_small_gestures_do_not_accumulate_a_restart() {
    let c = ChassisConfig::default();
    let mut controller = FollowController::default();
    let mut length = 1.95;
    for _ in 0..20 {
        for _ in 0..10 {
            length -= 0.0005;
            let (_, wheels) =
                controller.step_with_feedback(input(length, 0.05), 0.0, Some(0.0), 0.01, &c);
            assert_eq!(wheels, 0.0);
            assert!(!controller.status().retract_resume);
        }
        controller.step_with_feedback(input(length, 0.0), 0.0, Some(0.0), 0.01, &c);
        assert_eq!(controller.status().retract_distance_m, 0.0);
    }
}

#[test]
fn chassis_induced_cord_shortening_does_not_sustain_a_brief_gesture() {
    for time_constant in [0.0_f64, 0.15, 0.35] {
        for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
            let mut c = ChassisConfig::default();
            c.cord_exit_side = side;
            c.follow.max_speed_mps = 0.9;
            c.max_wheel_speed_mps = 1.1;
            let mut controller = FollowController::default();
            let mut wheels = 0.0;
            let mut actual = 0.0;
            let mut length = 1.95;
            let mut command_distance = 0.0;
            let alpha = if time_constant == 0.0 {
                1.0
            } else {
                1.0 - (-0.01 / time_constant).exp()
            };
            for tick in 0..500 {
                let hand = if (20..30).contains(&tick) { 0.3 } else { 0.0 };
                // Assumed taut straight cord: forward travel itself also
                // shortens the cord. This is a model, not measured dynamics.
                let retraction = hand + actual;
                length -= retraction * 0.01;
                let (_, next) = controller.step_with_feedback(
                    input(length, retraction),
                    wheels,
                    Some(actual * side.sign()),
                    0.01,
                    &c,
                );
                let command = next * side.sign();
                assert!(command >= 0.0);
                command_distance += command * 0.01;
                actual += (command - actual) * alpha;
                wheels = next;
            }
            assert!(command_distance > 0.0 && command_distance < 0.03);
            assert_eq!(wheels, 0.0);
            assert!(actual < 1e-6);
            assert!(!controller.status().retract_resume);
        }
    }
}

#[test]
fn length_cap_speed_cap_and_live_restart_settings_each_take_effect() {
    let mut c = ChassisConfig::default();
    c.follow.max_speed_mps = 0.9;
    c.max_wheel_speed_mps = 1.1;
    let mut controller = FollowController::default();
    let command = confirm(&mut controller, input(1.9, 0.3), &c);
    // First negative sample is at 1.927 m, last at 1.900 m.
    assert!((controller.status().retract_distance_m - 0.027).abs() < 1e-12);
    assert!((command.speed_mps - 0.034).abs() < 1e-12);
    for (gain, speed, expected) in [(1.0, 0.3, 0.017), (2.0, 0.04, 0.009), (0.0, 2.0, 0.0)] {
        c.follow.behind_speed.pull_back_length_gain = gain;
        let (command, _) =
            controller.step_with_feedback(input(1.9, speed), 0.0, Some(0.0), 0.01, &c);
        assert!((command.speed_mps - expected).abs() < 1e-12);
    }
    c.follow.behind_speed.pull_back_length_gain = 2.0;
    c.follow.behind_speed.pull_back_deceleration_mps2 = 0.4;
    let (command, wheels) =
        controller.step_with_feedback(input(1.9, 0.0), 0.1, Some(0.1), 0.01, &c);
    assert_eq!(command.linear_mps, 0.0);
    assert!((wheels - 0.096).abs() < 1e-12);
    assert_eq!(controller.status().acceleration_mps2, 0.4);
    assert!(!controller.status().adaptive_response);
    assert!(controller.status().retract_braking);
}

#[test]
fn release_and_payout_cancel_without_leaking_stroke_into_next_gesture() {
    for release in [0.0, -0.3, 0.01] {
        let c = ChassisConfig::default();
        let mut controller = FollowController::default();
        confirm(&mut controller, input(1.9, 0.4), &c);
        let (command, _) =
            controller.step_with_feedback(input(1.9, release), 0.0, Some(0.0), 0.01, &c);
        assert_eq!(command.linear_mps, 0.0);
        assert!(!controller.status().retract_resume);
        for tick in 0..100 {
            let (command, _) = controller.step_with_feedback(
                input(1.9 - tick as f64 * 0.00005, 0.4),
                0.0,
                Some(0.0),
                0.01,
                &c,
            );
            assert_eq!(
                command.linear_mps, 0.0,
                "old stroke must not count towards a new 5 mm gesture"
            );
        }
    }
}

#[test]
fn negative_displayed_cord_speed_after_long_stop_resumes_forward() {
    let mut c = ChassisConfig::default();
    c.follow.max_speed_mps = 0.9;
    c.max_wheel_speed_mps = 1.1;
    c.follow.encoder_center_deg = 160.0;
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        c.cord_exit_side = side;
        for retract_positive in [true, false] {
            let mut controller = FollowController::default();
            let mut sample = FollowInput {
                encoder_sample_time_s: None,
                cord_length_m: Some(1.82),
                cord_speed_mps: Some(0.0),
                pull_back_speed_mps: Some(0.0),
                encoder_angle_deg: Some(160.0),
            };
            let mut wheels = 0.0;
            for _ in 0..20 {
                (_, wheels) = controller.step_with_feedback(sample, wheels, Some(0.0), 0.01, &c);
                assert_eq!(wheels, 0.0);
            }
            for tick in 1..=50 {
                sample.cord_length_m = Some(1.82 - tick as f64 * 0.003);
                sample.cord_speed_mps = Some(-0.3);
                sample.pull_back_speed_mps = Some(if retract_positive { 0.3 } else { -0.3 });
                let measured = wheels;
                let (command, next) =
                    controller.step_with_feedback(sample, wheels, Some(measured), 0.01, &c);
                assert!(
                    command.linear_mps * side.sign() >= 0.0,
                    "retraction must never request backward travel"
                );
                assert!(next * side.sign() >= 0.0);
                wheels = next;
            }
            assert!(
                wheels * side.sign() > 0.0,
                "negative displayed speed must resume forward for either winding polarity"
            );
            assert_eq!(controller.status().travel_mode, Some(WalkMode::Behind));
        }
    }
}

fn input(length: f64, shaft_speed: f64) -> FollowInput {
    FollowInput {
        encoder_sample_time_s: None,
        cord_length_m: Some(length),
        cord_speed_mps: Some(-shaft_speed),
        pull_back_speed_mps: Some(shaft_speed),
        encoder_angle_deg: Some(0.0),
    }
}

fn confirm(
    controller: &mut FollowController,
    input: FollowInput,
    c: &ChassisConfig,
) -> ChassisCommand {
    let mut command = ChassisCommand::default();
    let final_length = input.cord_length_m.unwrap();
    let stroke = input.retraction_speed().max(0.0) * 0.1;
    controller.step_with_feedback(
        FollowInput {
            cord_length_m: Some(final_length + stroke),
            cord_speed_mps: Some(0.0),
            ..input
        },
        0.0,
        Some(0.0),
        0.01,
        c,
    );
    for tick in 1..=10 {
        (command, _) = controller.step_with_feedback(
            FollowInput {
                cord_length_m: Some(final_length + stroke * (1.0 - tick as f64 / 10.0)),
                ..input
            },
            0.0,
            Some(0.0),
            0.01,
            c,
        );
    }
    command
}

#[test]
fn stopped_retraction_resumes_behind_through_the_stop_band_with_either_exit() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for length in [1.8, 1.95, 2.15] {
            let c = ChassisConfig {
                cord_exit_side: side,
                ..Default::default()
            };
            let mut controller = FollowController::default();
            let command = confirm(&mut controller, input(length, 0.3), &c);
            assert!(controller.status().retract_resume);
            assert!(!controller.status().pull_back_ahead);
            assert_eq!(controller.status().travel_mode, Some(WalkMode::Behind));
            assert!(command.linear_mps * side.sign() > 0.0);
            assert!(command.speed_mps > 0.0 && command.speed_mps <= c.follow.max_speed_mps);
        }
    }
}

#[test]
fn far_ahead_region_clears_resumption_and_requires_fresh_confirmation_on_return() {
    let c = ChassisConfig::default();
    let mut controller = FollowController::default();
    confirm(&mut controller, input(1.9, 0.4), &c);
    assert!(controller.status().retract_resume);
    for speed in [-0.4, 0.0, 0.4] {
        let sample = input(c.follow.behind_ahead_switch_cord_length_m + 0.1, speed);
        let command = confirm(&mut controller, sample, &c);
        assert!(!controller.status().retract_resume);
        assert_eq!(controller.status().travel_mode, Some(WalkMode::Ahead));
        assert_eq!(command.linear_mps, compute(sample, &c).linear_mps);
    }
    controller.step_with_feedback(input(1.94, 0.0), 0.0, Some(0.0), 0.01, &c);
    for tick in 1..=7 {
        let (command, _) = controller.step_with_feedback(
            input(1.94 - tick as f64 * 0.004, 0.4),
            0.0,
            Some(0.0),
            0.01,
            &c,
        );
        assert_eq!(command.linear_mps, 0.0);
        assert!(!controller.status().retract_resume);
    }
    let (command, _) = controller.step_with_feedback(input(1.9, 0.4), 0.0, Some(0.0), 0.01, &c);
    assert!(controller.status().retract_resume);
    assert!(command.linear_mps > 0.0);
}

#[test]
fn resumption_brakes_residual_backward_motion_to_zero_before_forward_launch() {
    let mut c = ChassisConfig::default();
    c.follow.behind_speed.pull_back_acceleration_mps2 = 0.05;
    let mut controller = FollowController::default();
    confirm(&mut controller, input(1.9, 0.4), &c);
    let (_, zero) = controller.step_with_feedback(input(1.9, 0.4), -0.001, Some(-0.001), 0.01, &c);
    assert_eq!(zero, 0.0);
    let (_, forward) = controller.step_with_feedback(input(1.9, 0.4), zero, Some(0.0), 0.01, &c);
    assert!((forward - 0.0005).abs() < 1e-12);
}

#[test]
fn actual_motion_missing_feedback_and_nonfinite_feedback_prevent_switching() {
    let c = ChassisConfig::default();
    for feedback in [
        None,
        Some(0.1),
        Some(-0.1),
        Some(f64::NAN),
        Some(f64::INFINITY),
    ] {
        let mut controller = FollowController::default();
        for _ in 0..100 {
            let (command, _) =
                controller.step_with_feedback(input(1.9, 0.5), 0.0, feedback, 0.01, &c);
            assert_eq!(command.linear_mps, 0.0);
            assert!(!controller.status().retract_resume);
        }
    }
    let mut controller = FollowController::default();
    for _ in 0..100 {
        controller.step_with_feedback(input(1.9, 0.5), 0.1, Some(0.0), 0.01, &c);
        assert!(
            !controller.status().retract_resume,
            "nonzero command also prevents premature resumption"
        );
    }
}

#[test]
fn long_zone_is_remembered_while_stopping_but_short_zone_cannot_arm() {
    for pace in [false, true] {
        let mut c = ChassisConfig::default();
        c.follow.pace.enabled = pace;
        let mut controller = FollowController::default();
        confirm(&mut controller, input(1.7, 0.5), &c);
        assert!(!controller.status().retract_resume);
        controller.step_with_feedback(input(1.81, 0.5), 0.1, Some(0.1), 0.01, &c);
        let (command, _) =
            controller.step_with_feedback(input(1.78, 0.5), 0.05, Some(0.05), 0.01, &c);
        assert_eq!(command.linear_mps, 0.0);
        assert!(!controller.status().pace_active);
        confirm(&mut controller, input(1.76, 0.5), &c);
        assert_eq!(controller.status().retract_resume, !pace);
        assert_eq!(controller.status().pace_active, pace);
    }
}

#[test]
fn confirmation_rejects_deadband_wrong_sign_and_isolated_spikes() {
    let c = ChassisConfig::default();
    for speed in [-1.0, 0.0, 0.019, 0.02] {
        let mut controller = FollowController::default();
        confirm(&mut controller, input(1.9, speed), &c);
        assert!(!controller.status().retract_resume);
    }
    let mut controller = FollowController::default();
    for _ in 0..100 {
        for speed in [0.5, 0.0] {
            controller.step_with_feedback(input(1.9, speed), 0.0, Some(0.0), 0.01, &c);
            assert!(!controller.status().retract_resume);
        }
    }
    controller.step_with_feedback(input(1.9, 0.5), 0.0, Some(0.0), 10.0, &c);
    assert!(
        !controller.status().retract_resume,
        "a scheduling gap is not a sustained gesture"
    );
}

#[test]
fn displayed_negative_cord_speed_triggers_regardless_of_shaft_sign_or_legacy_input() {
    let c = ChassisConfig::default();
    for cord_speed in [-0.4, 0.4] {
        for shaft_speed in [None, Some(-0.4), Some(0.4)] {
            let mut controller = FollowController::default();
            let sample = FollowInput {
                cord_speed_mps: Some(cord_speed),
                pull_back_speed_mps: shaft_speed,
                ..input(1.9, 0.0)
            };
            confirm(&mut controller, sample, &c);
            assert_eq!(controller.status().retract_resume, cord_speed < 0.0);
        }
    }
}

#[test]
fn stationary_drive_keeps_position_following() {
    let c = ChassisConfig::default();
    let mut controller = FollowController::default();
    let sample = FollowInput {
        encoder_angle_deg: Some(30.0),
        ..input(1.9, 0.4)
    };
    for tick in 0..10 {
        let (command, wheels) = controller.step_with_feedback(
            FollowInput {
                cord_length_m: Some(1.94 - tick as f64 * 0.004),
                ..sample
            },
            0.0,
            Some(0.0),
            0.01,
            &c,
        );
        assert_eq!(command.steering_angle_deg, STEERING_CENTER_DEG + 30.0);
        assert!(wheels.abs().max(wheels.abs()) <= c.max_wheel_speed_mps);
    }
    assert!(controller.status().retract_resume);
}

#[test]
fn pull_back_acceleration_tracks_speed_and_uses_its_own_bounded_ceiling() {
    let mut c = ChassisConfig::default();
    c.follow.max_speed_mps = 0.9;
    c.max_wheel_speed_mps = 1.1;
    c.follow.behind_speed.response_acceleration_mps2 = 0.6;
    let mut controller = FollowController::default();
    confirm(&mut controller, input(1.9, 0.3), &c);
    for (speed, expected) in [(0.04, 0.64), (0.2, 0.96), (0.8, 1.2)] {
        let (_, wheels) =
            controller.step_with_feedback(input(1.9, speed), 0.0, Some(0.0), 0.01, &c);
        assert!((controller.status().acceleration_mps2 - expected).abs() < 1e-12);
        assert!((wheels - expected * 0.01).abs() < 1e-12);
        assert_eq!(wheels, wheels);
    }
    c.follow.behind_speed.pull_back_acceleration_mps2 = 0.4;
    controller.step_with_feedback(input(1.9, 1.0), 0.0, Some(0.0), 0.01, &c);
    assert_eq!(controller.status().acceleration_mps2, 0.4);
    let (_, wheels) = controller.step_with_feedback(input(1.9, 1.0), -0.1, Some(-0.1), 0.01, &c);
    assert!(
        (wheels + 0.092).abs() < 1e-12,
        "restart braking uses its dedicated ceiling"
    );
}

#[test]
fn legacy_continued_retraction_crosses_stop_boundary_but_release_cancels_the_gesture() {
    let c = legacy_follow_config();
    let mut controller = FollowController::default();
    confirm(&mut controller, input(1.9, 0.4), &c);
    for (length, speed) in [(1.81, 0.2), (1.79, 0.4), (1.3, 0.1)] {
        let (command, _) =
            controller.step_with_feedback(input(length, speed), 0.1, Some(0.1), 0.01, &c);
        assert!(controller.status().retract_resume);
        assert!(command.linear_mps > 0.0);
    }
    let (command, _) = controller.step_with_feedback(input(1.3, 0.0), 0.1, Some(0.1), 0.01, &c);
    assert_eq!(command.linear_mps, 0.0);
    assert!(!controller.status().retract_resume);
    assert!(controller.status().waiting_for_pull_back);
    assert!(controller.status().retract_braking);
    controller.step_with_feedback(input(1.2, 0.4), 0.0, Some(0.0), 0.01, &c);
    assert!(!controller.status().retract_resume);
    assert!(!controller.status().waiting_for_pull_back);
    confirm(&mut controller, input(1.7, 0.4), &c);
    assert!(!controller.status().retract_resume);
}

#[test]
fn invalid_input_stop_and_mode_change_clear_the_latch() {
    for reason in 0..4 {
        let mut c = ChassisConfig::default();
        let mut controller = FollowController::default();
        confirm(&mut controller, input(1.9, 0.4), &c);
        let mut sample = input(1.9, 0.4);
        match reason {
            0 => sample.pull_back_speed_mps = Some(f64::NAN),
            1 => c.follow.max_speed_mps = 0.0,
            2 => c.mode = WalkMode::Ahead,
            _ => controller.reset(),
        }
        controller.step_with_feedback(sample, 0.0, Some(0.0), 0.01, &c);
        assert!(!controller.status().retract_resume);
    }
}

#[test]
fn short_and_long_stop_bands_keep_position_following_in_both_modes() {
    for mode in [WalkMode::Behind, WalkMode::Ahead] {
        let c = ChassisConfig {
            mode,
            ..Default::default()
        };
        for length in [0.2, 0.75, if mode == WalkMode::Behind { 1.9 } else { 1.0 }] {
            for angle in [-80.0, -20.0, 0.0, 20.0, 80.0] {
                let command = compute(
                    FollowInput {
                        encoder_angle_deg: Some(angle),
                        ..input(length, 0.0)
                    },
                    &c,
                );
                assert_eq!(command.linear_mps, 0.0);
                assert_eq!(command.linear_mps, 0.0);
                assert_eq!(command.steering_angle_deg, STEERING_CENTER_DEG + angle);
            }
        }
    }
}

#[test]
fn wheel_feedback_conversion_undoes_gearing_and_inversion() {
    let mut c = ChassisConfig::default();
    for ratio in [0.5, 1.0, 10.0] {
        c.gear_ratio = ratio;
        for inverted in [false, true] {
            for speed in [-0.3, 0.0, 0.7] {
                assert!(
                    (c.wheel_mps(c.motor_rps(speed, inverted), inverted) - speed).abs() < 1e-12
                );
            }
        }
    }
}

#[test]
fn confirmation_requires_eighty_ms_even_with_cruise_response_disabled() {
    for enabled in [true, false] {
        let mut c = ChassisConfig::default();
        c.follow.behind_speed.enabled = enabled;
        let mut controller = FollowController::default();
        controller.step_with_feedback(input(1.93, 0.0), 0.0, Some(0.0), 0.01, &c);
        for tick in 1..=7 {
            controller.step_with_feedback(
                input(1.93 - tick as f64 * 0.003, 0.3),
                0.0,
                Some(0.0),
                0.01,
                &c,
            );
            assert!(!controller.status().retract_resume);
        }
        controller.step_with_feedback(input(1.9, 0.3), 0.0, Some(0.0), 0.01, &c);
        assert!(controller.status().retract_resume);
    }
}

#[test]
fn adaptive_pull_back_preserves_drive_and_steering_limits_over_time() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = ChassisConfig {
            cord_exit_side: side,
            ..Default::default()
        };
        c.follow.max_speed_mps = 0.9;
        c.max_wheel_speed_mps = 1.1;
        let mut controller = FollowController::default();
        confirm(&mut controller, input(1.9, 0.3), &c);
        let mut previous = 0.0;
        for i in 0..2000 {
            let phase = i as f64 * 0.02;
            let sample = FollowInput {
                encoder_angle_deg: Some(phase.sin() * 80.0),
                ..input(1.9, phase.cos() * 0.8)
            };
            let v0 = previous;
            let (command, wheels) =
                controller.step_with_feedback(sample, previous, Some(v0), 0.01, &c);
            let v = wheels;
            assert!((v - v0).abs() <= controller.status().acceleration_mps2 * 0.01 + 1e-10);
            assert_eq!(command.steering_angle_deg, steering_angle(sample, &c));
            assert!(wheels.abs().max(wheels.abs()) <= c.max_wheel_speed_mps + 1e-10);
            previous = wheels;
        }
    }
}
