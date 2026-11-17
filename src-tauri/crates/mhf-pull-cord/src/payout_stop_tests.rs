use super::*;

fn sample(length: f64, speed: f64) -> FollowInput {
    FollowInput {
        cord_length_m: Some(length),
        cord_speed_mps: Some(speed),
        encoder_angle_deg: Some(0.0),
        ..Default::default()
    }
}

fn tug(controller: &mut FollowController, origin: f64, c: &ChassisConfig) -> f64 {
    let mut wheels = 0.15 * c.cord_exit_side.sign();
    for tick in 0..4 {
        let (command, next) =
            controller.step(sample(origin + tick as f64 * 0.01, 1.0), wheels, 0.01, c);
        assert_eq!(controller.status().payout_stop, tick == 3);
        if tick == 3 {
            assert_eq!(command.linear_mps, 0.0);
        }
        wheels = next;
    }
    wheels
}

#[test]
fn tug_brakes_and_holds_after_release_for_both_exit_sides_and_cruise_switches() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for enabled in [false, true] {
            for adaptive in [false, true] {
                let mut c = legacy_follow_config();
                c.cord_exit_side = side;
                c.acceleration_mps2 = 0.01;
                c.follow.behind_speed.enabled = enabled;
                c.follow.behind_speed.adaptive_enabled = adaptive;
                c.follow.behind_speed.filter_ms = 5000.0;
                let mut controller = FollowController::default();
                let mut wheels = tug(&mut controller, 1.3, &c);
                for _ in 0..100 {
                    let (command, next) = controller.step(sample(1.33, 0.0), wheels, 0.01, &c);
                    assert_eq!(command.speed_mps, 0.0);
                    assert_eq!(command.linear_mps, 0.0);
                    assert!(controller.status().payout_stop);
                    assert!(!controller.status().waiting_for_pull_back);
                    assert!(!controller.status().adaptive_response);
                    assert_eq!(controller.status().acceleration_mps2, 1.2);
                    assert!(
                        (next * side.sign() - (wheels * side.sign() - 0.012).max(0.0)).abs()
                            < 1e-12
                    );
                    wheels = next;
                }
                assert_eq!(wheels, 0.0);
                let (command, next) = controller.step(sample(1.2, 0.0), wheels, 0.01, &c);
                assert!(!controller.status().payout_stop);
                assert!(command.linear_mps * side.sign() > 0.0);
                assert!(next * side.sign() > 0.0);
            }
        }
    }
}

#[test]
fn tug_inside_full_speed_band_requires_return_to_its_origin() {
    let c = legacy_follow_config();
    let mut controller = FollowController::default();
    tug(&mut controller, 1.0, &c);
    assert_eq!(controller.status().payout_stop_resume_length_m, Some(1.0));
    for length in [1.03, 1.03, 1.01, 1.001] {
        let (command, wheels) = controller.step(sample(length, 0.0), 0.0, 0.01, &c);
        assert!(controller.status().payout_stop);
        assert_eq!(command.linear_mps, 0.0);
        assert_eq!(wheels, 0.0);
    }
    let (command, _) = controller.step(sample(1.0, -0.1), 0.0, 0.01, &c);
    assert!(!controller.status().payout_stop);
    assert!(command.linear_mps > 0.0);
}

#[test]
fn return_length_alone_does_not_cancel_braking_before_a_confirmed_stop() {
    let c = legacy_follow_config();
    let mut controller = FollowController::default();
    tug(&mut controller, 1.3, &c);
    for (previous, measured, speed) in [
        (0.1, Some(0.0), -0.3),
        (0.0, Some(0.1), -0.3),
        (0.0, None, -0.3),
        (0.0, Some(f64::NAN), -0.3),
        (0.0, Some(f64::INFINITY), -0.3),
        (0.0, Some(0.0), 0.3),
    ] {
        let (command, _) =
            controller.step_with_feedback(sample(1.2, speed), previous, measured, 0.01, &c);
        assert!(controller.status().payout_stop);
        assert_eq!(command.linear_mps, 0.0);
    }
    let (command, _) = controller.step(sample(1.2, 0.0), 0.0, 0.01, &c);
    assert!(!controller.status().payout_stop);
    assert!(command.linear_mps > 0.0);
}

#[test]
fn retraction_restart_cannot_bypass_a_tug_stop_even_after_entering_the_long_stop_band() {
    let c = legacy_follow_config();
    let mut controller = FollowController::default();
    tug(&mut controller, 1.3, &c);
    controller.step(sample(1.95, 0.0), 0.0, 0.01, &c);
    for tick in 0..70 {
        let (command, wheels) =
            controller.step(sample(1.95 - tick as f64 * 0.01, -1.0), 0.0, 0.01, &c);
        assert!(controller.status().payout_stop);
        assert!(!controller.status().retract_resume);
        assert_eq!(command.linear_mps, 0.0);
        assert_eq!(wheels, 0.0);
    }
    let (command, _) = controller.step(sample(1.2, -1.0), 0.0, 0.01, &c);
    assert!(command.linear_mps > 0.0);
    assert!(!controller.status().payout_stop);
}

#[test]
fn reaching_or_crossing_the_ahead_switch_finishes_braking_before_ahead_takeover() {
    for switch in [2.2, 2.5] {
        for offset in [0.0, 0.001, 0.2] {
            let mut c = legacy_follow_config();
            c.follow.behind_ahead_switch_cord_length_m = switch;
            let mut controller = FollowController::default();
            tug(&mut controller, 1.3, &c);
            let input = sample(switch + offset, 1.0);
            let (command, _) = controller.step(input, 0.1, 0.01, &c);
            assert_eq!(controller.status().travel_mode, Some(WalkMode::Ahead));
            assert!(controller.status().payout_stop);
            assert_eq!(
                controller.status().braking,
                Some(BehindBraking::AheadSwitch)
            );
            assert_eq!(command.linear_mps, 0.0);
            assert!(!controller.status().waiting_for_pull_back);
            let (command, _) = controller.step(input, 0.0, 0.01, &c);
            assert!(!controller.status().payout_stop);
            assert_eq!(controller.status().braking, None);
            assert_eq!(command.linear_mps, compute(input, &c).linear_mps);
            assert!(command.linear_mps <= 0.0);
            if offset == 0.0 {
                // The Ahead length error is zero exactly at the threshold.
                assert_eq!(command.linear_mps, 0.0);
            }
            let (command, _) = controller.step(sample(1.2, 0.0), 0.0, 0.01, &c);
            assert_eq!(controller.status().travel_mode, Some(WalkMode::Behind));
            assert!(command.linear_mps > 0.0);
        }
    }
}

#[test]
fn switch_on_the_confirmation_tick_also_takes_priority() {
    let c = legacy_follow_config();
    let mut controller = FollowController::default();
    for tick in 0..3 {
        controller.step(sample(1.3 + tick as f64 * 0.01, 1.0), 0.15, 0.01, &c);
    }
    controller.step(sample(2.2, 1.0), 0.15, 0.01, &c);
    assert!(!controller.status().payout_stop);
    assert_eq!(controller.status().travel_mode, Some(WalkMode::Ahead));
}

#[test]
fn slow_payout_wrong_sign_and_velocity_without_actual_payout_do_not_trigger() {
    let c = legacy_follow_config();
    for (speed, stroke) in [
        (0.3, 0.1),
        (-1.0, 0.1),
        (0.0, 0.1),
        (1.0, 0.0),
        (1.0, 0.019),
    ] {
        let mut controller = FollowController::default();
        for tick in 0..=100 {
            let (command, _) = controller.step(
                sample(1.0 + stroke * tick as f64 / 100.0, speed),
                0.15,
                0.01,
                &c,
            );
            assert!(!controller.status().payout_stop);
            assert!(command.linear_mps > 0.0);
        }
    }
}

#[test]
fn separate_short_gestures_and_a_single_scheduling_gap_do_not_confirm_a_tug() {
    let c = legacy_follow_config();
    let mut controller = FollowController::default();
    for _ in 0..100 {
        for tick in 0..3 {
            controller.step(sample(1.0 + tick as f64 * 0.02, 1.0), 0.15, 0.01, &c);
            assert!(!controller.status().payout_stop);
        }
        controller.step(sample(1.0, 0.0), 0.15, 0.01, &c);
    }
    controller.step(sample(1.1, 1.0), 0.15, 5.0, &c);
    assert!(!controller.status().payout_stop);
}

#[test]
fn only_forward_cruise_can_arm_and_normalized_cord_speed_ignores_shaft_sign() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = legacy_follow_config();
        c.cord_exit_side = side;
        for speed in [0.0, -0.1, 0.15] {
            for shaft_speed in [None, Some(-1.0), Some(1.0)] {
                let mut controller = FollowController::default();
                for tick in 0..10 {
                    controller.step(
                        FollowInput {
                            pull_back_speed_mps: shaft_speed,
                            ..sample(1.3 + tick as f64 * 0.01, 1.0)
                        },
                        speed * side.sign(),
                        0.01,
                        &c,
                    );
                }
                assert_eq!(controller.status().payout_stop, speed > 0.01);
            }
        }
        let mut controller = FollowController::default();
        for tick in 0..10 {
            controller.step(sample(1.3 + tick as f64 * 0.01, 1.0), 0.0, 0.01, &c);
        }
        assert!(
            !controller.status().payout_stop,
            "steering alone is not forward travel"
        );
    }
}

#[test]
fn tug_stop_retains_heading_control_and_live_braking_settings() {
    let mut c = legacy_follow_config();
    let mut controller = FollowController::default();
    tug(&mut controller, 1.3, &c);
    c.follow.behind_speed.payout_stop_deceleration_mps2 = 0.4;
    let (_, next) = controller.step(sample(1.33, 0.0), 0.1, 0.01, &c);
    assert!((next - 0.096).abs() < 1e-12);
    let (command, wheels) = controller.step(
        FollowInput {
            encoder_angle_deg: Some(30.0),
            ..sample(1.33, 0.0)
        },
        0.0,
        0.01,
        &c,
    );
    assert_eq!(command.linear_mps, 0.0);
    assert_eq!(command.steering_angle_deg, STEERING_CENTER_DEG + 30.0);
    assert_eq!(wheels, 0.0);
    assert_eq!(wheels, 0.0);
    c.follow.behind_target_cord_length_m = 1.1;
    let (command, _) = controller.step(sample(1.2, 0.0), 0.0, 0.01, &c);
    assert_eq!(command.linear_mps, 0.0);
    assert_eq!(controller.status().payout_stop_resume_length_m, Some(1.1));
    let (command, _) = controller.step(sample(1.1, 0.0), 0.0, 0.01, &c);
    assert!(command.linear_mps > 0.0);
    assert!(!controller.status().payout_stop);
}

#[test]
fn reset_invalid_input_mode_change_zero_limit_and_disable_clear_the_latch() {
    for action in 0..5 {
        let mut c = legacy_follow_config();
        let mut controller = FollowController::default();
        tug(&mut controller, 1.3, &c);
        match action {
            0 => controller.reset(),
            1 => {
                controller.step(FollowInput::default(), 0.1, 0.01, &c);
            }
            2 => c.mode = WalkMode::Ahead,
            3 => c.follow.max_speed_mps = 0.0,
            _ => c.follow.behind_speed.payout_stop_enabled = false,
        }
        controller.step(sample(1.33, 0.0), 0.0, 0.01, &c);
        assert!(!controller.status().payout_stop);
        c = legacy_follow_config();
        let (command, _) = controller.step(sample(1.33, 0.0), 0.0, 0.01, &c);
        assert!(command.linear_mps > 0.0);
    }
}

#[test]
fn configurable_speed_confirmation_and_stroke_requirements_all_apply() {
    let mut c = legacy_follow_config();
    c.follow.behind_speed.payout_stop_speed_mps = 0.8;
    c.follow.behind_speed.payout_stop_confirm_ms = 80.0;
    c.follow.behind_speed.payout_stop_min_payout_m = 0.05;
    let mut controller = FollowController::default();
    for tick in 0..10 {
        controller.step(sample(1.0 + tick as f64 * 0.01, 0.79), 0.15, 0.01, &c);
        assert!(!controller.status().payout_stop);
    }
    for tick in 0..=10 {
        controller.step(sample(1.1 + tick as f64 * 0.0051, 0.8), 0.15, 0.01, &c);
        assert_eq!(controller.status().payout_stop, tick == 10);
    }
}
