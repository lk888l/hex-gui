use super::*;

fn sample(length: f64, speed: f64) -> FollowInput {
    FollowInput {
        cord_length_m: Some(length),
        cord_speed_mps: Some(speed),
        encoder_angle_deg: Some(0.0),
        ..Default::default()
    }
}

// Reproduce the reported 5 cm stop-to-switch band and slow launch setting.
fn config(side: CordExitSide) -> ChassisConfig {
    let mut c = ChassisConfig {
        cord_exit_side: side,
        acceleration_mps2: 0.1,
        max_wheel_speed_mps: 1.0,
        ..Default::default()
    };
    c.follow.max_speed_mps = 0.8;
    c.follow.behind_start_cord_length_m = 0.3;
    c.follow.behind_target_cord_length_m = 0.62;
    c.follow.behind_stop_cord_length_m = 0.68;
    c.follow.behind_ahead_switch_cord_length_m = 0.73;
    c.follow.behind_speed.deceleration_mps2 = 4.0;
    c.follow.behind_speed.adaptive_gain = 1.0;
    c.follow.behind_speed.adaptive_deceleration_mps2 = 1.5;
    c.follow.behind_speed.payout_stop_deceleration_mps2 = 3.0;
    c.validate().unwrap();
    c
}

#[test]
fn reported_lengths_use_independent_fixed_brakes_with_either_exit_and_cruise_switches() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for enabled in [false, true] {
            for adaptive in [false, true] {
                let mut c = config(side);
                c.follow.behind_speed.enabled = enabled;
                c.follow.behind_speed.adaptive_enabled = adaptive;
                c.follow.behind_speed.filter_ms = 5000.0;
                for (length, rate, braking) in [
                    (0.68, 1.5, BehindBraking::StopLine),
                    (0.729, 1.5, BehindBraking::StopLine),
                    (0.73, 3.0, BehindBraking::AheadSwitch),
                    (0.8, 3.0, BehindBraking::AheadSwitch),
                ] {
                    let mut controller = FollowController::default();
                    let (target, next) =
                        controller.step(sample(length, 0.2), 0.5 * side.sign(), 0.01, &c);
                    assert_eq!(target.linear_mps, 0.0);
                    assert_eq!(controller.status().braking, Some(braking));
                    assert_eq!(controller.status().acceleration_mps2, rate);
                    assert!(!controller.status().adaptive_response);
                    assert!((next * side.sign() - (0.5 - rate * 0.01)).abs() < 1e-12);
                }
            }
        }
    }
}

#[test]
fn ordinary_braking_is_not_capped_by_launch_or_a_lower_adaptive_ceiling() {
    for enabled in [false, true] {
        for adaptive in [false, true] {
            let mut c = config(CordExitSide::DrivenAxle);
            c.follow.pace.enabled = false; // This asserts the legacy adaptive ramp.
            c.follow.behind_speed.enabled = enabled;
            c.follow.behind_speed.adaptive_enabled = adaptive;
            let mut controller = FollowController::default();
            let (_, next) = controller.step(sample(0.65, 0.2), 0.5, 0.01, &c);
            assert_eq!(controller.status().braking, None);
            assert_eq!(controller.status().acceleration_mps2, 4.0);
            assert!((next - 0.46).abs() < 1e-12);
        }
    }
}

#[test]
fn stop_line_braking_survives_return_to_cruise_until_command_and_feedback_stop() {
    let mut c = config(CordExitSide::DrivenAxle);
    c.follow.pace.enabled = false; // Includes the legacy automatic cruise restart.
    c.follow.behind_speed.adaptive_enabled = false;
    let mut controller = FollowController::default();
    let (_, mut wheels) = controller.step(sample(0.68, 0.2), 0.5, 0.01, &c);
    for _ in 0..40 {
        let (target, next) =
            controller.step_with_feedback(sample(0.5, -0.2), wheels, Some(0.1), 0.01, &c);
        assert_eq!(target.linear_mps, 0.0);
        assert_eq!(controller.status().braking, Some(BehindBraking::StopLine));
        assert_eq!(controller.status().acceleration_mps2, 1.5);
        assert!((next - (wheels - 0.015).max(0.0)).abs() < 1e-12);
        wheels = next;
    }
    assert_eq!(wheels, 0.0);
    let (target, next) =
        controller.step_with_feedback(sample(0.5, -0.2), wheels, Some(0.0), 0.01, &c);
    assert_eq!(controller.status().braking, None);
    assert!(target.linear_mps > 0.0);
    assert_eq!(next, 0.001);
}

#[test]
fn switch_waits_for_physical_stop_then_launches_at_normal_acceleration() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let c = config(side);
        let mut controller = FollowController::default();
        let (_, zero) = controller.step(sample(0.8, 0.2), 0.005 * side.sign(), 0.01, &c);
        assert_eq!(zero, 0.0, "braking cannot launch in the other direction");
        for feedback in [
            Some(0.3),
            Some(-0.3),
            None,
            Some(f64::NAN),
            Some(f64::INFINITY),
        ] {
            let (target, next) =
                controller.step_with_feedback(sample(0.8, 0.2), zero, feedback, 0.01, &c);
            assert_eq!(
                controller.status().braking,
                Some(BehindBraking::AheadSwitch)
            );
            assert_eq!(controller.status().acceleration_mps2, 3.0);
            assert_eq!(target.linear_mps, 0.0);
            assert_eq!(next, zero);
        }
        let (target, next) =
            controller.step_with_feedback(sample(0.8, 0.2), zero, Some(0.019), 0.01, &c);
        assert_eq!(controller.status().braking, None);
        assert!(target.linear_mps * side.sign() < 0.0);
        assert_eq!(next, -0.001 * side.sign());
        let (_, continuing) = controller.step(sample(0.8, 0.2), next, 0.01, &c);
        assert_eq!(controller.status().braking, None);
        assert_eq!(continuing, -0.002 * side.sign());
    }
}

#[test]
fn measured_behind_motion_or_unknown_feedback_blocks_ahead_even_with_zero_command() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let c = config(side);
        for feedback in [Some(0.3 * side.sign()), None, Some(f64::NAN)] {
            let mut controller = FollowController::default();
            let (target, wheels) =
                controller.step_with_feedback(sample(0.8, 0.2), 0.0, feedback, 0.01, &c);
            assert_eq!(
                controller.status().braking,
                Some(BehindBraking::AheadSwitch)
            );
            assert_eq!(target.linear_mps, 0.0);
            assert_eq!(wheels, 0.0);
        }
        let mut controller = FollowController::default();
        let (_, wheels) = controller.step(sample(0.8, 0.2), 0.0, 0.01, &c);
        assert_eq!(controller.status().braking, None);
        assert_eq!(wheels, -0.001 * side.sign());
    }
}

#[test]
fn crossing_the_switch_upgrades_the_brake_and_cord_bounce_cannot_downgrade_it() {
    let c = config(CordExitSide::DrivenAxle);
    let mut controller = FollowController::default();
    let mut wheels = 0.5;
    for (length, rate) in [
        (0.68, 1.5),
        (0.73, 3.0),
        (0.72, 3.0),
        (0.67, 3.0),
        (0.5, 3.0),
    ] {
        let (target, next) = controller.step(sample(length, -0.2), wheels, 0.01, &c);
        assert_eq!(target.linear_mps, 0.0);
        assert_eq!(controller.status().acceleration_mps2, rate);
        assert!((wheels - next - rate * 0.01).abs() < 1e-12);
        wheels = next;
    }
}

#[test]
fn fixed_brakes_ignore_adaptive_caps_accept_live_edits_and_cap_scheduler_gaps() {
    let mut c = config(CordExitSide::DrivenAxle);
    c.follow.behind_speed.adaptive_deceleration_mps2 = 0.01;
    let mut controller = FollowController::default();
    let (_, first) = controller.step(sample(0.68, 0.0), 0.5, 1.0, &c);
    assert!((first - 0.455).abs() < 1e-12);
    c.follow.behind_speed.stop_deceleration_mps2 = 2.0;
    c.follow.behind_speed.switch_deceleration_mps2 = 0.8;
    let (_, next) = controller.step(sample(0.73, 0.0), first, 0.01, &c);
    assert_eq!(controller.status().acceleration_mps2, 2.0);
    assert!((first - next - 0.02).abs() < 1e-12);
    c.follow.behind_speed.switch_deceleration_mps2 = 3.5;
    controller.step(sample(0.72, 0.0), next, 0.01, &c);
    assert_eq!(controller.status().acceleration_mps2, 3.5);
}

#[test]
fn stronger_tug_braking_is_kept_across_both_lines_until_a_measured_stop() {
    let mut c = config(CordExitSide::DrivenAxle);
    c.follow.pace.enabled = false; // The tug gesture belongs to legacy cruise.
    c.follow.behind_speed.payout_stop_deceleration_mps2 = 4.0;
    let mut controller = FollowController::default();
    for tick in 0..4 {
        controller.step(sample(0.4 + tick as f64 * 0.01, 1.0), 0.5, 0.01, &c);
    }
    assert!(controller.status().payout_stop);
    for (length, previous, measured) in [
        (0.68, 0.5, 0.5),
        (0.8, 0.3, 0.3),
        (0.8, 0.005, 0.0),
        (0.4, 0.005, 0.0),
        (0.8, 0.0, 0.1),
    ] {
        let (target, _) =
            controller.step_with_feedback(sample(length, 0.0), previous, Some(measured), 0.01, &c);
        assert!(controller.status().payout_stop);
        assert_eq!(controller.status().acceleration_mps2, 4.0);
        assert_eq!(target.linear_mps, 0.0);
    }
    let (target, next) = controller.step(sample(0.8, 0.2), 0.0, 0.01, &c);
    assert!(!controller.status().payout_stop);
    assert_eq!(controller.status().braking, None);
    assert!(target.linear_mps < 0.0);
    assert_eq!(next, -0.001);
}

#[test]
fn ordinary_braking_cannot_spend_the_remaining_tick_accelerating_through_zero() {
    let mut c = config(CordExitSide::DrivenAxle);
    c.follow.pace.enabled = false;
    c.follow.behind_speed.adaptive_enabled = false;
    let mut controller = FollowController::default();
    let (_, zero) = controller.step(sample(0.5, 0.0), -0.005, 0.01, &c);
    assert_eq!(zero, 0.0);
    let (_, launch) = controller.step(sample(0.5, 0.0), zero, 0.01, &c);
    assert_eq!(launch, 0.001);
}

#[test]
fn braking_keeps_steering_position_and_drive_limits() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let c = config(side);
        let mut controller = FollowController::default();
        let mut wheels = 0.5 * side.sign();
        for tick in 0..100 {
            let dt: f64 = if tick % 10 == 0 { 0.2 } else { 0.01 };
            let input = FollowInput {
                encoder_angle_deg: Some(35.0),
                ..sample(if tick < 5 { 0.69 } else { 0.8 }, 0.2)
            };
            let v0 = wheels;
            let (target, next) = controller.step(input, wheels, dt, &c);
            let v = next;
            assert_eq!(target.steering_angle_deg, STEERING_CENTER_DEG + 35.0);
            assert!(next.abs().max(next.abs()) <= c.max_wheel_speed_mps + 1e-12);
            assert!((v - v0).abs() <= controller.status().acceleration_mps2 * dt.min(0.03) + 1e-12);
            if controller.status().braking.is_some() {
                assert_eq!(target.linear_mps, 0.0);
                assert!(v.abs() <= v0.abs() + 1e-12);
                assert!(v * side.sign() >= -1e-12);
            }
            wheels = next;
        }
        assert!(wheels + wheels != 0.0, "normal Ahead motion resumes");
    }
}

#[test]
fn reset_and_invalid_input_clear_braking_memory() {
    let c = config(CordExitSide::DrivenAxle);
    let mut controller = FollowController::default();
    controller.step(sample(0.8, 0.2), 0.5, 0.01, &c);
    controller.reset();
    assert_eq!(controller.status().braking, None);
    controller.step(sample(0.8, 0.2), 0.5, 0.01, &c);
    let (_, next) = controller.step(FollowInput::default(), 0.5, 0.01, &c);
    assert_eq!(next, 0.0);
    assert_eq!(controller.status().braking, None);
}
