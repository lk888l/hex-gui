use super::*;

fn sample(length: f64, speed: f64) -> FollowInput {
    FollowInput {
        cord_length_m: Some(length),
        cord_speed_mps: Some(speed),
        encoder_angle_deg: Some(0.0),
        ..Default::default()
    }
}

#[test]
fn faster_gestures_in_either_direction_raise_the_ramp_with_a_bounded_ceiling() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let c = ChassisConfig {
            cord_exit_side: side,
            ..legacy_follow_config()
        };
        for speeds in [[0.15, 0.3, 0.8], [-0.04, -0.2, -0.8]] {
            let mut last_acceleration = 0.0;
            for speed in speeds {
                let mut controller = FollowController::default();
                let (_, wheels) = controller.step(sample(1.0, speed), 0.1 * side.sign(), 0.01, &c);
                let a = controller.status().acceleration_mps2;
                assert!(controller.status().adaptive_response);
                assert!(a > last_acceleration);
                assert!(a <= c.follow.behind_speed.adaptive_acceleration_mps2);
                assert!((wheels * side.sign() - 0.1 - a * 0.01).abs() < 1e-12);
                last_acceleration = a;
            }
            assert_eq!(last_acceleration, 1.2);
        }
    }
}

#[test]
fn a_high_cruise_setting_does_not_make_a_slow_gesture_launch_hard() {
    let mut c = legacy_follow_config();
    c.follow.max_speed_mps = 3.0;
    c.max_wheel_speed_mps = 3.0;
    for speed in [0.0, 0.03, -0.03] {
        let mut controller = FollowController::default();
        let (target, next) = controller.step(sample(1.0, speed), 0.0, 0.01, &c);
        assert!(target.linear_mps > 2.0);
        assert!(
            next <= 0.0032 + 1e-12,
            "small gestures retain the soft base ramp"
        );
    }
}

#[test]
fn lower_raw_target_brakes_on_the_first_tick_even_with_a_slow_speed_filter() {
    for adaptive in [false, true] {
        let mut c = legacy_follow_config();
        c.follow.behind_speed.adaptive_enabled = adaptive;
        c.follow.behind_speed.filter_ms = 5000.0;
        for released_speed in [0.0, 0.3] {
            let mut controller = FollowController::default();
            controller.step(sample(1.0, -0.5), 0.0, 0.01, &c);
            let (target, next) = controller.step(sample(1.0, released_speed), 0.18, 0.01, &c);
            if adaptive {
                assert_eq!(
                    target.speed_mps,
                    compute(sample(1.0, released_speed), &c).speed_mps
                );
                assert!(next < 0.18, "release must not keep accelerating");
                assert!(next >= target.linear_mps, "braking must not overshoot");
            } else {
                assert!(next > 0.18, "reproduces the previous filter lag");
            }
        }
    }
}

#[test]
fn acceleration_is_still_filtered_and_deadband_noise_does_not_accumulate() {
    let c = legacy_follow_config();
    let mut controller = FollowController::default();
    let mut wheels = 0.15;
    for i in 0..2000 {
        let speed = if i % 2 == 0 { 0.019 } else { -0.019 };
        let (target, next) = controller.step(sample(1.0, speed), wheels, 0.01, &c);
        assert!((target.speed_mps - 0.15).abs() < 1e-12);
        assert!((next - 0.15).abs() < 1e-12);
        wheels = next;
    }
    let (target, _) = controller.step(sample(1.0, -0.12), wheels, 0.01, &c);
    assert!(target.speed_mps < compute(sample(1.0, -0.12), &c).speed_mps);
}

#[test]
fn stop_gates_brake_even_at_zero_cord_speed_and_keep_steering() {
    let mut c = legacy_follow_config();
    c.max_wheel_speed_mps = 1.0;
    c.follow.max_speed_mps = 0.8;
    for length in [0.0, 0.75, 1.8, 2.0] {
        let mut controller = FollowController::default();
        let mut wheels = 0.7;
        for _ in 0..150 {
            let (target, next) = controller.step(sample(length, 0.0), wheels, 0.01, &c);
            assert_eq!(target.linear_mps, 0.0);
            assert!(next <= wheels);
            assert!(next >= 0.0);
            wheels = next;
        }
        assert_eq!(wheels, 0.0);
        let (target, next) = controller.step(
            FollowInput {
                encoder_angle_deg: Some(30.0),
                ..sample(length, 0.0)
            },
            wheels,
            0.01,
            &c,
        );
        assert_eq!(target.steering_angle_deg, STEERING_CENTER_DEG + 30.0);
        assert_eq!(next, 0.0);
    }
}

#[test]
fn stronger_braking_stops_at_zero_before_a_separately_limited_reverse_launch() {
    let mut c = legacy_follow_config();
    c.follow.behind_speed.adaptive_acceleration_mps2 = 0.05;
    c.follow.behind_speed.adaptive_deceleration_mps2 = 4.0;
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        c.cord_exit_side = side;
        let mut controller = FollowController::default();
        let (_, zero) = controller.step(sample(1.0, -0.5), -0.001 * side.sign(), 0.01, &c);
        assert_eq!(zero, 0.0);
        let (_, next) = controller.step(sample(1.0, -0.5), zero, 0.01, &c);
        assert!((next - 0.0005 * side.sign()).abs() < 1e-12);
    }
}

#[test]
fn live_ceiling_changes_and_disable_take_effect_without_resetting_the_run() {
    let mut c = legacy_follow_config();
    let mut controller = FollowController::default();
    let (_, first) = controller.step(sample(1.0, -0.8), 0.0, 0.01, &c);
    assert_eq!(controller.status().acceleration_mps2, 1.2);
    c.follow.behind_speed.adaptive_acceleration_mps2 = 0.05;
    let (_, second) = controller.step(sample(1.0, -0.8), first, 0.01, &c);
    assert!((second - first - 0.0005).abs() < 1e-12);
    c.follow.behind_speed.adaptive_deceleration_mps2 = 0.08;
    c.follow.behind_speed.deceleration_mps2 = 0.04;
    let (_, braking) = controller.step(sample(1.799, 0.8), second, 0.01, &c);
    assert!((second - braking - 0.0008).abs() < 1e-12);
    c.follow.behind_speed.adaptive_enabled = false;
    let (_, legacy) = controller.step(sample(1.0, -0.8), 0.0, 0.01, &c);
    assert!(!controller.status().adaptive_response);
    assert_eq!(legacy, 0.003);
    c.follow.behind_speed.adaptive_enabled = true;
    c.follow.behind_speed.enabled = false;
    controller.step(sample(1.0, -0.8), 0.0, 0.01, &c);
    assert!(!controller.status().adaptive_response);
}

#[test]
fn adaptive_ramps_preserve_motion_limits_through_turns_and_scheduling_gaps() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for max_speed in [0.2, 1.0, 7.0] {
            let mut c = ChassisConfig {
                cord_exit_side: side,
                max_wheel_speed_mps: max_speed,
                ..legacy_follow_config()
            };
            c.follow.max_speed_mps = max_speed;
            let mut controller = FollowController::default();
            let mut previous = 0.0;
            for i in 0..2500 {
                let phase = i as f64 * 0.015;
                let input = FollowInput {
                    encoder_angle_deg: Some(phase.sin() * 85.0),
                    ..sample(1.2 + phase.cos() * 0.55, (phase * 1.3).sin())
                };
                let dt = if i % 40 == 0 { 0.2_f64 } else { 0.01 };
                let v0 = previous;
                let (command, next) =
                    controller.step_with_feedback(input, previous, Some(v0), dt, &c);
                let v = next;
                assert!(
                    (v - v0).abs() <= controller.status().acceleration_mps2 * dt.min(0.03) + 1e-10
                );
                assert_eq!(command.steering_angle_deg, steering_angle(input, &c));
                assert!(next.abs() <= max_speed + 1e-10);
                previous = next;
            }
            let (_, next) = controller.step(FollowInput::default(), previous, 0.01, &c);
            assert_eq!(next, 0.0);
            assert!(!controller.status().adaptive_response);
        }
    }
}

#[test]
fn ordinary_ahead_and_far_ahead_keep_their_existing_ramps() {
    let mut c = legacy_follow_config();
    for (mode, length) in [(WalkMode::Ahead, 1.4), (WalkMode::Behind, 2.4)] {
        c.mode = mode;
        let mut controller = FollowController::default();
        let (_, next) = controller.step(sample(length, 0.3), 0.0, 0.01, &c);
        assert!(!controller.status().adaptive_response);
        assert_eq!(next, -0.003);
    }
}

/// Ideal command response, not a vehicle dynamics or traction simulation.
#[test]
fn response_replay_compares_launch_retraction_and_stop_with_the_previous_ramps() {
    fn reach(adaptive: bool, length: f64, speed: f64, initial: f64, threshold: f64) -> (f64, f64) {
        let mut c = legacy_follow_config();
        c.follow.max_speed_mps = 0.8;
        c.max_wheel_speed_mps = 1.0;
        c.follow.behind_speed.adaptive_enabled = adaptive;
        let mut controller = FollowController::default();
        let mut wheels = initial;
        let mut distance = 0.0;
        for tick in 1..=2000 {
            let (_, next) = controller.step(sample(length, speed), wheels, 0.01, &c);
            distance += (wheels + next) * 0.005;
            wheels = next;
            if (threshold >= initial && wheels >= threshold - 1e-12)
                || (threshold < initial && wheels <= threshold + 1e-12)
            {
                return (tick as f64 * 0.01, distance);
            }
        }
        panic!("response failed to settle");
    }
    for (name, length, speed, initial, threshold) in [
        ("payout launch to 90%", 1.0, 0.3, 0.0, 0.432),
        ("retraction speed-up to 90%", 1.0, -0.3, 0.48, 0.714),
        ("short-cord stop to zero", 0.75, 0.0, 0.74, 0.0),
    ] {
        let old = reach(false, length, speed, initial, threshold);
        let new = reach(true, length, speed, initial, threshold);
        println!(
            "{name}: previous {:.2}s / {:.3}m, adaptive {:.2}s / {:.3}m",
            old.0, old.1, new.0, new.1
        );
        assert!(
            new.0 < old.0 * 0.6,
            "{name}: meaningful reduction in command lag"
        );
        if threshold == 0.0 {
            assert!(new.1 < old.1 * 0.6);
        }
    }
}
