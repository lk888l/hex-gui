use super::*;
use std::collections::VecDeque;

fn mpu_config() -> ChassisConfig {
    let mut c: ChassisConfig =
        serde_json::from_str(include_str!("../test/fixtures/paceMpuChassis.json")).unwrap();
    // Keep the captured MPU fixture unchanged. Its 2.5 m/s wheel setting was
    // already capped to 200 RPM at runtime; use that effective limit here.
    c.max_wheel_speed_mps = c.max_wheel_speed_mps.min(c.max_output_speed_mps());
    c.validate().unwrap();
    c
}

fn sample(c: &ChassisConfig, length: f64, relative: f64) -> FollowInput {
    FollowInput {
        cord_length_m: Some(length),
        cord_speed_mps: Some(relative),
        encoder_angle_deg: Some(c.follow.encoder_center_deg),
        ..Default::default()
    }
}

#[test]
fn mpu_launch_settings_are_preserved_and_length_hold_has_its_own_reference() {
    let c = mpu_config();
    assert_eq!(c.follow.behind_start_cord_length_m, 0.1);
    assert_eq!(c.follow.behind_target_cord_length_m, 0.4);
    assert_eq!(c.follow.behind_stop_cord_length_m, 0.68);
    assert_eq!(c.follow.behind_ahead_switch_cord_length_m, 0.7);
    assert_eq!(c.follow.pace.target_length_m, 0.68);
    assert!((c.follow.pace.stop_length(&c) - 1.18).abs() < 1e-12);
    let mut controller = FollowController::default();
    for _ in 0..100 {
        let (target, next) = controller.step(sample(&c, 0.69, 0.0), 0.0, 0.01, &c);
        assert_eq!(target.speed_mps, 0.0);
        assert_eq!(next, 0.0);
        assert!(controller.status().waiting_for_pull_back);
        assert!(!controller.status().pace_tracking);
    }
    let mut length = 0.69;
    let mut wheels = 0.0;
    let mut launch = None;
    for tick in 0..20 {
        length -= 0.15 * 0.01;
        let (_, next) =
            controller.step_with_feedback(sample(&c, length, -0.15), wheels, Some(0.0), 0.01, &c);
        if next > 0.0 && launch.is_none() {
            launch = Some(tick);
        }
        if tick < 7 {
            assert_eq!(next, 0.0, "keep restart confirmation");
        }
        wheels = next;
    }
    assert!(
        launch.is_some_and(|tick| (7..16).contains(&tick)),
        "launch={launch:?}"
    );
    assert!(controller.status().pace_tracking && controller.status().pace_active);
}

#[test]
fn distance_correction_recovers_either_side_of_the_target_and_is_neutral_near_it() {
    let c = mpu_config();
    for length in [0.45, 0.60, 0.65, 0.68, 0.71, 0.8, 1.0] {
        let mut controller = FollowController::default();
        controller.step(sample(&c, 0.6, 0.0), 0.3, 0.01, &c);
        let (target, _) = controller.step(sample(&c, length, 0.0), 0.3, 0.01, &c);
        assert!(controller.status().pace_tracking && controller.status().pace_active);
        assert_eq!(controller.status().travel_mode, Some(WalkMode::Behind));
        if length < 0.64 {
            assert!(target.speed_mps > 0.3);
        } else if length > 0.72 {
            assert!(target.speed_mps < 0.3 && target.speed_mps > 0.0);
        } else {
            assert!((target.speed_mps - 0.3).abs() < 1e-12);
        }
        assert!(controller.status().pace_correction_mps.abs() <= c.follow.pace.max_correction_mps);
    }
    // A distance error cannot launch a stationary person, including after a run.
    for length in [0.2, 0.5, 0.68, 0.9] {
        let mut observer = pace::PaceObserver::default();
        let request = observer.update(
            sample(&c, length, 0.0),
            Some(MeasuredMotion {
                linear_mps: 0.0,
                yaw_rate_rad_s: 0.0,
            }),
            0.01,
            &c,
        );
        assert_eq!(request.speed_mps, 0.0);
    }
}

#[test]
fn running_pace_crosses_old_stop_and_switch_but_keeps_the_new_protective_stop() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = mpu_config();
        c.cord_exit_side = side;
        let mut controller = FollowController::default();
        let mut wheels = 0.3 * side.sign();
        controller.step(sample(&c, 0.6, 0.0), wheels, 0.01, &c);
        for length in [0.68, 0.7, 0.72, 0.9] {
            let (target, next) = controller.step(sample(&c, length, 0.0), wheels, 0.01, &c);
            assert!(target.linear_mps * side.sign() > 0.0);
            assert!(controller.status().pace_active);
            assert_eq!(controller.status().braking, None);
            wheels = next;
        }
        let limit = c.follow.pace.stop_length(&c);
        for _ in 0..100 {
            let (target, next) = controller.step_with_feedback(
                sample(&c, limit, 0.3),
                wheels,
                Some(0.3 * side.sign()),
                0.01,
                &c,
            );
            assert_eq!(target.speed_mps, 0.0);
            assert_eq!(controller.status().travel_mode, Some(WalkMode::Behind));
            assert_eq!(controller.status().braking, Some(BehindBraking::StopLine));
            assert!(controller.status().acceleration_mps2 >= c.follow.pace.deceleration_mps2);
            assert!(controller.status().pace_tracking);
            wheels = next;
        }
        let (target, _) =
            controller.step_with_feedback(sample(&c, limit, 0.0), wheels, Some(0.0), 0.01, &c);
        assert_eq!(target.speed_mps, 0.0);
        assert!(controller.status().waiting_for_pull_back);
        assert!(controller.status().pace_tracking);
        controller.reset();
        assert!(!controller.status().pace_tracking);
    }
}

#[test]
fn gain_and_reference_edits_or_missing_feedback_do_not_reenter_launch_switching() {
    let mut c = mpu_config();
    let mut controller = FollowController::default();
    controller.step(sample(&c, 0.6, 0.0), 0.3, 0.01, &c);
    c.follow.pace.target_length_m = 0.72;
    c.follow.pace.length_gain_per_s = 0.5;
    let (target, _) = controller.step(sample(&c, 0.72, 0.0), 0.3, 0.01, &c);
    assert!((target.linear_mps - 0.3).abs() < 1e-12);
    let (target, _) = controller.step_with_feedback(sample(&c, 0.72, 0.0), 0.3, None, 0.01, &c);
    assert_eq!(target.speed_mps, 0.0);
    assert!(controller.status().pace_tracking);
    assert_eq!(controller.status().estimated_user_speed_mps, None);
    let (target, _) = controller.step(sample(&c, 0.72, 0.0), 0.0, 0.01, &c);
    assert_eq!(target.speed_mps, 0.0);
    assert_eq!(controller.status().travel_mode, Some(WalkMode::Behind));
}

fn walking_speed(t: f64) -> f64 {
    match t {
        t if t < 1.0 => 0.0,
        t if t < 2.0 => (t - 1.0) * 0.3,
        t if t < 14.0 => 0.3,
        t if t < 16.0 => 0.3 + (t - 14.0) * 0.2,
        t if t < 30.0 => 0.7,
        t if t < 32.0 => 0.7 - (t - 30.0) * 0.25,
        t if t < 46.0 => 0.2,
        t if t < 48.0 => 0.2 + (t - 46.0) * 0.15,
        t if t < 62.0 => 0.5,
        t if t < 64.0 => 0.5 - (t - 62.0) * 0.1,
        t if t < 78.0 => 0.3,
        t if t < 80.0 => 0.3 - (t - 78.0) * 0.15,
        _ => 0.0,
    }
}

#[test]
fn mpu_closed_loop_recovers_length_after_repeated_pace_changes_without_using_up_the_cord() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for lag in [0.0_f64, 0.15, 0.35] {
            for (swing, bias) in [(0.0, 0.0), (0.025, 0.01)] {
                let mut c = mpu_config();
                c.cord_exit_side = side;
                let mut controller = FollowController::default();
                let mut wheels = 0.0;
                let mut actual = 0.0;
                let mut gap = 0.69;
                let mut previous_hand = 0.0;
                let mut queue = VecDeque::from([0.0; 3]);
                let mut max_length_error = 0.0_f64;
                let mut max_speed_error = 0.0_f64;
                let mut min_gap = gap;
                let mut max_gap = gap;
                for tick in 0..8600 {
                    let t = tick as f64 * 0.01;
                    let user = walking_speed(t);
                    let hand =
                        swing * (t * 1.6 * std::f64::consts::TAU).sin() * smoothstep(user / 0.1);
                    let relative = actual - user
                        + (hand - previous_hand) / 0.01
                        + if user > 0.0 { bias } else { 0.0 };
                    previous_hand = hand;
                    let (command, next) = controller.step_with_feedback(
                        sample(&c, gap + hand, relative),
                        wheels,
                        Some(actual * side.sign()),
                        0.01,
                        &c,
                    );
                    if (5.0..78.0).contains(&t) {
                        assert!(
                            controller.status().pace_active,
                            "lost tracking at {t}, lag={lag}, gap={gap}, swing={swing}"
                        );
                        assert!(command.linear_mps * side.sign() >= 0.0);
                        min_gap = min_gap.min(gap + hand);
                        max_gap = max_gap.max(gap + hand);
                    }
                    if [11.0..14.0, 27.0..30.0, 43.0..46.0, 59.0..62.0, 75.0..78.0]
                        .iter()
                        .any(|r| r.contains(&t))
                    {
                        max_length_error = max_length_error.max((gap + hand - 0.68).abs());
                        max_speed_error = max_speed_error.max((actual - user).abs());
                    }
                    queue.push_back(next * side.sign());
                    let delayed = queue.pop_front().unwrap();
                    actual += (if lag == 0.0 {
                        1.0
                    } else {
                        1.0 - (-0.01 / lag).exp()
                    }) * (delayed - actual);
                    gap += (actual - user) * 0.01;
                    wheels = next;
                }
                eprintln!("hold lag={lag} swing={swing} bias={bias}: stable length error={max_length_error:.4}, speed error={max_speed_error:.4}, gap={min_gap:.3}..{max_gap:.3}, stopped={actual:.5}");
                assert!(
                    max_length_error < 0.13,
                    "distance keeps drifting: {max_length_error}"
                );
                assert!(max_speed_error < 0.16, "pace mismatch: {max_speed_error}");
                assert!(actual < 0.01, "must stop when the person stops");
            }
        }
    }
}
