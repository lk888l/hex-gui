use super::*;

fn config(side: CordExitSide) -> ChassisConfig {
    let mut c = ChassisConfig {
        cord_exit_side: side,
        max_wheel_speed_mps: 1.0,
        ..Default::default()
    };
    // HopeA3's reported local launch, stop and 1.0 m switch settings.
    c.follow.behind_start_cord_length_m = 0.55;
    c.follow.behind_target_cord_length_m = 0.65;
    c.follow.behind_stop_cord_length_m = 0.9;
    c.follow.behind_ahead_switch_cord_length_m = 1.0;
    c.follow.pace.target_length_m = 0.8;
    c.follow.pace.stop_margin_m = 0.2;
    c.follow.behind_speed.stop_deceleration_mps2 = 0.2;
    c.follow.behind_speed.switch_deceleration_mps2 = 0.3;
    c.validate().unwrap();
    c
}

fn sample(length: f64) -> FollowInput {
    FollowInput {
        cord_length_m: Some(length),
        cord_speed_mps: Some(0.0),
        encoder_angle_deg: Some(0.0),
        ..Default::default()
    }
}

#[test]
fn running_pace_switches_at_threshold_only_after_command_and_measured_motion_stop() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        // The switch wins both at and below the pace stop guard.
        for margin in [0.2, 0.4] {
            let mut c = config(side);
            c.follow.pace.stop_margin_m = margin;
            let mut controller = FollowController::default();
            let mut wheels = 0.3 * side.sign();
            controller.step(sample(0.8), wheels, 0.01, &c);
            let (following, _) = controller.step(sample(0.999), wheels, 0.01, &c);
            assert!(following.linear_mps * side.sign() > 0.0);
            assert!(controller.status().pace_active && controller.status().pace_tracking);
            assert_eq!(controller.status().travel_mode, Some(WalkMode::Behind));

            for _ in 0..50 {
                let (target, next) = controller.step_with_feedback(
                    sample(1.0),
                    wheels,
                    Some(0.3 * side.sign()),
                    0.01,
                    &c,
                );
                assert_eq!(target.linear_mps, 0.0);
                assert_eq!(
                    controller.status().braking,
                    Some(BehindBraking::AheadSwitch)
                );
                assert_eq!(
                    controller.status().acceleration_mps2,
                    c.follow.pace.deceleration_mps2
                );
                assert!(!controller.status().pace_active);
                assert!(
                    next * side.sign() >= 0.0,
                    "never reverse before a measured stop"
                );
                assert!(next.abs() <= wheels.abs());
                wheels = next;
            }
            assert_eq!(wheels, 0.0);
            for feedback in [
                None,
                Some(f64::NAN),
                Some(f64::INFINITY),
                Some(-0.3 * side.sign()),
            ] {
                let (target, next) =
                    controller.step_with_feedback(sample(1.05), wheels, feedback, 0.01, &c);
                assert_eq!(target.linear_mps, 0.0);
                assert_eq!(next, 0.0);
                assert_eq!(
                    controller.status().braking,
                    Some(BehindBraking::AheadSwitch)
                );
            }

            for length in [0.99, 0.8, 0.04, 1.0] {
                let (target, next) =
                    controller.step_with_feedback(sample(length), wheels, Some(0.1), 0.01, &c);
                assert_eq!(target.linear_mps, 0.0);
                assert_eq!(next, 0.0);
                assert_eq!(
                    controller.status().braking,
                    Some(BehindBraking::AheadSwitch)
                );
                assert_eq!(
                    controller.status().acceleration_mps2,
                    c.follow.pace.deceleration_mps2
                );
            }

            // Equality enters Ahead but has no length error to command motion.
            let (at_threshold, next) =
                controller.step_with_feedback(sample(1.0), wheels, Some(0.0), 0.01, &c);
            assert_eq!(controller.status().braking, None);
            assert_eq!(controller.status().travel_mode, Some(WalkMode::Ahead));
            assert!(!controller.status().pace_active && !controller.status().pace_tracking);
            assert_eq!(at_threshold.linear_mps, 0.0);
            assert_eq!(next, 0.0);

            let (ahead, next) = controller.step(sample(1.05), next, 0.01, &c);
            assert_eq!(ahead.linear_mps, compute(sample(1.05), &c).linear_mps);
            assert!(ahead.linear_mps * side.sign() < 0.0);
            assert_eq!(next, -c.acceleration_mps2 * 0.01 * side.sign());
            let (_, continuing) = controller.step(sample(1.05), next, 0.01, &c);
            assert!(continuing.abs() > next.abs());
            assert_eq!(controller.status().braking, None);
            assert_eq!(controller.status().travel_mode, Some(WalkMode::Ahead));
        }
    }
}

#[test]
fn running_pace_switch_upgrades_stop_braking_and_cord_bounce_does_not_release_it() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = config(side);
        c.follow.pace.stop_margin_m = 0.1;
        c.follow.behind_speed.switch_deceleration_mps2 = 2.0;
        let mut controller = FollowController::default();
        let mut wheels = 0.3 * side.sign();
        controller.step(sample(0.8), wheels, 0.01, &c);
        for (length, brake, rate) in [
            (0.9, BehindBraking::StopLine, 1.2),
            (1.0, BehindBraking::AheadSwitch, 2.0),
            (0.99, BehindBraking::AheadSwitch, 2.0),
            (0.8, BehindBraking::AheadSwitch, 2.0),
        ] {
            let (target, next) = controller.step(sample(length), wheels, 0.01, &c);
            assert_eq!(controller.status().braking, Some(brake));
            assert_eq!(controller.status().acceleration_mps2, rate);
            assert_eq!(target.linear_mps, 0.0);
            assert!((next - wheels + rate * 0.01 * side.sign()).abs() < 1e-12);
            wheels = next;
        }
        for _ in 0..50 {
            let (target, next) =
                controller.step_with_feedback(sample(0.8), wheels, Some(0.1), 0.01, &c);
            assert_eq!(target.linear_mps, 0.0);
            assert_eq!(
                controller.status().braking,
                Some(BehindBraking::AheadSwitch)
            );
            wheels = next;
        }
        assert_eq!(wheels, 0.0);
        let (held, _) = controller.step_with_feedback(sample(0.8), wheels, Some(0.0), 0.01, &c);
        assert_eq!(held.linear_mps, 0.0);
        assert_eq!(controller.status().braking, None);
        assert_eq!(controller.status().travel_mode, Some(WalkMode::Behind));
        assert!(controller.status().pace_tracking);
        // Let the pace filter observe a fresh approach after the stopped sample.
        for tick in 1..=50 {
            let (_, next) = controller.step(
                FollowInput {
                    cord_speed_mps: Some(-0.2),
                    ..sample(0.8 - tick as f64 * 0.002)
                },
                wheels,
                0.01,
                &c,
            );
            assert!(controller.status().pace_active);
            wheels = next;
        }
        assert!(wheels * side.sign() > 0.0);
    }
}

#[test]
fn editing_the_switch_threshold_takes_effect_during_pace_tracking() {
    let mut c = config(CordExitSide::PassiveAxle);
    c.follow.behind_ahead_switch_cord_length_m = 1.5;
    c.follow.pace.stop_margin_m = 0.5;
    let mut controller = FollowController::default();
    controller.step(sample(0.8), -0.3, 0.01, &c);
    controller.step(sample(1.1), -0.3, 0.01, &c);
    assert!(controller.status().pace_active && controller.status().pace_tracking);
    c.follow.behind_ahead_switch_cord_length_m = 1.0;
    c.validate().unwrap();
    let (target, next) = controller.step(sample(1.1), -0.3, 0.01, &c);
    assert_eq!(
        controller.status().braking,
        Some(BehindBraking::AheadSwitch)
    );
    assert_eq!(target.linear_mps, 0.0);
    assert!(next < 0.0 && next > -0.3);
}
