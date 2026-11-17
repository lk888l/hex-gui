use super::*;

fn sample(length: f64, speed: f64) -> FollowInput {
    FollowInput {
        encoder_sample_time_s: None,
        pull_back_speed_mps: None,
        cord_length_m: Some(length),
        cord_speed_mps: Some(speed),
        encoder_angle_deg: Some(0.0),
    }
}

#[test]
fn cord_speed_has_continuous_monotonic_response_with_headroom() {
    let c = legacy_follow_config();
    // Independent expected values: 60% payout, 75% neutral, then 0.5 gain
    // on retraction beyond 0.02 m/s, capped BEFORE the length envelope.
    for (speed, expected) in [
        (0.3, 0.12),
        (0.0, 0.15),
        (-0.06, 0.17),
        (-0.12, 0.2),
        (-100.0, 0.2),
    ] {
        assert!((compute(sample(1.2, speed), &c).speed_mps - expected).abs() < 1e-12);
        assert!((compute(sample(1.5, speed), &c).speed_mps - expected / 2.0).abs() < 1e-12);
    }
    for speed in [-0.02, -0.01, 0.0, 0.01, 0.02] {
        assert!((compute(sample(1.2, speed), &c).speed_mps - 0.15).abs() < 1e-12);
    }
    let mut last = 0.2;
    for i in -1000..=1000 {
        let next = compute(sample(1.2, i as f64 / 1000.0), &c).speed_mps;
        assert!(next <= last + 1e-12);
        assert!((next - last).abs() <= 0.000501);
        last = next;
    }
}

#[test]
fn gates_and_ahead_are_unaffected_and_disabled_restores_length_only() {
    let mut c = legacy_follow_config();
    for enabled in [true, false] {
        c.follow.behind_speed.enabled = enabled;
        for length in [0.0, 0.75, 1.8, 2.0, 2.2] {
            for speed in [-100.0, 0.0, 100.0] {
                let command = compute(
                    FollowInput {
                        encoder_angle_deg: Some(45.0),
                        ..sample(length, speed)
                    },
                    &c,
                );
                assert_eq!(command.linear_mps, 0.0);
                assert_eq!(command.linear_mps, 0.0);
                assert_eq!(command.steering_angle_deg, STEERING_CENTER_DEG + 45.0);
            }
        }
        assert!((compute(sample(2.3, 0.0), &c).linear_mps + 0.09).abs() < 1e-12);
    }
    for speed in [-100.0, 0.0, 100.0] {
        assert!((compute(sample(1.5, speed), &c).speed_mps - 0.1).abs() < 1e-12);
    }
    c.follow.behind_speed.enabled = true;
    c.follow.max_speed_mps = 0.0;
    assert_eq!(compute(sample(1.2, -100.0), &c).speed_mps, 0.0);
    c.follow.max_speed_mps = 0.2;
    c.mode = WalkMode::Ahead;
    for speed in [-0.1, 0.0, 0.1] {
        c.follow.behind_speed.enabled = true;
        let enabled = compute(sample(1.1, speed), &c);
        c.follow.behind_speed.enabled = false;
        let disabled = compute(sample(1.1, speed), &c);
        assert_eq!(enabled.linear_mps, disabled.linear_mps);
    }
}

#[test]
fn payout_accelerates_slowly_retraction_responds_and_release_decelerates() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = legacy_follow_config();
        c.cord_exit_side = side;
        c.follow.behind_speed.filter_ms = 0.0;
        c.follow.behind_speed.adaptive_enabled = false;
        let mut controller = FollowController::default();
        for (speed, expected_step) in [(0.3, 0.001), (-0.1, 0.003), (0.0, 0.003)] {
            let (_, wheels) = controller.step(sample(1.2, speed), 0.0, 0.01, &c);
            assert!((wheels - expected_step * side.sign()).abs() < 1e-12);
        }
        c.follow.behind_speed.deceleration_mps2 = 0.2;
        let fast = 0.2 * side.sign();
        let (_, slower) = controller.step(sample(1.2, -0.03), fast, 0.01, &c);
        assert!((slower - 0.198 * side.sign()).abs() < 1e-12);
        let (_, stopped_band) = controller.step(sample(1.8, -2.0), fast, 0.01, &c);
        assert!((stopped_band - 0.185 * side.sign()).abs() < 1e-12);
        c.acceleration_mps2 = 0.05;
        let (_, capped) = controller.step(sample(1.2, -0.1), 0.0, 0.01, &c);
        assert!((capped - 0.0005 * side.sign()).abs() < 1e-12);
    }
}

#[test]
fn filter_smooths_speed_but_never_delays_length_gates_or_leaks_across_segments() {
    let c = legacy_follow_config();
    let mut controller = FollowController::default();
    controller.step(sample(1.2, 0.0), 0.0, 0.01, &c);
    let (filtered, _) = controller.step(sample(1.2, -0.12), 0.0, 0.01, &c);
    assert!(filtered.speed_mps < compute(sample(1.2, -0.12), &c).speed_mps);
    for _ in 0..120 {
        controller.step(sample(1.2, -0.12), 0.0, 0.01, &c);
    }
    assert!(controller.filtered_speed_mps.unwrap() < -0.119);
    for length in [0.75, 1.8, 2.2] {
        let (stopped, _) = controller.step(sample(length, -0.12), 0.0, 0.01, &c);
        assert_eq!(stopped.speed_mps, 0.0);
        assert!(controller.filtered_speed_mps.is_none());
    }
    let (restarted, _) = controller.step(sample(1.2, 0.3), 0.0, 0.01, &c);
    assert_eq!(restarted.speed_mps, 0.12);
    let (far, _) = controller.step(sample(2.3, -0.1), 0.0, 0.01, &c);
    assert_eq!(far.speed_mps, compute(sample(2.3, -0.1), &c).speed_mps);
    assert!(controller.filtered_speed_mps.is_none());
    controller.step(sample(1.2, -0.3), 0.0, 0.01, &c);
    let (_, wheels) = controller.step(FollowInput::default(), 0.2, 0.01, &c);
    assert_eq!(wheels, 0.0);
    assert!(controller.filtered_speed_mps.is_none());
}

#[test]
fn steady_zero_speed_settles_without_accumulating_boost_and_live_tuning_applies() {
    let mut c = legacy_follow_config();
    let mut controller = FollowController::default();
    let mut wheels = 0.0;
    for _ in 0..1000 {
        (_, wheels) = controller.step(sample(1.2, 0.0), wheels, 0.01, &c);
    }
    assert!((wheels - 0.15).abs() < 1e-12);
    c.follow.behind_speed.payout_speed_ratio = 0.4;
    c.follow.behind_speed.neutral_boost_ratio = 0.1;
    let (retuned, _) = controller.step(sample(1.2, 0.0), wheels, 0.01, &c);
    assert!((retuned.speed_mps - 0.1).abs() < 1e-12);
    c.follow.behind_speed.enabled = false;
    let (disabled, next) = controller.step(sample(1.2, 0.0), wheels, 0.01, &c);
    assert_eq!(disabled.speed_mps, 0.2);
    assert!((next - 0.153).abs() < 1e-12);
    assert!(controller.filtered_speed_mps.is_none());
}

#[test]
fn old_and_partial_settings_get_defaults_custom_values_roundtrip_and_invalid_values_fail() {
    let mut saved = serde_json::to_value(legacy_follow_config()).unwrap();
    saved["follow"]
        .as_object_mut()
        .unwrap()
        .remove("behindSpeed");
    let restored: ChassisConfig = serde_json::from_value(saved.clone()).unwrap();
    restored.validate().unwrap();
    assert_eq!(
        restored.follow.behind_speed,
        BehindSpeedParameters::default()
    );
    saved["follow"]["behindSpeed"] = serde_json::json!({"enabled": false, "retractGain": 0.7});
    let restored: ChassisConfig = serde_json::from_value(saved.clone()).unwrap();
    assert!(!restored.follow.behind_speed.enabled);
    assert_eq!(restored.follow.behind_speed.retract_gain, 0.7);
    assert_eq!(restored.follow.behind_speed.filter_ms, 120.0);
    assert_eq!(restored.follow.behind_speed.stop_deceleration_mps2, 1.5);
    assert_eq!(restored.follow.behind_speed.switch_deceleration_mps2, 3.0);
    assert!(restored.follow.behind_speed.adaptive_enabled);
    assert_eq!(restored.follow.behind_speed.adaptive_gain, 2.0);
    assert_eq!(restored.follow.behind_speed.adaptive_acceleration_mps2, 1.2);
    assert_eq!(restored.follow.behind_speed.adaptive_deceleration_mps2, 1.2);
    assert!(restored.follow.behind_speed.payout_stop_enabled);
    assert_eq!(restored.follow.behind_speed.payout_stop_speed_mps, 0.5);
    assert_eq!(restored.follow.behind_speed.payout_stop_confirm_ms, 40.0);
    assert_eq!(restored.follow.behind_speed.payout_stop_min_payout_m, 0.02);
    assert_eq!(
        restored.follow.behind_speed.payout_stop_deceleration_mps2,
        1.2
    );
    assert_eq!(restored.follow.behind_speed.pull_back_min_retract_m, 0.01);
    assert_eq!(restored.follow.behind_speed.pull_back_length_gain, 2.0);
    assert_eq!(
        restored.follow.behind_speed.pull_back_deceleration_mps2,
        0.8
    );
    let again: ChassisConfig =
        serde_json::from_value(serde_json::to_value(&restored).unwrap()).unwrap();
    assert_eq!(restored.follow.behind_speed, again.follow.behind_speed);
    for (field, upper) in [
        ("payoutSpeedRatio", 1.0),
        ("neutralBoostRatio", 1.0),
        ("retractGain", 10.0),
        ("speedDeadbandMps", 3.0),
        ("payoutBlendMps", 3.0),
        ("filterMs", 5000.0),
        ("payoutAccelerationMps2", 4.0),
        ("responseAccelerationMps2", 4.0),
        ("decelerationMps2", 4.0),
        ("stopDecelerationMps2", 4.0),
        ("switchDecelerationMps2", 4.0),
        ("adaptiveGain", 10.0),
        ("adaptiveAccelerationMps2", 4.0),
        ("adaptiveDecelerationMps2", 4.0),
        ("payoutStopSpeedMps", 3.0),
        ("payoutStopConfirmMs", 1000.0),
        ("payoutStopMinPayoutM", 0.5),
        ("payoutStopDecelerationMps2", 4.0),
        ("pullBackAccelerationMps2", 4.0),
        ("pullBackAccelerationGain", 10.0),
        ("pullBackMinRetractM", 0.5),
        ("pullBackLengthGain", 10.0),
        ("pullBackDecelerationMps2", 4.0),
    ] {
        for value in [-0.001, upper + 0.001] {
            let mut invalid = saved.clone();
            invalid["follow"]["behindSpeed"][field] = value.into();
            let invalid: ChassisConfig = serde_json::from_value(invalid).unwrap();
            assert!(invalid.validate().is_err(), "{field}={value}");
        }
    }
    let mut invalid = legacy_follow_config();
    invalid.follow.behind_speed.payout_blend_mps = 0.0;
    assert!(invalid.validate().is_err());
    invalid.follow.behind_speed = BehindSpeedParameters::default();
    invalid.follow.behind_speed.retract_gain = f64::NAN;
    assert!(invalid.validate().is_err());
}
