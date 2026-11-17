use super::*;

fn sample(angle: f64) -> FollowInput {
    FollowInput {
        encoder_angle_deg: Some(angle),
        cord_length_m: Some(1.0),
        cord_speed_mps: Some(0.0),
        ..Default::default()
    }
}

#[test]
fn steering_uses_rotated_center_and_has_no_160_degree_stop() {
    let c = ChassisConfig::default();
    for (input, expected) in [
        (0.0, 180.0),
        (45.0, 225.0),
        (-45.0, 135.0),
        (160.0, 340.0),
        (-160.0, 20.0),
        (179.0, 359.0),
        (181.0, 1.0),
        (359.0, 179.0),
        (-361.0, 179.0),
    ] {
        assert_eq!(compute(sample(input), &c).steering_angle_deg, expected);
    }
}

#[test]
fn continuous_targets_cross_boundaries_and_preserve_multiple_turns() {
    for (measured, target, expected) in [
        (359.0, 1.0, 361.0),
        (1.0, 359.0, -1.0),
        (719.0, 1.0, 721.0),
        (-719.0, 359.0, -721.0),
        (1080.0, 180.0, 1260.0),
    ] {
        let actual = nearest_steering_angle(target, measured);
        assert_eq!(actual, expected);
        assert!((actual - measured).abs() <= 180.0);
        assert!((actual - target) % 360.0 == 0.0);
    }
}

#[test]
fn steering_speed_interpolates_endpoints_and_clamps_both_travel_directions() {
    for (rest, full) in [
        (2.0, 0.5),
        (0.5, 2.0),
        (1.0, 1.0),
        (STEERING_MAX_SPEED_RAD_S, STEERING_MIN_SPEED_RAD_S),
        (STEERING_MIN_SPEED_RAD_S, STEERING_MAX_SPEED_RAD_S),
        (STEERING_MIN_SPEED_RAD_S, STEERING_MAX_FULL_SPEED_RAD_S),
    ] {
        let mut c = ChassisConfig {
            steering_speed_at_rest_rad_s: rest,
            steering_speed_at_full_speed_rad_s: full,
            max_wheel_speed_mps: 1.0,
            ..Default::default()
        };
        c.follow.max_speed_mps = 0.8;
        assert_eq!(c.steering_full_speed_mps(), 0.8);
        assert_eq!(c.steering_speed_slope(), (full - rest) / 0.8);
        for sign in [-1.0, 1.0] {
            for (v, expected) in [
                (0.0, rest),
                (0.4, (rest + full) / 2.0),
                (0.8, full),
                (2.0, full),
            ] {
                assert!((c.steering_speed_rad_s(sign * v) - expected).abs() < 1e-12);
            }
        }
        c.follow.max_speed_mps = 0.0;
        assert_eq!(c.steering_speed_slope(), 0.0);
        assert_eq!(c.steering_speed_rad_s(1.0), rest);
        c.follow.max_speed_mps = 7.0;
        assert_eq!(c.steering_full_speed_mps(), 1.0);
        c.max_wheel_speed_mps = 7.0;
        assert!(
            (c.steering_full_speed_mps() - 200.0 / 60.0 * std::f64::consts::PI * 0.120).abs()
                < 1e-12
        );
    }
}

#[test]
fn both_steering_speed_endpoints_require_finite_positive_radians_per_second() {
    let rated_speed_rad_s = 200.0 * std::f64::consts::TAU / 60.0;
    assert_eq!(STEERING_MAX_SPEED_RAD_S, rated_speed_rad_s);
    assert_eq!(STEERING_MAX_FULL_SPEED_RAD_S, rated_speed_rad_s);
    for rest in [false, true] {
        let max_speed = if rest {
            STEERING_MAX_SPEED_RAD_S
        } else {
            STEERING_MAX_FULL_SPEED_RAD_S
        };
        for speed in [0.0, -0.1, 0.009, max_speed + 0.001, f64::NAN, f64::INFINITY] {
            let mut c = ChassisConfig::default();
            if rest {
                c.steering_speed_at_rest_rad_s = speed;
            } else {
                c.steering_speed_at_full_speed_rad_s = speed;
            }
            assert!(c.validate().unwrap_err().contains("steering speed"));
        }
        for speed in [STEERING_MIN_SPEED_RAD_S, 1.0, max_speed] {
            let mut c = ChassisConfig::default();
            if rest {
                c.steering_speed_at_rest_rad_s = speed;
            } else {
                c.steering_speed_at_full_speed_rad_s = speed;
            }
            c.validate().unwrap();
        }
    }
}

#[test]
fn position_follows_at_rest_without_deadband_or_length_gates() {
    for mode in [WalkMode::Ahead, WalkMode::Behind] {
        for length in [0.0, 0.5, 0.75, 1.8, 2.0, 2.2, 3.0] {
            let mut c = ChassisConfig::default();
            c.mode = mode;
            c.follow.max_speed_mps = 0.0;
            c.follow.direction_deadband_deg = 20.0;
            let mut i = sample(5.0);
            i.cord_length_m = Some(length);
            let command = compute(i, &c);
            assert_eq!(command.linear_mps, 0.0);
            assert_eq!(command.steering_angle_deg, 185.0);
        }
    }
}
#[test]
fn center_offset_and_travel_direction_do_not_flip_steering() {
    let mut c = ChassisConfig::default();
    c.follow.encoder_center_deg = 350.0;
    for mode in [WalkMode::Ahead, WalkMode::Behind] {
        for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
            c.mode = mode;
            c.cord_exit_side = side;
            assert_eq!(compute(sample(10.0), &c).steering_angle_deg, 200.0);
        }
    }
}

#[test]
fn steering_inversion_changes_only_the_centered_motor_target() {
    for mode in [WalkMode::Ahead, WalkMode::Behind] {
        for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
            let mut c = ChassisConfig::default();
            c.mode = mode;
            c.cord_exit_side = side;
            c.follow.encoder_center_deg = 350.0;
            for length in [0.0, 1.0, 2.5] {
                for (angle, inverted_target) in [
                    (350.0, 0.0),
                    (10.0, -20.0),
                    (330.0, 20.0),
                    (169.0, -179.0),
                    (170.0, 180.0),
                    (171.0, 179.0),
                ] {
                    let mut i = sample(angle);
                    i.cord_length_m = Some(length);
                    c.steering_inverted = false;
                    let normal = compute(i, &c);
                    c.steering_inverted = true;
                    let inverted = compute(i, &c);
                    assert_eq!(
                        inverted.steering_angle_deg,
                        STEERING_CENTER_DEG + inverted_target
                    );
                    assert_eq!(
                        inverted.steering_angle_deg,
                        360.0 - normal.steering_angle_deg
                    );
                    assert_eq!(inverted.encoder_error_deg, normal.encoder_error_deg);
                    assert_eq!(inverted.linear_mps, normal.linear_mps);
                    assert_eq!(inverted.speed_mps, normal.speed_mps);
                }
            }
            let invalid = FollowInput {
                encoder_angle_deg: None,
                ..sample(0.0)
            };
            assert_eq!(compute(invalid, &c).steering_angle_deg, 0.0);
        }
    }
}

#[test]
fn turning_does_not_create_a_drive_command_or_consume_acceleration() {
    let c = ChassisConfig::default();
    let mut controller = FollowController::default();
    for angle in [-160.0, -80.0, 0.0, 80.0, 160.0] {
        let mut i = sample(angle);
        i.cord_length_m = Some(0.0);
        let (command, speed) = controller.step(i, 0.0, 0.01, &c);
        assert_eq!(command.steering_angle_deg, STEERING_CENTER_DEG + angle);
        assert_eq!(speed, 0.0);
    }
    for angle in [-160.0, 0.0, 160.0] {
        let target = ChassisCommand {
            linear_mps: 0.2,
            steering_angle_deg: angle,
            ..Default::default()
        };
        assert_eq!(slew_motion(0.0, target, 0.01, &c), 0.003);
    }
}
#[test]
fn direct_drive_120mm_conversion_and_polarity() {
    let c = ChassisConfig::default();
    assert_eq!((c.drive_node_id, c.steering_node_id), (1, 2));
    assert_eq!(c.gear_ratio, 1.0);
    assert_eq!(c.wheel_diameter_m, 0.120);
    let circumference = std::f64::consts::PI * 0.120;
    assert!((c.motor_rps(circumference, false) - 1.0).abs() < 1e-12);
    assert!((c.motor_rps(circumference, true) + 1.0).abs() < 1e-12);
    assert!((c.wheel_mps(1.0, false) - circumference).abs() < 1e-12);
}

#[test]
fn torque_limits_reject_zero_nonfinite_and_above_peak_values_on_both_axes() {
    for drive_axis in [true, false] {
        for limit in [0.0, -1.0, 0.049, 7.001, f64::NAN, f64::INFINITY] {
            let mut config = ChassisConfig::default();
            if drive_axis {
                config.drive_max_torque_nm = limit;
            } else {
                config.steering_max_torque_nm = limit;
            }
            assert!(config.validate().is_err());
        }
        for limit in [CHASSIS_MIN_TORQUE_NM, 3.0, CHASSIS_MAX_TORQUE_NM] {
            let mut config = ChassisConfig::default();
            if drive_axis {
                config.drive_max_torque_nm = limit;
            } else {
                config.steering_max_torque_nm = limit;
            }
            config.validate().unwrap();
        }
    }
}

#[test]
fn overcurrent_and_imax_settings_are_bounded_per_axis() {
    for drive in [true, false] {
        for value in [0.0, -0.1, 0.99, f64::NAN, f64::INFINITY] {
            let mut config = ChassisConfig::default();
            if drive {
                config.drive_overcurrent_ratio = value;
            } else {
                config.steering_overcurrent_ratio = value;
            }
            assert!(config.validate().is_err());
        }
        for value in [-1.0, 100.1, f64::NAN, f64::INFINITY] {
            let mut config = ChassisConfig::default();
            if drive {
                config.drive_imax_a = value;
            } else {
                config.steering_imax_a = value;
            }
            assert!(config.validate().is_err());
        }
    }
    ChassisConfig {
        drive_overcurrent_ratio: 0.25,
        steering_overcurrent_ratio: 0.5,
        drive_imax_a: 20.0,
        steering_imax_a: 21.0,
        ..Default::default()
    }
    .validate()
    .unwrap();
}
