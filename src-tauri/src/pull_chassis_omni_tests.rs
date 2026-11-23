use super::*;
use mhf_pull_cord::chassis_model::{BehindBraking, CordExitSide};

const DT: f64 = 0.01;

fn omni_config() -> PullChassisConfig {
    PullChassisConfig {
        pace_motion: PaceMotionMode::Omni,
        cord_exit_side: CordExitSide::DrivenAxle,
        ..Default::default()
    }
}

fn input(config: &PullChassisConfig, speed: f64) -> FollowInput {
    FollowInput {
        cord_length_m: Some(config.follow.pace.target_length_m),
        cord_speed_mps: Some(speed),
        encoder_angle_deg: Some(config.follow.encoder_center_deg),
        ..Default::default()
    }
}

fn endpoint(config: &PullChassisConfig, y: f64, vy: f64) -> CordEndpoint {
    let x = config.follow.pace.target_length_m;
    CordEndpoint {
        frame_id: 1,
        segment: 1,
        time_us: 0,
        center_deg: config.follow.encoder_center_deg,
        x_m: x,
        y_m: y,
        length_m: x.hypot(y),
        bearing_rad: y.atan2(x),
        radial_speed_m_s: 0.0,
        velocity_m_s: Some([0.0, vy]),
    }
}

#[test]
fn sideways_motion_requires_the_existing_live_behind_launch() {
    let c = omni_config();
    let rest = Hopea3State::default();
    let hand = Some(endpoint(&c, 0.2, 0.6));
    let mut control = ChassisFollowController::default();
    for _ in 0..20 {
        assert_eq!(
            control.step(input(&c, 0.0), hand, &rest, DT, &c, Some(2.0)),
            (0.0, 0.0, 0.0)
        );
    }
    let mut short = input(&c, -0.3);
    short.cord_length_m = Some(c.follow.behind_start_cord_length_m);
    assert_eq!(
        control.step(short, hand, &rest, DT, &c, Some(2.0)),
        (0.0, 0.0, 0.0)
    );
    let diagonal = control.step(input(&c, -0.3), hand, &rest, DT, &c, Some(2.0));
    assert!(diagonal.0 > 0.0 && diagonal.1 > 0.0 && diagonal.2 == 0.0);
    for other in [
        PullChassisConfig {
            pace_motion: PaceMotionMode::Steering,
            ..c.clone()
        },
        PullChassisConfig {
            source: InputSource::Manual,
            ..c.clone()
        },
        PullChassisConfig {
            mode: WalkMode::Ahead,
            ..c.clone()
        },
        {
            let mut off = c.clone();
            off.follow.pace.enabled = false;
            off
        },
    ] {
        control.reset();
        let command = control.step(input(&other, -0.3), hand, &rest, DT, &other, Some(2.0));
        assert_eq!(command.1, 0.0);
        assert!(!control.lateral_pace_active());
    }
}

#[test]
fn lateral_pace_uses_body_feedback_and_signed_outlet_motion_even_at_zero_forward_speed() {
    for (side, side_sign) in [
        (CordExitSide::DrivenAxle, 1.0),
        (CordExitSide::PassiveAxle, -1.0),
    ] {
        for inverted in [false, true] {
            let c = PullChassisConfig {
                cord_exit_side: side,
                direction_inverted: inverted,
                ..omni_config()
            };
            let sign = if inverted { -side_sign } else { side_sign };
            let mut control = ChassisFollowController::default();
            let mut measured = Hopea3State::default();
            control.step(
                input(&c, -0.2),
                Some(endpoint(&c, 0.0, 0.0)),
                &measured,
                DT,
                &c,
                Some(2.0),
            );
            for relative_vy in [-0.35, 0.35] {
                // A hand moving past a stationary base requests signed lateral motion.
                let point = endpoint(&c, 0.0, relative_vy);
                measured.meas_vy = 0.0;
                for _ in 0..160 {
                    control.step(input(&c, 0.0), Some(point), &measured, DT, &c, Some(2.0));
                }
                assert!((control.previous_y - sign * relative_vy).abs() < 1e-6);
                assert_eq!(control.previous_x, 0.0);
                // At matched pace the hand is stationary in the outlet frame.
                measured.meas_vy = control.previous_y;
                let matched = endpoint(&c, 0.0, 0.0);
                for _ in 0..160 {
                    control.step(input(&c, 0.0), Some(matched), &measured, DT, &c, Some(2.0));
                }
                assert!((control.previous_y - sign * relative_vy).abs() < 1e-6);
                // A stationary hand in world coordinates cancels even residual yaw.
                measured.meas_wz = 0.16;
                let outlet_vy =
                    -(measured.meas_vy + measured.meas_wz * side_sign * point.x_m) / sign;
                for _ in 0..160 {
                    control.step(
                        input(&c, 0.0),
                        Some(endpoint(&c, 0.0, outlet_vy)),
                        &measured,
                        DT,
                        &c,
                        Some(2.0),
                    );
                }
                assert_eq!(control.previous_y, 0.0);
                measured.meas_wz = 0.0;
            }
        }
    }
}

#[test]
fn moving_hand_reverses_and_stops_with_wheel_lag_and_measured_cord_motion() {
    let c = omni_config();
    let mut control = ChassisFollowController::default();
    let mut measured = Hopea3State::default();
    let initial_y = 0.2;
    let mut y = initial_y;
    control.step(
        input(&c, -0.2),
        Some(endpoint(&c, y, 0.0)),
        &measured,
        DT,
        &c,
        Some(2.0),
    );
    for hand_vy in [0.35, -0.35, 0.0] {
        for _ in 0..500 {
            let relative_vy = hand_vy - measured.meas_vy;
            y += relative_vy * DT;
            let point = endpoint(&c, y, relative_vy);
            let sample = FollowInput {
                cord_length_m: Some(point.length_m),
                cord_speed_mps: Some(relative_vy * point.bearing_rad.sin()),
                encoder_angle_deg: Some(point.center_deg - point.bearing_rad.to_degrees()),
                ..Default::default()
            };
            let last = control.previous_y;
            let (_, commanded_vy, yaw) =
                control.step(sample, Some(point), &measured, DT, &c, Some(2.0));
            assert!(
                last * commanded_vy >= 0.0,
                "a reversal must pass through zero"
            );
            assert_eq!(yaw, 0.0);
            measured.meas_vy += 0.2 * (commanded_vy - measured.meas_vy);
        }
        assert!((measured.meas_vy - hand_vy).abs() < 0.015);
        assert!((y - initial_y).abs() < 0.08);
    }
}

#[test]
fn lateral_offset_has_a_deadband_cap_and_a_fresh_reference_after_a_gap_or_recalibration() {
    let c = omni_config();
    let rest = Hopea3State::default();
    let mut pace = LateralPace::default();
    let point = endpoint(&c, 0.25, 0.0);
    let target = |pace: &mut LateralPace, p| pace.target(p, &rest, &c, DT);
    assert_eq!(target(&mut pace, Some(point)), 0.0);
    assert_eq!(
        target(&mut pace, Some(CordEndpoint { y_m: 0.26, ..point })),
        0.0
    );
    assert_eq!(
        target(&mut pace, Some(CordEndpoint { y_m: 1.25, ..point })),
        c.follow.pace.max_correction_mps
    );
    let displaced = CordEndpoint {
        y_m: -0.75,
        ..point
    };
    for invalid in [
        None,
        Some(CordEndpoint {
            velocity_m_s: None,
            ..point
        }),
        Some(CordEndpoint {
            center_deg: 0.0,
            ..point
        }),
    ] {
        assert!(target(&mut pace, Some(displaced)) < 0.0);
        assert_eq!(target(&mut pace, invalid), 0.0);
        assert!(pace.segment.is_none());
        assert_eq!(target(&mut pace, Some(point)), 0.0);
    }
    for fresh in [
        CordEndpoint {
            segment: 2,
            ..displaced
        },
        CordEndpoint {
            frame_id: 2,
            y_m: 1.0,
            ..point
        },
    ] {
        assert_eq!(target(&mut pace, Some(fresh)), 0.0);
    }
}

#[test]
fn diagonal_velocity_and_slew_are_limited_as_vectors() {
    let mut c = omni_config();
    c.follow.max_speed_mps = 0.5;
    let mut control = ChassisFollowController::default();
    let rest = Hopea3State::default();
    let point = Some(endpoint(&c, 0.0, 1.0));
    for _ in 0..200 {
        let previous = (control.previous_x, control.previous_y);
        let (x, y, yaw) = control.step(input(&c, -0.8), point, &rest, DT, &c, Some(2.0));
        assert!(x.hypot(y) <= 0.5 + 1e-12);
        assert!(
            (x - previous.0).hypot(y - previous.1) <= c.follow.pace.acceleration_mps2 * DT + 1e-12
        );
        assert_eq!(yaw, 0.0);
    }
    assert!(control.previous_x > 0.3 && control.previous_y > 0.3);
    c.max_linear_mps = 0.2;
    let (x, y, _) = control.step(input(&c, -0.8), point, &rest, DT, &c, Some(2.0));
    assert!(x.hypot(y) <= 0.2 + 1e-12);
    assert!(x.hypot(y) >= 0.2 - c.follow.pace.deceleration_mps2 * DT);
    let rate = c.max_deceleration_mps2();
    let (bx, by, yaw) = control.step(input(&c, -2.0), point, &rest, DT, &c, Some(2.0));
    assert!(control.retract_braking && !control.lateral_pace_active());
    assert!(((x - bx).hypot(y - by) - rate * DT).abs() < 1e-9);
    assert_eq!(yaw, 0.0);
}

#[test]
fn pure_strafe_brakes_to_a_measured_stop_before_ahead_takeover_or_restart() {
    let c = omni_config();
    for (length, brake) in [
        (1.45, BehindBraking::StopLine),
        (1.7, BehindBraking::AheadSwitch),
    ] {
        let mut control = ChassisFollowController::default();
        let mut measured = Hopea3State::default();
        let point = Some(endpoint(&c, 0.0, 0.4));
        control.step(input(&c, -0.2), point, &measured, DT, &c, Some(2.0));
        for _ in 0..150 {
            control.step(input(&c, 0.0), point, &measured, DT, &c, Some(2.0));
        }
        assert_eq!(control.previous_x, 0.0);
        assert!(control.previous_y > 0.39);
        measured.meas_vy = control.previous_y;
        let mut sample = input(&c, 0.0);
        sample.cord_length_m = Some(length);
        for _ in 0..100 {
            let last_vy = control.previous_y;
            let (x, y, yaw) = control.step(sample, point, &measured, DT, &c, Some(2.0));
            assert_eq!(control.status().braking, Some(brake));
            assert_eq!((x, yaw), (0.0, 0.0));
            assert!(y >= 0.0 && y <= last_vy);
        }
        assert_eq!(control.previous_y, 0.0);
        // A brief return to the cruise band cannot bypass the measured-stop gate.
        control.step(input(&c, -0.2), point, &measured, DT, &c, Some(2.0));
        assert_eq!(control.status().braking, Some(brake));
        measured.meas_vy = 0.0;
        let (x, y, _) = control.step(sample, point, &measured, DT, &c, Some(2.0));
        assert_eq!(control.status().braking, None);
        assert_eq!(y, 0.0);
        if brake == BehindBraking::AheadSwitch {
            assert!(x < 0.0 && control.status().travel_mode == Some(WalkMode::Ahead));
        } else {
            assert_eq!(x, 0.0);
        }
    }
}
