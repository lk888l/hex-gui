use super::*;

fn config() -> ChassisConfig {
    let mut c = ChassisConfig::default();
    c.follow.max_speed_mps = 0.9;
    c.max_wheel_speed_mps = 1.1;
    c.follow.pace.target_length_m = 0.975;
    c
}
fn sample(length: f64, speed: f64) -> FollowInput {
    FollowInput {
        cord_length_m: Some(length),
        cord_speed_mps: Some(speed),
        encoder_angle_deg: Some(0.0),
        ..Default::default()
    }
}
fn reference(c: &ChassisConfig) -> f64 {
    c.follow.pace.target_length_m
}

#[test]
fn a_person_behind_the_cart_steps_towards_it_and_requests_more_forward_speed() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = config();
        c.cord_exit_side = side;
        let mut controller = FollowController::default();
        let mut wheels = 0.3 * side.sign();
        let mut length = reference(&c);
        for _ in 0..100 {
            let (_, next) = controller.step_with_feedback(
                sample(length, 0.0),
                wheels,
                Some(0.3 * side.sign()),
                0.01,
                &c,
            );
            wheels = next;
        }
        // Cart ahead at x=0, person behind at x=-L. dx_person/dt=1.0,
        // dx_cart/dt=0.3, therefore dL/dt=-0.7 during this step.
        for _ in 0..20 {
            length -= 0.007;
            let (target, next) = controller.step_with_feedback(
                sample(length, -0.7),
                wheels,
                Some(0.3 * side.sign()),
                0.01,
                &c,
            );
            assert!(
                target.speed_mps > 0.3,
                "approaching the cart must request acceleration"
            );
            assert!(next * side.sign() >= wheels * side.sign());
            assert!(!controller.status().payout_stop);
            wheels = next;
        }
    }
}

#[test]
fn distance_feedback_only_matches_exact_pace_near_the_configured_length() {
    let c = config();
    for (length, faster) in [(0.8, true), (1.2, false), (1.5, false), (1.79, false)] {
        let mut controller = FollowController::default();
        let (target, _) = controller.step(sample(length, 0.0), 0.3, 0.01, &c);
        assert_eq!(target.speed_mps > 0.3, faster);
        assert_eq!(target.speed_mps < 0.3, !faster);
    }
}

#[test]
fn an_approaching_step_can_cross_the_launch_length_but_not_the_near_origin_guard() {
    let c = config();
    let mut controller = FollowController::default();
    let (held, _) = controller.step(sample(0.7, -0.2), 0.0, 0.01, &c);
    assert_eq!(
        held.speed_mps, 0.0,
        "initial launch still requires the configured length"
    );
    controller.step(sample(0.9, -0.2), 0.0, 0.01, &c);
    let (moving, _) = controller.step(sample(0.7, -0.2), 0.3, 0.01, &c);
    assert!(moving.speed_mps > 0.0);
    assert_eq!(controller.status().pace_min_length_m, Some(0.05));
    let (short, _) = controller.step(sample(0.05, -1.2), 0.3, 0.01, &c);
    assert_eq!(short.speed_mps, 0.0);
    let (held, _) = controller.step(sample(0.7, -0.2), 0.0, 0.01, &c);
    assert_eq!(
        held.speed_mps, 0.0,
        "near-origin stop clears the old engagement"
    );
    controller.step(sample(0.9, -0.2), 0.0, 0.01, &c);
    controller.reset(); // Explicit stop clears the previous engagement.
    let (held, _) = controller.step(sample(0.7, -0.2), 0.0, 0.01, &c);
    assert_eq!(held.speed_mps, 0.0);
}

#[test]
fn a_brief_fast_retraction_keeps_the_cart_following() {
    let c = config();
    let mut controller = FollowController::default();
    let mut wheels = 0.3;
    let mut length = reference(&c);
    for tick in 0..500 {
        let relative = if (50..130).contains(&tick) { -1.2 } else { 0.0 };
        length += relative * 0.002;
        if tick % 5 == 0 {
            let (target, next) = controller.step_with_feedback(
                sample(length, relative),
                wheels,
                Some(0.3),
                0.01,
                &c,
            );
            assert!(target.speed_mps >= 0.3);
            assert!(next >= 0.3);
            wheels = next;
        }
    }
}

#[test]
fn zero_relative_speed_preserves_distinct_measured_walking_speeds() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = config();
        c.cord_exit_side = side;
        // Reproduce the locally saved, already saturated legacy tuning.
        c.follow.behind_speed.payout_speed_ratio = 0.8;
        c.follow.behind_speed.neutral_boost_ratio = 0.5;
        for speed in [0.0, 0.3, 0.8] {
            let mut controller = FollowController::default();
            let wheels = speed * side.sign();
            for _ in 0..300 {
                let (target, next) = controller.step(sample(reference(&c), 0.0), wheels, 0.01, &c);
                assert!((target.speed_mps - speed).abs() < 1e-12);
                assert_eq!(next, wheels);
                assert!(controller.status().pace_active);
            }
        }
    }
}

#[test]
fn matched_pace_is_preserved_at_configured_lengths_independent_of_launch_band() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for (start, target, stop) in [(0.75, 1.2, 1.8), (0.6, 0.62, 0.68)] {
            let mut c = config();
            c.cord_exit_side = side;
            c.follow.behind_start_cord_length_m = start;
            c.follow.behind_target_cord_length_m = target;
            c.follow.behind_stop_cord_length_m = stop;
            for fraction in [0.0, 0.25, 0.5, 0.9, 0.99] {
                let length = target + (stop - target) * fraction;
                c.follow.pace.target_length_m = length;
                for speed in [0.0, 0.03, 0.3, 0.8] {
                    let mut controller = FollowController::default();
                    let mut wheels = speed * side.sign();
                    for _ in 0..300 {
                        let (command, next) = controller.step_with_feedback(
                            sample(length, 0.0),
                            wheels,
                            Some(speed * side.sign()),
                            0.01,
                            &c,
                        );
                        assert!((command.speed_mps - speed).abs() < 1e-12,
                            "stable cord must preserve pace: length={length}, speed={speed}, target={}", command.speed_mps);
                        assert_eq!(next, wheels);
                        assert_eq!(controller.status().pace_correction_mps, 0.0);
                        assert!(controller.status().pace_active);
                        wheels = next;
                    }
                }
            }
        }
    }
}

#[test]
fn normal_band_retraction_accelerates_and_payout_decelerates_from_measured_pace() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = config();
        c.cord_exit_side = side;
        for initial_length in [1.2, 1.5, 1.7] {
            c.follow.pace.target_length_m = initial_length;
            for relative in [-0.2, 0.0, 0.2] {
                let mut controller = FollowController::default();
                let mut wheels = 0.3 * side.sign();
                for _ in 0..100 {
                    controller.step(sample(initial_length, 0.0), wheels, 0.01, &c);
                }
                let mut length = initial_length;
                for _ in 0..20 {
                    length += relative * 0.01;
                    let (command, next) = controller.step_with_feedback(
                        sample(length, relative),
                        wheels,
                        Some(0.3 * side.sign()),
                        0.01,
                        &c,
                    );
                    if relative < 0.0 {
                        assert!(command.speed_mps > 0.3, "closing the gap must accelerate");
                    } else if relative > 0.0 {
                        assert!(command.speed_mps < 0.3, "opening the gap must decelerate");
                    } else {
                        assert!((command.speed_mps - 0.3).abs() < 1e-12);
                    }
                    assert!(!controller.status().waiting_for_pull_back);
                    wheels = next;
                }
            }
        }
    }
}

#[test]
fn confirmed_restart_transitions_to_pace_before_reaching_the_target_length() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        // Confirm outside and just inside the stop boundary, above target.
        // The last case reaches matched speed on the first inside sample.
        for (origin, samples) in [(1.85, 25), (1.81, 25), (1.832, 10)] {
            let mut c = config();
            c.cord_exit_side = side;
            // Isolate the restart handoff; distance convergence is tested separately.
            c.follow.pace.length_gain_per_s = 0.0;
            let mut controller = FollowController::default();
            let mut wheels = 0.0;
            controller.step(sample(origin, 0.0), wheels, 0.01, &c);
            assert!(controller.status().waiting_for_pull_back);
            let mut length = origin;
            let mut confirmed = false;
            for _ in 0..samples {
                length -= 0.003;
                let (_, next) = controller.step_with_feedback(
                    sample(length, -0.3),
                    wheels,
                    Some(0.0),
                    0.01,
                    &c,
                );
                confirmed |= controller.status().retract_resume || controller.status().pace_active;
                wheels = next;
            }
            assert!(confirmed);
            if length >= c.follow.behind_stop_cord_length_m {
                assert!(controller.status().retract_resume);
                length = c.follow.behind_stop_cord_length_m - 0.001;
            }
            assert!(length > c.follow.behind_target_cord_length_m);
            for _ in 0..300 {
                let (command, next) = controller.step_with_feedback(
                    sample(length, 0.0),
                    wheels,
                    Some(0.3 * side.sign()),
                    0.01,
                    &c,
                );
                let status = controller.status();
                assert!(
                    status.pace_active,
                    "confirmed launch must enter continuous pace matching"
                );
                assert!(
                    !status.waiting_for_pull_back
                        && !status.retract_resume
                        && !status.retract_braking
                );
                assert!((command.speed_mps - 0.3).abs() < 1e-12);
                wheels = next;
            }
        }
    }
}

#[test]
fn restart_into_normal_band_tracks_changes_and_stop_with_motor_delay() {
    use std::collections::VecDeque;
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for lag in [0.0_f64, 0.15, 0.35] {
            let mut c = config();
            c.cord_exit_side = side;
            c.follow.pace.target_length_m = 1.65;
            let mut controller = FollowController::default();
            let mut wheels = 0.0;
            let mut actual = 0.0;
            let mut length = c.follow.behind_stop_cord_length_m + 0.05;
            let mut queue = VecDeque::from([0.0; 3]); // 30 ms transport delay.
            let mut max_error = 0.0_f64;
            let mut min_gap = length;
            for tick in 0..2500 {
                let user = match tick {
                    0..100 | 1900.. => 0.0,
                    100..800 | 1300..1900 => 0.3,
                    _ => 0.6,
                };
                let (command, next) = controller.step_with_feedback(
                    sample(length, actual - user),
                    wheels,
                    Some(actual * side.sign()),
                    0.01,
                    &c,
                );
                if tick < 100 {
                    assert_eq!(command.speed_mps, 0.0, "standing cannot restart");
                }
                if (300..1900).contains(&tick) {
                    assert!(
                        controller.status().pace_active,
                        "lost pace after launch: lag={lag}, tick={tick}, length={length}"
                    );
                    assert!(!controller.status().waiting_for_pull_back);
                    assert!(length > c.follow.behind_target_cord_length_m);
                    assert!(length < c.follow.pace.stop_length(&c));
                    min_gap = min_gap.min(length);
                }
                queue.push_back(next * side.sign());
                let delayed = queue.pop_front().unwrap();
                let alpha = if lag == 0.0 {
                    1.0
                } else {
                    1.0 - (-0.01 / lag).exp()
                };
                actual += alpha * (delayed - actual);
                length += (actual - user) * 0.01;
                wheels = next;
                if (500..800).contains(&tick)
                    || (1100..1300).contains(&tick)
                    || (1600..1900).contains(&tick)
                {
                    max_error = max_error.max((actual - user).abs());
                }
            }
            assert!(
                max_error < 0.05,
                "normal-band pace error: lag={lag}, error={max_error}"
            );
            assert!(
                actual < 0.01,
                "person stopped but cart kept going: {actual}"
            );
            eprintln!("restart pace lag={lag:.2}s delay=0.03s max plateau error={max_error:.6}m/s min walking gap={min_gap:.3}m final={actual:.6}m/s");
        }
    }
}

#[test]
fn unconfirmed_restart_and_unfinished_stop_cannot_enter_pace_on_a_boundary_crossing() {
    let c = config();
    for stopped in [false, true] {
        let mut controller = FollowController::default();
        let mut wheels = if stopped { 0.0 } else { 0.3 };
        controller.step(sample(1.81, 0.0), wheels, 0.01, &c);
        // Cross the boundary with a 30 ms pull, before confirmation or stop.
        for tick in 1..=3 {
            let (command, next) = controller.step_with_feedback(
                sample(1.81 - tick as f64 * 0.006, -0.6),
                wheels,
                Some(if stopped { 0.0 } else { 0.3 }),
                0.01,
                &c,
            );
            assert_eq!(command.speed_mps, 0.0);
            assert!(!controller.status().pace_active);
            wheels = next;
        }
        let (command, _) =
            controller.step_with_feedback(sample(1.792, 0.0), wheels, Some(0.0), 0.01, &c);
        assert_eq!(command.speed_mps, 0.0);
        assert!(!controller.status().pace_active);
    }
}

#[test]
fn estimate_uses_actual_wheels_not_command_and_never_invents_speed_when_standing() {
    let c = config();
    let mut controller = FollowController::default();
    let (command, _) =
        controller.step_with_feedback(sample(reference(&c), -0.1), 0.8, Some(0.2), 0.01, &c);
    assert!((command.speed_mps - 0.3).abs() < 1e-12);
    for length in [0.8, 1.0, 1.2, 1.7] {
        controller.reset();
        for speed in [0.0, 0.2, 1.0] {
            let (command, _) =
                controller.step_with_feedback(sample(length, speed), 0.0, Some(0.0), 0.01, &c);
            assert_eq!(command.linear_mps, 0.0);
        }
    }
}

#[test]
fn geometry_compensates_turning_and_either_cord_exit_without_angle_differentiation() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        let mut c = config();
        c.cord_exit_side = side;
        c.follow.encoder_center_deg = 160.0;
        for angle_deg in [-70.0_f64, -20.0, 0.0, 35.0, 100.0] {
            for yaw in [-1.0, 0.0, 1.0] {
                let mut observer = pace::PaceObserver::default();
                let angle = angle_deg.to_radians();
                let feedback = MeasuredMotion {
                    linear_mps: 0.3 * side.sign(),
                    yaw_rate_rad_s: yaw,
                };
                let offset = if side == CordExitSide::DrivenAxle {
                    0.0
                } else {
                    -c.wheelbase_m
                };
                let exit_projection =
                    side.sign() * (feedback.linear_mps * angle.cos() + yaw * offset * angle.sin());
                for user in [0.0, 0.3, 0.8] {
                    observer.reset();
                    let input = FollowInput {
                        encoder_angle_deg: Some(160.0 - angle_deg),
                        ..sample(reference(&c), exit_projection - user)
                    };
                    let request = observer.update(input, Some(feedback), 0.01, &c);
                    assert!((request.estimated_speed_mps.unwrap() - user).abs() < 1e-12);
                }
            }
        }
    }
}

#[test]
fn missing_feedback_gaps_reset_and_live_edits_cannot_replay_a_previous_pace() {
    let mut c = config();
    let mut controller = FollowController::default();
    for missing in [None, Some(f64::NAN), Some(f64::INFINITY)] {
        controller.step(sample(reference(&c), 0.0), 0.8, 0.01, &c);
        let (target, next) =
            controller.step_with_feedback(sample(reference(&c), 0.5), 0.8, missing, 0.01, &c);
        assert_eq!(target.linear_mps, 0.0);
        assert!(next < 0.8);
        assert_eq!(controller.status().estimated_user_speed_mps, None);
        let (target, _) = controller.step(sample(reference(&c), 0.0), 0.0, 0.01, &c);
        assert_eq!(target.speed_mps, 0.0);
    }
    for action in 0..6 {
        controller.step(sample(reference(&c), 0.0), 0.8, 0.01, &c);
        match action {
            0 => controller.reset(),
            1 => {
                controller.step(sample(0.0, 0.0), 0.0, 0.01, &c);
            }
            2 => {
                controller.step(sample(1.0, 0.0), 0.0, 0.2, &c);
            }
            3 => c.follow.encoder_center_deg += 1.0,
            4 => c.follow.pace.filter_ms += 10.0,
            _ => {
                controller.step(FollowInput::default(), 0.0, 0.01, &c);
            }
        }
        let (target, _) = controller.step(sample(reference(&c), 0.0), 0.0, 0.01, &c);
        assert_eq!(target.speed_mps, 0.0);
    }
}

#[test]
fn symmetric_hand_motion_is_filtered_without_rectifying_the_speed_estimate() {
    let c = config();
    for walking_speed in [0.0, 0.3, 0.8] {
        let mut observer = pace::PaceObserver::default();
        let mut sum = 0.0;
        let mut peak = 0.0_f64;
        for tick in 0..2000 {
            let phase = tick as f64 * 0.01 * 2.0 * std::f64::consts::PI * 2.0;
            let cord_speed = 0.35 * phase.cos();
            let request = observer.update(
                sample(
                    reference(&c) + 0.35 / (4.0 * std::f64::consts::PI) * phase.sin(),
                    cord_speed,
                ),
                Some(MeasuredMotion {
                    linear_mps: walking_speed,
                    yaw_rate_rad_s: 0.0,
                }),
                0.01,
                &c,
            );
            if tick >= 1000 {
                let error = request.estimated_speed_mps.unwrap() - walking_speed;
                sum += error;
                peak = peak.max(error.abs());
            }
        }
        assert!(sum.abs() / 1000.0 < 1e-6);
        assert!(
            peak < 0.11,
            "hand velocity ripple attenuated at least 3x: {peak}"
        );
    }
}

#[test]
fn brisk_step_from_rest_or_cruise_does_not_latch_a_tug_stop() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for initial in [0.0, 0.3] {
            for lag in [0.0_f64, 0.15, 0.35] {
                let mut c = config();
                c.cord_exit_side = side;
                let mut controller = FollowController::default();
                let mut wheels = initial * side.sign();
                let mut actual = initial;
                let mut length = reference(&c);
                let mut fast_retraction_samples = 0;
                for tick in 0..950 {
                    let user = match tick {
                        0..100 => initial,
                        100..120 => 1.3, // A large step, then ordinary walking.
                        120..650 => 0.6,
                        _ => 0.0,
                    };
                    let relative = actual - user;
                    fast_retraction_samples += usize::from(relative <= -0.5);
                    let (target, next) = controller.step_with_feedback(
                        sample(length, relative),
                        wheels,
                        Some(actual * side.sign()),
                        0.01,
                        &c,
                    );
                    assert!(
                        !controller.status().payout_stop,
                        "brisk step latched a stop: initial={initial}, lag={lag}, tick={tick}"
                    );
                    if tick < 650 {
                        assert!(controller.status().pace_active);
                    }
                    if (105..120).contains(&tick) {
                        assert!(target.speed_mps > initial);
                    }
                    let command = next * side.sign();
                    let alpha = if lag == 0.0 {
                        1.0
                    } else {
                        1.0 - (-0.01 / lag).exp()
                    };
                    actual += alpha * (command - actual);
                    length += (actual - user) * 0.01;
                    wheels = next;
                    // Evaluate the walking interval separately from the
                    // final stop, when the cart may briefly open the gap.
                    if tick < 650 {
                        assert!(length > 0.05 && length < c.follow.behind_stop_cord_length_m,
                            "lost following: initial={initial}, lag={lag}, tick={tick}, length={length}");
                    }
                    if tick == 649 {
                        assert!(
                            (actual - 0.6).abs() < 0.1,
                            "failed to match walking: {actual}"
                        );
                    }
                }
                assert!(fast_retraction_samples >= 4, "exercise a brisk retraction");
                assert!(
                    actual < 0.02,
                    "must still stop when the person stops: {actual}"
                );
            }
        }
    }
}

#[test]
fn falling_behind_still_brakes_at_the_long_cord_stop_line() {
    let c = config();
    let mut controller = FollowController::default();
    for _ in 0..100 {
        controller.step(sample(reference(&c), 0.0), 0.3, 0.01, &c);
    }
    let (target, next) = controller.step_with_feedback(
        sample(c.follow.behind_stop_cord_length_m, 0.6),
        0.6,
        Some(0.6),
        0.01,
        &c,
    );
    assert_eq!(controller.status().braking, Some(BehindBraking::StopLine));
    assert_eq!(target.speed_mps, 0.0);
    assert!(next < 0.6);
    assert!(!controller.status().payout_stop);
}

#[test]
fn fast_payout_still_stops_legacy_and_single_wheel_modes() {
    for (source, pace, tug_expected) in [
        (InputSource::Live, true, false),
        (InputSource::Live, false, true),
        (InputSource::Manual, true, true),
    ] {
        let mut c = config();
        c.source = source;
        c.follow.pace.enabled = pace;
        let mut controller = FollowController::default();
        for tick in 0..5 {
            controller.step(sample(1.0 + tick as f64 * 0.01, 1.0), 0.3, 0.01, &c);
        }
        assert_eq!(controller.status().payout_stop, tug_expected);
    }
}

#[test]
fn switching_to_pace_discards_partial_legacy_tug_confirmation() {
    let mut c = config();
    c.follow.pace.enabled = false;
    let mut controller = FollowController::default();
    for tick in 0..3 {
        controller.step(sample(1.0 + tick as f64 * 0.01, 1.0), 0.3, 0.01, &c);
    }
    assert!(!controller.status().payout_stop);
    c.follow.pace.enabled = true;
    controller.step(sample(1.03, 1.0), 0.3, 0.01, &c);
    assert!(!controller.status().payout_stop);
    c.follow.pace.enabled = false;
    for tick in 0..4 {
        controller.step(sample(1.04 + tick as f64 * 0.01, 1.0), 0.3, 0.01, &c);
        assert_eq!(controller.status().payout_stop, tick == 3);
    }
}

#[test]
fn gentle_retraction_launches_after_standing_without_a_distance_dead_zone() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for margin in [0.005, 0.05] {
            for user in [0.03, 0.05, 0.1] {
                let mut c = config();
                c.cord_exit_side = side;
                let mut controller = FollowController::default();
                let mut wheels = 0.0;
                let mut length = c.follow.behind_start_cord_length_m + margin;
                for _ in 0..100 {
                    let (_, next) = controller.step(sample(length, 0.0), wheels, 0.01, &c);
                    assert_eq!(next, 0.0, "standing must not launch");
                }
                let mut launched_at = None;
                let mut actual = 0.0;
                for tick in 0..200 {
                    let (_, next) = controller.step_with_feedback(
                        sample(length, actual - user),
                        wheels,
                        Some(actual * side.sign()),
                        0.01,
                        &c,
                    );
                    let command = next * side.sign();
                    if command >= 0.01 && launched_at.is_none() {
                        launched_at = Some(tick);
                    }
                    // A separate, lagged measurement: commands are not feedback.
                    actual += (1.0 - (-0.01_f64 / 0.15).exp()) * (command - actual);
                    length += (actual - user) * 0.01;
                    wheels = next;
                    assert!(!controller.status().payout_stop);
                }
                assert!(launched_at.is_some_and(|tick| tick < 25),
                    "gentle pull did not launch within 250 ms: margin={margin}, speed={user}, launch={launched_at:?}");
                assert!(
                    actual > user * 0.5,
                    "distance correction stalled a slow start: {actual}"
                );
            }
        }
    }
}

#[test]
fn enabling_pace_keeps_existing_tug_latch_until_return_and_preserves_length_brakes() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for legacy_enabled in [false, true] {
            let mut c = config();
            c.cord_exit_side = side;
            c.follow.behind_speed.enabled = legacy_enabled;
            c.follow.pace.enabled = false;
            let mut controller = FollowController::default();
            for tick in 0..5 {
                controller.step(
                    sample(1.0 + tick as f64 * 0.01, 1.0),
                    0.3 * side.sign(),
                    0.01,
                    &c,
                );
            }
            assert!(controller.status().payout_stop);
            assert!(!controller.status().pace_active);
            c.follow.pace.enabled = true;
            let (held, _) = controller.step(sample(1.04, 0.0), 0.0, 0.01, &c);
            assert_eq!(held.speed_mps, 0.0);
            let (returned, _) = controller.step(sample(1.0, 0.0), 0.0, 0.01, &c);
            assert!(!controller.status().payout_stop);
            assert_eq!(returned.speed_mps, 0.0);
            let (walking, _) = controller.step(sample(1.0, 0.1), 0.0, 0.01, &c);
            // First small filtered input may be inside the 0.01 m/s stop deadband.
            assert!(walking.speed_mps <= 0.1);
            for length in [1.8, 2.2] {
                let (target, _) = controller.step(sample(length, 0.0), 0.5 * side.sign(), 0.01, &c);
                assert_eq!(target.speed_mps, 0.0);
                assert!(controller.status().braking.is_some());
                assert!(!controller.status().pace_active);
            }
        }
    }
}

#[test]
fn closed_loop_tracks_slow_fast_slow_and_stop_with_motor_lag_and_hand_swing() {
    for lag in [0.0_f64, 0.15, 0.35] {
        for swing in [0.0, 0.025] {
            for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
                let mut c = config();
                c.cord_exit_side = side;
                let mut controller = FollowController::default();
                let mut wheels = 0.0;
                let mut actual = 0.0;
                let mut distance = reference(&c);
                let mut max_error = 0.0_f64;
                for tick in 0..3200 {
                    let t = tick as f64 * 0.01;
                    let user = if t < 2.0 {
                        0.0
                    } else if t < 3.0 {
                        (t - 2.0) * 0.3
                    } else if t < 8.0 {
                        0.3
                    } else if t < 10.0 {
                        0.3 + (t - 8.0) * 0.25
                    } else if t < 18.0 {
                        0.8
                    } else if t < 20.0 {
                        0.8 - (t - 18.0) * 0.25
                    } else if t < 26.0 {
                        0.3
                    } else if t < 27.0 {
                        0.3 * (27.0 - t)
                    } else {
                        0.0
                    };
                    let omega = 2.0 * std::f64::consts::PI * 1.6;
                    // No hand motion when the person has stopped.
                    let hand = if (3.0..26.0).contains(&t) { swing } else { 0.0 };
                    let input = sample(
                        distance + hand * (omega * t).sin(),
                        actual - user + hand * omega * (omega * t).cos(),
                    );
                    let (_, next) = controller.step_with_feedback(
                        input,
                        wheels,
                        Some(actual * side.sign()),
                        0.01,
                        &c,
                    );
                    assert!(
                        !controller.status().payout_stop,
                        "normal walking mistaken for tug: lag={lag}, t={t}"
                    );
                    let target = next * side.sign();
                    let alpha = if lag == 0.0 {
                        1.0
                    } else {
                        1.0 - (-0.01 / lag).exp()
                    };
                    actual += alpha * (target - actual);
                    distance += (actual - user) * 0.01;
                    wheels = next;
                    if (6.0..8.0).contains(&t)
                        || (15.0..18.0).contains(&t)
                        || (24.0..26.0).contains(&t)
                    {
                        max_error = max_error.max((actual - user).abs());
                    }
                    assert!(distance > 0.05 && distance < 1.8, "lost normal follow: lag={lag}, swing={swing}, t={t}, length={distance}, speed={actual}");
                }
                eprintln!("pace lag={lag:.2}s swing={swing:.3}m max plateau error={max_error:.4}m/s final={actual:.4}m/s");
                assert!(
                    max_error < 0.12,
                    "pace not matched: lag={lag}, swing={swing}, error={max_error}"
                );
                assert!(actual < 0.01, "person stopped but chassis still moving");
            }
        }
    }
}

#[test]
fn pace_respects_drive_ramps_and_steering_positions_with_turns_and_gaps() {
    for side in [CordExitSide::DrivenAxle, CordExitSide::PassiveAxle] {
        for cap in [0.2, 0.9, 7.0] {
            let mut c = config();
            c.cord_exit_side = side;
            c.follow.max_speed_mps = cap;
            c.max_wheel_speed_mps = cap;
            let mut controller = FollowController::default();
            let mut previous = 0.0;
            for tick in 0..1500 {
                let t = tick as f64 * 0.01;
                let dt: f64 = if tick % 100 == 0 { 0.2 } else { 0.01 };
                let input = FollowInput {
                    encoder_angle_deg: Some(85.0 * t.sin()),
                    ..sample(1.0 + 0.1 * (t * 0.7).cos(), 0.3 * (t * 1.3).sin())
                };
                let measured = previous * 0.8;
                let (target, next) = controller.step_with_motion(
                    input,
                    previous,
                    Some(MeasuredMotion::straight(measured)),
                    dt,
                    &c,
                );
                let actual = MeasuredMotion::straight(next);
                let before = previous;
                assert!(next.is_finite() && next.abs() <= cap + 1e-12);
                assert_eq!(target.steering_angle_deg, steering_angle(input, &c));
                assert!(
                    (actual.linear_mps - before).abs()
                        <= controller.status().acceleration_mps2 * dt.min(0.03) + 1e-12
                );
                assert!((0.0..=cap).contains(&target.speed_mps));
                assert!(target.linear_mps * side.sign() >= -1e-12);
                previous = next;
            }
        }
    }
}

#[test]
fn disabled_pace_bench_and_ahead_keep_their_existing_targets() {
    for (source, mode, enabled, length) in [
        (InputSource::Live, WalkMode::Behind, false, 1.0),
        (InputSource::Manual, WalkMode::Behind, true, 1.0),
        (InputSource::Manual, WalkMode::Behind, true, 1.0),
        (InputSource::Live, WalkMode::Ahead, true, 1.5),
        (InputSource::Live, WalkMode::Behind, true, 2.5),
    ] {
        let mut c = config();
        c.source = source;
        c.mode = mode;
        c.follow.pace.enabled = enabled;
        let mut controller = FollowController::default();
        let input = sample(length, 0.1);
        let (target, _) = controller.step(input, 0.0, 0.01, &c);
        assert!(!controller.status().pace_active);
        assert_eq!(target.speed_mps, compute(input, &c).speed_mps);
    }
}

#[test]
fn pace_defaults_migrate_preserve_custom_values_and_validate() {
    let mut saved = serde_json::to_value(config()).unwrap();
    saved["follow"].as_object_mut().unwrap().remove("pace");
    let old: ChassisConfig = serde_json::from_value(saved.clone()).unwrap();
    assert_eq!(old.follow.pace, PaceFollowParameters::default());
    saved["follow"]["pace"] = serde_json::json!({"enabled":false,"filterMs":400.0});
    let custom: ChassisConfig = serde_json::from_value(saved.clone()).unwrap();
    custom.validate().unwrap();
    assert!(!custom.follow.pace.enabled);
    assert_eq!(custom.follow.pace.filter_ms, 400.0);
    assert_eq!(custom.follow.pace.max_correction_mps, 0.12);
    let again: ChassisConfig =
        serde_json::from_value(serde_json::to_value(&custom).unwrap()).unwrap();
    assert_eq!(custom.follow.pace, again.follow.pace);
    for (key, min, max) in [
        ("targetLengthM", 0.1, 8.0),
        ("stopMarginM", 0.1, 4.0),
        ("filterMs", 20.0, 2000.0),
        ("lengthGainPerS", 0.0, 2.0),
        ("lengthDeadbandM", 0.0, 0.5),
        ("maxCorrectionMps", 0.0, 0.3),
        ("accelerationMps2", 0.01, 4.0),
        ("decelerationMps2", 0.01, 4.0),
    ] {
        for value in [min - 0.001, max + 0.001] {
            saved["follow"]["pace"] = serde_json::json!({key:value});
            let bad: ChassisConfig = serde_json::from_value(saved.clone()).unwrap();
            assert!(bad.validate().is_err(), "{key}={value}");
        }
    }
}
