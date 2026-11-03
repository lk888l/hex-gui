//! Position-held gravity loads, swept in torque space. No empirical torque
//! correction is applied: feedback is compared with the measured-angle load.
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};

use super::{check_range, ExperimentConfig, PendulumConfig};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TorqueAccuracyConfig {
    pub reference_id: String,
    pub start_nm: f64,
    pub end_nm: f64,
    pub step_nm: f64,
    pub hold_s: f64,
    pub move_speed_rad_s: f64,
    /// Side of the downward reference; each run ascends and descends in load.
    pub direction: i8,
}

pub fn gravity_peak(p: &PendulumConfig) -> f64 {
    (p.mass_kg + p.arm_mass_kg * 0.5) * 9.80665 * p.length_m
}

impl TorqueAccuracyConfig {
    pub fn validate(&self, config: &ExperimentConfig) -> Result<()> {
        for (value, min, max, label) in [
            (self.start_nm, 0.1, 5.9, "accuracy start torque (Nm)"),
            (self.end_nm, self.start_nm, 5.9, "accuracy end torque (Nm)"),
            (self.step_nm, 0.01, 0.5, "accuracy torque step (Nm)"),
            (self.hold_s, 0.3, 30.0, "accuracy hold time (s)"),
            (
                self.move_speed_rad_s,
                0.05,
                0.5,
                "accuracy move speed (rad/s)",
            ),
            (
                config.max_temperature_c,
                20.0,
                120.0,
                "accuracy return temperature (°C)",
            ),
            (config.kp, 1.0, 500.0, "accuracy position Kp"),
            (config.kd, 0.1, 5.0, "accuracy position Kd"),
        ] {
            check_range(value, min, max, label)?;
        }
        let p = config
            .pendulum
            .as_ref()
            .filter(|p| p.zero_measured)
            .ok_or_else(|| {
                anyhow::anyhow!("capture the downward reference before torque accuracy testing")
            })?;
        if self.reference_id.is_empty() || ![-1, 1].contains(&self.direction) {
            bail!("torque accuracy requires a current downward reference and direction ±1");
        }
        if self.end_nm > gravity_peak(p) || self.end_nm <= self.start_nm {
            bail!(
                "accuracy end torque must exceed the start and be reachable by the pendulum load"
            );
        }
        if self.point_count() > 401 || self.planned_duration_s() > 3600.0 {
            bail!("accuracy scan exceeds 401 holds or 3600 s; increase the step or shorten holds");
        }
        if !config.gravity_compensation {
            bail!("torque accuracy requires gravity feedforward compensation");
        }
        Ok(())
    }

    fn ascending_count(&self) -> usize {
        (((self.end_nm - self.start_nm) / self.step_nm - 1e-9).ceil() as usize) + 1
    }

    pub fn point_count(&self) -> usize {
        self.ascending_count() * 2 - 1
    }

    /// Conservative time budget, including positioning and return/settling.
    pub fn planned_duration_s(&self) -> f64 {
        self.point_count() as f64 * (self.hold_s + 0.1)
            + 6.0 * std::f64::consts::PI / self.move_speed_rad_s
            + 5.0
    }

    fn point(&self, index: usize, p: &PendulumConfig) -> (f64, f64, bool) {
        let ascending = self.ascending_count();
        let increasing = index < ascending;
        let offset = if increasing {
            index
        } else {
            2 * ascending - 2 - index
        };
        let torque = (self.start_nm + offset as f64 * self.step_nm).min(self.end_nm);
        let angle = f64::from(self.direction) * (torque / gravity_peak(p)).asin();
        (f64::from(self.direction) * torque, angle, increasing)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum TorqueAccuracyPhase {
    Moving,
    Holding,
    Returning,
    Complete,
}

#[derive(Clone, Debug, Serialize)]
pub struct TorqueAccuracyHold {
    pub index: usize,
    pub target_torque_nm: f64,
    pub target_angle_rad: f64,
    pub increasing: bool,
    pub started_s: f64,
    pub ended_s: Option<f64>,
    pub completed: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct TorqueAccuracyStatus {
    pub phase: TorqueAccuracyPhase,
    pub index: usize,
    pub point_count: usize,
    pub holds: Vec<TorqueAccuracyHold>,
    pub return_started_s: Option<f64>,
    pub return_reason: Option<String>,
    pub returned_to_zero: bool,
}

pub(super) struct TorqueAccuracyRuntime {
    from: f64,
    to: f64,
    last_position: f64,
    started_s: f64,
    move_s: f64,
    settled_since: Option<f64>,
    requested_return: Option<String>,
}

impl TorqueAccuracyRuntime {
    pub fn new(config: &ExperimentConfig) -> (Self, TorqueAccuracyStatus) {
        let a = config.torque_accuracy.as_ref().unwrap();
        let p = config.pendulum.as_ref().unwrap();
        let mut runtime = Self {
            from: 0.0,
            to: 0.0,
            last_position: config.origin_position_rad,
            started_s: 0.0,
            move_s: 0.0,
            settled_since: None,
            requested_return: None,
        };
        runtime.move_to(
            0.0,
            config.origin_position_rad,
            p.downward_zero_rad + a.point(0, p).1,
            a,
        );
        (
            runtime,
            TorqueAccuracyStatus {
                phase: TorqueAccuracyPhase::Moving,
                index: 0,
                point_count: a.point_count(),
                holds: Vec::new(),
                return_started_s: None,
                return_reason: None,
                returned_to_zero: false,
            },
        )
    }

    fn move_to(&mut self, t: f64, from: f64, to: f64, a: &TorqueAccuracyConfig) {
        self.from = from;
        self.to = to;
        self.started_s = t;
        // Smoothstep has peak derivative 1.5; endpoints have zero velocity.
        self.move_s = (1.5 * (to - from).abs() / a.move_speed_rad_s).max(0.1);
        self.settled_since = None;
    }

    pub fn temperature(
        &mut self,
        temperature: f64,
        config: &ExperimentConfig,
        state: &TorqueAccuracyStatus,
    ) {
        if temperature >= config.max_temperature_c
            && !matches!(
                state.phase,
                TorqueAccuracyPhase::Returning | TorqueAccuracyPhase::Complete
            )
        {
            self.requested_return = Some("temperature bound reached".into());
        }
    }

    fn begin_return(
        &mut self,
        t: f64,
        config: &ExperimentConfig,
        state: &mut TorqueAccuracyStatus,
        reason: String,
    ) {
        if let Some(hold) = state.holds.last_mut().filter(|h| h.ended_s.is_none()) {
            hold.ended_s = Some(t);
        }
        state.phase = TorqueAccuracyPhase::Returning;
        state.return_started_s = Some(t);
        state.return_reason = Some(reason);
        self.move_to(
            t,
            // Preserve the existing position error/support torque. Resetting
            // the target to measured position would abruptly unload a PD hold.
            self.last_position,
            config.pendulum.as_ref().unwrap().downward_zero_rad,
            config.torque_accuracy.as_ref().unwrap(),
        );
    }

    pub fn target(
        &mut self,
        t: f64,
        position: f64,
        velocity: f64,
        config: &ExperimentConfig,
        state: &mut TorqueAccuracyStatus,
    ) -> (f64, f64) {
        let a = config.torque_accuracy.as_ref().unwrap();
        let p = config.pendulum.as_ref().unwrap();
        if let Some(reason) = self.requested_return.take() {
            self.begin_return(t, config, state, reason);
        }
        match state.phase {
            TorqueAccuracyPhase::Moving if t - self.started_s + 1e-9 >= self.move_s => {
                let (torque, angle, increasing) = a.point(state.index, p);
                state.holds.push(TorqueAccuracyHold {
                    index: state.index,
                    target_torque_nm: torque,
                    target_angle_rad: angle,
                    increasing,
                    started_s: t,
                    ended_s: None,
                    completed: false,
                });
                state.phase = TorqueAccuracyPhase::Holding;
                self.started_s = t;
            }
            TorqueAccuracyPhase::Holding if t - self.started_s + 1e-9 >= a.hold_s => {
                let hold = state.holds.last_mut().unwrap();
                hold.ended_s = Some(t);
                hold.completed = true;
                if state.index + 1 == state.point_count {
                    self.begin_return(t, config, state, "scan completed".into());
                } else {
                    state.index += 1;
                    state.phase = TorqueAccuracyPhase::Moving;
                    self.move_to(
                        t,
                        self.to,
                        p.downward_zero_rad + a.point(state.index, p).1,
                        a,
                    );
                }
            }
            TorqueAccuracyPhase::Returning if t - self.started_s + 1e-9 >= self.move_s => {
                if (position - p.downward_zero_rad).abs() <= 0.02 && velocity.abs() <= 0.05 {
                    let since = *self.settled_since.get_or_insert(t);
                    if t - since + 1e-9 >= 0.2 {
                        state.returned_to_zero = true;
                        state.phase = TorqueAccuracyPhase::Complete;
                    }
                } else {
                    self.settled_since = None;
                }
                if state.phase != TorqueAccuracyPhase::Complete
                    && t - self.started_s >= self.move_s + 5.0
                {
                    state.return_reason = Some(format!(
                        "{}; return to zero timed out",
                        state.return_reason.as_deref().unwrap_or("scan ended")
                    ));
                    state.phase = TorqueAccuracyPhase::Complete;
                }
            }
            _ => {}
        }
        if matches!(
            state.phase,
            TorqueAccuracyPhase::Holding | TorqueAccuracyPhase::Complete
        ) {
            self.last_position = self.to;
            return (self.to, 0.0);
        }
        let u = ((t - self.started_s) / self.move_s).clamp(0.0, 1.0);
        self.last_position = self.from + (self.to - self.from) * u * u * (3.0 - 2.0 * u);
        (
            self.last_position,
            (self.to - self.from) * 6.0 * u * (1.0 - u) / self.move_s,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::el05::experiment::ExperimentKind;

    fn config() -> ExperimentConfig {
        ExperimentConfig {
            kind: ExperimentKind::TorqueAccuracy,
            kp: 80.0,
            kd: 2.0,
            max_temperature_c: 120.0,
            gravity_compensation: true,
            origin_position_rad: 0.7,
            pendulum: Some(PendulumConfig {
                mass_kg: 3.0,
                length_m: 0.18,
                downward_zero_rad: 0.7,
                zero_measured: true,
                ..Default::default()
            }),
            torque_accuracy: Some(TorqueAccuracyConfig {
                reference_id: "reference".into(),
                start_nm: 3.7,
                end_nm: 5.2,
                step_nm: 0.05,
                hold_s: 1.0,
                move_speed_rad_s: 0.3,
                direction: 1,
            }),
            ..Default::default()
        }
    }

    #[test]
    fn torque_accuracy_plan_is_load_spaced_and_round_trips_without_duplicate_peak() {
        let c = config();
        c.validate().unwrap();
        let a = c.torque_accuracy.as_ref().unwrap();
        let p = c.pendulum.as_ref().unwrap();
        assert_eq!(a.point_count(), 61);
        assert!((gravity_peak(p) - 5.295591).abs() < 1e-9);
        for index in 0..61 {
            let (load, angle, up) = a.point(index, p);
            assert!((gravity_peak(p) * angle.sin() - load).abs() < 1e-10);
            assert_eq!(up, index <= 30);
        }
        assert!((a.point(0, p).0 - 3.7).abs() < 1e-10);
        assert!((a.point(30, p).0 - 5.2).abs() < 1e-10);
        assert!((a.point(60, p).0 - 3.7).abs() < 1e-10);
        let mut partial = a.clone();
        partial.end_nm = 3.83;
        assert_eq!(partial.point_count(), 7);
        assert!((partial.point(3, p).0 - 3.83).abs() < 1e-10);
    }

    #[test]
    fn torque_accuracy_holds_for_configured_duration_and_returns_before_completion() {
        let mut c = config();
        c.torque_accuracy.as_mut().unwrap().end_nm = 3.8;
        c.torque_accuracy.as_mut().unwrap().hold_s = 0.6;
        c.torque_accuracy.as_mut().unwrap().direction = -1;
        let (mut runtime, mut status) = TorqueAccuracyRuntime::new(&c);
        let mut position = c.origin_position_rad;
        let mut velocity = 0.0;
        let mut saw_return = false;
        for i in 0..10_000 {
            let t = i as f64 * 0.005;
            let target = runtime.target(t, position, velocity, &c, &mut status);
            assert!(target.1.abs() <= 0.3 + 1e-10);
            position = target.0;
            velocity = target.1;
            saw_return |= status.phase == TorqueAccuracyPhase::Returning;
            if status.phase == TorqueAccuracyPhase::Complete {
                break;
            }
        }
        assert!(saw_return && status.returned_to_zero);
        assert_eq!(status.phase, TorqueAccuracyPhase::Complete);
        assert_eq!(status.holds.len(), 5);
        assert_eq!(status.return_reason.as_deref(), Some("scan completed"));
        assert!((position - 0.7).abs() < 1e-10);
        for h in status.holds {
            assert!(h.completed && h.target_angle_rad < 0.0 && h.target_torque_nm < 0.0);
            assert!((h.ended_s.unwrap() - h.started_s - 0.6).abs() <= 0.006);
        }
    }

    #[test]
    fn torque_accuracy_overtemperature_latches_return_and_excludes_interrupted_hold() {
        let c = config();
        let (mut runtime, mut status) = TorqueAccuracyRuntime::new(&c);
        let move_end = runtime.move_s;
        let held_position = runtime.to;
        runtime.target(move_end, held_position, 0.0, &c, &mut status);
        assert_eq!(status.phase, TorqueAccuracyPhase::Holding);
        runtime.temperature(119.9, &c, &status);
        runtime.target(move_end + 0.1, held_position, 0.0, &c, &mut status);
        assert_eq!(status.phase, TorqueAccuracyPhase::Holding);
        runtime.temperature(120.0, &c, &status);
        let start = move_end + 0.2;
        // Keep the holding error/support torque instead of jumping the target
        // down to the lagging measured position and abruptly unloading the arm.
        assert_eq!(
            runtime.target(start, held_position - 0.04, 0.0, &c, &mut status),
            (held_position, 0.0)
        );
        assert_eq!(status.phase, TorqueAccuracyPhase::Returning);
        assert!(!status.holds[0].completed);
        assert_eq!(status.holds[0].ended_s, Some(start));
        let return_end = start + runtime.move_s;
        runtime.temperature(123.0, &c, &status);
        runtime.target(return_end, 0.7, 0.0, &c, &mut status);
        assert_eq!(status.phase, TorqueAccuracyPhase::Returning);
        runtime.temperature(40.0, &c, &status);
        runtime.target(return_end + 0.21, 0.7, 0.0, &c, &mut status);
        assert!(status.returned_to_zero);
        assert_eq!(status.phase, TorqueAccuracyPhase::Complete);
        assert_eq!(
            status.return_reason.as_deref(),
            Some("temperature bound reached")
        );
    }

    #[test]
    fn torque_accuracy_return_timeout_is_explicit_and_never_claims_zero() {
        let c = config();
        let (mut runtime, mut status) = TorqueAccuracyRuntime::new(&c);
        runtime.temperature(120.0, &c, &status);
        runtime.target(0.5, 1.0, 0.0, &c, &mut status);
        runtime.target(0.5 + runtime.move_s + 5.1, 1.0, 0.0, &c, &mut status);
        assert_eq!(status.phase, TorqueAccuracyPhase::Complete);
        assert!(!status.returned_to_zero);
        assert!(status.return_reason.as_ref().unwrap().contains("timed out"));
    }

    #[test]
    fn torque_accuracy_rejects_unreachable_loads_invalid_timing_or_missing_reference() {
        let valid = config();
        for change in 0..9 {
            let mut c = valid.clone();
            match change {
                0 => c.torque_accuracy.as_mut().unwrap().end_nm = 5.8,
                1 => c.torque_accuracy.as_mut().unwrap().step_nm = 0.0,
                2 => c.torque_accuracy.as_mut().unwrap().hold_s = f64::NAN,
                3 => c.torque_accuracy.as_mut().unwrap().reference_id.clear(),
                4 => c.pendulum.as_mut().unwrap().zero_measured = false,
                5 => c.torque_accuracy.as_mut().unwrap().direction = 0,
                6 => c.max_temperature_c = 121.0,
                7 => c.gravity_compensation = false,
                _ => c.kp = 0.0,
            }
            assert!(c.validate().is_err(), "invalid configuration {change}");
        }
    }
}
