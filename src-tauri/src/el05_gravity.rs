//! Independent, continuously running pendulum gravity compensation. The host
//! computes damping from encoder-normalized velocity; MIT Kp/Kd remain zero.
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};
use std::{
    sync::atomic::{AtomicU64, Ordering},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use super::{check_range, El05Target};

pub const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(150);
pub const CAPTURE_MAX_SPEED: f64 = 0.1;
static NEXT_REFERENCE: AtomicU64 = AtomicU64::new(1);

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GravityCompensationConfig {
    pub reference_id: String,
    pub mass_kg: f64,
    pub length_m: f64,
    pub arm_mass_kg: f64,
    pub compensation_scale: f64,
    pub damping_nm_s_rad: f64,
    pub torque_limit_nm: f64,
    pub max_velocity_rad_s: f64,
    pub max_temperature_c: f64,
    pub ramp_s: f64,
}

impl GravityCompensationConfig {
    pub fn gravity_peak_nm(&self) -> f64 {
        (self.mass_kg + 0.5 * self.arm_mass_kg) * 9.80665 * self.length_m * self.compensation_scale
    }

    pub fn validate(&self) -> Result<()> {
        for (value, min, max, label) in [
            (
                self.mass_kg,
                0.0,
                20.0,
                "gravity compensation end mass (kg)",
            ),
            (
                self.length_m,
                0.001,
                2.0,
                "gravity compensation arm length (m)",
            ),
            (
                self.arm_mass_kg,
                0.0,
                20.0,
                "gravity compensation arm mass (kg)",
            ),
            (
                self.compensation_scale,
                0.0,
                1.5,
                "gravity compensation scale",
            ),
            (
                self.damping_nm_s_rad,
                0.0,
                5.0,
                "gravity compensation damping (Nm s/rad)",
            ),
            (
                self.torque_limit_nm,
                0.01,
                6.0,
                "gravity compensation torque limit (Nm)",
            ),
            (
                self.max_velocity_rad_s,
                0.1,
                20.0,
                "gravity compensation speed limit (rad/s)",
            ),
            (
                self.max_temperature_c,
                20.0,
                100.0,
                "gravity compensation temperature limit (°C)",
            ),
            (
                self.ramp_s,
                0.1,
                10.0,
                "gravity compensation startup ramp (s)",
            ),
        ] {
            check_range(value, min, max, label)?;
        }
        if self.reference_id.is_empty() {
            bail!("capture the pendulum downward reference before starting gravity compensation");
        }
        if self.mass_kg + self.arm_mass_kg <= 0.0 {
            bail!("gravity compensation requires a positive pendulum end or arm mass");
        }
        if self.gravity_peak_nm() > self.torque_limit_nm + 1e-9 {
            bail!("gravity compensation peak {:.3} Nm exceeds the {:.3} Nm torque limit; reduce the load/scale or increase the limit", self.gravity_peak_nm(), self.torque_limit_nm);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct GravityReference {
    pub id: String,
    pub motor_id: u8,
    /// Unwrapped encoder angle while the pendulum is vertically down.
    pub position_rad: f64,
    pub captured_at_s: f64,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct GravityCompensationStatus {
    pub active: bool,
    pub elapsed_s: f64,
    pub reference: Option<GravityReference>,
    pub config: Option<GravityCompensationConfig>,
    pub angle_rad: Option<f64>,
    pub gravity_torque_nm: f64,
    pub damping_torque_nm: f64,
    pub command_torque_nm: f64,
    pub limited: bool,
    pub reason: Option<String>,
}

#[derive(Default)]
pub(super) struct GravityCompensation {
    pub status: GravityCompensationStatus,
    started: Option<Instant>,
    /// Kept server-side so an old/UI-supplied angle cannot authorize motion.
    reference_binding: Option<String>,
}

impl GravityCompensation {
    pub fn elapsed_s(&self) -> f64 {
        self.started
            .map_or(self.status.elapsed_s, |t| t.elapsed().as_secs_f64())
    }

    pub fn capture(
        &mut self,
        motor_id: u8,
        binding: String,
        position_rad: f64,
        captured_at_s: f64,
    ) -> GravityReference {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        let reference = GravityReference {
            id: format!(
                "{timestamp:x}-{:x}",
                NEXT_REFERENCE.fetch_add(1, Ordering::Relaxed)
            ),
            motor_id,
            position_rad,
            captured_at_s,
        };
        self.status = GravityCompensationStatus {
            reference: Some(reference.clone()),
            ..Default::default()
        };
        self.reference_binding = Some(binding);
        reference
    }

    pub fn reference_matches(&self, id: &str, motor_id: u8, binding: Option<&str>) -> bool {
        self.status
            .reference
            .as_ref()
            .is_some_and(|r| r.id == id && r.motor_id == motor_id)
            && binding.is_some()
            && self.reference_binding.as_deref() == binding
    }

    pub fn invalidate_reference(&mut self, reason: impl Into<String>) {
        let reason = reason.into();
        self.finish(reason.clone());
        self.status.reference = None;
        self.reference_binding = None;
        self.status.reason = Some(reason);
    }

    pub fn begin(&mut self, config: GravityCompensationConfig) {
        self.started = Some(Instant::now());
        self.status.active = true;
        self.status.elapsed_s = 0.0;
        self.status.config = Some(config);
        self.status.angle_rad = None;
        self.status.gravity_torque_nm = 0.0;
        self.status.damping_torque_nm = 0.0;
        self.status.command_torque_nm = 0.0;
        self.status.limited = false;
        self.status.reason = None;
    }

    pub fn finish(&mut self, reason: impl Into<String>) {
        if self.status.active {
            self.status.elapsed_s = self.elapsed_s();
            self.status.active = false;
            self.started = None;
            self.status.command_torque_nm = 0.0;
            self.status.reason = Some(reason.into());
        }
    }

    pub fn runtime_target(&mut self, position_rad: f64, velocity_rad_s: f64) -> El05Target {
        let config = self
            .status
            .config
            .as_ref()
            .expect("active gravity configuration");
        let reference = self
            .status
            .reference
            .as_ref()
            .expect("active gravity reference");
        let elapsed_s = self.elapsed_s();
        let angle = position_rad - reference.position_rad;
        // Ramp in gravity feedforward from zero. Damping acts immediately and
        // always opposes motion in the increasing-encoder coordinate system.
        let gravity =
            config.gravity_peak_nm() * angle.sin() * (elapsed_s / config.ramp_s).clamp(0.0, 1.0);
        let damping = -config.damping_nm_s_rad * velocity_rad_s;
        let requested = gravity + damping;
        let commanded = requested.clamp(-config.torque_limit_nm, config.torque_limit_nm);
        self.status.elapsed_s = elapsed_s;
        self.status.angle_rad = Some(angle);
        self.status.gravity_torque_nm = gravity;
        self.status.damping_torque_nm = damping;
        self.status.command_torque_nm = commanded;
        self.status.limited = (commanded - requested).abs() > 1e-9;
        El05Target::Mit {
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_nm: commanded,
            kp: 0.0,
            kd: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config() -> GravityCompensationConfig {
        GravityCompensationConfig {
            reference_id: "test".into(),
            mass_kg: 0.5,
            length_m: 0.18,
            arm_mass_kg: 0.1,
            compensation_scale: 1.0,
            damping_nm_s_rad: 0.1,
            torque_limit_nm: 1.0,
            max_velocity_rad_s: 3.0,
            max_temperature_c: 70.0,
            ramp_s: 1.0,
        }
    }
    fn running() -> GravityCompensation {
        let mut gravity = GravityCompensation::default();
        gravity.capture(1, "uid".into(), 4.0, 0.0);
        gravity.begin(config());
        gravity.started = Some(Instant::now() - Duration::from_secs(2));
        gravity
    }
    #[test]
    fn pendulum_torque_tracks_downward_reference_and_damping_opposes_motion() {
        let mut gravity = running();
        gravity.runtime_target(4.0 + std::f64::consts::FRAC_PI_2, 0.0);
        assert!((gravity.status.gravity_torque_nm - 0.55 * 9.80665 * 0.18).abs() < 1e-9);
        gravity.runtime_target(4.0 - std::f64::consts::FRAC_PI_2, 0.0);
        assert!(gravity.status.command_torque_nm < 0.0);
        let target = gravity.runtime_target(4.0, 2.0);
        assert!((gravity.status.command_torque_nm + 0.2).abs() < 1e-9);
        assert!(matches!(
            target,
            El05Target::Mit {
                kp: 0.0,
                kd: 0.0,
                ..
            }
        ));
        gravity.runtime_target(4.0, -2.0);
        assert!((gravity.status.command_torque_nm - 0.2).abs() < 1e-9);
    }
    #[test]
    fn damping_sum_is_limited_and_startup_gravity_is_ramped() {
        let mut gravity = running();
        gravity.runtime_target(4.0 + std::f64::consts::FRAC_PI_2, -3.0);
        assert_eq!(gravity.status.command_torque_nm, 1.0);
        assert!(gravity.status.limited);
        gravity.started = Some(Instant::now());
        gravity.runtime_target(4.0 + std::f64::consts::FRAC_PI_2, 0.0);
        assert!(gravity.status.command_torque_nm < 0.01);
    }
    #[test]
    fn unsafe_configuration_and_stale_or_different_motor_reference_are_rejected() {
        let mut c = config();
        assert!(c.validate().is_ok());
        c.mass_kg = 2.0;
        assert!(c.validate().is_err());
        c.mass_kg = f64::NAN;
        assert!(c.validate().is_err());
        let mut gravity = running();
        let id = gravity.status.reference.as_ref().unwrap().id.clone();
        assert!(gravity.reference_matches(&id, 1, Some("uid")));
        assert!(!gravity.reference_matches(&id, 2, Some("uid")));
        assert!(!gravity.reference_matches(&id, 1, Some("another")));
        gravity.invalidate_reference("mechanical zero changed");
        assert!(!gravity.status.active);
        assert!(!gravity.reference_matches(&id, 1, Some("uid")));
        assert_eq!(gravity.status.command_torque_nm, 0.0);
    }
}
