//! A2 velocity-loop limits are raw driver values, independent of current
//! feedback scaling and the separately adjustable full-torque reference.
use super::{directional_target_raw, zone_profile, A1CurrentConfig, PullCordConfig, Zone};
use serde::{Deserialize, Serialize};

pub const A2_PROTOCOL_MAX_RAW: u16 = 2000;
/// CAN V2.36 p. 15: Speed Limit (0x20) allows 600000 × 0.01 °/s = 1000 rpm.
pub const A2_PROTOCOL_MAX_RPM: f64 = 1000.0;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct A2SpeedConfig {
    pub homing_raw: u16,
    /// Near-zone limit while the cord is stationary.
    pub near_raw: u16,
    /// Near-zone limit while the cord pays out.
    pub payout_raw: u16,
    /// Near-zone limit while the cord retracts.
    pub retract_raw: u16,
    /// Stationary limit at and beyond the alert boundary.
    pub alert_raw: u16,
    /// Absolute raw limit applied to homing and every running direction.
    pub max_raw: u16,
    /// Operator's observed full-torque reference, for percentage display.
    /// This does not rescale raw limits or measured q-axis current.
    pub full_torque_raw: u16,
    pub speed_kp: u16,
    pub speed_ki: u16,
    pub speed_kd: u16,
    pub speed_ramp_dps2: u32,
}

impl Default for A2SpeedConfig {
    fn default() -> Self {
        Self {
            homing_raw: 41,
            near_raw: 17,
            payout_raw: 8,
            retract_raw: 17,
            alert_raw: 28,
            max_raw: 91,
            full_torque_raw: 2048,
            speed_kp: 100,
            speed_ki: 0,
            speed_kd: 0,
            speed_ramp_dps2: 3600,
        }
    }
}

impl A2SpeedConfig {
    /// Legacy A1 running did not use the saved retraction speed. On its first
    /// A2 migration, bound valid speed targets to this drum's protocol limit.
    /// Explicit A2 tuning and invalid legacy values remain validation errors.
    pub fn normalize_legacy_speeds(config: &mut PullCordConfig) {
        if config.a2_speed.is_some()
            || !config.drum_radius_m.is_finite()
            || config.drum_radius_m <= 0.0
        {
            return;
        }
        let maximum_speed_m_s = A2_PROTOCOL_MAX_RPM / 60.0 * config.metres_per_rev();
        if !maximum_speed_m_s.is_finite() || maximum_speed_m_s <= 0.0 {
            return;
        }
        // Leave floating-point rounding room so a persisted boundary value
        // cannot deserialize just above the limit on the next service start.
        let maximum_speed_m_s = maximum_speed_m_s.next_down().next_down();
        if maximum_speed_m_s <= 0.0 {
            return;
        }
        for speed in [&mut config.homing_speed_m_s, &mut config.retract_speed_m_s] {
            if speed.is_finite() && *speed > 0.0 {
                *speed = speed.min(maximum_speed_m_s);
            }
        }
    }

    /// Keep the previous raw requests/host ceiling without assuming that
    /// observed A1 command saturation calibrates the A2 or feedback scales.
    pub fn from_legacy(config: &PullCordConfig) -> Self {
        let a1 = config
            .a1_current
            .unwrap_or_else(|| A1CurrentConfig::from_legacy(config));
        let cap = |raw: u16| raw.min(A2_PROTOCOL_MAX_RAW);
        let near = cap(a1.near_raw);
        Self {
            homing_raw: cap(a1.homing_raw),
            near_raw: near,
            payout_raw: near / 2,
            retract_raw: near,
            alert_raw: cap(a1.alert_raw),
            max_raw: cap(a1.max_raw),
            full_torque_raw: a1.full_current_raw,
            ..Self::default()
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        for (field, value) in [
            ("homing_raw", self.homing_raw),
            ("near_raw", self.near_raw),
            ("payout_raw", self.payout_raw),
            ("retract_raw", self.retract_raw),
            ("alert_raw", self.alert_raw),
            ("max_raw", self.max_raw),
            ("speed_kp", self.speed_kp),
            ("speed_ki", self.speed_ki),
            ("speed_kd", self.speed_kd),
        ] {
            if value > A2_PROTOCOL_MAX_RAW {
                return Err(format!("a2_speed.{field} must be 0..=2000"));
            }
        }
        if !(1..=2048).contains(&self.full_torque_raw) {
            return Err("a2_speed.full_torque_raw must be 1..=2048".into());
        }
        if self.payout_raw > self.near_raw || self.near_raw > self.retract_raw {
            return Err(
                "a2_speed limits must satisfy payout_raw <= near_raw <= retract_raw".into(),
            );
        }
        if !(1..=600_000).contains(&self.speed_ramp_dps2) {
            return Err("a2_speed.speed_ramp_dps2 must be 1..=600000".into());
        }
        Ok(())
    }

    /// Use the same length ramp in each direction. The directional offsets
    /// preserve payout <= stationary <= retract at every length, including
    /// legacy profiles whose alert limit was below their near-zone limit.
    pub fn zone_limit_raw(
        &self,
        length_m: f64,
        speed_m_s: f64,
        config: &PullCordConfig,
    ) -> (Zone, u16) {
        let (zone, stationary) = self.stationary_limit_raw(length_m, config);
        let requested = directional_target_raw(
            stationary,
            speed_m_s,
            self.payout_raw,
            self.near_raw,
            self.retract_raw,
        );
        (zone, self.limit_raw(requested))
    }

    /// Unclamped stationary profile retained for operator diagnostics.
    pub fn stationary_limit_raw(&self, length_m: f64, config: &PullCordConfig) -> (Zone, f64) {
        zone_profile(
            length_m,
            config,
            f64::from(self.near_raw),
            f64::from(self.alert_raw),
        )
    }

    /// Quantize downward so interpolation never exceeds its requested limit.
    pub fn limit_raw(&self, requested: f64) -> u16 {
        if !requested.is_finite() {
            return 0;
        }
        requested
            .floor()
            .clamp(0.0, f64::from(self.max_raw.min(A2_PROTOCOL_MAX_RAW))) as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pull_cord::validate_live_config;

    fn config() -> PullCordConfig {
        let mut config = PullCordConfig::default();
        config.zone_near_end_m = 1.0;
        config.zone_alert_start_m = 2.0;
        config.a2_speed = Some(A2SpeedConfig {
            near_raw: 100,
            payout_raw: 40,
            retract_raw: 200,
            alert_raw: 300,
            max_raw: 400,
            ..A2SpeedConfig::default()
        });
        config
    }

    #[test]
    fn directional_limits_preserve_zones_and_low_payout_at_every_length() {
        let config = config();
        let a2 = config.a2_speed.unwrap();
        for (length, zone, payout, stationary, retract) in [
            (0.5, Zone::NearOrigin, 40, 100, 200),
            (1.5, Zone::RampUp, 140, 200, 300),
            (2.5, Zone::Alert, 240, 300, 400),
        ] {
            assert_eq!(a2.zone_limit_raw(length, 0.1, &config), (zone, payout));
            assert_eq!(a2.zone_limit_raw(length, 0.0, &config), (zone, stationary));
            assert_eq!(a2.zone_limit_raw(length, -0.1, &config), (zone, retract));
        }
        for speed in [-0.005, 0.005, f64::NAN] {
            assert_eq!(a2.zone_limit_raw(0.0, speed, &config).1, 100);
        }
        assert_eq!(a2.zone_limit_raw(f64::NAN, 0.1, &config).1, 40);
        let descending = A2SpeedConfig {
            alert_raw: 10,
            ..a2
        };
        assert!(descending.validate().is_ok());
        assert_eq!(descending.zone_limit_raw(2.5, 0.1, &config).1, 0);
        assert_eq!(descending.zone_limit_raw(2.5, 0.0, &config).1, 10);
        assert_eq!(descending.zone_limit_raw(2.5, -0.1, &config).1, 110);
    }

    #[test]
    fn actual_raw_limit_is_independent_of_display_reference_and_feedback() {
        let mut a2 = A2SpeedConfig {
            max_raw: 1200,
            ..A2SpeedConfig::default()
        };
        for reference in [850, 950, 2048] {
            a2.full_torque_raw = reference;
            assert_eq!(a2.limit_raw(990.9), 990);
        }
        a2.max_raw = 90;
        assert_eq!(a2.limit_raw(990.0), 90);
        assert_eq!(a2.limit_raw(-1.0), 0);
        assert_eq!(a2.limit_raw(f64::NAN), 0);
        a2.max_raw = 0;
        assert_eq!(a2.limit_raw(990.0), 0);
        a2.max_raw = 2048;
        assert_eq!(a2.limit_raw(5000.0), 2000);
    }

    #[test]
    fn legacy_raw_tuning_is_preserved_with_no_automatic_retraction_boost() {
        let mut config = PullCordConfig::default();
        config.a1_current = Some(A1CurrentConfig {
            homing_raw: 2048,
            near_raw: 123,
            payout_current_raw: 35,
            retract_current_raw: 190,
            alert_raw: 80,
            max_raw: 2048,
            full_current_raw: 950,
        });
        let a2 = A2SpeedConfig::from_legacy(&config);
        a2.validate().unwrap();
        assert_eq!((a2.homing_raw, a2.max_raw), (2000, 2000));
        assert_eq!((a2.near_raw, a2.payout_raw, a2.retract_raw), (123, 61, 123));
        assert_eq!(a2.alert_raw, 80);
        assert_eq!(a2.full_torque_raw, 950);
    }

    #[test]
    fn raw_pid_reference_ramp_and_direction_bounds_are_validated() {
        for field in [
            "homing_raw",
            "near_raw",
            "payout_raw",
            "retract_raw",
            "alert_raw",
            "max_raw",
            "speed_kp",
            "speed_ki",
            "speed_kd",
        ] {
            let mut raw = serde_json::to_value(A2SpeedConfig::default()).unwrap();
            raw[field] = 2001.into();
            assert!(
                serde_json::from_value::<A2SpeedConfig>(raw)
                    .unwrap()
                    .validate()
                    .is_err(),
                "{field}"
            );
        }
        for (field, value) in [
            ("full_torque_raw", 0),
            ("full_torque_raw", 2049),
            ("speed_ramp_dps2", 0),
            ("speed_ramp_dps2", 600001),
            ("payout_raw", 18),
            ("retract_raw", 16),
        ] {
            let mut raw = serde_json::to_value(A2SpeedConfig::default()).unwrap();
            raw[field] = value.into();
            assert!(
                serde_json::from_value::<A2SpeedConfig>(raw)
                    .unwrap()
                    .validate()
                    .is_err(),
                "{field}={value}"
            );
        }
    }

    #[test]
    fn a2_tuning_and_reference_are_live_but_geometry_direction_and_mode_are_not() {
        let current = config();
        let mut next = current;
        let a2 = next.a2_speed.as_mut().unwrap();
        a2.max_raw = 900;
        a2.full_torque_raw = 950;
        a2.speed_kp = 150;
        a2.speed_ki = 10;
        a2.speed_kd = 5;
        a2.speed_ramp_dps2 = 7200;
        next.retract_speed_m_s = 1.0;
        next.validate().unwrap();
        validate_live_config(&current, &next).unwrap();
        next.drum_radius_m *= 1.1;
        assert!(validate_live_config(&current, &next).is_err());
        next = current;
        next.retract_positive = !current.retract_positive;
        assert!(validate_live_config(&current, &next).is_err());
        next = current;
        next.a2_speed = None;
        assert!(validate_live_config(&current, &next).is_err());
    }

    #[test]
    fn both_a2_speed_targets_obey_1000_rpm_for_the_configured_drum() {
        for radius in [0.005, 0.026, 0.0425] {
            let mut config = config();
            config.drum_radius_m = radius;
            let maximum_speed_m_s = 1000.0 / 60.0 * config.metres_per_rev();
            config.homing_speed_m_s = maximum_speed_m_s;
            config.retract_speed_m_s = maximum_speed_m_s;
            config.validate().unwrap();
            config.homing_speed_m_s += 0.001;
            assert!(config.validate().unwrap_err().contains("homing_speed_m_s"));
            config.homing_speed_m_s = maximum_speed_m_s;
            config.retract_speed_m_s += 0.001;
            assert!(config.validate().unwrap_err().contains("retract_speed_m_s"));
        }
    }

    #[test]
    fn only_valid_legacy_speed_values_are_clipped_during_first_a2_migration() {
        let mut legacy = PullCordConfig::default();
        legacy.drum_radius_m = 0.005;
        let maximum_speed_m_s = 1000.0 / 60.0 * legacy.metres_per_rev();
        legacy.homing_speed_m_s = 1.0;
        legacy.retract_speed_m_s = 2.6;
        A2SpeedConfig::normalize_legacy_speeds(&mut legacy);
        assert!(legacy.homing_speed_m_s <= maximum_speed_m_s);
        assert!(legacy.retract_speed_m_s <= maximum_speed_m_s);
        assert!((legacy.homing_speed_m_s - maximum_speed_m_s).abs() < 1e-12);
        assert!((legacy.retract_speed_m_s - maximum_speed_m_s).abs() < 1e-12);

        legacy.homing_speed_m_s = 0.1;
        legacy.retract_speed_m_s = 0.2;
        A2SpeedConfig::normalize_legacy_speeds(&mut legacy);
        assert_eq!(
            (legacy.homing_speed_m_s, legacy.retract_speed_m_s),
            (0.1, 0.2)
        );

        for invalid in [0.0, -1.0, f64::INFINITY, f64::NAN] {
            let mut config = legacy;
            config.homing_speed_m_s = invalid;
            config.retract_speed_m_s = invalid;
            A2SpeedConfig::normalize_legacy_speeds(&mut config);
            assert_eq!(config.homing_speed_m_s.to_bits(), invalid.to_bits());
            assert_eq!(config.retract_speed_m_s.to_bits(), invalid.to_bits());
            config.a2_speed = Some(A2SpeedConfig::default());
            assert!(config.validate().is_err());

            let mut config = legacy;
            config.drum_radius_m = invalid;
            config.homing_speed_m_s = 1.0;
            config.retract_speed_m_s = 2.6;
            A2SpeedConfig::normalize_legacy_speeds(&mut config);
            assert_eq!(
                (config.homing_speed_m_s, config.retract_speed_m_s),
                (1.0, 2.6)
            );
        }

        legacy.a2_speed = Some(A2SpeedConfig::default());
        legacy.homing_speed_m_s = 1.0;
        legacy.retract_speed_m_s = 2.6;
        A2SpeedConfig::normalize_legacy_speeds(&mut legacy);
        assert_eq!(
            (legacy.homing_speed_m_s, legacy.retract_speed_m_s),
            (1.0, 2.6)
        );
        assert!(legacy.validate().is_err());
    }
}
