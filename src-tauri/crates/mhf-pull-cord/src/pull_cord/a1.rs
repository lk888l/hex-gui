//! A1 command counts are independent of the drive's current feedback scale.
use super::{directional_target_raw, zone_profile, PullCordConfig, Zone};
use serde::{Deserialize, Serialize};

pub const A1_PROTOCOL_MAX_RAW: u16 = 2048;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(from = "A1CurrentConfigWire")]
pub struct A1CurrentConfig {
    pub homing_raw: u16,
    /// Stationary current in the near zone.
    pub near_raw: u16,
    pub payout_current_raw: u16,
    pub retract_current_raw: u16,
    /// Stationary current at the alert boundary.
    pub alert_raw: u16,
    /// Host command ceiling, independent of the saturation calibration point.
    pub max_raw: u16,
    /// Command magnitude at the observed maximum current. This is a reference
    /// for command percentages, not a send limit or a feedback-to-amp scale.
    pub full_current_raw: u16,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct A1CurrentConfigWire {
    homing_raw: u16,
    near_raw: u16,
    payout_current_raw: Option<u16>,
    retract_current_raw: Option<u16>,
    alert_raw: u16,
    max_raw: u16,
    full_current_raw: u16,
    // Older templates used these keys for a different control law. Keep
    // accepting them without turning previously ignored values into commands.
    #[serde(default, rename = "near_90_raw")]
    _retired_near_90_raw: Option<serde::de::IgnoredAny>,
    #[serde(default, rename = "payout_raw")]
    _retired_payout_raw: Option<serde::de::IgnoredAny>,
    #[serde(default, rename = "retract_raw")]
    _retired_retract_raw: Option<serde::de::IgnoredAny>,
    #[serde(default, rename = "damping_gain_raw_per_m_s")]
    _retired_damping_gain_raw_per_m_s: Option<serde::de::IgnoredAny>,
}

impl From<A1CurrentConfigWire> for A1CurrentConfig {
    fn from(wire: A1CurrentConfigWire) -> Self {
        Self {
            homing_raw: wire.homing_raw,
            near_raw: wire.near_raw,
            payout_current_raw: wire.payout_current_raw.unwrap_or(wire.near_raw / 2),
            retract_current_raw: wire.retract_current_raw.unwrap_or(wire.near_raw),
            alert_raw: wire.alert_raw,
            max_raw: wire.max_raw,
            full_current_raw: wire.full_current_raw,
        }
    }
}

impl A1CurrentConfig {
    /// Preserve old requested commands and their independent host ceiling.
    /// The former manual conversion is used only to migrate saved Nm inputs.
    pub fn from_legacy(config: &PullCordConfig) -> Self {
        let raw = |nm: f64| {
            (nm / (0.18 * 16.5 / 2048.0))
                .round()
                .clamp(0.0, f64::from(A1_PROTOCOL_MAX_RAW)) as u16
        };
        let near_raw = raw(config.torque_near_nm);
        Self {
            homing_raw: raw(config.homing_torque_nm),
            near_raw,
            payout_current_raw: near_raw / 2,
            retract_current_raw: near_raw,
            alert_raw: raw(config.torque_alert_nm),
            max_raw: raw(0.53 * f64::from(config.max_torque_permille) / 1000.0),
            full_current_raw: A1_PROTOCOL_MAX_RAW,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        for (field, value) in [
            ("homing_raw", self.homing_raw),
            ("near_raw", self.near_raw),
            ("payout_current_raw", self.payout_current_raw),
            ("retract_current_raw", self.retract_current_raw),
            ("alert_raw", self.alert_raw),
            ("max_raw", self.max_raw),
            ("full_current_raw", self.full_current_raw),
        ] {
            if value > A1_PROTOCOL_MAX_RAW {
                return Err(format!("a1_current.{field} must be 0..=2048"));
            }
        }
        if self.full_current_raw == 0 {
            return Err("a1_current.full_current_raw must be 1..=2048".into());
        }
        if self.payout_current_raw > self.near_raw || self.near_raw > self.retract_current_raw {
            return Err(
                "a1_current must satisfy payout_current_raw <= near_raw <= retract_current_raw"
                    .into(),
            );
        }
        Ok(())
    }

    /// Stationary current at this length before applying the host ceiling.
    pub fn zone_current(&self, length_m: f64, config: &PullCordConfig) -> (Zone, f64) {
        zone_profile(
            length_m,
            config,
            f64::from(self.near_raw),
            f64::from(self.alert_raw),
        )
    }

    pub fn running_current(
        &self,
        length_m: f64,
        cord_speed_m_s: f64,
        config: &PullCordConfig,
    ) -> (Zone, f64) {
        let (zone, stationary) = self.zone_current(length_m, config);
        let current = directional_target_raw(
            stationary,
            cord_speed_m_s,
            self.payout_current_raw,
            self.near_raw,
            self.retract_current_raw,
        );
        (zone, current)
    }

    /// Quantize once, enforce the raw host/protocol ceilings, then set polarity.
    /// A 990 request remains 990 when full_current_raw is 950 and max_raw >= 990.
    pub fn command(&self, requested: f64, retract_positive: bool) -> i16 {
        let magnitude = if requested.is_finite() {
            requested
                .round()
                .clamp(0.0, f64::from(self.max_raw.min(A1_PROTOCOL_MAX_RAW))) as i16
        } else {
            0
        };
        if retract_positive {
            magnitude
        } else {
            -magnitude
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pull_cord::validate_live_config;

    fn raw_config() -> A1CurrentConfig {
        A1CurrentConfig {
            homing_raw: 990,
            near_raw: 100,
            payout_current_raw: 40,
            retract_current_raw: 200,
            alert_raw: 300,
            max_raw: 1200,
            full_current_raw: 950,
        }
    }

    #[test]
    fn saturation_reference_does_not_rescale_or_clip_commands() {
        let mut a1 = raw_config();
        for full in [850, 950, 2048] {
            a1.full_current_raw = full;
            a1.validate().unwrap();
            assert_eq!(a1.command(990.0, true), 990);
            assert_eq!(a1.command(990.0, false), -990);
        }
        a1.max_raw = 700;
        assert_eq!(a1.command(990.0, true), 700);
        assert_eq!(a1.command(990.0, false), -700);
        a1.max_raw = 2048;
        assert_eq!(a1.command(5000.0, true), 2048);
        assert_eq!(a1.command(-1.0, true), 0);
        assert_eq!(a1.command(f64::NAN, true), 0);
        a1.max_raw = 0;
        assert_eq!(a1.command(990.0, true), 0);
    }

    #[test]
    fn measured_direction_and_length_select_current_before_command_clipping() {
        let mut config = PullCordConfig::default();
        config.zone_near_end_m = 1.0;
        config.zone_alert_start_m = 2.0;
        config.torque_near_nm = 5.0;
        config.retract_boost_torque_nm = 5.0;
        config.max_torque_permille = 0;
        let a1 = raw_config();
        for (length, zone, payout, stationary, retract) in [
            (0.0, Zone::NearOrigin, 40.0, 100.0, 200.0),
            (1.5, Zone::RampUp, 140.0, 200.0, 300.0),
            (2.5, Zone::Alert, 240.0, 300.0, 400.0),
        ] {
            assert_eq!(a1.running_current(length, 0.0051, &config), (zone, payout));
            assert_eq!(
                a1.running_current(length, -0.0051, &config),
                (zone, retract)
            );
            for speed in [
                -0.005,
                0.0,
                0.005,
                f64::NAN,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ] {
                assert_eq!(
                    a1.running_current(length, speed, &config),
                    (zone, stationary)
                );
            }
        }
        assert_eq!(
            a1.running_current(f64::NAN, 0.1, &config),
            (Zone::NearOrigin, 40.0)
        );
        let decreasing = A1CurrentConfig {
            alert_raw: 10,
            ..a1
        };
        let (_, outward) = decreasing.running_current(2.5, 0.1, &config);
        assert_eq!(decreasing.command(outward, false), 0);
        let capped = A1CurrentConfig { max_raw: 275, ..a1 };
        let (_, inward) = capped.running_current(1.5, -0.1, &config);
        assert_eq!(capped.command(inward, false), -275);
    }

    #[test]
    fn retired_motion_damping_and_angle_controls_are_ignored_and_not_serialized() {
        let expected = raw_config();
        let mut wire = serde_json::to_value(expected).unwrap();
        wire["near_90_raw"] = 800.into();
        wire["payout_raw"] = 0.into();
        wire["retract_raw"] = 1200.into();
        wire["damping_gain_raw_per_m_s"] = 20_000.into();
        let restored: A1CurrentConfig = serde_json::from_value(wire.clone()).unwrap();
        restored.validate().unwrap();
        assert_eq!(restored, expected);
        assert_eq!(
            serde_json::to_value(restored).unwrap(),
            serde_json::to_value(expected).unwrap()
        );
        wire["unrecognized_setting"] = 1.into();
        assert!(serde_json::from_value::<A1CurrentConfig>(wire).is_err());
    }

    #[test]
    fn raw_bounds_and_setup_only_values_are_validated() {
        let mut config = PullCordConfig::default();
        config.control_mode = crate::PullCordMode::A1Current;
        config.a1_current = Some(raw_config());
        let mut next = config;
        next.a1_current.as_mut().unwrap().homing_raw = 1100;
        next.a1_current.as_mut().unwrap().near_raw = 150;
        next.a1_current.as_mut().unwrap().payout_current_raw = 50;
        next.a1_current.as_mut().unwrap().retract_current_raw = 175;
        next.a1_current.as_mut().unwrap().alert_raw = 350;
        validate_live_config(&config, &next).unwrap();
        next.a1_current.as_mut().unwrap().full_current_raw = 850;
        assert!(validate_live_config(&config, &next).is_err());
        next = config;
        next.a1_current.as_mut().unwrap().max_raw = 1500;
        assert!(validate_live_config(&config, &next).is_err());
        for field in [
            "homing_raw",
            "near_raw",
            "payout_current_raw",
            "retract_current_raw",
            "alert_raw",
            "max_raw",
            "full_current_raw",
        ] {
            let mut value = serde_json::to_value(raw_config()).unwrap();
            value[field] = 2049.into();
            assert!(serde_json::from_value::<A1CurrentConfig>(value)
                .unwrap()
                .validate()
                .is_err());
        }
        let mut a1 = raw_config();
        a1.full_current_raw = 0;
        assert!(a1.validate().is_err());
        for (payout, retract, valid) in [(101, 200, false), (40, 99, false), (100, 100, true)] {
            let a1 = A1CurrentConfig {
                payout_current_raw: payout,
                retract_current_raw: retract,
                ..raw_config()
            };
            assert_eq!(a1.validate().is_ok(), valid);
        }
    }
}
