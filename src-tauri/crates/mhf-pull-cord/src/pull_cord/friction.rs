//! Bounded payout assistance. Motor-current torque is not a tension sensor:
//! at rest we preload in the known payout direction instead of claiming to
//! observe the operator's force. Never integrate a stationary torque error.

use super::{
    payout_compensation_scale, retract_torque_nm, torque_to_limit_permille, PullCordConfig,
};
use serde::{Deserialize, Serialize};
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FrictionMode {
    #[serde(alias = "velocity")]
    Preload,
    Off,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FrictionCompensationConfig {
    pub mode: FrictionMode,
    pub use_factory_calibration: bool,
    /// Fractions of calibrated magnitudes, always below full cancellation.
    pub static_ratio: f64,
    pub kinetic_ratio: f64,
    /// Continuous rest/payout/retract transition width, not an enable gate.
    pub transition_speed_m_s: f64,
    /// Physical-equivalent ceiling; factory friction itself stays in raw units.
    pub max_torque_nm: f64,
    pub slew_rate_nm_s: f64,
    /// Restore normal retraction and remove assistance at the mechanical stop.
    pub origin_fade_m: f64,
}

impl Default for FrictionCompensationConfig {
    fn default() -> Self {
        Self {
            mode: FrictionMode::Preload,
            use_factory_calibration: true,
            static_ratio: 0.6,
            kinetic_ratio: 0.8,
            transition_speed_m_s: 0.01,
            max_torque_nm: 0.15,
            slew_rate_nm_s: 0.5,
            origin_fade_m: 0.03,
        }
    }
}

impl FrictionCompensationConfig {
    pub(super) fn validate(&self) -> Result<(), String> {
        for (name, value, min, max) in [
            ("static_ratio", self.static_ratio, 0.0, 0.95),
            ("kinetic_ratio", self.kinetic_ratio, 0.0, 0.95),
            (
                "transition_speed_m_s",
                self.transition_speed_m_s,
                0.001,
                0.1,
            ),
            ("max_torque_nm", self.max_torque_nm, 0.0, 2.0),
            ("slew_rate_nm_s", self.slew_rate_nm_s, 0.01, 10.0),
            ("origin_fade_m", self.origin_fade_m, 0.001, 1.0),
        ] {
            if !value.is_finite() || value < min || value > max {
                return Err(format!(
                    "friction_compensation.{name} must be finite within {min}..={max}"
                ));
            }
        }
        Ok(())
    }
}

/// CRC-validated 0x4001:05..07 values in the motor's RAW command domain.
/// Do not multiply these by the factory torque factor a second time.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct FrictionCalibration {
    pub static_pos_raw_nm: f64,
    pub static_neg_raw_nm: f64,
    pub kinetic_pos_raw_nm: f64,
    pub kinetic_neg_raw_nm: f64,
    pub kinetic_reference_speed_rad_per_s: f64,
    pub calibration_temperature_c: f64,
}

impl FrictionCalibration {
    fn payout(self, retract_positive: bool) -> Option<(f64, f64)> {
        if ![
            self.static_pos_raw_nm,
            self.static_neg_raw_nm,
            self.kinetic_pos_raw_nm,
            self.kinetic_neg_raw_nm,
            self.kinetic_reference_speed_rad_per_s,
        ]
        .into_iter()
        .all(|v| v.is_finite() && v > 0.0)
            || !self.calibration_temperature_c.is_finite()
        {
            return None;
        }
        Some(if retract_positive {
            (self.static_neg_raw_nm, self.kinetic_neg_raw_nm)
        } else {
            (self.static_pos_raw_nm, self.kinetic_pos_raw_nm)
        })
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FrictionSource {
    #[default]
    Off,
    Factory,
    Manual,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FrictionOutput {
    pub base_torque_nm: f64,
    /// Cord-facing raw command torque; negative assists payout.
    pub tff_raw_nm: f64,
    pub source: FrictionSource,
    pub active: bool,
    pub preloading: bool,
    pub limited: bool,
}

#[derive(Default)]
pub struct FrictionRuntime {
    last_update: Option<Duration>,
    raw_magnitude_nm: f64,
}

fn smoothstep(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

impl FrictionRuntime {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        now: Duration,
        length_m: f64,
        speed_m_s: f64,
        permitted: bool,
        config: &PullCordConfig,
        factory: Option<FrictionCalibration>,
        torque_factor: f64,
        peak_torque_nm: f64,
    ) -> FrictionOutput {
        let base = retract_torque_nm(length_m, speed_m_s, config);
        let mut output = FrictionOutput {
            base_torque_nm: base,
            ..Default::default()
        };
        let cfg = config.friction_compensation;
        let zone_scale = payout_compensation_scale(length_m, config);
        if !permitted
            || !length_m.is_finite()
            || !speed_m_s.is_finite()
            || !torque_factor.is_finite()
            || torque_factor <= 0.0
            || !peak_torque_nm.is_finite()
            || peak_torque_nm <= 0.0
            || zone_scale == 0.0
            || cfg.mode == FrictionMode::Off
        {
            self.reset();
            return output;
        }

        // Wall time, capped after a scheduler pause. Repeated calls at one
        // instant cannot build torque as if new control periods had passed.
        let dt = self
            .last_update
            .replace(now)
            .map(|last| {
                now.saturating_sub(last)
                    .min(Duration::from_millis(20))
                    .as_secs_f64()
            })
            .unwrap_or(0.0);
        let factory = factory
            .filter(|_| cfg.use_factory_calibration)
            .and_then(|cal| cal.payout(config.retract_positive));
        let (static_raw, kinetic_raw) = factory.unwrap_or_else(|| {
            let raw = config.zero_friction_assist_nm * torque_factor;
            (raw, raw)
        });
        output.source = if factory.is_some() {
            FrictionSource::Factory
        } else {
            FrictionSource::Manual
        };
        // Cap preload below sliding drag too: a correct model must not
        // accelerate a free drum once the larger stiction is overcome.
        let kinetic = kinetic_raw * cfg.kinetic_ratio;
        let preload = (static_raw * cfg.static_ratio).min(kinetic);
        let payout = smoothstep(speed_m_s / cfg.transition_speed_m_s);
        let inward = 1.0 - smoothstep(-speed_m_s / cfg.transition_speed_m_s);
        let origin_fade = cfg.origin_fade_m.min(config.zone_near_end_m);
        let envelope = zone_scale * smoothstep(length_m / origin_fade) * inward;
        let requested_raw = preload + (kinetic - preload) * payout;
        let cap_raw = cfg.max_torque_nm * torque_factor;
        let step = cfg.slew_rate_nm_s * dt;
        self.raw_magnitude_nm += (requested_raw.min(cap_raw) - self.raw_magnitude_nm)
            .clamp(-step * torque_factor, step * torque_factor);
        // Lowered caps/model values and direction/zone gates beat smoothing.
        self.raw_magnitude_nm = self.raw_magnitude_nm.clamp(0.0, cap_raw.min(requested_raw));
        output.tff_raw_nm = -self.raw_magnitude_nm * envelope;
        output.active = envelope > 0.0;
        output.preloading = output.active && speed_m_s.abs() < cfg.transition_speed_m_s;
        output.limited = envelope > 0.0 && requested_raw > cap_raw;

        // Reserve the actual quantized Kd clamp in either torque direction.
        // Spring compensation uses the remaining budget after this output.
        let pd = torque_to_limit_permille(output.base_torque_nm, peak_torque_nm, torque_factor)
            .min(config.max_torque_permille);
        let available_raw = peak_torque_nm * f64::from(config.max_torque_permille - pd) / 1000.0;
        let limited_raw = output.tff_raw_nm.max(-available_raw);
        output.limited |= limited_raw > output.tff_raw_nm;
        if output.limited && output.tff_raw_nm < 0.0 {
            // Do not accumulate hidden assistance behind a reduced ceiling.
            self.raw_magnitude_nm *= limited_raw / output.tff_raw_nm;
        }
        output.tff_raw_nm = limited_raw;
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration() -> FrictionCalibration {
        FrictionCalibration {
            static_pos_raw_nm: 0.09,
            static_neg_raw_nm: 0.12,
            kinetic_pos_raw_nm: 0.06,
            kinetic_neg_raw_nm: 0.10,
            kinetic_reference_speed_rad_per_s: 1.0,
            calibration_temperature_c: 25.0,
        }
    }

    fn settle(
        config: &PullCordConfig,
        length: f64,
        speed: f64,
        cal: Option<FrictionCalibration>,
        factor: f64,
    ) -> FrictionOutput {
        let mut rt = FrictionRuntime::default();
        let mut output = FrictionOutput::default();
        for tick in 0..=600 {
            output = rt.update(
                Duration::from_millis(tick * 2),
                length,
                speed,
                true,
                config,
                cal,
                factor,
                10.0,
            );
        }
        output
    }

    #[test]
    fn stationary_preload_reduces_breakaway_without_a_velocity_trigger() {
        let cfg = PullCordConfig::default();
        let out = settle(&cfg, 1.0, 0.0, None, 1.0);
        assert!(out.active && out.preloading);
        assert_eq!(out.source, FrictionSource::Manual);
        assert!((out.base_torque_nm - cfg.torque_near_nm).abs() < 1e-12);
        assert!((out.tff_raw_nm + 0.027).abs() < 1e-12);
        // Synthetic gearbox, not a claim about the user's measured mechanism.
        // At rest: hand torque must exceed stiction + motor retract torque.
        let stiction = 0.08;
        let old_breakaway = stiction + cfg.torque_near_nm;
        let new_breakaway = stiction + out.base_torque_nm + out.tff_raw_nm;
        assert!(new_breakaway > 0.0 && new_breakaway < old_breakaway);
        let slow_pull = settle(&cfg, 1.0, 0.001, None, 1.0);
        assert!(
            slow_pull.tff_raw_nm < 0.0,
            "assistance must precede the old 0.03 m/s gate"
        );
    }

    #[test]
    fn factory_direction_and_raw_units_are_independent_of_torque_factor() {
        for retract_positive in [true, false] {
            for factor in [0.7, 1.0, 1.8] {
                let mut cfg = PullCordConfig::default();
                cfg.retract_positive = retract_positive;
                let out = settle(&cfg, 1.0, 0.0, Some(calibration()), factor);
                let expected = if retract_positive { 0.072 } else { 0.048 };
                assert_eq!(out.source, FrictionSource::Factory);
                assert!((out.tff_raw_nm + expected).abs() < 1e-12);
                let out = settle(&cfg, 1.0, 0.1, Some(calibration()), factor);
                let expected = if retract_positive { 0.08 } else { 0.048 };
                assert!((out.tff_raw_nm + expected).abs() < 1e-12);
                assert_eq!(out.base_torque_nm, cfg.torque_near_nm);
            }
        }
    }

    #[test]
    fn high_stiction_never_enables_more_than_sliding_drag() {
        let mut cal = calibration();
        cal.static_neg_raw_nm = 2.0;
        let cfg = PullCordConfig::default();
        for speed in [-0.005, 0.0, 0.001, 0.01, 0.1, 2.0] {
            let out = settle(&cfg, 1.0, speed, Some(cal), 1.0);
            assert!(out.tff_raw_nm <= 0.0);
            assert!(-out.tff_raw_nm <= 0.08 + 1e-12);
            // Even after breakaway, the model leaves positive drag on payout.
            assert!(cal.kinetic_neg_raw_nm + out.tff_raw_nm > 0.0);
        }
    }

    #[test]
    fn speed_noise_zero_crossing_and_zone_boundaries_are_continuous() {
        let cfg = PullCordConfig::default();
        for speed in [-0.01, 0.0, 0.01] {
            let left = settle(&cfg, 1.0, speed - 1e-8, None, 1.0);
            let right = settle(&cfg, 1.0, speed + 1e-8, None, 1.0);
            assert!((left.tff_raw_nm - right.tff_raw_nm).abs() < 1e-6);
            assert!((left.base_torque_nm - right.base_torque_nm).abs() < 1e-6);
        }
        for length in [
            0.0,
            cfg.friction_compensation.origin_fade_m,
            cfg.zone_near_end_m,
            cfg.zone_alert_start_m,
        ] {
            for speed in [0.0, 0.1] {
                let left = settle(&cfg, length - 1e-8, speed, None, 1.0);
                let right = settle(&cfg, length + 1e-8, speed, None, 1.0);
                assert!((left.tff_raw_nm - right.tff_raw_nm).abs() < 1e-6);
                assert!((left.base_torque_nm - right.base_torque_nm).abs() < 1e-6);
            }
        }
        let mut rt = FrictionRuntime::default();
        let mut previous: Option<FrictionOutput> = None;
        for tick in 0..1000 {
            let speed = if tick % 2 == 0 { 0.0001 } else { -0.0001 };
            let out = rt.update(
                Duration::from_millis(tick * 2),
                1.0,
                speed,
                true,
                &cfg,
                None,
                1.0,
                10.0,
            );
            if let Some(previous) = previous {
                assert!((out.base_torque_nm - previous.base_torque_nm).abs() < 0.0011);
                assert!((out.tff_raw_nm - previous.tff_raw_nm).abs() < 0.0011);
            }
            previous = Some(out);
        }
    }

    #[test]
    fn retract_origin_protection_invalid_feedback_and_off_remove_assist() {
        let cfg = PullCordConfig::default();
        for (length, speed, permitted) in [
            (1.0, -0.1, true),
            (0.0, 0.0, true),
            (1.0, 0.1, false),
            (1.0, f64::NAN, true),
            (f64::NAN, 0.0, true),
            (cfg.zone_alert_start_m, 0.1, true),
        ] {
            let mut rt = FrictionRuntime::default();
            for tick in 0..500 {
                rt.update(
                    Duration::from_millis(tick * 2),
                    1.0,
                    0.1,
                    true,
                    &cfg,
                    None,
                    1.0,
                    10.0,
                );
            }
            let out = rt.update(
                Duration::from_secs(1),
                length,
                speed,
                permitted,
                &cfg,
                None,
                1.0,
                10.0,
            );
            assert_eq!(out.tff_raw_nm, 0.0);
            assert_eq!(out.base_torque_nm, retract_torque_nm(length, speed, &cfg));
            assert!(!out.active);
        }
        let mut cfg = cfg;
        cfg.friction_compensation.mode = FrictionMode::Off;
        let out = settle(&cfg, 1.0, 0.1, Some(calibration()), 1.0);
        assert_eq!(out.source, FrictionSource::Off);
        assert_eq!(out.tff_raw_nm, 0.0);
        assert_eq!(out.base_torque_nm, cfg.torque_near_nm);
    }

    #[test]
    fn missing_invalid_or_deselected_factory_data_uses_manual_estimate_once() {
        let mut cfg = PullCordConfig::default();
        let mut bad = calibration();
        bad.static_pos_raw_nm = f64::NAN;
        for cal in [None, Some(bad)] {
            let out = settle(&cfg, 1.0, 0.1, cal, 1.5);
            assert_eq!(out.source, FrictionSource::Manual);
            assert!((out.tff_raw_nm + 0.045 * 0.8 * 1.5).abs() < 1e-12);
        }
        cfg.friction_compensation.use_factory_calibration = false;
        assert_eq!(
            settle(&cfg, 1.0, 0.1, Some(calibration()), 1.0).source,
            FrictionSource::Manual
        );
    }

    #[test]
    fn wall_time_slew_and_lowered_caps_cannot_accumulate_hidden_torque() {
        let mut cfg = PullCordConfig::default();
        let mut rt = FrictionRuntime::default();
        for _ in 0..500 {
            let out = rt.update(Duration::ZERO, 1.0, 0.0, true, &cfg, None, 1.0, 10.0);
            assert_eq!(out.tff_raw_nm, 0.0);
            assert_eq!(out.base_torque_nm, cfg.torque_near_nm);
        }
        let out = rt.update(
            Duration::from_secs(10),
            1.0,
            0.0,
            true,
            &cfg,
            None,
            1.0,
            10.0,
        );
        assert!(
            (-out.tff_raw_nm - 0.01).abs() < 1e-12,
            "long pauses are capped at 20 ms"
        );
        cfg.friction_compensation.max_torque_nm = 0.001;
        let out = rt.update(
            Duration::from_millis(10_002),
            1.0,
            0.0,
            true,
            &cfg,
            None,
            1.0,
            10.0,
        );
        assert!(out.limited && -out.tff_raw_nm <= 0.001);
        cfg.max_torque_permille = 0;
        let out = rt.update(
            Duration::from_millis(10_004),
            1.0,
            0.0,
            true,
            &cfg,
            None,
            1.0,
            10.0,
        );
        assert_eq!(out.tff_raw_nm, 0.0);
    }

    #[test]
    fn friction_reserves_the_quantized_pd_budget_in_both_directions() {
        let mut cfg = PullCordConfig::default();
        cfg.max_torque_permille = 7; // raw ceiling = 0.07 N m
        for speed in [-0.004, 0.0, 0.004, 0.05] {
            let out = settle(&cfg, 1.0, speed, Some(calibration()), 1.3);
            let pd_raw =
                f64::from(torque_to_limit_permille(out.base_torque_nm, 10.0, 1.3).min(7)) * 0.01;
            assert!(pd_raw + out.tff_raw_nm.abs() <= 0.07 + 1e-12);
        }
    }

    #[test]
    fn legacy_settings_gain_defaults_without_losing_existing_tuning() {
        let mut value = serde_json::to_value(PullCordConfig::default()).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .remove("friction_compensation");
        value["zero_friction_assist_nm"] = 0.071.into();
        let cfg: PullCordConfig = serde_json::from_value(value).unwrap();
        cfg.validate().unwrap();
        assert_eq!(
            cfg.friction_compensation,
            FrictionCompensationConfig::default()
        );
        assert_eq!(cfg.zero_friction_assist_nm, 0.071);
        let mut short_near = cfg;
        short_near.zone_near_end_m = 0.01;
        short_near.validate().unwrap();
        assert_eq!(
            settle(&short_near, 0.01, 0.1, None, 1.0).base_torque_nm,
            short_near.torque_near_nm
        );
        let mut value = serde_json::to_value(cfg).unwrap();
        value["friction_compensation"]["static_ratio"] = 1.0.into();
        assert!(serde_json::from_value::<PullCordConfig>(value.clone())
            .unwrap()
            .validate()
            .is_err());
        value["friction_compensation"]["static_ratio"] = 0.5.into();
        value["friction_compensation"]["typo"] = true.into();
        assert!(serde_json::from_value::<PullCordConfig>(value).is_err());
    }
}
