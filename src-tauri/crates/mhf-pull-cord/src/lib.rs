//! MHF5010 current and speed control based on string_chassis main.
//! The transport and desktop lifecycle belong to the caller.
#![forbid(unsafe_code)]

mod lingkong_pull;
pub mod chassis_model;
pub mod motion;
pub mod pull_cord;
pub mod trajectory;
// Retain the upstream diagnostic layout, including fields used only by its
// chassis application, so future upstream changes stay easy to compare.
#[allow(dead_code)]
mod runtime_support;

use can_transport::CanBus;
pub use lingkong_pull::{LingkongPull, PullCordEncoder};
pub use pull_cord::{Phase, PullCordConfig, PullCordMode, PullCordState};

pub fn chassis_defaults() -> chassis_model::ChassisConfig {
    serde_json::from_str(include_str!("../chassis-defaults.json"))
        .expect("embedded main-branch chassis defaults")
}
use std::sync::{atomic::AtomicBool, Arc, Mutex};

fn signed_degrees(value: f64) -> f64 {
    (value + 180.0).rem_euclid(360.0) - 180.0
}

/// Use the deployed main-branch defaults, not the historical HEX4310 defaults.
pub fn default_config() -> PullCordConfig {
    let config = serde_json::from_str(include_str!("../defaults.json"))
        .expect("embedded main-branch pull-cord defaults");
    validate_config(config).expect("valid main-branch pull-cord defaults")
}

pub fn validate_config(mut config: PullCordConfig) -> Result<PullCordConfig, String> {
    lingkong_pull::normalize_mhf_config(&mut config);
    lingkong_pull::validate_mhf_config(&config)?;
    Ok(config)
}

pub async fn start(
    bus: Arc<dyn CanBus>,
    node_id: u8,
    config: PullCordConfig,
    cancel: Arc<AtomicBool>,
    encoder: PullCordEncoder,
) -> Result<LingkongPull, String> {
    LingkongPull::start(
        bus,
        node_id,
        config,
        Arc::new(Mutex::new(
            runtime_support::PullCordCommandDiagnostics::default(),
        )),
        cancel,
        encoder,
    )
    .await
}

/// Also available after a failed startup, when no running handle was returned.
pub async fn close_motor(bus: &dyn CanBus, node_id: u8, mode: PullCordMode) -> Result<(), String> {
    lingkong_pull::disable_motor(bus, node_id, mode).await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_the_main_branch_assembly() {
        let config = default_config();
        assert_eq!(config.drum_radius_m, 0.026);
        assert!(!config.retract_positive);
        assert_eq!(config.homing_stall_hold_ms, 600);
        let a2 = config.a2_speed.unwrap();
        assert_eq!(
            (a2.payout_raw, a2.near_raw, a2.retract_raw),
            (100, 150, 200)
        );
        assert_eq!(a2.max_raw, 300);
        assert_eq!(a2.speed_kp, 20);
    }

    #[test]
    fn invalid_raw_limits_and_overspeed_are_rejected_before_start() {
        let mut config = default_config();
        config.a2_speed.as_mut().unwrap().payout_raw = 151;
        assert!(validate_config(config).is_err());
        config = default_config();
        config.homing_speed_m_s = 3.0;
        assert!(validate_config(config).is_err());
    }
}
