pub(crate) use crate::motion::*;
use crate::pull_cord::{Phase, Zone};
use std::time::Instant;

pub(crate) struct PullCordCommandDiagnostics {
    /// Monotonic freshness fence for the independent chassis controller.
    pub updated_at: Option<Instant>,
    pub active: bool,
    pub node_id: Option<u8>,
    pub run_id: u64,
    pub generation: u64,
    pub run_elapsed_us: u64,
    pub phase: Phase,
    pub zone: Option<Zone>,
    pub length_m: Option<f64>,
    pub cord_speed_m_s: Option<f64>,
    pub base_torque_nm: Option<f64>,
    pub cord_tff_nm: f64,
    pub spring_torque_nm: f64,
    pub spring_tff_nm: f64,
    pub spring_compensation_limited: bool,
    pub friction_tff_raw_nm: f64,
    pub friction_preloading: bool,
    pub friction_compensation_limited: bool,
    pub wire_tff_nm: f32,
    pub vdes_rev_per_s: f32,
    pub limit_permille: u16,
    pub send_ok: bool,
}

impl Default for PullCordCommandDiagnostics {
    fn default() -> Self {
        Self {
            updated_at: None,
            active: false,
            node_id: None,
            run_id: 0,
            generation: 0,
            run_elapsed_us: 0,
            phase: Phase::Idle,
            zone: None,
            length_m: None,
            cord_speed_m_s: None,
            base_torque_nm: None,
            cord_tff_nm: 0.0,
            spring_torque_nm: 0.0,
            spring_tff_nm: 0.0,
            spring_compensation_limited: false,
            friction_tff_raw_nm: 0.0,
            friction_preloading: false,
            friction_compensation_limited: false,
            wire_tff_nm: 0.0,
            vdes_rev_per_s: 0.0,
            limit_permille: 0,
            send_ok: false,
        }
    }
}
