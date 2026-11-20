// Desktop pull-cord configuration and telemetry, adapted from string_chassis main.
export interface A1CurrentConfig {
  homing_raw: number;
  near_raw: number;
  payout_current_raw: number;
  retract_current_raw: number;
  alert_raw: number;
  max_raw: number;
  full_current_raw: number;
}

export interface A2SpeedConfig {
  homing_raw: number;
  near_raw: number;
  payout_raw: number;
  retract_raw: number;
  alert_raw: number;
  max_raw: number;
  /** Reference for limit percentages, independently calibrated from feedback. */
  full_torque_raw: number;
  speed_kp: number;
  speed_ki: number;
  speed_kd: number;
  speed_ramp_dps2: number;
}

export type PullCordMode = "a1_current" | "a2_speed";

export interface PullCordConfig {
  control_mode: PullCordMode;
  drum_radius_m: number;
  cord_total_length_m: number;
  retract_positive: boolean;
  a1_current: A1CurrentConfig | null;
  a2_speed: A2SpeedConfig | null;
  spring_compensation: {
    enabled: boolean;
    stiffness_nm_per_rad: number;
    preload_length_m: number;
    gain: number;
    alert_scale: number;
    max_torque_nm: number;
    slew_rate_nm_s: number;
  };
  friction_compensation: {
    mode: "preload" | "off";
    use_factory_calibration: boolean;
    static_ratio: number;
    kinetic_ratio: number;
    transition_speed_m_s: number;
    max_torque_nm: number;
    slew_rate_nm_s: number;
    origin_fade_m: number;
  };

  homing_torque_nm: number;
  homing_speed_m_s: number;
  homing_stall_speed_m_s: number;
  homing_stall_hold_ms: number;
  homing_timeout_s: number;

  zone_near_end_m: number;
  zone_alert_start_m: number;

  torque_near_nm: number;
  torque_alert_nm: number;
  retract_boost_torque_nm: number;

  zero_friction_assist_nm: number;

  alert_pulse_hz: number;
  alert_duty: number;
  alert_pulse_nm: number;

  kd_nm_per_rev_per_s: number;
  retract_speed_m_s: number;

  max_torque_permille: number;
}

export type Zone = "near_origin" | "ramp_up" | "alert";
export type Phase = "idle" | "homing" | "running" | "fault";

/** Planar feedback relative to the outlet. +X points out along the calibrated
 * centre line, +Y to its left. time_us is monotonic within a motor run. */
export interface CordEndpoint {
  frame_id: number;
  segment: number;
  time_us: number;
  x_m: number;
  y_m: number;
  length_m: number;
  bearing_rad: number;
  radial_speed_m_s: number;
  velocity_m_s: [number, number] | null;
}

export interface TrajectoryCursor {
  frame_id: number;
  seq: number;
}

export interface TrajectoryUpdate {
  cursor: TrajectoryCursor;
  latest: CordEndpoint | null;
  retained_points: number;
  points: CordEndpoint[];
}

export interface PullCordState {
  running: boolean;
  node_id: number | null;
  phase: Phase;
  fault: string | null;
  config: PullCordConfig;

  length_m: number | null;
  cord_speed_m_s: number | null;
  zone: Zone | null;
  commanded_torque_nm: number | null;
  spring_torque_nm: number;
  spring_tff_nm: number;
  spring_compensation_limited: boolean;
  zero_friction_tff_nm: number;
  zero_friction_active: boolean;
  friction_tff_raw_nm: number;
  friction_source: "off" | "factory" | "manual";
  friction_preloading: boolean;
  friction_compensation_limited: boolean;
  alert_pulse_nm: number;
  alert_active: boolean;
  measured_torque_nm: number | null;
  measured_force_n: number | null;
  current_command_raw: number;
  torque_limit_raw: number;
  global_torque_limit_raw: number | null;
  speed_command_dps: number;
  encoder_offset_deg: number | null;
  stationary_current_raw: number | null;
  measured_current_raw: number | null;
  motor_speed_dps: number | null;
  position_angle_deg: number | null;
  bus_voltage_v: number | null;
  a1_feedback_age_ms: number | null;
  a2_feedback_age_ms: number | null;
  angle_feedback_age_ms: number | null;
  motor_status_age_ms: number | null;
  commanded_force_n: number | null;
  vdes_rev_per_s: number;
  limit_permille: number;

  motor_temp_c: number | null;
  driver_temp_c: number | null;
  peak_torque_nm: number | null;
  torque_factor: number | null;
  detailed_error: number | null;
  online: boolean;
  homing_progress: number;
}
