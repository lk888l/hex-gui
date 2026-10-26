import { invoke } from "@tauri-apps/api/core";

export type El05Mode = "Mit" | "PositionPp" | "Velocity" | "Current" | "PositionCsp";
export type El05Protocol = "Native" | "StandardMit";
export type El05Target =
  | { kind: "Mit"; position_rad: number; velocity_rad_s: number; torque_nm: number; kp: number; kd: number }
  | { kind: "PositionPp"; position_rad: number; velocity_rad_s: number; acceleration_rad_s2: number; deceleration_rad_s2?: number; current_limit_a: number }
  | { kind: "Velocity"; velocity_rad_s: number; acceleration_rad_s2: number; current_limit_a: number }
  | { kind: "Current"; current_a: number }
  | { kind: "PositionCsp"; position_rad: number; velocity_rad_s: number; current_limit_a: number };

export interface El05Device {
  motor_id: number;
  uid: string | null;
  online: boolean;
  selected: boolean;
  feedback_age_ms: number | null;
  rx_count: number;
}

export interface El05Identity {
  motor_id: number;
  uid: string;
}

export interface El05Version {
  motor_id: number;
  value: number;
  bytes: number[];
  raw: number[];
}

export interface El05State {
  protocol: El05Protocol;
  reply_id: number | null;
  status_reliable: boolean;
  connected: boolean;
  attached: boolean;
  host_id: number;
  motor_id: number | null;
  identity: El05Identity | null;
  version: El05Version | null;
  mode: El05Mode;
  online: boolean;
  enabled: boolean;
  streaming: boolean;
  reporting: boolean;
  status_code: number;
  position_rad: number | null;
  mechanical_position_rad: number | null;
  velocity_rad_s: number | null;
  dictionary_velocity_rad_s: number | null;
  dictionary_velocity_age_ms: number | null;
  position_velocity_rad_s: number | null;
  unwrapped_position_rad: number | null;
  feedback_sequence: number;
  feedback_time_s: number | null;
  torque_nm: number | null;
  current_a: number | null;
  current_age_ms: number | null;
  voltage_v: number | null;
  temperature_c: number | null;
  fault_bits: number;
  fault_code: number;
  warning_code: number;
  faults: string[];
  feedback_age_ms: number | null;
  feedback_rate_hz: number | null;
  rx_count: number;
  tx_count: number;
  last_error: string | null;
  devices: El05Device[];
  experiment: El05ExperimentStatus;
  gravity_compensation: El05GravityCompensationStatus;
}

export interface El05GravityReference {
  id: string;
  motor_id: number;
  position_rad: number;
  captured_at_s: number;
}

export interface El05GravityCompensationConfig {
  reference_id: string;
  mass_kg: number;
  length_m: number;
  arm_mass_kg: number;
  compensation_scale: number;
  damping_nm_s_rad: number;
  torque_limit_nm: number;
  max_velocity_rad_s: number;
  max_temperature_c: number;
  ramp_s: number;
}

export interface El05GravityCompensationStatus {
  active: boolean;
  elapsed_s: number;
  reference: El05GravityReference | null;
  config: El05GravityCompensationConfig | null;
  angle_rad: number | null;
  gravity_torque_nm: number;
  damping_torque_nm: number;
  command_torque_nm: number;
  limited: boolean;
  reason: string | null;
}

export type El05ExperimentKind = "Record" | "VelocitySteps" | "TorqueRamp" | "StaticTorque" | "PositionSteps" | "SineSweep" | "Impedance" | "Backdrive" | "ThermalHold" | "SlowTriangle" | "StaticFriction";
export interface El05PendulumConfig {
  mass_kg: number;
  length_m: number;
  arm_mass_kg: number;
  downward_zero_rad: number;
  zero_measured: boolean;
  extra_inertia_kg_m2?: number;
}
export interface El05ExperimentConfig {
  kind: El05ExperimentKind;
  duration_s: number;
  velocity_rad_s: number;
  torque_nm: number;
  kd: number;
  max_temperature_c: number;
  pendulum?: El05PendulumConfig | null;
  kp?: number;
  center_angle_rad?: number;
  position_amplitude_rad?: number;
  frequency_start_hz?: number;
  frequency_end_hz?: number;
  gravity_compensation?: boolean;
  disturbance_torque_nm?: number;
  origin_position_rad?: number;
  torque_ramp_rate_nm_s?: number;
  max_excursion_rad?: number;
  motion_threshold_rad?: number;
  motion_confirm_s?: number;
  settle_time_s?: number;
  friction_direction?: "Both" | "Positive" | "Negative";
}
export interface El05StaticFrictionPass {
  /** Absent in legacy single-direction recordings; infer from signed torque. */
  direction?: 1 | -1;
  started_s?: number;
  phase: "Settling" | "Ramping" | "Returning" | "Complete";
  outcome: "BreakawayDetected" | "AngleLimit" | "TorqueLimit" | "Timeout" | "NotSettled" | "Interrupted" | null;
  reference_position_rad: number | null;
  ramp_started_s: number | null;
  last_stationary_sample: El05TelemetrySample | null;
  onset_sample: El05TelemetrySample | null;
  confirmation_sample: El05TelemetrySample | null;
  peak_excursion_rad: number;
  return_started_s: number | null;
  return_settled?: boolean | null;
}
export interface El05StaticFrictionStatus extends El05StaticFrictionPass {
  completed_directions?: El05StaticFrictionPass[];
}
export interface El05ExperimentStatus {
  run_id?: number;
  recording_interval_s?: number;
  static_friction?: El05StaticFrictionStatus | null;
  active: boolean;
  config: El05ExperimentConfig | null;
  samples: number;
  elapsed_s: number;
  reason: string | null;
  dropped_frames: number;
}
export interface El05TelemetrySample {
  t_s: number;
  sequence: number;
  can_id: number;
  payload: number[];
  position_rad: number;
  unwrapped_position_rad: number;
  velocity_rad_s: number;
  position_velocity_rad_s: number | null;
  torque_nm: number;
  current_a: number | null;
  current_age_ms: number | null;
  dictionary_velocity_rad_s: number | null;
  dictionary_velocity_age_ms: number | null;
  temperature_c: number;
  fault_bits: number;
  status_code: number;
  target: El05Target | null;
  target_sequence: number;
  target_age_ms: number | null;
}
export interface El05ExperimentData {
  /** Absent in historical recordings, which used the native protocol. */
  protocol?: El05Protocol;
  feedback_velocity_bits?: 12 | 16;
  /** Multiplier applied to packed CAN feedback velocity; current recordings use +1. */
  feedback_velocity_sign?: -1 | 1;
  status: El05ExperimentStatus;
  motor_id: number | null;
  uid: string | null;
  samples: El05TelemetrySample[];
  commands: { t_s: number; completed_t_s: number; sequence: number; target: El05Target; can_id: number; payload: number[] }[];
  parameters: El05ParameterValue[];
  version: El05Version | null;
}

export interface El05ParameterValue {
  index: number;
  value: number;
  kind: "Float" | "U8" | "U16" | "U32";
}

export const el05Api = {
  connect: (iface: string, hostId: number, protocol: El05Protocol = "Native") => invoke<El05State>("el05_connect", { iface, hostId, protocol }),
  disconnect: () => invoke<void>("el05_disconnect"),
  getState: () => invoke<El05State | null>("el05_get_state"),
  scan: () => invoke<El05Device[]>("el05_scan"),
  selectMotor: (motorId: number) => invoke<El05State>("el05_select_motor", { motorId }),
  setMode: (mode: El05Mode) => invoke<El05State>("el05_set_mode", { mode }),
  enable: () => invoke<void>("el05_enable"),
  stop: () => invoke<void>("el05_stop"),
  clearFault: () => invoke<void>("el05_clear_fault"),
  setZero: () => invoke<void>("el05_set_zero"),
  readIdentity: () => invoke<El05Identity>("el05_read_identity"),
  changeId: (newId: number) => invoke<El05Identity>("el05_change_id", { newId }),
  readVersion: () => invoke<El05Version>("el05_read_version"),
  sendTarget: (target: El05Target, repeat: boolean) => invoke<void>("el05_send_target", { target, repeat }),
  readParameter: (index: number) => invoke<El05ParameterValue>("el05_read_parameter", { index }),
  writeParameter: (index: number, value: number) => invoke<void>("el05_write_parameter", { index, value }),
  startExperiment: (config: El05ExperimentConfig) => invoke<void>("el05_start_experiment", { config }),
  experimentData: (afterSequence?: number) => invoke<El05ExperimentData>("el05_experiment_data", { afterSequence }),
  captureGravityReference: () => invoke<El05GravityReference>("el05_capture_gravity_reference"),
  startGravityCompensation: (config: El05GravityCompensationConfig) => invoke<void>("el05_start_gravity_compensation", { config }),
  stopGravityCompensation: () => invoke<void>("el05_stop_gravity_compensation"),
};
