// Follow parameter and input shapes from string_chassis main 20133ac.
import type { Hopea3State } from "./types";

export interface PullChassisConfig {
  follow: FollowParameters;
  mode: "behind" | "ahead";
  source: "live" | "manual";
  cord_exit_side: "driven_axle" | "passive_axle";
  direction_inverted: boolean;
  max_linear_mps: number;
  max_angular_rad_s: number;
  acceleration_mps2: number;
  /** Null uses the live pull-cord run's applied retract target speed. */
  retract_brake_speed_mps: number | null;
  turn_gain: number;
  kd_si: [number, number, number];
  max_torque_permille: [number, number, number];
}

export interface PullChassisRunState {
  running: boolean;
  fault: string | null;
  stop_confirmed: boolean;
  config: PullChassisConfig;
  input: FollowInput;
  manual_enabled: boolean;
  follow_status: FollowStatus & { travel_mode: "behind" | "ahead" | null };
  retract_braking: boolean;
  retract_brake_speed_mps: number | null;
  retract_brake_deceleration_mps2: number;
  chassis: Hopea3State;
}

export interface PullChassisStatus {
  connected: boolean;
  iface: string | null;
  needs_stop: boolean;
  run: PullChassisRunState | null;
}


export interface PaceFollowParameters {
  enabled: boolean;
  targetLengthM: number;
  stopMarginM: number;
  filterMs: number;
  lengthGainPerS: number;
  lengthDeadbandM: number;
  maxCorrectionMps: number;
  accelerationMps2: number;
  decelerationMps2: number;
}

export interface BehindSpeedParameters {
  enabled: boolean;
  payoutSpeedRatio: number;
  neutralBoostRatio: number;
  retractGain: number;
  speedDeadbandMps: number;
  payoutBlendMps: number;
  filterMs: number;
  payoutAccelerationMps2: number;
  responseAccelerationMps2: number;
  decelerationMps2: number;
  stopDecelerationMps2: number;
  switchDecelerationMps2: number;
  adaptiveEnabled: boolean;
  adaptiveGain: number;
  adaptiveAccelerationMps2: number;
  adaptiveDecelerationMps2: number;
  payoutStopEnabled: boolean;
  payoutStopSpeedMps: number;
  payoutStopConfirmMs: number;
  payoutStopMinPayoutM: number;
  payoutStopDecelerationMps2: number;
  pullBackAccelerationMps2: number;
  pullBackAccelerationGain: number;
  pullBackMinRetractM: number;
  pullBackLengthGain: number;
  pullBackDecelerationMps2: number;
}

export interface FollowInput {
  /** Legacy shaft telemetry; resumption uses negative cordSpeedMps instead. */
  pullBackSpeedMps?: number | null;
  cordLengthM: number | null;
  cordSpeedMps: number | null;
  encoderAngleDeg: number | null;
  /** Backend acquisition time for rate estimation; optional on legacy input. */
  encoderSampleTimeS?: number | null;
}

export interface FollowParameters {
  startCordLengthM: number;
  behindStartCordLengthM: number;
  behindTargetCordLengthM: number;
  behindStopCordLengthM: number;
  behindAheadSwitchCordLengthM: number;
  behindSpeed: BehindSpeedParameters;
  pace: PaceFollowParameters;
  lengthGainPerS: number;
  cordSpeedFeedforward: number;
  maxSpeedMps: number;
  allowReverse: boolean;
  maxReverseSpeedMps: number;
  encoderCenterDeg: number;
  directionDeadbandDeg: number;
  smoothingMs: number;
}

export interface FollowStatus {
  pace_active?: boolean;
  pace_tracking?: boolean;
  pace_reference_length_m?: number | null;
  pace_stop_length_m?: number | null;
  estimated_user_speed_mps?: number | null;
  pace_forward_speed_mps?: number | null;
  pace_rotation_guard?: boolean;
  /** Longitudinal target marker; current backends always report 1 when available. */
  pace_turn_scale?: number | null;
  pace_correction_mps?: number;
  pace_target_mps?: number | null;
  pace_min_length_m?: number | null;
  braking?: "stop_line" | "ahead_switch" | null;
  payout_stop?: boolean;
  payout_stop_resume_length_m?: number | null;
  /** New backend: stopped retraction resumes the Behind direction. */
  retract_resume?: boolean;
  /** New length-bounded restart; absent on the previous backend. */
  retract_distance_m?: number;
  retract_braking?: boolean;
  /** Old backends may still report their former Ahead transition. */
  pull_back_ahead: boolean;
  waiting_for_pull_back: boolean;
}
/** Wheel output conversion from string_chassis/web/src/speedUnits.ts. */
export interface WheelGeometry {
  wheel_diameter_m: number;
  gear_ratio: number;
}

export function outputSpeedRadS(linearSpeedMps: number, geometry: WheelGeometry): number {
  return linearSpeedMps * 2 * geometry.gear_ratio / geometry.wheel_diameter_m;
}

