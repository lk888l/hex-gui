import type { El05State } from "./el05Api";

type GravityState = Pick<El05State, "connected" | "attached" | "online" | "motor_id" | "protocol"
  | "feedback_age_ms" | "position_rad" | "unwrapped_position_rad" | "velocity_rad_s" | "position_velocity_rad_s"
  | "enabled" | "fault_bits" | "fault_code" | "warning_code" | "gravity_compensation" | "experiment">;

export type GravityBlocker = "connect" | "select" | "feedback" | "position" | "position_saturated" | "fault" | "busy"
  | "experiment" | "active" | "enabled" | "moving" | "reference" | "reference_motor" | "model" | "torque";

export function gravityReadiness(state: GravityState | null, options: {
  busy: boolean; validModel: boolean; insufficientTorque: boolean;
}) {
  const fresh = !!state?.connected && state.attached && state.online
    && state.feedback_age_ms != null && Number.isFinite(state.feedback_age_ms)
    && state.feedback_age_ms >= 0 && state.feedback_age_ms <= 150;
  const position = state?.unwrapped_position_rad;
  const hasPosition = position != null && Number.isFinite(position);
  const positionSaturated = state?.protocol === "StandardMit" && state.position_rad != null
    && Number.isFinite(state.position_rad) && Math.abs(state.position_rad) >= 12.57 - 1e-6;
  const atRest = state?.velocity_rad_s != null && Number.isFinite(state.velocity_rad_s)
    && Math.abs(state.velocity_rad_s) <= 0.1
    && (state.position_velocity_rad_s == null || (Number.isFinite(state.position_velocity_rad_s)
      && Math.abs(state.position_velocity_rad_s) <= 0.1));
  const common: GravityBlocker | null = !state?.connected ? "connect" : !state.attached || state.motor_id == null ? "select"
    : !fresh ? "feedback" : !hasPosition ? "position" : positionSaturated ? "position_saturated"
    : state.fault_bits !== 0 || state.fault_code !== 0 || (state.protocol === "StandardMit" && state.warning_code !== 0) ? "fault"
    : options.busy ? "busy" : state.experiment.active ? "experiment" : state.gravity_compensation.active ? "active"
    : state.enabled ? "enabled" : null;
  const captureBlocker: GravityBlocker | null = common ?? (!atRest ? "moving" : null);
  const reference = state?.gravity_compensation.reference;
  const startBlocker: GravityBlocker | null = common ?? (!reference ? "reference"
    : reference.motor_id !== state?.motor_id ? "reference_motor"
    : !options.validModel ? "model" : options.insufficientTorque ? "torque" : null);
  return { fresh, position: hasPosition ? position : null, positionSaturated, atRest, captureBlocker, startBlocker,
    canCapture: captureBlocker === null, canStart: startBlocker === null };
}
