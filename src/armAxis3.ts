import type { Axis3Calibration, Axis3ModelInfo, El05GravityReference } from "./el05Api.ts";

export const AXIS3_GRAVITY = 9.80665;
export function axis3Angle(reference: El05GravityReference | null | undefined, encoder: number | null | undefined): number | null {
  if (!reference?.axis3 || encoder == null || !Number.isFinite(encoder)) return null;
  return reference.axis3.joint_angle_rad + reference.axis3.encoder_direction * (encoder - reference.position_rad);
}
export function axis3Torque(model: Axis3ModelInfo, angle: number, direction: 1 | -1, scale: number): number {
  return direction * scale * (model.sin_nm * Math.sin(angle) + model.cos_nm * Math.cos(angle));
}
export function axis3CalibrationMatches(reference: El05GravityReference | null | undefined, calibration: Axis3Calibration): boolean {
  return reference?.axis3 != null && reference.axis3.encoder_direction === calibration.encoder_direction
    && Math.abs(reference.axis3.joint_angle_rad - calibration.joint_angle_rad) < 1e-9;
}
