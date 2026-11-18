import assert from "node:assert/strict";
import test from "node:test";
import { axis3Angle, axis3CalibrationMatches, axis3Torque } from "../src/armAxis3.ts";
import type { Axis3ModelInfo, El05GravityReference } from "../src/el05Api.ts";

test("axis-3 preview uses the captured URDF pose, encoder origin and direction", () => {
  const reference: El05GravityReference = { id: "captured", motor_id: 7, position_rad: -2.5, captured_at_s: 1,
    axis3: { joint_angle_rad: 0.8, encoder_direction: -1 } };
  assert.ok(Math.abs(axis3Angle(reference, -2.9)! - 1.2) < 1e-12);
  assert.equal(axis3Angle(reference, null), null);
  assert.equal(axis3Angle(reference, NaN), null);
  assert.equal(axis3Angle({ ...reference, axis3: null }, -2.9), null);
  assert.equal(axis3CalibrationMatches(reference, reference.axis3!), true);
  assert.equal(axis3CalibrationMatches(reference, {joint_angle_rad: 0.8, encoder_direction: 1}), false);
  assert.equal(axis3CalibrationMatches(reference, {joint_angle_rad: 0, encoder_direction: -1}), false);
});

test("preview support torque includes the cosine component at URDF zero and maps to encoder direction", () => {
  const model = {sin_nm: 0.3, cos_nm: 0.7} as Axis3ModelInfo;
  assert.equal(axis3Torque(model, 0, 1, 1), 0.7);
  assert.equal(axis3Torque(model, 0, -1, 0.5), -0.35);
  assert.ok(Math.abs(axis3Torque(model, Math.PI / 2, 1, 1) - 0.3) < 1e-12);
  assert.equal(axis3Torque(model, 1.2, 1, 0), 0);
});
