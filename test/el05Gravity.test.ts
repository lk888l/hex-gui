import assert from "node:assert/strict";
import test from "node:test";
import { gravityReadiness } from "../src/el05Gravity.ts";

function stoppedMitState(): NonNullable<Parameters<typeof gravityReadiness>[0]> & { identity: null; status_code: 0 } {
  return {
    protocol: "StandardMit", connected: true, attached: true, online: true, motor_id: 1,
    identity: null, status_code: 0, enabled: false, feedback_age_ms: 16,
    position_rad: -3.352575417715723, unwrapped_position_rad: -3.352575417715723, velocity_rad_s: 0.012210012210012167,
    position_velocity_rad_s: null, fault_bits: 0, fault_code: 0, warning_code: 0,
    gravity_compensation: { active: false, elapsed_s: 0, reference: null, config: null, angle_rad: null,
      gravity_torque_nm: 0, damping_torque_nm: 0, command_torque_nm: 0, limited: false, reason: null },
    experiment: { active: false, config: null, samples: 0, elapsed_s: 0, reason: null, dropped_frames: 0 },
  };
}
const ready = { busy: false, validModel: true, insufficientTorque: false };

test("fresh standard MIT position is visible and capturable before any reference or UID exists", () => {
  const state = stoppedMitState(), result = gravityReadiness(state, ready);
  assert.equal(result.position, state.unwrapped_position_rad);
  assert.equal(result.canCapture, true);
  assert.equal(result.captureBlocker, null);
  assert.equal(result.canStart, false);
  assert.equal(result.startBlocker, "reference");
  state.gravity_compensation.reference = { id: "session-reference", motor_id: 1, position_rad: state.unwrapped_position_rad!, captured_at_s: 1 };
  assert.equal(gravityReadiness(state, ready).canStart, true);
});

test("missing or nonfinite position, stale feedback, enabled control and motion explain why capture is blocked", () => {
  const cases = [
    { change: { unwrapped_position_rad: null }, reason: "position" },
    { change: { unwrapped_position_rad: Number.NaN }, reason: "position" },
    { change: { feedback_age_ms: 151 }, reason: "feedback" },
    { change: { feedback_age_ms: Number.NaN }, reason: "feedback" },
    { change: { enabled: true }, reason: "enabled" },
    { change: { velocity_rad_s: 0.11 }, reason: "moving" },
    { change: { velocity_rad_s: null }, reason: "moving" },
    { change: { position_velocity_rad_s: 0.11 }, reason: "moving" },
    { change: { warning_code: 1 }, reason: "fault" },
  ];
  for (const { change, reason } of cases) {
    const result = gravityReadiness({ ...stoppedMitState(), ...change }, ready);
    assert.equal(result.canCapture, false, JSON.stringify(change));
    assert.equal(result.captureBlocker, reason, JSON.stringify(change));
    if (reason === "position") assert.equal(result.position, null);
  }
});

test("capture and start retain connection, operation and reference ownership requirements", () => {
  assert.equal(gravityReadiness(null, ready).captureBlocker, "connect");
  assert.equal(gravityReadiness({ ...stoppedMitState(), attached: false }, ready).captureBlocker, "select");
  assert.equal(gravityReadiness(stoppedMitState(), { ...ready, busy: true }).captureBlocker, "busy");
  const state = stoppedMitState();
  state.gravity_compensation.reference = { id: "other-motor", motor_id: 2, position_rad: 0, captured_at_s: 1 };
  assert.equal(gravityReadiness(state, ready).startBlocker, "reference_motor");
  state.gravity_compensation.reference.motor_id = 1;
  assert.equal(gravityReadiness(state, { ...ready, validModel: false }).startBlocker, "model");
  assert.equal(gravityReadiness(state, { ...ready, insufficientTorque: true }).startBlocker, "torque");
});

test("standard MIT endpoint saturation blocks gravity reference until the raw position is reset", () => {
  for (const position of [-12.57, 12.57, 12.57 - 0.0000005]) {
    const state = { ...stoppedMitState(), position_rad: position, unwrapped_position_rad: 0 };
    const result = gravityReadiness(state, ready);
    assert.equal(result.positionSaturated, true);
    assert.equal(result.captureBlocker, "position_saturated");
    assert.equal(result.startBlocker, "position_saturated");
  }
  const belowEndpoint = gravityReadiness({ ...stoppedMitState(), position_rad: 12.57 - 0.00001 }, ready);
  assert.equal(belowEndpoint.canCapture, true, "only the wire endpoint tolerance indicates saturation");
  const unwrapped = gravityReadiness({ ...stoppedMitState(), position_rad: 0.0001918, unwrapped_position_rad: 12.57 }, ready);
  assert.equal(unwrapped.canCapture, true, "unwrapped coordinates are not the wire range indicator");
});
