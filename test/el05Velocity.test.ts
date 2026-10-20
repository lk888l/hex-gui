import assert from "node:assert/strict";
import test from "node:test";
import type { El05ExperimentData, El05TelemetrySample } from "../src/el05Api.ts";
import { normalizeExperimentFeedbackVelocity } from "../src/el05Velocity.ts";

function sample(encodedVelocity: number, frameType = 2): El05TelemetrySample {
  const wireVelocity = encodedVelocity * 100 / 65535 - 50;
  return { t_s: 0, sequence: 1, can_id: frameType * 0x1000000 + 0x1fd,
    payload: [0x80, 0, encodedVelocity >> 8, encodedVelocity & 255, 0x80, 0, 1, 0],
    position_rad: 0.1, unwrapped_position_rad: 0.1, velocity_rad_s: wireVelocity,
    position_velocity_rad_s: 0.4, dictionary_velocity_rad_s: 0.5, dictionary_velocity_age_ms: 2,
    torque_nm: 0.1, current_a: 0.2, current_age_ms: 1, temperature_c: 25,
    status_code: 2, fault_bits: 0, target_sequence: 1, target_age_ms: 1,
    target: { kind: "Mit", position_rad: 0.1, velocity_rad_s: 0.7, torque_nm: 0.2, kp: 5, kd: 0.2 } };
}
function data(samples: El05TelemetrySample[]): El05ExperimentData {
  return { status: { active: false, config: { kind: "Record", duration_s: 1, velocity_rad_s: 0.7,
    torque_nm: 0.2, kd: 0.2, max_temperature_c: 70 }, samples: samples.length,
    elapsed_s: 1, reason: "duration completed", dropped_frames: 0 },
    motor_id: 1, uid: "test", samples, parameters: [], version: null,
    commands: [{ t_s: 0, completed_t_s: 0, sequence: 1,
      target: { kind: "Mit", position_rad: 0.1, velocity_rad_s: 0.7, torque_nm: 0.2, kp: 5, kd: 0.2 },
      can_id: 0x010001fd, payload: [0, 0, 0, 0, 0, 0, 0, 0] }] };
}

test("legacy inverted type-2 feedback is restored to the direct wire direction without changing other channels", () => {
  const inverted = sample(30000);
  inverted.velocity_rad_s *= -1;
  const original = data([inverted]), snapshot = structuredClone(original);
  const result = normalizeExperimentFeedbackVelocity(original);
  assert.equal(result.changed, true);
  assert.equal(result.data.feedback_velocity_sign, 1);
  assert.ok(result.data.samples[0].velocity_rad_s < 0);
  assert.equal(result.data.samples[0].velocity_rad_s, -original.samples[0].velocity_rad_s);
  assert.deepEqual(original, snapshot, "normalization does not mutate its input");
  const { velocity_rad_s: originalVelocity, ...originalOther } = original.samples[0];
  const { velocity_rad_s: correctedVelocity, ...correctedOther } = result.data.samples[0];
  assert.deepEqual(correctedOther, originalOther, "wire payload, targets, Iq, mechVel and position derivatives stay untouched");
  assert.deepEqual(result.data.commands, original.commands);
  assert.deepEqual(result.data.status.config, original.status.config);
});

test("type-24 direct feedback keeps its sign and mixed inverted samples are restored once", () => {
  const wire = sample(35000, 24), inverted = sample(30000, 24);
  inverted.velocity_rad_s *= -1;
  const result = normalizeExperimentFeedbackVelocity(data([wire, inverted]));
  assert.equal(result.changed, true);
  assert.ok(result.data.samples[0].velocity_rad_s > 0);
  assert.equal(result.data.samples[0].velocity_rad_s, wire.velocity_rad_s);
  assert.equal(result.data.samples[1].velocity_rad_s, -inverted.velocity_rad_s);
});

test("an already-corrected archive gets a direction marker and never flips again after export/import", () => {
  const canonical = sample(30000);
  const first = normalizeExperimentFeedbackVelocity(data([canonical]));
  assert.equal(first.changed, true);
  assert.equal(first.data.samples[0].velocity_rad_s, canonical.velocity_rad_s);
  const replay = JSON.parse(JSON.stringify(first.data));
  const second = normalizeExperimentFeedbackVelocity(replay);
  assert.equal(second.changed, false);
  assert.equal(second.data, replay);
  assert.equal(second.data.samples[0].velocity_rad_s, canonical.velocity_rad_s);
});

test("native +1 marked data is trusted without applying a second correction", () => {
  const original = data([sample(30000)]);
  original.feedback_velocity_sign = 1;
  original.samples[0].payload = [];
  const result = normalizeExperimentFeedbackVelocity(original);
  assert.deepEqual(result, { data: original, changed: false });
  assert.equal(result.data, original);
});

test("an explicit old -1 multiplier restores curves and events even without raw payloads", () => {
  const inverted = sample(30000);
  inverted.velocity_rad_s *= -1;
  inverted.payload = [];
  const original = data([inverted]);
  original.feedback_velocity_sign = -1;
  original.status.static_friction = { phase: "Complete", outcome: "BreakawayDetected",
    reference_position_rad: 0, ramp_started_s: 0, return_started_s: 0.8,
    last_stationary_sample: { ...inverted }, onset_sample: { ...inverted }, confirmation_sample: { ...inverted },
    peak_excursion_rad: 0.01 };
  const snapshot = structuredClone(original);
  const result = normalizeExperimentFeedbackVelocity(original);
  assert.equal(result.changed, true);
  assert.equal(result.data.feedback_velocity_sign, 1);
  assert.equal(result.data.samples[0].velocity_rad_s, -inverted.velocity_rad_s);
  const events = result.data.status.static_friction!;
  for (const event of [events.last_stationary_sample, events.onset_sample, events.confirmation_sample]) {
    assert.equal(event!.velocity_rad_s, -inverted.velocity_rad_s);
  }
  assert.deepEqual(original, snapshot);
  const replay = normalizeExperimentFeedbackVelocity(result.data);
  assert.equal(replay.changed, false);
  assert.equal(replay.data, result.data);
});

test("one unrecognizable curve frame prevents guessing or partially migrating a historical run", () => {
  const malformed = [
    { payload: [] }, { payload: [0, 0, 0, 0, 0, 0, 0, 256] },
    { payload: [0, 0, 0, 0, 0, 0, 0, 0.5] }, { can_id: 0 }, { can_id: 0x110001fd },
    { velocity_rad_s: 0.1234567 }, { velocity_rad_s: Number.NaN },
  ];
  for (const invalid of malformed) {
    const original = data([sample(30000), { ...sample(35000), ...invalid }]);
    const result = normalizeExperimentFeedbackVelocity(original);
    assert.equal(result.changed, false, JSON.stringify(invalid));
    assert.equal(result.data, original);
    assert.equal(result.data.feedback_velocity_sign, undefined);
    assert.equal(result.data.samples[0].velocity_rad_s, original.samples[0].velocity_rad_s);
  }
});

test("static-friction event frames receive the same evidenced correction as the curve", () => {
  const inverted = sample(30000);
  inverted.velocity_rad_s *= -1;
  const original = data([inverted]);
  const canonicalEvent = sample(35000, 24);
  const stationary = sample(32768), onset = sample(30000, 24);
  stationary.velocity_rad_s *= -1;
  onset.velocity_rad_s *= -1;
  original.status.static_friction = { phase: "Complete", outcome: "BreakawayDetected",
    reference_position_rad: 0, ramp_started_s: 0, return_started_s: 0.8, return_settled: true,
    last_stationary_sample: stationary, onset_sample: onset, confirmation_sample: canonicalEvent,
    peak_excursion_rad: 0.01 };
  const snapshot = structuredClone(original);
  const result = normalizeExperimentFeedbackVelocity(original);
  assert.equal(result.changed, true);
  const calibration = result.data.status.static_friction!;
  assert.equal(calibration.last_stationary_sample!.velocity_rad_s, -original.status.static_friction.last_stationary_sample!.velocity_rad_s);
  assert.equal(calibration.onset_sample!.velocity_rad_s, -original.status.static_friction.onset_sample!.velocity_rad_s);
  assert.equal(calibration.confirmation_sample!.velocity_rad_s, canonicalEvent.velocity_rad_s);
  assert.deepEqual(original, snapshot);
});

test("unverifiable event frames keep the entire run untouched even when curve frames are valid", () => {
  const original = data([sample(30000)]);
  original.status.static_friction = { phase: "Complete", outcome: "BreakawayDetected",
    reference_position_rad: 0, ramp_started_s: 0, return_started_s: 0.8, peak_excursion_rad: 0.01,
    last_stationary_sample: null, onset_sample: { ...sample(30000), payload: [] }, confirmation_sample: null };
  const result = normalizeExperimentFeedbackVelocity(original);
  assert.equal(result.changed, false);
  assert.equal(result.data, original);
});

test("empty recordings have no evidence for historical direction migration", () => {
  const original = data([]);
  assert.deepEqual(normalizeExperimentFeedbackVelocity(original), { data: original, changed: false });
});

test("bidirectional friction archives normalize both directions' event frames without mutating the source", () => {
  const original = data([sample(30000)]);
  const pass = { phase: "Complete" as const, outcome: "BreakawayDetected" as const,
    reference_position_rad: 0, ramp_started_s: 0.5, return_started_s: 0.8, return_settled: true,
    peak_excursion_rad: 0.01, last_stationary_sample: sample(32768), onset_sample: sample(35000), confirmation_sample: sample(36000) };
  original.feedback_velocity_sign = -1;
  original.status.static_friction = { ...structuredClone(pass), direction: -1, started_s: 2,
    completed_directions: [{ ...structuredClone(pass), direction: 1, started_s: 0 }] };
  const snapshot = structuredClone(original);
  const result = normalizeExperimentFeedbackVelocity(original);
  assert.equal(result.changed, true);
  for (const [i, corrected] of [...result.data.status.static_friction!.completed_directions!, result.data.status.static_friction!].entries()) {
    const before = i === 0 ? original.status.static_friction.completed_directions![0] : original.status.static_friction;
    assert.equal(corrected.onset_sample!.velocity_rad_s, -before.onset_sample!.velocity_rad_s);
    assert.equal(corrected.confirmation_sample!.velocity_rad_s, -before.confirmation_sample!.velocity_rad_s);
    assert.equal(corrected.direction, before.direction);
  }
  assert.deepEqual(original, snapshot);
  assert.equal(normalizeExperimentFeedbackVelocity(result.data).changed, false);
});
