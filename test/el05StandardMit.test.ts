import assert from "node:assert/strict";
import test from "node:test";
import type { El05ExperimentData, El05ExperimentKind, El05TelemetrySample } from "../src/el05Api.ts";
import { analyzeExperiment, experimentCsv, experimentJson, observedCommandRateHz } from "../src/el05Experiment.ts";
import { controlFeedbackEligible, feedbackVelocityBits, feedbackVelocityResolution } from "../src/el05Protocol.ts";
import { normalizeExperimentFeedbackVelocity } from "../src/el05Velocity.ts";

function sample(t: number, index: number): El05TelemetrySample {
  return {
    t_s: t, sequence: index, can_id: 1, payload: [1, 0x62, 0x21, 0x80, 0x08, 0x00, 0x01, 0x33],
    position_rad: -2.9332877088578613, unwrapped_position_rad: -2.9332877088578613,
    velocity_rad_s: 2048 * 100 / 4095 - 50, position_velocity_rad_s: 0,
    torque_nm: 2048 * 12 / 4095 - 6, current_a: null, current_age_ms: null,
    dictionary_velocity_rad_s: null, dictionary_velocity_age_ms: null,
    temperature_c: 30.7, fault_bits: 0, status_code: 0,
    target: { kind: "Mit", position_rad: -2.8, velocity_rad_s: 0.3, torque_nm: 0.2, kp: 3, kd: 0.5 },
    target_sequence: index + 1, target_age_ms: 2,
  };
}

function run(kind: El05ExperimentKind = "VelocitySteps"): El05ExperimentData {
  const samples = Array.from({ length: 101 }, (_, i) => sample(i / 50, i));
  return {
    protocol: "StandardMit", feedback_velocity_bits: 12, feedback_velocity_sign: 1,
    motor_id: 1, uid: null, parameters: [], version: null, commands: [], samples,
    status: {
      active: false, config: { kind, duration_s: 2, velocity_rad_s: 0.3, torque_nm: 0.3,
        kp: 3, kd: 0.5, max_temperature_c: 60 },
      samples: samples.length, elapsed_s: 2, reason: "duration completed", dropped_frames: 0,
    },
  };
}

test("standard MIT retains raw zero state and uses fresh control evidence for every bench kind", () => {
  const kinds: El05ExperimentKind[] = ["Record", "VelocitySteps", "TorqueRamp", "StaticTorque", "PositionSteps",
    "SineSweep", "Impedance", "Backdrive", "ThermalHold", "SlowTriangle", "StaticFriction"];
  for (const kind of kinds) {
    const data = run(kind), before = structuredClone(data), report = analyzeExperiment(data);
    assert.equal(report.quality.inactive_drive_samples, null, kind);
    assert.equal(report.quality.inactive_drive_fraction, null, kind);
    assert.equal(report.sample_count, data.samples.length, kind);
    assert.ok(report.tracking.position_rms_rad! > 0, `${kind} retains commanded motion feedback with status 0`);
    assert.equal(report.quality.fresh_current_samples, 0, kind);
    assert.equal(report.static_torque.torque_constant_nm_per_a, null, "missing Iq is never reconstructed from torque");
    assert.deepEqual(data, before, "analysis must preserve raw feedback and its zero state");
  }
});

test("standard MIT cannot turn stale targets, missing commands or fault feedback into performance evidence", () => {
  for (const change of [{ target_age_ms: 251 }, { target_age_ms: null }, { target_sequence: 0 }, { target: null }, { fault_bits: 1 }]) {
    const data = run();
    data.samples = data.samples.map(s => ({ ...s, ...change }));
    assert.equal(controlFeedbackEligible(data.samples[0], data), false, JSON.stringify(change));
    assert.equal(analyzeExperiment(data).tracking.position_rms_rad, null, JSON.stringify(change));
  }
  const native = run();
  delete native.protocol;
  delete native.feedback_velocity_bits;
  const report = analyzeExperiment(native);
  assert.equal(report.quality.inactive_drive_fraction, 1);
  assert.equal(report.tracking.position_rms_rad, null, "legacy native recordings still require drive state 2");
});

test("standard MIT static-friction events accept status zero only with fresh fault-free command evidence", () => {
  const data = run("StaticFriction"), onset = sample(1, 50), confirmation = sample(1.04, 52);
  onset.unwrapped_position_rad += 0.003;
  confirmation.unwrapped_position_rad += 0.004;
  data.status.static_friction = {
    phase: "Complete", outcome: "BreakawayDetected", direction: 1,
    reference_position_rad: sample(0, 0).position_rad, ramp_started_s: 0.5, return_started_s: 1.1,
    last_stationary_sample: sample(0.98, 49), onset_sample: onset, confirmation_sample: confirmation,
    peak_excursion_rad: 0.004,
  };
  assert.equal(analyzeExperiment(data).static_friction!.valid_breakaway, true);
  confirmation.target_age_ms = 300;
  assert.equal(analyzeExperiment(data).static_friction!.valid_breakaway, false);
  confirmation.target_age_ms = 2;
  confirmation.fault_bits = 1;
  assert.equal(analyzeExperiment(data).static_friction!.valid_breakaway, false);
});

test("standard 12-bit wire evidence uses its own packed velocity bytes during import", () => {
  const data = run();
  delete data.feedback_velocity_sign;
  data.samples[0].velocity_rad_s *= -1;
  const result = normalizeExperimentFeedbackVelocity(data);
  assert.equal(result.changed, true);
  assert.equal(result.data.samples[0].velocity_rad_s, 2048 * 100 / 4095 - 50);
  assert.equal(result.data.samples[0].status_code, 0);
  assert.deepEqual(result.data.samples[0].payload, data.samples[0].payload);
  assert.equal(normalizeExperimentFeedbackVelocity(result.data).changed, false);
});

test("protocol and quantization survive JSON and CSV exports; old records keep native resolution", () => {
  const data = run(), exported = JSON.parse(experimentJson(data));
  assert.equal(exported.protocol, "StandardMit");
  assert.equal(exported.feedback_velocity_bits, 12);
  assert.equal(exported.feedback_velocity_resolution_rad_s, 100 / 4095);
  assert.equal(exported.samples[0].status_code, 0);
  const [header, first] = experimentCsv(data).trim().split("\r\n").map(row => row.split(","));
  const row = Object.fromEntries(header.map((key, i) => [key, first[i]]));
  assert.equal(row.protocol, "StandardMit");
  assert.equal(row.feedback_velocity_bits, "12");
  assert.equal(row.status_code, "0");
  assert.equal(row.current_a, "");
  assert.equal(row.dictionary_velocity_rad_s, "");
  assert.equal(feedbackVelocityBits({}), 16);
  assert.equal(feedbackVelocityResolution({}), 100 / 65535);
  assert.equal(feedbackVelocityBits({ protocol: "StandardMit" }), 12);
});

test("command cadence follows send evidence and cannot be inferred from protocol or faster feedback", () => {
  const data = run();
  data.samples = Array.from({ length: 501 }, (_, i) => ({ ...sample(i * 0.002, i),
    target_sequence: Math.floor(i / 10) + 1, target_age_ms: (i % 10) * 2 }));
  assert.ok(Math.abs(observedCommandRateHz(data)! - 50) < 1e-8, "500Hz replies can contain only 50Hz commands");
  data.commands = Array.from({ length: 51 }, (_, i) => ({ t_s: i * 0.02, completed_t_s: i * 0.02 + 0.0001,
    sequence: i * 10 + 1, target: sample(0, 0).target!, can_id: 1, payload: [] }));
  assert.ok(Math.abs(observedCommandRateHz(data)! - 500) < 1e-8, "thinned commands preserve count and completion-time evidence");
  const quality = analyzeExperiment(data).quality;
  assert.ok(Math.abs(quality.command_rate_hz! - 500) < 1e-8);
  data.commands = [];
  data.samples = data.samples.map(s => ({ ...s, target_sequence: 0 }));
  assert.equal(observedCommandRateHz(data), null, "protocol does not establish a 50Hz or 500Hz rate without evidence");
});

test("frequency analysis admits higher-rate control while old 50Hz MIT recordings retain their measured limit", () => {
  const data = run("SineSweep"), duration = 30, dt = 0.002, f0 = 1, f1 = 10;
  data.status.config = { ...data.status.config!, duration_s: duration, frequency_start_hz: f0, frequency_end_hz: f1 };
  data.samples = Array.from({ length: Math.round(duration / dt) + 1 }, (_, i) => {
    const t = i * dt, phase = 2 * Math.PI * (f0 * t + 0.5 * (f1 - f0) / duration * t * t);
    const position = 0.1 * Math.sin(phase);
    return { ...sample(t, i), position_rad: position, unwrapped_position_rad: position,
      target_age_ms: 0, target: { kind: "Mit" as const, position_rad: position, velocity_rad_s: 0, torque_nm: 0, kp: 3, kd: 0.5 } };
  });
  data.status.samples = data.samples.length;
  const fast = analyzeExperiment(data);
  assert.ok(fast.frequency_response.points.some(point => point.frequency_hz > 5), "500Hz commands are not capped to old 50Hz update rate");
  assert.ok(Math.abs(fast.quality.command_rate_hz! - 500) < 1e-6);
  data.samples = data.samples.map((s, i) => ({ ...s, target_sequence: Math.floor(i / 10) + 1, target_age_ms: (i % 10) * 2 }));
  const slow = analyzeExperiment(data);
  assert.ok(Math.abs(slow.quality.command_rate_hz! - 50) < 1e-6);
  assert.ok(slow.frequency_response.points.length > 0);
  assert.ok(slow.frequency_response.points.every(point => point.frequency_hz <= 5.1), "historical standard MIT rate is measured, not upgraded by protocol name");
});
