import assert from "node:assert/strict";
import test from "node:test";
import type { El05ExperimentData, El05TelemetrySample } from "../src/el05Api.ts";
import { analyzeExperiment, experimentCsv, experimentJson } from "../src/el05Experiment.ts";
import { DEFAULT_TORQUE_ACCURACY, validateTorqueAccuracyRecording } from "../src/el05TorqueAccuracy.ts";

function recording(scale: (load: number, increasing: boolean) => number, direction: 1 | -1 = 1): El05ExperimentData {
  const peak = 3 * 0.18 * 9.80665, zero = 0.7;
  const loads = Array.from({ length: 31 }, (_, i) => 3.7 + i * 0.05);
  const schedule = [...loads, ...loads.slice(0, -1).reverse()];
  const holds = schedule.map((load, index) => ({ index, target_torque_nm: direction * load,
    target_angle_rad: direction * Math.asin(load / peak), increasing: index < loads.length,
    started_s: 2 + index * 1.4, ended_s: 3 + index * 1.4, completed: true }));
  const samples: El05TelemetrySample[] = holds.flatMap(hold => Array.from({ length: 100 }, (_, i) => {
    // Model real position droop. Fitting must use the actual angle/load, not
    // the requested position's ideal load, and must preserve signed feedback.
    const load = Math.abs(hold.target_torque_nm) - 0.07;
    const position = zero + direction * Math.asin(load / peak);
    return { t_s: hold.started_s + i / 100, sequence: hold.index * 100 + i + 1,
      can_id: 1, payload: [], position_rad: position, unwrapped_position_rad: position,
      velocity_rad_s: 50 / 4095, position_velocity_rad_s: 0,
      torque_nm: direction * load * scale(load, hold.increasing), current_a: null, current_age_ms: null,
      dictionary_velocity_rad_s: null, dictionary_velocity_age_ms: null, temperature_c: 45 + hold.index * 0.2,
      fault_bits: 0, status_code: 0, target_sequence: hold.index * 100 + i + 1, target_age_ms: 2,
      target: { kind: "Mit", position_rad: zero + hold.target_angle_rad, velocity_rad_s: 0, torque_nm: direction * load, kp: 80, kd: 2 } };
  }));
  return { protocol: "StandardMit", feedback_velocity_bits: 12, feedback_velocity_sign: 1,
    motor_id: 1, uid: null, parameters: [], version: null, commands: [], samples,
    status: { active: false, run_id: 456, config: { kind: "TorqueAccuracy", duration_s: 150,
      velocity_rad_s: 0.3, torque_nm: 0, kp: 80, kd: 2, max_temperature_c: 120, gravity_compensation: true,
      torque_accuracy: { ...DEFAULT_TORQUE_ACCURACY, reference_id: "captured-zero", direction },
      pendulum: { mass_kg: 3, length_m: 0.18, arm_mass_kg: 0, downward_zero_rad: zero, zero_measured: true } },
      samples: samples.length, elapsed_s: 100, reason: "scan completed; returned to calibrated zero", dropped_frames: 0,
      torque_accuracy: { phase: "Complete", index: holds.length - 1, point_count: holds.length, holds,
        return_started_s: 88, return_reason: "scan completed", returned_to_zero: true } } };
}

test("accuracy finds a step interval from actual-angle load and keeps send scale in the correct direction", () => {
  for (const direction of [1, -1] as const) {
    const data = recording(load => load < 4.2 ? 1 : 1.13, direction);
    validateTorqueAccuracyRecording(data);
    const result = analyzeExperiment(data).torque_accuracy!;
    assert.equal(result.points.filter(p => p.accepted).length, 61);
    assert.equal(result.paired_points, 30);
    assert.equal(result.model, "step");
    assert.ok(result.boundary_nm![0] < 4.2 && result.boundary_nm![1] > 4.2);
    assert.ok(Math.abs(result.high_scale! - 1.13) < 1e-10);
    assert.ok(Math.abs(result.low_scale! - 1) < 1e-10);
    assert.ok(Math.abs(result.points[0].theory_nm! - direction * 3.63) < 1e-10);
    assert.equal(result.points[0].target_torque_nm, direction * 3.7);
    assert.ok(result.boundary_feedback_nm![1] > 4.7);
  }
});

test("accuracy distinguishes constant, smooth linear, and boundary-then-linear scales", () => {
  const constant = analyzeExperiment(recording(() => 1.13)).torque_accuracy!;
  assert.equal(constant.model, "constant");
  assert.equal(constant.boundary_nm, null);
  const linear = analyzeExperiment(recording(load => 1 + (load - 3.6) * 0.08)).torque_accuracy!;
  assert.equal(linear.model, "linear");
  assert.equal(linear.boundary_nm, null);
  assert.ok(Math.abs(linear.slope_per_nm! - 0.08) < 1e-10);
  const hinged = analyzeExperiment(recording(load => 1 + Math.max(0, load - 4.2) * 0.12)).torque_accuracy!;
  assert.equal(hinged.model, "piecewise_linear");
  assert.ok(hinged.boundary_nm![0] < 4.2 && hinged.boundary_nm![1] > 4.2);
});

test("motion, stale commands, faults, missing derivative, saturation and truncated holds cannot calibrate torque", () => {
  for (const change of [{ velocity_rad_s: 0.2 }, { target_age_ms: 300 }, { fault_bits: 1 },
    { position_velocity_rad_s: null }, { torque_nm: 6 }]) {
    const data = recording(() => 1.13);
    data.samples = data.samples.map(s => ({ ...s, ...change }));
    const result = analyzeExperiment(data).torque_accuracy!;
    assert.equal(result.points.filter(p => p.accepted).length, 0, JSON.stringify(change));
    assert.equal(result.model, "inconclusive");
    assert.equal(result.high_scale, null);
  }
  const data = recording(() => 1);
  data.status.torque_accuracy!.holds.forEach(h => { h.completed = false; });
  assert.equal(analyzeExperiment(data).torque_accuracy!.points.filter(p => p.accepted).length, 0);
});

test("large approach hysteresis and incomplete return sweeps cannot claim a boundary", () => {
  const hysteresis = analyzeExperiment(recording((load, up) => (load < 4.2 ? 1 : 1.13) + (up ? 0.03 : -0.03))).torque_accuracy!;
  assert.equal(hysteresis.model, "inconclusive");
  assert.ok(hysteresis.hysteresis_scale! > 0.05);
  assert.match(hysteresis.reason!, /6\.00 个百分点/);
  assert.match(hysteresis.reason!, /不能确定力矩误差边界或发送补偿系数/);
  assert.equal(hysteresis.low_scale, null);
  assert.equal(hysteresis.high_scale, null);
  const data = recording(load => load < 4.2 ? 1 : 1.13);
  data.status.torque_accuracy!.holds = data.status.torque_accuracy!.holds.slice(0, 31);
  assert.equal(analyzeExperiment(data).torque_accuracy!.paired_points, 0);
  assert.equal(analyzeExperiment(data).torque_accuracy!.boundary_nm, null);
});

test("accuracy exports preserve hold timing, raw torque, theory and analysis, excluding return samples from fits", () => {
  const data = recording(load => load < 4.2 ? 1 : 1.13);
  const first = data.samples[0];
  data.samples.push({ ...first, sequence: 99999, t_s: 99, torque_nm: 6 });
  const json = JSON.parse(experimentJson(data));
  validateTorqueAccuracyRecording(json);
  assert.equal(json.analysis.torque_accuracy.model, "step");
  assert.equal(json.status.torque_accuracy.holds.length, 61);
  assert.equal(json.samples.at(-1).torque_nm, 6);
  const [header, row] = experimentCsv(data).trim().split("\r\n").map(line => line.split(","));
  const values = Object.fromEntries(header.map((name, i) => [name, row[i]]));
  assert.equal(values.accuracy_hold_index, "0");
  assert.equal(values.accuracy_target_torque_nm, "3.7");
  assert.equal(values.accuracy_hold_completed, "true");
  assert.ok(Math.abs(Number(values.target_torque_nm) - 3.63) < 1e-10, "gravity feedforward remains in exported MIT targets");
  assert.ok(Math.abs(Number(values.gravity_torque_nm) - 3.63) < 1e-10);
});

test("gravity feedforward preserves total-torque scale estimates and legacy zero-feedforward records remain valid", () => {
  const data = recording(load => load < 4.2 ? 1 : 1.13);
  validateTorqueAccuracyRecording(data);
  const withGravity = analyzeExperiment(data).torque_accuracy!;
  const legacy = structuredClone(data);
  legacy.status.config!.gravity_compensation = false;
  legacy.status.config!.max_temperature_c = 100;
  for (const sample of legacy.samples) if (sample.target?.kind === "Mit") sample.target.torque_nm = 0;
  validateTorqueAccuracyRecording(legacy);
  assert.deepEqual(analyzeExperiment(legacy).torque_accuracy, withGravity, "feedforward must not be subtracted from total holding feedback");
  data.status.config!.max_temperature_c = 121;
  assert.throws(() => validateTorqueAccuracyRecording(data));
  data.status.config!.max_temperature_c = 120;
  for (const torque of [NaN, 6.01, -6.01]) {
    for (const sample of data.samples) if (sample.target?.kind === "Mit") sample.target.torque_nm = torque;
    assert.equal(analyzeExperiment(data).torque_accuracy!.points.filter(p => p.accepted).length, 0);
  }
});

test("import rejects invalid scan metadata and duplicate feedback does not inflate stable samples", () => {
  for (const change of [ { hold_s: 0 }, { step_nm: 0 }, { end_nm: 6 }, { direction: 0 }, { reference_id: "" } ]) {
    const data = recording(() => 1);
    Object.assign(data.status.config!.torque_accuracy!, change);
    assert.throws(() => validateTorqueAccuracyRecording(data));
  }
  const invalid = recording(() => 1);
  invalid.status.torque_accuracy!.holds[1].started_s = 0;
  assert.throws(() => validateTorqueAccuracyRecording(invalid));
  const data = recording(() => 1);
  const before = analyzeExperiment(data).torque_accuracy!;
  data.samples = [...data.samples, ...data.samples].reverse();
  const after = analyzeExperiment(data).torque_accuracy!;
  assert.deepEqual(after.points.map(p => p.sample_count), before.points.map(p => p.sample_count));
  assert.equal(after.model, before.model);
});
