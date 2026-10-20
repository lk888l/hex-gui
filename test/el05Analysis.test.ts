import assert from "node:assert/strict";
import test from "node:test";
import { analyzeExperiment, deriveExperimentSamples, experimentCsv, experimentJson, experimentTorqueLimitNm, pendulumModel, staticHoldingTorquePeakNm } from "../src/el05Experiment.ts";
import { normalizeExperimentFeedbackVelocity } from "../src/el05Velocity.ts";
import type { El05ExperimentConfig, El05ExperimentData, El05TelemetrySample } from "../src/el05Api.ts";

const geometry = { mass_kg: 1, length_m: 0.18, arm_mass_kg: 0.1, extra_inertia_kg_m2: 0.004,
  downward_zero_rad: 0, zero_measured: true };
const model = pendulumModel(geometry.mass_kg, geometry.length_m, geometry.arm_mass_kg, geometry.extra_inertia_kg_m2);
function config(kind: El05ExperimentConfig["kind"], duration_s: number): El05ExperimentConfig {
  return { kind, duration_s, velocity_rad_s: 0.3, torque_nm: 0.3, kp: 8, kd: 0.4, max_temperature_c: 70,
    origin_position_rad: 0, pendulum: { ...geometry } };
}
function data(cfg: El05ExperimentConfig, samples: El05TelemetrySample[]): El05ExperimentData {
  return { status: { active: false, config: cfg, samples: samples.length, elapsed_s: samples.at(-1)?.t_s ?? 0,
    reason: "duration completed", dropped_frames: 0 }, motor_id: 1, uid: "test", samples, commands: [], parameters: [], version: null };
}
function sample(t: number, sequence: number, position: number, velocity: number, torque = 0): El05TelemetrySample {
  return { t_s: t, sequence, can_id: 0x020001fd, payload: [0, 0, 0, 0, 0, 0, 0, 0], position_rad: position,
    unwrapped_position_rad: position, velocity_rad_s: velocity, position_velocity_rad_s: velocity,
    torque_nm: torque, current_a: null, current_age_ms: null, dictionary_velocity_rad_s: null,
    dictionary_velocity_age_ms: null, temperature_c: 25, fault_bits: 0, status_code: 2, target_sequence: sequence,
    target_age_ms: 0, target: null };
}
function near(actual: number | null, expected: number, tolerance: number, description = "") {
  assert.notEqual(actual, null, description);
  assert.ok(Math.abs(actual! - expected) <= tolerance, `${description}: ${actual} expected ${expected} ± ${tolerance}`);
}

test("derived samples preserve protocol coordinates, suppress cached Iq and include additional inertia", () => {
  const samples = Array.from({ length: 101 }, (_, i) => {
    const t = i / 100, p = sample(t, i, 25.14 + t * t, 2 * t, 0.5);
    p.position_velocity_rad_s = 2 * Math.max(0, t - 0.075); // Delayed position-difference channel.
    p.position_rad -= 25.14;
    p.current_a = 0.5 + Math.floor(i / 20) * 0.1;
    p.current_age_ms = i % 20 * 10;
    p.target = { kind: "Mit", position_rad: p.position_rad + 0.2, velocity_rad_s: 2 * t, torque_nm: 0, kp: 8, kd: 0.4 };
    return p;
  });
  const run = data(config("PositionSteps", 1), samples), derived = deriveExperimentSamples(run);
  near(derived[50].target_position_rad, samples[50].unwrapped_position_rad + 0.2, 1e-10);
  near(derived[50].position_error_rad, 0.2, 1e-10);
  near(derived[50].velocity_rad_s, 1, 1e-10, "native synchronous velocity takes priority");
  near(derived[50].acceleration_rad_s2, 2, 1e-8);
  near(derived[50].inertial_torque_nm, 2 * model.inertiaKgM2, 1e-8);
  assert.equal(derived.filter(p => p.fresh_current_a !== null).length, 6);
  assert.equal(derived[0].acceleration_rad_s2, null, "one-sided derivative is excluded");
  run.status.config!.pendulum!.zero_measured = false;
  const noZero = deriveExperimentSamples(run);
  assert.equal(noZero[50].angle_rad, null);
  assert.equal(noZero[50].gravity_torque_nm, null);
  assert.equal(noZero[50].friction_torque_nm, null);
});

test("friction regression recovers a known bidirectional mechanical model and refuses single-speed excitation", () => {
  const fc = 0.06, b = 0.035, omega = 1.2, amplitude = 0.5;
  const samples = Array.from({ length: 2001 }, (_, i) => {
    const t = i / 100, theta = amplitude * Math.sin(omega * t), v = amplitude * omega * Math.cos(omega * t);
    const alpha = -amplitude * omega * omega * Math.sin(omega * t);
    return sample(t, i, theta, v, model.gravityNm * Math.sin(theta) + model.inertiaKgM2 * alpha + fc * Math.sign(v) + b * v);
  });
  const run = data(config("Record", 20), samples), report = analyzeExperiment(run);
  near(report.friction.coulomb_nm, fc, 0.0001);
  near(report.friction.viscous_nm_s_rad, b, 0.0001);
  assert.ok(report.friction.r_squared! > 0.9999);
  const constant = samples.map((s, i) => ({ ...s, position_velocity_rad_s: 0.2, velocity_rad_s: 0.2 }));
  assert.equal(analyzeExperiment(data(config("Record", 20), constant)).friction.coulomb_nm, null);
  run.status.config!.pendulum!.zero_measured = false;
  assert.equal(analyzeExperiment(run).friction.viscous_nm_s_rad, null);
});

test("correcting packed feedback polarity restores acceleration and physical friction instead of taking an absolute coefficient", () => {
  const fc = 0.06, b = 0.035, decay = 0.15, omega = 1.6, amplitude = 0.6;
  const motion = (t: number) => {
    const envelope = amplitude * Math.exp(-decay * t);
    return {
      theta: envelope * Math.sin(omega * t),
      velocity: envelope * (omega * Math.cos(omega * t) - decay * Math.sin(omega * t)),
      acceleration: envelope * ((decay ** 2 - omega ** 2) * Math.sin(omega * t) - 2 * decay * omega * Math.cos(omega * t)),
    };
  };
  const samples = Array.from({ length: 2401 }, (_, i) => {
    const t = i / 200, m = motion(t);
    // The previous decoder inverted the direct packed feedback velocity.
    const packedVelocity = Math.round((50 + m.velocity) * 65535 / 100);
    const wireVelocity = packedVelocity * 100 / 65535 - 50;
    const p = sample(t, i, m.theta, -wireVelocity, model.gravityNm * Math.sin(m.theta)
      + model.inertiaKgM2 * m.acceleration + fc * Math.sign(m.velocity) + b * m.velocity);
    p.can_id = i % 2 ? 0x180001fd : 0x020001fd;
    p.payload[2] = packedVelocity >> 8;
    p.payload[3] = packedVelocity & 0xff;
    p.position_velocity_rad_s = m.velocity;
    p.dictionary_velocity_rad_s = m.velocity;
    return p;
  });
  const legacy = data(config("Record", 12), samples);
  legacy.feedback_velocity_sign = -1;
  const normalized = normalizeExperimentFeedbackVelocity(legacy);
  assert.equal(normalized.changed, true);
  const derived = deriveExperimentSamples(normalized.data);
  for (const i of [200, 600]) { // Both directions and both acceleration signs.
    const m = motion(samples[i].t_s);
    near(derived[i].velocity_rad_s, m.velocity, 100 / 65535);
    near(derived[i].acceleration_rad_s2, m.acceleration, 0.015);
    near(derived[i].inertial_torque_nm, model.inertiaKgM2 * m.acceleration, 0.0006);
  }
  const corrected = analyzeExperiment(normalized.data).friction;
  near(corrected.coulomb_nm, fc, 0.0002);
  near(corrected.viscous_nm_s_rad, b, 0.0002);
  assert.ok(corrected.r_squared! > 0.9999);
  // Nonconstant amplitude couples the wrong acceleration into the fit: merely
  // forcing a negative B positive cannot recover the original damping value.
  const wrong = analyzeExperiment(legacy).friction;
  assert.ok(wrong.coulomb_nm! < 0);
  assert.ok(Math.abs(Math.abs(wrong.viscous_nm_s_rad!) - b) > 0.015);
  assert.deepEqual(normalizeExperimentFeedbackVelocity(normalized.data), { data: normalized.data, changed: false });
});

test("static plateau regression uses independent fresh Iq and known gravity, not feedback torque", () => {
  const levels = [0, 0.15, 0.3, 0.15, 0, -0.15, -0.3, -0.15, 0, 0.3, 0.15, 0];
  const effectiveKt = 0.82, offset = 0.015;
  const samples = Array.from({ length: 2400 }, (_, i) => {
    const t = i / 100, plateau = Math.floor(t / 2), theta = levels[plateau];
    const p = sample(t, i, theta, 0, 3.7); // Deliberately unrelated drive estimate.
    p.target = { kind: "Mit", position_rad: theta, velocity_rad_s: 0, torque_nm: 0, kp: 8, kd: 0.4 };
    p.current_a = (model.gravityNm * Math.sin(theta) - offset) / effectiveKt;
    p.current_age_ms = i % 20 * 10;
    return p;
  });
  const report = analyzeExperiment(data(config("StaticTorque", 24), samples));
  near(report.static_torque.torque_constant_nm_per_a, effectiveKt, 1e-10);
  near(report.static_torque.intercept_nm, offset, 1e-10);
  assert.ok(report.static_torque.platforms.length >= 10);
  assert.ok(report.quality.fresh_current_samples <= 120);
  assert.ok(report.static_torque.platforms.every(p => p.fresh_current_samples <= 5));
  // Holding a cached value with unknown age must never fabricate 2400 current observations.
  samples.forEach(p => { p.current_age_ms = null; });
  assert.equal(analyzeExperiment(data(config("StaticTorque", 24), samples)).static_torque.torque_constant_nm_per_a, null);
});

test("target holding torque follows sent static angles, frozen geometry and the unwrapped downward reference", () => {
  const angles = [0, 0.3, 0.6, 0.3, 0, -0.3, -0.6, -0.3, 0, 0.6, 0.3, 0];
  const cfg = config("StaticTorque", 72);
  cfg.pendulum = { ...geometry, mass_kg: 1, length_m: 0.18, arm_mass_kg: 0, downward_zero_rad: 30 };
  const samples = angles.map((angle, i) => {
    const p = sample(i * 6, i + 1, 30 + angle + 0.07, 0);
    p.position_rad = (p.unwrapped_position_rad + 12.57) % 25.14 - 12.57;
    p.target = { kind: "Mit", position_rad: p.position_rad - 0.07, velocity_rad_s: 0, torque_nm: 0, kp: 8, kd: 0.5 };
    return p;
  });
  const run = data(cfg, samples), derived = deriveExperimentSamples(run);
  for (const [i, angle] of angles.entries()) {
    near(derived[i].target_holding_torque_nm, 1.765197 * Math.sin(angle), 1e-12);
    near(derived[i].gravity_torque_nm, 1.765197 * Math.sin(angle + 0.07), 1e-12);
  }
  near(derived[1].target_holding_torque_nm, 0.5216513822, 1e-9);
  near(derived[2].target_holding_torque_nm, 0.9967052001095, 1e-12);
  const csv = experimentCsv(run);
  assert.ok(csv.split("\r\n")[0].split(",").includes("target_holding_torque_nm"));
  cfg.pendulum.arm_mass_kg = 0.2;
  near(deriveExperimentSamples(run)[1].target_holding_torque_nm, 1.9417167 * Math.sin(0.3), 1e-12);
  samples[1].target_age_ms = 251;
  assert.equal(deriveExperimentSamples(run)[1].target_holding_torque_nm, null);
  samples[1].target_age_ms = 0;
  cfg.pendulum.zero_measured = false;
  assert.ok(deriveExperimentSamples(run).every(p => p.target_holding_torque_nm === null));
  cfg.pendulum = null;
  assert.ok(deriveExperimentSamples(run).every(p => p.target_holding_torque_nm === null));
});

test("static holding torque and drive feedback retain the same signed Nm unit in CSV and JSON", () => {
  const cfg = config("StaticTorque", 12);
  cfg.pendulum = { ...geometry, arm_mass_kg: 0 };
  const samples = [-1, 1].map((direction, i) => {
    // A horizontal 1 kg load at 180 mm requires ±1.765197 Nm.
    // Native feedback words 0x5555/0xAAAA represent exactly -2/+2 Nm;
    // the estimate need not equal the ideal gravity-only load.
    const p = sample(i * 6, i + 1, direction * Math.PI / 2, 0, direction * 2);
    p.payload[4] = p.payload[5] = direction < 0 ? 0x55 : 0xaa;
    p.target = { kind: "Mit", position_rad: p.position_rad, velocity_rad_s: 0, torque_nm: 0, kp: 8, kd: 0.5 };
    return p;
  });
  const run = data(cfg, samples), exported = JSON.parse(experimentJson(run));
  assert.equal(exported.torque_unit, "Nm");
  assert.deepEqual(exported.torque_feedback_range_nm, [-6, 6]);
  const [headers, ...rows] = experimentCsv(run).trim().split("\r\n").map(line => line.split(","));
  for (const [i, direction] of [-1, 1].entries()) {
    near(exported.samples[i].torque_nm, direction * 2, 1e-12);
    near(exported.derived_samples[i].target_holding_torque_nm, direction * 1.765197, 1e-12);
    near(exported.derived_samples[i].gravity_torque_nm, direction * 1.765197, 1e-12);
    near(Number(rows[i][headers.indexOf("torque_nm")]), direction * 2, 1e-12);
    near(Number(rows[i][headers.indexOf("target_holding_torque_nm")]), direction * 1.765197, 1e-12);
    near(Number(rows[i][headers.indexOf("gravity_torque_nm")]), direction * 1.765197, 1e-12);
  }
});

test("9.96 kg at 180 mm is reported as over the EL05 torque range without rescaling theory", () => {
  const cfg = config("StaticTorque", 72);
  cfg.position_amplitude_rad = 0.6;
  cfg.pendulum = { ...geometry, mass_kg: 9.96, length_m: 0.18, arm_mass_kg: 0.06 };
  const p = sample(6, 1, 0.6, 0, 6);
  p.target = { kind: "Mit", position_rad: 0.6, velocity_rad_s: 0, torque_nm: 0, kp: 12, kd: 0.5 };
  const run = data(cfg, [p]), derived = deriveExperimentSamples(run);
  near(staticHoldingTorquePeakNm(cfg), 9.957084949093867, 1e-12);
  near(derived[0].target_holding_torque_nm, 9.957084949093867, 1e-12);
  near(derived[0].gravity_torque_nm, 9.957084949093867, 1e-12);
  assert.equal(experimentTorqueLimitNm(run), 6);
  const section = analyzeExperiment(run).sections.find(section => section.id === "static")!;
  near(section.metrics.find(metric => metric.key === "target_holding_peak")!.value, 9.957084949093867, 1e-12);
  assert.ok(section.notes.some(note => note.includes("9.957 Nm") && note.includes("6.000 Nm")));
  // A smaller captured limit is a capacity setting, not a different torque unit.
  run.parameters = [{ index: 0x700b, value: 2, kind: "Float" }];
  assert.equal(experimentTorqueLimitNm(run), 2);
  assert.ok(analyzeExperiment(run).sections.find(section => section.id === "static")!.notes.some(note => note.includes("2.000 Nm")));
  const exported = JSON.parse(experimentJson(run));
  near(exported.derived_samples[0].target_holding_torque_nm, 9.957084949093867, 1e-12);
  assert.equal(exported.samples[0].torque_nm, 6);
  cfg.pendulum.zero_measured = false;
  assert.equal(staticHoldingTorquePeakNm(cfg), null);
});

test("free decay identifies period, logarithmic decrement and total damping of a known damped oscillator", () => {
  const decay = 0.09, omega = 4, amplitude = 0.2, offset = 1.3;
  const samples = Array.from({ length: 3001 }, (_, i) => {
    const t = i / 100, e = Math.exp(-decay * t);
    return sample(t, i, offset + amplitude * e * Math.cos(omega * t), amplitude * e * (-decay * Math.cos(omega * t) - omega * Math.sin(omega * t)));
  });
  const cfg = config("Record", 30);
  cfg.pendulum!.zero_measured = false; // Peak-to-valley envelope is offset independent.
  const report = analyzeExperiment(data(cfg, samples));
  near(report.free_decay.period_s, 2 * Math.PI / omega, 0.015);
  near(report.free_decay.log_decrement, decay * 2 * Math.PI / omega, 0.003);
  near(report.free_decay.damping_ratio, decay / Math.hypot(decay, omega), 0.0005);
  near(report.free_decay.equivalent_viscous_nm_s_rad, 2 * decay * model.inertiaKgM2, 0.0001);
  assert.ok(report.free_decay.envelope_r_squared! > 0.999);
  const interrupted = analyzeExperiment(data(cfg, samples.slice(0, 160)));
  assert.equal(interrupted.free_decay.period_s, null);
});

test("step response measures a first-order plant and incomplete response remains unsettled", () => {
  const tau = 0.2, dt = 0.005;
  const samples = Array.from({ length: 1201 }, (_, i) => {
    const t = i * dt, input = t < 1 ? 0 : 0.4;
    const output = t < 1 ? 0 : input * (1 - Math.exp(-(t - 1) / tau));
    const velocity = t < 1 ? 0 : input / tau * Math.exp(-(t - 1) / tau);
    const p = sample(t, i, output, velocity);
    p.target = { kind: "Mit", position_rad: input, velocity_rad_s: 0, torque_nm: 0, kp: 8, kd: 0.4 };
    return p;
  });
  const run = data(config("PositionSteps", 6), samples), report = analyzeExperiment(run);
  assert.equal(report.tracking.position_steps.length, 1);
  near(report.tracking.position_steps[0].rise_time_s, tau * Math.log(9), 0.006);
  near(report.tracking.position_steps[0].settling_time_s, -tau * Math.log(0.02), 0.01);
  near(report.tracking.position_steps[0].overshoot_percent, 0, 1e-9);
  const partial = analyzeExperiment(data(config("PositionSteps", 6), samples.slice(0, 241)));
  assert.equal(partial.tracking.position_steps[0].settling_time_s, null);
  assert.equal(partial.tracking.position_steps[0].rise_time_s, null);
  assert.ok(partial.quality.notes.some(n => n.includes("未覆盖计划全程")));
  run.samples.forEach(p => { p.status_code = 0; });
  const stopped = analyzeExperiment(run);
  assert.equal(stopped.tracking.position_rms_rad, null);
  assert.equal(stopped.quality.inactive_drive_fraction, 1);
});

test("chirp analysis recovers first-order gain/phase and reports bandwidth only with measured coverage", () => {
  const duration = 600, dt = 0.02, f0 = 0.2, f1 = 2, tau = 0.2, amplitude = 0.2;
  const cfg = { ...config("SineSweep", duration), frequency_start_hz: f0, frequency_end_hz: f1 };
  let output = 0;
  const samples = Array.from({ length: Math.round(duration / dt) + 1 }, (_, i) => {
    const t = i * dt, phase = 2 * Math.PI * (f0 * t + 0.5 * (f1 - f0) / duration * t * t);
    const input = amplitude * Math.sin(phase), velocity = (input - output) / tau;
    const p = sample(t, i, output, velocity);
    p.target = { kind: "Mit", position_rad: input, velocity_rad_s: 0, torque_nm: 0, kp: 8, kd: 0.4 };
    output += dt * velocity;
    return p;
  });
  const report = analyzeExperiment(data(cfg, samples));
  assert.ok(report.frequency_response.points.length >= 8);
  for (const p of report.frequency_response.points) {
    near(p.gain, 1 / Math.hypot(1, 2 * Math.PI * p.frequency_hz * tau), 0.035, `gain at ${p.frequency_hz} Hz`);
    near(p.phase_deg, -Math.atan(2 * Math.PI * p.frequency_hz * tau) * 180 / Math.PI, 6, `phase at ${p.frequency_hz} Hz`);
  }
  near(report.frequency_response.bandwidth_hz, 1 / (2 * Math.PI * tau), 0.11);
  const shortRun = analyzeExperiment(data(cfg, samples.slice(0, 300)));
  assert.equal(shortRun.frequency_response.bandwidth_hz, null);
  assert.equal(shortRun.frequency_response.points.length, 0);
});

test("injection response never treats internal feedforward torque as an external stiffness measurement", () => {
  const samples = Array.from({ length: 1001 }, (_, i) => {
    const t = i / 100, active = t >= 3 && t < 3.5;
    const theta = t < 3 ? 0 : t < 3.5 ? 0.03 * (1 - Math.exp(-(t - 3) / 0.08)) : 0.03 * Math.exp(-(t - 3.5) / 0.1);
    const p = sample(t, i, theta, 0);
    p.target = { kind: "Mit", position_rad: 0, velocity_rad_s: 0, torque_nm: active ? 0.15 : 0, kp: 8, kd: 0.4 };
    return p;
  });
  const cfg = config("Impedance", 10), report = analyzeExperiment(data(cfg, samples));
  assert.equal(report.impedance.responses.length, 1);
  near(report.impedance.responses[0].peak_displacement_rad, 0.03, 0.001);
  assert.ok(report.impedance.responses[0].recovery_s! >= 0.17);
  assert.equal(report.impedance.equivalent_stiffness_nm_rad, null);
  near(report.impedance.command_stiffness_nm_rad, 5, 0.05);
  const partial = analyzeExperiment(data(cfg, samples.slice(0, 340)));
  assert.equal(partial.impedance.responses[0].recovery_s, null);
  assert.equal(partial.impedance.command_stiffness_nm_rad, null);
});

test("thermal metrics follow recorded time and exports retain raw fields plus derived analysis", () => {
  const samples = Array.from({ length: 6001 }, (_, i) => {
    const t = i / 100, p = sample(t, i, 0.2 + t * 0.0001, 0.0001);
    p.temperature_c = 25 + t * 0.02;
    p.current_a = 0.4;
    p.current_age_ms = i % 20 * 10;
    return p;
  });
  const run = data(config("ThermalHold", 60), samples), report = analyzeExperiment(run);
  near(report.thermal.temperature_slope_c_min, 1.2, 1e-10);
  near(report.thermal.position_drift_rad, 0.0054, 0.00002);
  near(report.thermal.fresh_current_rms_a, 0.4, 1e-10);
  const json = JSON.parse(experimentJson(run, false));
  assert.equal(json.analysis.kind, "ThermalHold");
  assert.equal(json.derived_samples, undefined);
  near(json.pendulum.inertiaKgM2, model.inertiaKgM2, 1e-12);
  const [headers, firstRow] = experimentCsv(run).trim().split("\r\n").map(line => line.split(","));
  assert.equal(headers.length, firstRow.length);
  assert.ok(headers.includes("current_age_ms"));
  assert.ok(headers.includes("gravity_torque_nm"));
  assert.ok(headers.includes("target_unwrapped_position_rad"));
});

test("sampling report detects duplicate frames and thinning without changing the raw export", () => {
  const a = sample(0, 1, 0, 0), b = sample(0.01, 2, 0, 0), c = sample(1, 3, 0, 0);
  const run = data(config("Record", 10), [a, b, b, c]);
  run.status.samples = 100;
  const report = analyzeExperiment(run);
  assert.equal(report.sample_count, 3);
  assert.equal(report.quality.duplicate_samples, 1);
  assert.ok(report.quality.notes.some(n => n.includes("抽样")));
  assert.equal(JSON.parse(experimentJson(run)).derived_samples.length, 4);
});

function staticFrictionRun(direction: 1 | -1 = 1) {
  const cfg = { ...config("StaticFriction", 45), torque_nm: direction * 0.6, torque_ramp_rate_nm_s: 0.02,
    max_excursion_rad: 3 * Math.PI / 180, motion_threshold_rad: 0.15 * Math.PI / 180,
    motion_confirm_s: 0.04, settle_time_s: 0.5 };
  // A captured rest position may differ slightly from the independently measured gravitational zero.
  const reference = 0.006, rampStart = 0.5, onsetTime = 10.5;
  const event = (t: number, sequence: number, displacement: number, velocity: number) => {
    const theta = reference + direction * displacement;
    const p = sample(t, sequence, theta, direction * velocity,
      model.gravityNm * Math.sin(theta) + direction * (0.16 + 0.012));
    p.target = { kind: "Mit", position_rad: reference, velocity_rad_s: 0,
      torque_nm: direction * (t - rampStart) * cfg.torque_ramp_rate_nm_s, kp: 0, kd: 0 };
    return p;
  };
  const stationary = event(10.48, 1048, 0.002, 0.0001);
  const onset = event(onsetTime, 1050, 0.003, 0.012);
  const confirmation = event(onsetTime + 0.04, 1054, 0.005, 0.07);
  // Event frames deliberately do not occur in the thinned plotting series.
  const run = data(cfg, [sample(0, 0, reference, 0), sample(0.5, 50, reference, 0),
    sample(10.4, 1040, reference, 0), sample(11, 1100, reference, 0)]);
  run.status.static_friction = { phase: "Complete", outcome: "BreakawayDetected", reference_position_rad: reference,
    ramp_started_s: rampStart, last_stationary_sample: stationary, onset_sample: onset,
    confirmation_sample: confirmation, peak_excursion_rad: 0.012, return_started_s: confirmation.t_s };
  return run;
}

function bidirectionalStaticFrictionRun() {
  const positive = staticFrictionRun(1), negative = staticFrictionRun(-1);
  const shifted = (sample: El05TelemetrySample | null) => sample ? { ...sample, t_s: sample.t_s + 12, sequence: sample.sequence + 1200 } : null;
  const first = { ...positive.status.static_friction!, direction: 1 as const, started_s: 0, return_settled: true };
  negative.status.config = { ...positive.status.config!, friction_direction: "Both" };
  negative.status.static_friction = { ...negative.status.static_friction!, direction: -1, started_s: 12,
    ramp_started_s: 12.5, return_started_s: 22.54, return_settled: true, completed_directions: [first],
    last_stationary_sample: shifted(negative.status.static_friction!.last_stationary_sample),
    onset_sample: shifted(negative.status.static_friction!.onset_sample),
    confirmation_sample: shifted(negative.status.static_friction!.confirmation_sample) };
  negative.samples = [...positive.samples, ...negative.samples.map(sample => shifted(sample)!)];
  negative.status.elapsed_s = 23;
  negative.status.samples = negative.samples.length;
  negative.feedback_velocity_sign = 1;
  return negative;
}

test("one bidirectional calibration keeps both signed onset events in analysis, CSV and JSON", () => {
  const run = bidirectionalStaticFrictionRun();
  // Make the reverse mechanism measurably different from the forward one.
  run.status.static_friction!.onset_sample!.torque_nm -= 0.08;
  const report = analyzeExperiment(run), sf = report.static_friction!;
  assert.deepEqual(sf.directions.map(pass => pass.direction), [1, -1]);
  assert.equal(sf.both_directions_valid, true);
  near(sf.directions[0].onset_command_torque_nm, 0.2, 1e-10);
  near(sf.directions[1].onset_command_torque_nm, -0.2, 1e-10);
  near(sf.directions[0].onset_time_s, 10.5, 1e-10);
  near(sf.directions[1].onset_time_s, 22.5, 1e-10);
  near(sf.directions[1].ramp_started_s, 12.5, 1e-10);
  near(sf.directions[0].onset_gravity_adjusted_torque_nm, 0.172, 1e-10);
  near(sf.directions[1].onset_gravity_adjusted_torque_nm, -0.252, 1e-10);
  near(sf.adjusted_magnitude_difference_nm, -0.08, 1e-10);
  assert.ok(report.sections.find(section => section.id === "static_friction_positive"));
  assert.ok(report.sections.find(section => section.id === "static_friction_negative"));
  const summary = report.sections.find(section => section.id === "static_friction_summary")!;
  assert.equal(summary.metrics.find(metric => metric.key === "static_friction_both_valid")!.value, 1);
  const exported = JSON.parse(experimentJson(run));
  assert.equal(exported.status.static_friction.completed_directions[0].onset_sample.sequence, 1050);
  assert.equal(exported.status.static_friction.onset_sample.sequence, 2250);
  assert.equal(exported.analysis.static_friction.directions.length, 2);
  const csv = experimentCsv(run).trim().split("\r\n").map(row => row.split(","));
  const direction = csv[0].indexOf("static_friction_direction"), phase = csv[0].indexOf("static_friction_phase");
  assert.deepEqual(csv.slice(1).map(row => row[direction]), ["1", "1", "1", "1", "-1", "-1", "-1", "-1"]);
  assert.deepEqual(csv.slice(1).map(row => row[phase]), ["Settling", "Ramping", "Ramping", "Returning", "Settling", "Ramping", "Ramping", "Returning"]);
});

test("a failed or interrupted second direction preserves the first result without a fabricated comparison", () => {
  for (const outcome of ["Timeout", "Interrupted", "AngleLimit", "NotSettled", "TorqueLimit"] as const) {
    const run = bidirectionalStaticFrictionRun();
    run.status.static_friction!.outcome = outcome;
    const sf = analyzeExperiment(run).static_friction!;
    assert.equal(sf.directions[0].valid_breakaway, true);
    near(sf.directions[0].onset_command_torque_nm, 0.2, 1e-10);
    assert.equal(sf.directions[1].valid_breakaway, false);
    assert.equal(sf.directions[1].onset_feedback_torque_nm, null);
    assert.equal(sf.both_directions_valid, false);
    assert.equal(sf.feedback_magnitude_difference_nm, null);
    assert.equal(sf.adjusted_magnitude_difference_nm, null);
  }
  const run = staticFrictionRun();
  run.status.config!.friction_direction = "Both";
  const sf = analyzeExperiment(run).static_friction!;
  assert.equal(sf.directions.length, 1, "an untested reverse direction must not be invented");
  assert.equal(sf.both_directions_valid, false);
});

test("static friction analysis preserves captured first-motion events and signed forward/reverse torque estimates", () => {
  for (const direction of [1, -1] as const) {
    const run = staticFrictionRun(direction), report = analyzeExperiment(run), sf = report.static_friction!;
    assert.equal(sf.valid_breakaway, true);
    assert.equal(sf.outcome, "BreakawayDetected");
    near(sf.last_stationary_command_torque_nm, direction * 0.1996, 1e-10);
    near(sf.last_stationary_feedback_torque_nm, run.status.static_friction!.last_stationary_sample!.torque_nm, 1e-10);
    near(sf.onset_command_torque_nm, direction * 0.2, 1e-10);
    near(sf.confirmation_command_torque_nm, direction * 0.2008, 1e-10);
    near(sf.confirmation_feedback_torque_nm, run.status.static_friction!.confirmation_sample!.torque_nm, 1e-10);
    near(sf.detection_delay_s, 0.04, 1e-10);
    near(sf.command_increase_during_detection_nm, direction * 0.0008, 1e-10);
    near(sf.onset_angle_rad, direction * 0.003, 1e-10);
    near(sf.onset_gravity_torque_nm, model.gravityNm * Math.sin(0.006 + direction * 0.003), 1e-10);
    near(sf.onset_gravity_adjusted_torque_nm, direction * 0.172, 1e-10,
      "first-motion estimate still includes the deliberately added inertial torque");
    near(sf.onset_time_s, 10.5, 1e-10);
    near(sf.ramp_started_s, 0.5, 1e-10);
    near(sf.ramp_rate_nm_s, 0.02, 1e-10);
    assert.ok(!report.quality.notes.some(note => note.includes("未覆盖计划全程")), "event-completed calibration is not an incomplete timed experiment");
    const section = report.sections.find(s => s.id === "static_friction")!;
    near(section.metrics.find(m => m.key === "static_friction_onset_angle")!.value, direction * 0.003 * 180 / Math.PI, 1e-10);
    assert.equal(section.metrics.find(m => m.key === "static_friction_max_angle")!.unit, "°");
    assert.ok(section.notes.some(note => note.includes("严格上下界")));
    assert.ok(section.notes.some(note => note.includes("惯性")));
    const exported = JSON.parse(experimentJson(run));
    assert.equal(exported.status.static_friction.onset_sample.sequence, 1050);
    near(exported.analysis.static_friction.onset_command_torque_nm, direction * 0.2, 1e-10);
  }
});

test("static friction does not treat an unmeasured rest position as a gravitational zero", () => {
  const run = staticFrictionRun();
  run.status.config!.pendulum!.zero_measured = false;
  const report = analyzeExperiment(run), sf = report.static_friction!;
  assert.equal(sf.valid_breakaway, true);
  near(sf.onset_command_torque_nm, 0.2, 1e-10);
  near(sf.onset_feedback_torque_nm, run.status.static_friction!.onset_sample!.torque_nm, 1e-10);
  assert.equal(sf.onset_gravity_torque_nm, null);
  assert.equal(sf.onset_gravity_adjusted_torque_nm, null);
  assert.ok(report.sections.find(section => section.id === "static_friction")!.notes.some(note => note.includes("重力修正留空")));
});

test("static friction return timeout preserves confirmed breakaway and reports return stability separately", () => {
  const run = staticFrictionRun();
  run.status.static_friction!.return_settled = false;
  const report = analyzeExperiment(run), sf = report.static_friction!;
  assert.equal(sf.valid_breakaway, true);
  assert.equal(sf.return_settled, false);
  near(sf.onset_command_torque_nm, 0.2, 1e-10);
  near(sf.onset_gravity_adjusted_torque_nm, 0.172, 1e-10);
  const section = report.sections.find(section => section.id === "static_friction")!;
  assert.equal(section.metrics.find(metric => metric.key === "static_friction_return_settled")!.value, 0);
  assert.ok(section.notes.some(note => note.includes("回位在 3 秒内未稳定")));
  run.status.static_friction!.return_settled = true;
  assert.equal(analyzeExperiment(run).sections.find(section => section.id === "static_friction")!
    .metrics.find(metric => metric.key === "static_friction_return_settled")!.value, 1);
  delete run.status.static_friction!.return_settled;
  assert.equal(analyzeExperiment(run).static_friction!.return_settled, null, "older recordings do not imply successful return");
});

test("unsuccessful static friction outcomes retain excursion evidence without fabricated calibration results", () => {
  for (const outcome of ["AngleLimit", "TorqueLimit", "Timeout", "NotSettled", "Interrupted"] as const) {
    const run = staticFrictionRun();
    run.status.static_friction!.outcome = outcome;
    // An unconfirmed candidate, or an old event in an interrupted run, must not count as a calibration.
    run.status.static_friction!.confirmation_sample = null;
    const report = analyzeExperiment(run), sf = report.static_friction!;
    assert.equal(sf.valid_breakaway, false, outcome);
    assert.equal(sf.outcome, outcome);
    assert.equal(sf.onset_command_torque_nm, null);
    assert.equal(sf.onset_feedback_torque_nm, null);
    assert.equal(sf.onset_gravity_adjusted_torque_nm, null);
    assert.equal(sf.last_stationary_feedback_torque_nm, null);
    assert.equal(sf.confirmation_command_torque_nm, null);
    assert.equal(sf.onset_time_s, null);
    near(sf.peak_excursion_rad, 0.012, 1e-10);
    near(sf.max_excursion_rad, 3 * Math.PI / 180, 1e-10);
    assert.ok(sf.reason!.includes("未"));
    assert.equal(report.sections.find(section => section.id === "static_friction")!.metrics[0].value, 0);
  }
});

test("static friction requires complete backend events instead of inferring onset again from plot data", () => {
  const run = staticFrictionRun();
  const state = run.status.static_friction!;
  state.confirmation_sample = null;
  assert.equal(analyzeExperiment(run).static_friction!.valid_breakaway, false);
  state.confirmation_sample = { ...state.onset_sample!, t_s: state.onset_sample!.t_s - 0.01 };
  assert.equal(analyzeExperiment(run).static_friction!.onset_feedback_torque_nm, null, "out-of-order event times cannot be calibrated");
  run.status.static_friction = null;
  run.samples = Array.from({ length: 100 }, (_, i) => sample(i / 100, i, i * 0.001, 0.1, 0.2));
  const sf = analyzeExperiment(run).static_friction!;
  assert.equal(sf.valid_breakaway, false);
  assert.equal(sf.onset_command_torque_nm, null);
  assert.ok(sf.reason!.includes("缺少后端"));
  assert.equal(analyzeExperiment(data(config("TorqueRamp", 1), run.samples)).static_friction, undefined);
});

test("static friction does not calibrate hand-driven motion while the drive is inactive or faulted", () => {
  for (const faulted of [false, true]) {
    const run = staticFrictionRun();
    if (faulted) run.status.static_friction!.onset_sample!.fault_bits = 1;
    else run.status.static_friction!.confirmation_sample!.status_code = 0;
    const sf = analyzeExperiment(run).static_friction!;
    assert.equal(sf.valid_breakaway, false);
    assert.equal(sf.onset_feedback_torque_nm, null);
    assert.ok(sf.reason!.includes("驱动非运行状态或故障"));
  }
});
