import assert from "node:assert/strict";
import test from "node:test";
import { pendulumModel, experimentCsv } from "../src/el05Experiment.ts";
import type { El05ExperimentData } from "../src/el05Api.ts";

test("180 mm, 1 kg end mass gravity and inertia include the uniform arm", () => {
  const bare = pendulumModel(1, 0.18, 0);
  assert.ok(Math.abs(bare.gravityNm - 1.765197) < 1e-10);
  assert.ok(Math.abs(bare.inertiaKgM2 - 0.0324) < 1e-10);
  const arm = pendulumModel(1, 0.18, 0.2);
  assert.ok(Math.abs(arm.gravityNm - 1.9417167) < 1e-10);
  assert.ok(Math.abs(arm.inertiaKgM2 - 0.03456) < 1e-10);
});

test("CSV preserves distinct speed sources, missing values, ages and signed MIT targets", () => {
  const data: El05ExperimentData = { motor_id: 1, uid: "abc", commands: [], parameters: [], version: null, status: { active: false, config: null, samples: 1, elapsed_s: 1, reason: "duration completed", dropped_frames: 0 },
    samples: [{ t_s: 0.123, sequence: 7, can_id: 0x028001fd, payload: [0, 1, 2, 3, 4, 5, 6, 7], position_rad: 0.1, unwrapped_position_rad: 25.24,
      velocity_rad_s: -0.5, position_velocity_rad_s: -0.49, torque_nm: -0.1, current_a: null, current_age_ms: null,
      dictionary_velocity_rad_s: -0.48, dictionary_velocity_age_ms: 83, temperature_c: 25, fault_bits: 0, status_code: 2,
      target_sequence: 4, target_age_ms: 12, target: { kind: "Mit", position_rad: 0, velocity_rad_s: -0.6, torque_nm: 0, kp: 0, kd: 0.5 } }] };
  const lines = experimentCsv(data).trim().split("\r\n").map(line => line.split(","));
  const row = Object.fromEntries(lines[0].map((name, i) => [name, lines[1][i]]));
  assert.equal(row.velocity_rad_s, "-0.5");
  assert.equal(row.position_velocity_rad_s, "-0.49");
  assert.equal(row.dictionary_velocity_rad_s, "-0.48");
  assert.equal(row.dictionary_velocity_age_ms, "83");
  assert.equal(row.current_a, "");
  assert.equal(row.target_velocity_rad_s, "-0.6");
  assert.equal(row.target_torque_nm, "0");
  assert.equal(row.unwrapped_position_rad, "25.24");
  assert.equal(row.sequence, "7");
  assert.equal(row.target_sequence, "4");
  assert.equal(row.target_age_ms, "12");
  assert.equal(row.can_id_hex, "028001fd");
  assert.equal(row.payload_hex, "00 01 02 03 04 05 06 07");
  assert.equal(lines[0].length, lines[1].length);
});
