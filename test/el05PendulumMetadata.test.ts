import assert from "node:assert/strict";
import test from "node:test";
import { experimentJson } from "../src/el05Experiment.ts";
import type { El05ExperimentData } from "../src/el05Api.ts";

test("JSON retains the recorded pendulum geometry, downward reference and measurement status", () => {
  const geometry = { mass_kg: 1, length_m: 0.18, arm_mass_kg: 0, downward_zero_rad: 30.25, zero_measured: true };
  const data: El05ExperimentData = {
    status: { active: false, config: { kind: "Record", duration_s: 12, velocity_rad_s: 0.3, torque_nm: 0.3,
      kd: 0.5, max_temperature_c: 60, pendulum: { ...geometry } },
      samples: 0, elapsed_s: 12, reason: "duration completed", dropped_frames: 0 },
    motor_id: 1, uid: "abc", samples: [], commands: [], parameters: [], version: null,
  };
  // Changes to the user's next-run fields cannot alter the recorded metadata.
  geometry.mass_kg = 2;
  geometry.downward_zero_rad = 50;
  const exported = JSON.parse(experimentJson(data));
  assert.equal(exported.pendulum.mass_kg, 1);
  assert.equal(exported.pendulum.downward_zero_rad, 30.25);
  assert.equal(exported.pendulum.zero_measured, true);
  assert.equal(exported.status.config.pendulum.downward_zero_rad, 30.25);
  assert.ok(Math.abs(exported.pendulum.gravityNm - 1.765197) < 1e-10);
  assert.ok(Math.abs(exported.pendulum.inertiaKgM2 - 0.0324) < 1e-10);
  data.status.config!.pendulum!.zero_measured = false;
  assert.equal(JSON.parse(experimentJson(data)).pendulum.zero_measured, false);
  data.status.config!.pendulum = null;
  assert.equal(JSON.parse(experimentJson(data)).pendulum, null);
});
