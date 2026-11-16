import assert from "node:assert/strict";
import { test } from "node:test";
import { chipAttitude, defaultPdos, eulerDegrees, pdoBytes, relativeQuaternion, wireBytes, testSummary } from "../src/hpmImu.ts";
import type { HpmImuValue, Quaternion, HpmImuTestRow } from "../src/hpmImu.ts";

const value = (index: number, sub: number, value: number): HpmImuValue => ({ index, sub, value, age_ms: 0, changed_age_ms: 0, rx_unix_ms: 123 });
const samples = () => [value(0x3008, 1, 100), value(0x3008, 2, 5), value(0x3008, 5, 7),
  ...[1, 0, 0, 0].map((q, i) => value(0x3000, i + 1, q))];

test("bimu defaults match the 29-byte firmware layouts and FD padding", () => {
  const pdos = defaultPdos(31);
  assert.equal(pdos[0].cob_id, 0x19f);
  assert.equal(pdos[1].cob_id, 0x29f);
  assert.equal(pdos[2].cob_id, 0x8000039f);
  assert.equal(pdos[0].mapping.length, 13);
  assert.equal(pdoBytes(pdos[0]), 29);
  assert.equal(pdoBytes(pdos[1]), 29);
  assert.equal(wireBytes(29), 32);
  assert.equal(wireBytes(6), 6);
  assert.equal(wireBytes(64), 64);
});

test("attitude requires fresh flags, a changing counter and all four components", () => {
  assert.deepEqual(chipAttitude(samples()), [1, 0, 0, 0]);
  assert.equal(chipAttitude(samples(), 250), null);
  const counterStalled = samples(); counterStalled[0].changed_age_ms = 300;
  assert.equal(chipAttitude(counterStalled), null);
  const noSample = samples(); noSample[1].value = 0xffffffff;
  assert.equal(chipAttitude(noSample), null);
  const invalid = samples(); invalid[2].value = 6;
  assert.equal(chipAttitude(invalid), null);
  assert.equal(chipAttitude(samples().slice(0, -1)), null);
  const corrupt = samples(); corrupt[3].value = 0;
  assert.equal(chipAttitude(corrupt), null);
});

test("display reference respects WXYZ body-to-world quaternions", () => {
  const s = Math.sqrt(0.5);
  for (let axis = 0; axis < 3; axis++) {
    const q: Quaternion = [s, 0, 0, 0]; q[axis + 1] = s;
    const euler = eulerDegrees(q);
    assert.ok(Math.abs(euler[axis] - 90) < 1e-5);
    const zero = relativeQuaternion(q, q);
    assert.ok(Math.abs(zero[0] - 1) < 1e-10);
    assert.ok(Math.hypot(...zero.slice(1)) < 1e-10);
  }
});

test("angle test summaries retain absolute error when signed errors cancel", () => {
  assert.equal(testSummary([]), null);
  const rows = [-1.8, 1.8].map((error, i): HpmImuTestRow => ({
    round: i + 1, target_deg: 180, motor_deg: 180, home_raw_deg: 0, raw_deg: 180 + error,
    error_deg: error, error_percent: error / 1.8, absolute_error_percent: 1, return_raw_deg: null,
  }));
  const summary = testSummary(rows)!;
  assert.equal(summary.meanError, 0);
  assert.equal(summary.meanAbsolutePercent, 1);
  assert.equal(summary.maxAbsolutePercent, 1);
  assert.equal(summary.rmsDegrees, 1.8);
});
