import assert from "node:assert/strict";
import test from "node:test";
import { smartknobClampTuning, smartknobInitialOutputLimits, smartknobModeTuning } from "../src/smartknobUi.ts";

const savedMode = {
  pGain: 4, dGain: 0.02, strengthScale: 0.15,
  effortLimit: 2, maxOutputPermille: 700,
  frictionCompensation: 0.03, clickEffort: 0.1,
};

for (const kind of ["meow", "damiao"] as const) {
  test(`${kind} mode changes restore feel while preserving reduced session limits`, () => {
    const current = { ...savedMode, pGain: 1, effortLimit: 0.1, maxOutputPermille: 50 };
    assert.deepEqual(smartknobModeTuning(kind, current, savedMode), {
      ...savedMode, effortLimit: 0.1, maxOutputPermille: 50,
    });
    assert.equal(savedMode.effortLimit, 2);
    assert.equal(current.pGain, 1);
  });

  test(`${kind} mode changes preserve zero output limits`, () => {
    const current = { ...savedMode, effortLimit: 0, maxOutputPermille: 0 };
    const restored = smartknobModeTuning(kind, current, savedMode);
    assert.equal(restored.effortLimit, 0);
    assert.equal(restored.maxOutputPermille, 0);
  });
}

test("DM-H3510 starts below its 0.18 Nm continuous rating", () => {
  const limits = smartknobInitialOutputLimits({
    target: { kind: "damiao", nodeId: 1 },
    effortUnit: "Nm", effortLimitMax: 0.45, maxOutputPermille: 1000,
  });
  assert.deepEqual(limits, { effortLimit: 0.12, maxOutputPermille: 400 });
});

test("DM-H3510 defaults respect lower limits from the motor profile", () => {
  assert.deepEqual(smartknobInitialOutputLimits({
    target: { kind: "damiao", nodeId: 1 },
    effortUnit: "Nm", effortLimitMax: 0.08, maxOutputPermille: 100,
  }), { effortLimit: 0.08, maxOutputPermille: 100 });
});

test("existing protocols retain their per-mode tuning behavior", () => {
  for (const kind of ["canopen", "rollercan"] as const) {
    assert.deepEqual(smartknobModeTuning(kind, { ...savedMode, effortLimit: 0 }, savedMode), savedMode);
  }
});

test("Meow uses the live physical peak for its single limit, not a saved percent", () => {
  const profile = { target: { kind: "meow" as const, nodeId: 1 }, effortLimitMax: 2.5 };
  const tuned = smartknobClampTuning(profile, {
    ...savedMode, effortLimit: 0.31, maxOutputPermille: 0, frictionCompensation: 0,
  });
  assert.equal(tuned.effortLimit, 0.31);
  assert.equal(tuned.maxOutputPermille, 124);
  assert.equal(tuned.frictionCompensation, 0.31);
  const restored = smartknobModeTuning("meow", tuned, savedMode);
  assert.equal(restored.effortLimit, 0.31);
  assert.equal(restored.maxOutputPermille, 124);
});

test("Meow clamps cached limits to a replacement motor and preserves zero", () => {
  const profile = { target: { kind: "meow" as const, nodeId: 1 }, effortLimitMax: 0.08 };
  const capped = smartknobClampTuning(profile, savedMode);
  assert.equal(capped.effortLimit, 0.08);
  assert.equal(capped.maxOutputPermille, 1000);
  for (const limit of [0, -1, NaN, Infinity]) {
    const zero = smartknobClampTuning(profile, { ...savedMode, effortLimit: limit });
    assert.equal(zero.effortLimit, 0);
    assert.equal(zero.maxOutputPermille, 0);
    assert.equal(zero.frictionCompensation, 0);
  }
  assert.equal(smartknobClampTuning({ ...profile, effortLimitMax: NaN }, savedMode).effortLimit, 0);
});

test("Other protocols retain their existing internal motor protection", () => {
  for (const kind of ["canopen", "rollercan", "damiao"] as const) {
    assert.deepEqual(smartknobClampTuning({ target: { kind, nodeId: 1 }, effortLimitMax: 2.5 }, savedMode), savedMode);
  }
});
