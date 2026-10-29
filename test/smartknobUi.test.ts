import assert from "node:assert/strict";
import test from "node:test";
import { smartknobInitialOutputLimits, smartknobModeTuning } from "../src/smartknobUi.ts";

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
