import assert from "node:assert/strict";
import test from "node:test";
import { smartknobModeTuning } from "../src/smartknobUi.ts";

const savedMode = {
  pGain: 4, dGain: 0.02, strengthScale: 0.15,
  effortLimit: 2, maxOutputPermille: 700,
  frictionCompensation: 0.03, clickEffort: 0.1,
};

test("Meow mode changes restore feel while preserving reduced session limits", () => {
  const current = { ...savedMode, pGain: 1, effortLimit: 0.1, maxOutputPermille: 50 };
  assert.deepEqual(smartknobModeTuning("meow", current, savedMode), {
    ...savedMode, effortLimit: 0.1, maxOutputPermille: 50,
  });
  assert.equal(savedMode.effortLimit, 2);
  assert.equal(current.pGain, 1);
});

test("Meow mode changes preserve zero output limits", () => {
  const current = { ...savedMode, effortLimit: 0, maxOutputPermille: 0 };
  const restored = smartknobModeTuning("meow", current, savedMode);
  assert.equal(restored.effortLimit, 0);
  assert.equal(restored.maxOutputPermille, 0);
});

test("existing protocols retain their per-mode tuning behavior", () => {
  for (const kind of ["canopen", "rollercan"] as const) {
    assert.deepEqual(smartknobModeTuning(kind, { ...savedMode, effortLimit: 0 }, savedMode), savedMode);
  }
});
