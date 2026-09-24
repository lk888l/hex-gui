import assert from "node:assert/strict";
import test from "node:test";
import { hopeSpeedPlan, hopeRampEstimate } from "../src/hopea3Speed.ts";

test("20 m/s uses wheel output speed and the three omni-wheel projections", () => {
  const forward = hopeSpeedPlan(20, "forward");
  assert.equal(forward.kmh, 72);
  assert.ok(Math.abs(forward.wheelRevPerS - 41.33894626) < 1e-8);
  assert.ok(Math.abs(forward.wheels[0] + 35.80057762) < 1e-8);
  assert.equal(forward.wheels[1], 0);
  assert.equal(forward.wheels[2], -forward.wheels[0]);
  const left = hopeSpeedPlan(20, "left");
  assert.equal(left.wheels[1], -forward.wheelRevPerS);
  assert.equal(left.wheels[0], left.wheels[2]);
  assert.equal(left.wheels[0], -left.wheels[1] / 2);
});

test("the default 2 m/s² ramp to 20 m/s needs 10 s and 100 m", () => {
  assert.deepEqual(hopeRampEstimate(20, 2), { seconds: 10, metres: 100 });
  assert.equal(hopeRampEstimate(20, 0), null);
  assert.deepEqual(hopeRampEstimate(0, 2), { seconds: 0, metres: 0 });
});
