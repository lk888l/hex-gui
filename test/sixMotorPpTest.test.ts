import assert from "node:assert/strict";
import test from "node:test";

import {
  SIX_MOTOR_PP_DEFAULT_INTERVAL_MS,
  SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS,
  SIX_MOTOR_PP_DEFAULT_WAYPOINTS,
  SIX_MOTOR_PP_IDS,
  cloneSixMotorPpDefaultWaypoints,
  validateSixMotorPpWaypoints,
} from "../src/sixMotorPpTest.ts";

test("six-motor PP defaults bind IDs 1-6 to four complete waypoints", () => {
  assert.deepEqual(SIX_MOTOR_PP_IDS, [1, 2, 3, 4, 5, 6]);
  assert.equal(SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS, 10);
  assert.equal(SIX_MOTOR_PP_DEFAULT_INTERVAL_MS, 2_500);
  assert.deepEqual(SIX_MOTOR_PP_DEFAULT_WAYPOINTS, [
    [0, 0.25, 0.25, -0.01, 0, 0],
    [0, 0.18, 0.18, -0.07, 0.01, 0],
    [-0.14, 0.11, 0.08, 0.06, 0.03, 0],
    [0.12, 0.1, 0.2, -0.14, 0.1, 0],
  ]);
  assert.doesNotThrow(() => validateSixMotorPpWaypoints(SIX_MOTOR_PP_DEFAULT_WAYPOINTS));
});

test("six-motor PP waypoint copies are editable without changing defaults", () => {
  const copy = cloneSixMotorPpDefaultWaypoints();
  copy[0][0] = 1;
  assert.equal(SIX_MOTOR_PP_DEFAULT_WAYPOINTS[0][0], 0);
});

test("six-motor PP waypoints reject a missing axis or out-of-range position", () => {
  const missingAxis = cloneSixMotorPpDefaultWaypoints();
  missingAxis[0].pop();
  assert.throws(() => validateSixMotorPpWaypoints(missingAxis), /6 positions/);

  const outOfRange = cloneSixMotorPpDefaultWaypoints();
  outOfRange[2][4] = 128;
  assert.throws(() => validateSixMotorPpWaypoints(outOfRange), /motor 5/);
});
