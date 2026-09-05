import assert from "node:assert/strict";
import test from "node:test";

import {
  SIX_MOTOR_PP_DEFAULT_INTERVAL_MS,
  SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS,
  SIX_MOTOR_PP_DEFAULT_WAYPOINTS,
  SIX_MOTOR_PP_IDS,
  cloneSixMotorPpRequest,
  createSixMotorPpDefaultRequest,
  validateSixMotorPpRequest,
} from "../src/sixMotorPpTest.ts";

test("six-motor PP defaults retain the latest committed four-waypoint sequence", () => {
  const request = createSixMotorPpDefaultRequest();
  assert.deepEqual(SIX_MOTOR_PP_IDS, [1, 2, 3, 4, 5, 6]);
  assert.equal(SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS, 10);
  assert.equal(SIX_MOTOR_PP_DEFAULT_INTERVAL_MS, 2_500);
  assert.equal(request.waypoint_dwell_ms, SIX_MOTOR_PP_DEFAULT_INTERVAL_MS);
  assert.deepEqual(request.waypoints, SIX_MOTOR_PP_DEFAULT_WAYPOINTS);
  assert.doesNotThrow(() => validateSixMotorPpRequest(request));
});

test("six-motor PP request copies are independently editable", () => {
  const source = createSixMotorPpDefaultRequest();
  const copy = cloneSixMotorPpRequest(source);
  copy.waypoints[0][0] = 1;
  copy.profile.velocity_rev_per_s = 0.5;
  assert.equal(source.waypoints[0][0], 0);
  assert.equal(source.profile.velocity_rev_per_s, 0.2);
});

test("six-motor PP validation rejects missing axes and unsafe arrival settings", () => {
  const missingAxis = createSixMotorPpDefaultRequest();
  missingAxis.waypoints[0].pop();
  assert.throws(() => validateSixMotorPpRequest(missingAxis), /6 positions/);

  const outOfRange = createSixMotorPpDefaultRequest();
  outOfRange.waypoints[2][4] = 128;
  assert.throws(() => validateSixMotorPpRequest(outOfRange), /motor 5/);

  const invalidTimeout = createSixMotorPpDefaultRequest();
  invalidTimeout.movement_timeout_ms = 999;
  assert.throws(() => validateSixMotorPpRequest(invalidTimeout), /Movement timeout/);
});
