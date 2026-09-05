import assert from "node:assert/strict";
import test from "node:test";

import {
  SIX_MOTOR_MIT_CONTROL_RATE_HZ,
  SIX_MOTOR_MIT_DEFAULT_GRAVITY_DIRECTIONS,
  SIX_MOTOR_MIT_DEFAULT_GRAVITY_LIMITS_NM,
  SIX_MOTOR_MIT_DEFAULT_GRAVITY_SCALES,
  SIX_MOTOR_MIT_DEFAULT_MAX_TORQUE_PERMILLE,
  SIX_MOTOR_MIT_DEFAULT_PARK_ANGLES_RAD,
  SIX_MOTOR_MIT_DEFAULT_PARK_POSITIONS_REV,
  SIX_MOTOR_MIT_DEFAULT_WAYPOINTS,
  SIX_MOTOR_MIT_IDS,
  cloneSixMotorMitRequest,
  createSixMotorMitDefaultRequest,
  sixMotorMitSessionNeedsStop,
  validateSixMotorMitRequest,
} from "../src/sixMotorMitTest.ts";

test("six-motor MIT defaults bind IDs 1-6 to four complete waypoints at 500 Hz", () => {
  const request = createSixMotorMitDefaultRequest();
  assert.deepEqual(SIX_MOTOR_MIT_IDS, [1, 2, 3, 4, 5, 6]);
  assert.equal(SIX_MOTOR_MIT_CONTROL_RATE_HZ, 500);
  assert.deepEqual(request.waypoints, SIX_MOTOR_MIT_DEFAULT_WAYPOINTS);
  assert.equal(request.gravity_enabled, false);
  assert.equal(request.capture_gravity_park_on_start, true);
  assert.deepEqual(request.gravity_vector_m_per_s2, [0, 0, -9.81]);
  assert.equal(request.tff_slew_rate_nm_per_s, 5);
  assert.equal(request.payload_mass_kg, 0);
  assert.deepEqual(request.payload_center_of_mass_m, [0, 0, 0]);
  assert.equal(request.waypoint_dwell_ms, 5_000);
  assert.equal(request.tuning.length, 6);
  assert.deepEqual(
    request.tuning.map((tuning) => tuning.gravity_direction),
    SIX_MOTOR_MIT_DEFAULT_GRAVITY_DIRECTIONS,
  );
  assert.deepEqual(
    request.tuning.map((tuning) => tuning.gravity_park_position_rev),
    SIX_MOTOR_MIT_DEFAULT_PARK_POSITIONS_REV,
  );
  assert.deepEqual(
    request.tuning.map((tuning) => tuning.gravity_park_angle_rad),
    SIX_MOTOR_MIT_DEFAULT_PARK_ANGLES_RAD,
  );
  assert.deepEqual(
    request.tuning.map((tuning) => tuning.gravity_scale),
    SIX_MOTOR_MIT_DEFAULT_GRAVITY_SCALES,
  );
  assert.deepEqual(
    request.tuning.map((tuning) => tuning.gravity_max_torque_nm),
    SIX_MOTOR_MIT_DEFAULT_GRAVITY_LIMITS_NM,
  );
  assert.deepEqual(
    request.tuning.map((tuning) => tuning.max_torque_permille),
    SIX_MOTOR_MIT_DEFAULT_MAX_TORQUE_PERMILLE,
  );
  assert.deepEqual(request.tuning.map((tuning) => tuning.kp_nm_per_rad), [80, 80, 80, 80, 80, 80]);
  assert.deepEqual(request.tuning.map((tuning) => tuning.kd_nm_s_per_rad), [15, 15, 15, 15, 15, 15]);
  assert.deepEqual(
    request.tuning.map((tuning) => tuning.kp_kd_limit_permille),
    [500, 500, 500, 500, 500, 500],
  );
  assert.doesNotThrow(() => validateSixMotorMitRequest(request));
});

test("six-motor MIT request copies do not mutate defaults or their source", () => {
  const source = createSixMotorMitDefaultRequest();
  const copy = cloneSixMotorMitRequest(source);
  copy.waypoints[0][0] = 1;
  copy.gravity_vector_m_per_s2[2] = -10;
  copy.payload_center_of_mass_m[0] = 0.2;
  copy.tuning[0].kp_nm_per_rad = 99;
  copy.tuning[0].gravity_park_position_rev = 0.5;
  assert.equal(source.waypoints[0][0], 0);
  assert.equal(source.gravity_vector_m_per_s2[2], -9.81);
  assert.equal(source.payload_center_of_mass_m[0], 0);
  assert.equal(source.tuning[0].kp_nm_per_rad, 80);
  assert.equal(source.tuning[0].gravity_park_position_rev, 0.000269);
  assert.equal(SIX_MOTOR_MIT_DEFAULT_WAYPOINTS[0][0], 0);
});

test("six-motor MIT recovery sessions still require an explicit stop", () => {
  const motors = Array.from({ length: SIX_MOTOR_MIT_IDS.length }, () => ({}));
  assert.equal(sixMotorMitSessionNeedsStop({ phase: "moving", motors }), true);
  assert.equal(sixMotorMitSessionNeedsStop({ phase: "error", motors }), true);
  assert.equal(sixMotorMitSessionNeedsStop({ phase: "error", motors: [{}] }), true);
  assert.equal(sixMotorMitSessionNeedsStop({ phase: "stopped", motors }), false);
  assert.equal(sixMotorMitSessionNeedsStop({ phase: "idle", motors: [] }), false);
  assert.equal(sixMotorMitSessionNeedsStop(null), false);
});

test("six-motor MIT validation rejects incomplete waypoints and invalid SDO limits", () => {
  const missingAxis = createSixMotorMitDefaultRequest();
  missingAxis.waypoints[0].pop();
  assert.throws(() => validateSixMotorMitRequest(missingAxis), /6 positions/);

  const outOfRange = createSixMotorMitDefaultRequest();
  outOfRange.waypoints[2][4] = 128;
  assert.throws(() => validateSixMotorMitRequest(outOfRange), /motor 5/);

  const invalidLimit = createSixMotorMitDefaultRequest();
  invalidLimit.tuning[1].kp_kd_limit_permille = 1001;
  assert.throws(() => validateSixMotorMitRequest(invalidLimit), /Motor 2/);

  const invalidGravity = createSixMotorMitDefaultRequest();
  invalidGravity.tuning[3].gravity_max_torque_nm = -0.01;
  assert.throws(() => validateSixMotorMitRequest(invalidGravity), /Motor 4 gravity/);

  const invalidCombinedTff = createSixMotorMitDefaultRequest();
  invalidCombinedTff.tuning[1].tff_nm = 999;
  invalidCombinedTff.tuning[1].gravity_max_torque_nm = 2;
  assert.throws(
    () => validateSixMotorMitRequest(invalidCombinedTff),
    /Motor 2 constant Tff plus maximum gravity torque/,
  );
});

test("six-motor MIT validation enforces gravity model and mapping bounds", () => {
  const invalidDirection = createSixMotorMitDefaultRequest();
  invalidDirection.tuning[0].gravity_direction = 0;
  assert.throws(() => validateSixMotorMitRequest(invalidDirection), /direction/);

  const invalidParkPosition = createSixMotorMitDefaultRequest();
  invalidParkPosition.tuning[1].gravity_park_position_rev = 128;
  assert.throws(() => validateSixMotorMitRequest(invalidParkPosition), /park position/);

  const invalidParkAngle = createSixMotorMitDefaultRequest();
  invalidParkAngle.tuning[2].gravity_park_angle_rad = 2 * Math.PI + 0.001;
  assert.throws(() => validateSixMotorMitRequest(invalidParkAngle), /park angle/);

  const invalidScale = createSixMotorMitDefaultRequest();
  invalidScale.tuning[3].gravity_scale = 1.5001;
  assert.throws(() => validateSixMotorMitRequest(invalidScale), /gravity scale/);

  const invalidGravityVector = createSixMotorMitDefaultRequest();
  invalidGravityVector.gravity_enabled = true;
  invalidGravityVector.gravity_vector_m_per_s2 = [0, 0, 0];
  assert.throws(() => validateSixMotorMitRequest(invalidGravityVector), /magnitude/);

  invalidGravityVector.gravity_enabled = false;
  assert.doesNotThrow(() => validateSixMotorMitRequest(invalidGravityVector));

  const invalidSlew = createSixMotorMitDefaultRequest();
  invalidSlew.tff_slew_rate_nm_per_s = 0;
  assert.throws(() => validateSixMotorMitRequest(invalidSlew), /slew rate/);

  const invalidPayload = createSixMotorMitDefaultRequest();
  invalidPayload.payload_mass_kg = 100.01;
  assert.throws(() => validateSixMotorMitRequest(invalidPayload), /Payload mass/);

  const invalidPayloadCom = createSixMotorMitDefaultRequest();
  invalidPayloadCom.payload_center_of_mass_m = [10.01, 0, 0];
  assert.throws(() => validateSixMotorMitRequest(invalidPayloadCom), /Payload center of mass/);
});
