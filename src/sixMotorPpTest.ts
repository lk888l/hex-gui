import type { SixMotorPpRequest } from "./types";

export const SIX_MOTOR_PP_IDS = [1, 2, 3, 4, 5, 6] as const;
export const SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS = 10;
// Keep the original 1-6 PP test's pause between complete waypoints.  The
// sequence is now measured motor-by-motor, but its four poses and dwell remain
// compatible with the latest committed frontend test.
export const SIX_MOTOR_PP_DEFAULT_INTERVAL_MS = 2_500;

export const SIX_MOTOR_PP_DEFAULT_WAYPOINTS: readonly (readonly number[])[] = [
  [0, 0.25, 0.25, -0.01, 0, 0],
  [0, 0.18, 0.18, -0.07, 0.01, 0],
  [-0.14, 0.11, 0.08, 0.06, 0.03, 0],
  [0.12, 0.1, 0.2, -0.14, 0.1, 0],
];

export function createSixMotorPpDefaultRequest(): SixMotorPpRequest {
  return {
    waypoints: SIX_MOTOR_PP_DEFAULT_WAYPOINTS.map((waypoint) => [...waypoint]),
    tpdo_event_timer_ms: SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS,
    max_torque_permille: 200,
    profile: {
      velocity_rev_per_s: 0.2,
      acceleration_rev_per_s2: 0.2,
      deceleration_rev_per_s2: 0.2,
    },
    arrival_position_tolerance_rev: 0.003,
    arrival_velocity_tolerance_rev_per_s: 0.02,
    settle_time_ms: 200,
    movement_timeout_ms: 15_000,
    waypoint_dwell_ms: SIX_MOTOR_PP_DEFAULT_INTERVAL_MS,
  };
}

export function cloneSixMotorPpRequest(request: SixMotorPpRequest): SixMotorPpRequest {
  return {
    ...request,
    waypoints: request.waypoints.map((waypoint) => [...waypoint]),
    profile: { ...request.profile },
  };
}

export function validateSixMotorPpRequest(request: SixMotorPpRequest): void {
  if (request.waypoints.length !== SIX_MOTOR_PP_DEFAULT_WAYPOINTS.length) {
    throw new RangeError(`Expected ${SIX_MOTOR_PP_DEFAULT_WAYPOINTS.length} waypoints`);
  }
  request.waypoints.forEach((waypoint, waypointIndex) => {
    if (waypoint.length !== SIX_MOTOR_PP_IDS.length) {
      throw new RangeError(
        `Waypoint ${waypointIndex + 1} must contain ${SIX_MOTOR_PP_IDS.length} positions`,
      );
    }
    waypoint.forEach((positionRev, motorIndex) => {
      if (!Number.isFinite(positionRev) || positionRev < -128 || positionRev >= 128) {
        throw new RangeError(
          `Waypoint ${waypointIndex + 1}, motor ${SIX_MOTOR_PP_IDS[motorIndex]} must be in [-128, 128) Rev`,
        );
      }
    });
  });
  validateInteger(request.tpdo_event_timer_ms, 1, 100, "TPDO event timer");
  validateInteger(request.max_torque_permille, 0, 1_000, "Maximum torque");
  validatePositive(request.profile.velocity_rev_per_s, 5, "Profile velocity");
  validatePositive(request.profile.acceleration_rev_per_s2, 50, "Profile acceleration");
  validatePositive(request.profile.deceleration_rev_per_s2, 50, "Profile deceleration");
  validatePositive(request.arrival_position_tolerance_rev, 1, "Position tolerance");
  validatePositive(
    request.arrival_velocity_tolerance_rev_per_s,
    5,
    "Velocity tolerance",
  );
  validateInteger(request.settle_time_ms, 20, 5_000, "Settle time");
  validateInteger(request.movement_timeout_ms, 1_000, 600_000, "Movement timeout");
  validateInteger(request.waypoint_dwell_ms, 0, 60_000, "Waypoint dwell");
}

function validatePositive(value: number, max: number, label: string): void {
  if (!Number.isFinite(value) || value <= 0 || value > max) {
    throw new RangeError(`${label} must be in (0, ${max}]`);
  }
}

function validateInteger(value: number, min: number, max: number, label: string): void {
  if (!Number.isInteger(value) || value < min || value > max) {
    throw new RangeError(`${label} must be an integer in [${min}, ${max}]`);
  }
}
