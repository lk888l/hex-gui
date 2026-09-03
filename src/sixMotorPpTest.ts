import {
  PP_POSITION_MAX_REV_EXCLUSIVE,
  PP_POSITION_MIN_REV,
  isRepresentablePpPosition,
} from "./ppAccuracy.ts";

export const SIX_MOTOR_PP_IDS = [1, 2, 3, 4, 5, 6] as const;

export const SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS = 10;
export const SIX_MOTOR_PP_DEFAULT_INTERVAL_MS = 2_500;

export const SIX_MOTOR_PP_DEFAULT_WAYPOINTS: readonly (readonly number[])[] = [
  [0, 0.25, 0.25, -0.01, 0, 0],
  [0, 0.18, 0.18, -0.07, 0.01, 0],
  [-0.14, 0.11, 0.08, 0.06, 0.03, 0],
  [0.12, 0.1, 0.2, -0.14, 0.1, 0],
];

export function cloneSixMotorPpDefaultWaypoints(): number[][] {
  return SIX_MOTOR_PP_DEFAULT_WAYPOINTS.map((waypoint) => [...waypoint]);
}

export function validateSixMotorPpWaypoints(waypoints: readonly (readonly number[])[]): void {
  if (waypoints.length !== SIX_MOTOR_PP_DEFAULT_WAYPOINTS.length) {
    throw new RangeError(`Expected ${SIX_MOTOR_PP_DEFAULT_WAYPOINTS.length} waypoints`);
  }

  waypoints.forEach((waypoint, waypointIndex) => {
    if (waypoint.length !== SIX_MOTOR_PP_IDS.length) {
      throw new RangeError(
        `Waypoint ${waypointIndex} must contain ${SIX_MOTOR_PP_IDS.length} positions`,
      );
    }
    waypoint.forEach((positionRev, motorIndex) => {
      if (!isRepresentablePpPosition(positionRev)) {
        throw new RangeError(
          `Waypoint ${waypointIndex}, motor ${SIX_MOTOR_PP_IDS[motorIndex]} must be in ` +
            `[${PP_POSITION_MIN_REV}, ${PP_POSITION_MAX_REV_EXCLUSIVE}) rev`,
        );
      }
    });
  });
}
