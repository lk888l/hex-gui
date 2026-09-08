import type { SixMotorMitRequest, SixMotorMitTuning } from "./types";

export const SIX_MOTOR_MIT_IDS = [1, 2, 3, 4, 5, 6] as const;
export const SIX_MOTOR_MIT_CONTROL_RATE_HZ = 500;

export const SIX_MOTOR_MIT_DEFAULT_WAYPOINTS: readonly (readonly number[])[] = [
  [0, 0.25, 0.25, -0.01, 0, 0],
  [0, 0.18, 0.18, -0.07, 0.01, 0],
  [-0.14, 0.11, 0.08, 0.06, 0.03, 0],
  [0.12, 0.1, 0.2, -0.14, 0.1, 0],
];

export const SIX_MOTOR_MIT_DEFAULT_GRAVITY_DIRECTIONS = [-1, -1, 1, 1, 1, 1] as const;
export const SIX_MOTOR_MIT_DEFAULT_PARK_POSITIONS_REV = [
  0.000269,
  0.249714,
  0.251193,
  -0.00159,
  -0.004028,
  0.000018,
] as const;
export const SIX_MOTOR_MIT_DEFAULT_PARK_ANGLES_RAD = [0, -1.57, 3.14, 0, 0, 0] as const;
export const SIX_MOTOR_MIT_DEFAULT_GRAVITY_SCALES = [0, 0.3, 0.7, 0.7, 0, 0] as const;
export const SIX_MOTOR_MIT_DEFAULT_GRAVITY_LIMITS_NM = [0.2, 5, 5, 1, 0.2, 0.1] as const;
export const SIX_MOTOR_MIT_DEFAULT_KP_NM_PER_RAD = [80, 80, 120, 110, 80, 80] as const;
export const SIX_MOTOR_MIT_DEFAULT_MAX_TORQUE_PERMILLE = [650, 650, 650, 650, 650, 650] as const;

/**
 * Error/recovery views can have `running=false` while a retained 500 Hz hold
 * still owns the bus. Treat every non-terminal six-axis session as requiring
 * an explicit Stop so navigation and unmount cleanup cannot orphan it.
 */
export function sixMotorMitSessionNeedsStop(
  view: { motors: readonly unknown[]; phase: string } | null | undefined,
): boolean {
  return Boolean(view && view.phase !== "idle" && view.phase !== "stopped");
}

export const SIX_MOTOR_MIT_DEFAULT_TUNING: Readonly<SixMotorMitTuning> = {
  tff_nm: 0,
  gravity_direction: SIX_MOTOR_MIT_DEFAULT_GRAVITY_DIRECTIONS[0],
  gravity_park_position_rev: SIX_MOTOR_MIT_DEFAULT_PARK_POSITIONS_REV[0],
  gravity_park_angle_rad: SIX_MOTOR_MIT_DEFAULT_PARK_ANGLES_RAD[0],
  gravity_scale: SIX_MOTOR_MIT_DEFAULT_GRAVITY_SCALES[0],
  gravity_max_torque_nm: SIX_MOTOR_MIT_DEFAULT_GRAVITY_LIMITS_NM[0],
  kp_nm_per_rad: SIX_MOTOR_MIT_DEFAULT_KP_NM_PER_RAD[0],
  kd_nm_s_per_rad: 15,
  kp_kd_limit_permille: 500,
  max_torque_permille: SIX_MOTOR_MIT_DEFAULT_MAX_TORQUE_PERMILLE[0],
};

export function createSixMotorMitDefaultRequest(): SixMotorMitRequest {
  return {
    waypoints: SIX_MOTOR_MIT_DEFAULT_WAYPOINTS.map((waypoint) => [...waypoint]),
    // The public ROS2 project deliberately does not contain a validated real-
    // hardware direction profile. Keep the useful model defaults prefilled,
    // but require the operator to verify each sign before enabling gravity.
    gravity_enabled: false,
    capture_gravity_park_on_start: true,
    gravity_vector_m_per_s2: [0, 0, -9.81],
    tff_slew_rate_nm_per_s: 5,
    payload_mass_kg: 0,
    payload_center_of_mass_m: [0, 0, 0],
    max_velocity_rev_per_s: 0.2,
    acceleration_rev_per_s2: 0.2,
    deceleration_rev_per_s2: 0.2,
    arrival_position_tolerance_rev: 0.003,
    arrival_velocity_tolerance_rev_per_s: 0.02,
    settle_time_ms: 200,
    movement_timeout_ms: 15_000,
    waypoint_dwell_ms: 5_000,
    tuning: SIX_MOTOR_MIT_IDS.map((_, index) => ({
      ...SIX_MOTOR_MIT_DEFAULT_TUNING,
      gravity_direction: SIX_MOTOR_MIT_DEFAULT_GRAVITY_DIRECTIONS[index],
      gravity_park_position_rev: SIX_MOTOR_MIT_DEFAULT_PARK_POSITIONS_REV[index],
      gravity_park_angle_rad: SIX_MOTOR_MIT_DEFAULT_PARK_ANGLES_RAD[index],
      gravity_scale: SIX_MOTOR_MIT_DEFAULT_GRAVITY_SCALES[index],
      gravity_max_torque_nm: SIX_MOTOR_MIT_DEFAULT_GRAVITY_LIMITS_NM[index],
      kp_nm_per_rad: SIX_MOTOR_MIT_DEFAULT_KP_NM_PER_RAD[index],
      max_torque_permille: SIX_MOTOR_MIT_DEFAULT_MAX_TORQUE_PERMILLE[index],
    })),
  };
}

export function cloneSixMotorMitRequest(request: SixMotorMitRequest): SixMotorMitRequest {
  return {
    ...request,
    waypoints: request.waypoints.map((waypoint) => [...waypoint]),
    gravity_vector_m_per_s2: [...request.gravity_vector_m_per_s2],
    payload_center_of_mass_m: [...request.payload_center_of_mass_m],
    tuning: request.tuning.map((tuning) => ({ ...tuning })),
  };
}

export function validateSixMotorMitRequest(request: SixMotorMitRequest): void {
  if (request.waypoints.length !== SIX_MOTOR_MIT_DEFAULT_WAYPOINTS.length) {
    throw new RangeError(`Expected ${SIX_MOTOR_MIT_DEFAULT_WAYPOINTS.length} waypoints`);
  }
  request.waypoints.forEach((waypoint, waypointIndex) => {
    if (waypoint.length !== SIX_MOTOR_MIT_IDS.length) {
      throw new RangeError(
        `Waypoint ${waypointIndex + 1} must contain ${SIX_MOTOR_MIT_IDS.length} positions`,
      );
    }
    waypoint.forEach((positionRev, motorIndex) => {
      if (!Number.isFinite(positionRev) || positionRev < -128 || positionRev >= 128) {
        throw new RangeError(
          `Waypoint ${waypointIndex + 1}, motor ${SIX_MOTOR_MIT_IDS[motorIndex]} must be in [-128, 128) Rev`,
        );
      }
    });
  });

  validatePositive(request.max_velocity_rev_per_s, 5, "Maximum velocity");
  validatePositive(request.acceleration_rev_per_s2, 50, "Acceleration");
  validatePositive(request.deceleration_rev_per_s2, 50, "Deceleration");
  validatePositive(request.arrival_position_tolerance_rev, 1, "Position tolerance");
  validatePositive(
    request.arrival_velocity_tolerance_rev_per_s,
    5,
    "Velocity tolerance",
  );
  validateInteger(request.settle_time_ms, 20, 5_000, "Settle time");
  validateInteger(request.movement_timeout_ms, 1_000, 600_000, "Movement timeout");
  validateInteger(request.waypoint_dwell_ms, 0, 60_000, "Waypoint dwell");
  if (typeof request.gravity_enabled !== "boolean") {
    throw new TypeError("Gravity compensation enabled must be a boolean");
  }
  if (typeof request.capture_gravity_park_on_start !== "boolean") {
    throw new TypeError("Capture gravity park on start must be a boolean");
  }
  validateVector3(request.gravity_vector_m_per_s2, "Gravity vector", -20, 20);
  if (request.gravity_enabled) {
    const gravityMagnitude = Math.hypot(...request.gravity_vector_m_per_s2);
    if (gravityMagnitude < 8 || gravityMagnitude > 12) {
      throw new RangeError("Gravity vector magnitude must be in [8, 12] m/s^2 when enabled");
    }
  }
  validatePositive(request.tff_slew_rate_nm_per_s, 1_000, "Tff slew rate");
  validateFiniteRange(request.payload_mass_kg, 0, 100, "Payload mass");
  validateVector3(request.payload_center_of_mass_m, "Payload center of mass", -10, 10);

  if (request.tuning.length !== SIX_MOTOR_MIT_IDS.length) {
    throw new RangeError(`Expected tuning for ${SIX_MOTOR_MIT_IDS.length} motors`);
  }
  request.tuning.forEach((tuning, index) => {
    const id = SIX_MOTOR_MIT_IDS[index];
    if (!Number.isFinite(tuning.tff_nm) || Math.abs(tuning.tff_nm) > 1_000) {
      throw new RangeError(`Motor ${id} Tff must be finite and within +/-1000 Nm`);
    }
    if (tuning.gravity_direction !== -1 && tuning.gravity_direction !== 1) {
      throw new RangeError(`Motor ${id} gravity direction must be -1 or +1`);
    }
    if (
      !Number.isFinite(tuning.gravity_park_position_rev) ||
      tuning.gravity_park_position_rev < -128 ||
      tuning.gravity_park_position_rev >= 128
    ) {
      throw new RangeError(
        `Motor ${id} gravity park position must be finite and in [-128, 128) Rev`,
      );
    }
    validateFiniteRange(
      tuning.gravity_park_angle_rad,
      -2 * Math.PI,
      2 * Math.PI,
      `Motor ${id} gravity park angle`,
    );
    validateFiniteRange(tuning.gravity_scale, 0, 1.5, `Motor ${id} gravity scale`);
    if (
      !Number.isFinite(tuning.gravity_max_torque_nm) ||
      tuning.gravity_max_torque_nm < 0 ||
      tuning.gravity_max_torque_nm > 1_000
    ) {
      throw new RangeError(
        `Motor ${id} gravity maximum torque must be finite and in [0, 1000] Nm`,
      );
    }
    if (Math.abs(tuning.tff_nm) + tuning.gravity_max_torque_nm > 1_000) {
      throw new RangeError(
        `Motor ${id} constant Tff plus maximum gravity torque must stay within +/-1000 Nm`,
      );
    }
    for (const [name, value] of [
      ["Kp", tuning.kp_nm_per_rad],
      ["Kd", tuning.kd_nm_s_per_rad],
    ] as const) {
      if (!Number.isFinite(value) || value < 0) {
        throw new RangeError(`Motor ${id} ${name} must be finite and non-negative`);
      }
    }
    validateInteger(
      tuning.kp_kd_limit_permille,
      0,
      1_000,
      `Motor ${id} Kp/Kd output limit`,
    );
    validateInteger(
      tuning.max_torque_permille,
      0,
      1_000,
      `Motor ${id} maximum torque`,
    );
  });
}

function validatePositive(value: number, max: number, label: string): void {
  if (!Number.isFinite(value) || value <= 0 || value > max) {
    throw new RangeError(`${label} must be in (0, ${max}]`);
  }
}

function validateFiniteRange(value: number, min: number, max: number, label: string): void {
  if (!Number.isFinite(value) || value < min || value > max) {
    throw new RangeError(`${label} must be finite and in [${min}, ${max}]`);
  }
}

function validateVector3(
  value: readonly number[],
  label: string,
  min: number,
  max: number,
): void {
  if (value.length !== 3) {
    throw new RangeError(`${label} must contain exactly 3 values`);
  }
  value.forEach((component, index) =>
    validateFiniteRange(component, min, max, `${label} component ${index + 1}`),
  );
}

function validateInteger(value: number, min: number, max: number, label: string): void {
  if (!Number.isInteger(value) || value < min || value > max) {
    throw new RangeError(`${label} must be an integer in [${min}, ${max}]`);
  }
}
