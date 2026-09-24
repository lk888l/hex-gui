// Fixed HopeA3 geometry, matching hopea3.rs. Velocities are at the output shaft.
export const HOPE_WHEEL_DIAMETER_M = 0.154;
export const HOPE_DEFAULT_MAX_LINEAR_MPS = 20;

export function hopeSpeedPlan(speedMps: number, direction: "forward" | "left") {
  const wheelRevPerS = speedMps / (Math.PI * HOPE_WHEEL_DIAMETER_M);
  const wheels = direction === "forward"
    ? [-Math.sqrt(3) / 2 * wheelRevPerS, 0, Math.sqrt(3) / 2 * wheelRevPerS]
    : [wheelRevPerS / 2, -wheelRevPerS, wheelRevPerS / 2];
  return { kmh: speedMps * 3.6, wheelRevPerS, wheels };
}

export function hopeRampEstimate(speedMps: number, acceleration: number) {
  if (acceleration <= 0) return null;
  return { seconds: speedMps / acceleration, metres: speedMps ** 2 / (2 * acceleration) };
}
