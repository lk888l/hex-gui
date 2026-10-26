import type { El05ExperimentConfig, El05ExperimentData, El05StaticFrictionPass, El05TelemetrySample } from "./el05Api";
import { controlFeedbackEligible, feedbackVelocityBits, feedbackVelocityResolution } from "./el05Protocol.ts";

/** Native uint16 and standard MIT uint12 torque both map to -6..6 Nm. */
export const EL05_TORQUE_UNIT = "Nm";
export const EL05_TORQUE_LIMIT_NM = 6;

/**
 * Point mass at the arm end plus a uniform arm, angle measured from down.
 * kg × m/s² × m gives output-shaft Nm, the same unit as native drive feedback.
 * Neither the 9:1 gear ratio nor the 0.94 Nm/Arms constant is a unit conversion here.
 */
export function pendulumModel(massKg: number, lengthM: number, armMassKg: number, extraInertiaKgM2 = 0) {
  const gravityNm = 9.80665 * lengthM * (massKg + armMassKg / 2);
  const inertiaKgM2 = lengthM ** 2 * (massKg + armMassKg / 3) + extraInertiaKgM2;
  return { gravityNm, inertiaKgM2 };
}

/** Maximum gravity-only demand at the five distinct StaticTorque plateau targets. */
export function staticHoldingTorquePeakNm(config: El05ExperimentConfig | null | undefined): number | null {
  const pendulum = config?.pendulum;
  if (config?.kind !== "StaticTorque" || !pendulum?.zero_measured) return null;
  const { gravityNm } = pendulumModel(pendulum.mass_kg, pendulum.length_m, pendulum.arm_mass_kg);
  const center = config.center_angle_rad ?? 0, amplitude = config.position_amplitude_rad ?? 0.25;
  return Math.max(...[-1, -0.5, 0, 0.5, 1].map(offset => Math.abs(gravityNm * Math.sin(center + offset * amplitude))));
}

/** A captured 0x700B limit may be smaller than the native feedback/command range. */
export function experimentTorqueLimitNm(data: El05ExperimentData): number {
  const limit = data.parameters.find(parameter => parameter.index === 0x700b)?.value;
  return typeof limit === "number" && Number.isFinite(limit) && limit >= 0
    ? Math.min(limit, EL05_TORQUE_LIMIT_NM) : EL05_TORQUE_LIMIT_NM;
}

export interface El05DerivedSample {
  t_s: number;
  relative_position_rad: number | null;
  angle_rad: number | null;
  velocity_rad_s: number | null;
  acceleration_rad_s2: number | null;
  gravity_torque_nm: number | null;
  target_holding_torque_nm: number | null;
  inertial_torque_nm: number | null;
  friction_torque_nm: number | null;
  target_position_rad: number | null;
  position_error_rad: number | null;
  velocity_error_rad_s: number | null;
  fresh_current_a: number | null;
}

export interface El05AnalysisMetric {
  key: string;
  label: string;
  label_en?: string;
  value: number | null;
  unit: string;
  digits?: number;
  note?: string;
}
export interface El05AnalysisSection {
  id: string;
  title: string;
  title_en?: string;
  metrics: El05AnalysisMetric[];
  notes: string[];
}
export interface El05StepResponse {
  start_s: number;
  end_s: number;
  from: number;
  to: number;
  rise_time_s: number | null;
  overshoot_percent: number | null;
  settling_time_s: number | null;
  steady_error: number | null;
  complete: boolean;
}
export interface El05FrequencyPoint {
  frequency_hz: number;
  gain: number;
  gain_db: number;
  phase_deg: number;
  input_amplitude_rad: number;
  output_amplitude_rad: number;
  cycles: number;
  sample_count: number;
}
export interface El05StaticPlatform {
  start_s: number;
  end_s: number;
  target_angle_rad: number | null;
  angle_rad: number | null;
  gravity_torque_nm: number | null;
  current_a: number | null;
  feedback_torque_nm: number | null;
  approach: "positive" | "negative" | null;
  sample_count: number;
  fresh_current_samples: number;
}
export interface El05StaticFrictionPassAnalysis {
  direction: 1 | -1;
  phase: string | null;
  outcome: string | null;
  valid_breakaway: boolean;
  return_settled: boolean | null;
  reference_position_rad: number | null;
  last_stationary_command_torque_nm: number | null;
  last_stationary_feedback_torque_nm: number | null;
  onset_command_torque_nm: number | null;
  onset_feedback_torque_nm: number | null;
  confirmation_command_torque_nm: number | null;
  confirmation_feedback_torque_nm: number | null;
  onset_gravity_torque_nm: number | null;
  onset_gravity_adjusted_torque_nm: number | null;
  detection_delay_s: number | null;
  command_increase_during_detection_nm: number | null;
  onset_angle_rad: number | null;
  peak_excursion_rad: number | null;
  max_excursion_rad: number | null;
  ramp_started_s: number | null;
  onset_time_s: number | null;
  confirmation_time_s: number | null;
  ramp_rate_nm_s: number | null;
  reason: string | null;
}
export interface El05StaticFrictionAnalysis extends El05StaticFrictionPassAnalysis {
  directions: El05StaticFrictionPassAnalysis[];
  both_directions_valid: boolean;
  feedback_magnitude_difference_nm: number | null;
  adjusted_magnitude_difference_nm: number | null;
}
export interface El05ExperimentAnalysis {
  kind: string;
  sample_count: number;
  duration_s: number;
  quality: {
    sample_rate_hz: number | null;
    /** Measured command cadence; absent in older frozen analyses. */
    command_rate_hz?: number | null;
    median_interval_s: number | null;
    max_gap_s: number | null;
    duplicate_samples: number;
    dropped_frames: number;
    fresh_current_samples: number;
    inactive_drive_samples: number | null;
    inactive_drive_fraction: number | null;
    notes: string[];
  };
  sections: El05AnalysisSection[];
  limitations: string[];
  tracking: {
    position_rms_rad: number | null;
    velocity_rms_rad_s: number | null;
    position_steady_error_rad: number | null;
    velocity_steady_error_rad_s: number | null;
    position_steps: El05StepResponse[];
    velocity_steps: El05StepResponse[];
  };
  static_torque: {
    platforms: El05StaticPlatform[];
    torque_constant_nm_per_a: number | null;
    intercept_nm: number | null;
    r_squared: number | null;
    forward_reverse_torque_difference_nm: number | null;
    reason: string | null;
  };
  static_friction?: El05StaticFrictionAnalysis;
  friction: {
    coulomb_nm: number | null;
    viscous_nm_s_rad: number | null;
    r_squared: number | null;
    rmse_nm: number | null;
    sample_count: number;
    reason: string | null;
  };
  free_decay: {
    period_s: number | null;
    frequency_hz: number | null;
    log_decrement: number | null;
    damping_ratio: number | null;
    equivalent_viscous_nm_s_rad: number | null;
    peak_count: number;
    envelope_r_squared: number | null;
    reason: string | null;
  };
  frequency_response: { points: El05FrequencyPoint[]; bandwidth_hz: number | null; reason: string | null };
  impedance: {
    responses: { start_s: number; end_s: number; peak_displacement_rad: number | null; recovery_s: number | null;
      steady_displacement_rad: number | null; injection_torque_nm: number | null; command_stiffness_nm_rad: number | null }[];
    command_stiffness_nm_rad: number | null;
    command_stiffness_reason: string | null;
    equivalent_stiffness_nm_rad: number | null;
    reason: string;
  };
  thermal: {
    start_temperature_c: number | null;
    end_temperature_c: number | null;
    temperature_rise_c: number | null;
    temperature_slope_c_min: number | null;
    position_drift_rad: number | null;
    fresh_current_rms_a: number | null;
    reason: string | null;
  };
  motion: {
    speed_ripple_rad_s: number | null;
    positive_speed_rad_s: number | null;
    negative_speed_rad_s: number | null;
    minimum_observed_steady_speed_rad_s: number | null;
    reversal_count: number;
    breakaway_time_s: number | null;
    breakaway_motor_torque_nm: number | null;
    breakaway_gravity_adjusted_torque_nm: number | null;
    reason: string | null;
  };
}

type RunConfig = NonNullable<El05ExperimentData["status"]["config"]> & {
  origin_position_rad?: number;
  frequency_start_hz?: number;
  frequency_end_hz?: number;
  gravity_compensation?: boolean;
  disturbance_torque_nm?: number;
  torque_ramp_rate_nm_s?: number;
  max_excursion_rad?: number;
};
const finite = (n: unknown): n is number => typeof n === "number" && Number.isFinite(n);
const mean = (a: number[]): number | null => a.length ? a.reduce((s, n) => s + n, 0) / a.length : null;
const rms = (a: number[]): number | null => a.length ? Math.sqrt(a.reduce((s, n) => s + n * n, 0) / a.length) : null;
const median = (a: number[]): number | null => {
  if (!a.length) return null;
  const sorted = [...a].sort((a, b) => a - b);
  const mid = sorted.length >> 1;
  return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
};
const maximum = (a: number[]): number | null => a.length ? a.reduce((m, n) => Math.max(m, n), -Infinity) : null;
const minimum = (a: number[]): number | null => a.length ? a.reduce((m, n) => Math.min(m, n), Infinity) : null;
const deviation = (a: number[]): number | null => {
  const avg = mean(a);
  return avg === null ? null : rms(a.map(n => n - avg));
};
const present = (a: (number | null | undefined)[]): number[] => a.filter(finite);

/**
 * Use actual send completion times; sparse archives preserve sequence deltas.
 * Older recordings can fall back to the target receipt evidenced by telemetry.
 * Feedback cadence alone never establishes the controller's update frequency.
 */
export function observedCommandRateHz(data: El05ExperimentData): number | null {
  const estimate = (events: { sequence: number; time: number }[]) => {
    const ordered = events.filter(event => Number.isInteger(event.sequence) && event.sequence > 0 && finite(event.time))
      .sort((a, b) => a.sequence - b.sequence || a.time - b.time);
    const intervals: number[] = [];
    let previous: { sequence: number; time: number } | undefined;
    for (const event of ordered) {
      if (previous && event.sequence > previous.sequence && event.time > previous.time) {
        intervals.push((event.time - previous.time) / (event.sequence - previous.sequence));
      }
      if (!previous || event.sequence > previous.sequence) previous = event;
    }
    const interval = median(intervals);
    return interval != null && interval > 0 ? 1 / interval : null;
  };
  const sent = estimate(data.commands.map(command => ({ sequence: command.sequence,
    time: finite(command.completed_t_s) ? command.completed_t_s : command.t_s })));
  if (sent != null) return sent;
  return estimate(data.samples.filter(sample => sample.target?.kind === "Mit"
    && finite(sample.target_age_ms) && sample.target_age_ms >= 0 && sample.target_age_ms <= 250)
    .map(sample => ({ sequence: sample.target_sequence, time: sample.t_s - sample.target_age_ms! / 1000 })));
}

function regression(x: number[], y: number[]) {
  const mx = mean(x), my = mean(y);
  if (x.length < 3 || mx === null || my === null) return null;
  let xx = 0, xy = 0, yy = 0;
  for (let i = 0; i < x.length; i++) {
    xx += (x[i] - mx) ** 2;
    xy += (x[i] - mx) * (y[i] - my);
    yy += (y[i] - my) ** 2;
  }
  if (xx < 1e-10) return null;
  const slope = xy / xx, intercept = my - slope * mx;
  const residual = y.reduce((sum, n, i) => sum + (n - intercept - slope * x[i]) ** 2, 0);
  return { slope, intercept, r2: yy > 1e-12 ? 1 - residual / yy : residual < 1e-12 ? 1 : 0, rmse: Math.sqrt(residual / x.length) };
}

/** Keep a strictly increasing receive-time series and reject repeated CAN feedback. */
function orderedIndices(samples: El05TelemetrySample[]) {
  const seen = new Set<number>();
  const sorted = samples.map((_, i) => i).filter(i => finite(samples[i].t_s)).sort((a, b) => samples[a].t_s - samples[b].t_s);
  let lastTime = -Infinity;
  return sorted.filter(i => {
    const s = samples[i];
    if (s.t_s <= lastTime || seen.has(s.sequence)) return false;
    lastTime = s.t_s;
    seen.add(s.sequence);
    return finite(s.unwrapped_position_rad) && finite(s.velocity_rad_s) && finite(s.torque_nm);
  });
}

/**
 * Derivatives use a local least-squares fit of measured velocity (a 100 ms window),
 * never command velocity. Iq is accepted once per inferred dictionary reception,
 * at <= 250 ms age, so a cached 5 Hz reading is not treated as 500 Hz evidence.
 * The output remains aligned with the input, including duplicate/invalid rows.
 */
export function deriveExperimentSamples(data: El05ExperimentData): El05DerivedSample[] {
  const config = data.status.config as RunConfig | null;
  const pendulum = config?.pendulum;
  const geometry = pendulum ? pendulumModel(pendulum.mass_kg, pendulum.length_m, pendulum.arm_mass_kg,
    (pendulum as typeof pendulum & { extra_inertia_kg_m2?: number }).extra_inertia_kg_m2 ?? 0) : null;
  const origin = config?.origin_position_rad ?? data.samples.find(s => finite(s.unwrapped_position_rad))?.unwrapped_position_rad ?? 0;
  const result: El05DerivedSample[] = data.samples.map(s => {
    // Native fast feedback is synchronous with position and estimated torque.
    // The separate position-derived backend speed is a 150 ms backward
    // difference; preferring it would introduce about 75 ms of phase delay.
    const velocity = finite(s.velocity_rad_s) ? s.velocity_rad_s : finite(s.position_velocity_rad_s) ? s.position_velocity_rad_s : null;
    const angle = pendulum?.zero_measured && finite(s.unwrapped_position_rad) ? s.unwrapped_position_rad - pendulum.downward_zero_rad : null;
    const target = s.target?.kind === "Mit" && (s.target_age_ms === null || s.target_age_ms <= 250) ? s.target : null;
    // Both values are in the drive protocol window. Their difference is valid
    // across any number of revolutions; using a 2π wrap here would be incorrect.
    const targetPosition = target && target.kp > 0 && finite(s.position_rad) && finite(s.unwrapped_position_rad)
      ? s.unwrapped_position_rad + target.position_rad - s.position_rad : null;
    return {
      t_s: s.t_s,
      relative_position_rad: finite(s.unwrapped_position_rad) ? s.unwrapped_position_rad - origin : null,
      angle_rad: angle,
      velocity_rad_s: velocity,
      acceleration_rad_s2: null,
      gravity_torque_nm: angle !== null && geometry ? geometry.gravityNm * Math.sin(angle) : null,
      target_holding_torque_nm: targetPosition !== null && geometry && pendulum?.zero_measured
        ? geometry.gravityNm * Math.sin(targetPosition - pendulum.downward_zero_rad) : null,
      inertial_torque_nm: null,
      friction_torque_nm: null,
      target_position_rad: targetPosition,
      position_error_rad: targetPosition === null ? null : targetPosition - s.unwrapped_position_rad,
      velocity_error_rad_s: target && velocity !== null && (target.kd > 0 || target.kp > 0) ? target.velocity_rad_s - velocity : null,
      fresh_current_a: null,
    };
  });
  const indices = orderedIndices(data.samples);
  let left = 0, right = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, count = 0;
  let currentReception = -Infinity;
  const update = (index: number, sign: number) => {
    const p = result[indices[index]];
    if (p.velocity_rad_s === null) return;
    // Relative timestamps avoid cancellation on a long recording.
    const t = p.t_s - (data.samples[indices[0]]?.t_s ?? 0);
    sx += sign * t; sy += sign * p.velocity_rad_s; sxx += sign * t * t;
    sxy += sign * t * p.velocity_rad_s; count += sign;
  };
  for (let i = 0; i < indices.length; i++) {
    const source = data.samples[indices[i]], p = result[indices[i]];
    while (right < indices.length && result[indices[right]].t_s <= p.t_s + 0.05) update(right++, 1);
    while (left < right && result[indices[left]].t_s < p.t_s - 0.05) update(left++, -1);
    const span = right > left ? result[indices[right - 1]].t_s - result[indices[left]].t_s : 0;
    const denominator = count * sxx - sx * sx;
    // Reject gaps and one-sided endpoints; their acceleration is not reliable.
    if (count >= 4 && span >= 0.025 && p.t_s - result[indices[left]].t_s >= 0.01
      && result[indices[right - 1]].t_s - p.t_s >= 0.01 && denominator > 1e-10) {
      p.acceleration_rad_s2 = (count * sxy - sx * sy) / denominator;
      if (geometry && geometry.inertiaKgM2 > 0) p.inertial_torque_nm = geometry.inertiaKgM2 * p.acceleration_rad_s2;
      if (p.gravity_torque_nm !== null && p.inertial_torque_nm !== null) {
        p.friction_torque_nm = source.torque_nm - p.gravity_torque_nm - p.inertial_torque_nm;
      }
    }
    if (finite(source.current_a) && finite(source.current_age_ms) && source.current_age_ms >= 0 && source.current_age_ms <= 250) {
      const receipt = source.t_s - source.current_age_ms / 1000;
      if (receipt > currentReception + 0.002) {
        p.fresh_current_a = source.current_a;
        currentReception = receipt;
      }
    }
  }
  return result;
}

type Point = { s: El05TelemetrySample; d: El05DerivedSample };
type Segment = { start: number; end: number; target: number; previous: number | null; points: Point[] };

function targetSegments(points: Point[], mode: "position" | "velocity"): Segment[] {
  const segments: Segment[] = [];
  const tolerance = mode === "position" ? 1e-4 : 1e-3;
  const medianInterval = median(points.slice(1).map((p, i) => p.s.t_s - points[i].s.t_s)) ?? 0;
  const gapLimit = Math.max(0.15, medianInterval * 5);
  for (const p of points) {
    const target = mode === "position" ? p.d.target_position_rad : p.s.target?.kind === "Mit"
      && p.d.velocity_error_rad_s !== null ? p.s.target.velocity_rad_s : null;
    if (target === null) continue;
    const last = segments[segments.length - 1];
    const gap = last && p.s.t_s - (last.points[last.points.length - 1]?.s.t_s ?? p.s.t_s) > gapLimit;
    if (!last || gap || Math.abs(last.target - target) > tolerance) {
      if (last && !gap) last.end = p.s.t_s;
      segments.push({ start: p.s.t_s, end: p.s.t_s, target, previous: gap ? null : last?.target ?? null, points: [p] });
    } else {
      last.end = p.s.t_s;
      last.points.push(p);
    }
  }
  return segments;
}

function stepResponses(points: Point[], mode: "position" | "velocity"): El05StepResponse[] {
  const segments = targetSegments(points, mode);
  return segments.flatMap((segment, index) => {
    if (segment.previous === null || Math.abs(segment.target - segment.previous) < (mode === "position" ? 0.01 : 0.02)
      || segment.points.length < 4 || segment.end - segment.start < 0.1) return [];
    const amplitude = segment.target - segment.previous;
    const value = (p: Point) => mode === "position" ? p.s.unwrapped_position_rad : p.d.velocity_rad_s;
    const previousPoints = segments[index - 1].points;
    const baseline = mean(present(previousPoints.filter(p => p.s.t_s >= segment.start - 0.2).map(value))) ?? segment.previous;
    const response = segment.points.filter(p => value(p) !== null);
    const crossing = (level: number) => {
      for (let i = 0; i < response.length; i++) {
        const y = ((value(response[i]) as number) - baseline) / amplitude;
        if (y >= level) {
          if (!i) return response[i].s.t_s;
          const prev = ((value(response[i - 1]) as number) - baseline) / amplitude;
          const f = y === prev ? 1 : Math.max(0, Math.min(1, (level - prev) / (y - prev)));
          return response[i - 1].s.t_s + f * (response[i].s.t_s - response[i - 1].s.t_s);
        }
      }
      return null;
    };
    const ten = crossing(0.1), ninety = crossing(0.9);
    const tolerance = Math.max(Math.abs(amplitude) * 0.02, mode === "position" ? 0.005 : 0.01);
    let lastOutside = -1;
    response.forEach((p, i) => { if (Math.abs((value(p) as number) - segment.target) > tolerance) lastOutside = i; });
    const settleAt = response[lastOutside + 1]?.s.t_s ?? null;
    const sustained = settleAt !== null && segment.end - settleAt >= Math.max(0.15, (segment.end - segment.start) * 0.1);
    const steadyStart = segment.end - Math.min(0.5, (segment.end - segment.start) * 0.2);
    const steady = response.filter(p => p.s.t_s >= steadyStart);
    const peak = maximum(response.map(p => ((value(p) as number) - segment.target) / amplitude));
    return [{ start_s: segment.start, end_s: segment.end, from: segment.previous, to: segment.target,
      rise_time_s: ten !== null && ninety !== null ? ninety - ten : null,
      overshoot_percent: peak === null ? null : Math.max(0, peak * 100),
      settling_time_s: sustained ? settleAt! - segment.start : null,
      steady_error: steady.length >= 3 && segment.end - steadyStart >= 0.1 ? mean(steady.map(p => segment.target - (value(p) as number))) : null,
      complete: index < segments.length - 1,
    }];
  });
}

function fitFriction(points: Point[], hasReference: boolean): El05ExperimentAnalysis["friction"] {
  const eligible = points.filter(p => p.d.friction_torque_nm !== null && p.d.velocity_rad_s !== null && Math.abs(p.d.velocity_rad_s) > 0.03);
  const empty = (reason: string) => ({ coulomb_nm: null, viscous_nm_s_rad: null, r_squared: null, rmse_nm: null, sample_count: eligible.length, reason });
  if (!hasReference) return empty("需实测下垂参考及负载惯量，才能扣除重力矩和惯性矩。");
  if (eligible.length < 40) return empty("有效运动样本不足，需连续采集正反向运动。");
  const velocities = eligible.map(p => p.d.velocity_rad_s as number);
  const positive = velocities.filter(v => v > 0).length;
  const absolute = velocities.map(Math.abs);
  if (positive < 10 || velocities.length - positive < 10 || (deviation(absolute) ?? 0) < Math.max(0.03, (mean(absolute) ?? 0) * 0.1)) {
    return empty("需正反向及足够多的不同速度，恒定单一速度无法分离库仑摩擦和粘性阻尼。");
  }
  let signVelocity = 0, velocity2 = 0, signTorque = 0, velocityTorque = 0;
  for (const p of eligible) {
    const v = p.d.velocity_rad_s!, torque = p.d.friction_torque_nm!;
    signVelocity += Math.abs(v); velocity2 += v * v;
    signTorque += Math.sign(v) * torque; velocityTorque += v * torque;
  }
  const n = eligible.length, det = n * velocity2 - signVelocity ** 2;
  if (det / (n * velocity2) < 0.005) return empty("速度激励不足，回归矩阵接近奇异。");
  const coulomb = (signTorque * velocity2 - velocityTorque * signVelocity) / det;
  const viscous = (velocityTorque * n - signTorque * signVelocity) / det;
  const ymean = mean(eligible.map(p => p.d.friction_torque_nm!))!;
  let residual = 0, total = 0;
  for (const p of eligible) {
    residual += (p.d.friction_torque_nm! - coulomb * Math.sign(p.d.velocity_rad_s!) - viscous * p.d.velocity_rad_s!) ** 2;
    total += (p.d.friction_torque_nm! - ymean) ** 2;
  }
  const r2 = total > 1e-12 ? 1 - residual / total : null;
  const reason = coulomb < 0 || viscous < 0 ? "拟合出现负摩擦参数；应检查重力零点、惯量、转矩估计和模型适用性。"
    : r2 === null || r2 < 0.5 ? "拟合质量较低，二参数摩擦模型未能充分解释数据。" : null;
  return { coulomb_nm: coulomb, viscous_nm_s_rad: viscous, r_squared: r2, rmse_nm: Math.sqrt(residual / n), sample_count: n, reason };
}

function staticAnalysis(points: Point[], config: RunConfig | null): El05ExperimentAnalysis["static_torque"] {
  const platforms: El05StaticPlatform[] = [];
  const regressionCurrent: number[] = [], regressionTorque: number[] = [];
  const reference = config?.pendulum?.zero_measured ?? false;
  for (const segment of targetSegments(points, "position")) {
    const duration = segment.end - segment.start;
    if (duration < 0.3 || segment.points.length < 5) continue;
    const stable = segment.points.filter(p => p.s.t_s >= segment.start + Math.max(duration * 0.6, 0.2)
      && p.d.velocity_rad_s !== null && Math.abs(p.d.velocity_rad_s) <= 0.03
      && p.d.acceleration_rad_s2 !== null && Math.abs(p.d.acceleration_rad_s2) <= 0.2
      && p.d.position_error_rad !== null && Math.abs(p.d.position_error_rad) < 0.08);
    if (stable.length < 3 || stable[stable.length - 1].s.t_s - stable[0].s.t_s < 0.1) continue;
    const fresh = stable.filter(p => p.d.fresh_current_a !== null);
    for (const p of fresh) if (p.d.gravity_torque_nm !== null) {
      regressionCurrent.push(p.d.fresh_current_a!);
      regressionTorque.push(p.d.gravity_torque_nm);
    }
    platforms.push({ start_s: segment.start, end_s: segment.end,
      target_angle_rad: reference ? segment.target - config!.pendulum!.downward_zero_rad : null,
      angle_rad: mean(present(stable.map(p => p.d.angle_rad))),
      gravity_torque_nm: mean(present(stable.map(p => p.d.gravity_torque_nm))),
      feedback_torque_nm: mean(stable.map(p => p.s.torque_nm)),
      current_a: mean(present(fresh.map(p => p.d.fresh_current_a))),
      approach: segment.previous === null ? null : segment.target > segment.previous ? "positive" : "negative",
      sample_count: stable.length, fresh_current_samples: fresh.length });
  }
  let reason: string | null = null;
  const fit = regression(regressionCurrent, regressionTorque);
  const torqueSpread = (maximum(regressionTorque) ?? 0) - (minimum(regressionTorque) ?? 0);
  const validFit = fit && regressionCurrent.length >= 6 && platforms.filter(p => p.fresh_current_samples > 0).length >= 3 && torqueSpread >= 0.05;
  if (!reference) reason = "未实测下垂参考，无法用重力负载标定转矩与 Iq 的关系。";
  else if (!validFit) reason = "需至少三个不同负载的稳定平台、六个独立新鲜 Iq 读数和足够的转矩跨度。";
  const differences: number[] = [];
  for (let i = 0; i < platforms.length; i++) for (let j = i + 1; j < platforms.length; j++) {
    const a = platforms[i], b = platforms[j];
    if (a.target_angle_rad !== null && b.target_angle_rad !== null && Math.abs(a.target_angle_rad - b.target_angle_rad) < 0.005
      && a.approach && b.approach && a.approach !== b.approach && a.feedback_torque_nm !== null && b.feedback_torque_nm !== null) {
      differences.push((a.approach === "positive" ? 1 : -1) * (a.feedback_torque_nm - b.feedback_torque_nm));
    }
  }
  return { platforms, torque_constant_nm_per_a: validFit ? fit.slope : null,
    intercept_nm: validFit ? fit.intercept : null, r_squared: validFit ? fit.r2 : null,
    forward_reverse_torque_difference_nm: mean(differences), reason };
}

function decayAnalysis(points: Point[], config: RunConfig | null): El05ExperimentAnalysis["free_decay"] {
  const empty = (reason: string): El05ExperimentAnalysis["free_decay"] => ({ period_s: null, frequency_hz: null,
    log_decrement: null, damping_ratio: null, equivalent_viscous_nm_s_rad: null, peak_count: 0, envelope_r_squared: null, reason });
  if (points.length < 20) return empty("需至少数个完整自由摆动周期。");
  const peaks: { t: number; value: number; sign: number }[] = [];
  // Position quantization may create short flat maxima: detect slope changes
  // across the last nonzero increment instead of requiring a strict point peak.
  let previousDirection = 0;
  for (let i = 1; i < points.length; i++) {
    const delta = points[i].s.unwrapped_position_rad - points[i - 1].s.unwrapped_position_rad;
    const direction = Math.abs(delta) > 1e-6 ? Math.sign(delta) : 0;
    if (direction && previousDirection && direction !== previousDirection) {
      const p = points[i - 1], sign = previousDirection;
      const peak = { t: p.s.t_s, value: p.s.unwrapped_position_rad, sign };
      const last = peaks[peaks.length - 1];
      if (!last || peak.t - last.t >= 0.12) peaks.push(peak);
      else if (last.sign === sign && sign * peak.value > sign * last.value) peaks[peaks.length - 1] = peak;
    }
    if (direction) previousDirection = direction;
  }
  const maxima = peaks.filter(p => p.sign > 0);
  const periods = maxima.slice(1).map((p, i) => p.t - maxima[i].t);
  const period = median(periods);
  if (maxima.length < 3 || period === null || period <= 0 || (deviation(periods) ?? 0) / period > 0.2) return empty("未识别到至少三个间隔一致的同向峰值，不能可靠估计周期。");
  const envelopes = maxima.flatMap((peak, i) => {
    const next = maxima[i + 1];
    if (!next) return [];
    const valley = peaks.find(p => p.sign < 0 && p.t > peak.t && p.t < next.t);
    const amplitude = valley ? (peak.value - valley.value) / 2 : 0;
    return amplitude > 0.005 ? [{ t: (peak.t + valley!.t) / 2, amplitude }] : [];
  });
  const fit = regression(envelopes.map(p => p.t), envelopes.map(p => Math.log(p.amplitude)));
  const decrements = envelopes.slice(1).flatMap((p, i) => {
    const decrement = Math.log(envelopes[i].amplitude / p.amplitude);
    return decrement > 0 && decrement < 3 ? [decrement] : [];
  });
  const delta = decrements.length >= 2 ? median(decrements) : null;
  const exponential = fit !== null && fit.slope < 0 && fit.r2 >= 0.9;
  const zeta = delta !== null && exponential ? delta / Math.sqrt(4 * Math.PI ** 2 + delta ** 2) : null;
  const pendulum = config?.pendulum;
  const inertia = pendulum ? pendulumModel(pendulum.mass_kg, pendulum.length_m, pendulum.arm_mass_kg,
    (pendulum as typeof pendulum & { extra_inertia_kg_m2?: number }).extra_inertia_kg_m2 ?? 0).inertiaKgM2 : null;
  const smallAngle = (maximum(envelopes.map(p => p.amplitude)) ?? Infinity) <= 0.35;
  const equivalent = zeta !== null && inertia !== null && inertia > 0 && smallAngle ? 2 * zeta * inertia * (2 * Math.PI / period) / Math.sqrt(1 - zeta ** 2) : null;
  const reason = !exponential ? "包络未满足指数衰减，阻尼比不作粘性阻尼解释；可能包含库仑摩擦或噪声。"
    : !smallAngle ? "摆幅超过约 20°，小角度线性等效阻尼不输出。" : !inertia ? "缺少有效负载惯量，无法估计等效粘性阻尼。" : null;
  return { period_s: period, frequency_hz: 1 / period, log_decrement: delta, damping_ratio: zeta,
    equivalent_viscous_nm_s_rad: equivalent, peak_count: peaks.length, envelope_r_squared: fit?.r2 ?? null, reason };
}

/** Solve a small least-squares normal matrix with pivoting. */
function solve(matrix: number[][], vector: number[]) {
  const a = matrix.map((row, i) => [...row, vector[i]]), n = vector.length;
  for (let i = 0; i < n; i++) {
    let pivot = i;
    for (let j = i + 1; j < n; j++) if (Math.abs(a[j][i]) > Math.abs(a[pivot][i])) pivot = j;
    [a[i], a[pivot]] = [a[pivot], a[i]];
    if (Math.abs(a[i][i]) < 1e-9) return null;
    const scale = a[i][i];
    for (let k = i; k <= n; k++) a[i][k] /= scale;
    for (let j = 0; j < n; j++) if (j !== i) {
      const factor = a[j][i];
      for (let k = i; k <= n; k++) a[j][k] -= factor * a[i][k];
    }
  }
  return a.map(row => row[n]);
}

function harmonicFit(points: Point[], phase: (t: number) => number, value: (p: Point) => number) {
  const center = (points[0].s.t_s + points[points.length - 1].s.t_s) / 2;
  const span = points[points.length - 1].s.t_s - points[0].s.t_s;
  const matrix = Array.from({ length: 4 }, () => Array(4).fill(0) as number[]), vector = [0, 0, 0, 0];
  const rows = points.map(p => [Math.sin(phase(p.s.t_s)), Math.cos(phase(p.s.t_s)), 1, (p.s.t_s - center) / span]);
  for (let i = 0; i < points.length; i++) for (let j = 0; j < 4; j++) {
    vector[j] += rows[i][j] * value(points[i]);
    for (let k = 0; k < 4; k++) matrix[j][k] += rows[i][j] * rows[i][k];
  }
  const coefficients = solve(matrix, vector);
  if (!coefficients) return null;
  const avg = mean(points.map(value))!;
  let residual = 0, total = 0;
  for (let i = 0; i < points.length; i++) {
    residual += (value(points[i]) - rows[i].reduce((sum, x, j) => sum + x * coefficients[j], 0)) ** 2;
    total += (value(points[i]) - avg) ** 2;
  }
  return { amplitude: Math.hypot(coefficients[0], coefficients[1]), phase: Math.atan2(coefficients[1], coefficients[0]), r2: total > 1e-12 ? 1 - residual / total : 0 };
}

function frequencyAnalysis(points: Point[], config: RunConfig | null, commandHz: number | null): El05ExperimentAnalysis["frequency_response"] {
  const result: El05ExperimentAnalysis["frequency_response"] = { points: [], bandwidth_hz: null, reason: null };
  if (!config || config.kind as string !== "SineSweep") return { ...result, reason: "仅扫频工况计算频响。" };
  const f0 = config.frequency_start_hz ?? 0.1, f1 = config.frequency_end_hz ?? 2;
  if (!(f0 > 0 && f1 > f0 && config.duration_s > 0)) return { ...result, reason: "需有效的递增扫频范围。" };
  const slope = (f1 - f0) / config.duration_s;
  const phase = (t: number) => 2 * Math.PI * (f0 * t + 0.5 * slope * t * t);
  const usable = points.filter(p => p.d.target_position_rad !== null);
  if (usable.length < 30) return { ...result, reason: "实际位置命令覆盖不足。" };
  // Enforce >= 10 actual commands/cycle independently of feedback cadence.
  if (commandHz == null) return { ...result, reason: "缺少可核对的指令时序，不能从反馈采样率推测控制更新率。" };
  const maximumFrequency = Math.min(f1, commandHz / 10);
  const binCount = Math.max(12, Math.ceil(Math.log(f1 / f0) / Math.log(1.35)));
  const ratio = Math.pow(f1 / f0, 1 / binCount);
  for (let bin = 0; bin < binCount; bin++) {
    const low = f0 * ratio ** bin, high = f0 * ratio ** (bin + 1);
    const center = Math.sqrt(low * high);
    if (center > maximumFrequency) continue;
    const window = usable.filter(p => { const f = f0 + slope * p.s.t_s; return f >= low && f <= high; });
    if (window.length < 24) continue;
    const start = window[0].s.t_s, end = window[window.length - 1].s.t_s;
    const cycles = (phase(end) - phase(start)) / (2 * Math.PI);
    const gaps = window.slice(1).map((p, i) => p.s.t_s - window[i].s.t_s);
    if (cycles < 3 || (maximum(gaps) ?? Infinity) > Math.min(0.2, 0.25 / high) || window.length / cycles < 12) continue;
    const input = harmonicFit(window, phase, p => p.d.target_position_rad!);
    const output = harmonicFit(window, phase, p => p.s.unwrapped_position_rad);
    if (!input || !output || input.amplitude < 0.005 || input.r2 < 0.8 || output.r2 < 0.5) continue;
    const gain = output.amplitude / input.amplitude;
    let degrees = (output.phase - input.phase) * 180 / Math.PI;
    while (degrees > 180) degrees -= 360;
    while (degrees < -180) degrees += 360;
    result.points.push({ frequency_hz: f0 + slope * ((start + end) / 2), gain, gain_db: 20 * Math.log10(gain), phase_deg: degrees,
      input_amplitude_rad: input.amplitude, output_amplitude_rad: output.amplitude, cycles, sample_count: window.length });
  }
  const first = result.points[0];
  if (!first) result.reason = "没有覆盖至少三个周期且命令、采样密度充分的频段；请延长扫频时长或缩小频率范围。";
  else if (first.frequency_hz > f0 * 1.5 || first.gain < 0.7 || first.gain > 1.3) {
    result.reason = "缺少可用低频基准，暂不报告闭环 −3 dB 带宽。";
  } else {
    const threshold = first.gain_db - 3.01029995664;
    for (let i = 1; i < result.points.length; i++) {
      const a = result.points[i - 1], b = result.points[i];
      if (a.gain_db > threshold && b.gain_db <= threshold) {
        const mix = (threshold - a.gain_db) / (b.gain_db - a.gain_db);
        result.bandwidth_hz = Math.exp(Math.log(a.frequency_hz) + mix * Math.log(b.frequency_hz / a.frequency_hz));
        break;
      }
    }
    if (result.bandwidth_hz === null) result.reason = "实测频段未跨过相对低频增益的 −3 dB 点，不能给出带宽数值。";
  }
  return result;
}

function impedanceAnalysis(points: Point[], config: RunConfig | null): El05ExperimentAnalysis["impedance"] {
  const result: El05ExperimentAnalysis["impedance"] = { responses: [], equivalent_stiffness_nm_rad: null,
    command_stiffness_nm_rad: null, command_stiffness_reason: null,
    reason: "注入转矩为控制器内部前馈激励，不能当作已知外力；实际抗扰刚度需外部转矩或力传感依据。" };
  if (!config || config.kind as string !== "Impedance") return result;
  const injected = (p: Point) => p.s.target?.kind === "Mit"
    ? p.s.target.torque_nm - (config.gravity_compensation ? p.d.gravity_torque_nm ?? 0 : 0) : 0;
  const threshold = Math.max(0.01, Math.abs(config.disturbance_torque_nm ?? 0.15) * 0.4);
  const pulses: { start: number; end: number; sign: number; ended: boolean }[] = [];
  let inPulse = false;
  for (const p of points) {
    const torque = injected(p), active = Math.abs(torque) >= threshold;
    if (active && !inPulse) pulses.push({ start: p.s.t_s, end: p.s.t_s, sign: Math.sign(torque), ended: false });
    if (inPulse && !active) { pulses[pulses.length - 1].end = p.s.t_s; pulses[pulses.length - 1].ended = true; }
    else if (active) pulses[pulses.length - 1].end = p.s.t_s;
    inPulse = active;
  }
  for (let i = 0; i < pulses.length; i++) {
    const pulse = pulses[i];
    const baseline = mean(points.filter(p => p.s.t_s >= pulse.start - 0.3 && p.s.t_s < pulse.start).map(p => p.s.unwrapped_position_rad));
    if (baseline === null) continue;
    const until = pulses[i + 1]?.start ?? points[points.length - 1]?.s.t_s ?? pulse.end;
    const response = points.filter(p => p.s.t_s >= pulse.start && p.s.t_s < until);
    const peak = maximum(response.map(p => Math.abs(p.s.unwrapped_position_rad - baseline)));
    const tolerance = Math.max(0.005, (peak ?? 0) * 0.05);
    const after = response.filter(p => p.s.t_s >= pulse.end);
    let lastOutside = -1;
    after.forEach((p, i) => { if (Math.abs(p.s.unwrapped_position_rad - baseline) > tolerance) lastOutside = i; });
    const settled = after[lastOutside + 1]?.s.t_s;
    const recovery = pulse.ended && settled !== undefined && (after[after.length - 1]?.s.t_s ?? 0) - settled >= 0.15 ? settled - pulse.end : null;
    const stable = (p: Point) => p.d.velocity_rad_s !== null && Math.abs(p.d.velocity_rad_s) < 0.02
      && p.d.acceleration_rad_s2 !== null && Math.abs(p.d.acceleration_rad_s2) < 0.15;
    const before = points.filter(p => p.s.t_s >= pulse.start - 0.3 && p.s.t_s < pulse.start);
    const plateau = response.filter(p => p.s.t_s >= pulse.start + (pulse.end - pulse.start) * 0.65 && p.s.t_s < pulse.end);
    // A compliance ratio is meaningful only after both sides have settled.
    // It stays in nominal command-torque units, not an independent force calibration.
    const enough = (a: Point[]) => a.length >= 4 && a[a.length - 1].s.t_s - a[0].s.t_s >= 0.1
      && a.every(stable) && (deviation(a.map(p => p.s.unwrapped_position_rad)) ?? Infinity) < 0.002;
    let displacement: number | null = null, injection: number | null = null, stiffness: number | null = null;
    if (pulse.ended && enough(before) && enough(plateau)) {
      displacement = mean(plateau.map(p => p.s.unwrapped_position_rad))! - mean(before.map(p => p.s.unwrapped_position_rad))!;
      injection = mean(plateau.map(injected))! - mean(before.map(injected))!;
      if (Math.abs(displacement) >= 0.003 && injection * displacement > 0) stiffness = injection / displacement;
    }
    result.responses.push({ start_s: pulse.start, end_s: pulse.end, peak_displacement_rad: peak, recovery_s: recovery,
      steady_displacement_rad: displacement, injection_torque_nm: injection, command_stiffness_nm_rad: stiffness });
  }
  result.command_stiffness_nm_rad = median(present(result.responses.map(p => p.command_stiffness_nm_rad)));
  if (result.command_stiffness_nm_rad === null) result.command_stiffness_reason = "需注入前及脉冲末段均静止至少 100 ms，稳定角偏移至少 0.003 rad，且完整记录脉冲撤去。可延长工况时长。";
  return result;
}

function thermalAnalysis(points: Point[]): El05ExperimentAnalysis["thermal"] {
  const duration = points.length ? points[points.length - 1].s.t_s - points[0].s.t_s : 0;
  const margin = duration * 0.1;
  const first = points.filter(p => p.s.t_s <= (points[0]?.s.t_s ?? 0) + margin);
  const last = points.filter(p => p.s.t_s >= (points[points.length - 1]?.s.t_s ?? 0) - margin);
  const start = mean(first.map(p => p.s.temperature_c).filter(finite)), end = mean(last.map(p => p.s.temperature_c).filter(finite));
  const temperatures = points.filter(p => finite(p.s.temperature_c));
  const fit = regression(temperatures.map(p => p.s.t_s), temperatures.map(p => p.s.temperature_c));
  const initialPosition = mean(first.map(p => p.s.unwrapped_position_rad)), finalPosition = mean(last.map(p => p.s.unwrapped_position_rad));
  return { start_temperature_c: start, end_temperature_c: end, temperature_rise_c: start !== null && end !== null ? end - start : null,
    temperature_slope_c_min: duration >= 10 && fit ? fit.slope * 60 : null,
    position_drift_rad: duration >= 10 && initialPosition !== null && finalPosition !== null ? finalPosition - initialPosition : null,
    fresh_current_rms_a: rms(present(points.map(p => p.d.fresh_current_a))), reason: duration < 10 ? "记录不足 10 秒，温升斜率和位置漂移暂不报告。" : null };
}

function motionAnalysis(points: Point[], config: RunConfig | null): El05ExperimentAnalysis["motion"] {
  const positive: number[] = [], negative: number[] = [], ripple: number[] = [], steadySpeeds: number[] = [];
  for (const segment of targetSegments(points, "velocity")) {
    if (segment.end - segment.start < 0.4 || Math.abs(segment.target) < 0.01) continue;
    const speed = present(segment.points.filter(p => p.s.t_s > segment.start + (segment.end - segment.start) * 0.4).map(p => p.d.velocity_rad_s));
    const avg = mean(speed), std = deviation(speed);
    if (avg === null || speed.length < 5) continue;
    for (const v of speed) {
      (segment.target > 0 ? positive : negative).push(v);
      ripple.push(v - avg);
    }
    if (Math.abs(avg) > 0.01 && std !== null && std / Math.abs(avg) < 0.2) steadySpeeds.push(Math.abs(avg));
  }
  let reversals = 0, lastDirection = 0;
  for (const p of points) {
    if (p.d.velocity_rad_s === null || Math.abs(p.d.velocity_rad_s) < 0.03) continue;
    const direction = Math.sign(p.d.velocity_rad_s);
    if (lastDirection && direction !== lastDirection) reversals++;
    lastDirection = direction;
  }
  let onset: Point | null = null;
  const initiallyStationary = points.length > 0 && Math.abs(points[0].d.velocity_rad_s ?? Infinity) < 0.03;
  if (config?.kind as string === "TorqueRamp" && initiallyStationary) {
    let movingSince: number | null = null, candidate: Point | null = null;
    for (const p of points) {
      if (Math.abs(p.d.velocity_rad_s ?? 0) > 0.03) {
        if (movingSince === null) { movingSince = p.s.t_s; candidate = p; }
        if (p.s.t_s - movingSince >= 0.1) { onset = candidate; break; }
      } else { movingSince = null; candidate = null; }
    }
  }
  return { speed_ripple_rad_s: rms(ripple), positive_speed_rad_s: mean(positive), negative_speed_rad_s: mean(negative),
    minimum_observed_steady_speed_rad_s: minimum(steadySpeeds), reversal_count: reversals,
    breakaway_time_s: onset?.s.t_s ?? null, breakaway_motor_torque_nm: onset?.s.torque_nm ?? null,
    breakaway_gravity_adjusted_torque_nm: onset?.d.gravity_torque_nm !== null && onset ? onset.s.torque_nm - onset.d.gravity_torque_nm! : null,
    reason: config?.kind as string === "TorqueRamp" && !onset ? initiallyStationary ? "未观测到持续 100 ms 的起动。" : "采集开始时已在运动，不能识别起动转矩。" : null };
}

/** Chronological passes, including the current one; legacy runs contain only one. */
export function staticFrictionPasses(data: El05ExperimentData): El05StaticFrictionPass[] {
  const state = data.status.static_friction;
  return state ? [...(state.completed_directions ?? []), state] : [];
}

/** Use captured event frames, even when the plot samples have been thinned. */
function staticFrictionPassAnalysis(state: El05StaticFrictionPass | undefined, config: RunConfig | null, active: boolean, data: El05ExperimentData): El05StaticFrictionPassAnalysis {
  const onset = state?.onset_sample, confirmation = state?.confirmation_sample;
  const reference = finite(state?.reference_position_rad) ? state.reference_position_rad : null;
  const driveEvidenceValid = !!onset && !!confirmation && controlFeedbackEligible(onset, data) && controlFeedbackEligible(confirmation, data)
    && onset.fault_bits === 0 && confirmation.fault_bits === 0;
  const valid = Boolean(state?.outcome === "BreakawayDetected" && onset && confirmation && reference !== null
    && driveEvidenceValid
    && finite(onset.t_s) && finite(confirmation.t_s) && confirmation.t_s >= onset.t_s
    && finite(onset.unwrapped_position_rad) && finite(onset.torque_nm));
  const acceptedOnset = valid ? onset : null, acceptedConfirmation = valid ? confirmation : null;
  const stationary = valid ? state?.last_stationary_sample : null;
  const commandTorque = (s: El05TelemetrySample | null | undefined) => s?.target?.kind === "Mit"
    && (s.target_age_ms === null || s.target_age_ms <= 250) && finite(s.target.torque_nm) ? s.target.torque_nm : null;
  const feedbackTorque = (s: El05TelemetrySample | null | undefined) => finite(s?.torque_nm) ? s.torque_nm : null;
  const onsetCommand = commandTorque(acceptedOnset), confirmationCommand = commandTorque(acceptedConfirmation);
  const pendulum = config?.pendulum;
  // The captured rest position is a displacement reference, not proof of the gravitational down direction.
  const gravity = acceptedOnset && pendulum?.zero_measured
    ? pendulumModel(pendulum.mass_kg, pendulum.length_m, pendulum.arm_mass_kg).gravityNm
      * Math.sin(acceptedOnset.unwrapped_position_rad - pendulum.downward_zero_rad) : null;
  const reasons: Record<string, string> = {
    AngleLimit: "达到允许角偏移，未得到有效静摩擦标定。",
    TorqueLimit: "达到最大施加转矩仍未确认起动，未得到有效静摩擦标定。",
    Timeout: "达到试验时限，未得到有效静摩擦标定。",
    NotSettled: "起始摆锤未保持静止，未进行有效静摩擦标定。",
    Interrupted: "试验中断，未得到有效静摩擦标定。",
  };
  return {
    direction: state?.direction ?? (config?.torque_nm != null && config.torque_nm < 0 ? -1 : 1),
    phase: state?.phase ?? null, outcome: state?.outcome ?? null, valid_breakaway: valid,
    return_settled: typeof state?.return_settled === "boolean" ? state.return_settled : null,
    reference_position_rad: reference,
    last_stationary_command_torque_nm: commandTorque(stationary),
    last_stationary_feedback_torque_nm: feedbackTorque(stationary),
    onset_command_torque_nm: onsetCommand, onset_feedback_torque_nm: feedbackTorque(acceptedOnset),
    confirmation_command_torque_nm: confirmationCommand, confirmation_feedback_torque_nm: feedbackTorque(acceptedConfirmation),
    onset_gravity_torque_nm: gravity,
    onset_gravity_adjusted_torque_nm: acceptedOnset && gravity !== null ? acceptedOnset.torque_nm - gravity : null,
    detection_delay_s: acceptedOnset && acceptedConfirmation ? acceptedConfirmation.t_s - acceptedOnset.t_s : null,
    command_increase_during_detection_nm: onsetCommand !== null && confirmationCommand !== null ? confirmationCommand - onsetCommand : null,
    onset_angle_rad: acceptedOnset && reference !== null ? acceptedOnset.unwrapped_position_rad - reference : null,
    peak_excursion_rad: finite(state?.peak_excursion_rad) ? state.peak_excursion_rad : null,
    max_excursion_rad: finite(config?.max_excursion_rad) ? config.max_excursion_rad : null,
    ramp_started_s: finite(state?.ramp_started_s) ? state.ramp_started_s : null,
    onset_time_s: acceptedOnset?.t_s ?? null, confirmation_time_s: acceptedConfirmation?.t_s ?? null,
    ramp_rate_nm_s: finite(config?.torque_ramp_rate_nm_s) ? Math.abs(config.torque_ramp_rate_nm_s) : null,
    reason: valid ? null : state?.outcome === "BreakawayDetected" && onset && confirmation && !driveEvidenceValid
      ? data.protocol === "StandardMit" ? "起动证据缺少新鲜 MIT 控制指令或包含故障，未得到有效静摩擦标定。" : "起动证据包含驱动非运行状态或故障，未得到有效静摩擦标定。"
      : state?.outcome ? reasons[state.outcome] ?? "起动事件记录不完整，未得到有效静摩擦标定。"
      : active ? "标定正在运行，等待起动确认。" : "缺少后端起动事件记录，未得到有效静摩擦标定。",
  };
}

function staticFrictionAnalysis(data: El05ExperimentData, config: RunConfig | null): El05StaticFrictionAnalysis | undefined {
  if (config?.kind as string !== "StaticFriction") return undefined;
  const states = staticFrictionPasses(data);
  const directions = (states.length ? states : [undefined]).map(state => staticFrictionPassAnalysis(state, config, data.status.active, data));
  const positive = directions.find(pass => pass.direction === 1 && pass.valid_breakaway);
  const negative = directions.find(pass => pass.direction === -1 && pass.valid_breakaway);
  const magnitudeDifference = (a: number | null | undefined, b: number | null | undefined) => a != null && b != null ? Math.abs(a) - Math.abs(b) : null;
  return { ...directions[directions.length - 1], directions, both_directions_valid: !!positive && !!negative,
    feedback_magnitude_difference_nm: magnitudeDifference(positive?.onset_feedback_torque_nm, negative?.onset_feedback_torque_nm),
    adjusted_magnitude_difference_nm: magnitudeDifference(positive?.onset_gravity_adjusted_torque_nm, negative?.onset_gravity_adjusted_torque_nm) };
}

function metric(key: string, label: string, label_en: string, value: number | null, unit: string, note?: string): El05AnalysisMetric {
  return { key, label, label_en, value, unit, note, digits: 4 };
}

/** Pure analysis of recorded evidence. Missing or unidentifiable results stay null. */
export function analyzeExperiment(data: El05ExperimentData): El05ExperimentAnalysis {
  const derived = deriveExperimentSamples(data), indices = orderedIndices(data.samples);
  const points = indices.map(i => ({ s: data.samples[i], d: derived[i] }));
  const config = data.status.config as RunConfig | null, kind = config?.kind as string ?? "Record";
  const duration = points.length ? points[points.length - 1].s.t_s - points[0].s.t_s : 0;
  const intervals = points.slice(1).map((p, i) => p.s.t_s - points[i].s.t_s);
  const interval = median(intervals), maxGap = maximum(intervals), commandHz = observedCommandRateHz(data);
  const qualityNotes: string[] = [];
  if (data.status.samples > data.samples.length) qualityNotes.push("当前数据包含抽样或增量片段；指标仅基于已载入样本，历史原始帧数多于当前样本。");
  if (maxGap !== null && interval !== null && maxGap > Math.max(interval * 5, 0.1)) qualityNotes.push("存在明显采样间断；缺口附近的导数和动态响应需谨慎解读。");
  if (data.samples.length !== indices.length) qualityNotes.push("重复、同时间戳或无效反馈已从分析中剔除，导出仍保留原始行。");
  if (data.status.active) qualityNotes.push("实验仍在运行，当前为阶段性分析。");
  else if (config && kind !== "StaticFriction" && (points[points.length - 1]?.s.t_s ?? 0) < config.duration_s * 0.95) qualityNotes.push("实际记录未覆盖计划全程，仅分析已经完成的激励和响应。");
  const freshCount = points.filter(p => p.d.fresh_current_a !== null).length;
  const standardMit = data.protocol === "StandardMit";
  const inactiveCount = standardMit ? null : points.filter(p => p.s.status_code !== 2).length;
  if (kind !== "Record" && inactiveCount) qualityNotes.push(`驱动非运行状态 ${inactiveCount} 帧（${(inactiveCount / Math.max(1, points.length) * 100).toFixed(1)}%）；这些帧不参与控制性能分析。`);
  if (standardMit) qualityNotes.push(`标准 MIT 状态位不用于判断运行；性能分析使用无故障且具有新鲜 MIT 指令的反馈。速度量化步长为 ${feedbackVelocityResolution(data).toFixed(5)} rad/s（${feedbackVelocityBits(data)} 位）。`);
  const quality = { sample_rate_hz: interval !== null && interval > 0 ? 1 / interval : null, command_rate_hz: commandHz, median_interval_s: interval,
    max_gap_s: maxGap, duplicate_samples: data.samples.length - indices.length, dropped_frames: data.status.dropped_frames,
    fresh_current_samples: freshCount, inactive_drive_samples: inactiveCount,
    inactive_drive_fraction: points.length && inactiveCount !== null ? inactiveCount / points.length : null, notes: qualityNotes };
  const performancePoints = kind === "Record" ? points : points.filter(p => controlFeedbackEligible(p.s, data));
  const positionErrors = present(performancePoints.map(p => p.d.position_error_rad));
  const velocityErrors = present(performancePoints.map(p => p.d.velocity_error_rad_s));
  const positionSteps = kind === "PositionSteps" ? stepResponses(performancePoints, "position") : [];
  const velocitySteps = kind === "VelocitySteps" ? stepResponses(performancePoints, "velocity") : [];
  const tracking = { position_rms_rad: rms(positionErrors), velocity_rms_rad_s: rms(velocityErrors),
    position_steady_error_rad: mean(present(positionSteps.map(s => s.steady_error))),
    velocity_steady_error_rad_s: mean(present(velocitySteps.map(s => s.steady_error))),
    position_steps: positionSteps, velocity_steps: velocitySteps };
  const staticTorque = kind === "StaticTorque" ? staticAnalysis(performancePoints, config) : { platforms: [], torque_constant_nm_per_a: null,
    intercept_nm: null, r_squared: null, forward_reverse_torque_difference_nm: null, reason: null };
  const staticHoldingPeak = staticHoldingTorquePeakNm(config), torqueLimit = experimentTorqueLimitNm(data);
  const staticTorqueRangeNotes = staticHoldingPeak !== null && staticHoldingPeak > torqueLimit
    ? [`目标角最大理论保持力矩 ${staticHoldingPeak.toFixed(3)} Nm 超过本组驱动限值 ${torqueLimit.toFixed(3)} Nm（EL05 峰值 / 协议量程上限 6 Nm）；部分目标角无法仅靠该力矩保持。理论与反馈均为 Nm，理论需求按实际负载保留。`]
    : [];
  const staticFriction = staticFrictionAnalysis(data, config);
  const friction = fitFriction(performancePoints, Boolean(config?.pendulum?.zero_measured));
  const freeDecay = kind === "Record" || kind === "Backdrive" ? decayAnalysis(performancePoints, config)
    : { period_s: null, frequency_hz: null, log_decrement: null, damping_ratio: null, equivalent_viscous_nm_s_rad: null,
      peak_count: 0, envelope_r_squared: null, reason: "仅自由记录或反驱摆动工况分析自由衰减。" };
  const frequencyResponse = frequencyAnalysis(performancePoints, config, commandHz), impedance = impedanceAnalysis(performancePoints, config);
  const thermal = thermalAnalysis(performancePoints), motion = motionAnalysis(performancePoints, config);
  const limitations = [
    "转矩为驱动器反馈估计值，未经外部转矩传感器验证。Iq 回归给出输出端有效 Nm/A，不直接等同手册的 0.94 Nm/Arms。",
    "摩擦、阻尼和惯量反映电机与摆臂、轴承及负载组成的整体；负载模型不包含未知电机/减速器反射惯量，应填入附加惯量。",
    `时间为主机单调 CAN 接收时间，非驱动器采样时钟；动态参数受通讯、量化和实际命令更新率影响。${commandHz == null ? "本组缺少可核对的指令时序。" : `本组实测命令更新率约 ${commandHz.toFixed(1)} Hz。`}`,
  ];
  if (standardMit) limitations.push("标准 MIT 未提供 Iq、电压和字典速度；相关指标保留为空，电流回归不可用。原始状态位保留，不构成硬件使能证明。");
  if (!config?.pendulum?.zero_measured) limitations.push("下垂参考未实测：重力矩、真实摆角、静态转矩标定与扣除重力的摩擦参数不输出。");
  const sections: El05AnalysisSection[] = [{ id: "quality", title: "采样质量", title_en: "Sampling quality", metrics: [
    metric("sample_count", "有效反馈样本", "Valid feedback samples", points.length, ""),
    metric("sample_rate_hz", "反馈采样率", "Feedback sample rate", quality.sample_rate_hz, "Hz"),
    metric("command_rate_hz", "实测命令更新率", "Measured command update rate", commandHz, "Hz"),
    metric("max_gap_s", "最大采样间隔", "Largest sample gap", maxGap, "s"),
    metric("fresh_current_samples", "独立新鲜 Iq 样本", "Independent fresh Iq samples", freshCount, ""),
    metric("dropped_frames", "接收丢帧", "Dropped receive frames", data.status.dropped_frames, ""),
  ], notes: qualityNotes }];
  if (kind === "StaticTorque") sections.push({ id: "static", title: "静态转矩与电流", title_en: "Static torque and current", metrics: [
    metric("stable_platforms", "有效稳定平台", "Stable plateaus", staticTorque.platforms.length, ""),
    metric("target_holding_peak", "目标角最大理论保持力矩", "Peak target holding torque", staticHoldingPeak, EL05_TORQUE_UNIT),
    metric("drive_torque_limit", "本组驱动力矩上限", "Run drive torque limit", torqueLimit, EL05_TORQUE_UNIT,
      "优先使用开始时读取的 0x700B；读数不可用时显示 EL05 的 6 Nm 峰值 / 协议量程上限。"),
    metric("effective_torque_constant", "重力负载 / Iq 回归斜率", "Gravity load / Iq slope", staticTorque.torque_constant_nm_per_a, `${EL05_TORQUE_UNIT}/A`),
    metric("torque_intercept", "转矩回归截距", "Torque regression intercept", staticTorque.intercept_nm, EL05_TORQUE_UNIT),
    metric("torque_r2", "转矩回归 R²", "Torque regression R²", staticTorque.r_squared, ""),
    metric("approach_difference", "正反接近的估计转矩差", "Approach direction torque difference", staticTorque.forward_reverse_torque_difference_nm, EL05_TORQUE_UNIT),
  ], notes: [...staticTorqueRangeNotes, ...(staticTorque.reason ? [staticTorque.reason] : []), "仅取各平台末 40% 且速度与加速度接近零的样本。重力负载回归受静摩擦影响；正反接近差为驱动估计转矩差，非独立实测回差。"] });
  if (staticFriction) {
    if (config?.friction_direction === "Both" || staticFriction.directions.length > 1) sections.push({
      id: "static_friction_summary", title: "双向静摩擦对比", title_en: "Bidirectional static friction comparison", metrics: [
        metric("static_friction_both_valid", "双向均取得有效起动", "Valid breakaway in both directions", staticFriction.both_directions_valid ? 1 : 0, ""),
        metric("static_friction_valid_directions", "有效起动方向数", "Directions with valid breakaway", staticFriction.directions.filter(pass => pass.valid_breakaway).length, "/ 2"),
        metric("static_friction_feedback_difference", "起动反馈转矩幅值差（正向 − 反向）", "Breakaway feedback magnitude difference (positive − negative)", staticFriction.feedback_magnitude_difference_nm, "Nm"),
        metric("static_friction_adjusted_difference", "扣除重力后幅值差（正向 − 反向）", "Gravity-adjusted magnitude difference (positive − negative)", staticFriction.adjusted_magnitude_difference_nm, "Nm"),
      ], notes: ["各方向独立保存起动事件；只有两边均有效时才计算幅值差。回位未稳定或角度保护触发时停止后续方向。"] });
    for (const sf of staticFriction.directions) {
      const degrees = (angle: number | null) => angle === null ? null : angle * 180 / Math.PI;
      const outcomes: Record<string, string> = { BreakawayDetected: "已确认起动", AngleLimit: "达到角偏移上限",
        TorqueLimit: "达到最大施加转矩", Timeout: "达到试验时限", NotSettled: "起始未静止", Interrupted: "试验中断" };
      sections.push({ id: staticFriction.directions.length > 1 ? `static_friction_${sf.direction > 0 ? "positive" : "negative"}` : "static_friction",
        title: `${sf.direction > 0 ? "正向 (+)" : "反向 (−)"} 静摩擦标定`, title_en: `${sf.direction > 0 ? "Positive (+)" : "Negative (−)"} static friction calibration`, metrics: [
        metric("static_friction_valid", "有效起动标志", "Valid breakaway event", sf.valid_breakaway ? 1 : 0, "", `结局：${sf.outcome ? outcomes[sf.outcome] ?? sf.outcome : data.status.active ? "进行中" : "无有效事件"}。`),
        metric("static_friction_return_settled", "回位稳定标志", "Stable return confirmed", sf.return_settled === null ? null : sf.return_settled ? 1 : 0, ""),
        metric("static_friction_stationary_command", "最后静止帧指令转矩", "Last stationary commanded torque", sf.last_stationary_command_torque_nm, "Nm"),
        metric("static_friction_stationary_feedback", "最后静止帧反馈转矩", "Last stationary feedback torque", sf.last_stationary_feedback_torque_nm, "Nm"),
        metric("static_friction_onset_command", "起动首帧指令转矩", "First motion frame commanded torque", sf.onset_command_torque_nm, "Nm"),
        metric("static_friction_onset_feedback", "起动首帧估计转矩", "First motion frame estimated torque", sf.onset_feedback_torque_nm, "Nm"),
        metric("static_friction_confirmation_command", "确认帧指令转矩", "Confirmation frame commanded torque", sf.confirmation_command_torque_nm, "Nm"),
        metric("static_friction_confirmation_feedback", "确认帧反馈转矩", "Confirmation frame feedback torque", sf.confirmation_feedback_torque_nm, "Nm"),
        metric("static_friction_gravity", "起动首帧重力矩", "Gravity torque at first motion frame", sf.onset_gravity_torque_nm, "Nm"),
        metric("static_friction_adjusted", "扣除重力的起动转矩估计", "Gravity-adjusted breakaway torque estimate", sf.onset_gravity_adjusted_torque_nm, "Nm", "反馈转矩 − G sinθ；仍含首帧起动惯性及驱动估计误差。"),
        metric("static_friction_detection_delay", "起动到确认的时间", "Motion-to-confirmation delay", sf.detection_delay_s === null ? null : sf.detection_delay_s * 1000, "ms"),
        metric("static_friction_detection_command_increase", "确认期间指令转矩增量", "Signed commanded torque increase during confirmation", sf.command_increase_during_detection_nm, "Nm"),
        metric("static_friction_onset_angle", "起动首帧相对起点角度", "First motion displacement from rest", degrees(sf.onset_angle_rad), "°"),
        metric("static_friction_peak_angle", "实测最大角偏移", "Observed peak displacement", degrees(sf.peak_excursion_rad), "°"),
        metric("static_friction_max_angle", "允许角偏移上限", "Configured displacement limit", degrees(sf.max_excursion_rad), "°"),
        metric("static_friction_ramp_start", "转矩斜坡开始时刻", "Torque ramp start time", sf.ramp_started_s, "s"),
        metric("static_friction_onset_time", "起动首帧时刻", "First motion frame time", sf.onset_time_s, "s"),
        metric("static_friction_ramp_duration", "从斜坡开始到起动", "Ramp-to-motion duration", sf.onset_time_s !== null && sf.ramp_started_s !== null ? sf.onset_time_s - sf.ramp_started_s : null, "s"),
        metric("static_friction_ramp_rate", "施加转矩斜坡速率", "Applied torque ramp rate", sf.ramp_rate_nm_s, "Nm/s"),
      ], notes: [
        ...(sf.reason ? [sf.reason] : []),
        ...(sf.return_settled === false ? ["回位在 3 秒内未稳定，已停止驱动；已确认的起动转矩结果仍保留。"] : []),
        "采用后端保存的最后静止、首次越过位移阈值和持续确认事件帧；确认后撤去斜坡并回落，不把确认帧转矩冒充首次起动转矩。",
        "最后静止与起动首帧的指令值仅表示本次名义指令变化，不能作为真实静摩擦力矩的严格上下界；运动首帧仍含惯性。",
        "标定结果为电机与摆臂机构的起动转矩估计，单位 Nm。未知接触法向载荷及接触半径时不计算无量纲摩擦系数 μ。",
        "角偏移以本次捕获的静止位置为基准；该位置不能替代独立测定的重力下垂零点。",
        ...(!config?.pendulum?.zero_measured ? ["未实测重力下垂零点：仅报告原始起动转矩估计，重力修正留空。"] : []),
      ] });
    }
  }
  if (positionErrors.length || velocityErrors.length) {
    const steps = positionSteps.length ? positionSteps : velocitySteps;
    sections.push({ id: "tracking", title: "带载跟踪与阶跃响应", title_en: "Loaded tracking and step response", metrics: [
      metric("position_rms", "位置误差 RMS", "Position error RMS", tracking.position_rms_rad, "rad"),
      metric("velocity_rms", "速度误差 RMS", "Velocity error RMS", tracking.velocity_rms_rad_s, "rad/s"),
      metric("steady_position_error", "阶跃稳态位置误差", "Steady position error", tracking.position_steady_error_rad, "rad"),
      metric("steady_velocity_error", "阶跃稳态速度误差", "Steady velocity error", tracking.velocity_steady_error_rad_s, "rad/s"),
      metric("rise_time", "阶跃上升时间（中位数）", "Median step rise time", median(present(steps.map(s => s.rise_time_s))), "s"),
      metric("overshoot", "最大阶跃超调", "Maximum step overshoot", maximum(present(steps.map(s => s.overshoot_percent))), "%"),
      metric("settling_time", "阶跃稳定时间（中位数）", "Median step settling time", median(present(steps.map(s => s.settling_time_s))), "s"),
    ], notes: ["参考使用实际已发送 MIT 目标；上升时间为 10–90%，稳定带为阶跃幅度 2%（位置至少 0.005 rad、速度至少 0.01 rad/s），需持续稳定至少 150 ms。未达到或未观测到稳定时留空。"] });
  }
  if (["Record", "Backdrive", "SlowTriangle", "SineSweep", "VelocitySteps", "TorqueRamp"].includes(kind)) sections.push({ id: "friction", title: "综合机械摩擦辨识", title_en: "Combined mechanical friction", metrics: [
    metric("coulomb", "库仑摩擦", "Coulomb friction", friction.coulomb_nm, "Nm"),
    metric("viscous", "粘性阻尼", "Viscous damping", friction.viscous_nm_s_rad, "Nm·s/rad"),
    metric("friction_r2", "摩擦模型 R²", "Friction model R²", friction.r_squared, ""),
    metric("friction_rmse", "摩擦模型残差 RMS", "Friction residual RMS", friction.rmse_nm, "Nm"),
  ], notes: [...(friction.reason ? [friction.reason] : []), "拟合 τmotor − G sinθ − Jα = Fc·sign(ω) + B·ω，剔除接近零速区。加速度用约 100 ms 局部速度回归，非直接差分。"] });
  if (kind === "Record" || kind === "Backdrive") sections.push({ id: "decay", title: "自由摆动衰减", title_en: "Free oscillation decay", metrics: [
    metric("period", "摆动周期", "Oscillation period", freeDecay.period_s, "s"),
    metric("frequency", "摆动频率", "Oscillation frequency", freeDecay.frequency_hz, "Hz"),
    metric("log_decrement", "峰值包络对数衰减", "Envelope log decrement", freeDecay.log_decrement, ""),
    metric("damping_ratio", "等效阻尼比", "Equivalent damping ratio", freeDecay.damping_ratio, ""),
    metric("equivalent_viscous", "等效总粘性阻尼", "Equivalent total viscous damping", freeDecay.equivalent_viscous_nm_s_rad, "Nm·s/rad"),
    metric("envelope_r2", "指数包络 R²", "Exponential envelope R²", freeDecay.envelope_r_squared, ""),
  ], notes: [...(freeDecay.reason ? [freeDecay.reason] : []), "包络由相邻峰谷差计算，可消除固定角度零点偏置。阻尼比要求近似指数衰减；有 Kd 或其他控制时包含控制器的等效阻尼。"] });
  if (kind === "SineSweep") sections.push({ id: "frequency", title: "闭环扫频响应", title_en: "Closed-loop frequency response", metrics: [
    metric("frequency_points", "有效频响测点", "Valid frequency points", frequencyResponse.points.length, ""),
    metric("bandwidth", "实测 −3 dB 带宽", "Measured −3 dB bandwidth", frequencyResponse.bandwidth_hz, "Hz"),
  ], notes: [...(frequencyResponse.reason ? [frequencyResponse.reason] : []), "局部频段用实际目标与响应的谐波回归计算幅相；每段至少三个周期、12 个反馈/周期及 10 个命令/周期。包含控制器、负载与通讯延迟。"] });
  if (kind === "Impedance") sections.push({ id: "impedance", title: "刚度阻尼与注入响应", title_en: "Impedance injection response", metrics: [
    metric("disturbance_peak", "注入后的最大角偏移", "Peak injected-response displacement", maximum(present(impedance.responses.map(r => r.peak_displacement_rad))), "rad"),
    metric("recovery", "注入撤去后恢复时间", "Recovery after injection", median(present(impedance.responses.map(r => r.recovery_s))), "s"),
    metric("command_stiffness", "指令响应等效刚度", "Command-response stiffness", impedance.command_stiffness_nm_rad, "Nm/rad", impedance.command_stiffness_reason ?? "Δ前馈指令 / Δ稳定角偏移，含重力刚度与静摩擦影响；使用名义指令转矩单位。"),
    metric("stiffness", "外扰等效刚度", "External disturbance stiffness", impedance.equivalent_stiffness_nm_rad, "Nm/rad", impedance.reason),
  ], notes: [impedance.reason, "指令响应等效刚度仅在注入前和末段稳定时计算 ΔFF / Δ角度；包括控制刚度、重力和静摩擦，不能当作实测外力标定。", "恢复时间以注入前角度为基线，容差为峰偏移的 5% 且至少 0.005 rad；仅对已撤去且观测到恢复的注入给出结果。"] });
  if (["SlowTriangle", "Backdrive", "VelocitySteps", "TorqueRamp"].includes(kind)) sections.push({ id: "motion", title: "低速与反驱表现", title_en: "Low-speed and backdrive behavior", metrics: [
    metric("positive_speed", "正向稳定段平均速度", "Positive plateau speed", motion.positive_speed_rad_s, "rad/s"),
    metric("negative_speed", "反向稳定段平均速度", "Negative plateau speed", motion.negative_speed_rad_s, "rad/s"),
    metric("speed_ripple", "稳定段速度波动 RMS", "Plateau speed ripple RMS", motion.speed_ripple_rad_s, "rad/s"),
    metric("minimum_steady_speed", "本次观测最低平稳速度", "Lowest observed steady speed", motion.minimum_observed_steady_speed_rad_s, "rad/s", "仅代表本次已测试速度，非电机最低可控速度极限。"),
    metric("reversals", "有效换向次数", "Observed direction reversals", motion.reversal_count, ""),
    metric("breakaway_time", "起动时间", "Observed breakaway time", motion.breakaway_time_s, "s"),
    metric("breakaway_torque", "起动瞬间估计转矩", "Estimated torque at breakaway", motion.breakaway_motor_torque_nm, "Nm"),
    metric("breakaway_adjusted", "起动转矩扣除重力", "Gravity-adjusted breakaway torque", motion.breakaway_gravity_adjusted_torque_nm, "Nm", "尚含起动惯性影响，不直接视为静摩擦极限。"),
  ], notes: [...(motion.reason ? [motion.reason] : []), "换向忽略 ±0.03 rad/s 内的微动；低速波动仅取实际恒速目标后 60% 的段落。"] });
  if (kind === "ThermalHold") sections.push({ id: "thermal", title: "持续承载与温升", title_en: "Sustained load and heating", metrics: [
    metric("start_temperature", "开始温度", "Initial temperature", thermal.start_temperature_c, "°C"),
    metric("end_temperature", "结束温度", "Final temperature", thermal.end_temperature_c, "°C"),
    metric("temperature_rise", "温升", "Temperature rise", thermal.temperature_rise_c, "°C"),
    metric("temperature_slope", "温升线性斜率", "Temperature slope", thermal.temperature_slope_c_min, "°C/min"),
    metric("position_drift", "承载位置漂移", "Loaded position drift", thermal.position_drift_rad, "rad"),
    metric("current_rms", "独立 Iq 样本 RMS", "Independent Iq RMS", thermal.fresh_current_rms_a, "A"),
  ], notes: [...(thermal.reason ? [thermal.reason] : []), "开始/结束取首尾 10% 样本均值；温升斜率覆盖实际记录全程。无法仅凭单次温升判断持续额定转矩或热稳态。"] });
  return { kind, sample_count: points.length, duration_s: duration, quality, sections, limitations, tracking,
    static_torque: staticTorque, ...(staticFriction ? { static_friction: staticFriction } : {}), friction, free_decay: freeDecay, frequency_response: frequencyResponse, impedance, thermal, motion };
}

/** Export only metadata frozen by the backend at the start of this run. */
export function experimentJson(data: El05ExperimentData, includeDerivedSamples = true): string {
  const pendulum = data.status.config?.pendulum;
  return JSON.stringify({
    ...data,
    protocol: data.protocol ?? "Native",
    feedback_velocity_bits: feedbackVelocityBits(data),
    feedback_velocity_resolution_rad_s: feedbackVelocityResolution(data),
    pendulum: pendulum ? { ...pendulum, ...pendulumModel(pendulum.mass_kg, pendulum.length_m, pendulum.arm_mass_kg,
      (pendulum as typeof pendulum & { extra_inertia_kg_m2?: number }).extra_inertia_kg_m2 ?? 0) } : null,
    analysis: analyzeExperiment(data),
    ...(includeDerivedSamples ? { derived_samples: deriveExperimentSamples(data) } : {}),
    time_source: "host monotonic CAN receive time; no drive timestamp",
    torque_unit: EL05_TORQUE_UNIT,
    torque_feedback_range_nm: [-EL05_TORQUE_LIMIT_NM, EL05_TORQUE_LIMIT_NM],
    torque_source: "drive feedback estimate; not a torque sensor",
    pendulum_angle_definition: "theta_rad = unwrapped_position_rad - downward_zero_rad; use for gravity analysis only when zero_measured is true",
  }, null, 2);
}

export function experimentCsv(data: El05ExperimentData): string {
  const fields = ["t_s", "sequence", "position_rad", "unwrapped_position_rad", "velocity_rad_s", "position_velocity_rad_s",
    "torque_nm", "current_a", "current_age_ms", "dictionary_velocity_rad_s", "dictionary_velocity_age_ms",
    "temperature_c", "fault_bits", "status_code", "target_sequence", "target_age_ms"] as const;
  const derived = deriveExperimentSamples(data);
  const derivedFields = ["relative_position_rad", "angle_rad", "acceleration_rad_s2", "gravity_torque_nm", "target_holding_torque_nm", "inertial_torque_nm",
    "friction_torque_nm", "position_error_rad", "velocity_error_rad_s", "fresh_current_a"] as const;
  const headers = [...fields, "can_id_hex", "payload_hex", "target_position_rad", "target_velocity_rad_s", "target_torque_nm", "kp", "kd", "motor_id", "uid",
    "derived_velocity_rad_s", "target_unwrapped_position_rad", ...derivedFields, "static_friction_direction", "static_friction_phase", "protocol", "feedback_velocity_bits"];
  const passes = staticFrictionPasses(data).reverse();
  const rows = data.samples.map((sample, i) => {
    const target = sample.target?.kind === "Mit" ? sample.target : null;
    const pass = passes.find(pass => sample.t_s >= (pass.started_s ?? 0));
    const phase = !pass ? "" : pass.return_started_s != null && sample.t_s >= pass.return_started_s ? "Returning"
      : pass.ramp_started_s != null && sample.t_s >= pass.ramp_started_s ? "Ramping" : "Settling";
    return [...fields.map(field => sample[field] ?? ""), sample.can_id.toString(16).padStart(8, "0"), sample.payload.map(byte => byte.toString(16).padStart(2, "0")).join(" "), target?.position_rad ?? "", target?.velocity_rad_s ?? "",
      target?.torque_nm ?? "", target?.kp ?? "", target?.kd ?? "", data.motor_id ?? "", data.uid ?? "",
      derived[i].velocity_rad_s ?? "", derived[i].target_position_rad ?? "", ...derivedFields.map(field => derived[i][field] ?? ""),
      pass ? pass.direction ?? Math.sign(data.status.config?.torque_nm ?? 1) : "", phase, data.protocol ?? "Native", feedbackVelocityBits(data)].join(",");
  });
  return [headers.join(","), ...rows].join("\r\n") + "\r\n";
}
