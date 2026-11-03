import type { El05ExperimentData, El05TorqueAccuracyConfig, El05TorqueAccuracyHold } from "./el05Api";
import type { El05DerivedSample } from "./el05Experiment";
import { controlFeedbackEligible, feedbackVelocityResolution } from "./el05Protocol.ts";

export const TORQUE_ACCURACY_MAX_TEMPERATURE_C = 120;

export const DEFAULT_TORQUE_ACCURACY: El05TorqueAccuracyConfig = {
  reference_id: "", start_nm: 3.7, end_nm: 5.2, step_nm: 0.05, hold_s: 1,
  move_speed_rad_s: 0.3, direction: 1,
};

export interface TorqueAccuracyPoint extends El05TorqueAccuracyHold {
  sample_count: number;
  accepted: boolean;
  reason: string | null;
  angle_rad: number | null;
  theory_nm: number | null;
  feedback_nm: number | null;
  feedback_std_nm: number | null;
  error_nm: number | null;
  error_percent: number | null;
  /** Feedback/theory ratio. Keep the legacy export key; this is not a validated command correction. */
  send_scale: number | null;
  temperature_c: number | null;
}
export type TorqueAccuracyModel = "constant" | "linear" | "step" | "piecewise_linear" | "inconclusive";
export interface TorqueAccuracyAnalysis {
  points: TorqueAccuracyPoint[];
  model: TorqueAccuracyModel;
  boundary_nm: [number, number] | null;
  boundary_feedback_nm: [number, number] | null;
  low_scale: number | null;
  high_scale: number | null;
  slope_per_nm: number | null;
  intercept: number | null;
  hinge_nm: number | null;
  fit_rmse: number | null;
  paired_points: number;
  hysteresis_scale: number | null;
  candidates: { model: Exclude<TorqueAccuracyModel, "inconclusive">; bic: number; rmse: number }[];
  reason: string | null;
}

export const torqueAccuracyModelLabel = (model: TorqueAccuracyModel, zh: boolean) => ({
  constant: ["本区间反馈/理论比值近似恒定", "Approximately constant feedback/theory ratio in this range"],
  linear: ["反馈/理论比值随负载近似线性变化", "Feedback/theory ratio varies approximately linearly with load"],
  step: ["疑似反馈/理论比值跃迁", "Possible step in feedback/theory ratio"],
  piecewise_linear: ["疑似边界后线性变化", "Possible linear change above a boundary"],
  inconclusive: ["证据不足，尚不能区分", "Insufficient evidence to distinguish models"],
})[model][zh ? 0 : 1];

const mean = (values: number[]) => values.length ? values.reduce((a, b) => a + b, 0) / values.length : null;
const std = (values: number[]) => {
  const m = mean(values);
  return m == null ? null : Math.sqrt(mean(values.map(v => (v - m) ** 2))!);
};

/** Validate the extra metadata before imported records reach charts/analysis. */
export function validateTorqueAccuracyRecording(data: El05ExperimentData): void {
  if (data.status.config?.kind !== "TorqueAccuracy") return;
  const c = data.status.config, a = c.torque_accuracy, p = c.pendulum, s = data.status.torque_accuracy;
  const invalid = () => { throw new Error("Invalid torque accuracy configuration or hold timeline / 力矩准确度配置或保持段时序无效"); };
  if (data.protocol !== "StandardMit" || !a || !p?.zero_measured || !s) return invalid();
  if (typeof a.reference_id !== "string" || !a.reference_id
    || ![a.start_nm, a.end_nm, a.step_nm, a.hold_s, a.move_speed_rad_s, p.mass_kg, p.arm_mass_kg, p.length_m, p.downward_zero_rad].every(Number.isFinite)
    || a.start_nm < 0.1 || a.end_nm <= a.start_nm || a.end_nm > 5.9
    || a.end_nm > 9.80665 * p.length_m * (p.mass_kg + p.arm_mass_kg / 2) + 1e-9
    || a.step_nm < 0.01 || a.step_nm > 0.5 || a.hold_s < 0.3 || a.hold_s > 30
    || a.move_speed_rad_s < 0.05 || a.move_speed_rad_s > 0.5 || ![1, -1].includes(a.direction)
    || c.max_temperature_c < 20 || c.max_temperature_c > TORQUE_ACCURACY_MAX_TEMPERATURE_C
    || (c.gravity_compensation != null && typeof c.gravity_compensation !== "boolean")
    || !(c.kp! >= 1 && c.kp! <= 500) || !(c.kd >= 0.1 && c.kd <= 5)) return invalid();
  const count = 2 * Math.ceil((a.end_nm - a.start_nm) / a.step_nm - 1e-9) + 1;
  if (count > 401 || s.point_count !== count || !Array.isArray(s.holds) || s.holds.length > count
    || !["Moving", "Holding", "Returning", "Complete"].includes(s.phase)
    || !Number.isInteger(s.index) || s.index < 0 || s.index >= count
    || typeof s.returned_to_zero !== "boolean" || (s.return_reason != null && typeof s.return_reason !== "string")
    || (s.return_started_s != null && (!Number.isFinite(s.return_started_s) || s.return_started_s < 0))) return invalid();
  let previousEnd = -Infinity;
  for (const [index, h] of s.holds.entries()) {
    if (!h || h.index !== index || ![h.started_s, h.target_angle_rad, h.target_torque_nm].every(Number.isFinite)
      || h.started_s < 0 || h.started_s < previousEnd || typeof h.increasing !== "boolean" || typeof h.completed !== "boolean"
      || (h.ended_s != null && (!Number.isFinite(h.ended_s) || h.ended_s < h.started_s))
      || (h.completed && (h.ended_s == null || h.ended_s - h.started_s < a.hold_s - 0.001))) return invalid();
    previousEnd = h.ended_s ?? Infinity;
  }
}

/** Only completed holds contribute; movement/return and interrupted holds never fit. */
export function analyzeTorqueAccuracy(data: El05ExperimentData, derived: El05DerivedSample[]): TorqueAccuracyAnalysis | undefined {
  if (data.status.config?.kind !== "TorqueAccuracy") return undefined;
  const config = data.status.config, a = config.torque_accuracy, p = config.pendulum;
  const result: TorqueAccuracyAnalysis = { points: [], model: "inconclusive", boundary_nm: null,
    boundary_feedback_nm: null, low_scale: null, high_scale: null, slope_per_nm: null, intercept: null,
    hinge_nm: null, fit_rmse: null, paired_points: 0, hysteresis_scale: null, candidates: [], reason: null };
  if (!a || !p?.zero_measured || !data.status.torque_accuracy) {
    result.reason = "缺少已测下垂参考、扫描配置或保持段时序。";
    return result;
  }
  const seen = new Set<number>();
  let lastTime = -Infinity;
  const ordered = data.samples.map((sample, i) => ({ sample, d: derived[i] }))
    .sort((a, b) => a.sample.t_s - b.sample.t_s).filter(({ sample }) => {
      if (!Number.isFinite(sample.t_s) || seen.has(sample.sequence) || sample.t_s <= lastTime) return false;
      seen.add(sample.sequence); lastTime = sample.t_s;
      return true;
    });
  let cursor = 0;
  result.points = data.status.torque_accuracy.holds.map(hold => {
    const start = hold.started_s + a.hold_s * 0.4;
    const end = Math.min(hold.ended_s ?? Infinity, hold.started_s + a.hold_s);
    while (cursor < ordered.length && ordered[cursor].sample.t_s < start) cursor++;
    let next = cursor;
    while (next < ordered.length && ordered[next].sample.t_s < end) next++;
    const window = ordered.slice(cursor, next);
    const stable = window.filter(({ sample: s, d }) => controlFeedbackEligible(s, data)
      && s.target?.kind === "Mit" && s.target.kp > 0 && Number.isFinite(s.target.torque_nm)
      // The measured total holding torque includes the feedforward component.
      // Old recordings without gravity compensation still require zero Tff.
      && (config.gravity_compensation ? Math.abs(s.target.torque_nm) <= 6 : Math.abs(s.target.torque_nm) < 0.005)
      && Math.abs(s.target.velocity_rad_s) < 0.001
      && d.target_position_rad != null && Math.abs(d.target_position_rad - p.downward_zero_rad - hold.target_angle_rad) < 0.002
      && d.gravity_torque_nm != null && Number.isFinite(d.gravity_torque_nm)
      && Number.isFinite(s.torque_nm) && Number.isFinite(s.temperature_c)
      && Math.abs(s.velocity_rad_s) <= Math.max(0.03, feedbackVelocityResolution(data) * 1.5)
      && s.position_velocity_rad_s != null && Math.abs(s.position_velocity_rad_s) <= 0.03
      && d.acceleration_rad_s2 != null && Math.abs(d.acceleration_rad_s2) <= 0.2);
    const theory = mean(stable.map(v => v.d.gravity_torque_nm!));
    const feedback = mean(stable.map(v => v.sample.torque_nm));
    const angles = stable.map(v => v.sample.unwrapped_position_rad - p.downward_zero_rad);
    const feedbackStd = std(stable.map(v => v.sample.torque_nm));
    let reason: string | null = null;
    const coverage = stable.length > 1 ? stable[stable.length - 1].sample.t_s - stable[0].sample.t_s : 0;
    const gaps = stable.slice(1).map((v, i) => v.sample.t_s - stable[i].sample.t_s);
    if (!hold.completed) reason = "保持段未完成";
    else if (stable.length < 5 || stable.length < window.length * 0.8 || coverage < a.hold_s * 0.48
      || gaps.some(gap => gap > Math.min(0.1, a.hold_s * 0.2))) reason = "稳定样本不足或采样间断，请延长停留";
    else if (Math.max(...angles) - Math.min(...angles) > 0.004) reason = "位置仍在漂移";
    else if (feedbackStd! > 0.08) reason = "反馈力矩波动过大";
    else if (stable.some(v => Math.abs(v.sample.torque_nm) >= 5.985)) reason = "反馈力矩接近协议饱和";
    else if (theory == null || feedback == null || theory * a.direction < 0.1 || feedback * theory <= 0) reason = "重力矩或反馈方向无效";
    const error = theory != null && feedback != null ? feedback - theory : null;
    return { ...hold, sample_count: stable.length, accepted: reason == null, reason,
      angle_rad: mean(angles), theory_nm: theory, feedback_nm: feedback, feedback_std_nm: feedbackStd,
      error_nm: error, error_percent: theory && error != null ? 100 * error / theory : null,
      send_scale: reason == null ? feedback! / theory! : null,
      temperature_c: mean(stable.map(v => v.sample.temperature_c)) };
  });

  // Pair the same requested load from both approaches before fitting; do not
  // turn one-direction friction into a torque calibration discontinuity.
  const groups = new Map<string, TorqueAccuracyPoint[]>();
  for (const point of result.points.filter(p => p.accepted)) {
    const key = point.target_torque_nm.toFixed(6);
    groups.set(key, [...(groups.get(key) ?? []), point]);
  }
  const pairs = [...groups.values()].filter(group => group.some(p => p.increasing) && group.some(p => !p.increasing));
  result.paired_points = pairs.length;
  result.hysteresis_scale = mean(pairs.map(g => Math.abs(g[0].send_scale! - g[1].send_scale!)));
  const rows = pairs.map(g => ({ x: mean(g.map(p => Math.abs(p.theory_nm!)))!,
    y: mean(g.map(p => p.send_scale!))!, feedback: mean(g.map(p => Math.abs(p.feedback_nm!)))! }))
    .sort((a, b) => a.x - b.x);
  if (rows.length < 8 || rows[rows.length - 1].x - rows[0].x < 0.2) {
    result.reason = "需至少 8 个有效递增/回扫配对点，且实际理论力矩跨度至少 0.2 Nm。";
    return result;
  }
  type Fit = { model: Exclude<TorqueAccuracyModel, "inconclusive">; predicted: number[]; parameters: number;
    intercept: number; slope: number; split?: number; high?: number; hinge?: number; bic?: number; rmse?: number };
  const y = rows.map(r => r.y), x = rows.map(r => r.x), n = rows.length;
  const regress = (xs: number[]) => {
    const mx = mean(xs)!, my = mean(y)!;
    const xx = xs.reduce((s, v) => s + (v - mx) ** 2, 0);
    const slope = xx > 1e-12 ? xs.reduce((s, v, i) => s + (v - mx) * (y[i] - my), 0) / xx : 0;
    return { slope, intercept: my - slope * mx };
  };
  const line = regress(x), average = mean(y)!;
  const fits: Fit[] = [
    { model: "constant", predicted: y.map(() => average), parameters: 1, intercept: average, slope: 0 },
    { model: "linear", predicted: x.map(v => line.intercept + line.slope * v), parameters: 2, ...line },
  ];
  for (let split = 3; split <= n - 3; split++) {
    const low = mean(y.slice(0, split))!, high = mean(y.slice(split))!;
    fits.push({ model: "step", predicted: y.map((_, i) => i < split ? low : high), parameters: 3,
      intercept: low, slope: 0, high, split });
    const hinge = (x[split - 1] + x[split]) / 2, shifted = x.map(v => Math.max(0, v - hinge));
    const fit = regress(shifted);
    fits.push({ model: "piecewise_linear", predicted: shifted.map(v => fit.intercept + fit.slope * v),
      parameters: 3, ...fit, hinge, split });
  }
  // A quantization/model noise floor avoids claiming certainty from perfectly
  // flat synthetic or rounded telemetry. BIC penalizes the boundary search.
  const noiseFloor = Math.max(0.002, (result.hysteresis_scale ?? 0) / 2);
  for (const fit of fits) {
    const mse = mean(y.map((v, i) => (v - fit.predicted[i]) ** 2))!;
    fit.rmse = Math.sqrt(mse);
    fit.bic = n * Math.log(Math.max(mse, noiseFloor ** 2)) + fit.parameters * Math.log(n);
  }
  const candidates = [...new Set(fits.map(f => f.model))].map(model => fits.filter(f => f.model === model).sort((a, b) => a.bic! - b.bic! || a.rmse! - b.rmse!)[0])
    .sort((a, b) => a.bic! - b.bic!);
  result.candidates = candidates.map(f => ({ model: f.model, bic: f.bic!, rmse: f.rmse! }));
  const best = candidates[0];
  result.fit_rmse = best.rmse!;
  // Prefer a simpler model already below the measurement noise floor over a
  // more flexible boundary model that cannot improve measurable residuals.
  const ambiguous = best.model !== "constant" && candidates[1].bic! - best.bic! < 6
    && !(best.parameters < candidates[1].parameters && best.rmse! <= noiseFloor);
  if ((result.hysteresis_scale ?? 0) > 0.025) {
    result.reason = `往返反馈/理论比值差均值为 ${(result.hysteresis_scale! * 100).toFixed(2)} 个百分点，超过 2.5 个百分点；本组不能确定力矩误差边界或发送补偿系数。请在相近温度下从两个方向接近同一负载复测，并核对下垂零点和负载几何。`;
    return result;
  }
  if (ambiguous || best.rmse! > 0.02) {
    result.reason = "模型区分度不足或残差过大；请缩小步长、延长停留并复测。";
    return result;
  }
  result.model = best.model;
  result.intercept = best.intercept;
  result.slope_per_nm = best.slope;
  result.hinge_nm = best.hinge ?? null;
  result.low_scale = best.model === "linear" ? null : best.intercept;
  result.high_scale = best.model === "step" ? best.high! : best.model === "constant" ? best.intercept : null;
  if (best.split != null) {
    result.boundary_nm = [x[best.split - 1], x[best.split]];
    result.boundary_feedback_nm = [rows[best.split - 1].feedback, rows[best.split].feedback];
  }
  result.reason = best.model === "constant" ? "当前扫描区间内未分辨出变化边界；不代表区间外不存在边界。" : null;
  return result;
}
