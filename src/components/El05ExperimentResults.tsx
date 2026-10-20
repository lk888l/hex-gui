import { useEffect, useMemo, useState } from "react";
import { Alert, Card, Collapse, Descriptions, Empty, Segmented, Space, Table, Tag, Typography } from "antd";
import ReactECharts from "echarts-for-react";
import type { El05ExperimentData, El05ExperimentStatus } from "../el05Api";
import { analyzeExperiment, deriveExperimentSamples, EL05_TORQUE_UNIT, experimentTorqueLimitNm, staticFrictionPasses } from "../el05Experiment";
import { LEGACY_VELOCITY_ANALYSIS_NOTE, normalizeExperimentFeedbackVelocity } from "../el05Velocity";

export type ExperimentAnalysis = ReturnType<typeof analyzeExperiment>;
export interface ExperimentRecord {
  id: string;
  name: string;
  saved_at: string;
  data: El05ExperimentData;
  analysis: ExperimentAnalysis;
  source_samples: number;
  reduced: boolean;
}
export const HISTORY_KEY = "hexmeow.el05.pendulum-history.v2";
const HISTORY_BUDGET = 1_350_000; // UTF-16 localStorage: roughly 2.7 MB, including JSON metadata.

/** Preserve endpoints and the whole time span when reducing a plot/archive. */
export function thin<T>(items: T[], limit: number): T[] {
  if (items.length <= limit) return items;
  return Array.from({ length: limit }, (_, i) => items[Math.round(i * (items.length - 1) / (limit - 1))]);
}
export function runKey(data: El05ExperimentData): string {
  return `${data.uid ?? data.motor_id ?? "motor"}:${data.status.run_id ?? `legacy-${data.samples[0]?.sequence ?? 0}-${data.samples[0]?.t_s ?? 0}`}`;
}
export function makeRecord(data: El05ExperimentData, name: string): ExperimentRecord {
  return { id: runKey(data), name, saved_at: new Date().toISOString(),
    data: { ...data, status: { ...data.status, active: false, reason: data.status.reason ?? (data.status.active ? "snapshot" : null) }, samples: thin(data.samples, 1400), commands: thin(data.commands, 900) },
    analysis: analyzeExperiment(data), source_samples: data.samples.length, reduced: data.samples.length > 1400 || data.commands.length > 900 };
}
function isAnalysis(value: unknown): value is ExperimentAnalysis {
  const report = value as ExperimentAnalysis | null;
  return !!report && Number.isFinite(report.sample_count) && Number.isFinite(report.duration_s)
    && !!report.quality && (report.quality.sample_rate_hz == null || Number.isFinite(report.quality.sample_rate_hz)) && (report.quality.max_gap_s == null || Number.isFinite(report.quality.max_gap_s))
    && (!report.frequency_response || (Array.isArray(report.frequency_response.points) && report.frequency_response.points.every(point => [point.frequency_hz, point.gain_db, point.phase_deg].every(Number.isFinite))))
    && Array.isArray(report.quality.notes) && report.quality.notes.every(note => typeof note === "string")
    && Array.isArray(report.limitations) && report.limitations.every(note => typeof note === "string")
    && Array.isArray(report.sections) && report.sections.every(section => typeof section.id === "string" && typeof section.title === "string"
      && Array.isArray(section.notes) && section.notes.every(note => typeof note === "string")
      && Array.isArray(section.metrics) && section.metrics.every(metric => typeof metric.key === "string" && typeof metric.label === "string"
        && typeof metric.unit === "string" && (metric.value === null || Number.isFinite(metric.value))
        && (metric.digits == null || (Number.isInteger(metric.digits) && metric.digits >= 0 && metric.digits <= 8))));
}
export function loadRecords(): ExperimentRecord[] {
  try {
    const raw = localStorage.getItem(HISTORY_KEY);
    if (!raw || raw.length > HISTORY_BUDGET + 1000) return [];
    const records = JSON.parse(raw);
    if (!Array.isArray(records)) return [];
    return records.filter(record => record?.id && record?.data?.status?.config && Array.isArray(record?.data?.samples) && isAnalysis(record?.analysis)).slice(0, 6).flatMap(record => {
      try {
        const parsed = parseExperimentValue(record);
        return [{ ...record, data: parsed.data, analysis: parsed.analysis ?? analyzeExperiment(parsed.data),
          source_samples: parsed.source_samples ?? parsed.data.samples.length, reduced: parsed.reduced ?? false }];
      } catch { return []; }
    });
  } catch { return []; }
}
/** A small, explicit localStorage budget avoids growing browser storage without bounds. */
export function storeRecords(records: ExperimentRecord[]): { records: ExperimentRecord[]; persisted: boolean } {
  let compact = records.slice(0, 6);
  let serialized = JSON.stringify(compact);
  for (let attempt = 0; serialized.length > HISTORY_BUDGET && attempt < 8; attempt++) {
    compact = compact.map(record => ({ ...record, reduced: true, data: { ...record.data,
      samples: thin(record.data.samples, Math.max(60, Math.floor(record.data.samples.length / 2))),
      commands: thin(record.data.commands, Math.max(40, Math.floor(record.data.commands.length / 2))) } }));
    serialized = JSON.stringify(compact);
  }
  if (serialized.length > HISTORY_BUDGET) compact = compact.slice(0, 3);
  try { localStorage.setItem(HISTORY_KEY, JSON.stringify(compact)); }
  catch {
    compact = compact.map(record => ({ ...record, reduced: true, data: { ...record.data, samples: thin(record.data.samples, 180), commands: thin(record.data.commands, 100) } }));
    try { localStorage.setItem(HISTORY_KEY, JSON.stringify(compact)); }
    catch { return { records: compact, persisted: false }; }
  }
  return { records: compact, persisted: true };
}

export function parseExperimentJson(text: string): { data: El05ExperimentData; analysis?: ExperimentAnalysis; source_samples?: number; reduced?: boolean } {
  return parseExperimentValue(JSON.parse(text));
}
function parseExperimentValue(parsed: any): { data: El05ExperimentData; analysis?: ExperimentAnalysis; source_samples?: number; reduced?: boolean } {
  const data = parsed?.data?.samples ? parsed.data : parsed;
  if (!data?.status || !Array.isArray(data.samples) || !data.samples.length || data.samples.length > 250_000) {
    throw new Error("JSON needs status and 1–250000 telemetry samples / JSON 需要 status 和 1–250000 条遥测数据");
  }
  const kinds = ["Record", "VelocitySteps", "TorqueRamp", "StaticTorque", "PositionSteps", "SineSweep", "Impedance", "Backdrive", "ThermalHold", "SlowTriangle", "StaticFriction"];
  if (!kinds.includes(data.status.config?.kind)) throw new Error("Missing or unsupported experiment configuration / 缺少或不支持的测试配置");
  const config = data.status.config;
  const numeric = ["duration_s", "velocity_rad_s", "torque_nm", "kd", "max_temperature_c"];
  const optionalNumeric = ["kp", "center_angle_rad", "position_amplitude_rad", "frequency_start_hz", "frequency_end_hz", "disturbance_torque_nm", "origin_position_rad", "torque_ramp_rate_nm_s", "max_excursion_rad", "motion_threshold_rad", "motion_confirm_s", "settle_time_s"];
  if (numeric.some(key => !Number.isFinite(config[key])) || config.duration_s <= 0 || optionalNumeric.some(key => config[key] != null && !Number.isFinite(config[key]))) {
    throw new Error("Invalid experiment numeric configuration / 测试配置缺少必要数值或包含无效数值");
  }
  if (config.gravity_compensation != null && typeof config.gravity_compensation !== "boolean") throw new Error("Invalid gravity compensation setting / 重力补偿设置无效");
  if (config.friction_direction != null && !["Both", "Positive", "Negative"].includes(config.friction_direction)) throw new Error("Invalid friction direction / 静摩擦方向无效");
  if (config.pendulum && typeof config.pendulum.zero_measured !== "boolean") throw new Error("Invalid downward reference flag / 下垂参考标记无效");
  if (config.pendulum && (["mass_kg", "length_m", "arm_mass_kg", "downward_zero_rad"].some(key => !Number.isFinite(config.pendulum[key])) || (config.pendulum.extra_inertia_kg_m2 != null && !Number.isFinite(config.pendulum.extra_inertia_kg_m2)))) {
    throw new Error("Invalid pendulum geometry / 摆锤几何或参考角数据无效");
  }
  const validateSample = (sample: any) => {
    if (!sample || ![sample.t_s, sample.sequence, sample.unwrapped_position_rad, sample.velocity_rad_s, sample.torque_nm, sample.temperature_c].every(Number.isFinite)) {
      throw new Error("Invalid telemetry sample / 遥测样本包含缺失或无效的数值");
    }
    if (sample.target?.kind === "Mit" && ["position_rad", "velocity_rad_s", "torque_nm", "kp", "kd"].some(key => !Number.isFinite(sample.target[key]))) {
      throw new Error("Invalid MIT target in telemetry / 遥测中的 MIT 目标数值无效");
    }
    sample.payload = Array.isArray(sample.payload) ? sample.payload : [];
    sample.can_id = Number.isFinite(sample.can_id) ? sample.can_id : 0;
    sample.position_rad = Number.isFinite(sample.position_rad) ? sample.position_rad : sample.unwrapped_position_rad;
    sample.target = sample.target?.kind ? sample.target : null;
    sample.target_sequence ??= 0;
    sample.fault_bits ??= 0;
    sample.status_code ??= 0;
  };
  for (const sample of data.samples) validateSample(sample);
  const calibration = data.status.static_friction;
  if (calibration != null) {
    if (calibration.completed_directions != null && (!Array.isArray(calibration.completed_directions) || calibration.completed_directions.length > 1)) {
      throw new Error("Invalid completed friction directions / 已完成的静摩擦方向无效");
    }
    const phases = ["Settling", "Ramping", "Returning", "Complete"];
    const outcomes = ["BreakawayDetected", "AngleLimit", "TorqueLimit", "Timeout", "NotSettled", "Interrupted"];
    const passes = [...(calibration.completed_directions ?? []), calibration];
    for (const pass of passes) {
      if (!pass || typeof pass !== "object" || !phases.includes(pass.phase) || (pass.outcome !== null && !outcomes.includes(pass.outcome))
        || !Number.isFinite(pass.peak_excursion_rad)
        || (pass.direction != null && pass.direction !== 1 && pass.direction !== -1)
        || (pass.started_s != null && (!Number.isFinite(pass.started_s) || pass.started_s < 0))
        || (pass.return_settled != null && typeof pass.return_settled !== "boolean")
        || ["reference_position_rad", "ramp_started_s", "return_started_s"].some(key => pass[key] !== null && !Number.isFinite(pass[key]))) {
        throw new Error("Invalid static friction calibration status / 静摩擦标定状态无效");
      }
      for (const key of ["last_stationary_sample", "onset_sample", "confirmation_sample"]) {
        if (pass[key] != null) validateSample(pass[key]);
      }
    }
    if (passes.length === 2 && (passes.some(pass => pass.direction == null || pass.started_s == null)
      || passes[0].direction === passes[1].direction || passes[0].started_s > passes[1].started_s
      || passes[0].phase !== "Complete" || passes[0].return_settled !== true)) {
      throw new Error("Invalid friction direction sequence / 静摩擦方向序列无效");
    }
  }
  data.samples.sort((a: { t_s: number }, b: { t_s: number }) => a.t_s - b.t_s);
  data.commands = Array.isArray(data.commands) ? data.commands.filter((command: { t_s: number; sequence: number }) => Number.isFinite(command?.t_s) && Number.isFinite(command?.sequence)) : [];
  data.parameters = Array.isArray(data.parameters) ? data.parameters : [];
  data.status.active = false;
  data.status.samples = Number.isFinite(data.status.samples) ? data.status.samples : data.samples.length;
  data.status.elapsed_s = Number.isFinite(data.status.elapsed_s) ? data.status.elapsed_s : data.samples[data.samples.length - 1].t_s;
  data.status.dropped_frames = Number.isFinite(data.status.dropped_frames) ? data.status.dropped_frames : 0;
  data.status.reason = typeof data.status.reason === "string" ? data.status.reason : null;
  const normalized = normalizeExperimentFeedbackVelocity(data);
  const sourceSamples = Number.isFinite(parsed.source_samples ?? parsed.ui_archive?.source_samples) ? parsed.source_samples ?? parsed.ui_archive?.source_samples : undefined;
  const reduced = (parsed.reduced ?? parsed.ui_archive?.reduced) === true;
  if (normalized.changed) {
    const analysis = analyzeExperiment(normalized.data);
    analysis.quality.notes.push(LEGACY_VELOCITY_ANALYSIS_NOTE);
    return { data: normalized.data, analysis, source_samples: normalized.data.samples.length,
      reduced: reduced || (sourceSamples ?? 0) > normalized.data.samples.length || data.status.samples > normalized.data.samples.length };
  }
  const frozenAnalysis = isAnalysis(parsed.ui_archive?.analysis) ? parsed.ui_archive.analysis : parsed.analysis;
  return { data, analysis: isAnalysis(frozenAnalysis) ? frozenAnalysis : undefined,
    source_samples: sourceSamples, reduced };
}

function translated(object: object, key: string, fallback: string, zh: boolean): string {
  return !zh && typeof (object as Record<string, unknown>)[`${key}_en`] === "string"
    ? (object as Record<string, string>)[`${key}_en`] : fallback;
}
export function metricLabel(metric: { label: string }, zh: boolean): string { return translated(metric, "label", metric.label, zh); }
export function metricValue(metric: { value: number | null; digits?: number; unit: string }): string {
  return metric.value != null && Number.isFinite(metric.value) ? `${metric.value.toFixed(metric.digits ?? 4)}${metric.unit ? ` ${metric.unit}` : ""}` : "—";
}
export function staticFrictionStatusText(status: El05ExperimentStatus, zh: boolean): string {
  const calibration = status.static_friction;
  if (!calibration) return "";
  const phases = {
    Settling: ["确认自然静止", "Confirming natural rest"], Ramping: ["缓慢增加转矩", "Slow torque ramp"],
    Returning: ["撤去斜坡并回位", "Ramp ended; returning"], Complete: ["标定结束", "Calibration ended"],
  };
  const outcomes = {
    BreakawayDetected: ["已检测到起动", "Breakaway detected"], AngleLimit: ["接近角度上限，未取得有效标定", "Excursion limit approached; calibration unsuccessful"],
    TorqueLimit: ["达到最大转矩，未检测到起动", "Torque limit reached without breakaway"], Timeout: ["标定超时", "Calibration timed out"],
    NotSettled: ["未能确认静止，未开始标定", "Rest not confirmed; calibration not started"], Interrupted: ["标定中断", "Calibration interrupted"],
  };
  const direction = calibration.direction ?? Math.sign(status.config?.torque_nm ?? 1);
  const prefix = direction > 0 ? ["正向 (+)", "Positive (+)"] : ["反向 (−)", "Negative (−)"];
  const passes = [...(calibration.completed_directions ?? []), calibration];
  const bidirectional = status.config?.friction_direction === "Both" || passes.length > 1;
  const progress = bidirectional ? `${passes.filter(pass => pass.outcome === "BreakawayDetected").length}/2 ${zh ? "方向已起动" : "directions detected"}` : null;
  const untested = bidirectional && passes.length === 1 && !status.active && calibration.phase === "Complete"
    ? ["反向未测试", "Negative direction not tested"] : null;
  const returned = calibration.return_settled === false ? ["回位未完成（已停止）", "Return incomplete (stopped)"]
    : calibration.return_settled === true ? ["已稳定回到起始位置", "Settled at the starting position"] : null;
  return [prefix[zh ? 0 : 1], phases[calibration.phase]?.[zh ? 0 : 1], calibration.outcome && outcomes[calibration.outcome]?.[zh ? 0 : 1], returned?.[zh ? 0 : 1], progress, untested?.[zh ? 0 : 1]].filter(Boolean).join(" · ");
}
export function experimentReport(data: El05ExperimentData, analysis: ExperimentAnalysis, name: string, zh: boolean, reduced = false): string {
  const tr = (cn: string, en: string) => zh ? cn : en;
  const geometry = data.status.config?.pendulum;
  const lines = [`# ${name}`, "", `${tr("导出时间", "Exported")}: ${new Date().toISOString()}`, "",
    `${tr("电机", "Motor")}: ${data.motor_id ?? "—"} · UID ${data.uid ?? "—"}`,
    `${tr("力矩单位", "Torque unit")}: ${EL05_TORQUE_UNIT} (N·m) · ${tr("理论保持力矩、驱动反馈与指令使用相同单位", "Theoretical holding torque, drive feedback and commands use the same unit")}`,
    `${tr("工况", "Experiment")}: ${data.status.config?.kind ?? "—"} · ${data.status.elapsed_s.toFixed(3)} s · ${data.samples.length} ${tr("保留样本", "retained samples")}`,
    `${tr("状态", "Status")}: ${data.status.static_friction ? staticFrictionStatusText(data.status, zh) : data.status.reason ?? (data.status.active ? tr("进行中", "Running") : tr("完成", "Complete"))}`, "",
    ...(reduced ? [analysis.quality.notes.includes(LEGACY_VELOCITY_ANALYSIS_NOTE)
      ? tr("历史曲线已抽样；反馈速度方向已修正，以下指标按保留样本重新计算。CSV/JSON 中的曲线样本为归档抽样数据。", "History traces are reduced. Feedback velocity direction is corrected and metrics are recalculated from retained samples; exported CSV/JSON contains reduced plot samples.")
      : tr("历史数据已按时间抽样，以下指标保留自归档前的完整数据分析；CSV/JSON 中的曲线样本为归档抽样数据。", "History samples are reduced over the full time span. Metrics retain the original analysis; exported CSV/JSON contains reduced plot samples."), ""] : []),
    "## " + tr("参数", "Configuration"), "", "```json", JSON.stringify(data.status.config, null, 2), "```", "",
    ...(geometry ? [`${tr("下垂参考", "Downward reference")}: ${geometry.zero_measured ? geometry.downward_zero_rad + " rad" : tr("未测定", "Unmeasured")}`, ""] : [])];
  for (const section of analysis.sections) {
    lines.push(`## ${translated(section, "title", section.title, zh)}`, "", `| ${tr("指标", "Metric")} | ${tr("结果", "Result")} | ${tr("说明", "Note")} |`, "| --- | --- | --- |");
    for (const metric of section.metrics) lines.push(`| ${metricLabel(metric, zh)} | ${metricValue(metric)} | ${(metric.note ?? "").replace(/\|/g, "/").replace(/\n/g, " ")} |`);
    lines.push("", ...section.notes.map(note => `- ${note}`), "");
  }
  const fmt = (value: number | null | undefined, digits = 4) => value != null && Number.isFinite(value) ? value.toFixed(digits) : "—";
  const detailTable = (title: string, headers: string[], rows: string[][]) => {
    if (!rows.length) return;
    lines.push(`## ${title}`, "", `| ${headers.join(" | ")} |`, `| ${headers.map(() => "---").join(" | ")} |`, ...rows.map(row => `| ${row.join(" | ")} |`), "");
  };
  if (analysis.kind === "StaticTorque") detailTable(tr("逐平台静态结果", "Static plateau results"),
    ["t (s)", tr("目标角 (rad)", "Target (rad)"), tr("实际角 (rad)", "Angle (rad)"), `${tr("理论重力矩", "Theoretical gravity torque")} (${EL05_TORQUE_UNIT})`, "Iq (A)", `${tr("驱动反馈力矩", "Drive feedback torque")} (${EL05_TORQUE_UNIT})`, tr("接近方向", "Approach")],
    (analysis.static_torque?.platforms ?? []).map(point => [fmt(point.start_s, 2), fmt(point.target_angle_rad), fmt(point.angle_rad), fmt(point.gravity_torque_nm), fmt(point.current_a), fmt(point.feedback_torque_nm), point.approach ?? "—"]));
  const position = analysis.kind !== "VelocitySteps";
  const steps = position ? analysis.tracking?.position_steps : analysis.tracking?.velocity_steps;
  detailTable(tr("逐阶跃响应结果", "Individual step responses"),
    ["t (s)", position ? "rad" : "rad/s", tr("上升时间 (s)", "Rise (s)"), tr("超调 (%)", "Overshoot (%)"), tr("稳定时间 (s)", "Settling (s)"), tr("稳态误差", "Steady error")],
    (steps ?? []).map(step => [fmt(step.start_s, 2), `${fmt(step.from)} → ${fmt(step.to)}`, fmt(step.rise_time_s), fmt(step.overshoot_percent, 2), fmt(step.settling_time_s), fmt(step.steady_error)]));
  if (analysis.kind === "SineSweep") detailTable(tr("逐频率响应测点", "Frequency response points"),
    ["Hz", tr("增益 (dB)", "Gain (dB)"), tr("相位 (°)", "Phase (°)"), tr("输入幅值 (rad)", "Input amplitude (rad)"), tr("输出幅值 (rad)", "Output amplitude (rad)"), tr("周期数", "Cycles")],
    (analysis.frequency_response?.points ?? []).map(point => [fmt(point.frequency_hz, 3), fmt(point.gain_db, 3), fmt(point.phase_deg, 2), fmt(point.input_amplitude_rad), fmt(point.output_amplitude_rad), fmt(point.cycles, 2)]));
  if (analysis.kind === "Impedance") detailTable(tr("逐次注入响应", "Individual injection responses"),
    [tr("起始 (s)", "Start (s)"), tr("结束 (s)", "End (s)"), tr("峰值角偏移 (rad)", "Peak deflection (rad)"), tr("恢复时间 (s)", "Recovery (s)")],
    (analysis.impedance?.responses ?? []).map(response => [fmt(response.start_s, 2), fmt(response.end_s, 2), fmt(response.peak_displacement_rad), fmt(response.recovery_s)]));
  lines.push("## " + tr("数据质量与分析边界", "Data quality and interpretation"), "",
    `${tr("平均记录频率", "Recorded rate")}: ${analysis.quality.sample_rate_hz?.toFixed(2) ?? "—"} Hz; ${tr("最大采样间隔", "Maximum gap")}: ${analysis.quality.max_gap_s?.toFixed(4) ?? "—"} s`,
    ...analysis.quality.notes.map(note => `- ${note}`), ...analysis.limitations.map(note => `- ${note}`));
  return lines.join("\n") + "\n";
}

export function El05ExperimentResults({ data, analysis, zh, reduced }: { data: El05ExperimentData | null; analysis: ExperimentAnalysis | null; zh: boolean; reduced?: boolean }) {
  const tr = (cn: string, en: string) => zh ? cn : en;
  const [chart, setChart] = useState("motion");
  useEffect(() => { setChart(data?.status.config?.kind === "StaticFriction" ? "friction" : "motion"); }, [data?.status.config?.kind]);
  const chartOption = useMemo(() => {
    if (!data?.samples.length) return {};
    if (chart === "friction") {
      const samples = thin(data.samples, 3000);
      const passes = staticFrictionPasses(data);
      const origin = data.status.config?.origin_position_rad ?? samples[0].unwrapped_position_rad;
      const limitDeg = (data.status.config?.max_excursion_rad ?? 3 * Math.PI / 180) * 180 / Math.PI;
      const thresholdDeg = (data.status.config?.motion_threshold_rad ?? 0.15 * Math.PI / 180) * 180 / Math.PI;
      const events = passes.flatMap(pass => {
        const prefix = (pass.direction ?? Math.sign(data.status.config?.torque_nm ?? 1)) > 0 ? "+ " : "− ";
        return [
          { name: prefix + tr("开始斜坡", "Ramp starts"), time: pass.ramp_started_s },
          { name: prefix + tr("起动", "Motion onset"), time: pass.onset_sample?.t_s },
          { name: prefix + tr("回位", "Return starts"), time: pass.return_started_s },
        ];
      }).filter(event => event.time != null && Number.isFinite(event.time));
      return { animation: false, color: ["#60a5fa", "#f59e0b", "#34d399"],
        tooltip: { trigger: "axis", valueFormatter: (value: number | null) => value != null ? value.toFixed(5) : "—" },
        legend: { textStyle: { color: "#94a3b8" }, top: 0 }, grid: { left: 65, right: 65, top: 55, bottom: 70 },
        xAxis: { type: "value", name: "s", min: 0, axisLabel: { color: "#94a3b8" } },
        yAxis: [{ type: "value", name: tr("偏转角 (°)", "Excursion (°)"), scale: true, axisLabel: { color: "#94a3b8" } }, { type: "value", name: EL05_TORQUE_UNIT, scale: true, axisLabel: { color: "#94a3b8" } }],
        dataZoom: [{ type: "inside", filterMode: "none" }, { type: "slider", height: 20, bottom: 10 }],
        series: [
          { type: "line", name: tr("相对各方向静止起点的偏转角", "Excursion from each direction's rest"), showSymbol: false, data: samples.map(sample => {
            const pass = [...passes].reverse().find(pass => sample.t_s >= (pass.started_s ?? 0));
            return [sample.t_s, (sample.unwrapped_position_rad - (pass?.reference_position_rad ?? origin)) * 180 / Math.PI];
          }),
            markLine: { symbol: "none", silent: true, label: { color: "#94a3b8", fontSize: 10 }, data: [
              ...[limitDeg, -limitDeg].map(value => ({ name: tr("角度上限", "Excursion limit"), yAxis: value, lineStyle: { color: "#f59e0b", type: "dashed" }, label: { formatter: `${Math.abs(value).toFixed(2)}° ${tr("上限", "limit")}` } })),
              ...[thresholdDeg, -thresholdDeg].map(value => ({ name: tr("起动阈值", "Motion threshold"), yAxis: value, lineStyle: { color: "#64748b", type: "dotted" }, label: { formatter: `${Math.abs(value).toFixed(2)}° ${tr("起动阈值", "threshold")}` } })),
              ...events.map(event => ({ name: event.name, xAxis: event.time, lineStyle: { color: "#a78bfa", type: "dashed" }, label: { formatter: event.name, position: "insideEndTop" } })),
            ] } },
          { type: "line", name: tr("转矩指令（含回位）", "Torque command including return"), yAxisIndex: 1, showSymbol: false, data: samples.map(sample => [sample.t_s, sample.target?.kind === "Mit" ? sample.target.torque_nm : null]) },
          { type: "line", name: tr("驱动估计转矩", "Drive torque estimate"), yAxisIndex: 1, showSymbol: false, data: samples.map(sample => [sample.t_s, sample.torque_nm]) },
        ] };
    }
    if (chart === "static") {
      const platforms = analysis?.static_torque?.platforms ?? [];
      const valid = platforms.filter(point => point.current_a != null && point.gravity_torque_nm != null);
      const slope = analysis?.static_torque?.torque_constant_nm_per_a;
      const intercept = analysis?.static_torque?.intercept_nm;
      const currents = valid.map(point => point.current_a!);
      const bounds = currents.length ? [Math.min(...currents), Math.max(...currents)] : [];
      return { animation: false, color: ["#60a5fa", "#f59e0b", "#34d399"], tooltip: { trigger: "item" },
        legend: { textStyle: { color: "#94a3b8" } }, grid: { left: 65, right: 35, top: 45, bottom: 45 },
        xAxis: { type: "value", name: "Iq (A)", scale: true, axisLabel: { color: "#94a3b8" } },
        yAxis: { type: "value", name: `${tr("理论重力矩", "Theoretical gravity torque")} (${EL05_TORQUE_UNIT})`, scale: true, axisLabel: { color: "#94a3b8" } },
        series: [
          { type: "scatter", name: tr("正向接近", "Positive approach"), symbolSize: 10, data: valid.filter(point => point.approach !== "negative").map(point => [point.current_a, point.gravity_torque_nm]) },
          { type: "scatter", name: tr("反向接近", "Negative approach"), symbolSize: 10, data: valid.filter(point => point.approach === "negative").map(point => [point.current_a, point.gravity_torque_nm]) },
          { type: "line", name: tr("线性回归", "Linear regression"), showSymbol: false, data: slope != null && intercept != null ? bounds.map(current => [current, intercept + slope * current]) : [] },
        ] };
    }
    if (chart === "frequency") {
      const points = analysis?.frequency_response?.points ?? [];
      return { animation: false, color: ["#60a5fa", "#f59e0b"], tooltip: { trigger: "axis" },
        legend: { data: [tr("增益", "Gain"), tr("相位", "Phase")], textStyle: { color: "#94a3b8" } },
        grid: { left: 65, right: 65, top: 45, bottom: 45 },
        xAxis: { type: "log", name: "Hz", axisLabel: { color: "#94a3b8" } },
        yAxis: [{ type: "value", name: "dB", axisLabel: { color: "#94a3b8" } }, { type: "value", name: "°", axisLabel: { color: "#94a3b8" } }],
        series: [{ type: "line", name: tr("增益", "Gain"), data: points.map(point => [point.frequency_hz, point.gain_db]) },
          { type: "line", name: tr("相位", "Phase"), yAxisIndex: 1, data: points.map(point => [point.frequency_hz, point.phase_deg]) }] };
    }
    const derived = thin(deriveExperimentSamples(data), 2200);
    const samples = thin(data.samples, 2200);
    const series: { name: string; data: [number, number | null][]; yAxisIndex?: number; step?: "end"; lineStyle?: { width: number; type: "dashed" } }[] = [];
    const add = (name: string, values: (number | null)[], yAxisIndex = 0) => series.push({ name, data: values.map((value, index) => [samples[index].t_s, value != null && Number.isFinite(value) ? value : null]), yAxisIndex });
    let units = ["rad", "rad/s"];
    if (chart === "motion") {
      const origin = data.status.config?.origin_position_rad ?? samples[0].unwrapped_position_rad;
      add(tr("位置（相对起点）", "Position from start"), samples.map(sample => sample.unwrapped_position_rad - origin));
      add(tr("目标位置", "Target position"), derived.map(sample => sample.target_position_rad != null ? sample.target_position_rad - origin : null));
      add(tr("速度", "Velocity"), samples.map(sample => sample.velocity_rad_s), 1);
      add(tr("目标速度", "Target velocity"), samples.map(sample => sample.target?.kind === "Mit" ? sample.target.velocity_rad_s : null), 1);
    } else if (chart === "torque") {
      units = [EL05_TORQUE_UNIT, "A"];
      add(tr("驱动反馈力矩", "Drive feedback torque"), samples.map(sample => sample.torque_nm));
      add(tr("前馈转矩指令", "Feedforward torque"), samples.map(sample => sample.target?.kind === "Mit" ? sample.target.torque_nm : null));
      add(tr("重力保持转矩", "Gravity holding torque"), derived.map(sample => sample.gravity_torque_nm));
      add(tr("摩擦转矩估计", "Estimated friction"), derived.map(sample => sample.friction_torque_nm));
      add("Iq", derived.map(sample => sample.fresh_current_a), 1);
      if (data.status.config?.kind === "StaticTorque") {
        const torqueLimit = experimentTorqueLimitNm(data);
        Object.assign(series[0], { markLine: { silent: true, symbol: "none", lineStyle: { color: "#f87171", type: "dashed" },
          label: { color: "#f87171", formatter: `{c} ${EL05_TORQUE_UNIT} ${tr("驱动上限", "drive limit")}` },
          data: (torqueLimit === 0 ? [0] : [-torqueLimit, torqueLimit]).map(value => ({ yAxis: value })),
        } });
        add(tr("目标角理论保持力矩", "Theoretical holding torque at target"), derived.map(sample => sample.target_holding_torque_nm));
        Object.assign(series[series.length - 1], { step: "end", lineStyle: { width: 2, type: "dashed" } });
      }
    } else if (chart === "thermal") {
      units = ["°C", "A"];
      add(tr("温度", "Temperature"), samples.map(sample => sample.temperature_c));
      add("Iq", derived.map(sample => sample.fresh_current_a), 1);
    } else {
      units = ["rad", "rad/s"];
      add(tr("位置误差", "Position error"), derived.map(sample => sample.position_error_rad));
      add(tr("速度误差", "Velocity error"), derived.map(sample => sample.velocity_error_rad_s), 1);
    }
    return { animation: false, color: ["#60a5fa", "#f59e0b", "#34d399", "#f472b6", "#a78bfa", "#22d3ee"],
      tooltip: { trigger: "axis", valueFormatter: (value: number | null) => value != null ? value.toFixed(5) : "—" },
      legend: { type: "scroll", textStyle: { color: "#94a3b8" }, top: 0 },
      grid: { left: 65, right: 65, top: 50, bottom: 70 },
      xAxis: { type: "value", name: "s", min: 0, axisLabel: { color: "#94a3b8" }, splitLine: { lineStyle: { color: "rgba(148,163,184,.12)" } } },
      yAxis: units.map(unit => ({ type: "value", name: unit, scale: true, axisLabel: { color: "#94a3b8" }, splitLine: { lineStyle: { color: "rgba(148,163,184,.12)" } } })),
      dataZoom: [{ type: "inside", filterMode: "none" }, { type: "slider", height: 20, bottom: 10 }],
      series: series.map(item => ({ ...item, type: "line", showSymbol: false, connectNulls: false, lineStyle: item.lineStyle ?? { width: 1.5 }, sampling: item.step ? undefined : "lttb" })) };
  }, [data, analysis, chart, zh]);
  if (!data?.samples.length || !analysis) return <Empty description={tr("开始测试或导入 JSON 后显示曲线与自动分析", "Start a run or import JSON to view traces and analysis")} />;
  return <div className="el05-experiment-results">
    {reduced && <Alert type="info" showIcon message={analysis.quality.notes.includes(LEGACY_VELOCITY_ANALYSIS_NOTE)
      ? tr("历史曲线已抽样；反馈速度方向已修正，指标按当前保留样本重新计算。", "History traces are reduced. Feedback velocity direction is corrected and metrics are recalculated from retained samples.")
      : tr("本地历史的曲线已抽样；指标保留自原始完整数据分析。需要完整原始帧时，请在下一组开始前导出 JSON。", "Local history traces are reduced; metrics retain the original full-data analysis. Export JSON before the next run to keep every original frame.")} />}
    {data.status.static_friction && <Alert showIcon type={data.status.active ? "info" : staticFrictionPasses(data).every(pass => pass.outcome === "BreakawayDetected" && pass.return_settled !== false)
      && (data.status.config?.friction_direction !== "Both" || analysis.static_friction?.both_directions_valid) ? "success" : "warning"} message={staticFrictionStatusText(data.status, zh)} description={tr("正反方向分别保存最后静止、首次起动和确认帧，图中标记各自的斜坡、起动和回位。起动力矩保留正负号，统一为 Nm；回位数据不参与起动力矩计算。", "Each direction preserves its last-rest, first-motion and confirmation frames; the chart marks each ramp, onset and return. Breakaway torque retains its sign in Nm; return samples are excluded from the calculation.")} />}
    <div className="el05-experiment-quality">
      <Space wrap><Tag>{analysis.sample_count} {tr("分析样本", "analyzed samples")}</Tag><Tag>{analysis.duration_s.toFixed(2)} s</Tag><Tag>{analysis.quality.sample_rate_hz?.toFixed(1) ?? "—"} Hz</Tag><Tag>{tr("最大间隔", "Max gap")} {analysis.quality.max_gap_s?.toFixed(3) ?? "—"} s</Tag><Tag>{tr("丢弃帧", "Dropped")} {analysis.quality.dropped_frames}</Tag></Space>
    </div>
    <Segmented value={chart} onChange={value => setChart(String(value))} options={[
      ...(data.status.config?.kind === "StaticFriction" ? [{ value: "friction", label: tr("小角度静摩擦标定", "Small-angle static friction") }] : []),
      { value: "motion", label: tr("位置 / 速度", "Position / velocity") }, { value: "torque", label: tr("转矩 / 电流", "Torque / current") },
      { value: "error", label: tr("跟踪误差", "Tracking errors") }, { value: "thermal", label: tr("温升", "Temperature") }, ...(data.status.config?.kind === "SineSweep" ? [{ value: "frequency", label: tr("频率响应", "Frequency response") }] : []), ...(data.status.config?.kind === "StaticTorque" ? [{ value: "static", label: tr("转矩 / Iq 标定", "Torque / Iq fit") }] : [])]} />
    {chart === "frequency" && !analysis.frequency_response?.points.length && <Typography.Paragraph type="secondary" className="el05-experiment-notes">{analysis.frequency_response?.reason ?? tr("可辨识的完整周期不足，暂无法提取频率响应。", "Insufficient identifiable complete cycles for a frequency response.")}</Typography.Paragraph>}
    {chart === "torque" && data.status.config?.kind === "StaticTorque" && <Typography.Paragraph type={data.status.config.pendulum?.zero_measured ? "secondary" : "warning"} className="el05-experiment-notes">{data.status.config.pendulum?.zero_measured
      ? tr(`理论保持力矩和驱动反馈统一使用 ${EL05_TORQUE_UNIT}（N·m）。目标角理论保持力矩按目标角和本组负载几何计算；重力保持转矩按实际反馈角计算，均不含摩擦和惯性。与驱动反馈的差值可能包含静摩擦和驱动估计偏差。`, `Theoretical holding torque and drive feedback both use ${EL05_TORQUE_UNIT} (N·m). Target holding torque uses target angles and this run's load geometry; gravity holding torque uses measured angles. Both exclude friction and inertia. Differences from drive feedback may include static friction and drive estimation error.`)
      : tr("本组未测定下垂参考，无法绘制有效的目标角理论保持力矩；请在开始测试前测定下垂参考。", "This run has no measured downward reference, so target holding torque is unavailable. Measure the downward reference before starting a run.")}</Typography.Paragraph>}
    <ReactECharts option={chartOption} notMerge style={{ height: 350 }} />
    <div className="el05-experiment-analysis">
      {analysis.sections.map(section => <Card size="small" key={section.id} title={translated(section, "title", section.title, zh)}>
        <Table size="small" pagination={false} rowKey="key" dataSource={section.metrics} columns={[
          { title: tr("指标", "Metric"), key: "label", render: (_, metric) => metricLabel(metric, zh) },
          { title: tr("结果", "Result"), key: "value", render: (_, metric) => <span className="el05-number">{metricValue(metric)}</span> },
          { title: tr("说明", "Note"), dataIndex: "note", key: "note", render: (note, metric) => <Typography.Text type="secondary">{translated(metric, "note", note ?? "", zh)}</Typography.Text> },
        ]} />
        {!!section.notes.length && <Typography.Paragraph className="el05-experiment-notes" type="secondary">{section.notes.join(" ")}</Typography.Paragraph>}
      </Card>)}
    </div>
    <El05ExperimentDetails analysis={analysis} zh={zh} />
    <Descriptions size="small" column={2} className="el05-experiment-source">
      <Descriptions.Item label={tr("数据来源", "Telemetry")}>{tr("主机单调接收时间；驱动转矩估计", "Host monotonic receive time; drive torque estimate")}</Descriptions.Item>
      <Descriptions.Item label={tr("电流有效样本", "Fresh current samples")}>{analysis.quality.fresh_current_samples}</Descriptions.Item>
    </Descriptions>
    <Typography.Paragraph type="secondary" className="el05-experiment-notes">{[...analysis.quality.notes, ...analysis.limitations].join(" ")}</Typography.Paragraph>
  </div>;
}


function El05ExperimentDetails({ analysis, zh }: { analysis: ExperimentAnalysis; zh: boolean }) {
  const tr = (cn: string, en: string) => zh ? cn : en;
  const fmt = (value: number | null | undefined, digits = 4) => value != null && Number.isFinite(value) ? value.toFixed(digits) : "—";
  const items = [];
  if (analysis.kind === "StaticTorque" && analysis.static_torque?.platforms?.length) items.push({ key: "static", label: tr("逐平台静态结果", "Static plateau results"), children:
    <Table size="small" pagination={false} scroll={{ x: 780 }} rowKey="start_s" dataSource={analysis.static_torque.platforms} columns={[
      { title: "t (s)", dataIndex: "start_s", render: value => fmt(value, 2) },
      { title: tr("目标角 (rad)", "Target (rad)"), dataIndex: "target_angle_rad", render: value => fmt(value) },
      { title: tr("实际角 (rad)", "Angle (rad)"), dataIndex: "angle_rad", render: value => fmt(value) },
      { title: `${tr("理论重力矩", "Theoretical gravity torque")} (${EL05_TORQUE_UNIT})`, dataIndex: "gravity_torque_nm", render: value => fmt(value) },
      { title: "Iq (A)", dataIndex: "current_a", render: value => fmt(value) },
      { title: `${tr("驱动反馈力矩", "Drive feedback torque")} (${EL05_TORQUE_UNIT})`, dataIndex: "feedback_torque_nm", render: value => fmt(value) },
      { title: tr("接近方向", "Approach"), dataIndex: "approach", render: value => value === "positive" ? "+" : value === "negative" ? "−" : "—" },
      { title: tr("有效 Iq 样本", "Fresh Iq samples"), dataIndex: "fresh_current_samples" },
    ]} /> });
  const position = analysis.kind !== "VelocitySteps";
  const steps = position ? analysis.tracking?.position_steps : analysis.tracking?.velocity_steps;
  if (steps?.length) items.push({ key: "steps", label: tr("逐阶跃响应结果", "Individual step responses"), children:
    <Table size="small" pagination={false} scroll={{ x: 780 }} rowKey="start_s" dataSource={steps} columns={[
      { title: "t (s)", dataIndex: "start_s", render: value => fmt(value, 2) },
      { title: tr("阶跃", "Step") + (position ? " (rad)" : " (rad/s)"), key: "step", render: (_, row) => `${fmt(row.from)} → ${fmt(row.to)}` },
      { title: tr("上升时间 (s)", "Rise (s)"), dataIndex: "rise_time_s", render: value => fmt(value) },
      { title: tr("超调 (%)", "Overshoot (%)"), dataIndex: "overshoot_percent", render: value => fmt(value, 2) },
      { title: tr("稳定时间 (s)", "Settling (s)"), dataIndex: "settling_time_s", render: value => fmt(value) },
      { title: tr("稳态误差", "Steady error"), dataIndex: "steady_error", render: value => fmt(value) },
      { title: tr("平台完整", "Complete plateau"), dataIndex: "complete", render: value => value ? tr("是", "Yes") : tr("否", "No") },
    ]} /> });
  if (analysis.kind === "SineSweep" && analysis.frequency_response?.points?.length) items.push({ key: "frequency", label: tr("逐频率响应测点", "Frequency response points"), children:
    <Table size="small" pagination={false} scroll={{ x: 650 }} rowKey="frequency_hz" dataSource={analysis.frequency_response.points} columns={[
      { title: "Hz", dataIndex: "frequency_hz", render: value => fmt(value, 3) },
      { title: tr("增益 (dB)", "Gain (dB)"), dataIndex: "gain_db", render: value => fmt(value, 3) },
      { title: tr("相位 (°)", "Phase (°)"), dataIndex: "phase_deg", render: value => fmt(value, 2) },
      { title: tr("输入幅值 (rad)", "Input amplitude (rad)"), dataIndex: "input_amplitude_rad", render: value => fmt(value) },
      { title: tr("输出幅值 (rad)", "Output amplitude (rad)"), dataIndex: "output_amplitude_rad", render: value => fmt(value) },
      { title: tr("周期数", "Cycles"), dataIndex: "cycles", render: value => fmt(value, 2) },
    ]} /> });
  if (analysis.kind === "Impedance" && analysis.impedance?.responses?.length) items.push({ key: "impedance", label: tr("逐次注入响应", "Individual injection responses"), children:
    <Table size="small" pagination={false} rowKey="start_s" dataSource={analysis.impedance.responses} columns={[
      { title: tr("起始 (s)", "Start (s)"), dataIndex: "start_s", render: value => fmt(value, 2) },
      { title: tr("结束 (s)", "End (s)"), dataIndex: "end_s", render: value => fmt(value, 2) },
      { title: tr("峰值角偏移 (rad)", "Peak deflection (rad)"), dataIndex: "peak_displacement_rad", render: value => fmt(value) },
      { title: tr("撤去后恢复 (s)", "Recovery (s)"), dataIndex: "recovery_s", render: value => fmt(value) },
    ]} /> });
  return items.length ? <Collapse className="el05-experiment-details" items={items} /> : null;
}
