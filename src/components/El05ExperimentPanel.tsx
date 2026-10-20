import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Alert, App, Button, Card, Checkbox, Collapse, Descriptions, Input, InputNumber, Progress, Select, Space, Table, Tag, Typography } from "antd";
import { el05Api, type El05ExperimentConfig, type El05ExperimentData, type El05ExperimentKind, type El05State } from "../el05Api";
import { analyzeExperiment, EL05_TORQUE_LIMIT_NM, EL05_TORQUE_UNIT, experimentCsv, experimentJson, pendulumModel, staticHoldingTorquePeakNm } from "../el05Experiment";
import { errMsg } from "../api";
import { El05ExperimentResults, experimentReport, loadRecords, makeRecord, metricLabel, metricValue, parseExperimentJson, runKey, staticFrictionStatusText, storeRecords, thin, type ExperimentRecord } from "./El05ExperimentResults";

const KINDS: { value: El05ExperimentKind; cn: string; en: string; description: [string, string]; preset: Partial<El05ExperimentConfig> }[] = [
  { value: "Record", cn: "自由衰减 / 被动记录", en: "Free decay / passive recording", description: ["自动停机后记录。手动拨动并释放摆锤，估算周期、阻尼比与衰减，并结合重力和惯量辨识摩擦。", "Automatically stops the drive and records. Deflect and release the pendulum to estimate period, damping and decay; gravity and inertia support friction estimation."], preset: { duration_s: 20 } },
  { value: "StaticFriction", cn: "静摩擦标定 · 双向小角度起动", en: "Static friction · bidirectional breakaway", description: ["将摆锤自然垂直下垂后开始。默认依次测正、反两个方向：确认静止，从零缓慢增加转矩；检测到微小起动便撤去斜坡，回位并重新确认静止后测反向，最后回位停止。两个方向的起动事件独立保存。", "Start with the pendulum naturally hanging down. By default tests positive then negative: confirms rest, ramps torque from zero, detects small motion, returns and confirms rest again before reversing. Returns and stops after the second direction; each direction keeps independent onset events."], preset: { duration_s: 45, friction_direction: "Both", torque_nm: 0.6, torque_ramp_rate_nm_s: 0.02, max_excursion_rad: 3 * Math.PI / 180, motion_threshold_rad: 0.15 * Math.PI / 180, motion_confirm_s: 0.04, settle_time_s: 0.5, kp: 8, kd: 0.8, gravity_compensation: false } },
  { value: "VelocitySteps", cn: "正反速度阶跃", en: "Bidirectional velocity steps", description: ["按静止→正转→静止→反转→静止运行，比较速度误差、波动及正反差异。MIT 速度响应由 Kd 决定，无速度积分项。", "Runs rest → positive → rest → negative → rest. Compares speed errors, ripple and directional differences. MIT speed response uses Kd without an integral term."], preset: { duration_s: 20, velocity_rad_s: 0.3, kd: 0.5 } },
  { value: "TorqueRamp", cn: "转矩斜坡 / 起动辨识", en: "Torque ramp / breakaway", description: ["前馈转矩从零线性增加至设定值，检测起动时刻及起动转矩。Kp、Kd 为零；开始运动后斜坡仍继续。", "Ramps feedforward torque from zero to the signed endpoint, identifying motion onset and breakaway torque. Kp and Kd are zero; the ramp continues after motion starts."], preset: { duration_s: 10, torque_nm: 0.3 } },
  { value: "StaticTorque", cn: "静态多角度转矩标定", en: "Multi-angle static torque", description: [`在中心及 ±半幅、±全幅间分 12 段悬停，同角度含不同接近方向。理论重力矩与驱动反馈统一使用 ${EL05_TORQUE_UNIT}（N·m）；用稳定段的重力矩与 Iq 拟合有效转矩系数和偏置，并对照驱动反馈。`, `Holds 12 segments at center, half and full amplitudes in both directions. Theoretical gravity torque and drive feedback both use ${EL05_TORQUE_UNIT} (N·m). Fits effective torque constant and bias from settled gravity torque and Iq, with drive feedback for comparison.`], preset: { duration_s: 72, kp: 12, kd: 0.5, position_amplitude_rad: 0.6 } },
  { value: "PositionSteps", cn: "带载位置阶跃", en: "Loaded position steps", description: ["中心→正幅值→中心→负幅值→中心，自动计算上升时间、超调、稳定时间及稳态误差，可更换配重比较惯量适应能力。", "Steps center → positive → center → negative → center. Computes rise time, overshoot, settling and steady error; compare different loads and inertias."], preset: { duration_s: 20, kp: 8, kd: 0.5, position_amplitude_rad: 0.25 } },
  { value: "SineSweep", cn: "正弦扫频 / 闭环频响", en: "Sine sweep / frequency response", description: ["位置与解析速度目标按线性扫频输出，提取增益、相位和可辨识的带宽。低频分析段需至少 3 个周期，默认 0.2→2 Hz、10 分钟；缩短可查看部分频响，但可能无法测出带宽。", "Applies a linear position chirp with analytic velocity targets. Extracts gain, phase and identifiable bandwidth. Low-frequency windows need at least 3 cycles: the default is 0.2–2 Hz over 10 minutes. Shorter runs can show partial frequency response while bandwidth may remain unavailable."], preset: { duration_s: 600, kp: 8, kd: 0.5, position_amplitude_rad: 0.15, frequency_start_hz: 0.2, frequency_end_hz: 2 } },
  { value: "Impedance", cn: "阻抗 / 扰动恢复", en: "Impedance / disturbance recovery", description: ["保持中心位置，在时长 30–35% 和 65–70% 分别叠加正、负前馈扰动，测峰值角偏移和撤扰恢复，也可手动推扰观察。", "Holds center and adds positive/negative feedforward disturbances at 30–35% and 65–70% of the run. Measures peak deflection and recovery; manual disturbances can also be observed."], preset: { duration_s: 30, kp: 8, kd: 0.5, disturbance_torque_nm: 0.15 } },
  { value: "Backdrive", cn: "零转矩反驱", en: "Zero-torque backdrive", description: ["使能 MIT，Kp=0、前馈转矩=0，仅施加所设 Kd。手动拨动摆锤，与自由衰减对比反驱阻力、耗散和不同阻尼设置。", "Enables MIT with Kp=0 and zero feedforward torque, retaining the chosen Kd. Move the pendulum by hand and compare resistance and dissipation with passive decay or other Kd values."], preset: { duration_s: 20, kd: 0 } },
  { value: "ThermalHold", cn: "持续承载 / 温升", en: "Loaded hold / temperature", description: ["在设定中心角持续悬停，记录温升速率、有效电流、位置漂移和热态保持误差。可设最长 3600 s。", "Continuously holds the center angle and measures heating rate, current, position drift and hot holding error. Runs can last up to 3600 s."], preset: { duration_s: 300, kp: 12, kd: 0.5, center_angle_rad: 0.6 } },
  { value: "SlowTriangle", cn: "低速往返 / 摩擦辨识", en: "Slow traverse / friction", description: ["在整段时长内走中心→正幅值→负幅值→中心，测低速波动、换向迟滞并拟合库仑与粘性摩擦。单向匀速段的目标速度为 4×幅值/时长。", "Traverses center → positive → negative → center over one run. Measures low-speed ripple and reversal lag, and fits Coulomb/viscous friction. Traverse speed is 4 × amplitude / duration."], preset: { duration_s: 40, kp: 8, kd: 0.5, position_amplitude_rad: 0.5 } },
];
const DEFAULT_CONFIG: El05ExperimentConfig = { kind: "Record", duration_s: 20, velocity_rad_s: 0.3, torque_nm: 0.3, kp: 8, kd: 0.5, center_angle_rad: 0, position_amplitude_rad: 0.25, frequency_start_hz: 0.1, frequency_end_hz: 2, gravity_compensation: false, disturbance_torque_nm: 0.15, max_temperature_c: 0, torque_ramp_rate_nm_s: 0.02, max_excursion_rad: 3 * Math.PI / 180, motion_threshold_rad: 0.15 * Math.PI / 180, motion_confirm_s: 0.04, settle_time_s: 0.5 };
const POSITION_KINDS: El05ExperimentKind[] = ["StaticTorque", "PositionSteps", "SineSweep", "Impedance", "ThermalHold", "SlowTriangle"];

export function El05ExperimentPanel({ state, zh, busy, coordinateRevision, refresh, onWorkingChange }: { state: El05State | null; zh: boolean; busy: boolean; coordinateRevision: number; refresh: () => Promise<void>; onWorkingChange: (working: boolean) => void }) {
  const tr = (cn: string, en: string) => zh ? cn : en;
  const { message } = App.useApp();
  const [working, setWorkingState] = useState(false);
  const setWorking = (next: boolean) => { setWorkingState(next); onWorkingChange(next); };
  const [stopping, setStopping] = useState(false);
  const [config, setConfig] = useState<El05ExperimentConfig>(DEFAULT_CONFIG);
  const [mass, setMass] = useState(1);
  const [lengthMm, setLengthMm] = useState(180);
  const [armMass, setArmMass] = useState(0);
  const [extraInertia, setExtraInertia] = useState(0);
  const [downwardZero, setDownwardZero] = useState(0);
  const [zeroMeasured, setZeroMeasured] = useState(false);
  const [live, setLive] = useState<El05ExperimentData | null>(null);
  const liveRef = useRef<El05ExperimentData | null>(null);
  const [records, setRecords] = useState<ExperimentRecord[]>(loadRecords);
  const recordsRef = useRef(records);
  const [selected, setSelected] = useState("live");
  const [runName, setRunName] = useState("");
  const [pollError, setPollError] = useState<string | null>(null);
  const [compareKeys, setCompareKeys] = useState<string[]>([]);
  const inputRef = useRef<HTMLInputElement>(null);
  const pollGeneration = useRef(0);
  const mounted = useRef(true);
  useEffect(() => { mounted.current = true; return () => { mounted.current = false; pollGeneration.current++; }; }, []);
  useEffect(() => { setZeroMeasured(false); }, [state?.connected, state?.motor_id, state?.identity?.uid, coordinateRevision]);
  const runNameRef = useRef(runName);
  runNameRef.current = runName;

  const persist = useCallback((next: ExperimentRecord[]) => {
    try {
      const stored = storeRecords(next);
      recordsRef.current = stored.records;
      setRecords(stored.records);
      if (!stored.persisted) message.warning(zh ? "本地历史存储不可用，记录暂存在内存中，请导出 JSON。" : "Local history storage is unavailable. Records remain in memory; export JSON to keep them.");
    } catch {
      recordsRef.current = next.slice(0, 6);
      setRecords(recordsRef.current);
      message.warning(zh ? "浏览器本地历史存储已满，本次记录暂存在内存中，请导出 JSON。" : "Local history storage is full. This record remains in memory; export JSON to keep it.");
    }
  }, [message, zh]);
  const archive = useCallback((data: El05ExperimentData) => {
    if (!data.samples.length || !data.status.config) return;
    const id = runKey(data);
    const previous = recordsRef.current.find(item => item.id === id);
    if (previous && previous.source_samples === data.samples.length && previous.data.status.reason === data.status.reason) return;
    const kind = KINDS.find(item => item.value === data.status.config?.kind);
    const record = makeRecord(data, previous?.name ?? (runNameRef.current.trim() || `${zh ? kind?.cn : kind?.en} · ${new Date().toLocaleString()}`));
    persist([record, ...recordsRef.current.filter(item => item.id !== id)]);
  }, [persist, zh]);
  const acceptChunk = useCallback((chunk: El05ExperimentData) => {
    if (!mounted.current || !chunk.status.config) return;
    const previous = liveRef.current;
    let next = chunk;
    if (previous && runKey(previous) === runKey(chunk)) {
      const cursor = previous.samples[previous.samples.length - 1]?.sequence ?? -1;
      const commandCursor = previous.commands[previous.commands.length - 1]?.sequence ?? -1;
      next = { ...chunk, samples: thin([...previous.samples, ...chunk.samples.filter(sample => sample.sequence > cursor)], 120_000),
        commands: thin([...previous.commands, ...chunk.commands.filter(command => command.sequence > commandCursor)], 120_000) };
    } else if (previous?.samples.length) archive(previous);
    liveRef.current = next;
    setLive(next);
    if (!next.status.active && next.samples.length) archive(next);
  }, [archive]);
  const archiveRef = useRef(archive);
  archiveRef.current = archive;
  useEffect(() => () => { if (liveRef.current?.samples.length) archiveRef.current(liveRef.current); }, []);
  useEffect(() => { if (!state?.connected && liveRef.current?.samples.length) archive(liveRef.current); }, [state?.connected, archive]);
  const readData = useCallback(async (full = false) => {
    const generation = pollGeneration.current;
    const current = liveRef.current;
    let chunk = await el05Api.experimentData(full ? undefined : current?.samples[current.samples.length - 1]?.sequence);
    if (!full && current && runKey(current) !== runKey(chunk) && chunk.status.config) chunk = await el05Api.experimentData();
    if (generation === pollGeneration.current) acceptChunk(chunk);
  }, [acceptChunk]);
  useEffect(() => {
    if (!state?.connected || !state.attached) return;
    let disposed = false;
    let timer: number | undefined;
    const poll = async () => {
      try { await readData(); if (!disposed) setPollError(null); }
      catch (error) { if (!disposed) setPollError(errMsg(error)); }
      finally { if (!disposed) timer = window.setTimeout(poll, 650); }
    };
    void poll();
    return () => { disposed = true; if (timer != null) window.clearTimeout(timer); };
  }, [state?.connected, state?.attached, readData]);

  // Keep acquisition responsive; expensive derivation/plots update at most every two seconds during a run.
  const [analysisData, setAnalysisData] = useState<El05ExperimentData | null>(null);
  const lastAnalysisTime = useRef(0);
  useEffect(() => {
    if (!live) return;
    const remaining = 2000 - (performance.now() - lastAnalysisTime.current);
    const updateAnalysis = () => { lastAnalysisTime.current = performance.now(); setAnalysisData(liveRef.current); };
    if (!live.status.active || remaining <= 0) { updateAnalysis(); return; }
    const timer = window.setTimeout(updateAnalysis, remaining);
    return () => window.clearTimeout(timer);
  }, [live]);
  const status = state?.connected && state.experiment?.config ? state.experiment : live?.status ?? state?.experiment;
  const active = !!state?.connected && !!state.experiment?.active;
  const locked = busy || working || active || !!state?.gravity_compensation?.active;
  const kind = KINDS.find(item => item.value === config.kind)!;
  const positional = POSITION_KINDS.includes(config.kind);
  const staticFriction = config.kind === "StaticFriction";
  const statusConfig = status?.config ?? config;
  const frictionStatus = status?.static_friction;
  const directionCount = statusConfig.friction_direction === "Both" ? 2 : 1;
  const plannedDuration = statusConfig.kind === "StaticFriction" ? (statusConfig.duration_s + 3) * directionCount : statusConfig.duration_s;
  const progress = frictionStatus ? 100 * ((frictionStatus.completed_directions?.length ?? 0)
    + (frictionStatus.phase === "Complete" ? 1 : Math.min(0.99, Math.max(0, (status?.elapsed_s ?? 0) - (frictionStatus.started_s ?? 0)) / (statusConfig.duration_s + 3)))) / directionCount
    : 100 * (status?.elapsed_s ?? 0) / plannedDuration;
  const model = pendulumModel(mass, lengthMm / 1000, armMass);
  const staticHoldingPeak = staticHoldingTorquePeakNm({ ...config, pendulum: {
    mass_kg: mass, length_m: lengthMm / 1000, arm_mass_kg: armMass, downward_zero_rad: downwardZero, zero_measured: zeroMeasured,
  } });
  const selectedRecord = records.find(record => record.id === selected);
  const displayedData = selected === "live" ? analysisData ?? live : selectedRecord?.data ?? null;
  const liveAnalysis = useMemo(() => analysisData?.samples.length ? analyzeExperiment(analysisData) : null, [analysisData]);
  const analysis = selected === "live" ? liveAnalysis : selectedRecord?.analysis ?? null;
  const update = (key: keyof El05ExperimentConfig, value: number | boolean) => setConfig(previous => ({ ...previous, [key]: value }));
  const field = (label: string, key: keyof El05ExperimentConfig, min: number, max: number, step: number) => <label className="el05-field">
    <Typography.Text type="secondary">{label}</Typography.Text><InputNumber disabled={locked} value={config[key] as number} min={min} max={max} step={step} onChange={value => { if (value != null) update(key, value); }} /></label>;
  const angleField = (label: string, key: "max_excursion_rad" | "motion_threshold_rad", fallbackDeg: number, min: number, max: number, step: number) => <label className="el05-field">
    <Typography.Text type="secondary">{label}</Typography.Text><InputNumber disabled={locked} value={Number(((config[key] ?? fallbackDeg * Math.PI / 180) * 180 / Math.PI).toFixed(4))} min={min} max={max} step={step} onChange={value => {
      if (value == null) return;
      const radians = value * Math.PI / 180;
      setConfig(previous => ({ ...previous, [key]: radians, ...(key === "max_excursion_rad" && (previous.motion_threshold_rad ?? 0.15 * Math.PI / 180) >= radians ? { motion_threshold_rad: radians / 2 } : {}) }));
    }} /></label>;
  const download = (name: string, text: string, type: string) => {
    const url = URL.createObjectURL(new Blob([text], { type }));
    const link = document.createElement("a"); link.href = url; link.download = name; link.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  const exportData = async (format: "csv" | "json" | "md") => {
    if (selected === "live") { try { await readData(); } catch { /* The last accepted samples remain exportable. */ } }
    const data = selected === "live" ? liveRef.current : selectedRecord?.data;
    if (!data?.samples.length) return;
    const result = selected === "live" ? analyzeExperiment(data) : selectedRecord!.analysis;
    const name = selected === "live" ? runName.trim() || `${zh ? KINDS.find(item => item.value === data.status.config?.kind)?.cn : data.status.config?.kind}` : selectedRecord!.name;
    const stem = `el05-${data.motor_id}-${data.status.config?.kind}-${new Date().toISOString().replace(/[:.]/g, "-")}`;
    if (format === "csv") download(`${stem}.csv`, experimentCsv(data), "text/csv;charset=utf-8");
    else if (format === "md") download(`${stem}-report.md`, experimentReport(data, result, name, zh, selectedRecord?.reduced), "text/markdown;charset=utf-8");
    else {
      const exported = JSON.parse(experimentJson(data));
      exported.analysis = result;
      exported.ui_archive = { name, analysis: result, reduced: selectedRecord?.reduced ?? false, source_samples: selectedRecord?.source_samples ?? data.samples.length };
      download(`${stem}.json`, JSON.stringify(exported, null, 2), "application/json");
    }
  };
  const start = async () => {
    setWorking(true);
    try {
      if (liveRef.current?.samples.length) {
        try { await readData(); } catch { /* Keep the already captured data if the final incremental read is unavailable. */ }
        archive(liveRef.current);
      }
      pollGeneration.current++;
      await el05Api.startExperiment({ ...config, pendulum: { mass_kg: mass, length_m: lengthMm / 1000, arm_mass_kg: armMass,
        extra_inertia_kg_m2: extraInertia, downward_zero_rad: downwardZero, zero_measured: zeroMeasured } });
      setSelected("live");
      await refresh();
      await readData(true);
    } catch (error) { message.error(errMsg(error)); }
    finally { setWorking(false); }
  };
  const stop = async () => {
    setStopping(true);
    try { await el05Api.stop(); await refresh(); await readData(); }
    catch (error) { message.error(errMsg(error)); }
    finally { setStopping(false); }
  };
  const importJson = async (file: File) => {
    if (file.size > 120 * 1024 * 1024) { message.error(tr("JSON 文件超过 120 MB，请使用较小记录。", "JSON exceeds 120 MB; use a smaller record.")); return; }
    setWorking(true);
    try {
      const parsed = parseExperimentJson(await file.text());
      const record = makeRecord(parsed.data, file.name.replace(/\.json$/i, ""));
      record.id = `import-${Date.now()}`;
      if (parsed.analysis) record.analysis = parsed.analysis;
      record.source_samples = parsed.source_samples ?? record.source_samples;
      record.reduced = !!parsed.reduced || record.reduced;
      persist([record, ...recordsRef.current]);
      setSelected(record.id);
      message.success(tr("已导入并完成分析", "Imported and analyzed"));
    } catch (error) { message.error(errMsg(error)); }
    finally { setWorking(false); }
  };
  const reuseConfig = () => {
    if (!displayedData?.status.config) return;
    const previous = displayedData.status.config;
    setConfig({ ...DEFAULT_CONFIG, ...previous, origin_position_rad: undefined });
    if (previous.pendulum) {
      setMass(previous.pendulum.mass_kg); setLengthMm(previous.pendulum.length_m * 1000); setArmMass(previous.pendulum.arm_mass_kg);
      setExtraInertia(previous.pendulum.extra_inertia_kg_m2 ?? 0); setDownwardZero(previous.pendulum.downward_zero_rad); setZeroMeasured(previous.pendulum.zero_measured);
    }
    message.success(tr("已载入该组工况与几何参数", "Loaded this run's configuration and geometry"));
  };
  const comparable = useMemo(() => {
    const rows = records.filter(record => !analysisData || record.id !== runKey(analysisData)).map(record => ({ key: record.id, name: record.name, config: record.data.status.config, analysis: record.analysis }));
    if (analysisData && liveAnalysis) rows.unshift({ key: "live", name: tr("当前记录", "Current run"), config: analysisData.status.config, analysis: liveAnalysis });
    return rows;
  }, [records, analysisData, liveAnalysis, zh]);
  const metricOptions = useMemo(() => {
    const options = new Map<string, { value: string; label: string }>();
    for (const row of comparable) for (const section of row.analysis.sections) for (const metric of section.metrics) {
      if (metric.value != null) options.set(`${section.id}:${metric.key}`, { value: `${section.id}:${metric.key}`, label: metricLabel(metric, zh) });
    }
    return [...options.values()];
  }, [comparable, zh]);
  const performanceOptions = metricOptions.filter(option => !option.value.startsWith("quality:"));
  const selectedMetrics = compareKeys.length ? compareKeys : (performanceOptions.length ? performanceOptions : metricOptions).slice(0, 3).map(option => option.value);
  return <div className="el05-experiments">
    <Card size="small" title={tr("摆锤电机性能测试台", "Pendulum motor test bench")} extra={<Tag color={active ? "processing" : "default"}>{active ? tr("测试中", "Running") : tr("就绪", "Ready")}</Tag>}>
      <div className="el05-experiment-presets">
        {KINDS.map(item => <button type="button" disabled={locked} key={item.value} className={`el05-experiment-preset${config.kind === item.value ? " el05-experiment-preset--selected" : ""}`} onClick={() => setConfig(previous => ({ ...previous, center_angle_rad: 0, ...item.preset, kind: item.value }))}>{zh ? item.cn : item.en}</button>)}
      </div>
      <Typography.Paragraph className="el05-experiment-description">{kind.description[zh ? 0 : 1]}</Typography.Paragraph>
      {staticHoldingPeak !== null && <Alert showIcon type={staticHoldingPeak > EL05_TORQUE_LIMIT_NM ? "warning" : "info"}
        message={tr(`目标角最大理论保持力矩 ${staticHoldingPeak.toFixed(3)} ${EL05_TORQUE_UNIT} · EL05 峰值 ${EL05_TORQUE_LIMIT_NM} ${EL05_TORQUE_UNIT}`, `Peak target holding torque ${staticHoldingPeak.toFixed(3)} ${EL05_TORQUE_UNIT} · EL05 peak ${EL05_TORQUE_LIMIT_NM} ${EL05_TORQUE_UNIT}`)}
        description={staticHoldingPeak > EL05_TORQUE_LIMIT_NM
          ? tr("部分目标角的重力保持需求已超过 EL05 峰值，请减小摆角或负载。理论力矩与驱动反馈均为 Nm；驱动实际设置的力矩上限还可能更低。", "Some target angles require more gravity holding torque than the EL05 peak. Reduce the angle or load. Theory and drive feedback both use Nm; the configured drive limit may be lower.")
          : tr("理论值按负载几何计算，单位与驱动反馈相同；还需为摩擦和运动加减速留出力矩余量。", "The theoretical value uses load geometry in the same unit as drive feedback. Allow torque margin for friction and acceleration.")} />}
      <div className="el05-target__fields">
        <label className="el05-field el05-experiment-name"><Typography.Text type="secondary">{tr("本组名称（可选）", "Run name (optional)")}</Typography.Text><Input disabled={locked} value={runName} onChange={event => setRunName(event.target.value)} maxLength={100} placeholder={zh ? kind.cn : kind.en} /></label>
        {field(staticFriction ? tr("每方向超时 (s)", "Timeout per direction (s)") : tr("测试时长 (s)", "Duration (s)"), "duration_s", 2, 3600, 1)}
        {config.kind === "VelocitySteps" && field(tr("速度幅值 (rad/s)", "Speed amplitude (rad/s)"), "velocity_rad_s", 0.001, 50, 0.05)}
        {config.kind === "TorqueRamp" && field(tr("斜坡终点转矩 (Nm，带符号)", "Ramp endpoint (Nm, signed)"), "torque_nm", -6, 6, 0.05)}
        {staticFriction && <>
          <label className="el05-field"><Typography.Text type="secondary">{tr("测试方向", "Test directions")}</Typography.Text>
            <Select disabled={locked} value={config.friction_direction ?? "Both"} onChange={value => setConfig(previous => ({ ...previous, friction_direction: value }))} options={[
              { value: "Both", label: tr("双向：正向 → 反向", "Both: positive → negative") },
              { value: "Positive", label: tr("仅正向 (+)", "Positive (+) only") },
              { value: "Negative", label: tr("仅反向 (−)", "Negative (−) only") },
            ]} /></label>
          {field(tr("最大转矩幅值 (Nm)", "Maximum torque magnitude (Nm)"), "torque_nm", 0.001, 6, 0.05)}
          {field(tr("转矩增加速率 (Nm/s)", "Torque ramp rate (Nm/s)"), "torque_ramp_rate_nm_s", 0.001, 1, 0.005)}
          {angleField(tr("最大偏转角 (°)", "Maximum excursion (°)"), "max_excursion_rad", 3, 0.3, 30, 0.5)}
          {angleField(tr("起动检测位移 (°)", "Motion threshold (°)"), "motion_threshold_rad", 0.15, 0.05, Math.max(0.05, (config.max_excursion_rad ?? 3 * Math.PI / 180) * 180 / Math.PI - 0.01), 0.05)}
        </>}
        {positional && field(tr("中心角 (rad)", "Center angle (rad)"), "center_angle_rad", -12, 12, 0.05)}
        {positional && !["Impedance", "ThermalHold"].includes(config.kind) && field(tr("位置半幅值 (rad)", "Position amplitude (rad)"), "position_amplitude_rad", 0.001, 6, 0.05)}
        {positional && field("Kp (Nm/rad)", "kp", 0, 500, 0.5)}
        {!["Record", "TorqueRamp", "StaticFriction"].includes(config.kind) && field("Kd (Nm·s/rad)", "kd", 0, 5, 0.05)}
        {config.kind === "SineSweep" && <>{field(tr("起始频率 (Hz)", "Start frequency (Hz)"), "frequency_start_hz", 0.01, 5, 0.05)}{field(tr("终止频率 (Hz)", "End frequency (Hz)"), "frequency_end_hz", 0.01, 5, 0.1)}</>}
        {config.kind === "Impedance" && field(tr("扰动转矩幅值 (Nm)", "Disturbance amplitude (Nm)"), "disturbance_torque_nm", 0, 6, 0.05)}
        {field(tr("温度上限 (°C，0 关闭)", "Temperature limit (°C, 0 off)"), "max_temperature_c", 0, 135, 5)}
      </div>
      {staticFriction && <>
        <Typography.Paragraph type="secondary">{tr("默认起动检测仅 0.15°，角度上限默认 3°；通信与机械惯性可能产生少量越界。每方向有独立超时，回位最多另加 3 秒。双向测试共用转矩幅值和速率；回位未稳定或触发角度保护时停止后续方向。结果为起动静摩擦力矩估计（Nm），扣除重力仍需已测下垂参考。", "The default motion threshold is 0.15° and excursion limit is 3°; delay and inertia can cause some overshoot. Each direction has its own timeout plus up to 3 seconds for return. Both directions use the same torque magnitude and ramp rate. Unstable return or an angle guard ends further testing. Results estimate breakaway torque (Nm); gravity correction requires a measured downward reference.")}</Typography.Paragraph>
        <Collapse ghost items={[{ key: "static-friction-timing", label: tr("标定时序与回位参数", "Calibration timing & return parameters"), children: <>
          <div className="el05-target__fields">
            {field(tr("连续静止时间 (s)", "Required rest duration (s)"), "settle_time_s", 0.2, 5, 0.1)}
            {field(tr("起动确认时间 (s)", "Motion confirmation (s)"), "motion_confirm_s", 0.01, 0.5, 0.01)}
            {field(tr("回位 Kp (Nm/rad)", "Return Kp (Nm/rad)"), "kp", 0.1, 500, 0.5)}
            {field(tr("回位 Kd (Nm·s/rad)", "Return Kd (Nm·s/rad)"), "kd", 0.01, 5, 0.05)}
          </div>
          <Typography.Text type="secondary">{tr("斜坡期间 Kp、Kd 均为 0，无重力补偿；回位参数只在检测起动或结束加扭矩后使用。检测位移需小于最大偏转角。", "During the ramp Kp and Kd are zero and gravity compensation is off. Return gains apply only after motion is detected or the ramp ends. The motion threshold must be smaller than the excursion limit.")}</Typography.Text>
        </> }]} />
      </>}
      {positional && <Typography.Paragraph type="secondary">{zeroMeasured ? tr("中心角相对于已测下垂参考；例如 0 为下垂、1.571 为水平。", "Center angle is relative to the measured downward reference: 0 is down and 1.571 is horizontal.") : tr("未测下垂参考：中心角相对于本组开始时的位置。", "Without a downward reference, the center angle is relative to the position at run start.")}</Typography.Paragraph>}
      {!["Record", "Backdrive", "StaticFriction"].includes(config.kind) && <Space wrap className="el05-experiment-compensation"><Checkbox disabled={locked} checked={config.gravity_compensation} onChange={event => update("gravity_compensation", event.target.checked)}>{tr("前馈补偿摆锤重力", "Feedforward gravity compensation")}</Checkbox><Typography.Text type="secondary">{tr("使用已测下垂参考与本组几何；参考未测时不施加补偿。", "Uses this run's geometry and measured downward reference; inactive when the reference is unmeasured.")}</Typography.Text></Space>}
      <Collapse ghost className="el05-experiment-geometry" items={[{ key: "geometry", label: tr(`摆锤几何与参考 · 重力矩 ${model.gravityNm.toFixed(3)} Nm · 惯量 ${(model.inertiaKgM2 + extraInertia).toFixed(5)} kg·m²`, `Pendulum geometry & reference · gravity ${model.gravityNm.toFixed(3)} Nm · inertia ${(model.inertiaKgM2 + extraInertia).toFixed(5)} kg·m²`), children: <>
        <div className="el05-target__fields">
          <label className="el05-field"><Typography.Text type="secondary">{tr("末端质量 (kg)", "End mass (kg)")}</Typography.Text><InputNumber disabled={locked} min={0} max={20} step={0.05} value={mass} onChange={value => setMass(value ?? 0)} /></label>
          <label className="el05-field"><Typography.Text type="secondary">{tr("轴至负重重心 (mm)", "Axis to load CoM (mm)")}</Typography.Text><InputNumber disabled={locked} min={1} max={2000} value={lengthMm} onChange={value => setLengthMm(value ?? 180)} /></label>
          <label className="el05-field"><Typography.Text type="secondary">{tr("均匀力臂质量 (kg)", "Uniform arm mass (kg)")}</Typography.Text><InputNumber disabled={locked} min={0} max={20} step={0.01} value={armMass} onChange={value => setArmMass(value ?? 0)} /></label>
          <label className="el05-field"><Typography.Text type="secondary">{tr("额外惯量 (kg·m²)", "Additional inertia (kg·m²)")}</Typography.Text><InputNumber disabled={locked} min={0} max={10} step={0.0001} value={extraInertia} onChange={value => setExtraInertia(value ?? 0)} /></label>
          <label className="el05-field"><Typography.Text type="secondary">{tr("下垂参考 θ₀ (rad)", "Downward reference θ₀ (rad)")}</Typography.Text><InputNumber disabled={locked} min={-1e6} max={1e6} step={0.001} value={downwardZero} onChange={value => { if (value != null) { setDownwardZero(value); setZeroMeasured(true); } }} /></label>
        </div>
        <Space wrap><Button disabled={locked || state?.unwrapped_position_rad == null} onClick={() => { setDownwardZero(state!.unwrapped_position_rad!); setZeroMeasured(true); }}>{tr("将当前角记为下垂参考", "Use current angle as downward reference")}</Button><Checkbox disabled={locked} checked={zeroMeasured} onChange={event => setZeroMeasured(event.target.checked)}>{tr("下垂参考已测定", "Downward reference measured")}</Checkbox><Typography.Text type="secondary">θ={zeroMeasured && state?.unwrapped_position_rad != null ? (state.unwrapped_position_rad - downwardZero).toFixed(4) : "—"} rad</Typography.Text></Space>
        <Typography.Paragraph type="secondary" className="el05-experiment-notes">{tr("把摆锤放在竖直向下的位置后取当前角。该参考只用于中心角、重力模型和分析，不修改电机机械零位。开始时冻结本组参数；重连、更换电机或修改机械零位后重新测定。模型 J=mL²+maL²/3+额外惯量，重力矩=(m+ma/2)gL·sin(θ)。", "Place the pendulum vertically down before capturing its angle. This reference defines center angles, gravity and analysis without changing the drive zero. Values are frozen at run start; remeasure after reconnecting, changing motors or changing the drive zero. J=mL²+maL²/3+additional inertia; gravity torque=(m+ma/2)gL·sin(θ).")}</Typography.Paragraph>
      </> }]} />
      <div className="el05-experiment-actions"><Button type="primary" size="large" disabled={!state?.online || locked} loading={working} onClick={() => void start()}>{staticFriction ? tr("自然下垂后开始标定", "Start calibration from natural rest") : tr("一键开始测试", "Start experiment")}</Button><Button danger type="primary" size="large" disabled={!state?.attached} loading={stopping} onClick={() => void stop()}>{tr("停止测试和电机", "Stop experiment & motor")}</Button><Typography.Text type="secondary">{tr("自动准备模式并使能；结束后停机。", "Automatically prepares and enables the drive; stops at completion.")}</Typography.Text></div>
      <div className="el05-experiment-status"><Typography.Text>{status?.static_friction ? staticFrictionStatusText(status, zh) : status?.active ? tr("记录中", "Recording") : status?.reason ?? tr("尚未开始", "No run yet")} · {(status?.elapsed_s ?? 0).toFixed(2)} / {plannedDuration} s · {status?.samples ?? 0} {tr("帧", "frames")}</Typography.Text>{status?.config && <Progress percent={Math.min(100, progress)} status={status.active ? "active" : "normal"} showInfo={false} size="small" />}</div>
      {status?.recording_interval_s != null && status.recording_interval_s > 0.002 && <Typography.Text type="secondary">{tr("长时记录最小间隔：", "Long-run minimum recording interval: ")}{(status.recording_interval_s * 1000).toFixed(1)} ms</Typography.Text>}
      {pollError && <Typography.Paragraph type="warning">{tr("数据读取：", "Data read: ")}{pollError}</Typography.Paragraph>}
    </Card>
    <Card size="small" title={tr("曲线与自动结果", "Traces & automatic results")} extra={<Tag>{selected === "live" ? tr("当前记录", "Current run") : tr("本地历史", "Local history")}</Tag>}>
      <div className="el05-experiment-toolbar"><Select style={{ minWidth: 0, flex: "1 1 270px", width: "100%" }} value={selected} onChange={setSelected} options={[{ value: "live", label: tr("当前记录", "Current run") }, ...records.map(record => ({ value: record.id, label: record.name }))]} /><Space wrap><Button disabled={!displayedData?.samples.length} onClick={() => void exportData("csv")}>CSV</Button><Button disabled={!displayedData?.samples.length} onClick={() => void exportData("json")}>JSON</Button><Button disabled={!analysis} onClick={() => void exportData("md")}>{tr("导出报告", "Export report")}</Button><Button disabled={working} onClick={() => inputRef.current?.click()}>{tr("导入 JSON", "Import JSON")}</Button><Button disabled={locked || !displayedData?.status.config} onClick={reuseConfig}>{tr("复用参数", "Reuse configuration")}</Button></Space></div>
      <input ref={inputRef} hidden type="file" accept=".json,application/json" onChange={event => { const file = event.target.files?.[0]; if (file) void importJson(file); event.target.value = ""; }} />
      {selectedRecord && <Space wrap className="el05-experiment-record-meta"><Typography.Text type="secondary">{new Date(selectedRecord.saved_at).toLocaleString()} · {selectedRecord.data.status.config?.kind} · {selectedRecord.source_samples} {tr("原始分析样本", "original analysis samples")}</Typography.Text><Button size="small" danger onClick={() => { persist(recordsRef.current.filter(record => record.id !== selected)); setSelected("live"); }}>{tr("删除此历史", "Delete this record")}</Button></Space>}
      <El05ExperimentResults data={displayedData} analysis={analysis} zh={zh} reduced={selectedRecord?.reduced} />
    </Card>
    <Card size="small" title={tr("本地历史与性能比较", "Local history & comparison")}>
      <Typography.Paragraph type="secondary">{tr("每组结束或开始下一组前自动保存，保留最近 6 组。历史曲线按整段时间抽样，原始分析结果保留；完整帧请导出 JSON。可将不同负重、参数和冷/热状态的指标并排比较。", "Runs are saved at completion and before the next run, retaining the latest 6. History traces are reduced over the full duration while original analysis results are kept; export JSON for every frame. Compare different loads, gains and cold/hot conditions.")}</Typography.Paragraph>
      <Select mode="multiple" maxCount={4} style={{ width: "100%", marginBottom: 12 }} value={selectedMetrics} placeholder={tr("选择比较指标", "Choose comparison metrics")} onChange={setCompareKeys} options={metricOptions} />
      <Table size="small" pagination={false} scroll={{ x: 850 }} rowKey="key" dataSource={comparable} locale={{ emptyText: tr("完成一组测试后可比较结果", "Complete a run to compare results") }} columns={[
        { title: tr("记录", "Run"), dataIndex: "name", key: "name", width: 240, render: (name, row) => <Button className="el05-experiment-history-name" type="link" onClick={() => setSelected(row.key)}>{name}</Button> },
        { title: tr("质量 / 力臂", "Mass / arm"), key: "geometry", render: (_, row) => row.config?.pendulum ? `${row.config.pendulum.mass_kg} kg / ${(row.config.pendulum.length_m * 1000).toFixed(0)} mm` : "—" },
        ...selectedMetrics.map(key => ({ title: metricOptions.find(option => option.value === key)?.label ?? key, key, render: (_: unknown, row: typeof comparable[number]) => { const metric = row.analysis.sections.flatMap(section => section.metrics.map(item => ({ ...item, scopedKey: `${section.id}:${item.key}` }))).find(item => item.scopedKey === key); return metric ? metricValue(metric) : "—"; } })),
      ]} />
    </Card>
  </div>;
}
