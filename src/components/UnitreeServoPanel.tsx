import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { Alert, App, AutoComplete, Button, Card, Descriptions, Empty, InputNumber, Segmented, Select, Space, Switch, Tag, Tooltip, Typography } from "antd";
import ReactECharts from "echarts-for-react";
import { UnitreeBusTestPanel } from "./UnitreeBusTestPanel";
import { errMsg } from "../api";
import { useI18n } from "../i18n";
import { unitreeApi, type UnitreeState } from "../unitreeApi";
import {
  DEFAULT_UNITREE_SETTINGS, DEFAULT_UNITREE_TARGET, UNITREE_PROFILE_KEY,
  parseUnitreeProfile, validateUnitreeId, validateUnitreeIdChange, validateUnitreeSettings, validateUnitreeTarget,
  type UnitreeProfile, type UnitreeSettings, type UnitreeTarget,
} from "../unitreeServo";
import "./UnitreeServoPanel.css";

function initialProfile(): UnitreeProfile {
  try {
    const saved = localStorage.getItem(UNITREE_PROFILE_KEY);
    if (saved) return parseUnitreeProfile(saved);
  } catch { /* Invalid or inaccessible storage falls back to validated defaults. */ }
  return {
    schema_version: 1,
    port: /linux/i.test(navigator.userAgent) ? "/dev/ttyACM0" : "COM3",
    selected_id: 0,
    settings: { ...DEFAULT_UNITREE_SETTINGS },
    target: { ...DEFAULT_UNITREE_TARGET },
  };
}

interface Sample { t: number; position: number | null; velocity: number | null; torque: number | null }

export function UnitreeServoPanel({ onBusyChange }: { onBusyChange?: (busy: boolean) => void }) {
  const { lang } = useI18n();
  const zh = lang === "zh";
  const tr = (cn: string, en: string) => zh ? cn : en;
  const { message } = App.useApp();
  const [profile, setProfile] = useState(initialProfile);
  const [state, setState] = useState<UnitreeState | null>(null);
  const [ports, setPorts] = useState<{ name: string; description: string }[]>([]);
  const [busy, setBusy] = useState(false);
  const [stopping, setStopping] = useState(false);
  const [pollError, setPollError] = useState<string | null>(null);
  const [tab, setTab] = useState("motion");
  const [view, setView] = useState("numeric");
  const [repeat, setRepeat] = useState(true);
  const [workMode, setWorkMode] = useState(1);
  const [newId, setNewId] = useState<number | null>(null);
  const [recoveryId, setRecoveryId] = useState<number | null>(null);
  const [chartRevision, setChartRevision] = useState(0);
  const [rates, setRates] = useState({ tx: 0, rx: 0 });
  const alive = useRef(true);
  const action = useRef(false);
  const stopInFlight = useRef(false);
  const epoch = useRef(0);
  const samples = useRef<Sample[]>([]);
  const sampleState = useRef({ id: -1, sequence: -1, online: false });
  const lastRates = useRef({ t: performance.now(), tx: 0, rx: 0 });
  const fileInput = useRef<HTMLInputElement>(null);
  const connected = !!state?.connected;
  const active = !!state?.motion_active;
  const changingId = !!state?.id_change && !["complete", "failed"].includes(state.id_change.phase);
  const busRunning = !!state?.bus_test?.running;
  const locked = busy || stopping || changingId || busRunning;
  const settingsDirty = connected && JSON.stringify(state.settings) !== JSON.stringify(profile.settings);
  const ready = connected && !!state?.online && !state.feedback?.error_bits && !state.feedback?.timed_out && !settingsDirty && !state.bootloader_ids?.length;
  const idle = !active && (state?.stop_confirmed ?? true);

  useEffect(() => { alive.current = true; return () => { alive.current = false; }; }, []);
  useEffect(() => { onBusyChange?.(locked); return () => onBusyChange?.(false); }, [locked, onBusyChange]);

  const acceptState = useCallback((next: UnitreeState | null) => {
    if (!alive.current) return;
    const previous = sampleState.current;
    const id = next?.selected_id ?? -1;
    if (id !== previous.id || !next) {
      samples.current = [];
      previous.id = id;
      previous.sequence = -1;
      previous.online = false;
      if (id >= 0) setProfile(p => ({ ...p, selected_id: id }));
    }
    if (next?.online && next.feedback && next.feedback_time_s != null && next.feedback_sequence !== previous.sequence) {
      samples.current.push({ t: next.feedback_time_s, position: next.feedback.position_rad, velocity: next.feedback.velocity_rad_s, torque: next.feedback.torque_nm });
    } else if (previous.online && next && !next.online) {
      samples.current.push({ t: (next.feedback_time_s ?? 0) + (next.feedback_age_ms ?? 0) / 1000, position: null, velocity: null, torque: null });
    }
    previous.sequence = next?.feedback_sequence ?? -1;
    previous.online = !!next?.online;
    if (samples.current.length > 600) samples.current.splice(0, samples.current.length - 600);
    const now = performance.now();
    const rate = lastRates.current;
    if (now - rate.t >= 1000 || !next || (next.tx_count < rate.tx)) {
      const dt = Math.max(0.001, (now - rate.t) / 1000);
      setRates({ tx: next ? Math.max(0, (next.tx_count - rate.tx) / dt) : 0, rx: next ? Math.max(0, (next.rx_count - rate.rx) / dt) : 0 });
      lastRates.current = { t: now, tx: next?.tx_count ?? 0, rx: next?.rx_count ?? 0 };
    }
    setState(next);
  }, []);

  useEffect(() => {
    let disposed = false;
    let timer: number | undefined;
    let failures = 0;
    const poll = async () => {
      const captured = epoch.current;
      try {
        const next = await unitreeApi.getState();
        if (disposed || captured !== epoch.current) return;
        acceptState(next);
        setPollError(null);
        failures = 0;
      } catch (error) {
        if (!disposed && ++failures >= 3) setPollError(errMsg(error));
      } finally {
        if (!disposed) timer = window.setTimeout(poll, 50);
      }
    };
    void poll();
    return () => { disposed = true; if (timer != null) window.clearTimeout(timer); };
  }, [acceptState]);

  useEffect(() => { void unitreeApi.listPorts().then(setPorts).catch(() => {}); }, []);

  const run = async (operation: () => Promise<unknown>, success?: string) => {
    if (action.current || stopInFlight.current) return;
    action.current = true;
    setBusy(true);
    try {
      await operation();
      acceptState(await unitreeApi.getState());
      if (alive.current && success) message.success(success);
    } catch (error) {
      if (alive.current) message.error(errMsg(error));
      try { acceptState(await unitreeApi.getState()); } catch { /* Preserve operation error. */ }
    } finally {
      action.current = false;
      if (alive.current) setBusy(false);
    }
  };

  const stop = useCallback(async () => {
    if (stopInFlight.current) return;
    stopInFlight.current = true;
    setStopping(true);
    try {
      acceptState(await unitreeApi.stop());
      if (alive.current) message.success(zh ? "舵机已确认锁定，超时标志已清除" : "Servo acknowledged lock; timeout cleared");
    } catch (error) {
      if (alive.current) message.error(errMsg(error));
      try { acceptState(await unitreeApi.getState()); } catch { /* Preserve stop error. */ }
    } finally {
      stopInFlight.current = false;
      if (alive.current) setStopping(false);
    }
  }, [acceptState, message, zh]);

  useEffect(() => {
    const handler = (event: KeyboardEvent) => {
      if (event.ctrlKey && event.key === "Enter" && connected) { event.preventDefault(); void stop(); }
    };
    window.addEventListener("keydown", handler);
    return () => window.removeEventListener("keydown", handler);
  }, [connected, stop]);

  const setSettings = <K extends keyof UnitreeSettings>(key: K, value: UnitreeSettings[K]) => {
    setProfile(p => ({ ...p, settings: { ...p.settings, [key]: value } }));
  };
  const setTarget = (key: keyof UnitreeTarget, value: number | null) => {
    if (value != null) setProfile(p => ({ ...p, target: { ...p.target, [key]: value } }));
  };
  const selectServo = (id: number) => void run(async () => {
    validateUnitreeId(id);
    epoch.current++;
    setProfile(p => ({ ...p, selected_id: id }));
    await unitreeApi.select(id);
  });

  const changeId = () => void run(async () => {
    const { oldId, newId: destination } = validateUnitreeIdChange(state!.selected_id!, newId!);
    epoch.current++;
    const next = await unitreeApi.changeId(oldId, destination);
    acceptState(next);
    setNewId(null);
    const updated = { ...profile, selected_id: next.selected_id ?? destination };
    setProfile(updated);
    // Flash is already saved. Local storage failure must not misreport that
    // hardware operation as failed.
    try { localStorage.setItem(UNITREE_PROFILE_KEY, JSON.stringify(updated)); }
    catch { message.warning(tr("舵机 ID 已保存；本机配置存储不可用", "Servo ID saved; local profile storage is unavailable")); }
  }, tr("ID 已写入并保存到舵机，已按新 ID 恢复通信", "ID saved to servo Flash; communication restored at the new ID"));

  const saveProfile = () => {
    try {
      const checked = parseUnitreeProfile(JSON.stringify(profile));
      localStorage.setItem(UNITREE_PROFILE_KEY, JSON.stringify(checked));
      message.success(tr("配置已保存到本机", "Profile saved on this computer"));
    } catch (error) { message.error(errMsg(error)); }
  };
  const exportProfile = () => {
    try {
      const checked = parseUnitreeProfile(JSON.stringify(profile));
      const url = URL.createObjectURL(new Blob([JSON.stringify(checked, null, 2)], { type: "application/json" }));
      const link = document.createElement("a");
      link.href = url;
      link.download = `unitree-j288s288-id${checked.selected_id}.json`;
      link.click();
      window.setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (error) { message.error(errMsg(error)); }
  };

  const chartOption = useMemo(() => {
    const recent = samples.current;
    const end = recent[recent.length - 1]?.t ?? 0;
    const data = recent.filter(s => s.t >= end - 20);
    const colors = ["#4f8cff", "#65d6ad", "#ffb45e"];
    return {
      animation: false,
      backgroundColor: "transparent",
      tooltip: { trigger: "axis" },
      grid: [0, 1, 2].map(i => ({ left: 68, right: 25, top: 22 + 112 * i, height: 76 })),
      xAxis: [0, 1, 2].map(i => ({ type: "value", gridIndex: i, min: Math.max(0, end - 20), max: Math.max(20, end), axisLabel: { show: i === 2, color: "#94a3b8" }, name: i === 2 ? "s" : "", splitLine: { lineStyle: { color: "#263249" } } })),
      yAxis: ["rad", "rad/s", "Nm"].map((unit, i) => ({ type: "value", gridIndex: i, name: unit, scale: true, axisLabel: { color: "#94a3b8" }, splitLine: { lineStyle: { color: "#263249" } } })),
      series: (["position", "velocity", "torque"] as const).map((key, i) => ({
        name: [tr("位置", "Position"), tr("速度", "Velocity"), tr("力矩", "Torque")][i],
        type: "line", xAxisIndex: i, yAxisIndex: i, showSymbol: false, connectNulls: false, lineStyle: { width: 1.7, color: colors[i] }, itemStyle: { color: colors[i] }, data: data.map(s => [s.t, s[key]]),
      })),
    };
  }, [state?.feedback_sequence, state?.online, chartRevision, lang]);

  const field = (label: string, control: ReactNode) => <label className="unitree-field"><span>{label}</span>{control}</label>;
  const value = (number: number | undefined, digits = 3) => state?.online && number != null ? number.toFixed(digits) : "—";
  const metric = (label: string, number: number | undefined, unit: string, digits = 3) => (
    <div className="unitree-metric"><Typography.Text type="secondary">{label}</Typography.Text><div>{value(number, digits)} <small>{unit}</small></div></div>
  );
  const f = state?.feedback;

  return <div className="unitree-panel">
    <Card size="small">
      <div className="unitree-connection">
        {field(tr("USB 串口", "USB serial port"), <AutoComplete aria-label="USB serial port" value={profile.port} disabled={connected || locked} onChange={port => setProfile(p => ({ ...p, port }))} options={ports.map(p => ({ value: p.name, label: `${p.name} · ${p.description}` }))} placeholder="/dev/ttyACM0" style={{ width: 290 }} />)}
        <Button disabled={locked} onClick={() => void run(async () => { setPorts(await unitreeApi.listPorts()); })}>{tr("刷新串口", "Refresh ports")}</Button>
        {field(tr("选择现有 ID", "Select existing ID"), <InputNumber aria-label="Servo ID" min={0} max={14} precision={0} value={profile.selected_id} disabled={connected || locked} onChange={id => id != null && setProfile(p => ({ ...p, selected_id: id }))} />)}
        <Tag>6 Mbps · 8N1 · TTL</Tag>
        <Button type={connected ? "default" : "primary"} disabled={locked} loading={busy} onClick={() => void run(async () => {
          epoch.current++;
          if (connected) { await unitreeApi.disconnect(); acceptState(null); }
          else { validateUnitreeId(profile.selected_id); await unitreeApi.connect(profile.port.trim(), profile.selected_id, validateUnitreeSettings(profile.settings)); }
        }, connected ? tr("串口已断开", "Serial port disconnected") : tr("串口已打开，等待舵机应答", "Serial opened; waiting for servo feedback"))}>{connected ? tr("断开串口", "Disconnect") : tr("打开串口", "Open serial")}</Button>
        <Button disabled={!connected || locked || !idle} onClick={() => void run(() => unitreeApi.scan(), tr("扫描完成", "Scan completed"))}>{tr("扫描总线 0–14", "Scan bus 0–14")}</Button>
        <Tooltip title={changingId ? tr("当前已停机，ID 保存事务完成后执行停止请求", "Motion is already stopped; Stop will run after ID saving completes") : undefined}><Button danger type="primary" disabled={!connected || stopping} loading={stopping} onClick={() => void stop()}>{tr("停止 / 清除超时", "Stop / clear timeout")}</Button></Tooltip>
      </div>
      <div className="unitree-connection-status">
        <Tag color={connected ? "blue" : "default"}>{connected ? tr("串口已打开", "Serial open") : tr("串口未连接", "Serial closed")}</Tag>
        <Tag color={state?.online ? "green" : "default"}>{state?.online ? tr("舵机在线", "Servo online") : tr("舵机未应答", "No servo feedback")}</Tag>
        <Typography.Text type="secondary">{tr("发送", "TX")} {rates.tx.toFixed(1)} Hz · {tr("接收", "RX")} {rates.rx.toFixed(1)} Hz · Ctrl+Enter {tr("停止", "to stop")}</Typography.Text>
      </div>
    </Card>

    {pollError && <Alert className="unitree-alert" type="error" showIcon message={tr("无法读取后台状态", "Backend state unavailable")} description={pollError} />}
    {connected && !state?.online && !changingId && !busRunning && !state?.bootloader_ids?.length && <Alert className="unitree-alert" type="info" showIcon message={tr("串口已打开，尚未收到有效舵机反馈", "Serial open; no valid servo feedback yet")} description={tr("舵机未上电时这是正常状态。检查供电、共地、USB 单总线适配器和 ID；上电后可扫描总线。", "This is expected while the servo is unpowered. Check power, common ground, USB single-wire adapter and ID; scan after powering on.")} />}
    {state?.last_error && <Alert className="unitree-alert" type="warning" showIcon message={state.last_error} />}
    {connected && !state.stop_confirmed && !active && !changingId && !state.bootloader_ids?.length && <Alert className="unitree-alert" type="error" showIcon message={tr("停止尚未得到舵机确认，请重试停止", "Servo stop is unconfirmed; retry Stop")} />}
    {!!state?.pending_stop_ids?.length && !active && !busRunning && <Alert className="unitree-alert" type="warning" showIcon message={tr(`以下 ID 尚未确认锁定：${state.pending_stop_ids.join(", ")}，请重试停止全部舵机`, `Lock unconfirmed at IDs ${state.pending_stop_ids.join(", ")}; retry Stop for all servos`)} />}
    {!!state?.bootloader_ids?.length && !changingId && tab !== "device" && <Alert className="unitree-alert" type="warning" showIcon message={tr("舵机可能仍在引导模式，请在「舵机设置」中恢复 APP", "Servo may still be in Boot; restore APP in Servo settings")} action={<Button onClick={() => setTab("device")}>{tr("舵机设置", "Servo settings")}</Button>} />}

    <div className="unitree-workspace">
      <aside>
        <Card size="small" title={tr("总线舵机", "Bus servos")}>
          {!state?.devices.length && <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={tr("等待扫描或反馈", "Scan or await feedback")} />}
          {state?.devices.map(device => <button key={device.id} type="button" className={`unitree-device ${state.selected_id === device.id ? "unitree-device--selected" : ""}`} disabled={busy || stopping || changingId || (busRunning ? !state.bus_test?.nodes.some(node => node.id === device.id) : !idle)} onClick={() => selectServo(device.id)}>
            <strong>J288 / S288 · ID {device.id}</strong><span>{device.feedback.voltage_v.toFixed(1)} V · {device.feedback.shell_temperature_c} °C</span>
            <Tag color={device.online ? "green" : "default"}>{device.online ? tr("在线", "Online") : tr("已发现", "Discovered")}</Tag>
          </button>)}
          <div className="unitree-manual">
            <InputNumber aria-label="Manual servo ID" min={0} max={14} precision={0} value={profile.selected_id} disabled={locked || !idle} onChange={id => id != null && setProfile(p => ({ ...p, selected_id: id }))} />
            <Button disabled={!connected || locked || !idle} onClick={() => selectServo(profile.selected_id)}>{tr("选择", "Select")}</Button>
          </div>
          <Typography.Paragraph type="secondary" className="unitree-note">{tr("ID 15 是无应答广播地址。扫描和空闲监控发送锁定模式（0）。", "ID 15 is a broadcast address with no reply. Scans and idle monitoring send lock mode (0).")}</Typography.Paragraph>
        </Card>
      </aside>

      <div className="unitree-detail">
        <Card size="small">
          <div className="unitree-summary"><Space wrap>
            <Typography.Title level={4} style={{ margin: 0 }}>J288 / S288 {state?.selected_id != null ? `· ID ${state.selected_id}` : ""}</Typography.Title>
            <Tag color={active ? "orange" : "default"}>{active ? state?.streaming ? tr("连续控制中", "Streaming") : tr("单次控制中", "Single command") : tr("待机", "Idle")}</Tag>
            <Tag>{tr("反馈模式", "Feedback mode")}: {state?.online && f ? `${f.mode === 0 ? tr("锁定", "Lock") : f.mode === 1 ? "FOC" : tr("其他", "Other")} (${f.mode})` : "—"}</Tag>
          </Space><Segmented value={view} onChange={v => setView(String(v))} options={[{ value: "numeric", label: tr("数值", "Values") }, { value: "chart", label: tr("曲线", "Chart") }]} /></div>
          {view === "numeric" ? <div className="unitree-metrics">
            {metric(tr("多圈输出位置", "Multi-turn output position"), f?.position_rad, "rad")}
            {metric(tr("输出端单圈角度", "Output single-turn angle"), f?.output_angle_rad, "rad")}
            {metric(tr("输出速度", "Output velocity"), f?.velocity_rad_s, "rad/s")}
            {metric(tr("输出力矩", "Output torque"), f?.torque_nm, "Nm")}
            {metric(tr("电压", "Voltage"), f?.voltage_v, "V", 1)}
            <div className="unitree-metric"><Typography.Text type="secondary">{tr("壳体 / 绕组温度", "Shell / winding temperature")}</Typography.Text><div>{value(f?.shell_temperature_c, 0)} / {value(f?.winding_temperature_c, 0)} <small>°C</small></div></div>
          </div> : samples.current.length ? <ReactECharts option={chartOption} notMerge style={{ height: 370 }} /> : <Empty description={tr("等待有效反馈数据", "Waiting for valid feedback")} />}
          <div className="unitree-feedback-footer"><Typography.Text type="secondary">{tr("反馈年龄", "Feedback age")}: {state?.feedback_age_ms != null ? `${state.feedback_age_ms} ms` : "—"} · {tr("减速比", "Gear ratio")}: 70070 / 243 ≈ 288.354</Typography.Text>
            <Button size="small" onClick={() => { samples.current = []; setChartRevision(v => v + 1); }}>{tr("清除曲线", "Clear chart")}</Button>
          </div>
          {state?.online && f && (f.error_bits !== 0 || f.warning_bits !== 0 || f.timed_out) && <Alert className="unitree-alert" showIcon type={f.error_bits ? "error" : "warning"} message={`${tr("故障", "Errors")} 0x${f.error_bits.toString(16).padStart(8, "0").toUpperCase()} · ${tr("警告", "Warnings")} 0x${f.warning_bits.toString(16).toUpperCase()}`} description={<>{[...f.errors, ...f.warnings, ...(f.timed_out ? [tr("通信超时保护已触发；停止可清除", "Communication timeout latched; Stop clears it")] : [])].map(text => <div key={text}>{text}</div>)}</>} />}
        </Card>

        <Card size="small" className="unitree-controls">
          <Segmented className="unitree-tabs" value={tab} onChange={v => setTab(String(v))} options={[
            { value: "motion", label: tr("运动控制", "Motion") }, { value: "bus", label: tr("一拖多测试", "Multi-servo test") }, { value: "settings", label: tr("通信与限制", "Communication & limits") },
            { value: "device", label: tr("舵机设置", "Servo settings") }, { value: "diagnostics", label: tr("通信诊断", "Diagnostics") },
          ]} />
          {tab === "motion" && <>
            <div className="unitree-form-row">
              {field(tr("工作模式", "Operating mode"), <Select aria-label="Operating mode" value={workMode} disabled={locked} onChange={setWorkMode} style={{ width: 220 }} options={[{ value: 0, label: tr("锁定（0）", "Lock (0)") }, { value: 1, label: tr("混合 FOC（1）", "Hybrid FOC (1)") }]} />)}
              <Button disabled={!connected || locked || (workMode === 1 && !ready)} onClick={() => void run(() => unitreeApi.setMode(workMode), workMode === 0 ? tr("已确认锁定模式", "Lock mode confirmed") : tr("已确认 FOC 模式；当前输出为零，可发送目标", "FOC confirmed with zero output; ready for targets"))}>{workMode === 1 ? tr("使能 FOC（零力矩 / 零增益）", "Enable FOC (zero torque / gains)") : tr("切换到锁定模式", "Switch to lock mode")}</Button>
            </div>
            <div className="unitree-form-row">
              {field(tr("控制预设（修改输入值）", "Control preset (edits inputs)"), <Select aria-label="Control preset" placeholder={tr("选择预设", "Choose preset")} disabled={locked} style={{ width: 230 }} onChange={preset => setProfile(p => ({ ...p, target: {
                position_rad: state?.online && f ? f.position_rad : p.target.position_rad,
                velocity_rad_s: preset === "velocity" ? 0.2 : 0,
                torque_nm: preset === "torque" ? 0.05 : 0,
                kp: preset === "position" ? 1 : 0,
                kd: preset === "torque" ? 0 : 0.1,
              } }))} options={[{ value: "position", label: tr("位置保持", "Position hold") }, { value: "velocity", label: tr("速度控制", "Velocity") }, { value: "torque", label: tr("力矩控制", "Torque") }, { value: "damping", label: tr("阻尼 / 可手转", "Damping") }]} />)}
              <Typography.Text type="secondary">τ = τff + Kp (qdes − q) + Kd (ωdes − ω)</Typography.Text>
            </div>
            <div className="unitree-form-row">
              {field(tr("目标位置 (rad)", "Position (rad)"), <InputNumber aria-label="Target position" value={profile.target.position_rad} step={0.01} min={profile.settings.position_min_rad} max={profile.settings.position_max_rad} disabled={locked} onChange={v => setTarget("position_rad", v)} />)}
              {field(tr("目标速度 (rad/s)", "Velocity (rad/s)"), <InputNumber aria-label="Target velocity" value={profile.target.velocity_rad_s} step={0.01} min={-profile.settings.velocity_limit_rad_s} max={profile.settings.velocity_limit_rad_s} disabled={locked} onChange={v => setTarget("velocity_rad_s", v)} />)}
              {field(tr("前馈力矩 (Nm)", "Feedforward torque (Nm)"), <InputNumber aria-label="Target torque" value={profile.target.torque_nm} step={0.01} min={-profile.settings.torque_limit_nm} max={profile.settings.torque_limit_nm} disabled={locked} onChange={v => setTarget("torque_nm", v)} />)}
              {field("Kp (Nm/rad)", <InputNumber aria-label="Kp" value={profile.target.kp} min={0} max={2128.523254} step={0.1} disabled={locked} onChange={v => setTarget("kp", v)} />)}
              {field("Kd (Nm·s/rad)", <InputNumber aria-label="Kd" value={profile.target.kd} min={0} max={21.285233} step={0.01} disabled={locked} onChange={v => setTarget("kd", v)} />)}
            </div>
            <Space wrap>
              <Button disabled={!state?.online || locked} onClick={() => f && setTarget("position_rad", f.position_rad)}>{tr("使用当前位置", "Use current position")}</Button>
              <Switch checked={repeat} disabled={locked || active} onChange={setRepeat} checkedChildren={tr("连续发送", "Continuous")} unCheckedChildren={tr("单次发送", "Single")} />
              <Button type="primary" disabled={!ready || locked || workMode !== 1} loading={busy} onClick={() => void run(() => unitreeApi.sendTarget(validateUnitreeTarget(profile.target, state!.settings), repeat))}>{active ? tr("更新目标", "Update target") : tr("发送 FOC 目标", "Send FOC target")}</Button>
            </Space>
            {settingsDirty && <Alert className="unitree-alert" type="info" showIcon message={tr("通信或限制已修改，请先在「通信与限制」中应用设置", "Communication settings or limits changed; apply them before sending")} />}
            <Typography.Paragraph type="secondary" className="unitree-note">{tr("所有目标与反馈均使用输出端单位。位置、速度、力矩预设均使用混合 FOC。切换工作模式时，程序会以零力矩、零增益重复确认反馈，适应首帧模式滞后；确认后才发送目标。预设只修改输入框。单次目标约 200 ms 后自动锁定。", "All targets and feedback use output-side units. Position, velocity and torque presets all use hybrid FOC. Mode changes use bounded zero-torque, zero-gain confirmation to accommodate delayed first-frame status, then apply the target. Presets edit inputs only. A single target is followed by lock after about 200 ms.")}</Typography.Paragraph>
          </>}

          {tab === "bus" && <UnitreeBusTestPanel state={state} disabled={locked || active} settingsReady={!settingsDirty} target={profile.target} run={run} stop={stop} />}

          {tab === "settings" && <>
            <div className="unitree-form-row">
              {field(tr("发送频率 (Hz)", "Command frequency (Hz)"), <InputNumber aria-label="Command frequency" min={65} max={500} precision={0} value={profile.settings.command_hz} disabled={locked || !idle} onChange={v => v != null && setSettings("command_hz", v)} />)}
              {field(tr("响应等待 (ms)", "Response timeout (ms)"), <InputNumber aria-label="Response timeout" min={5} max={100} precision={0} value={profile.settings.response_timeout_ms} disabled={locked || !idle} onChange={v => v != null && setSettings("response_timeout_ms", v)} />)}
              {field(tr("舵机通信超时保护（约 1 s）", "Servo timeout protection (~1 s)"), <Switch aria-label="Timeout protection" checked={profile.settings.timeout_protection} disabled={locked || !idle} onChange={v => setSettings("timeout_protection", v)} />)}
            </div>
            <div className="unitree-form-row">
              {field(tr("位置下限 (rad)", "Position minimum (rad)"), <InputNumber aria-label="Position minimum" value={profile.settings.position_min_rad} min={-1428.018898} max={1428.018898} step={0.1} disabled={locked || !idle} onChange={v => v != null && setSettings("position_min_rad", v)} />)}
              {field(tr("位置上限 (rad)", "Position maximum (rad)"), <InputNumber aria-label="Position maximum" value={profile.settings.position_max_rad} min={-1428.018898} max={1428.018898} step={0.1} disabled={locked || !idle} onChange={v => v != null && setSettings("position_max_rad", v)} />)}
              {field(tr("速度绝对值上限 (rad/s)", "Absolute velocity limit (rad/s)"), <InputNumber aria-label="Velocity limit" value={profile.settings.velocity_limit_rad_s} min={0.001} max={278.90143} step={0.1} disabled={locked || !idle} onChange={v => v != null && setSettings("velocity_limit_rad_s", v)} />)}
              {field(tr("前馈力矩绝对值上限 (Nm)", "Absolute feedforward torque limit (Nm)"), <InputNumber aria-label="Torque limit" value={profile.settings.torque_limit_nm} min={0.001} max={36.908174} step={0.05} disabled={locked || !idle} onChange={v => v != null && setSettings("torque_limit_nm", v)} />)}
            </div>
            <Space wrap>
              <Button type="primary" disabled={!connected || locked || !idle || !settingsDirty} onClick={() => void run(() => unitreeApi.configure(validateUnitreeSettings(profile.settings)), tr("会话设置已应用", "Session settings applied"))}>{tr("应用会话设置", "Apply session settings")}</Button>
              <Button onClick={saveProfile} disabled={locked}>{tr("保存到本机", "Save locally")}</Button>
              <Button onClick={exportProfile} disabled={locked}>{tr("导出 JSON", "Export JSON")}</Button>
              <Button onClick={() => fileInput.current?.click()} disabled={connected || locked}>{tr("导入 JSON", "Import JSON")}</Button>
              <input ref={fileInput} type="file" accept=".json,application/json" hidden onChange={async event => {
                const file = event.target.files?.[0]; event.target.value = "";
                if (!file) return;
                try {
                  if (file.size > 65536) throw new Error(tr("配置文件过大", "Profile exceeds 64 KiB"));
                  setProfile(parseUnitreeProfile(await file.text()));
                  message.success(tr("本机配置已导入；尚未连接或发送", "Local profile imported; connect and send explicitly"));
                } catch (error) { message.error(errMsg(error)); }
              }} />
            </Space>
            <Typography.Paragraph type="secondary" className="unitree-note">{tr("这些是上位机会话设置，保存在此电脑；不会写入舵机 Flash。速度及前馈力矩限制由后台检查，位置上下限检查目标值；Kp/Kd 产生的反馈力矩不受此前馈上限约束。连续控制在界面心跳丢失 1.5 s、连续 3 次应答失败或舵机报告故障后停止。", "These host session settings are saved on this computer and do not write servo Flash. The backend validates target position, velocity and feedforward torque. The feedforward limit does not cap torque generated by Kp/Kd. Motion stops on a 1.5 s client heartbeat loss, three failed responses, or servo fault feedback.")}</Typography.Paragraph>
          </>}

          {tab === "device" && <>
            <Descriptions size="small" column={2} style={{ marginTop: 20 }} items={[
              { key: "id", label: tr("当前反馈 ID", "Feedback ID"), children: state?.online ? f?.id : "—" },
              { key: "baud", label: tr("固定波特率", "Fixed baud rate"), children: "6,000,000 bps" },
            ]} />
            <div className="unitree-form-row">
              {field(tr("新舵机 ID", "New servo ID"), <InputNumber aria-label="New servo ID" min={0} max={14} precision={0} value={newId} disabled={locked || !idle || !state?.online} onChange={setNewId} />)}
              <Button type="primary" loading={changingId} disabled={!ready || !idle || locked || newId == null || newId === state?.selected_id} onClick={changeId}>{tr("修改 ID 并保存到舵机", "Change ID and save to servo")}</Button>
              {newId != null && <Tag>{state?.selected_id ?? "—"} → {newId}</Tag>}
            </div>
            <Typography.Paragraph type="secondary">{tr("配置 ID 时请在此串口总线上仅连接一台已上电舵机。ID 范围为 0–14；操作前先停止运动。写入后会保存到舵机 Flash、回读 ID、恢复 APP 并切换到新 ID，不会启动运动。", "Connect one powered servo to this serial bus for ID configuration. IDs range from 0 to 14; stop motion first. The operation saves to servo Flash, reads back the ID, restores APP and selects the new ID. Motion remains stopped.")}</Typography.Paragraph>
            {state?.id_change && <Alert className="unitree-alert" showIcon
              type={changingId ? "info" : state.id_change.phase === "complete" ? "success" : "warning"}
              message={changingId ? tr("正在配置舵机 ID，请保持连接与供电", "Configuring servo ID; keep serial and power connected") : state.id_change.phase === "complete" ? tr(`ID ${state.id_change.old_id} → ${state.id_change.new_id} 已保存到舵机 Flash`, `ID ${state.id_change.old_id} → ${state.id_change.new_id} saved to servo Flash`) : tr("ID 配置未完成", "ID configuration incomplete")}
              description={changingId ? ({ checking: tr("检查新 ID 是否被占用", "Checking for an occupied ID"), entering_boot: tr("进入引导模式", "Entering Boot"), unlocking: tr("解锁参数存储", "Unlocking parameter storage"), writing: tr("写入新 ID", "Writing the new ID"), saving: tr("保存到舵机 Flash", "Saving to servo Flash"), verifying: tr("回读保存后的 ID", "Reading back the saved ID"), restoring: tr("恢复 APP 并验证新 ID 通信", "Restoring APP and verifying communication"), complete: "", failed: "" }[state.id_change.phase]) : state.id_change.error ?? tr("新 ID 已回读且已收到锁定反馈。可重新上电后扫描，进一步核验断电保持。", "ID readback and lock feedback confirmed. Power-cycle and scan to additionally verify retention.")}
            />}
            <div className="unitree-form-row">
              {field(tr("恢复 APP 的舵机 ID", "Servo ID for APP recovery"), <InputNumber aria-label="APP recovery ID" min={0} max={14} precision={0} value={recoveryId ?? state?.selected_id ?? profile.selected_id} disabled={locked || active} onChange={setRecoveryId} />)}
              <Button disabled={!connected || locked || active} onClick={() => void run(() => unitreeApi.restoreApp(validateUnitreeId(recoveryId ?? state?.selected_id ?? profile.selected_id)), tr("已恢复 APP 并收到锁定反馈", "APP restored; lock response confirmed"))}>{tr("恢复 APP / 重新建立通信", "Restore APP / reconnect servo")}</Button>
            </div>
            {!!state?.bootloader_ids?.length && <Alert type="warning" showIcon message={tr(`尚未恢复 APP，请尝试 ID ${state.bootloader_ids.join(" / ")}；必要时重新上电后再点恢复 APP`, `APP recovery pending. Try ID ${state.bootloader_ids.join(" / ")}; if necessary power-cycle before restoring APP`)} />}
          </>}

          {tab === "diagnostics" && <>
            <Descriptions className="unitree-diagnostics" size="small" column={{ xs: 1, sm: 2, md: 3 }} items={[
              { key: "tx", label: "TX", children: state?.tx_count ?? 0 }, { key: "rx", label: "RX", children: state?.rx_count ?? 0 },
              { key: "crc", label: tr("无效反馈帧", "Invalid feedback frames"), children: state?.crc_error_count ?? 0 },
              { key: "timeouts", label: tr("响应超时次数", "Response timeouts"), children: state?.timeout_count ?? 0 },
              { key: "error", label: tr("故障码", "Error bits"), children: f ? `0x${f.error_bits.toString(16).padStart(8, "0").toUpperCase()}` : "—" },
              { key: "warn", label: tr("警告码", "Warning bits"), children: f ? `0x${f.warning_bits.toString(16).toUpperCase()}` : "—" },
            ]} />
            <Typography.Paragraph><strong>{tr("最近 TX", "Last TX")}</strong><code className="unitree-frame">{state?.last_tx_hex ?? "—"}</code></Typography.Paragraph>
            <Typography.Paragraph><strong>{tr("最近有效 RX", "Last valid RX")}</strong><code className="unitree-frame">{state?.last_rx_hex ?? "—"}</code></Typography.Paragraph>
          </>}
        </Card>
      </div>
    </div>
  </div>;
}
