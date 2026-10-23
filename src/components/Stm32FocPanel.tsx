import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { Alert, App, Button, Card, Checkbox, Collapse, Descriptions, Divider, Empty, Input, InputNumber, List, Segmented, Select, Space, Tag, Tooltip, Typography } from "antd";
import { TranslationOutlined } from "@ant-design/icons";
import ReactECharts from "echarts-for-react";
import { errMsg } from "../api";
import { isGsUsbSpec } from "../canProfile";
import { useI18n } from "../i18n";
import { stm32FocApi, type FocConfig, type FocDevice, type FocMode, type FocState, type FocTarget } from "../stm32FocApi";
import "./Stm32FocPanel.css";

const DEFAULT_CONFIG: FocConfig = {
  mode: "Velocity", current_limit_permille: 1000, profile_velocity: 2,
  acceleration: 2, deceleration: 2, quick_stop_deceleration: 4,
  following_error_window: 0.25, following_error_timeout_ms: 1000,
};
const DEFAULT_TARGET: FocTarget = {
  position_rev: 0, velocity_rev_s: 0, torque_permille: 0,
  torque_ff_nm: 0, kp: 0, kd: 0, relative: false,
};
const HISTORY_MS = 60_000;
const MODES: FocMode[] = ["SingleTurn", "Position", "Velocity", "Torque", "Mit"];
type Sample = { time: number; position: number | null; velocity: number | null; torque: number | null };
type Event = { id: number; time: string; text: string; error: boolean };

const hex = (v: number | null | undefined, width = 4) => v == null ? "—" : `0x${v.toString(16).toUpperCase().padStart(width, "0")}`;
const value = (v: number | null | undefined, digits = 3) => v == null || !Number.isFinite(v) ? "—" : v.toFixed(digits);
const defaultIface = () => /linux/i.test(navigator.userAgent) ? "can0" : "gs_usb0";

function modeName(mode: FocMode, zh: boolean): string {
  return ({
    SingleTurn: zh ? "单圈最短路径位置" : "Single-turn position",
    Position: zh ? "多圈累计位置" : "Multi-turn position",
    Velocity: zh ? "速度" : "Velocity",
    Torque: zh ? "力矩" : "Torque",
    Mit: zh ? "MIT · 非压缩" : "MIT · uncompressed",
  })[mode];
}

function driveName(status: number | null | undefined, zh: boolean) {
  if (status == null) return "—";
  const states: Record<number, [string, string]> = {
    0x40: ["Switch-on disabled", "关桥 / 禁止使能"],
    0x21: ["Ready to switch on", "准备上电"],
    0x23: ["Switched on", "已上电 / 闭环未使能"],
    0x27: ["Operation enabled", "运行已使能"],
    0x07: ["Quick stop active", "快速减速中"],
    0x08: ["Fault", "故障"],
  };
  const state = states[status & 0x006f];
  return state ? state[zh ? 1 : 0] : hex(status & 0x006f);
}

function calibrationName(state: number | null | undefined, zh: boolean) {
  if (state == null) return "—";
  const labels = zh
    ? ["需要校准", "对齐中", "扫描中", "停稳计算中", "校准完成", "校准失败"]
    : ["Required", "Aligning", "Sweeping", "Settling", "Complete", "Failed"];
  return labels[state] ?? String(state);
}

function faultName(code: number | null | undefined, zh: boolean) {
  if (code == null) return "—";
  const names: Record<number, [string, string]> = {
    0: ["No fault", "无故障"],
    0x2310: ["Phase overcurrent", "相电流过流"],
    0x3210: ["Bus overvoltage", "母线过压"],
    0x3220: ["Bus undervoltage", "母线欠压"],
    0x4210: ["MCU overtemperature", "MCU 过温"],
    0x5000: ["Driver input / user button", "驱动故障输入 / 用户按钮"],
    0x5300: ["Control timing / supervision", "控制时序 / 任务监督异常"],
    0x6320: ["Invalid configuration / command", "配置 / 采样 / 命令非法"],
    0x7340: ["Encoder fault", "编码器故障"],
    0x8130: ["Communication / command timeout", "通信 / 命令期限故障"],
    0x8312: ["Calibration failed", "电角校准失败"],
    0x8500: ["Position following error", "位置跟随误差超时"],
  };
  const name = names[code];
  return `${hex(code)} · ${name ? name[zh ? 1 : 0] : zh ? "未知故障" : "Unknown fault"}`;
}

function nmtName(state: number | null | undefined) {
  return state == null ? "—" : ({ 0: "Boot-up", 4: "Stopped", 5: "Operational", 127: "Pre-operational" } as Record<number, string>)[state] ?? hex(state, 2);
}

function actualModeName(mode: number | null | undefined, zh: boolean) {
  if (mode == null) return "—";
  if (mode === 0) return zh ? "0 · 未运行" : "0 · Not running";
  const named = ({ [-1]: "SingleTurn", 1: "Position", 3: "Velocity", 4: "Torque", 5: "Mit" } as Record<number, FocMode>)[mode];
  return named ? `${mode} · ${modeName(named, zh)}` : String(mode);
}

const STATUS_BITS: [number, string][] = [[1, "RTSO"], [2, "SO"], [4, "OpEn"], [8, "Fault"], [0x20, "QStop"], [0x40, "SOD"], [0x400, "TgtReached"], [0x1000, "TargetAck"]];
const nodeName = (node: number) => hex(node, 2);

export function Stm32FocPanel() {
  const { message } = App.useApp();
  const { lang, toggle } = useI18n();
  const zh = lang === "zh";
  const text = (cn: string, en: string) => zh ? cn : en;
  const [iface, setIface] = useState(defaultIface);
  const [dataBitrate, setDataBitrate] = useState(5_000_000);
  const [connected, setConnected] = useState(false);
  const [devices, setDevices] = useState<FocDevice[]>([]);
  const [selectedNode, setSelectedNode] = useState<number | null>(null);
  const [manualNode, setManualNode] = useState(1);
  const [view, setView] = useState<FocState | null>(null);
  const [config, setConfig] = useState<FocConfig>(DEFAULT_CONFIG);
  const [target, setTarget] = useState<FocTarget>(DEFAULT_TARGET);
  const [pending, setPending] = useState<string | null>(null);
  const [stopping, setStopping] = useState(false);
  const [halted, setHalted] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [display, setDisplay] = useState<"panel" | "chart">("panel");
  const [rateHz, setRateHz] = useState(50);
  const [windowSec, setWindowSec] = useState(10);
  const [chartVersion, setChartVersion] = useState(0);
  const [events, setEvents] = useState<Event[]>([]);
  const alive = useRef(true);
  const connectedRef = useRef(false);
  const selectedRef = useRef<number | null>(null);
  const epoch = useRef(0);
  const pendingRef = useRef(false);
  const autoSelectionAttempted = useRef(false);
  const samples = useRef<Sample[]>([]);
  const sampleCount = useRef(-1);
  const eventId = useRef(0);
  const lastPhase = useRef("");
  const lastError = useRef<string | null>(null);
  const lastPollError = useRef("");
  const appliedConfig = useRef<string | null>(null);

  const addEvent = (description: string, isError = false) => {
    if (!alive.current) return;
    setEvents((old) => [{ id: ++eventId.current, time: new Date().toLocaleTimeString(), text: description, error: isError }, ...old].slice(0, 40));
  };

  const acceptView = (next: FocState) => {
    if (!alive.current || next.node_id !== selectedRef.current) return;
    setView(next);
    if (next.phase !== lastPhase.current) {
      lastPhase.current = next.phase;
      if (next.phase) addEvent(next.phase);
    }
    if (next.last_error && next.last_error !== lastError.current) addEvent(next.last_error, true);
    lastError.current = next.last_error;
    if (next.rx_count !== sampleCount.current) {
      sampleCount.current = next.rx_count;
      const now = performance.now();
      const buffer = samples.current;
      buffer.push({ time: now, position: next.position_rev, velocity: next.velocity_rev_s, torque: next.torque_permille });
      while (buffer.length && (buffer[0].time < now - HISTORY_MS || buffer.length > 6500)) buffer.shift();
    }
  };

  const clearSelection = () => {
    ++epoch.current;
    selectedRef.current = null;
    setSelectedNode(null);
    setView(null);
    setTarget({ ...DEFAULT_TARGET });
    setConfig({ ...DEFAULT_CONFIG });
    setHalted(false);
    setError(null);
    setEvents([]);
    samples.current = [];
    sampleCount.current = -1;
    lastPhase.current = "";
    lastError.current = null;
    lastPollError.current = "";
    appliedConfig.current = null;
    setChartVersion((v) => v + 1);
  };

  useEffect(() => {
    alive.current = true;
    return () => {
      alive.current = false;
      ++epoch.current;
      // App awaits disconnect before leaving; native close joins Rust tasks.
      connectedRef.current = false;
      selectedRef.current = null;
    };
  }, []);

  const refreshDevices = async () => {
    const generation = epoch.current;
    const next = await stm32FocApi.listDevices();
    if (alive.current && connectedRef.current && generation === epoch.current) setDevices(next);
  };

  useEffect(() => {
    if (!connected) return;
    let cancelled = false;
    let timer: number | undefined;
    const poll = async () => {
      try {
        const next = await stm32FocApi.listDevices();
        if (!cancelled) setDevices(next);
      } catch (cause) {
        if (!cancelled) setError(errMsg(cause));
      } finally {
        if (!cancelled) timer = window.setTimeout(poll, 500);
      }
    };
    void poll();
    return () => { cancelled = true; window.clearTimeout(timer); };
  }, [connected]);

  useEffect(() => {
    if (!connected || selectedNode == null) return;
    let cancelled = false;
    let timer: number | undefined;
    const generation = epoch.current;
    const poll = async () => {
      const started = performance.now();
      try {
        const next = await stm32FocApi.getState();
        if (!cancelled && generation === epoch.current && next.node_id === selectedRef.current) {
          acceptView(next);
          lastPollError.current = "";
        }
      } catch (cause) {
        const details = errMsg(cause);
        if (!cancelled && generation === epoch.current && details !== lastPollError.current) {
          lastPollError.current = details;
          setError(details);
          addEvent(details, true);
        }
      } finally {
        if (!cancelled) timer = window.setTimeout(poll, Math.max(1, 1000 / rateHz - (performance.now() - started)));
      }
    };
    void poll();
    const chartTick = window.setInterval(() => { if (!cancelled) setChartVersion((v) => v + 1); }, 50);
    return () => { cancelled = true; window.clearTimeout(timer); window.clearInterval(chartTick); };
    // acceptView uses refs so outstanding responses cannot cross selections.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [connected, selectedNode, rateHz]);

  const act = async (label: string, operation: () => Promise<unknown>) => {
    if (pendingRef.current) return;
    pendingRef.current = true;
    setPending(label);
    setError(null);
    try {
      await operation();
      if (alive.current) {
        addEvent(label);
        if (connectedRef.current && selectedRef.current != null) acceptView(await stm32FocApi.getState());
        if (connectedRef.current) await refreshDevices();
      }
    } catch (cause) {
      if (alive.current) {
        const details = errMsg(cause);
        setError(details);
        addEvent(label + ": " + details, true);
        message.error(details);
      }
    } finally {
      pendingRef.current = false;
      if (alive.current) setPending(null);
    }
  };

  const selectNode = (node: number) => {
    if (pendingRef.current || node === selectedRef.current) return;
    return act(text("选择节点 ", "Select node ") + nodeName(node), async () => {
      clearSelection();
      const generation = epoch.current;
      try {
        const next = await stm32FocApi.selectNode(node);
        if (!alive.current || generation !== epoch.current) return;
        selectedRef.current = node;
        setSelectedNode(node);
        acceptView(next);
        if (next.initialized) setConfig((old) => ({ ...old, mode: next.mode }));
        setTarget({ ...DEFAULT_TARGET, position_rev: next.position_rev ?? 0 });
      } catch (cause) {
        if (alive.current) await refreshDevices().catch(() => {});
        throw cause;
      }
    });
  };

  useEffect(() => {
    if (!connected || selectedNode != null || pendingRef.current || autoSelectionAttempted.current) return;
    const first = devices.find((device) => device.online);
    if (!first) return;
    autoSelectionAttempted.current = true;
    void selectNode(first.node_id);
    // Auto-selection observes only; initialization and enable remain explicit.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [connected, devices, selectedNode, pending]);

  const connect = () => act(text("连接总线", "Connect bus"), async () => {
    const next = await stm32FocApi.connect(iface.trim(), dataBitrate);
    if (!alive.current) { await stm32FocApi.disconnect(); return; }
    connectedRef.current = next.connected;
    autoSelectionAttempted.current = false;
    clearSelection();
    setDevices([]);
    setConnected(next.connected);
  });

  const disconnect = () => act(text("断开连接", "Disconnect"), async () => {
    try { await stm32FocApi.disconnect(); }
    finally {
      connectedRef.current = false;
      if (alive.current) { setConnected(false); setDevices([]); clearSelection(); }
    }
  });

  const initializeInner = async (resetTarget = true) => {
    const next = await stm32FocApi.initialize(config);
    acceptView(next);
    appliedConfig.current = JSON.stringify(config);
    if (resetTarget) setTarget({ ...DEFAULT_TARGET, position_rev: next.position_rev ?? 0 });
    setHalted(false);
  };

  const enable = () => act(text("使能", "Enable"), async () => {
    const current = await stm32FocApi.getState();
    if (!current.initialized || current.mode !== config.mode || appliedConfig.current !== JSON.stringify(config)) await initializeInner(false);
    await stm32FocApi.enable();
    setHalted(false);
  });

  // Direct bridge-off remains available during initialization or calibration.
  const disable = async () => {
    if (stopping) return;
    setStopping(true);
    try {
      await stm32FocApi.disable();
      if (alive.current) {
        setHalted(false);
        addEvent(text("已失能", "Drive disabled"));
        acceptView(await stm32FocApi.getState());
      }
    } catch (cause) {
      if (alive.current) { setError(errMsg(cause)); addEvent(errMsg(cause), true); }
    } finally { if (alive.current) setStopping(false); }
  };

  const updateConfig = <K extends keyof FocConfig>(key: K, next: FocConfig[K]) => setConfig((old) => ({ ...old, [key]: next }));
  const updateTarget = <K extends keyof FocTarget>(key: K, next: FocTarget[K]) => setTarget((old) => ({ ...old, [key]: next }));
  const busy = pending !== null || stopping;
  const enabled = view?.enabled ?? false;
  const isPosition = config.mode === "Position" || config.mode === "SingleTurn";
  const hasFault = (view?.error_code ?? 0) !== 0 || ((view?.statusword ?? 0) & 8) !== 0;
  const selectedDevice = devices.find((device) => device.node_id === selectedNode);
  const online = selectedDevice?.online ?? false;
  const observedHeartbeat = (selectedDevice?.heartbeat_count ?? 0) > 0;
  const selectedIdentity = view?.identity ?? selectedDevice?.identity;
  const canOperate = connected && selectedNode != null;

  const chartOption = useMemo(() => {
    const cutoff = performance.now() - windowSec * 1000;
    const visible = samples.current.filter((sample) => sample.time >= cutoff);
    const stride = Math.max(1, Math.ceil(visible.length / 1500));
    const plotted = visible.filter((_, index) => index % stride === 0 || index === visible.length - 1);
    const lastTime = plotted[plotted.length - 1]?.time ?? performance.now();
    const signals = [
      { key: "position" as const, title: zh ? "位置 · rev" : "Position · rev", color: "#7ba7d9" },
      { key: "velocity" as const, title: zh ? "速度 · rev/s" : "Velocity · rev/s", color: "#62d18f" },
      { key: "torque" as const, title: zh ? "力矩电流比例 · ‰" : "Torque current ratio · ‰", color: "#f4bb56" },
    ];
    return {
      animation: false, tooltip: { trigger: "axis" },
      title: signals.map((s, i) => ({ text: s.title, left: 12, top: (i * 33 + 1) + "%", textStyle: { color: s.color, fontSize: 11, fontWeight: "normal" } })),
      grid: signals.map((_, i) => ({ left: 65, right: 20, top: (i * 33 + 10) + "%", height: "18%" })),
      xAxis: signals.map((_, i) => ({ type: "value", gridIndex: i, min: -windowSec, max: 0, axisLabel: { show: i === 2, color: "#9aa4b2", formatter: "{value}s" }, splitLine: { lineStyle: { color: "#232932" } } })),
      yAxis: signals.map((_, i) => ({ type: "value", gridIndex: i, scale: true, axisLabel: { color: "#9aa4b2" }, splitLine: { lineStyle: { color: "#232932" } } })),
      series: signals.map((s, i) => ({ name: s.title, type: "line", xAxisIndex: i, yAxisIndex: i, showSymbol: false, connectNulls: false, lineStyle: { color: s.color, width: 1.5 }, itemStyle: { color: s.color }, data: plotted.map((sample) => [(sample.time - lastTime) / 1000, sample[s.key]]) })),
    };
  }, [chartVersion, windowSec, zh]);

  return (
    <div className="foc-workspace">
      <section className="app-command-dock foc-connection" aria-label={text("连接控制台", "Connection dock")}>
        <div className="app-command-dock__label">{text("连接", "Connection")}</div>
        <Space className="connect-bar" wrap size={[8, 8]}>
          <span className="connect-bar__label">{text("总线", "Bus")}</span>
          <Input aria-label="CAN interface" className="foc-interface" value={iface} onChange={(event) => setIface(event.target.value)} disabled={connected || busy} placeholder={defaultIface()} />
          {isGsUsbSpec(iface) && <><span className="connect-bar__label">{text("FD 数据段", "FD data rate")}</span><Select aria-label="FD data rate" className="foc-data-rate" value={dataBitrate} onChange={setDataBitrate} disabled={connected || busy} options={[1, 2, 4, 5].map((m) => ({ value: m * 1_000_000, label: m + " Mbit/s" }))} /></>}
          <Button type={connected ? "default" : "primary"} danger={connected} loading={pending === (zh ? "连接总线" : "Connect bus") || pending === (zh ? "断开连接" : "Disconnect")} disabled={busy || (!connected && !iface.trim())} onClick={connected ? disconnect : connect}>{connected ? text("断开", "Disconnect") : text("连接", "Connect")}</Button>
          <Tag color={connected ? "green" : "default"}>{connected ? text("已连接", "Connected") : text("未连接", "Disconnected")}</Tag>
          <Tag>{isGsUsbSpec(iface) ? "CANopen · N 1 Mbit/s" : text("CANopen · 系统速率", "CANopen · system timing")}</Tag>
          {connected && <Tag color="blue">{text("心跳发现中", "Heartbeat discovery")}</Tag>}
        </Space>
      </section>

      <div className="foc-motor-layout">
        <aside className="foc-sidebar" aria-label={text("设备列表", "Device list")}>
          <div className="foc-sidebar__inventory">
            <Space className="foc-sidebar__heading">
              <Typography.Text strong>{text("设备", "Devices")} ({devices.length})</Typography.Text>
              <Button size="small" disabled={!connected || busy} onClick={() => act(text("清除离线设备", "Forget offline"), stm32FocApi.forgetOffline)}>{text("清除离线", "Forget offline")}</Button>
            </Space>
            {devices.length === 0 ? <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={connected ? text("等待 CANopen 心跳…", "Waiting for CANopen heartbeats…") : text("尚未连接", "Not connected")} /> : <List dataSource={devices} rowKey="node_id" renderItem={(device) => {
              const selected = device.node_id === selectedNode;
              const initialized = selected ? view?.initialized ?? device.initialized : device.initialized;
              const deviceEnabled = selected ? view?.enabled ?? device.enabled : device.enabled;
              return <List.Item role="button" tabIndex={busy ? -1 : 0} data-node-id={device.node_id} className={"foc-device" + (selected ? " foc-device--selected" : "")} aria-disabled={busy} aria-pressed={selected} onClick={() => { if (!busy) void selectNode(device.node_id); }} onKeyDown={(event) => { if (!busy && (event.key === "Enter" || event.key === " ")) { event.preventDefault(); void selectNode(device.node_id); } }}>
                <div className="foc-device__body">
                  <Space className="foc-device__title"><Typography.Text strong>{initialized ? "G431 FOC" : text("CANopen 节点", "CANopen node")}</Typography.Text><Typography.Text code>{nodeName(device.node_id)}</Typography.Text></Space>
                  <Space size={[0, 4]} wrap><Tag color={device.online ? "green" : "default"}>{device.online ? text("在线", "Online") : text("离线", "Offline")}</Tag>{device.heartbeat_count > 0 ? <Tag color="blue">{nmtName(device.nmt_state)}</Tag> : <Tag>{text("未收到心跳", "No heartbeat")}</Tag>}{initialized && <Tag color="cyan">{text("已初始化", "Initialized")}</Tag>}{deviceEnabled && <Tag color="green">{text("已使能", "Enabled")}</Tag>}{!!device.error_code && <Tag color="red">{hex(device.error_code)}</Tag>}</Space>
                  <div className="foc-device__heartbeat">{device.heartbeat_count > 0 ? text("心跳距今 ", "Heartbeat age ") + device.heartbeat_age_ms + " ms · " + device.heartbeat_count : text("手动选择的节点", "Manually selected node")}</div>
                </div>
              </List.Item>;
            }} />}
            <Collapse size="small" ghost className="foc-manual" items={[{ key: "manual", label: text("手动填写节点", "Enter a node manually"), children: <Space><InputNumber aria-label="Manual Node ID" min={1} max={127} precision={0} value={manualNode} onChange={(next) => next != null && setManualNode(next)} disabled={!connected || busy} /><Button disabled={!connected || busy} onClick={() => void selectNode(manualNode)}>{text("选择", "Select")}</Button></Space> }]} />
          </div>
          <div className="foc-sidebar__footer"><Tooltip title={zh ? "Switch to English" : "切换到中文"}><Button block icon={<TranslationOutlined />} onClick={toggle}>{zh ? "English" : "中文"}</Button></Tooltip></div>
        </aside>

        <main className="foc-detail">
          {(error || view?.last_error) && <Alert type="error" showIcon message={error ?? view?.last_error} />}
          {selectedNode == null ? <div className="foc-selection-empty"><Empty description={connected ? text("从左侧选择电机", "Select a motor from the device list") : text("请先连接 CAN 总线", "Connect the CAN bus first")} /></div> : <>
            <Card size="small" className="foc-device-header">
              <div className="foc-device-header__row">
                <Space direction="vertical" size={2}>
                  <Space wrap align="center"><Typography.Title level={4} style={{ margin: 0 }}>{view?.initialized ? "G431 FOC" : text("CANopen 节点", "CANopen node")}</Typography.Title><Typography.Text code>{nodeName(selectedNode)}</Typography.Text><Tag color={online ? "green" : "default"}>{online ? text("在线", "Online") : text("离线", "Offline")}</Tag><Tag color={view?.initialized ? "cyan" : "default"}>{view?.initialized ? text("已初始化", "Initialized") : text("待初始化", "Not initialized")}</Tag><Tag color={hasFault ? "red" : enabled ? "green" : "default"}>{driveName(view?.statusword, zh)}</Tag></Space>
                  {selectedIdentity && <Typography.Text type="secondary" className="foc-identity">vendor {hex(selectedIdentity[0], 8)} · product {hex(selectedIdentity[1], 8)} · serial {hex(selectedIdentity[3], 8)}</Typography.Text>}
                </Space>
                <Space size={4} className="foc-refresh"><Typography.Text type="secondary">{text("刷新", "Refresh")}</Typography.Text><Segmented size="small" value={rateHz} onChange={(next) => setRateHz(Number(next))} options={[{ label: text("低 · 50 Hz", "Low · 50 Hz"), value: 50 }, { label: text("高 · 100 Hz", "High · 100 Hz"), value: 100 }]} /></Space>
              </div>
            </Card>

            <Card size="small" title={text("显示", "Display")} extra={<Space wrap>{display === "chart" && <Space size={4}><Typography.Text type="secondary">{text("时间窗", "Window")}</Typography.Text><InputNumber aria-label="Chart window seconds" size="small" min={1} max={60} value={windowSec} onChange={(next) => setWindowSec(next ?? 10)} addonAfter="s" style={{ width: 90 }} /></Space>}<Segmented value={display} onChange={(next) => setDisplay(next as "panel" | "chart")} options={[{ label: text("数值", "Numeric"), value: "panel" }, { label: text("曲线", "Chart"), value: "chart" }]} /></Space>}>
              {display === "chart" ? <ReactECharts option={chartOption} lazyUpdate style={{ height: 360, width: "100%" }} /> : <Descriptions bordered size="small" column={2} className="foc-live-table">
                <Descriptions.Item label={text("在线 / NMT", "Online / NMT")}><Space wrap size={[0, 4]}><Tag color={online ? "green" : "default"}>{online ? text("在线", "Online") : text("离线", "Offline")}</Tag><Tag>{observedHeartbeat ? nmtName(selectedDevice?.nmt_state) : text("心跳未知", "Heartbeat unknown")}</Tag></Space></Descriptions.Item>
                <Descriptions.Item label={text("驱动状态", "Drive state")}>{driveName(view?.statusword, zh)}</Descriptions.Item>
                <Descriptions.Item label={text("累计位置 · rev", "Position · rev")}>{value(view?.position_rev, 5)}</Descriptions.Item>
                <Descriptions.Item label={text("速度 · rev/s", "Velocity · rev/s")}>{value(view?.velocity_rev_s)}</Descriptions.Item>
                <Descriptions.Item label={text("力矩电流比例 · ‰", "Torque current ratio · ‰")}>{value(view?.torque_permille, 0)}</Descriptions.Item>
                <Descriptions.Item label={text("母线电压 · V", "Bus voltage · V")}>{value(view?.voltage_v, 2)}</Descriptions.Item>
                <Descriptions.Item label="Statusword" span={2}><Space size={[0, 4]} wrap><Tag>{hex(view?.statusword)}</Tag>{view?.statusword != null && STATUS_BITS.filter(([mask]) => (view.statusword! & mask) !== 0).map(([, name]) => <Tag key={name} color={name === "Fault" ? "red" : "blue"}>{name}</Tag>)}</Space></Descriptions.Item>
                <Descriptions.Item label={text("实际模式 · 0x6061", "Actual mode · 0x6061")}>{actualModeName(view?.actual_mode, zh)}</Descriptions.Item>
                <Descriptions.Item label={text("电角校准", "Calibration")}>{calibrationName(view?.calibration_state, zh)}</Descriptions.Item>
                <Descriptions.Item label={text("MCU 温度 · °C", "MCU temperature · °C")}>{value(view?.temperature_c, 1)}</Descriptions.Item>
                <Descriptions.Item label={text("TPDO 距今", "TPDO age")}>{view?.feedback_age_ms == null ? "—" : view.feedback_age_ms + " ms"}</Descriptions.Item>
                <Descriptions.Item label={text("心跳距今", "Heartbeat age")}>{observedHeartbeat ? selectedDevice?.heartbeat_age_ms + " ms" : text("未收到心跳", "No heartbeat")}</Descriptions.Item>
                <Descriptions.Item label="PDO TX / RX">{view ? view.tx_count + " / " + view.rx_count : "—"}</Descriptions.Item>
                <Descriptions.Item label={text("驱动故障", "Drive fault")} span={2}><Typography.Text type={hasFault ? "danger" : undefined}>{faultName(view?.error_code, zh)}</Typography.Text></Descriptions.Item>
              </Descriptions>}
            </Card>

            <Card size="small" title={text("控制", "Control")}>
              {hasFault && <Alert type="error" showIcon message={faultName(view?.error_code, zh)} className="foc-control-error" />}
              <Space wrap align="center" className="foc-mode-row"><Typography.Text type="secondary">{text("模式", "Mode")}</Typography.Text><Segmented value={config.mode} disabled={enabled || busy} onChange={(next) => updateConfig("mode", next as FocMode)} options={MODES.map((mode) => ({ label: modeName(mode, zh), value: mode }))} /><Button type="primary" disabled={!canOperate || enabled || busy} loading={pending === (zh ? "使能" : "Enable")} onClick={enable}>{text("使能", "Enable")}</Button><Button danger disabled={!canOperate || stopping} loading={stopping} onClick={() => void disable()}>{text("失能", "Disable")}</Button><Button disabled={!canOperate || busy} onClick={() => act(text("故障复位", "Reset fault"), async () => { acceptView(await stm32FocApi.resetFault()); setHalted(false); appliedConfig.current = null; })}>{text("故障复位", "Reset fault")}</Button><Button disabled={!canOperate || busy} onClick={() => act(text("初始化完成", "Initialization complete"), initializeInner)}>{view?.initialized ? text("重新初始化", "Reinitialize") : text("初始化", "Initialize")}</Button></Space>
              <Divider className="foc-divider" />
              <Space wrap align="end" className="foc-target-row">
                {(isPosition || config.mode === "Mit") && <NumberField label={text("目标位置 · rev", "Position target · rev")} value={target.position_rev} min={-100000} max={100000} step={0.01} disabled={busy} onChange={(next) => updateTarget("position_rev", next)} />}
                {(config.mode === "Velocity" || config.mode === "Mit") && <NumberField label={text("目标速度 · rev/s", "Velocity target · rev/s")} value={target.velocity_rev_s} min={-40} max={40} step={0.1} disabled={busy} onChange={(next) => updateTarget("velocity_rev_s", next)} />}
                {config.mode === "Torque" && <NumberField label={text("目标力矩电流比例 · ‰", "Torque current target · ‰")} value={target.torque_permille} min={-1000} max={1000} step={10} precision={0} disabled={busy} onChange={(next) => updateTarget("torque_permille", next)} />}
                {config.mode === "Mit" && <><NumberField label={text("前馈力矩 · Nm", "Feedforward torque · Nm")} value={target.torque_ff_nm} min={-1} max={1} step={0.001} disabled={busy} onChange={(next) => updateTarget("torque_ff_nm", next)} /><NumberField label="Kp · Nm/rev" value={target.kp} min={0} max={65.535} step={0.001} precision={3} disabled={busy} onChange={(next) => updateTarget("kp", next)} /><NumberField label="Kd · Nm/(rev/s)" value={target.kd} min={0} max={20} step={0.001} precision={3} disabled={busy} onChange={(next) => updateTarget("kd", next)} /></>}
                {isPosition && <Checkbox checked={target.relative} disabled={busy} onChange={(event) => updateTarget("relative", event.target.checked)}>{text("相对位移", "Relative")}</Checkbox>}
                <Button type="primary" disabled={!canOperate || !enabled || busy} onClick={() => act(text("目标已发送", "Target sent"), () => stm32FocApi.setTarget(target))}>{text("发送目标", "Send target")}</Button>
                {config.mode === "Velocity" && <Button disabled={!canOperate || !enabled || busy} onClick={() => act(text("目标速度置零", "Set velocity to zero"), async () => { const next = { ...target, velocity_rev_s: 0 }; await stm32FocApi.setTarget(next); setTarget(next); })}>{text("速度归零", "Zero velocity")}</Button>}
              </Space>
              <Divider className="foc-divider" />
              <Space wrap><Button disabled={!canOperate || !enabled || busy} onClick={() => act(halted ? text("解除 Halt", "Resume from Halt") : "Halt", async () => { await stm32FocApi.halt(!halted); setHalted(!halted); })}>{halted ? text("解除 Halt", "Resume") : "Halt"}</Button><Button disabled={!canOperate || !enabled || busy} onClick={() => act(text("快速停止", "Quick stop"), stm32FocApi.quickStop)}>Quick stop</Button><Tag color={view?.streaming ? "green" : "default"}>{view?.streaming ? "RPDO · 10 ms" : "RPDO · idle"}</Tag><Typography.Text type="secondary" className="foc-protocol-hint">{text("TPDO 20 ms · 位置目标自动处理应答", "TPDO 20 ms · position acknowledgement handled automatically")}</Typography.Text></Space>
              <div className="foc-phase" aria-live="polite">{pending ?? view?.phase ?? "—"}</div>
            </Card>

            <Collapse size="small" className="foc-settings" items={[
              { key: "parameters", label: text("运行参数与电角校准", "Parameters & electrical calibration"), children: <>
                <div className="foc-parameter-fields">
                  <NumberField label={text("全局电流比例上限 · ‰", "Global current limit · ‰")} value={config.current_limit_permille} min={0} max={1000} step={10} precision={0} disabled={busy || enabled} onChange={(next) => updateConfig("current_limit_permille", next)} />
                  <NumberField label={text("轨迹最大速度 · rev/s", "Profile velocity · rev/s")} value={config.profile_velocity} min={0} max={40} step={0.1} disabled={busy || enabled} onChange={(next) => updateConfig("profile_velocity", next)} />
                  <NumberField label={text("加速度 · rev/s²", "Acceleration · rev/s²")} value={config.acceleration} min={0} max={100} step={0.1} disabled={busy || enabled} onChange={(next) => updateConfig("acceleration", next)} />
                  <NumberField label={text("减速度 · rev/s²", "Deceleration · rev/s²")} value={config.deceleration} min={0} max={100} step={0.1} disabled={busy || enabled} onChange={(next) => updateConfig("deceleration", next)} />
                  <NumberField label={text("Quick stop 减速度 · rev/s²", "Quick stop deceleration · rev/s²")} value={config.quick_stop_deceleration} min={0} max={100} step={0.1} disabled={busy || enabled} onChange={(next) => updateConfig("quick_stop_deceleration", next)} />
                  <NumberField label={text("跟随误差窗口 · rev", "Following error window · rev")} value={config.following_error_window} min={0} max={100000} step={0.01} disabled={busy || enabled} onChange={(next) => updateConfig("following_error_window", next)} />
                  <NumberField label={text("跟随误差持续时间 · ms", "Following error timeout · ms")} value={config.following_error_timeout_ms} min={0} max={65535} step={100} precision={0} disabled={busy || enabled} onChange={(next) => updateConfig("following_error_timeout_ms", next)} />
                </div>
                <Space wrap className="foc-settings-actions"><Button disabled={!canOperate || busy} onClick={() => act(text("参数已应用（未使能）", "Parameters applied (disabled)"), initializeInner)}>{text("应用并初始化", "Apply & initialize")}</Button><Button disabled={!canOperate || busy || enabled} onClick={() => act(text("电角校准并保存", "Calibrate & save"), async () => { acceptView(await stm32FocApi.calibrate()); appliedConfig.current = null; })}>{text("电角校准并保存", "Calibrate & save")}</Button><Button disabled={!canOperate || busy || enabled} onClick={() => act(text("保存参数", "Save parameters"), stm32FocApi.save)}>{text("保存参数", "Save parameters")}</Button></Space>
                <Typography.Paragraph type="secondary" className="foc-hint">{text("首次未校准时先执行电角校准并保存（约 7 秒，会转动电机）。初始化保持未使能；点击使能时会应用所选模式并执行 6 → 7 → F 握手。", "For an uncalibrated motor, calibrate and save first (about 7 seconds; the motor rotates). Initialization leaves the drive disabled. Enable applies the selected mode and performs the 6 → 7 → F handshake.")}</Typography.Paragraph>
                <Typography.Paragraph type="secondary" className="foc-hint">{text("位置为 rev，速度为 rev/s；力矩为配置电流上限的千分比。MIT 使用非压缩 20 B FD+BRS，Kp/Kd 按物理单位输入。加减速度、轨迹速度和跟随窗口须大于零。", "Position uses rev, velocity rev/s, and torque permille of the configured current limit. MIT uses uncompressed 20-byte FD+BRS with physical Kp/Kd units. Profile velocity, acceleration, deceleration and following window must be positive.")}</Typography.Paragraph>
              </> },
              { key: "events", label: text("操作记录与设备标识", "Operations & device identity"), children: <>{selectedIdentity && <div className="foc-identity">{["Vendor", "Product", "Revision", "Serial"].map((label, i) => <span key={label}>{label}: {hex(selectedIdentity[i], 8)}</span>)}</div>}<div className="foc-events">{events.length ? events.map((event) => <div key={event.id} className={event.error ? "foc-events__error" : ""}><time>{event.time}</time><span>{event.text}</span></div>) : <Typography.Text type="secondary">{text("暂无记录", "No operations yet")}</Typography.Text>}</div></> },
            ]} />
          </>}
        </main>
      </div>
    </div>
  );
}

function Field({ label, children }: { label: string; children: ReactNode }) {
  return <div className="foc-field"><div>{label}</div>{children}</div>;
}

function NumberField({ label, value: number, onChange, ...props }: {
  label: string; value: number; min: number; max: number; step: number;
  disabled?: boolean; precision?: number; onChange: (value: number) => void;
}) {
  return <Field label={label}><InputNumber aria-label={label} value={number} {...props} onChange={(next) => { if (next != null && Number.isFinite(next)) onChange(next); }} /></Field>;
}
