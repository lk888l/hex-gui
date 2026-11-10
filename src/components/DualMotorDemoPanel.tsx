import { useCallback, useEffect, useRef, useState } from "react";
import { Alert, App, Button, Card, Input, InputNumber, Select, Space, Switch, Tag, Typography } from "antd";
import ReactECharts from "echarts-for-react";
import { errMsg } from "../api";
import { dualDemoApi, type DemoConnection, type DemoMotion, type DemoMotorView, type DemoView } from "../dualDemoApi";
import { useI18n } from "../i18n";
import "./DualMotorDemoPanel.css";

const STORAGE_KEY = "hexmeow-dual-motor-demo-v1";
const INITIAL_MOTION: DemoMotion = { amplitude_deg: 180, period_s: 5, speed_variation: 0.35,
  meow_kp: 3, meow_kd: 0.15, dm_kp: 3, dm_kd: 0.15, dm_reverse: false };
const MEOW_RATES = [1_000_000, 2_000_000, 4_000_000, 5_000_000];
const DM_RATES = [125_000, 200_000, 250_000, 500_000, 1_000_000, 2_000_000, 2_500_000, 3_200_000, 4_000_000, 5_000_000];

function initialConnection(): DemoConnection {
  const usb = !/linux/i.test(navigator.userAgent);
  const defaults: DemoConnection = {
    meow_iface: usb ? "gs_usb0" : "can0", meow_bitrate: 5_000_000,
    dm_iface: usb ? "gs_usb1" : "can1", dm_bitrate: 1_000_000,
    host_node_id: 10, meow_node_id: null, dm_motor_id: null,
  };
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? "null");
    if (saved && typeof saved.meow_iface === "string" && typeof saved.dm_iface === "string"
      && MEOW_RATES.includes(saved.meow_bitrate) && DM_RATES.includes(saved.dm_bitrate)) {
      return { ...defaults, ...saved };
    }
  } catch { /* Defaults also work when storage is unavailable. */ }
  return defaults;
}

type Sample = { time: number; meowTarget: number; dmTarget: number; meow: number | null; dm: number | null };

export function DualMotorDemoPanel({ onBusyChange }: { onBusyChange: (busy: boolean) => void }) {
  const { lang } = useI18n();
  const zh = lang === "zh";
  const { message } = App.useApp();
  const [config, setConfig] = useState(initialConnection);
  const [motion, setMotion] = useState(INITIAL_MOTION);
  const [view, setView] = useState<DemoView | null>(null);
  const [busy, setBusy] = useState(false);
  const [stopping, setStopping] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [samples, setSamples] = useState<Sample[]>([]);
  const mounted = useRef(true);
  const busyRef = useRef(false);
  const stopRef = useRef(false);
  const operationEpoch = useRef(0);
  const pendingStart = useRef<Promise<DemoView> | null>(null);
  const connected = view?.connected ?? false;
  const running = view?.running ?? false;
  const configLocked = connected || busy;
  const motionLocked = running || busy || stopping;

  const accept = useCallback((next: DemoView) => {
    if (!mounted.current) return;
    setView(next);
    if (next.connection) setConfig(next.connection);
    if (next.running) {
      setMotion(next.motion);
      if (next.phase === "running") setSamples((previous) => {
        const sample: Sample = { time: next.elapsed_s, meowTarget: next.meow.target_deg,
          dmTarget: next.dm.target_deg, meow: next.meow.online ? next.meow.actual_deg : null,
          dm: next.dm.online ? next.dm.actual_deg : null };
        const retained = previous.length && previous[previous.length - 1].time > sample.time ? [] : previous;
        return [...retained.filter((s) => s.time >= sample.time - 30), sample].slice(-400);
      });
    }
  }, []);

  useEffect(() => {
    mounted.current = true;
    let alive = true;
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      if (!busyRef.current && !stopRef.current) {
        const epoch = operationEpoch.current;
        try {
          const next = await dualDemoApi.get();
          if (alive && epoch === operationEpoch.current && !busyRef.current && !stopRef.current) accept(next);
        } catch (e) { if (alive && epoch === operationEpoch.current) setError(errMsg(e)); }
      }
      if (alive) timer = setTimeout(poll, 100);
    };
    void poll();
    return () => { alive = false; mounted.current = false; clearTimeout(timer); };
  }, [accept]);

  const stop = useCallback(async () => {
    if (stopRef.current) return;
    operationEpoch.current += 1;
    stopRef.current = true;
    setStopping(true);
    try {
      const starting = pendingStart.current;
      let next = await dualDemoApi.stop();
      // A delayed Start IPC must not re-enable motion after this Stop.
      if (starting) {
        await starting.catch(() => {});
        next = await dualDemoApi.stop();
      }
      accept(next); setError(null);
    }
    catch (e) { setError(errMsg(e)); }
    finally { stopRef.current = false; if (mounted.current) setStopping(false); }
  }, [accept]);

  useEffect(() => {
    const onKey = (event: KeyboardEvent) => { if (event.key === "Escape" && connected) void stop(); };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [connected, stop]);

  const act = async (operation: () => Promise<void>) => {
    if (busyRef.current) return;
    operationEpoch.current += 1;
    busyRef.current = true;
    setBusy(true); onBusyChange(true); setError(null);
    try { await operation(); }
    catch (e) { if (mounted.current) setError(errMsg(e)); }
    finally { busyRef.current = false; if (mounted.current) setBusy(false); onBusyChange(false); }
  };

  const connect = () => act(async () => {
    const next = await dualDemoApi.connect(config);
    accept(next);
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(config)); } catch { /* Optional persistence. */ }
    message.success(zh ? "两路 CAN 已连接，电机已识别" : "Both CAN channels connected; motors identified");
  });
  const disconnect = () => act(async () => {
    await dualDemoApi.disconnect(); accept(await dualDemoApi.get()); setSamples([]);
  });
  const start = () => act(async () => {
    setSamples([]);
    const epoch = operationEpoch.current;
    const request = dualDemoApi.start(motion);
    pendingStart.current = request;
    try {
      const next = await request;
      if (epoch === operationEpoch.current) accept(next);
    } finally { pendingStart.current = null; }
  });
  const phaseLabels = zh
    ? { disconnected: "未连接", ready: "已就绪", starting: "正在初始化", running: "MIT 钟摆演示中", stopping: "正在停止", error: "已停止 · 请检查" }
    : { disconnected: "Disconnected", ready: "Ready", starting: "Starting", running: "Demo running", stopping: "Stopping", error: "Stopped · check feedback" };
  const chartSeries = [
    { name: "hexmeow · " + (zh ? "实际" : "Actual"), key: "meow", color: "#56c8ef", dashed: false },
    { name: "hexmeow · " + (zh ? "目标" : "Target"), key: "meowTarget", color: "#56c8ef", dashed: true },
    { name: "DAMIAO · " + (zh ? "实际" : "Actual"), key: "dm", color: "#e9b366", dashed: false },
    { name: "DAMIAO · " + (zh ? "目标" : "Target"), key: "dmTarget", color: "#e9b366", dashed: true },
  ] as const;
  const options = {
    animation: false, backgroundColor: "transparent",
    tooltip: { trigger: "axis", valueFormatter: (v: number | null) => v == null ? "—" : `${v.toFixed(1)}°` },
    legend: { top: 4, textStyle: { color: "#b8c1cf" } },
    grid: { left: 54, right: 20, top: 48, bottom: 32 },
    xAxis: { type: "value", name: "s", min: samples.length ? Math.max(0, samples[samples.length - 1].time - 30) : 0,
      axisLabel: { color: "#8e9bad", formatter: (v: number) => v.toFixed(0) }, splitLine: { show: false } },
    yAxis: { type: "value", name: "°", axisLabel: { color: "#8e9bad" }, splitLine: { lineStyle: { color: "#27303c" } } },
    series: chartSeries.map((s) => ({ name: s.name, type: "line", showSymbol: false,
      lineStyle: { color: s.color, width: s.dashed ? 1 : 2, type: s.dashed ? "dashed" : "solid", opacity: s.dashed ? 0.65 : 1 },
      itemStyle: { color: s.color }, data: samples.map((sample) => [sample.time, sample[s.key]]) })),
  };

  return <div className="dual-demo">
    <section className="dual-demo__toolbar">
      <Space wrap>
        <Tag color={running ? "green" : connected ? "blue" : "default"}>{phaseLabels[view?.phase ?? "disconnected"]}</Tag>
        <Typography.Text type="secondary">{zh ? "双 CAN · MIT 位置钟摆" : "DUAL CAN · MIT PENDULUM"}</Typography.Text>
        <Typography.Text type="secondary">{(view?.elapsed_s ?? 0).toFixed(1)} s</Typography.Text>
      </Space>
      <Space wrap>
        <Button onClick={connected ? disconnect : connect} loading={busy && !running} disabled={busy || stopping}>
          {connected ? (zh ? "停止并断开" : "Stop & disconnect") : (zh ? "连接并识别" : "Connect & discover")}
        </Button>
        <Button type="primary" onClick={start} disabled={!connected || running || busy || stopping || !!view?.stop_error}>
          {zh ? "开始演示" : "Start demo"}
        </Button>
        <Button danger type="primary" size="large" onClick={stop} loading={stopping} disabled={!connected}>
          {zh ? "停止两台电机" : "Stop both motors"} · Esc
        </Button>
      </Space>
    </section>
    {(error || view?.error || view?.stop_error) && <Alert type="error" showIcon
      message={error || view?.error || view?.stop_error}
      description={view?.stop_error ? `${zh ? "停止未确认，可重试停止：" : "Disable not confirmed; retry Stop: "}${view.stop_error}` : undefined} />}
    <div className="dual-demo__motors">
      {(["meow", "dm"] as const).map((side) => {
        const meow = side === "meow";
        const data = view?.[side];
        return <Card key={side} className={`dual-demo__motor dual-demo__motor--${side}`}
          title={meow ? "hexmeow-4310" : "DM-J4310-2EC"}
          extra={<Tag color={data?.online ? "green" : "default"}>{data?.online ? (zh ? "在线" : "Online") : (zh ? "离线" : "Offline")}</Tag>}>
          <div className="dual-demo__fields">
            <label>CAN {zh ? "接口 / 通道" : "interface / channel"}
              <Input aria-label={meow ? "hexmeow CAN" : "DAMIAO CAN"} value={meow ? config.meow_iface : config.dm_iface} disabled={configLocked}
                onChange={(e) => setConfig({ ...config, [meow ? "meow_iface" : "dm_iface"]: e.target.value })} />
            </label>
            <label>{meow ? (zh ? "CAN FD 数据速率" : "CAN FD data rate") : (zh ? "电机波特率" : "Motor bitrate")}
              <Select aria-label={meow ? "hexmeow bitrate" : "DAMIAO bitrate"} value={meow ? config.meow_bitrate : config.dm_bitrate} disabled={configLocked}
                onChange={(value) => setConfig({ ...config, [meow ? "meow_bitrate" : "dm_bitrate"]: value })}
                options={(meow ? MEOW_RATES : DM_RATES).map((value) => ({ value, label: `${value / 1e6} Mbit/s` }))} />
            </label>
            <label>{meow ? "Node-ID" : "Motor ID"} · {zh ? "留空自动识别" : "blank = auto"}
              <InputNumber min={meow ? 1 : 0} max={meow ? 127 : 2047} precision={0}
                value={meow ? config.meow_node_id : config.dm_motor_id} disabled={configLocked} placeholder="Auto"
                onChange={(value) => setConfig({ ...config, [meow ? "meow_node_id" : "dm_motor_id"]: value })} />
            </label>
          </div>
          <MotorReadout data={data} zh={zh} />
          <Typography.Text className="dual-demo__protocol" type="secondary">
            {meow ? "CAN FD · 1M arbitration · MIT" : "DAMIAO CAN · MIT"}
            {data?.node_id != null ? ` · ID ${data.node_id} (0x${data.node_id.toString(16).toUpperCase()})` : ""}
          </Typography.Text>
        </Card>;
      })}
    </div>
    <Card title={zh ? "钟摆与 MIT 设置" : "Pendulum & MIT settings"}>
      <div className="dual-demo__motion">
        <label>{zh ? "左右摆幅 (±°)" : "Swing amplitude (±°)"}
          <InputNumber min={1} max={180} value={motion.amplitude_deg} disabled={motionLocked}
            onChange={(value) => value != null && setMotion({ ...motion, amplitude_deg: value })} />
        </label>
        <label>{zh ? "基准周期 (s)" : "Base cycle (s)"}
          <InputNumber min={2} max={30} value={motion.period_s} disabled={motionLocked}
            onChange={(value) => value != null && setMotion({ ...motion, period_s: value })} />
        </label>
        <label>{zh ? "变速幅度 (%)" : "Speed variation (%)"}
          <InputNumber min={0} max={80} value={Math.round(motion.speed_variation * 100)} disabled={motionLocked}
            onChange={(value) => value != null && setMotion({ ...motion, speed_variation: value / 100 })} />
        </label>
        <label>{zh ? "达妙镜像摆动" : "Mirror DAMIAO swing"}
          <Switch checked={motion.dm_reverse} disabled={motionLocked} onChange={(value) => setMotion({ ...motion, dm_reverse: value })} />
        </label>
        <label>{zh ? "主机 Node-ID" : "Host Node-ID"}
          <InputNumber min={1} max={127} precision={0} value={config.host_node_id} disabled={configLocked}
            onChange={(value) => value != null && setConfig({ ...config, host_node_id: value })} />
        </label>
      </div>
      <div className="dual-demo__gains">
        {(["meow", "dm"] as const).map((side) => <div key={side}>
          <Typography.Text>{side === "meow" ? "hexmeow" : "DAMIAO"} · MIT</Typography.Text>
          <label>Kp (Nm/rad)<InputNumber min={0.1} max={30} step={0.1} value={motion[`${side}_kp`]} disabled={motionLocked}
            onChange={(value) => value != null && setMotion({ ...motion, [`${side}_kp`]: value })} /></label>
          <label>Kd (Nm·s/rad)<InputNumber min={0.01} max={5} step={0.01} value={motion[`${side}_kd`]} disabled={motionLocked}
            onChange={(value) => value != null && setMotion({ ...motion, [`${side}_kd`]: value })} /></label>
        </div>)}
      </div>
      <Typography.Paragraph type="secondary" className="dual-demo__hint">
        {zh ? "每次启动以各自当前位置为中心，在 MIT 模式下左右摆动。默认摆幅 ±180°、基准周期 5 秒，摆动节奏按 ±35% 平滑变化，起步平滑展开。镜像开关可让两台反向摆动。停止、离开此 app 或关闭窗口都会停止两台电机。"
          : "Each start uses each motor’s current position as its center. Both swing in MIT mode: ±180° amplitude, 5 s base cycle, and a smoothly varying pace (±35%) with a gentle startup. Mirror mode makes the motors swing in opposite directions. Stop, leaving this app, or closing the window stops both motors."}
      </Typography.Paragraph>
      <Typography.Text type="secondary">
        {zh ? "Windows/macOS：gs_usb0 = CAN0，gs_usb1 = CAN1。Linux SocketCAN 沿用系统接口时序。" : "Windows/macOS: gs_usb0 = CAN0, gs_usb1 = CAN1. Linux SocketCAN uses system-configured timing."}
      </Typography.Text>
    </Card>
    <Card title={zh ? "相对摆动中心的角度 · 最近 30 秒" : "Angle relative to swing center · last 30 seconds"}>
      <ReactECharts option={options} notMerge style={{ height: 270 }} />
    </Card>
  </div>;
}

function MotorReadout({ data, zh }: { data?: DemoMotorView; zh: boolean }) {
  const fresh = data?.online && data.feedback_age_ms != null && data.feedback_age_ms < 750;
  return <div className="dual-demo__readout">
    <div><span>{zh ? "实际角度" : "Actual angle"}</span><strong>{fresh && data.actual_deg != null ? data.actual_deg.toFixed(1) : "—"}<small> °</small></strong></div>
    <div><span>{zh ? "目标角度" : "Target angle"}</span><strong>{(data?.target_deg ?? 0).toFixed(1)}<small> °</small></strong></div>
    <div><span>{zh ? "实时转速" : "Actual speed"}</span><strong>{fresh && data.actual_rpm != null ? data.actual_rpm.toFixed(1) : "—"}<small> RPM</small></strong></div>
    <Tag color={data?.enabled ? "orange" : "default"}>{data?.enabled ? (zh ? "已使能" : "Enabled") : (zh ? "未使能" : "Disabled")}</Tag>
  </div>;
}
