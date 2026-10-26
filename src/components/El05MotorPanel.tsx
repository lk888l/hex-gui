import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { Alert, App, Button, Card, Collapse, Descriptions, Empty, Input, InputNumber, Popconfirm, Segmented, Select, Space, Switch, Tag, Typography } from "antd";
import ReactECharts from "echarts-for-react";
import { errMsg } from "../api";
import { el05Api, type MotorWorkspaceApi, type El05Mode, type El05Protocol, type El05State, type El05Target } from "../el05Api";
import { useI18n } from "../i18n";
import "./El05MotorPanel.css";
import { El05ExperimentPanel } from "./El05ExperimentPanel";
import { El05GravityPanel } from "./El05GravityPanel";
import { StandardMitSettings } from "./StandardMitSettings";

const MODES: El05Mode[] = ["Mit", "PositionPp", "Velocity", "Current", "PositionCsp"];
const MODE_LABELS: Record<El05Mode, [string, string]> = {
  Mit: ["运控 / MIT", "Motion / MIT"],
  PositionPp: ["位置 PP", "Position PP"],
  Velocity: ["速度", "Velocity"],
  Current: ["电流", "Current"],
  PositionCsp: ["位置 CSP", "Position CSP"],
};

interface Parameter {
  index: number;
  name: string;
  label: [string, string];
  unit?: string;
  readOnly?: boolean;
  min?: number;
  max?: number;
  integer?: boolean;
}

const PARAMETERS: Parameter[] = [
  { index: 0x7005, name: "run_mode", label: ["控制模式", "Control mode"], integer: true },
  { index: 0x7006, name: "iq_ref", label: ["目标电流", "Current target"], unit: "A", min: -11, max: 11 },
  { index: 0x700a, name: "spd_ref", label: ["目标速度", "Velocity target"], unit: "rad/s", min: -50, max: 50 },
  { index: 0x700b, name: "limit_torque", label: ["转矩限制", "Torque limit"], unit: "Nm", min: 0, max: 6 },
  { index: 0x7010, name: "cur_kp", label: ["电流 Kp", "Current Kp"], min: 0 },
  { index: 0x7011, name: "cur_ki", label: ["电流 Ki", "Current Ki"], min: 0 },
  { index: 0x7014, name: "cur_filt_gain", label: ["电流滤波系数", "Current filter gain"], min: 0, max: 1 },
  { index: 0x7016, name: "loc_ref", label: ["目标位置", "Position target"], unit: "rad" },
  { index: 0x7017, name: "limit_spd", label: ["CSP 速度限制", "CSP speed limit"], unit: "rad/s", min: 0, max: 50 },
  { index: 0x7018, name: "limit_cur", label: ["速度 / 位置电流限制", "Velocity / position current limit"], unit: "A", min: 0, max: 11 },
  { index: 0x7019, name: "mechPos", label: ["多圈机械位置", "Multi-turn position"], unit: "rad", readOnly: true },
  { index: 0x701a, name: "iqf", label: ["实际 Iq 电流", "Measured Iq current"], unit: "A", readOnly: true },
  { index: 0x701b, name: "mechVel", label: ["实际速度", "Measured velocity"], unit: "rad/s", readOnly: true },
  { index: 0x701c, name: "VBUS", label: ["母线电压", "Bus voltage"], unit: "V", readOnly: true },
  { index: 0x701e, name: "loc_kp", label: ["位置 Kp", "Position Kp"], min: 0 },
  { index: 0x701f, name: "spd_kp", label: ["速度 Kp", "Velocity Kp"], min: 0 },
  { index: 0x7020, name: "spd_ki", label: ["速度 Ki", "Velocity Ki"], min: 0 },
  { index: 0x7021, name: "spd_filt_gain", label: ["速度滤波系数", "Velocity filter gain"], min: 0, max: 1 },
  { index: 0x7022, name: "acc_rad", label: ["速度模式加速度", "Velocity acceleration"], unit: "rad/s²", min: 0 },
  { index: 0x7024, name: "vel_max", label: ["PP 最高速度", "PP maximum velocity"], unit: "rad/s", min: 0, max: 50 },
  { index: 0x7025, name: "acc_set", label: ["PP 加速度", "PP acceleration"], unit: "rad/s²", min: 0 },
  { index: 0x7026, name: "EPScan_time", label: ["主动反馈周期", "Feedback period"], min: 0, max: 65535, integer: true },
  { index: 0x7028, name: "canTimeout", label: ["CAN 超时阈值", "CAN timeout"], min: 0, max: 4294967295, integer: true },
  { index: 0x7029, name: "zero_sta", label: ["上电位置范围", "Power-on position range"], min: 0, max: 1, integer: true },
  { index: 0x702b, name: "add_offset", label: ["零位偏置", "Zero offset"], unit: "rad" },
  { index: 0x702c, name: "alveolous_open", label: ["齿槽补偿", "Cogging compensation"], min: 0, max: 1, integer: true },
  { index: 0x702d, name: "iq_test", label: ["上电初始化校准", "Power-on initialization calibration"], min: 0, max: 1, integer: true },
  { index: 0x702e, name: "dcc_set", label: ["PP 独立减速度", "PP separate deceleration"], unit: "rad/s²", min: 0 },
];

interface Sample {
  t: number;
  position: number | null;
  velocity: number | null;
  torque: number | null;
  positionVelocity: number | null;
}

function defaultInterface() {
  return /linux/i.test(navigator.userAgent) ? "can0" : "gs_usb0";
}

export function El05MotorPanel() {
  return <MotorCanWorkspace protocol="Native" api={el05Api} />;
}

export function MotorCanWorkspace({ protocol, api }: { protocol: El05Protocol; api: MotorWorkspaceApi }) {
  const { lang } = useI18n();
  const { message } = App.useApp();
  const zh = lang === "zh";
  const tr = (cn: string, en: string) => zh ? cn : en;
  const [iface, setIface] = useState(defaultInterface);
  const [extensions, setExtensions] = useState(false);
  const [hostId, setHostId] = useState("0xFD");
  const [manualId, setManualId] = useState("0x01");
  const [newMotorId, setNewMotorId] = useState("");
  const [state, setState] = useState<El05State | null>(null);
  const [busy, setBusy] = useState(false);
  const [benchWorking, setBenchWorking] = useState(false);
  const [stopping, setStopping] = useState(false);
  const [pollError, setPollError] = useState<string | null>(null);
  const [refreshHz, setRefreshHz] = useState(20);
  const [view, setView] = useState<"numeric" | "chart">("numeric");
  const [workspaceTab, setWorkspaceTab] = useState<"bench" | "gravity" | "manual" | "settings">(protocol === "StandardMit" ? "manual" : "bench");
  const [windowSec, setWindowSec] = useState(10);
  const [repeat, setRepeat] = useState(true);
  const [position, setPosition] = useState(0);
  const [velocity, setVelocity] = useState(0);
  const [torque, setTorque] = useState(0);
  const [kp, setKp] = useState(0);
  const [kd, setKd] = useState(0.1);
  const [current, setCurrent] = useState(0);
  const [speedLimit, setSpeedLimit] = useState(1);
  const [currentLimit, setCurrentLimit] = useState(1);
  const [acceleration, setAcceleration] = useState(5);
  const [parameterIndex, setParameterIndex] = useState(0x7019);
  const [parameterValue, setParameterValue] = useState<number | null>(null);
  const [parameterReadback, setParameterReadback] = useState<string | null>(null);
  const samples = useRef<Sample[]>([]);
  const lastSample = useRef<{ motor: number | null; rx: number; online: boolean }>({ motor: null, rx: -1, online: false });
  const [chartVersion, setChartVersion] = useState(0);
  const [coordinateRevision, setCoordinateRevision] = useState(0);
  const alive = useRef(true);
  const actionInFlight = useRef(false);
  const connectionEpoch = useRef(0);
  const connected = !!state?.connected;
  const standardMit = protocol === "StandardMit";
  const manualExtensions = standardMit && (connected ? !!state.manual_extensions : extensions);
  const hasDictionary = !standardMit || manualExtensions;
  const selected = connected && state?.attached && state.motor_id != null;
  const mode = state?.mode ?? "Mit";
  const faulted = !!state && (state.fault_bits !== 0 || state.fault_code !== 0 || (standardMit && state.warning_code !== 0));
  const enabled = !!state?.enabled;
  const gravityActive = !!state?.gravity_compensation?.active;
  const controlLocked = busy || gravityActive || benchWorking || stopping;
  const motionReady = selected && !!state?.online && enabled && !faulted && !state?.experiment.active && !gravityActive && !benchWorking && !stopping;
  const parameter = PARAMETERS.find((item) => item.index === parameterIndex)!;
  const proposedId = /^(?:0x[0-9a-f]+|\d+)$/i.test(newMotorId.trim()) ? Number(newMotorId) : NaN;
  const canChangeId = hasDictionary && selected && !controlLocked && Number.isInteger(proposedId) && proposedId >= 0 && proposedId <= 127
    && proposedId !== state?.motor_id && proposedId !== state?.host_id
    && (!standardMit || (!enabled && !state?.experiment.active && proposedId !== state?.reply_id));

  const acceptState = useCallback((next: El05State | null) => {
    if (!alive.current) return;
    const motor = next?.motor_id ?? null;
    const previous = lastSample.current;
    if (previous.motor !== motor) {
      samples.current = [];
      previous.motor = motor;
      previous.rx = -1;
      previous.online = false;
      setParameterValue(null);
      setParameterReadback(null);
      setNewMotorId("");
      if (next) {
        const initialPosition = next.mode === "Mit" ? next.position_rad : next.mechanical_position_rad;
        if (initialPosition != null) setPosition(initialPosition);
      }
    }
    if (next && motor != null) {
      const fresh = next.feedback_age_ms != null && next.feedback_age_ms <= 750;
      if (next.feedback_sequence > 0 && next.feedback_sequence !== previous.rx && fresh && next.feedback_time_s != null) {
        samples.current.push({ t: next.feedback_time_s * 1000, position: next.position_rad, velocity: next.velocity_rad_s, torque: next.torque_nm, positionVelocity: next.position_velocity_rad_s });
      } else if (previous.online && !fresh) {
        samples.current.push({ t: (next.feedback_time_s ?? 0) * 1000 + (next.feedback_age_ms ?? 0), position: null, velocity: null, torque: null, positionVelocity: null });
      }
      previous.rx = next.feedback_sequence;
      previous.online = fresh;
      const cutoff = (samples.current[samples.current.length - 1]?.t ?? 0) - 60_000;
      let remove = 0;
      while (remove < samples.current.length && samples.current[remove].t < cutoff) remove++;
      if (remove) samples.current.splice(0, remove);
    }
    setState(next);
  }, []);

  useEffect(() => {
    alive.current = true;
    return () => { alive.current = false; };
  }, []);

  // This serialized state poll also renews the backend's operator heartbeat.
  useEffect(() => {
    let disposed = false;
    let timer: number | undefined;
    let failures = 0;
    const poll = async () => {
      const started = performance.now();
      const epoch = connectionEpoch.current;
      try {
        const next = await api.getState();
        if (disposed || epoch !== connectionEpoch.current) return;
        acceptState(next);
        failures = 0;
        setPollError(null);
      } catch (error) {
        if (!disposed && ++failures >= 3) setPollError(errMsg(error));
      } finally {
        if (!disposed) timer = window.setTimeout(poll, Math.max(0, 1000 / refreshHz - (performance.now() - started)));
      }
    };
    void poll();
    return () => {
      disposed = true;
      if (timer != null) window.clearTimeout(timer);
    };
  }, [acceptState, refreshHz, api]);

  useEffect(() => {
    if (view !== "chart") return;
    const timer = window.setInterval(() => setChartVersion((value) => value + 1), 50);
    return () => window.clearInterval(timer);
  }, [view]);

  const run = async (operation: () => Promise<unknown>, success?: string) => {
    if (actionInFlight.current) return;
    actionInFlight.current = true;
    setBusy(true);
    try {
      await operation();
      acceptState(await api.getState());
      if (success) message.success(success);
    } catch (error) {
      message.error(errMsg(error));
      // An ID write can succeed even if the following mode query fails.
      try { acceptState(await api.getState()); } catch { /* Keep the original operation error. */ }
    } finally {
      actionInFlight.current = false;
      if (alive.current) setBusy(false);
    }
  };

  const connect = () => void run(async () => {
    const id = parseId(hostId, 0xff);
    if (!iface.trim()) throw new Error(tr("请输入 CAN 接口", "Enter a CAN interface"));
    connectionEpoch.current++;
    await api.connect(iface.trim(), id, manualExtensions);
  }, tr("电机 CAN 已连接", "Motor CAN connected"));

  const disconnect = () => void run(async () => {
    connectionEpoch.current++;
    await api.disconnect();
    acceptState(null);
  }, tr("电机已停止，CAN 已断开", "Motor stopped and CAN disconnected"));

  const stop = async () => {
    setStopping(true);
    try {
      await api.stop();
      acceptState(await api.getState());
      message.success(tr("电机已停止", "Motor stopped"));
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      if (alive.current) setStopping(false);
    }
  };

  const updatePositionTarget = async (next: El05State | null) => {
    if (!next) return;
    if (next.mode === "PositionPp" || next.mode === "PositionCsp") {
      // The periodic feedback wraps at ±4π. Read the absolute multi-turn
      // parameter before prefilling an absolute position command.
      const measured = await api.readParameter(0x7019);
      setPosition(measured.value);
    } else if (next.mode === "Mit" && next.position_rad != null) {
      setPosition(next.position_rad);
    }
  };

  const selectMotorOperation = async (motorId: number) => {
    setCoordinateRevision(value => value + 1);
    await api.selectMotor(motorId);
    const next = await api.getState();
    await updatePositionTarget(next);
    acceptState(next);
    setParameterValue(null);
    setParameterReadback(null);
  };

  const selectMotor = (motorId: number) => void run(() => selectMotorOperation(motorId));

  const changeMode = (nextMode: El05Mode) => void run(async () => {
    await api.setMode(nextMode);
    const next = await api.getState();
    await updatePositionTarget(next);
    acceptState(next);
  }, tr("控制模式已切换，请重新使能", "Mode changed; enable the motor to continue"));

  const makeTarget = (): El05Target => {
    switch (mode) {
      case "Mit": return { kind: mode, position_rad: position, velocity_rad_s: velocity, torque_nm: torque, kp, kd };
      case "Current": return { kind: mode, current_a: current };
      case "Velocity": return { kind: mode, velocity_rad_s: velocity, acceleration_rad_s2: acceleration, current_limit_a: currentLimit };
      case "PositionPp": return { kind: mode, position_rad: position, velocity_rad_s: speedLimit, acceleration_rad_s2: acceleration, current_limit_a: currentLimit };
      case "PositionCsp": return { kind: mode, position_rad: position, velocity_rad_s: speedLimit, current_limit_a: currentLimit };
    }
  };

  const setZero = () => run(async () => {
    setCoordinateRevision(value => value + 1);
    // Clear coordinates even if an acknowledgement is lost after zeroing.
    setPosition(0);
    samples.current = [];
    setParameterValue(null);
    setParameterReadback(null);
    try {
      await api.setZero();
      const next = await api.getState();
      acceptState(next);
      await updatePositionTarget(next);
    } finally {
      samples.current = [];
      setChartVersion((value) => value + 1);
    }
  }, tr("机械零点已设置，位置目标已更新；电机保持停止", "Mechanical zero set and position target refreshed; motor remains stopped"));

  const changeMotorId = () => run(async () => {
    const id = parseId(newMotorId, 127);
    await api.changeId(id);
    setManualId(hexId(id));
    setNewMotorId("");
    const next = await api.getState();
    acceptState(next);
    await updatePositionTarget(next);
    setCoordinateRevision(value => value + 1);
  }, standardMit ? tr("ID 修改已应答，已选择新地址；请重新记录下垂参考", "ID change acknowledged; new address selected. Capture a new downward reference.") : tr("ID 已修改并通过 MCU UID 核对，已选择新地址", "ID changed and MCU UID verified; new address selected"));

  const modeHint: Record<El05Mode, string> = {
    Mit: tr("τ = Kp·(目标位置−实际位置) + Kd·(目标速度−实际速度) + 前馈转矩。速度单位 rad/s（1 rad/s ≈ 9.5493 rpm）；Kd=0 时速度目标不起作用，Kp>0 且固定位置时不适合连续匀速转动。", "τ = Kp·(position target−position) + Kd·(velocity target−velocity) + feedforward. Speed is rad/s (1 rad/s ≈ 9.5493 rpm). Kd=0 ignores velocity; a fixed position with Kp>0 opposes continuous rotation.") + " " + (standardMit
      ? tr("速度反馈范围 ±50 rad/s，12 位分辨率约 0.02442 rad/s。", "Feedback is ±50 rad/s, with 12-bit resolution ≈0.02442 rad/s.")
      : tr("速度反馈范围 ±50 rad/s，16 位分辨率约 0.001526 rad/s。", "Feedback is ±50 rad/s, with 16-bit resolution ≈0.001526 rad/s.")),
    PositionPp: tr("一次发送位置、最高速度和加速度，电机内部完成轨迹。加速度同时用于加速和减速。", "Send position, maximum speed and acceleration once; the motor plans the trajectory. Acceleration also controls deceleration."),
    Velocity: standardMit ? tr("按手册标准帧发送目标速度和电流限制；速度单位 rad/s。", "Sends the manual's standard frame with velocity (rad/s) and current limit.") : tr("先应用电流限制和加速度，再发送目标速度。速度单位为 rad/s。", "Applies current limit and acceleration before the velocity target, in rad/s."),
    Current: tr("直接控制 Iq 电流。正负号决定输出方向，范围为 ±11 A。", "Direct Iq current control. The sign sets output direction; the range is ±11 A."),
    PositionCsp: standardMit ? tr("按手册模式 1 发送位置和最高速度，电流限制请在设备参数 0x7018 中设置。使能前需读回多圈位置（需固件支持参数读取）。", "Manual mode 1 sends position and maximum velocity. Set the current limit using parameter 0x7018. Enabling first reads the multi-turn position and requires dictionary support.") : tr("位置指令先应用速度和电流限制，再更新目标角度。可连续更新目标位置。", "Applies velocity and current limits before the position target. Position targets can be updated continuously."),
  };

  const readParameter = () => void run(async () => {
    const result = await api.readParameter(parameterIndex);
    setParameterValue(result.value);
    setParameterReadback(`${hexId(result.index, 4)} = ${result.value} (${result.kind})`);
  });

  const writeParameter = () => void run(async () => {
    if (parameterValue == null || !Number.isFinite(parameterValue)) throw new Error(tr("请填写有效的参数值", "Enter a finite parameter value"));
    if (parameterIndex === 0x7005) {
      const modes: Partial<Record<number, El05Mode>> = standardMit ? { 0: "Mit", 1: "PositionCsp", 2: "Velocity" } : { 0: "Mit", 1: "PositionPp", 2: "Velocity", 3: "Current", 5: "PositionCsp" };
      const nextMode = modes[parameterValue];
      if (!nextMode) throw new Error(standardMit ? tr("MIT 模式值必须为 0、1 或 2", "MIT mode must be 0, 1 or 2") : tr("模式值必须为 0、1、2、3 或 5", "Mode must be 0, 1, 2, 3 or 5"));
      await api.setMode(nextMode);
      const next = await api.getState();
      await updatePositionTarget(next);
      acceptState(next);
    } else {
      await api.writeParameter(parameterIndex, parameterValue);
    }
    const result = await api.readParameter(parameterIndex);
    setParameterValue(result.value);
    setParameterReadback(`${hexId(result.index, 4)} = ${result.value} (${result.kind})`);
  }, tr("参数已写入并读回", "Parameter written and read back"));

  const statusLabel = !state?.online ? tr("离线", "Offline") : state.status_reliable === false
    ? enabled ? tr("控制已使能", "Control enabled") : tr("停止已应答", "Stop acknowledged")
    : enabled ? tr("运行", "Running") : state.status_code === 1 ? tr("标定", "Calibrating") : tr("已停止", "Stopped");

  const exportMonitor = () => {
    const rows = samples.current.map(sample => [sample.t / 1000, sample.position, sample.velocity, sample.torque, sample.positionVelocity].map(value => value ?? "").join(","));
    const url = URL.createObjectURL(new Blob([["session_time_s,position_rad,velocity_rad_s,torque_nm,position_velocity_rad_s", ...rows].join("\n")], { type: "text/csv;charset=utf-8" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = `${standardMit ? "mit" : "el05"}-monitor-${state?.motor_id ?? "motor"}.csv`;
    link.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
  };

  return (
    <div className="el05-panel">
      <Card size="small">
        <div className="el05-connection">
          {standardMit && <Field label={tr("电机指令配置", "Motor command profile")}><Select aria-label={tr("电机指令配置", "Motor command profile")} value={manualExtensions ? "el05" : "basic"} disabled={connected || busy} style={{ width: 240 }} onChange={value => setExtensions(value === "el05")} options={[
            { value: "basic", label: tr("基础 MIT 兼容", "Basic MIT compatible") },
            { value: "el05", label: tr("EL05 手册扩展 (260922)", "EL05 manual extensions (260922)") },
          ]} /></Field>}
          <Field label={tr("CAN 接口", "CAN interface")}><Input value={iface} disabled={connected || busy} onChange={(event) => setIface(event.target.value)} style={{ width: 150 }} placeholder={defaultInterface()} /></Field>
          {!standardMit && <Field label={tr("主机 CAN ID", "Host CAN ID")}><Input value={hostId} disabled={connected || busy} onChange={(event) => setHostId(event.target.value)} style={{ width: 110 }} /></Field>}
          {connected ? <Button danger loading={busy} onClick={disconnect}>{tr("停止并断开", "Stop & disconnect")}</Button> : <Button type="primary" loading={busy} onClick={connect}>{tr("连接", "Connect")}</Button>}
          <Space wrap>
            <Tag color={connected ? "green" : "default"}>{connected ? tr("已连接", "Connected") : tr("未连接", "Disconnected")}</Tag>
            <Tag>Classic CAN · 1 Mbps</Tag>
            <Typography.Text type="secondary">{standardMit ? tr("标准帧 MIT · 回复 ID 自动识别", "Standard MIT · reply ID detected automatically") : tr("RobStride 私有协议 · 29 位扩展帧", "RobStride native protocol · 29-bit extended frames")}</Typography.Text>
          </Space>
        </div>
        {standardMit && <Typography.Paragraph type="secondary" style={{ margin: "12px 0 0" }}>{tr("连接后扫描或输入电机 ID；回复 ID 自动识别。扫描会向 0–255 地址发送停止命令。基础配置提供 MIT 控制与测试；确认电机支持 EL05 手册后，可选扩展配置使用位置、速度和设备参数。", "Connect, then scan or enter a motor ID; the reply ID is detected automatically. Scanning sends Stop to IDs 0–255. The basic profile provides MIT control and tests. Choose the extended profile for position, velocity and settings when your drive supports the EL05 manual.")}</Typography.Paragraph>}
        {pollError && <Alert style={{ marginTop: 12 }} type="error" showIcon message={pollError} />}
      </Card>

      <div className="el05-workspace">
        <aside className="el05-sidebar">
          <Card size="small" title={standardMit ? tr("标准 MIT 电机", "Standard MIT motors") : tr("EL05 电机", "EL05 motors")} extra={<Button size="small" disabled={!connected || controlLocked || (standardMit && (enabled || state?.experiment.active))} loading={busy} onClick={() => void run(() => api.scan())}>{tr("扫描", "Scan")}</Button>}>
            {(state?.devices.length ?? 0) === 0 ? (
              <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={connected ? tr("点击扫描发现电机", "Scan to discover motors") : tr("先连接 CAN", "Connect CAN first")} />
            ) : state?.devices.map((device) => (
              <button key={device.motor_id} className={`el05-device${device.selected ? " el05-device--selected" : ""}`} type="button" disabled={controlLocked} onClick={() => selectMotor(device.motor_id)}>
                <span className="el05-device__title"><strong>{standardMit ? tr("MIT 兼容电机", "MIT compatible motor") : "RobStride EL05"}</strong><code>{hexId(device.motor_id)}</code></span>
                <span><Tag color={device.online ? "green" : "default"}>{device.online ? tr("在线", "Online") : tr("已发现", "Discovered")}</Tag>{device.selected && <Tag color="blue">{tr("已选择", "Selected")}</Tag>}</span>
                {device.uid && <Typography.Text type="secondary" className="el05-device__uid">UID {device.uid}</Typography.Text>}
              </button>
            ))}
            <Typography.Text type="secondary">{tr("或指定电机 ID（十进制 / 0x）", "Or enter a motor ID (decimal / 0x)")}</Typography.Text>
            <div className="el05-manual">
              <Input aria-label={tr("电机 ID", "Motor ID")} value={manualId} disabled={!connected || controlLocked} onChange={(event) => setManualId(event.target.value)} />
              <Button disabled={!connected || controlLocked} onClick={() => void run(() => selectMotorOperation(parseId(manualId, 0xff)))}>{tr("选择", "Select")}</Button>
            </div>
            <Typography.Text className="el05-description" type="secondary">{tr("切换电机前会停止当前电机。", "Changing selection stops the current motor.")}</Typography.Text>
          </Card>
        </aside>

        <section className="el05-detail">
          <Space direction="vertical" size={12} style={{ width: "100%" }}>
            <Card size="small">
              <div className="el05-summary">
                <div>
                  <Space wrap><Typography.Title level={4} style={{ margin: 0 }}>{standardMit ? tr("标准 MIT 电机", "Standard MIT motor") : "RobStride EL05"}</Typography.Title>{state?.motor_id != null && <Typography.Text code>{hexId(state.motor_id)}</Typography.Text>}<Tag color={faulted ? "red" : enabled ? "green" : state?.online ? "blue" : "default"}>{statusLabel}</Tag>{selected && <Tag>{MODE_LABELS[mode][zh ? 0 : 1]}</Tag>}</Space>
                  <Typography.Text className="el05-description" type="secondary">{standardMit ? tr("EL05 兼容量程：位置 ±12.57 rad · 速度 ±50 rad/s · 转矩 ±6 Nm", "EL05-compatible ranges: position ±12.57 rad · velocity ±50 rad/s · torque ±6 Nm") : <>48 V · {tr("额定 1.8 Nm · 峰值 6 Nm · 峰值相电流 11 A", "Rated 1.8 Nm · Peak 6 Nm · Peak phase current 11 A")}</>}</Typography.Text>
                </div>
                <Space wrap><Space size={5}><Typography.Text type="secondary">{tr("界面刷新", "UI refresh")}</Typography.Text><Segmented size="small" value={refreshHz} onChange={(value) => setRefreshHz(Number(value))} options={[{ label: "20 Hz", value: 20 }, { label: "50 Hz", value: 50 }]} /></Space><Button danger disabled={!selected} loading={stopping} onClick={() => void stop()}>{tr("停止电机", "Stop motor")}</Button></Space>
              </div>
            </Card>

            {!selected && <Alert type="info" showIcon message={tr("连接总线后扫描并选择电机，或手动输入电机 ID。", "Connect the bus, then scan and select a motor, or enter its ID manually.")} />}
            {selected && !state?.online && <Alert type="warning" showIcon message={tr("尚无有效反馈，请检查电源、1 Mbps 总线速率、接线及电机当前协议。", "No fresh feedback. Check power, 1 Mbps bus timing, wiring and the motor's current protocol.")} />}
            {state?.last_error && <Alert type="error" showIcon message={state.last_error} />}
            {faulted && <Alert type="error" showIcon message={tr("电机故障 / 预警", "Motor fault / warning")} description={<Space wrap>{!standardMit && faultLabels(state!.fault_bits, zh).map((label) => <Tag color="red" key={label}>{label}</Tag>)}<Typography.Text code>{`Fault ${hexId(state!.fault_code, 8)}`}</Typography.Text>{state!.faults.length > 0 && <Typography.Text>{state!.faults.join(" · ")}</Typography.Text>}</Space>} />}
            {gravityActive && workspaceTab !== "gravity" && <Alert type="info" showIcon
              message={tr("零重力保持正在运行；调整控制或开始测试前请先停止。", "Zero-gravity hold is running. Stop it before changing control or starting an experiment.")}
              action={<Button size="small" onClick={() => setWorkspaceTab("gravity")}>{tr("查看补偿", "View compensation")}</Button>} />}
            {benchWorking && <Alert type="info" showIcon message={tr("正在处理摆锤测试，请稍候；可随时停止电机。", "Processing the pendulum experiment. You can stop the motor at any time.")} />}

            <Segmented className="el05-workspace-tabs" block value={workspaceTab} onChange={value => setWorkspaceTab(value as typeof workspaceTab)} options={[
              { label: tr("摆锤性能测试", "Pendulum tests"), value: "bench" },
              { label: tr("零重力保持", "Zero-gravity hold"), value: "gravity" },
              { label: tr("手动控制与监视", "Manual control & monitor"), value: "manual" },
              { label: tr("设备设置与参数", "Settings & parameters"), value: "settings" },
            ]} />
            <div hidden={workspaceTab !== "bench"}>
              <El05ExperimentPanel api={api} protocol={protocol} state={state} zh={zh} busy={busy || stopping} coordinateRevision={coordinateRevision} refresh={async () => acceptState(await api.getState())} onWorkingChange={setBenchWorking} />
            </div>
            <div hidden={workspaceTab !== "gravity"}>
              <El05GravityPanel api={api} state={state} zh={zh} busy={busy || benchWorking} stopping={stopping} run={run} stop={stop} setZero={setZero} />
            </div>
            <div hidden={workspaceTab !== "manual"}><Space direction="vertical" size={12} style={{ width: "100%" }}>
            <Card size="small" title={tr("电机控制", "Motor control")} extra={state?.streaming && <Tag color="processing">{tr("目标连续发送中", "Target streaming")}</Tag>}>
              <Space wrap>
                <Select aria-label={tr("控制模式", "Control mode")} style={{ width: 200 }} value={mode} disabled={!selected || controlLocked || (standardMit && !manualExtensions)} options={(standardMit ? manualExtensions ? ["Mit", "PositionCsp", "Velocity"] as El05Mode[] : ["Mit" as const] : MODES).map((value) => ({ label: MODE_LABELS[value][zh ? 0 : 1], value }))} onChange={changeMode} />
                <Button type="primary" disabled={!selected || !state?.online || enabled || faulted || controlLocked || state?.experiment.active} onClick={() => void run(() => api.enable())}>{tr("使能", "Enable")}</Button>
                <Button type="primary" danger disabled={!selected} loading={stopping} onClick={() => void stop()}>{tr("停止 / 失能", "Stop / disable")}</Button>
                {hasDictionary && <Button disabled={!selected || controlLocked || state?.experiment.active} onClick={() => void run(() => api.clearFault())}>{tr("清除故障", "Clear fault")}</Button>}
              </Space>
              <Typography.Text className="el05-description" type="secondary">{tr("切换模式会先停止电机；使能后点击「发送目标」执行。", "Changing mode stops the motor. Enable, then press Send target to execute.")}</Typography.Text>
              {standardMit && mode === "Mit" && <Space wrap style={{ marginTop: 12 }}>
                <Typography.Text type="secondary">{tr("填入预设", "Fill preset")}</Typography.Text>
                <Button size="small" disabled={controlLocked || !!state?.experiment.active} onClick={() => { setKp(0); setKd(0.5); setVelocity(0); setTorque(0); }}>{tr("被动阻尼", "Damping")}</Button>
                <Button size="small" disabled={controlLocked || !!state?.experiment.active} onClick={() => { setKp(0); setKd(0.5); setVelocity(0.3); setTorque(0); }}>{tr("低速转动", "Low speed")}</Button>
                <Button size="small" disabled={controlLocked || !state?.online || state.position_rad == null || Math.abs(state.position_rad) >= 12.56 || !!state?.experiment.active} onClick={() => { setPosition(state!.position_rad!); setKp(3); setKd(0.5); setVelocity(0); setTorque(0); }}>{tr("保持当前角度", "Hold current angle")}</Button>
                <Typography.Text type="secondary">{tr("仅填入参数，点击发送目标后执行", "Fills fields; press Send target to apply")}</Typography.Text>
              </Space>}
              <div className="el05-target">
                <Typography.Text strong>{MODE_LABELS[mode][zh ? 0 : 1]}</Typography.Text>
                <div className="el05-target__fields">
                  {(mode === "Mit" || mode === "PositionPp" || mode === "PositionCsp") && <NumberField label={tr("目标位置", "Target position")} unit="rad" value={position} min={mode === "Mit" ? -12.57 : undefined} max={mode === "Mit" ? 12.57 : undefined} step={0.01} onChange={setPosition} />}
                  {(mode === "Mit" || mode === "Velocity") && <NumberField label={tr("目标速度", "Target velocity")} unit="rad/s" value={velocity} min={-50} max={50} step={0.1} onChange={setVelocity} />}
                  {mode === "Mit" && <><NumberField label={tr("前馈转矩", "Feed-forward torque")} unit="Nm" value={torque} min={-6} max={6} step={0.05} onChange={setTorque} /><NumberField label="Kp" unit="Nm/rad" value={kp} min={0} max={500} step={0.1} onChange={setKp} /><NumberField label="Kd" unit="Nm·s/rad" value={kd} min={0} max={5} step={0.01} onChange={setKd} /></>}
                  {mode === "Current" && <NumberField label={tr("目标 Iq 电流", "Target Iq current")} unit="A" value={current} min={-11} max={11} step={0.05} onChange={setCurrent} />}
                  {(mode === "PositionPp" || mode === "PositionCsp") && <NumberField label={tr("最高速度", "Maximum velocity")} unit="rad/s" value={speedLimit} min={0.001} max={50} step={0.1} onChange={setSpeedLimit} />}
                  {(mode === "Velocity" || mode === "PositionPp" || (!standardMit && mode === "PositionCsp")) && <NumberField label={tr("电流限制", "Current limit")} unit="A" value={currentLimit} min={0} max={11} step={0.1} onChange={setCurrentLimit} />}
                  {!standardMit && (mode === "Velocity" || mode === "PositionPp") && <NumberField label={tr("加速度", "Acceleration")} unit="rad/s²" value={acceleration} min={0.001} step={0.5} onChange={setAcceleration} />}
                </div>
                <Typography.Text type="secondary">{modeHint[mode]}</Typography.Text>
              </div>
              <Space wrap style={{ marginTop: 14 }}>
                <Button type="primary" loading={busy} disabled={!motionReady || state?.experiment.active} onClick={() => void run(() => api.sendTarget(makeTarget(), standardMit || (repeat && mode !== "PositionPp")))}>{tr("发送目标", "Send target")}</Button>
                <Switch checked={standardMit || (repeat && mode !== "PositionPp")} disabled={standardMit || mode === "PositionPp" || busy} onChange={setRepeat} />
                <Typography.Text>{standardMit ? tr("标准 MIT 需要持续通信", "Standard MIT requires continuous commands") : mode === "PositionPp" ? tr("PP 轨迹单次发送", "PP trajectory is sent once") : tr("连续发送目标", "Repeat target")}</Typography.Text>
                {(mode === "Mit" || mode === "PositionPp" || mode === "PositionCsp") && <Button disabled={!state?.online || (mode !== "Mit" && state.mechanical_position_rad == null)} onClick={() => setPosition((mode === "Mit" ? state?.position_rad : state?.mechanical_position_rad) ?? 0)}>{tr("取当前位置", "Use current position")}</Button>}
              </Space>
            </Card>

            <Card size="small" title={tr("信息显示", "Display")} extra={<Space wrap>{view === "chart" && <Space size={4}><InputNumber aria-label={tr("图表窗口秒数", "Chart window seconds")} size="small" min={1} max={60} value={windowSec} onChange={(value) => setWindowSec(value ?? 10)} style={{ width: 72 }} /><Typography.Text type="secondary">s</Typography.Text></Space>}<Segmented value={view} onChange={(value) => setView(value as "numeric" | "chart")} options={[{ label: tr("数值", "Numeric"), value: "numeric" }, { label: tr("图表", "Chart"), value: "chart" }]} /></Space>}>
              {view === "chart" ? <El05Chart samples={samples.current} version={chartVersion} windowSec={windowSec} zh={zh} /> : (
                <div className="el05-metrics">
                  <Metric label={tr("周期位置", "Wrapped position")} value={state?.position_rad} unit="rad" />
                  <Metric label={tr("反馈速度", "Feedback velocity")} value={state?.velocity_rad_s} unit="rad/s" digits={4} />
                  <Metric label={tr("驱动估计转矩", "Drive torque estimate")} value={state?.torque_nm} unit="Nm" />
                  <Metric label={tr("Iq 电流", "Iq current")} value={state?.current_a} unit="A" />
                  <Metric label={tr("母线电压", "Bus voltage")} value={state?.voltage_v} unit="V" digits={2} />
                  <Metric label={tr("温度", "Temperature")} value={state?.temperature_c} unit="°C" digits={1} />
                </div>
              )}
              <Descriptions size="small" column={{ xs: 1, sm: 2, lg: 3 }} style={{ marginTop: 16 }}>
                <Descriptions.Item label={tr("多圈机械位置", "Multi-turn position")}>{!hasDictionary ? tr("基础配置不提供", "Unavailable in basic profile") : <><Numeric value={state?.mechanical_position_rad} /> rad</>}</Descriptions.Item>
                {standardMit && <Descriptions.Item label={tr("连接期间累计位置", "Position unwrapped this session")}><Numeric value={state?.unwrapped_position_rad} /> rad</Descriptions.Item>}
                <Descriptions.Item label={tr("位置差分速度（约 150 ms 窗口）", "Position-derived speed (~150 ms)")}><Numeric value={state?.position_velocity_rad_s} digits={4} /> rad/s</Descriptions.Item>
                <Descriptions.Item label={tr("mechVel 参数（独立采样）", "mechVel (separate sample)")}>{!hasDictionary ? tr("基础配置不提供", "Unavailable in basic profile") : <><Numeric value={state?.dictionary_velocity_rad_s} digits={4} /> rad/s {state?.dictionary_velocity_age_ms != null ? `(${state.dictionary_velocity_age_ms} ms)` : ""}</>}</Descriptions.Item>
                <Descriptions.Item label={tr("转速换算", "Speed in rpm")}><Numeric value={state?.velocity_rad_s == null ? null : state.velocity_rad_s * 60 / (2 * Math.PI)} /> rpm</Descriptions.Item>
                <Descriptions.Item label={tr("反馈频率", "Feedback rate")}><Numeric value={state?.feedback_rate_hz} digits={1} /> Hz</Descriptions.Item>
                <Descriptions.Item label={tr("反馈年龄", "Feedback age")}>{state?.feedback_age_ms == null ? "—" : `${state.feedback_age_ms} ms`}</Descriptions.Item>
                <Descriptions.Item label={tr("接收 / 发送帧", "RX / TX frames")}>{state ? `${state.rx_count} / ${state.tx_count}` : "—"}</Descriptions.Item>
                <Descriptions.Item label={tr("反馈方式", "Feedback source")}>{state?.reporting ? tr("主动上报开启", "Periodic reporting on") : standardMit ? tr("命令应答", "Command replies") : tr("主动上报关闭", "Periodic reporting off")}</Descriptions.Item>
                {standardMit && <Descriptions.Item label={tr("回复 CAN ID", "Reply CAN ID")}>{state?.reply_id == null ? "—" : hexId(state.reply_id, 3)}</Descriptions.Item>}
                {standardMit && <Descriptions.Item label={tr("原始状态位", "Raw status bits")}>{state?.status_code ?? "—"} · {tr("不用于判定使能", "Not used to confirm enable")}</Descriptions.Item>}
                <Descriptions.Item label={tr("警告码", "Warning code")}><Typography.Text code>{state ? hexId(state.warning_code, 8) : "—"}</Typography.Text></Descriptions.Item>
              </Descriptions>
              {standardMit && <Typography.Paragraph type="secondary" style={{ marginTop: 12 }}>{manualExtensions
                ? tr("运动帧提供位置、速度、估计转矩和温度；电流、电压和多圈位置可在停止后单独读取参数，其数值保留至下次读取。控制状态以主机使能/停止应答为准。", "Motion frames provide position, velocity, estimated torque and temperature. Read current, voltage and multi-turn position separately while stopped; these values remain until the next read. Control state follows the host enable/stop exchange.")
                : tr("基础配置提供位置、速度、估计转矩和温度；Iq、电压与硬件多圈位置不提供。控制状态表示主机已完成使能或停止应答，兼容电机的原始状态位可能始终为 0。", "The basic profile provides position, velocity, estimated torque and temperature; Iq, voltage and hardware multi-turn position are unavailable. Control state follows the host enable/stop exchange; compatible drives may always report raw status zero.")}</Typography.Paragraph>}
              <Space wrap style={{ marginTop: 12 }}>
                <Button size="small" disabled={!samples.current.length} onClick={exportMonitor}>{tr("导出监视 CSV", "Export monitor CSV")}</Button>
                <Button size="small" disabled={!samples.current.length} onClick={() => { samples.current = []; setChartVersion(value => value + 1); }}>{tr("清空曲线", "Clear traces")}</Button>
                <Typography.Text type="secondary">{tr("保留最近 60 秒界面采样；原始测试数据在摆锤测试中导出。", "Keeps 60 seconds of UI samples; export experiment data in Pendulum tests.")}</Typography.Text>
              </Space>
            </Card>

            </Space></div>
            <div hidden={workspaceTab !== "settings"}><Space direction="vertical" size={12} style={{ width: "100%" }}>
            {standardMit ? <StandardMitSettings state={state} zh={zh} extensions={manualExtensions}
              disabled={!selected || controlLocked || !!state?.experiment.active} run={run} setZero={setZero}
              newMotorId={newMotorId} setNewMotorId={setNewMotorId} canChangeId={!!canChangeId} changeMotorId={changeMotorId} /> : <>
            <Card size="small" title={tr("设备设置与信息", "Device settings & information")}>
              <Space direction="vertical" size={16} style={{ width: "100%" }}>
                <Descriptions size="small" layout="vertical" column={{ xs: 1, sm: 1, md: 2, lg: 2, xl: 2, xxl: 2 }}>
                  <Descriptions.Item label={tr("当前 CAN ID", "Current CAN ID")}>
                    <Space wrap size={8}>
                      <Typography.Text className="el05-number">{state?.motor_id == null ? "—" : `${state.motor_id} (${hexId(state.motor_id)})`}</Typography.Text>
                      {state?.identity && <Tag color="green">{tr("UID 已读取", "UID read")}</Tag>}
                    </Space>
                  </Descriptions.Item>
                  <Descriptions.Item label="MCU UID">{state?.identity ? <Typography.Text code copyable>{state.identity.uid}</Typography.Text> : "—"}</Descriptions.Item>
                  <Descriptions.Item label={tr("电机版本", "Motor version")}>
                    {state?.version ? <Typography.Text code copyable>{hexId(state.version.value, 8)}</Typography.Text> : tr("未读取", "Not read")}
                  </Descriptions.Item>
                  <Descriptions.Item label={tr("版本原始数据", "Raw version data")}>
                    {state?.version ? <Typography.Text code copyable>{state.version.raw.map((byte) => byte.toString(16).toUpperCase().padStart(2, "0")).join(" ")}</Typography.Text> : "—"}
                  </Descriptions.Item>
                </Descriptions>
                <Space wrap>
                  <Button disabled={!selected || controlLocked} onClick={() => void run(() => el05Api.readIdentity(), tr("ID / UID 已读取", "ID / UID read"))}>{tr("读取 ID / UID", "Read ID / UID")}</Button>
                  <Button disabled={!selected || controlLocked} onClick={() => void run(() => el05Api.readVersion(), tr("电机版本已读取，电机保持停止", "Motor version read; motor remains stopped"))}>{tr("读取版本（先停机）", "Read version (stops motor)")}</Button>
                </Space>
                <Typography.Text type="secondary">{tr("版本值按手册第 31 页的 4 字节字段显示。手册虽列出 AppCodeVersion、BootCodeVersion 等固件字符串，但未给出私有协议读取格式，也未定义独立硬件版本，因此此处不推测这些版本号。", "The motor version is the four-byte field defined on manual p.31. AppCodeVersion and BootCodeVersion are listed, but their native-protocol string reads and a separate hardware revision are not documented.")}</Typography.Text>
                <div className="el05-settings-row">
                  <Field label={tr("新 CAN ID（0–127）", "New CAN ID (0–127)")}>
                    <Input aria-label={tr("新 CAN ID", "New CAN ID")} value={newMotorId} disabled={!selected || controlLocked} placeholder={tr("十进制或 0x 十六进制", "Decimal or 0x hex")} onChange={(event) => setNewMotorId(event.target.value)} />
                  </Field>
                  <Popconfirm title={tr("修改电机 CAN ID？", "Change motor CAN ID?")}
                    description={tr(`将先停机并检查新 ID ${newMotorId}，修改后通过 UID 核对并选择新地址。`, `Stops the motor, checks ID ${newMotorId}, and selects the new address after verifying its UID.`)}
                    disabled={!canChangeId} okText={tr("修改 ID", "Change ID")} cancelText={tr("取消", "Cancel")} onConfirm={changeMotorId}>
                    <Button disabled={!canChangeId}>{tr("修改 ID", "Change ID")}</Button>
                  </Popconfirm>
                </div>
                <div className="el05-settings-zero">
                  <div>
                    <Typography.Text strong>{tr("机械零点", "Mechanical zero")}</Typography.Text>
                    <Typography.Paragraph type="secondary" style={{ margin: "6px 0" }}>{tr("将当前轴位置设为机械零点，先停机并读回多圈位置。手册支持 MIT / CSP 模式；设置后会清除旧位置曲线并更新目标位置。", "Sets the current shaft position as zero after stopping, then reads back multi-turn position. Supported in MIT / CSP. Clears the old position chart and refreshes the position target.")}</Typography.Paragraph>
                    <Typography.Text>{tr("当前多圈位置：", "Current multi-turn position: ")}<Numeric value={state?.mechanical_position_rad} /> rad</Typography.Text>
                  </div>
                  <Popconfirm title={tr("将当前位置设置为机械零点？电机会先停止。", "Set the current position as mechanical zero? The motor will stop first.")} okText={tr("设置零点", "Set zero")} cancelText={tr("取消", "Cancel")} onConfirm={setZero}>
                    <Button disabled={!selected || controlLocked || (mode !== "Mit" && mode !== "PositionCsp")}>{tr("设置零点", "Set zero")}</Button>
                  </Popconfirm>
                </div>
              </Space>
            </Card>
            </>}
            {hasDictionary && <Collapse items={[{ key: "parameters", label: tr("参数读写", "Read / write parameters"), children: <Space direction="vertical" size={12} style={{ width: "100%" }}>
              <div className="el05-parameter">
                <Field label={tr("手册参数", "Manual parameter")}><Select disabled={busy} showSearch optionFilterProp="label" value={parameterIndex} style={{ width: "100%" }} onChange={(value) => { setParameterIndex(value); setParameterValue(null); setParameterReadback(null); }} options={PARAMETERS.map((item) => ({ value: item.index, label: `${hexId(item.index, 4)} · ${item.name} · ${item.label[zh ? 0 : 1]}${item.readOnly ? tr("（只读）", " (read only)") : ""}` }))} /></Field>
                <NumberField label={tr("参数值", "Value")} unit={parameter.unit} value={parameterValue} min={parameter.min} max={parameter.max} step={parameter.integer ? 1 : 0.01} precision={parameter.integer ? 0 : undefined} onChange={setParameterValue} disabled={parameter.readOnly || busy} />
                <Button disabled={!selected || controlLocked || state?.experiment.active || (standardMit && enabled)} onClick={readParameter}>{tr("读取", "Read")}</Button>
                <Button disabled={!selected || controlLocked || parameter.readOnly || parameterValue == null || enabled || state?.experiment.active} onClick={writeParameter}>{tr("写入并读回", "Write & read back")}</Button>
              </div>
              {parameterReadback && <Typography.Text code copyable>{parameterReadback}</Typography.Text>}
              <Typography.Text type="secondary">{standardMit && parameterIndex === 0x7005 ? tr("0=MIT，1=位置 CSP，2=速度。", "0=MIT, 1=position CSP, 2=velocity.") : parameterHint(parameterIndex, zh)} {tr("参数写入在电机停止后开放。", "Parameter writes are available while the motor is stopped.")}</Typography.Text>
              {standardMit && <Typography.Text type="secondary">{tr("参数读写需要固件 1.0.5.0.4 或兼容实现；写入会读回校验，掉电保存请使用「保存参数」。", "Parameter access requires firmware 1.0.5.0.4 or a compatible implementation. Writes are verified by readback; use Save parameters for persistence.")}</Typography.Text>}
            </Space> }]} />}
            </Space></div>
          </Space>
        </section>
      </div>
    </div>
  );
}

function Field({ label, children }: { label: ReactNode; children: ReactNode }) {
  return <label className="el05-field"><Typography.Text type="secondary">{label}</Typography.Text>{children}</label>;
}

function NumberField({ label, unit, value, onChange, ...props }: { label: string; unit?: string; value: number | null; onChange: (value: number) => void; min?: number; max?: number; step?: number; precision?: number; disabled?: boolean }) {
  return <Field label={`${label}${unit ? ` (${unit})` : ""}`}><InputNumber value={value} onChange={(next) => onChange(next ?? 0)} {...props} /></Field>;
}

function Numeric({ value, digits = 3 }: { value: number | null | undefined; digits?: number }) {
  return <span className="el05-number">{value == null || !Number.isFinite(value) ? "—" : Number(value.toFixed(digits)).toFixed(digits)}</span>;
}

function Metric({ label, value, unit, digits }: { label: string; value: number | null | undefined; unit: string; digits?: number }) {
  return <div className="el05-metric"><Typography.Text type="secondary">{label}</Typography.Text><div className="el05-metric__value"><Numeric value={value} digits={digits} /><span className="el05-metric__unit">{unit}</span></div></div>;
}

function hexId(value: number, width = 2) {
  return `0x${value.toString(16).toUpperCase().padStart(width, "0")}`;
}

function parseId(raw: string, max: number) {
  if (!/^(?:0x[0-9a-f]+|\d+)$/i.test(raw.trim())) throw new Error(`Invalid CAN ID: ${raw}`);
  const value = Number(raw);
  if (!Number.isInteger(value) || value < 0 || value > max) throw new Error(`CAN ID: 0–${max}`);
  return value;
}

function faultLabels(bits: number, zh: boolean) {
  const labels = zh ? ["欠压", "驱动故障", "过温", "磁编码器故障", "堵转过载", "未标定"] : ["Undervoltage", "Drive fault", "Overtemperature", "Encoder fault", "Overload / stall", "Uncalibrated"];
  return labels.filter((_, index) => !!(bits & (1 << index)));
}

function parameterHint(index: number, zh: boolean) {
  const hints: Record<number, [string, string]> = {
    0x7005: ["0=运控，1=PP，2=速度，3=电流，5=CSP。", "0=Motion, 1=PP, 2=Velocity, 3=Current, 5=CSP."],
    0x7026: ["本机固件 0x0A050001 实测：0 约为 10 ms，1 约为 15 ms，与通用手册的起始编号不同。显示反馈频率包含主动上报和命令应答，请按抓包核对周期。", "Measured on firmware 0x0A050001: 0 ≈ 10 ms, 1 ≈ 15 ms, with a different starting index from the generic manual. Displayed receive rate combines periodic reports and command replies; verify periods from captured frames."],
    0x7028: ["20000 表示 1 秒；0 关闭电机 CAN 超时保护。", "20000 means 1 second; 0 disables the motor CAN timeout."],
    0x7029: ["0=上电位置 0～2π；1=上电位置 −π～π。", "0=power-on position 0 to 2π; 1=−π to π."],
    0x702c: ["0=关闭；1=开启。需要电机已完成齿槽标定。", "0=off; 1=on. Requires completed cogging calibration."],
    0x702d: ["1=下次上电进行更精细的初始化校准，初始化时间会延长。", "1 enables more precise initialization calibration on the next power-on, increasing initialization time."],
    0x702e: ["仅在固件 acc_stutus 已设置为 1 时单独控制 PP 减速度。", "Controls PP deceleration separately only when firmware acc_stutus is set to 1."],
  };
  return hints[index]?.[zh ? 0 : 1] ?? "";
}

function El05Chart({ samples, version, windowSec, zh }: { samples: Sample[]; version: number; windowSec: number; zh: boolean }) {
  const option = useMemo(() => {
    const signals = [
      { key: "position" as const, name: zh ? "周期位置 (rad)" : "Wrapped position (rad)", color: "#4f8cff", minSpan: 0.05 },
      { key: "velocity" as const, name: zh ? "速度 (rad/s)" : "Velocity (rad/s)", color: "#2ecc71", minSpan: 0.1 },
      { key: "torque" as const, name: zh ? "转矩 (Nm)" : "Torque (Nm)", color: "#f39c12", minSpan: 0.05 },
    ];
    const latestT = samples.length ? samples[samples.length - 1].t : performance.now();
    const recent = samples.filter((point) => point.t >= latestT - windowSec * 1000);
    const axisStyle = { axisLine: { lineStyle: { color: "#3a414d" } }, splitLine: { lineStyle: { color: "#222831" } }, axisLabel: { color: "#8a93a3" } };
    return {
      animation: false,
      grid: signals.map((_, index) => ({ left: 65, right: 25, top: `${8 + index * 31}%`, height: "22%" })),
      title: signals.map((signal, index) => ({ text: signal.name, right: 25, top: `${index * 31}%`, textStyle: { fontSize: 12, color: signal.color, fontWeight: "normal" } })),
      xAxis: signals.map((_, index) => ({ ...axisStyle, type: "value", gridIndex: index, min: -windowSec, max: 0, axisLabel: { color: "#8a93a3", show: index === 2, formatter: "{value}s" } })),
      yAxis: signals.map((signal, index) => {
        const values = recent.flatMap((point) => signal.key === "velocity" ? [point.velocity, point.positionVelocity] : [point[signal.key]]).filter((value): value is number => value != null && Number.isFinite(value));
        const min = Math.min(...values);
        const max = Math.max(...values);
        const narrow = values.length > 0 && max - min < signal.minSpan;
        return { ...axisStyle, axisLabel: { color: "#8a93a3", formatter: (value: number) => Number(value.toFixed(3)).toString() }, type: "value", gridIndex: index, scale: true, min: narrow ? (min + max - signal.minSpan) / 2 : null, max: narrow ? (min + max + signal.minSpan) / 2 : null };
      }),
      tooltip: { trigger: "axis", axisPointer: { type: "cross" }, valueFormatter: (value: unknown) => typeof value === "number" ? value.toFixed(4) : "—" },
      legend: { top: "31%", left: 65, textStyle: { color: "#8a93a3" }, data: [signals[1].name, zh ? "位置差分速度" : "Position-derived velocity"] },
      series: [...signals.map((signal, index) => ({ name: signal.name, type: "line", xAxisIndex: index, yAxisIndex: index, showSymbol: false, connectNulls: false, sampling: "lttb", lineStyle: { color: signal.color, width: 1.5 }, itemStyle: { color: signal.color }, data: recent.map((point) => [(point.t - latestT) / 1000, point[signal.key]]) })),
        { name: zh ? "位置差分速度" : "Position-derived velocity", type: "line", xAxisIndex: 1, yAxisIndex: 1, showSymbol: false, connectNulls: false, lineStyle: { color: "#b388ff", type: "dashed", width: 1.5 }, data: recent.map(point => [(point.t - latestT) / 1000, point.positionVelocity]) }],
    };
  }, [samples, version, windowSec, zh]);
  return <ReactECharts option={option} lazyUpdate style={{ height: 440, width: "100%" }} />;
}
