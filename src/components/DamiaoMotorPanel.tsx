import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { QuestionCircleOutlined } from "@ant-design/icons";
import {
  Alert,
  App,
  Button,
  Card,
  Collapse,
  Descriptions,
  Divider,
  Empty,
  Input,
  InputNumber,
  List,
  Popconfirm,
  Segmented,
  Select,
  Space,
  Switch,
  Tag,
  Tooltip,
  Typography,
} from "antd";
import { api, errMsg } from "../api";
import { useI18n } from "../i18n";
import { useDamiaoTelemetry } from "../useDamiaoTelemetry";
import type { DamiaoBitrateState, DamiaoConfig, DamiaoDiscoveredDevice, DamiaoMode, DamiaoState, DamiaoTarget } from "../types";
import { DamiaoLiveChart } from "./DamiaoLiveChart";
import "./DamiaoMotorPanel.css";

const MODE_OPTIONS: DamiaoMode[] = ["Mit", "PositionVelocity", "Velocity"];
const MAPPING_P_MAX_RAD = 12.5;
const MAPPING_V_MAX_RAD_S = 30;
const MAPPING_T_MAX_NM = 10;
const PHYSICAL_MAX_SPEED_RAD_S = (200 * 2 * Math.PI) / 60;
const PHYSICAL_PEAK_TORQUE_NM = 7;
const DEVICE_POLL_MS = 500;
const BITRATE_OPTIONS = [125_000, 200_000, 250_000, 500_000, 1_000_000, 2_000_000, 2_500_000, 3_200_000, 4_000_000, 5_000_000];

export function DamiaoMotorPanel({ connected, onBusyChange }: { connected: boolean; onBusyChange?: (busy: boolean) => void }) {
  const { message } = App.useApp();
  const { lang } = useI18n();
  const zh = lang === "zh";
  const [devices, setDevices] = useState<DamiaoDiscoveredDevice[]>([]);
  const [selectedMotorId, setSelectedMotorId] = useState<number | null>(null);
  const [manualKey, setManualKey] = useState(0);
  const [manual, setManual] = useState(false);
  const [discoveryError, setDiscoveryError] = useState<string | null>(null);
  const [listBusy, setListBusy] = useState<"rescan" | "disable" | null>(null);
  const [detailBusy, setDetailBusy] = useState(false);
  const listBusyRef = useRef(false);

  useEffect(() => {
    onBusyChange?.(detailBusy || listBusy != null);
  }, [detailBusy, listBusy, onBusyChange]);

  useEffect(() => () => onBusyChange?.(false), [onBusyChange]);

  const refreshDevices = useCallback(async () => {
    if (!connected) {
      setDevices([]);
      return;
    }
    const next = await api.damiaoListDevices();
    setDevices(next);
  }, [connected]);

  useEffect(() => {
    if (!connected) {
      setDevices([]);
      setSelectedMotorId(null);
      return;
    }
    let alive = true;
    let timer: number | undefined;
    const tick = async () => {
      try {
        const next = await api.damiaoListDevices();
        if (alive) {
          setDevices(next);
          setDiscoveryError(null);
        }
      } catch (error) {
        if (alive) setDiscoveryError(errMsg(error));
      } finally {
        if (alive) timer = window.setTimeout(tick, DEVICE_POLL_MS);
      }
    };
    void tick();
    return () => {
      alive = false;
      if (timer != null) window.clearTimeout(timer);
    };
  }, [connected]);

  useEffect(() => {
    if (!manual && selectedMotorId == null && devices.length > 0) {
      setSelectedMotorId(devices[0].motor_id);
    }
  }, [devices, manual, selectedMotorId]);

  const selectedDevice = devices.find((device) => device.motor_id === selectedMotorId);
  const attachedCount = devices.filter((device) => device.attached).length;
  const copy = zh
    ? {
        motors: "达妙电机",
        automatic: "自动发现电机",
        disconnected: "CAN 未连接",
        discoveryFailed: "无法刷新电机列表",
        waiting: "正在安全扫描 ID 0x0–0xF…",
        manual: "手动配置 ID",
        rescan: "安全重扫",
        rescanOk: "已请求安全重扫（仅失能未挂载 ID）",
        disableAll: "全部失能",
        disableAllOk: "所有已挂载电机均已失能",
        attached: "已挂载",
        online: "在线",
        offline: "离线",
      }
    : {
        motors: "DAMIAO motors",
        automatic: "Automatic motor discovery",
        disconnected: "CAN disconnected",
        discoveryFailed: "Unable to refresh the motor list",
        waiting: "Safely scanning IDs 0x0–0xF…",
        manual: "Configure an ID manually",
        rescan: "Safe rescan",
        rescanOk: "Safe rescan requested (only unattached IDs are disabled)",
        disableAll: "Disable all",
        disableAllOk: "All attached motors disabled",
        attached: "Attached",
        online: "Online",
        offline: "Offline",
      };

  const rescan = async () => {
    if (!connected || listBusyRef.current || detailBusy) return;
    listBusyRef.current = true;
    setListBusy("rescan");
    try {
      await api.damiaoSafeRescan();
      message.success(copy.rescanOk);
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      listBusyRef.current = false;
      setListBusy(null);
    }
  };

  const disableAll = async () => {
    if (!connected || listBusyRef.current || detailBusy) return;
    listBusyRef.current = true;
    setListBusy("disable");
    try {
      await api.damiaoDisableAll();
      message.success(copy.disableAllOk);
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      listBusyRef.current = false;
      setListBusy(null);
    }
  };

  return (
    <div className="damiao-workspace">
      <aside className="damiao-device-list">
        <div className="damiao-device-list__heading">
          <div>
            <Typography.Text strong>{copy.motors} ({devices.length})</Typography.Text>
            <Typography.Text type="secondary">{connected ? copy.automatic : copy.disconnected}</Typography.Text>
          </div>
          <Space size={6} wrap>
            <Button size="small" loading={listBusy === "rescan"} disabled={!connected || listBusy != null || detailBusy} onClick={rescan}>{copy.rescan}</Button>
            <Button size="small" danger loading={listBusy === "disable"} disabled={!connected || attachedCount === 0 || listBusy != null || detailBusy} onClick={disableAll}>{copy.disableAll}</Button>
          </Space>
        </div>
        {connected && discoveryError && <Alert className="damiao-discovery-error" type="warning" showIcon message={copy.discoveryFailed} description={discoveryError} />}
        {devices.length === 0 ? (
          <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={connected ? copy.waiting : undefined} />
        ) : (
          <List
            dataSource={devices}
            rowKey={(device) => device.motor_id}
            renderItem={(device) => {
              const selected = !manual && device.motor_id === selectedMotorId;
              return (
                <List.Item
                  className={`damiao-device-list__item${selected ? " damiao-device-list__item--selected" : ""}`}
                  role="button"
                  tabIndex={0}
                  aria-pressed={selected}
                  aria-disabled={detailBusy}
                  onKeyDown={(event) => {
                    if (!detailBusy && (event.key === "Enter" || event.key === " ")) {
                      event.preventDefault();
                      setManual(false);
                      setSelectedMotorId(device.motor_id);
                    }
                  }}
                  onClick={() => {
                    if (detailBusy) return;
                    setManual(false);
                    setSelectedMotorId(device.motor_id);
                  }}
                >
                  <div>
                    <Space style={{ justifyContent: "space-between", width: "100%" }}>
                      <Typography.Text strong>DM-J4310-2EC V1.1</Typography.Text>
                      <Typography.Text code>{formatCanId(device.motor_id)}</Typography.Text>
                    </Space>
                    <Space size={4} wrap>
                      <Tag color={device.online ? "green" : "default"}>{device.online ? copy.online : copy.offline}</Tag>
                      {device.attached && <Tag color="blue">{copy.attached}</Tag>}
                      {device.feedback_can_id != null && <Tag>RX {formatCanId(device.feedback_can_id)}</Tag>}
                    </Space>
                  </div>
                </List.Item>
              );
            }}
          />
        )}
        <Button
          block
          className="damiao-device-list__manual"
          type={manual ? "primary" : "default"}
          disabled={detailBusy}
          onClick={() => {
            setManual(true);
            setSelectedMotorId(null);
            setManualKey((value) => value + 1);
          }}
        >
          {copy.manual}
        </Button>
      </aside>
      <section className="damiao-workspace__detail">
        <DamiaoMotorDetail
          key={manual ? `manual-${manualKey}` : `motor-${selectedDevice?.motor_id ?? "default"}`}
          connected={connected}
          initialDevice={manual ? undefined : selectedDevice}
          onDevicesChanged={refreshDevices}
          onBusyChange={setDetailBusy}
        />
      </section>
    </div>
  );
}

function DamiaoMotorDetail({
  connected,
  initialDevice,
  onDevicesChanged,
  onBusyChange,
}: {
  connected: boolean;
  initialDevice?: DamiaoDiscoveredDevice;
  onDevicesChanged: () => Promise<void>;
  onBusyChange: (busy: boolean) => void;
}) {
  const { message } = App.useApp();
  const { lang } = useI18n();
  const zh = lang === "zh";

  const initialMotorId = initialDevice?.motor_id ?? 1;
  const [motorId, setMotorId] = useState(formatCanId(initialMotorId));
  const [masterId, setMasterId] = useState(formatCanId(initialDevice?.feedback_can_id ?? 0));
  const [mode, setMode] = useState<DamiaoMode>("Mit");
  const [pMax, setPMax] = useState(MAPPING_P_MAX_RAD);
  const [vMax, setVMax] = useState(MAPPING_V_MAX_RAD_S);
  const [tMax, setTMax] = useState(MAPPING_T_MAX_NM);
  const [state, setState] = useState<DamiaoState | null>(null);
  const [detachedLocally, setDetachedLocally] = useState(false);
  const [busy, setBusy] = useState(false);
  const busyRef = useRef(false);
  const [bitrateState, setBitrateState] = useState<DamiaoBitrateState | null>(null);
  const [newBitrate, setNewBitrate] = useState(1_000_000);
  const [bitrateError, setBitrateError] = useState<string | null>(null);
  const [bitrateOutcome, setBitrateOutcome] = useState<DamiaoBitrateState | null>(null);
  const [zeroConfirmOpen, setZeroConfirmOpen] = useState(false);
  const [zeroError, setZeroError] = useState<string | null>(null);
  const [zeroOutcome, setZeroOutcome] = useState<DamiaoState | null>(null);
  const [repeat, setRepeat] = useState(true);
  const [rateHz, setRateHz] = useState(50);
  const [view, setView] = useState<"panel" | "chart">("panel");
  const [windowSec, setWindowSec] = useState(10);
  const targetRepeats = repeat && mode !== "PositionVelocity";

  const [mit, setMit] = useState({
    position_rad: 0,
    velocity_rad_s: 0,
    torque_nm: 0,
    kp: 0,
    kd: 0.1,
  });
  const [positionVelocity, setPositionVelocity] = useState({
    position_rad: 0,
    velocity_rad_s: 1,
  });
  const [velocity, setVelocity] = useState(0);

  const copy = useMemo(
    () =>
      zh
        ? {
            attach: "挂载电机",
            detach: "安全失能并卸载",
            connection: "电机与协议配置",
            telemetry: "实时反馈",
            control: "电机控制",
            display: "信息显示",
            numeric: "数值",
            chart: "图表",
            window: "窗口",
            refresh: "刷新",
            refreshLow: "低",
            refreshHigh: "高",
            refreshHint: "界面轮询只读取后端最新快照，不改变 CAN 接收和控制发送速率；图表仅记录 rx_count 增长后的真实新反馈。",
            connected: "已挂载",
            detached: "未挂载",
            online: "反馈在线",
            offline: "无新反馈",
            motorId: "电机 CAN ID",
            masterId: "反馈 Master ID",
            detectedMasterId: "实际反馈 ID",
            physicalMode: "运行时控制模式",
            mapping: "电机映射参数",
            mappingHint: "挂载时自动读取电机的 PMAX / VMAX / TMAX。",
            attachOk: "已挂载 DM-J4310-2EC V1.1（尚未使能）",
            detachOk: "电机已失能并卸载",
            actionOk: "命令已发送",
            modeSwitchOk: "控制模式已切换，原定时目标已停止",
            sendOk: targetRepeats ? "目标已发送并以 50 Hz 持续更新" : "目标已发送一次",
            stopStream: "停止定时发送",
            enable: "使能",
            disable: "失能",
            clearFault: "清故障",
            setZero: "将当前位置设为零点",
            setZeroConfirm: "保存当前位置为电机零点？",
            setZeroDescription: "将停止定时目标并失能电机，随后把当前位置保存为零点。位置坐标将改变，界面运动目标和图表历史会清零。请固定电机，保持轴静止。",
            send: "发送目标",
            periodic: "50 Hz 定时发送",
            positionOneShot: "位置速度目标只发送一次，由驱动器内部完成梯形轨迹",
            position: "控制位置",
            speed: "速度",
            torque: "前馈转矩",
            mosTemp: "MOS 温度",
            rotorTemp: "线圈温度",
            status: "状态",
            feedbackAge: "反馈时延",
            feedbackRate: "反馈频率",
            rxCount: "反馈帧数",
            configureFirst: "请先连接 CAN，再挂载电机。挂载时只发送一次安全失能探测，不会使能或运动。",
            noFeedback:
              "安全失能探测后仍未收到反馈。请确认 24 V 供电、CAN_H/CAN_L、共地、双端 120 Ω、主机与电机的波特率及 CAN / CAN FD 格式一致，以及电机 ID。",
            masterMismatch: "检测到的反馈 ID 与填写的 Master ID 不同；已自动接收，请按检测值更新电机或界面配置。",
            safety:
              "DM-J4310-2EC V1.1 使用 24 V、标准 CAN ID，出厂波特率为 1 Mbps。请使用限流电源并固定电机；空载转矩模式也会迅速加速。挂载后可在下方运行时切换控制模式，切换过程会短暂失能。",
            modeLocked:
              "挂载后也可直接切换。切换会停止原定时目标、先安全失能，再通过 0x7FF 临时写入模式；若切换前已使能，会自动恢复使能。模式不会保存到 Flash，速度模式严格发送 4 字节目标帧。",
            targetLocked: "收到“使能”反馈后才允许发送运动目标。",
            tmaxHint:
              "TMAX 是协议映射范围，不等于安全工作转矩。该型号额定 3 Nm、峰值 7 Nm；界面把前馈转矩限制在 ±7 Nm。",
            vmaxHint:
              "VMAX 是协议映射范围。该型号空载最大转速 200 rpm（约 20.94 rad/s），运动命令不会超过该值。",
            posVelSpeed: "最高速度",
          }
        : {
            attach: "Attach motor",
            detach: "Safe disable & detach",
            connection: "Motor and protocol configuration",
            telemetry: "Live feedback",
            control: "Motor control",
            display: "Display",
            numeric: "Numeric",
            chart: "Chart",
            window: "Window",
            refresh: "Refresh",
            refreshLow: "Low",
            refreshHigh: "High",
            refreshHint: "UI polling reads only the latest backend snapshot and does not change CAN receive or control rates. The chart records only real new feedback when rx_count advances.",
            connected: "Attached",
            detached: "Detached",
            online: "Feedback online",
            offline: "No fresh feedback",
            motorId: "Motor CAN ID",
            masterId: "Feedback Master ID",
            detectedMasterId: "Detected feedback ID",
            physicalMode: "Runtime control mode",
            mapping: "Motor mapping parameters",
            mappingHint: "PMAX / VMAX / TMAX are read from the motor when attaching.",
            attachOk: "DM-J4310-2EC V1.1 attached (not enabled)",
            detachOk: "Motor disabled and detached",
            actionOk: "Command sent",
            modeSwitchOk: "Control mode changed; the previous periodic target was stopped",
            sendOk: targetRepeats ? "Target sent and streaming at 50 Hz" : "Target sent once",
            stopStream: "Stop periodic send",
            enable: "Enable",
            disable: "Disable",
            clearFault: "Clear fault",
            setZero: "Set current position as zero",
            setZeroConfirm: "Save the current position as the motor zero?",
            setZeroDescription: "Periodic targets will stop and the motor will be disabled before its current position is saved as zero. This changes position coordinates and clears local targets and chart history. Secure the motor and keep the shaft still.",
            send: "Send target",
            periodic: "Periodic send at 50 Hz",
            positionOneShot: "Position-velocity goals are sent once; the drive completes the internal trapezoidal trajectory",
            position: "Control position",
            speed: "Velocity",
            torque: "Feed-forward torque",
            mosTemp: "MOS temperature",
            rotorTemp: "Winding temperature",
            status: "Status",
            feedbackAge: "Feedback age",
            feedbackRate: "Feedback rate",
            rxCount: "Feedback frames",
            configureFirst: "Connect CAN, then attach the motor. Attach sends one safe disable probe; it never enables motion.",
            noFeedback:
              "No feedback after the safe disable probe. Check 24 V power, CAN_H/CAN_L, common ground, dual 120 Ω termination, matching host/motor bitrate and CAN / CAN FD format, and the motor ID.",
            masterMismatch: "The detected feedback ID differs from the configured Master ID. It is being received automatically; update the drive or this field to match.",
            safety:
              "DM-J4310-2EC V1.1 uses 24 V, standard CAN IDs, and a factory bitrate of 1 Mbps. Use a current-limited supply and secure the motor; unloaded torque mode can accelerate rapidly. The mode can be changed after attaching; switching briefly disables the drive.",
            modeLocked:
              "You can switch after attaching. The transaction stops the previous periodic target, safely disables the motor, and writes the volatile mode through 0x7FF. If it was enabled, enable is restored automatically. Velocity targets use an exact 4-byte frame.",
            targetLocked: "Motion targets unlock after an Enabled feedback frame is received.",
            tmaxHint:
              "TMAX is a protocol mapping range, not a safe torque. This motor is rated 3 Nm and peaks at 7 Nm; feed-forward torque is limited to ±7 Nm here.",
            vmaxHint:
              "VMAX is a protocol mapping range. The motor's no-load maximum is 200 rpm (about 20.94 rad/s), which this panel enforces.",
            posVelSpeed: "Maximum speed",
          },
    [targetRepeats, zh],
  );

  const attached = connected && !detachedLocally && (state ? state.attached : !!initialDevice?.attached);
  const attachedMotorId = attached ? state?.motor_id ?? initialMotorId : null;
  const { latest, samples, chartVersion, resetHistory } = useDamiaoTelemetry(
    attachedMotorId,
    connected,
    rateHz,
    view === "chart",
  );
  const faulted = state != null && state.status_code !== 0 && state.status_code !== 1;
  const requiresReconnect = !!bitrateOutcome?.requires_reconnect || !!state?.requires_reconnect;
  const controlsReady = connected && attachedMotorId != null && !requiresReconnect;
  const motionReady = controlsReady && !!state?.online && !!state?.enabled && !faulted;
  const zeroReady = controlsReady && !!state?.online && state?.position_rad != null && !faulted && !busy;
  const masterMismatch = state?.feedback_can_id != null && state.feedback_can_id !== state.master_id;
  const commandSpeedMax = Math.min(vMax, PHYSICAL_MAX_SPEED_RAD_S);
  const commandTorqueMax = Math.min(tMax, PHYSICAL_PEAK_TORQUE_NM);
  const singleTurnAngle = state?.output_position_rad != null
    ? (((state.output_position_rad + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI) - Math.PI
    : null;

  useEffect(() => {
    if (requiresReconnect) setBitrateState(null);
  }, [requiresReconnect]);

  useEffect(() => {
    if (!zeroReady) setZeroConfirmOpen(false);
  }, [zeroReady]);

  useEffect(() => {
    if (!connected) {
      setState(null);
      setBitrateState(null);
      setBitrateError(null);
      setBitrateOutcome(null);
      setZeroOutcome(null);
      setZeroError(null);
      return;
    }
    if (latest == null || !attached) return;
    setState(latest);
    setMotorId(formatCanId(latest.motor_id));
    setMasterId(formatCanId(latest.master_id));
    setMode(latest.mode);
    setPMax(latest.p_max);
    setVMax(latest.v_max);
    setTMax(latest.t_max);
  }, [attached, connected, latest]);

  const run = useCallback(
    async (success: string, action: () => Promise<unknown>) => {
      if (!connected || busyRef.current) return;
      busyRef.current = true;
      setBusy(true);
      onBusyChange(true);
      try {
        await action();
        if (success) message.success(success);
      } catch (error) {
        message.error(errMsg(error));
        throw error;
      } finally {
        busyRef.current = false;
        setBusy(false);
        onBusyChange(false);
      }
    },
    [connected, message, onBusyChange],
  );

  const attach = async () => {
    let config: DamiaoConfig;
    try {
      config = {
        motor_id: parseCanId(motorId, 0xff, copy.motorId),
        master_id: parseCanId(masterId, 0x7ff, copy.masterId),
        mode,
        p_max: pMax,
        v_max: vMax,
        t_max: tMax,
      };
    } catch (error) {
      message.error(errMsg(error));
      return;
    }
    try {
      await run(copy.attachOk, async () => {
        setState(await api.damiaoAttach(config));
        setDetachedLocally(false);
        await onDevicesChanged().catch(() => {});
      });
    } catch {
      // `run` already presented the backend error.
    }
  };

  const detach = async () => {
    if (attachedMotorId == null) return;
    try {
      await run(copy.detachOk, async () => {
        await api.damiaoDetach(attachedMotorId);
        setDetachedLocally(true);
        setState(null);
        setBitrateState(null);
        setBitrateOutcome(null);
        setBitrateError(null);
        setZeroOutcome(null);
        setZeroError(null);
        await onDevicesChanged().catch(() => {});
      });
    } catch {
      // Keep the attached state visible so a failed safe-disable is retryable.
    }
  };

  const changeMode = async (nextMode: DamiaoMode) => {
    if (nextMode === mode) return;
    if (attachedMotorId == null) {
      setMode(nextMode);
      return;
    }

    const previousMode = mode;
    try {
      await run(copy.modeSwitchOk, async () => {
        const next = await api.damiaoSetMode(attachedMotorId, nextMode);
        setState(next);
        setMode(next.mode);
      });
    } catch {
      // The backend commits the new encoder/ID state as soon as the mode
      // write succeeds, even if restoring enable later fails. Re-read it so
      // the UI never sends a target using a stale local mode.
      try {
        const latest = await api.damiaoGetState(attachedMotorId);
        setState(latest);
        setMode(latest.mode);
      } catch {
        setMode(previousMode);
      }
    }
  };

  const action = async (fn: () => Promise<unknown>, success = copy.actionOk) => {
    try {
      await run(success, fn);
    } catch {
      // Error was surfaced by `run`.
    }
  };

  const readBitrate = async () => {
    if (attachedMotorId == null || requiresReconnect) return;
    try {
      await run("", async () => {
        const next = await api.damiaoGetBitrate(attachedMotorId);
        setBitrateState(next);
        setNewBitrate(next.bitrate);
        setBitrateError(null);
        setBitrateOutcome(null);
      });
    } catch (error) {
      setBitrateState(null);
      setBitrateError(errMsg(error));
    }
  };

  const changeBitrate = async () => {
    if (attachedMotorId == null || !bitrateState || requiresReconnect) return;
    try {
      await run("", async () => {
        const next = await api.damiaoSetBitrate(attachedMotorId, newBitrate);
        setBitrateOutcome(next);
        setBitrateState(next.requires_reconnect ? null : next);
        setBitrateError(null);
        if (next.requires_reconnect) {
          message.warning(zh ? "请将主机改为目标波特率后重新连接，再读取确认并保存参数" : "Reconnect the host at the target bitrate, then read to verify and save parameters");
        } else {
          message.success(zh ? "已确认电机正在使用该波特率，无需修改" : "The motor already uses this bitrate; no change was needed");
        }
      });
    } catch (error) {
      setBitrateState(null);
      setBitrateError(errMsg(error));
    }
  };

  const setZero = async () => {
    if (attachedMotorId == null || !zeroReady) return;
    setZeroConfirmOpen(false);
    try {
      await run("", async () => {
        setZeroError(null);
        setZeroOutcome(null);
        // Even a missing acknowledgement can follow a real zero change.
        // Clear old coordinates before the transaction and never send them.
        setMit({ position_rad: 0, velocity_rad_s: 0, torque_nm: 0, kp: 0, kd: 0 });
        setPositionVelocity({ position_rad: 0, velocity_rad_s: 0 });
        setVelocity(0);
        setRepeat(false);
        resetHistory();
        const next = await api.damiaoSetZero(attachedMotorId);
        setState(next);
        setZeroOutcome(next);
        resetHistory(next);
        message.success(zh ? "零点已保存，已收到失能且接近零位的反馈" : "Zero saved; disabled feedback confirmed near zero");
      });
    } catch (error) {
      setZeroError(errMsg(error));
      resetHistory();
    }
  };

  const zeroPanel = (
    <Card
      className="damiao-zero-card"
      size="small"
      title={zh ? "位置零点" : "Position zero"}
      extra={busy ? <Tag color="processing">{zh ? "命令执行中" : "Working"}</Tag> : zeroOutcome ? <Tag color="success">{zh ? "反馈已验证" : "Feedback verified"}</Tag> : undefined}
    >
      <div className="damiao-zero-settings">
        <div className="damiao-zero-current">
          <Typography.Text type="secondary">{zh ? "当前控制位置" : "Current control position"}</Typography.Text>
          <Typography.Text strong><TelemetryNumber value={state?.position_rad} suffix=" rad" /></Typography.Text>
          <Typography.Text type="secondary">
            {busy
              ? zh ? "等待当前命令完成" : "Wait for the current command to finish"
              : zeroReady
              ? zh ? "保持轴静止后设定零点" : "Keep the shaft still before setting zero"
              : zh ? "需挂载、反馈在线、位置已更新且无故障" : "Requires an attached motor with fresh position and fault-free feedback"}
          </Typography.Text>
        </div>
        <Popconfirm
          title={copy.setZeroConfirm}
          description={<>
            <Typography.Paragraph style={{ marginBottom: 8 }}>
              {zh ? "电机：" : "Motor: "}{formatCanId(attachedMotorId ?? initialMotorId)}
              {zh ? "；当前控制位置：" : "; current control position: "}
              <TelemetryNumber value={state?.position_rad} suffix=" rad" />
            </Typography.Paragraph>
            {copy.setZeroDescription}
          </>}
          overlayClassName="damiao-zero-confirm"
          open={zeroConfirmOpen}
          onOpenChange={(open) => setZeroConfirmOpen(open && zeroReady)}
          disabled={!zeroReady}
          okText={zh ? "失能并设置零点" : "Disable & set zero"}
          cancelText={zh ? "取消" : "Cancel"}
          onConfirm={() => void setZero()}
        >
          <Button disabled={!zeroReady}>{copy.setZero}</Button>
        </Popconfirm>
      </div>
      <Typography.Paragraph className="damiao-zero-hint" type="secondary">
        {zh
          ? "操作会停止定时目标、失能电机并保存当前位置为零点，之后查询新反馈确认。完成后保持失能；位置、速度、转矩和增益目标会清零，定时发送关闭，图表从新零点开始记录。"
          : "Stops periodic targets, disables the motor, saves the current position as zero, and verifies fresh feedback. The motor stays disabled. Position, velocity, torque, and gain targets are cleared, periodic send is turned off, and the chart starts a new reference."}
      </Typography.Paragraph>
      {zeroOutcome && (
        <Alert
          type="success"
          showIcon
          message={zh ? "零点已保存并验证，电机保持失能" : "Zero saved and verified; motor remains disabled"}
          description={<Space wrap size={4}>
            <Typography.Text>{zh ? "设置后反馈位置：" : "Verified position after setting: "}</Typography.Text>
            <TelemetryNumber value={zeroOutcome.position_rad} suffix=" rad" />
            <Typography.Text type="secondary">{zh ? "重新设置目标后再使能和发送。" : "Configure new targets before enabling and sending."}</Typography.Text>
          </Space>}
        />
      )}
      {zeroError && (
        <Alert
          type="warning"
          showIcon
          message={zh ? "零点尚未确认" : "Zero not verified"}
          description={`${zeroError} ${zh ? "界面旧目标已清除。请查看错误原因和最新反馈，保持轴静止后再操作。" : "Previous local targets were cleared. Check the error and fresh feedback before retrying with the shaft still."}`}
        />
      )}
    </Card>
  );

  const bitratePanel = (
    <Card
      className="damiao-bitrate-card"
      size="small"
      title={zh ? "CAN 波特率" : "CAN bitrate"}
      extra={busy ? <Tag color="processing">{zh ? "命令执行中" : "Working"}</Tag> : <Tag>{bitrateState ? bitrateState.can_fd ? "CAN FD" : "CAN 2.0" : zh ? "未读取" : "Not read"}</Tag>}
    >
      <div className="damiao-bitrate-settings">
        <div className="damiao-bitrate-current">
          <Typography.Text type="secondary">{zh ? "电机当前值" : "Current motor setting"}</Typography.Text>
          <Typography.Text strong className="damiao-tabular">
            {bitrateState ? formatBitrate(bitrateState.bitrate) : "—"}
          </Typography.Text>
          <Typography.Text type="secondary">
            {requiresReconnect
              ? zh ? "主机需按目标波特率重新连接后再读取" : "Reconnect the host at the target bitrate before reading"
              : bitrateState
              ? `${zh ? "已读取" : "Read"} · ${zh ? "代码" : "code"} ${bitrateState.register_code}`
              : zh ? "挂载后读取电机，再选择新速率" : "Attach and read the motor before choosing a new rate"}
          </Typography.Text>
        </div>
        <Space wrap align="end" className="damiao-bitrate-actions">
          <Button disabled={!controlsReady || busy} onClick={() => void readBitrate()}>
            {zh ? "读取波特率" : "Read bitrate"}
          </Button>
          <Field label={zh ? "目标波特率" : "Target bitrate"}>
            <Select
              value={newBitrate}
              disabled={!controlsReady || busy || !bitrateState}
              onChange={setNewBitrate}
              options={BITRATE_OPTIONS.map((bitrate) => ({ value: bitrate, label: `${formatBitrate(bitrate)} · ${bitrate > 1_000_000 ? "CAN FD" : "CAN 2.0"}` }))}
            />
          </Field>
          <Button type="primary" disabled={!controlsReady || busy || !bitrateState || newBitrate === bitrateState.bitrate} onClick={() => void changeBitrate()}>
            {zh ? "失能并切换波特率" : "Disable & change bitrate"}
          </Button>
          <Button disabled={!controlsReady || busy || !bitrateState} onClick={() => attachedMotorId != null && action(() => api.damiaoSaveParameters(attachedMotorId), zh ? "电机已失能，当前全部参数已保存到 Flash" : "Motor disabled; all current parameters saved to Flash")}>
            {zh ? "失能并保存全部参数" : "Disable & save all parameters"}
          </Button>
        </Space>
      </div>
      <Typography.Paragraph className="damiao-bitrate-hint" type="secondary">
        {zh
          ? "切换会停止定时目标并失能电机。≤ 1 Mbps 使用 CAN 2.0，> 1 Mbps 切换为 CAN FD 反馈。写入后需让主机使用匹配的速率重新连接。"
          : "Changing bitrate stops periodic targets and disables the motor. ≤ 1 Mbps uses CAN 2.0; > 1 Mbps switches feedback to CAN FD. Reconnect the host at the matching rate after writing."}
      </Typography.Paragraph>
      <Typography.Paragraph className="damiao-bitrate-hint" type="secondary">
        {zh
          ? "波特率写入后先在新速率下读取确认，再保存以便断电保留。保存会将当前全部参数（包含运行时控制模式）写入 Flash。手册建议在装机前单独配置波特率；本应用仅在目标电机单独连接时允许切换。"
          : "Read and verify at the new rate before saving for persistence. Save writes all current parameters, including the runtime control mode, to Flash. The manual recommends configuring bitrate before installation; this app allows changing only when the target motor is connected on its own."}
      </Typography.Paragraph>
      {bitrateError && <Alert type="warning" showIcon message={zh ? "波特率尚未确认" : "Bitrate not verified"} description={bitrateError} />}
      {bitrateOutcome?.requires_reconnect && (
        <Alert
          type="warning"
          showIcon
          message={bitrateOutcome.acknowledged
            ? zh ? `电机已应答切换到 ${formatBitrate(bitrateOutcome.bitrate)}，请重新连接主机` : `Motor acknowledged ${formatBitrate(bitrateOutcome.bitrate)}; reconnect the host`
            : zh ? "未收到可验证的切换应答，电机实际速率尚未确认" : "Bitrate change was not confirmed; actual motor bitrate is uncertain"}
          description={<Space direction="vertical" size={4}>
            <Typography.Text>
              {bitrateOutcome.can_fd
                ? zh ? `主机需使用 CAN FD，常用配置为仲裁速率 1 Mbps、数据速率 ${formatBitrate(bitrateOutcome.bitrate)}；连接后以读取结果确认。` : `Use CAN FD; the typical host setting is nominal 1 Mbps, data ${formatBitrate(bitrateOutcome.bitrate)}. Read back after connecting to verify.`
                : zh ? `主机 CAN 波特率需设为 ${formatBitrate(bitrateOutcome.bitrate)}。` : `Set the host CAN bitrate to ${formatBitrate(bitrateOutcome.bitrate)}.`}
            </Typography.Text>
            <Typography.Text>
              {zh ? "在顶部断开后，用匹配速率重新连接；重新挂载、读取确认后再保存参数。SocketCAN 的速率需在系统中配置。切换期间运动控制保持锁定。" : "Disconnect above, reconnect at the matching rate, then attach, read to verify, and save parameters. Configure SocketCAN timing in the system. Motion controls remain locked during the change."}
            </Typography.Text>
          </Space>}
        />
      )}
    </Card>
  );

  const target = (): DamiaoTarget => {
    switch (mode) {
      case "Mit":
        return { kind: "Mit", ...mit };
      case "PositionVelocity":
        return { kind: "PositionVelocity", ...positionVelocity };
      case "Velocity":
        return { kind: "Velocity", velocity_rad_s: velocity };
    }
  };

  const statusTag = state?.online ? (
    <Tag color={faulted ? "red" : state.enabled ? "green" : "blue"}>
      {statusLabel(state.status_code, zh)} · 0x{state.status_code.toString(16).toUpperCase()}
    </Tag>
  ) : (
    <Tag>{copy.offline}</Tag>
  );

  const connectionPanel = (
    <Space direction="vertical" size={12} style={{ width: "100%" }}>
      <CollapsibleWarning zh={zh} message={copy.safety} />
      {!connected && <Alert type="info" showIcon message={copy.configureFirst} />}
      <div className="damiao-config-grid">
        <Field label={copy.motorId}>
          <Input value={motorId} disabled={attached || busy} onChange={(event) => setMotorId(event.target.value)} />
        </Field>
        <Field label={copy.masterId}>
          <Input value={masterId} disabled={attached || busy} onChange={(event) => setMasterId(event.target.value)} />
        </Field>
      </div>
      <Card size="small" title={copy.mapping} extra={<Typography.Text type="secondary">{copy.mappingHint}</Typography.Text>}>
        <Space wrap>
          <Field label="PMAX (rad)">
            <InputNumber value={attached ? pMax : null} placeholder={zh ? "挂载后读取" : "Read on attach"} disabled />
          </Field>
          <Tooltip title={copy.vmaxHint}>
            <span>
              <Field label="VMAX (rad/s)">
              <InputNumber value={attached ? vMax : null} placeholder={zh ? "挂载后读取" : "Read on attach"} disabled />
              </Field>
            </span>
          </Tooltip>
          <Tooltip title={copy.tmaxHint}>
            <span>
              <Field label="TMAX (Nm)">
              <InputNumber value={attached ? tMax : null} placeholder={zh ? "挂载后读取" : "Read on attach"} disabled />
              </Field>
            </span>
          </Tooltip>
        </Space>
      </Card>
      <Space wrap>
        {!attached ? (
          <Button type="primary" loading={busy} disabled={!connected} onClick={attach}>
            {copy.attach}
          </Button>
        ) : (
          <Button danger loading={busy} disabled={!connected || requiresReconnect} onClick={detach}>
            {copy.detach}
          </Button>
        )}
        <Tag color={attached ? "green" : "default"}>{attached ? copy.connected : copy.detached}</Tag>
        {attached && statusTag}
      </Space>
    </Space>
  );

  const feedbackAlerts = (
    <Space direction="vertical" size={12} style={{ width: "100%" }}>
      {attached && state && !state.online && (
        <CollapsibleWarning zh={zh} message={copy.noFeedback} />
      )}
      {state && !state.online && state.rx_count > 0 && <Typography.Text type="secondary">{zh ? "当前显示最后一次反馈；收到新反馈后才允许发送运动目标。" : "Showing the last feedback. Motion targets unlock when fresh feedback resumes."}</Typography.Text>}
      {masterMismatch && state && (
        <CollapsibleWarning
          zh={zh}
          message={`${copy.masterMismatch} 0x${state.master_id.toString(16).toUpperCase()} → 0x${state.feedback_can_id!.toString(16).toUpperCase()}`}
        />
      )}
      {state?.last_error && <Alert type="error" showIcon message={state.last_error} />}
      {faulted && state && (
        <Alert type="error" showIcon message={`${copy.status}: ${statusLabel(state.status_code, zh)} (0x${state.status_code.toString(16).toUpperCase()})`} />
      )}
    </Space>
  );

  const telemetryPanel = (
    <Descriptions
      className="damiao-telemetry-table"
      bordered
      size="small"
      column={{ xs: 1, sm: 2, md: 2, lg: 2, xl: 2, xxl: 2 }}
    >
      <Descriptions.Item label={zh ? "输出轴单圈角度" : "Output-shaft single-turn angle"}>
        <Space wrap size={8}>
          <TelemetryNumber value={singleTurnAngle} suffix=" rad" />
          <Typography.Text type="secondary">
            (<TelemetryNumber value={singleTurnAngle != null ? singleTurnAngle * 180 / Math.PI : null} digits={2} suffix="°" />)
          </Typography.Text>
        </Space>
      </Descriptions.Item>
      <Descriptions.Item label={zh ? "输出轴累计位置 (rad)" : "Output-shaft cumulative position (rad)"}>
        <TelemetryNumber value={state?.output_position_rad} />
      </Descriptions.Item>
      <Descriptions.Item label={zh ? "控制位置 (rad)" : "Control position (rad)"}>
        <TelemetryNumber value={state?.position_rad} />
      </Descriptions.Item>
      <Descriptions.Item label={zh ? "编码器角度更新" : "Encoder angle age"}>
        {state?.position_read_age_ms != null ? <Tag color={state.output_position_rad != null ? "success" : "warning"}>{state.position_read_age_ms} ms</Tag> : "—"}
      </Descriptions.Item>
      <Descriptions.Item label={`${copy.speed} (rad/s)`}><TelemetryNumber value={state?.velocity_rad_s} /></Descriptions.Item>
      <Descriptions.Item label={`${copy.torque} (Nm)`}><TelemetryNumber value={state?.torque_nm} digits={3} /></Descriptions.Item>
      <Descriptions.Item label={copy.status}>{state ? statusTag : "—"}</Descriptions.Item>
      <Descriptions.Item label={`${copy.mosTemp} (°C)`}><TelemetryNumber value={state?.mos_temp_c} digits={1} /></Descriptions.Item>
      <Descriptions.Item label={`${copy.rotorTemp} (°C)`}><TelemetryNumber value={state?.rotor_temp_c} digits={1} /></Descriptions.Item>
      <Descriptions.Item label={copy.feedbackRate}>
        {state?.feedback_rate_hz != null ? <TelemetryNumber value={state.feedback_rate_hz} digits={1} suffix=" Hz" /> : "—"}
      </Descriptions.Item>
      <Descriptions.Item label={copy.feedbackAge}>
        {state?.feedback_age_ms != null ? (
          <Tag color={feedbackAgeColor(state)}>{state.feedback_age_ms} ms</Tag>
        ) : "—"}
      </Descriptions.Item>
      <Descriptions.Item label={copy.rxCount}>
        <span className="damiao-tabular">{state?.rx_count ?? 0}</span>
      </Descriptions.Item>
      <Descriptions.Item label={copy.physicalMode}>{modeLabel(state?.mode ?? mode, zh)}</Descriptions.Item>
      <Descriptions.Item label={copy.detectedMasterId}>
        {state?.feedback_can_id != null ? formatCanId(state.feedback_can_id) : "—"}
      </Descriptions.Item>
      <Descriptions.Item label="CAN">
        {state
          ? `Mode 0x7FF · Enable ${formatCanId(state.motor_id)} · Target ${formatCanId(controlCanId(state.motor_id, state.mode))} / DLC ${state.mode === "Velocity" ? 4 : 8}`
          : "—"}
      </Descriptions.Item>
    </Descriptions>
  );

  const controlPanel = (
    <Card
      size="small"
      title={copy.control}
      extra={busy ? <Tag color="processing">{zh ? "命令执行中" : "Working"}</Tag> : state?.streaming ? <Tag color="processing">50 Hz</Tag> : undefined}
    >
      <Space wrap align="center">
        <Typography.Text type="secondary">{copy.physicalMode}</Typography.Text>
        <Segmented
          value={mode}
          disabled={busy || requiresReconnect}
          onChange={(value) => void changeMode(value as DamiaoMode)}
          options={MODE_OPTIONS.map((value) => ({ label: modeLabel(value, zh), value }))}
        />
        <Button type="primary" disabled={!controlsReady || busy || !state?.online || !!state?.enabled || faulted} onClick={() => attachedMotorId != null && action(() => api.damiaoEnable(attachedMotorId))}>
          {copy.enable}
        </Button>
        <Button danger disabled={!controlsReady || busy} onClick={() => attachedMotorId != null && action(() => api.damiaoDisable(attachedMotorId))}>
          {copy.disable}
        </Button>
        <Button disabled={!controlsReady || busy} onClick={() => attachedMotorId != null && action(() => api.damiaoClearFault(attachedMotorId))}>
          {copy.clearFault}
        </Button>
      </Space>
      <Typography.Text className="damiao-mode-hint" type="secondary">{copy.modeLocked}</Typography.Text>

      <Divider style={{ margin: "12px 0" }} />
      <div className="damiao-target-editor">
        <Typography.Text strong>{modeLabel(mode, zh)}</Typography.Text>
        <Space wrap align="end">
          {mode === "Mit" && (
            <>
              <NumberField disabled={busy} label={`${copy.position} (rad)`} value={mit.position_rad} min={-pMax} max={pMax} step={0.01} onChange={(value) => setMit((current) => ({ ...current, position_rad: value }))} />
              <NumberField disabled={busy} label={`${copy.speed} (rad/s)`} value={mit.velocity_rad_s} min={-commandSpeedMax} max={commandSpeedMax} step={0.1} onChange={(value) => setMit((current) => ({ ...current, velocity_rad_s: value }))} />
              <Tooltip title={copy.tmaxHint}>
                <span>
                  <NumberField disabled={busy} label={`${copy.torque} (Nm)`} value={mit.torque_nm} min={-commandTorqueMax} max={commandTorqueMax} step={0.05} onChange={(value) => setMit((current) => ({ ...current, torque_nm: value }))} />
                </span>
              </Tooltip>
              <NumberField disabled={busy} label="Kp (Nm/rad)" value={mit.kp} min={0} max={500} step={0.1} onChange={(value) => setMit((current) => ({ ...current, kp: value }))} />
              <NumberField disabled={busy} label="Kd (Nm·s/rad)" value={mit.kd} min={0} max={5} step={0.01} onChange={(value) => setMit((current) => ({ ...current, kd: value }))} />
            </>
          )}
          {mode === "PositionVelocity" && (
            <>
              <NumberField disabled={busy} label={`${copy.position} (rad)`} value={positionVelocity.position_rad} step={0.01} onChange={(value) => setPositionVelocity((current) => ({ ...current, position_rad: value }))} />
              <NumberField disabled={busy} label={`${copy.posVelSpeed} (rad/s)`} value={positionVelocity.velocity_rad_s} min={0} max={commandSpeedMax} step={0.1} onChange={(value) => setPositionVelocity((current) => ({ ...current, velocity_rad_s: value }))} />
            </>
          )}
          {mode === "Velocity" && (
            <NumberField disabled={busy} label={`${copy.speed} (rad/s)`} value={velocity} min={-commandSpeedMax} max={commandSpeedMax} step={0.1} onChange={setVelocity} />
          )}
        </Space>
      </div>

      {!motionReady && <Alert type="info" showIcon message={requiresReconnect ? zh ? "波特率已切换，请先重新连接 CAN 并读取确认。" : "Bitrate changed; reconnect CAN and read to verify first." : copy.targetLocked} />}
      <Space wrap className="damiao-target-actions">
        <Button type="primary" disabled={!motionReady || busy} onClick={() => attachedMotorId != null && action(() => api.damiaoSendTarget(attachedMotorId, target(), targetRepeats), copy.sendOk)}>
          {copy.send}
        </Button>
        <Space size={6}>
          <Switch checked={targetRepeats} disabled={busy || mode === "PositionVelocity"} onChange={setRepeat} />
          <Typography.Text>{mode === "PositionVelocity" ? copy.positionOneShot : copy.periodic}</Typography.Text>
        </Space>
        <Button disabled={!controlsReady || busy || !state?.streaming} onClick={() => attachedMotorId != null && action(() => api.damiaoStopStream(attachedMotorId))}>
          {copy.stopStream}
        </Button>
        {state?.streaming && <Tag color="processing">50 Hz</Tag>}
      </Space>
    </Card>
  );

  return (
    <div className="damiao-panel">
      <Space direction="vertical" size={12} style={{ width: "100%" }}>
        <Card size="small" className="damiao-summary-card">
          <div className="damiao-summary-card__content">
            <Space direction="vertical" size={3}>
              <Space align="center" wrap>
                <Typography.Title level={4} style={{ margin: 0 }}>DM-J4310-2EC V1.1</Typography.Title>
                <Typography.Text code>{formatCanId(state?.motor_id ?? initialMotorId)}</Typography.Text>
                <Tag color={attached ? "blue" : "default"}>{attached ? copy.connected : copy.detached}</Tag>
                {attached && statusTag}
                <Tag>{modeLabel(state?.mode ?? mode, zh)}</Tag>
              </Space>
              <Typography.Text type="secondary" className="damiao-summary-card__meta">
                {state
                  ? `RX ${state.feedback_can_id != null ? formatCanId(state.feedback_can_id) : "—"} · Target ${formatCanId(controlCanId(state.motor_id, state.mode))} / DLC ${state.mode === "Velocity" ? 4 : 8} · ${state.feedback_rate_hz != null ? `${state.feedback_rate_hz.toFixed(1)} Hz` : "— Hz"}`
                  : `24 V · CAN STD · ${modeLabel(mode, zh)}`}
              </Typography.Text>
            </Space>
            <Space size={5}>
              <Typography.Text type="secondary">{copy.refresh}</Typography.Text>
              <Tooltip title={copy.refreshHint}><Typography.Text type="secondary"><QuestionCircleOutlined /></Typography.Text></Tooltip>
              <Segmented
                size="small"
                value={rateHz}
                onChange={(value) => setRateHz(value as number)}
                options={[
                  { label: copy.refreshLow, value: 50 },
                  { label: copy.refreshHigh, value: 100 },
                ]}
              />
            </Space>
          </div>
        </Card>

      <Collapse
        defaultActiveKey={attached ? [] : ["connection"]}
        items={[
          { key: "connection", label: copy.connection, extra: attached ? <Tag color="green">{copy.connected}</Tag> : undefined, children: connectionPanel },
        ]}
      />

        {zeroPanel}

        {bitratePanel}

        <Card
          size="small"
          title={copy.display}
          extra={
            <Space>
              {view === "chart" && (
                <Space size={4}>
                  <Typography.Text type="secondary">{copy.window}</Typography.Text>
                  <InputNumber
                    size="small"
                    min={1}
                    max={60}
                    value={windowSec}
                    onChange={(value) => setWindowSec(value ?? 10)}
                    style={{ width: 70 }}
                  />
                  <Typography.Text type="secondary">s</Typography.Text>
                </Space>
              )}
              <Segmented
                value={view}
                onChange={(value) => setView(value as "panel" | "chart")}
                options={[
                  { label: copy.numeric, value: "panel" },
                  { label: copy.chart, value: "chart" },
                ]}
              />
            </Space>
          }
        >
          <Space direction="vertical" size={12} style={{ width: "100%" }}>
            {feedbackAlerts}
            <Typography.Text type="secondary">
              {zh
                ? "单圈角度显示为 −180～180°（−π～π rad），以当前零点为参考；累计位置保留转动圈数。运动目标使用控制位置坐标。"
                : "Single-turn angle is shown from −180 to 180° (−π to π rad) relative to the current zero. Cumulative positions retain turns; motion targets use the control coordinate."}
            </Typography.Text>
            {state?.position_read_error && (
              <Alert type="warning" showIcon message={zh ? "输出轴角度尚未更新" : "Output-shaft angle not updated"} description={state.position_read_error} />
            )}
            {view === "panel" ? telemetryPanel : (
              <DamiaoLiveChart samples={samples} chartVersion={chartVersion} windowSec={windowSec} zh={zh} />
            )}
          </Space>
        </Card>

        {controlPanel}
      </Space>
    </div>
  );
}

function CollapsibleWarning({
  message,
  description,
  zh,
}: {
  message: ReactNode;
  description?: ReactNode;
  zh: boolean;
}) {
  return (
    <Collapse
      size="small"
      items={[{
        key: "warning",
        label: <Typography.Text type="warning">{zh ? "注意" : "Caution"}</Typography.Text>,
        children: <Alert type="warning" showIcon message={message} description={description} />,
      }]}
    />
  );
}

function Field({ label, children, className }: { label: ReactNode; children: ReactNode; className?: string }) {
  return (
    <label className={`damiao-field${className ? ` ${className}` : ""}`}>
      <Typography.Text type="secondary">{label}</Typography.Text>
      {children}
    </label>
  );
}

function NumberField({ label, value, onChange, ...props }: { label: ReactNode; value: number; onChange: (value: number) => void; min?: number; max?: number; step?: number; disabled?: boolean }) {
  return (
    <Field label={label}>
      <InputNumber value={value} onChange={(next) => onChange(next ?? 0)} {...props} />
    </Field>
  );
}

function parseCanId(raw: string, max: number, label: string): number {
  const text = raw.trim();
  if (!/^(?:0x[0-9a-f]+|\d+)$/i.test(text)) throw new Error(`${label}: ${raw}`);
  const value = Number(text);
  if (!Number.isInteger(value) || value < 0 || value > max) {
    throw new Error(`${label}: 0x000–0x${max.toString(16).toUpperCase()}`);
  }
  return value;
}

function modeLabel(mode: DamiaoMode, zh: boolean): string {
  switch (mode) {
    case "Mit":
      return "MIT";
    case "PositionVelocity":
      return zh ? "位置速度" : "Position + velocity";
    case "Velocity":
      return zh ? "速度" : "Velocity";
  }
}

function statusLabel(code: number, zh: boolean): string {
  const labels: Record<number, [string, string]> = {
    0x0: ["Disabled", "失能"],
    0x1: ["Enabled", "使能"],
    0x3: ["Output-shaft calibration fault", "输出轴标定异常"],
    0x4: ["Output sensor fault", "输出传感器异常"],
    0x5: ["Motor encoder calibration fault", "电机编码器标定异常"],
    0x8: ["Over-voltage", "过压"],
    0x9: ["Under-voltage", "欠压"],
    0xa: ["Over-current", "过流"],
    0xb: ["MOS over-temperature", "MOS 过温"],
    0xc: ["Motor over-temperature", "线圈过温"],
    0xd: ["Communication lost", "通讯丢失"],
    0xe: ["Overload", "过载"],
  };
  const pair = labels[code] ?? ["Unknown", "未知"];
  return zh ? pair[1] : pair[0];
}

function TelemetryNumber({
  value,
  digits = 4,
  suffix = "",
}: {
  value: number | null | undefined;
  digits?: number;
  suffix?: string;
}) {
  if (value == null || !Number.isFinite(value)) return <span>—</span>;
  const rounded = Number(value.toFixed(digits));
  const sign = rounded < 0 ? "-" : " ";
  return (
    <span className="damiao-tabular">
      {sign}{Math.abs(rounded).toFixed(digits)}{suffix}
    </span>
  );
}

function feedbackAgeColor(state: DamiaoState): string {
  if (!state.online) return "default";
  const age = state.feedback_age_ms ?? Number.POSITIVE_INFINITY;
  if (age <= 100) return "green";
  if (age <= 300) return "gold";
  return "orange";
}

function formatCanId(value: number): string {
  return `0x${value.toString(16).toUpperCase().padStart(2, "0")}`;
}

function formatBitrate(value: number): string {
  return value >= 1_000_000 ? `${value / 1_000_000} Mbps` : `${value / 1_000} kbps`;
}

function controlCanId(motorId: number, mode: DamiaoMode): number {
  return motorId + (mode === "Mit" ? 0 : mode === "PositionVelocity" ? 0x100 : 0x200);
}
