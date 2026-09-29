import { useCallback, useEffect, useRef, useState } from "react";
import {
  Alert, App, Button, Card, Descriptions, Input, InputNumber, Modal,
  Select, Space, Tag, Typography,
} from "antd";
import { api, errMsg } from "../api";
import { isGsUsbSpec } from "../canProfile";
import { useI18n } from "../i18n";
import type { HostCanStatus, LingkongParameterValue, LingkongProbe, LingkongSnapshot } from "../types";
import { lingkongParameters, type LingkongParameterDef } from "../lingkongParameters";
import "./LingkongMotorPanel.css";

const ERROR_BITS = [
  ["欠压", "Undervoltage"], ["过压", "Overvoltage"],
  ["驱动过温", "Driver overtemperature"], ["电机过温", "Motor overtemperature"],
  ["电机过流", "Motor overcurrent"], ["电机短路", "Motor short circuit"],
  ["堵转", "Stall"], ["输入信号超时", "Input signal timeout"],
] as const;

export function LingkongMotorPanel() {
  const { message, modal } = App.useApp();
  const { lang } = useI18n();
  const zh = lang === "zh";
  const [spec, setSpec] = useState("can0");
  const [bitrate, setBitrate] = useState(1_000_000);
  const [host, setHost] = useState<HostCanStatus | null>(null);
  const [hostError, setHostError] = useState("");
  const [hostBusy, setHostBusy] = useState(false);
  const [passwordOpen, setPasswordOpen] = useState(false);
  const [password, setPassword] = useState("");
  const [connected, setConnected] = useState(false);
  const [busy, setBusy] = useState(false);
  const busyRef = useRef(false);
  const pollRef = useRef(false);
  const [devices, setDevices] = useState<LingkongProbe[]>([]);
  const [selectedId, setSelectedId] = useState<number | null>(null);
  const selectedRef = useRef<number | null>(null);
  const [initializedId, setInitializedId] = useState<number | null>(null);
  const [parameters, setParameters] = useState<Record<string, LingkongParameterValue>>({});
  const [drafts, setDrafts] = useState<Record<string, number[]>>({});
  const [pendingSave, setPendingSave] = useState(false);
  const [needsReboot, setNeedsReboot] = useState(false);
  const [snapshot, setSnapshot] = useState<LingkongSnapshot | null>(null);
  const [readError, setReadError] = useState("");
  const [mode, setMode] = useState<"velocity" | "position" | "torque">("velocity");
  const [target, setTarget] = useState<number | null>(0);
  const [limit, setLimit] = useState<number | null>(100);
  const gsUsb = isGsUsbSpec(spec);
  const ready = !!host?.up && host.bitrate === bitrate && !host.fd_enabled;
  const motorReady = connected && selectedId != null && initializedId === selectedId;

  const copy = zh ? {
    bus: "CAN 接口", inspect: "刷新接口状态", init: "初始化 / 修改 CAN 口",
    rate: "仲裁波特率", up: "已启动", down: "未启动", unknown: "未设置",
    fd: "CAN-FD 已开启", classic: "Classic CAN", password: "输入本机 sudo 密码",
    passwordHint: "将先关闭该接口，再设置波特率、关闭 CAN-FD 并启动。已初始化但参数不对时也可直接修改。",
    apply: "应用配置", applied: "CAN 口配置已验证", connect: "连接电机总线",
    disconnect: "安全断开", scan: "扫描 ID 1–32", devices: "发现的电机",
    manual: "手动选择 ID", status: "电机状态", select: "连接后选择或扫描电机 ID",
    refresh: "读取状态", run: "运行", stop: "停止（仍使能）", close: "关闭", clear: "清除故障",
    motion: "运动控制", send: "发送目标", target: "目标值", limit: "限制值",
    velocity: "速度 (°/s)", position: "多圈位置 (°)", torque: "转矩电流原始值",
    velocityLimit: "转矩电流限制 (1–2048)", positionLimit: "最高速度 (°/s)",
    torqueHint: "MHF 手册标称 ±2048 约对应 ±16.5 A；0xA1 转矩电流命令不受 Max Torque Current 限制。",
    stallHint: "MHF 手册将 Protect Stall 标为“暂未开放”。转矩模式不等于关闭堵转保护，不能保证堵转后持续输出。",
    estimatedCurrent: "目标电流估算",
    runFirst: "先点击“运行”，再发送运动目标。首次测试请从小目标和低限制值开始。",
    connectionHint: "V2.36 标准帧，ID = 0x140 + 电机 ID；默认 1 Mbps。连接和扫描不会使电机运动。",
    noDevice: "没有收到电机回复。检查供电、CAN H/L、终端电阻和电机波特率。",
    success: "命令已执行并读取到状态", connected: "总线已连接", disconnected: "总线已断开",
    temp: "电机温度", voltage: "母线电压", current: "母线电流", speed: "速度",
    iq: "转矩电流原始值", encoder: "编码器", angle: "多圈角度", state: "电机状态",
    errors: "故障", open: "运行", closed: "关闭", unknownState: "其他状态", none: "无", stale: "状态读取失败",
    socketOnly: "这里的 CAN 口初始化仅适用于 Linux SocketCAN。gs_usb 会在连接时配置为 Classic CAN 1 Mbps。",
    motorInit: "初始化所选电机", initDone: "电机初始化完成，保持关闭；参数已读取", initRequired: "请先初始化所选电机",
    initFlow: "配置主机 CAN → 连接总线 → 扫描/选择 ID → 初始化电机 → 检查参数 → 运行。初始化会关闭已运行的电机并尝试清除故障；关闭会清除圈数和旧指令，不会自动运行。",
    initState: "初始化状态", initialized: "已初始化（关闭）", notInitialized: "未初始化",
    readParams: "重新读取全部参数", ramParams: "控制参数 · RAM", settingParams: "设定参数 · 保存到 ROM",
    paramNote: "读回后才能编辑。控制参数 0xC1 写入 RAM 后立即生效，断电丢失；设定参数 0x42 暂存后由 0x44 保存到 ROM，重启后新设置生效。写入仅允许电机关闭且无故障。",
    write: "写入并核对", paramCurrent: "当前", range: "范围", readDone: "参数已重新读取", writeDone: "参数写入并读回一致",
    save: "保存设定到 ROM", saveDone: "ROM 保存成功，请重启电机", reboot: "重启电机", rebootDone: "重启命令已发送；请重新连接并初始化",
    savePending: "有未保存的设定参数", rebootPending: "设定已保存，需重启后生效",
    communicationWarning: "修改 ID、总线类型或 CAN 波特率后，保存并重启会改变后续连接方式。请确认新的 ID 和主机波特率。",
    rebootWarning: "重启后电机默认可能处于开启状态。页面会尝试重新关闭；如果 ID、总线类型或波特率改变，请按新参数重新连接并立即初始化。",
    rebootUnconfirmed: "重启命令已发送，但未确认电机重新关闭。请保持连接并再次初始化所选电机。",
    reconnectRate: "电机通讯参数已改变，请同步配置主机 CAN 口后重新连接。",
    unavailable: "使用手册还列出温度/电压/堵转等保护、旋转方向、刹车电阻与校准电压，但 V2.36 CAN 参数表没有给出这些项目的地址和编码；此页不会猜测写入。",
  } : {
    bus: "CAN interface", inspect: "Refresh interface", init: "Initialize / change CAN link",
    rate: "Nominal bitrate", up: "Up", down: "Down", unknown: "Unset",
    fd: "CAN-FD enabled", classic: "Classic CAN", password: "Enter your local sudo password",
    passwordHint: "The interface will be taken down, configured at the selected bitrate with CAN-FD off, then brought up. This also corrects an existing profile.",
    apply: "Apply configuration", applied: "CAN link configuration verified", connect: "Connect motor bus",
    disconnect: "Safe disconnect", scan: "Scan IDs 1–32", devices: "Discovered motors",
    manual: "Select ID manually", status: "Motor status", select: "Connect, then select or scan a motor ID",
    refresh: "Read status", run: "Run", stop: "Stop (enabled)", close: "Close", clear: "Clear fault",
    motion: "Motion control", send: "Send target", target: "Target", limit: "Limit",
    velocity: "Velocity (°/s)", position: "Multiturn position (°)", torque: "Torque current raw value",
    velocityLimit: "Torque current limit (1–2048)", positionLimit: "Maximum speed (°/s)",
    torqueHint: "The MHF manual maps ±2048 raw units to about ±16.5 A; the 0xA1 command is not limited by Max Torque Current.",
    stallHint: "The MHF manual marks Protect Stall as unavailable. Torque mode does not disable stall protection or guarantee continued output when stalled.",
    estimatedCurrent: "Estimated target current",
    runFirst: "Press Run before sending a motion target. Start with small targets and low limits when testing.",
    connectionHint: "V2.36 standard frames, ID = 0x140 + motor ID; default 1 Mbps. Connect and scan do not command motion.",
    noDevice: "No motor replied. Check power, CAN H/L, termination, and the motor bitrate.",
    success: "Command completed and status read", connected: "Bus connected", disconnected: "Bus disconnected",
    temp: "Motor temperature", voltage: "Bus voltage", current: "Bus current", speed: "Speed",
    iq: "Torque current raw", encoder: "Encoder", angle: "Multiturn angle", state: "Motor state",
    errors: "Faults", open: "Running", closed: "Closed", unknownState: "Other state", none: "None", stale: "Status read failed",
    socketOnly: "Host CAN setup here applies to Linux SocketCAN only. gs_usb is configured for Classic CAN 1 Mbps on connection.",
    motorInit: "Initialize selected motor", initDone: "Motor initialized and left closed; parameters loaded", initRequired: "Initialize the selected motor first",
    initFlow: "Configure host CAN → connect bus → scan/select ID → initialize motor → review parameters → Run. Initialization closes a running motor and attempts to clear faults. Close resets the turn count and prior target; initialization never starts motion.",
    initState: "Initialization", initialized: "Initialized (closed)", notInitialized: "Not initialized",
    readParams: "Read all parameters", ramParams: "Control parameters · RAM", settingParams: "Settings · save to ROM",
    paramNote: "Read back before editing. 0xC1 control writes take effect in RAM and are lost on power off. 0x42 settings are saved to ROM with 0x44 and take effect after reboot. Writes require a closed, fault-free motor.",
    write: "Write and verify", paramCurrent: "Current", range: "Range", readDone: "Parameters refreshed", writeDone: "Parameter written and read back",
    save: "Save settings to ROM", saveDone: "ROM save confirmed; reboot the motor", reboot: "Reboot motor", rebootDone: "Reboot sent; reconnect and initialize again",
    savePending: "Unsaved setting changes", rebootPending: "Saved settings need a reboot",
    communicationWarning: "Changing the ID, bus type or CAN bitrate changes how this motor is reached after save and reboot. Confirm the new ID and host bitrate.",
    rebootWarning: "The motor may power up enabled after reboot. The app will try to close it again. If the ID, bus type or bitrate changes, reconnect with the new settings and initialize immediately.",
    rebootUnconfirmed: "Reboot was sent, but closing the motor again was not confirmed. Keep the bus connected and initialize the selected motor again.",
    reconnectRate: "Motor communication settings changed. Match the host CAN link before reconnecting.",
    unavailable: "The user manual also lists temperature, voltage and stall protections, spin direction, brake resistor and alignment voltage. CAN V2.36 gives no parameter address or encoding for them, so this page does not guess writes.",
  };

  const chooseId = (id: number | null) => {
    selectedRef.current = id;
    setSelectedId(id);
    setInitializedId(null);
    setParameters({});
    setDrafts({});
    setPendingSave(false);
    setNeedsReboot(false);
    setSnapshot(null);
    setReadError("");
  };

  const acceptParameters = (values: LingkongParameterValue[]) => {
    setParameters(Object.fromEntries(values.map((value) => [value.key, value])));
    setDrafts(Object.fromEntries(values.map((value) => [value.key, [...value.values]])));
  };

  const inspect = useCallback(async () => {
    if (isGsUsbSpec(spec)) {
      setHost(null);
      setHostError("");
      return;
    }
    try {
      const iface = spec.trim().replace(/^socketcan:/, "");
      const next = await api.hostCanInspect(iface);
      setHost(next);
      setHostError("");
    } catch (error) {
      setHost(null);
      setHostError(errMsg(error));
    }
  }, [spec]);

  useEffect(() => {
    if (!connected) void inspect();
  }, [connected, inspect]);

  const configure = async () => {
    if (!password) return;
    const secret = password;
    setPassword("");
    setHostBusy(true);
    try {
      const next = await api.hostCanApply(spec.trim().replace(/^socketcan:/, ""), bitrate, secret);
      setHost(next);
      setHostError("");
      setPasswordOpen(false);
      message.success(copy.applied);
    } catch (error) {
      message.error(errMsg(error));
      await inspect();
    } finally {
      setHostBusy(false);
    }
  };

  const scan = async () => {
    if (busyRef.current) return;
    busyRef.current = true;
    setBusy(true);
    try {
      const found = await api.lingkongScan();
      setDevices(found);
      if (found.length === 0) message.warning(copy.noDevice);
      if (selectedRef.current == null && found.length > 0) chooseId(found[0].id);
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  };

  const connect = async () => {
    if (busyRef.current) return;
    busyRef.current = true;
    setBusy(true);
    try {
      await api.lingkongStart(spec.trim(), gsUsb ? 1_000_000 : bitrate);
      setConnected(true);
      message.success(copy.connected);
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  };

  const disconnect = async () => {
    if (busyRef.current) return;
    busyRef.current = true;
    setBusy(true);
    try {
      await api.lingkongStop();
      setConnected(false);
      setDevices([]);
      chooseId(null);
      message.info(copy.disconnected);
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  };

  const refresh = useCallback(async (id: number, showError = false) => {
    if (pollRef.current || busyRef.current) return;
    pollRef.current = true;
    try {
      const next = await api.lingkongSnapshot(id);
      if (selectedRef.current === id) {
        setSnapshot(next);
        setReadError("");
        if (next.error_state !== 0 || (next.motor_state !== 0x00 && next.motor_state !== 0x10)) {
          setInitializedId(null);
        }
      }
    } catch (error) {
      if (selectedRef.current === id) setReadError(errMsg(error));
      if (showError) message.error(errMsg(error));
    } finally {
      pollRef.current = false;
    }
  }, [message]);

  useEffect(() => {
    if (!connected || selectedId == null) return;
    setSnapshot(null);
    void refresh(selectedId);
    const timer = window.setInterval(() => void refresh(selectedId), 900);
    return () => window.clearInterval(timer);
  }, [connected, selectedId, refresh]);

  const action = async (kind: string, value = 0, cap = 0) => {
    if (selectedId == null || busyRef.current) return;
    busyRef.current = true;
    setBusy(true);
    try {
      const next = await api.lingkongAction(selectedId, kind, value, cap);
      setSnapshot(next);
      setReadError("");
      if (kind === "clear_fault") setInitializedId(null);
      message.success(copy.success);
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  };

  const initialize = async () => {
    if (selectedId == null || busyRef.current) return;
    const id = selectedId;
    busyRef.current = true;
    setBusy(true);
    setInitializedId(null);
    try {
      const result = await api.lingkongInitialize(id);
      if (selectedRef.current === id) {
        setSnapshot(result.snapshot);
        acceptParameters(result.parameters);
        setInitializedId(id);
        setPendingSave(result.settings_dirty);
        setNeedsReboot(result.reboot_required);
        setReadError("");
        message.success(copy.initDone);
      }
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  };

  const readParameters = async () => {
    if (selectedId == null || busyRef.current || !motorReady) return;
    busyRef.current = true;
    setBusy(true);
    try {
      acceptParameters(await api.lingkongReadParameters(selectedId));
      message.success(copy.readDone);
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  };

  const writeParameter = async (def: LingkongParameterDef) => {
    if (selectedId == null || busyRef.current || !motorReady) return;
    const values = drafts[def.key];
    if (!values || values.length !== (def.fields?.length ?? 1)
      || values.some((value) => !Number.isInteger(value) || value < 0 || value > def.max)) return;
    const id = selectedId;
    const write = async () => {
      busyRef.current = true;
      setBusy(true);
      if (def.group === "setting") setPendingSave(true);
      try {
        const result = await api.lingkongWriteParameter(id, def.key, values);
        setParameters((current) => ({ ...current, [def.key]: result }));
        setDrafts((current) => ({ ...current, [def.key]: [...result.values] }));
        message.success(copy.writeDone);
      } catch (error) {
        message.error(errMsg(error));
      } finally {
        busyRef.current = false;
        setBusy(false);
      }
    };
    if (["setting_driver_id", "setting_bus_type", "setting_can_baudrate"].includes(def.key)) {
      modal.confirm({ title: copy.communicationWarning, onOk: write });
    } else {
      await write();
    }
  };

  const saveSettings = async () => {
    if (selectedId == null || busyRef.current || !pendingSave) return;
    busyRef.current = true;
    setBusy(true);
    try {
      await api.lingkongSaveSettings(selectedId);
      setPendingSave(false);
      setNeedsReboot(true);
      message.success(copy.saveDone);
    } catch (error) {
      message.error(errMsg(error));
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  };

  const reboot = () => {
    if (selectedId == null || busyRef.current || !needsReboot || pendingSave) return;
    const id = selectedId;
    modal.confirm({ title: copy.reboot, content: copy.rebootWarning, onOk: async () => {
      busyRef.current = true;
      setBusy(true);
      try {
        const result = await api.lingkongReboot(id);
        setInitializedId(null);
        if (!result.closed_after_reboot && result.next_bitrate === (gsUsb ? 1_000_000 : bitrate) && result.next_id != null) {
          chooseId(result.next_id);
          message.warning(copy.rebootUnconfirmed);
        } else {
          await api.lingkongStop();
          setConnected(false);
          setDevices([]);
          chooseId(null);
          if (!gsUsb) setBitrate(result.next_bitrate);
          message.info(result.closed_after_reboot ? copy.rebootDone : copy.reconnectRate);
        }
      } catch (error) {
        message.error(errMsg(error));
      } finally {
        busyRef.current = false;
        setBusy(false);
      }
    }});
  };

  const parameterRow = (def: LingkongParameterDef) => {
    const actual = parameters[def.key]?.values;
    const draft = drafts[def.key];
    const scale = def.scale ?? 1;
    const changed = !!actual && !!draft && draft.some((value, index) => value !== actual[index]);
    const valid = !!draft && draft.length === (def.fields?.length ?? 1)
      && draft.every((value) => Number.isInteger(value) && value >= 0 && value <= def.max);
    return <div className="lingkong-param-row" key={def.key}>
      <div>
        <Typography.Text strong>{zh ? def.zh : def.en}</Typography.Text>
        <Typography.Text type="secondary" className="lingkong-param-meta">{def.address} · {copy.range} 0–{def.max / scale}</Typography.Text>
        <Typography.Text type="secondary" className="lingkong-param-meta">{copy.paramCurrent}: {actual ? actual.map((value) => value / scale).join(" / ") : "—"}</Typography.Text>
        {(zh ? def.noteZh : def.noteEn) && <Typography.Text type="secondary" className="lingkong-param-meta">{zh ? def.noteZh : def.noteEn}</Typography.Text>}
      </div>
      <div className="lingkong-param-values">
        {(def.fields ?? [""]).map((field, index) => <label className="lingkong-field" key={field || def.key}>
          <span>{field || copy.target}</span>
          {def.options ? <Select value={draft?.[index]} disabled={!motorReady || busy || !actual}
            onChange={(value) => setDrafts((current) => ({ ...current, [def.key]: [value] }))}
            style={{ width: 145 }} options={def.options.map((option) => ({ value: option.value, label: zh ? option.zh : option.en }))} />
            : <InputNumber value={draft?.[index] == null || !Number.isFinite(draft[index]) ? null : draft[index] / scale}
              min={0} max={def.max / scale} step={1 / scale} precision={scale === 100 ? 2 : 0}
              disabled={!motorReady || busy || !actual}
              onChange={(value) => setDrafts((current) => {
                const next = [...(current[def.key] ?? actual ?? [])];
                next[index] = value == null ? NaN : Math.round(value * scale);
                return { ...current, [def.key]: next };
              })} style={{ width: 145 }} />}
        </label>)}
      </div>
      <Button disabled={!motorReady || busy || !changed || !valid} onClick={() => void writeParameter(def)}>{copy.write}</Button>
    </div>;
  };

  const faults = snapshot
    ? ERROR_BITS.flatMap(([cn, en], bit) => snapshot.error_state & (1 << bit) ? [zh ? cn : en] : [])
    : [];

  return (
    <div className="lingkong-panel">
      <Alert type="info" showIcon message={copy.initFlow} description={copy.connectionHint} />
      <Card title={copy.bus}>
        <Space wrap align="end">
          <label className="lingkong-field">
            <span>{copy.bus}</span>
            <Input value={spec} disabled={connected || busy} onChange={(event) => setSpec(event.target.value)} style={{ width: 145 }} />
          </label>
          {!gsUsb && <label className="lingkong-field">
            <span>{copy.rate}</span>
            <Select value={bitrate} disabled={connected || busy} onChange={setBitrate} style={{ width: 140 }}
              options={[100_000, 125_000, 250_000, 500_000, 1_000_000].map((value) => ({
                value, label: `${value / 1000} kbit/s`,
              }))} />
          </label>}
          {!gsUsb && <Button disabled={connected || busy} onClick={() => void inspect()}>{copy.inspect}</Button>}
          {!gsUsb && <Button disabled={connected || busy || hostBusy || !host} onClick={() => {
            if (ready) message.info(copy.applied);
            else setPasswordOpen(true);
          }}>{copy.init}</Button>}
          {connected
            ? <Button danger loading={busy} onClick={() => void disconnect()}>{copy.disconnect}</Button>
            : <Button type="primary" loading={busy} disabled={!spec.trim() || (!gsUsb && !ready)} onClick={() => void connect()}>{copy.connect}</Button>}
        </Space>
        <div className="lingkong-interface-status">
          {gsUsb ? <Typography.Text type="secondary">{copy.socketOnly}</Typography.Text> : <>
            {host && <Space wrap>
              <Tag color={host.up ? "green" : "red"}>{host.up ? copy.up : copy.down}</Tag>
              <Tag color={host.bitrate === bitrate ? "blue" : "orange"}>{host.bitrate == null ? copy.unknown : `${host.bitrate / 1000} kbit/s`}</Tag>
              <Tag color={host.fd_enabled ? "orange" : "default"}>{host.fd_enabled ? copy.fd : copy.classic}</Tag>
            </Space>}
            {hostError && <Alert type="warning" showIcon message={hostError} />}
          </>}
        </div>
      </Card>

      <div className="lingkong-grid">
        <Card title={copy.devices} extra={<Button disabled={!connected} loading={busy} onClick={() => void scan()}>{copy.scan}</Button>}>
          <Space direction="vertical" style={{ width: "100%" }}>
            <label className="lingkong-field">
              <span>{copy.manual}</span>
              <InputNumber min={1} max={32} precision={0} disabled={!connected || busy} value={selectedId}
                onChange={(value) => chooseId(value)} style={{ width: 145 }} />
            </label>
            <div className="lingkong-device-list">
              {devices.map((device) => <Button key={device.id} type={selectedId === device.id ? "primary" : "default"}
                disabled={busy} onClick={() => chooseId(device.id)}>ID {device.id} · 0x{(0x140 + device.id).toString(16).toUpperCase()}</Button>)}
            </div>
            <Space wrap>
              <Button type="primary" disabled={!connected || selectedId == null || busy} onClick={() => void initialize()}>{copy.motorInit}</Button>
              <Tag color={motorReady ? "green" : "default"}>{copy.initState}: {motorReady ? copy.initialized : copy.notInitialized}</Tag>
            </Space>
            {pendingSave && <Tag color="orange">{copy.savePending}</Tag>}
            {needsReboot && <Tag color="blue">{copy.rebootPending}</Tag>}
          </Space>
        </Card>

        <Card title={copy.status} extra={<Button disabled={!connected || selectedId == null} onClick={() => selectedId != null && void refresh(selectedId, true)}>{copy.refresh}</Button>}>
          {!snapshot ? <Typography.Text type="secondary">{copy.select}</Typography.Text> : <>
            <Space wrap style={{ marginBottom: 12 }}>
              <Tag color={snapshot.motor_state === 0x00 ? "green" : snapshot.motor_state === 0x10 ? "default" : "orange"}>
                {snapshot.motor_state === 0x00 ? copy.open : snapshot.motor_state === 0x10 ? copy.closed : `${copy.unknownState} 0x${snapshot.motor_state.toString(16).toUpperCase()}`}
              </Tag>
              <Tag color={faults.length ? "red" : "green"}>{copy.errors}: {faults.length ? faults.join(", ") : copy.none}</Tag>
            </Space>
            <Descriptions size="small" column={{ xs: 1, sm: 2, lg: 3 }}>
              <Descriptions.Item label={copy.temp}>{snapshot.temperature_c} °C</Descriptions.Item>
              <Descriptions.Item label={copy.voltage}>{snapshot.voltage_v.toFixed(2)} V</Descriptions.Item>
              <Descriptions.Item label={copy.current}>{snapshot.bus_current_a.toFixed(2)} A</Descriptions.Item>
              <Descriptions.Item label={copy.speed}>{snapshot.speed_dps} °/s</Descriptions.Item>
              <Descriptions.Item label={copy.iq}>{snapshot.iq_raw}</Descriptions.Item>
              <Descriptions.Item label={copy.encoder}>{snapshot.encoder}</Descriptions.Item>
              <Descriptions.Item label={copy.angle}>{snapshot.angle_deg == null ? "—" : `${snapshot.angle_deg.toFixed(2)} °`}</Descriptions.Item>
              <Descriptions.Item label={copy.state}>0x{snapshot.motor_state.toString(16).padStart(2, "0")}</Descriptions.Item>
            </Descriptions>
          </>}
          {readError && <Alert type="warning" showIcon message={`${copy.stale}: ${readError}`} style={{ marginTop: 12 }} />}
        </Card>
      </div>

      <Card title={copy.ramParams} extra={<Button disabled={!motorReady || busy} onClick={() => void readParameters()}>{copy.readParams}</Button>}>
        <Alert type="info" showIcon message={copy.paramNote} style={{ marginBottom: 12 }} />
        {lingkongParameters.filter((def) => def.group === "ram").map(parameterRow)}
      </Card>

      <Card title={copy.settingParams} extra={<Space wrap>
        <Button disabled={!motorReady || busy || !pendingSave} onClick={() => void saveSettings()}>{copy.save}</Button>
        <Button disabled={!motorReady || busy || pendingSave || !needsReboot} onClick={reboot}>{copy.reboot}</Button>
      </Space>}>
        {lingkongParameters.filter((def) => def.group === "setting").map(parameterRow)}
        <Alert type="warning" showIcon message={copy.unavailable} style={{ marginTop: 16 }} />
      </Card>

      <Card title={copy.motion}>
        <Alert type="warning" showIcon message={motorReady ? copy.runFirst : copy.initRequired} style={{ marginBottom: 14 }} />
        <Space wrap style={{ marginBottom: 18 }}>
          <Button disabled={!motorReady || busy || !!readError || pendingSave || needsReboot || snapshot?.motor_state !== 0x10 || snapshot?.error_state !== 0} loading={busy} onClick={() => void action("run")}>{copy.run}</Button>
          <Button danger type="primary" disabled={!connected || selectedId == null || busy} loading={busy} onClick={() => void action("stop")}>{copy.stop}</Button>
          <Button danger disabled={!connected || selectedId == null || busy} loading={busy} onClick={() => void action("close")}>{copy.close}</Button>
          <Button disabled={!connected || selectedId == null || busy} loading={busy} onClick={() => void action("clear_fault")}>{copy.clear}</Button>
        </Space>
        <div className="lingkong-controls">
          <label className="lingkong-field">
            <span>{copy.motion}</span>
            <Select value={mode} onChange={(value) => { setMode(value); setTarget(0); setLimit(value === "position" ? 60 : 100); }} style={{ width: 190 }}
              options={[{ value: "velocity", label: copy.velocity }, { value: "position", label: copy.position }, { value: "torque", label: copy.torque }]} />
          </label>
          <label className="lingkong-field">
            <span>{mode === "velocity" ? copy.velocity : mode === "position" ? copy.position : copy.torque}</span>
            <InputNumber value={target} onChange={setTarget} min={mode === "torque" ? -2048 : mode === "velocity" ? -6000 : -21_474_836}
              max={mode === "torque" ? 2048 : mode === "velocity" ? 6000 : 21_474_836} step={mode === "torque" ? 1 : 0.1} style={{ width: 160 }} />
          </label>
          {mode !== "torque" && <label className="lingkong-field">
            <span>{mode === "velocity" ? copy.velocityLimit : copy.positionLimit}</span>
            <InputNumber value={limit} onChange={setLimit} min={1} max={mode === "velocity" ? 2048 : 65535} precision={0} style={{ width: 160 }} />
          </label>}
          <Button type="primary" disabled={!motorReady || busy || !!readError || pendingSave || needsReboot || snapshot?.motor_state !== 0x00 || snapshot?.error_state !== 0 || target == null || (mode !== "torque" && limit == null)} loading={busy}
            onClick={() => void action(mode, target ?? 0, limit ?? 0)}>{copy.send}</Button>
        </div>
        {mode === "torque" && <div style={{ marginTop: 12 }}>
          <Alert type="info" showIcon message={copy.stallHint} style={{ marginBottom: 8 }} />
          <Typography.Text type="secondary">
            {copy.torqueHint} {target != null && `${copy.estimatedCurrent}: ${(target * 16.5 / 2048).toFixed(2)} A`}
          </Typography.Text>
        </div>}
      </Card>

      <Modal title={copy.password} open={passwordOpen} okText={copy.apply} okButtonProps={{ disabled: !password }} confirmLoading={hostBusy}
        onCancel={() => { if (!hostBusy) { setPassword(""); setPasswordOpen(false); } }} onOk={() => void configure()}>
        <Typography.Paragraph type="secondary">{copy.passwordHint}</Typography.Paragraph>
        <Input.Password autoFocus autoComplete="off" value={password} onChange={(event) => setPassword(event.target.value)}
          onPressEnter={() => void configure()} />
      </Modal>
    </div>
  );
}
