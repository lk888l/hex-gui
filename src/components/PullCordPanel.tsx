import { useEffect, useRef, useState } from "react";
import { Alert, App, Button, Card, Descriptions, Input, InputNumber, Progress, Segmented, Space, Tag, Typography } from "antd";
import { api, errMsg, type PullCordStatus } from "../pullCordApi";
import { useI18n } from "../i18n";
import type { A1CurrentConfig, A2SpeedConfig, PullCordConfig, PullCordMode, PullCordState } from "../pullCordTypes";
import { PullCordChart as CordChart, type CordSample } from "./PullCordChart";
import { PullCordEncoder } from "./PullCordEncoder";
import { PullChassisPanel } from "./PullChassisPanel";
import "./PullCordPanel.css";

const POLL_MS = 100;

export function PullCordPanel() {
  const { message } = App.useApp();
  const { lang } = useI18n();
  const tr = (zh: string, en: string) => lang === "zh" ? zh : en;
  const [iface, setIface] = useState(() => /linux/i.test(navigator.userAgent) ? "can1" : "gs_usb0");
  const [encoderNode, setEncoderNode] = useState(16);
  const [encoderCenter, setEncoderCenter] = useState<number | null>(null);
  const [status, setStatus] = useState<PullCordStatus | null>(null);
  const [pollError, setPollError] = useState<string | null>(null);
  const [nid, setNid] = useState(1);
  const [draft, setDraft] = useState<PullCordConfig | null>(null);
  const [run, setRun] = useState<PullCordState | null>(null);
  const [busy, setBusy] = useState(false);
  const actions = useRef(0);
  const [stopping, setStopping] = useState(false);
  const samples = useRef<CordSample[]>([]);
  const [chartVersion, setChartVersion] = useState(0);

  useEffect(() => {
    let alive = true;
    void Promise.all([api.status(), api.settingsGet(), api.chassisSettingsGet()])
      .then(([next, settings, chassisSettings]) => {
        if (!alive) return;
        setStatus(next);
        setRun(next.run);
        setDraft(next.run?.config ?? { ...settings, control_mode: next.pending_mode ?? settings.control_mode });
        if (next.iface) setIface(next.iface);
        if (next.pending_node != null) setNid(next.pending_node);
        if (next.encoder) setEncoderNode(next.encoder.node_id);
        setEncoderCenter(next.connected ? next.encoder_center_deg : chassisSettings.follow.encoderCenterDeg);
      })
      .catch(error => { if (alive) message.error(errMsg(error)); });
    return () => { alive = false; };
  }, [message]);

  useEffect(() => {
    let alive = true;
    let pending = false;
    const tick = async () => {
      if (pending) return;
      pending = true;
      try {
        const next = await api.status();
        if (!alive) return;
        const state = next.run;
        setStatus(next);
        setRun(state);
        if (next.connected) setEncoderCenter(next.encoder_center_deg);
        setPollError(null);
        if (state?.running) {
          const now = performance.now();
          const currentMode = state.config.control_mode === "a1_current";
          samples.current.push({
            t: now,
            control_mode: state.config.control_mode,
            length_m: state.length_m,
            current_command_raw: currentMode ? state.current_command_raw : null,
            torque_limit_raw: currentMode ? null : state.torque_limit_raw,
            global_torque_limit_raw: currentMode ? null : state.global_torque_limit_raw,
            feedback_raw: state.measured_current_raw,
            speed_m_s: state.cord_speed_m_s,
            target_speed_m_s: currentMode || state.speed_command_dps == null ? null
              : state.speed_command_dps / 360 * 2 * Math.PI * state.config.drum_radius_m
                * (state.config.retract_positive ? -1 : 1),
          });
          while (samples.current.length && samples.current[0].t < now - 120_000) samples.current.shift();
          setChartVersion(n => n + 1);
        }
      } catch (error) { if (alive) setPollError(errMsg(error)); }
      finally { pending = false; }
    };
    void tick();
    const timer = window.setInterval(tick, POLL_MS);
    return () => { alive = false; window.clearInterval(timer); };
  }, []);

  const action = async (label: string, fn: () => Promise<unknown>) => {
    actions.current += 1;
    setBusy(true);
    try { await fn(); message.success(label); }
    catch (error) { message.error(`${label}: ${errMsg(error)}`); }
    finally {
      try { const next = await api.status(); setStatus(next); setRun(next.run); }
      catch (error) { setPollError(errMsg(error)); }
      actions.current -= 1;
      setBusy(actions.current > 0);
    }
  };
  const setNumber = (key: keyof PullCordConfig) => (value: number | null) => {
    if (value != null) setDraft(current => current ? { ...current, [key]: value } : current);
  };
  const setSpeedConfig = (key: keyof A2SpeedConfig) => (value: number | null) => {
    if (value != null) setDraft(current => current?.a2_speed
      ? { ...current, a2_speed: { ...current.a2_speed, [key]: value } } : current);
  };
  const setCurrentConfig = (key: keyof A1CurrentConfig) => (value: number | null) => {
    if (value != null) setDraft(current => current?.a1_current
      ? { ...current, a1_current: { ...current.a1_current, [key]: value } } : current);
  };
  const running = run?.running === true;
  const motorConnected = status?.connected === true;
  const needsStop = status?.pending_node != null;
  const controlActive = running && run.phase !== "fault" && !pollError;
  const mode = run?.config.control_mode ?? status?.pending_mode ?? draft?.control_mode ?? "a2_speed";
  const currentMode = mode === "a1_current";
  const chartMode = samples.current[samples.current.length - 1]?.control_mode ?? mode;
  const a1 = currentMode ? draft?.a1_current : null;
  const a2 = currentMode ? null : draft?.a2_speed;
  const liveA1 = run?.config.a1_current;
  const liveA2 = run?.config.a2_speed;
  const profile = a1 ?? a2;
  const payoutRaw = a1?.payout_current_raw ?? a2?.payout_raw ?? 0;
  const retractRaw = a1?.retract_current_raw ?? a2?.retract_raw ?? 0;
  const clippedTargets = profile && Math.max(profile.homing_raw,
    retractRaw + Math.max(0, profile.alert_raw - profile.near_raw)) > profile.max_raw;
  const invalidLimits = !!profile && (payoutRaw > profile.near_raw || profile.near_raw > retractRaw);
  // CAN V2.36 Speed Limit: 600000 × 0.01 °/s = 1000 rpm.
  const maxSpeed = draft ? 1000 / 60 * (2 * Math.PI * draft.drum_radius_m) : 0;
  const invalidSpeed = !!draft && (draft.homing_speed_m_s > maxSpeed || (!currentMode && draft.retract_speed_m_s > maxSpeed));
  const validProfile = !!profile && !invalidLimits && !invalidSpeed;
  const resetDraft = async () => {
    const defaults = await api.pullCordDefaultConfig();
    if (needsStop || running) {
      defaults.control_mode = mode;
      const active = run?.config ?? draft;
      if (active) {
        defaults.drum_radius_m = active.drum_radius_m;
        defaults.retract_positive = active.retract_positive;
        if (currentMode && active.a1_current && defaults.a1_current) {
          defaults.a1_current.max_raw = active.a1_current.max_raw;
          defaults.a1_current.full_current_raw = active.a1_current.full_current_raw;
          defaults.max_torque_permille = active.max_torque_permille;
        }
      }
    }
    setDraft(defaults);
  };

  return <Space direction="vertical" size={12} className="pull-cord-panel">
    {pollError && <Alert type="error" showIcon message={tr("状态读取失败", "Status unavailable")} description={pollError} />}
    <Card size="small" title={tr("瓴控 MHF5010 拉线电机", "Lingkong MHF5010 pull cord")}>
      <Space wrap>
        <label>{tr("CAN 接口", "CAN interface")} <Input value={iface} onChange={event => setIface(event.target.value)} disabled={motorConnected || busy}
          aria-label="Pull cord CAN interface" placeholder="can1 / gs_usb0" style={{ width: 160 }} /></label>
        <label>{tr("编码器节点（十进制）", "Encoder node (decimal)")} <InputNumber aria-label="Encoder node ID" min={1} max={127} precision={0}
          disabled={motorConnected || busy} value={encoderNode} onChange={value => { if (value != null) setEncoderNode(value); }} style={{ width: 80 }} /></label>
        <label>{tr("方向中心角", "Direction centre")} <InputNumber aria-label="Encoder centre" min={0} max={359.99} step={1}
          disabled={motorConnected || busy} value={encoderCenter} onChange={value => { if (value != null) setEncoderCenter(value); }} addonAfter="°" style={{ width: 110 }} /></label>
        {motorConnected ?
          <Button danger disabled={busy} onClick={() => void action(tr("断开电机 CAN", "Disconnect motor CAN"), async () => {
            await api.pullMotorDisconnect();
          })}>{tr("断开", "Disconnect")}</Button> :
          <Button disabled={busy || !iface.trim() || !!pollError || encoderCenter == null} onClick={() => void action(tr("连接电机 CAN", "Connect motor CAN"), async () => {
            if (encoderCenter == null) return;
            await api.pullMotorConnect(iface, encoderNode, encoderCenter);
          })}>{tr("连接 1 Mbps 经典 CAN", "Connect Classic CAN 1 Mbps")}</Button>}
        <Tag color={motorConnected ? "green" : "default"}>{motorConnected ? tr("已连接", "Connected") : tr("未连接", "Disconnected")}</Tag>
      </Space>
      <Typography.Paragraph type="secondary" style={{ marginTop: 8, marginBottom: 0 }}>
        {tr("MHF5010 与方向编码器共用 1 Mbps 经典 CAN，默认编码器节点为 0x10。Linux 的 can1 需预先配置为 1 Mbps、关闭 FD；USB-CAN 可填写 gs_usb0。连接只开启反馈监测，点击“启动 / 归零”后才运行电机。", "MHF5010 and the direction encoder share Classic CAN at 1 Mbps; the default encoder node is 0x10. Preconfigure Linux can1 at 1 Mbps with FD off, or select gs_usb0 for USB-CAN. Connecting starts monitoring; only Start / home enables the motor.")}
      </Typography.Paragraph>
    </Card>

    <Card size="small" title={tr("运行", "Run")}>
      <Space wrap align="center">
        <span>{tr("电机 ID", "Motor ID")}</span>
        <InputNumber aria-label="MHF motor ID" min={1} max={32} precision={0} value={nid}
          disabled={needsStop || busy} onChange={value => { if (value != null) setNid(value); }} />
        <span>{tr("拉线控制模式", "Cord control mode")}</span>
        <Segmented<PullCordMode> aria-label={tr("拉线控制模式", "Cord control mode")} value={mode}
          disabled={!draft || !status || busy || needsStop || running || !!pollError}
          options={[
            { value: "a2_speed", label: tr("A2 速度模式", "A2 speed mode") },
            { value: "a1_current", label: tr("A1 电流模式", "A1 current mode") },
          ]}
          onChange={control_mode => setDraft(current => current ? { ...current, control_mode } : current)} />
        <Button type="primary" disabled={!motorConnected || !validProfile || needsStop || busy || !!pollError}
          onClick={() => void action(tr("启动拉线", "Start pull cord"), async () => {
            if (!draft) return;
            samples.current = [];
            setChartVersion(n => n + 1);
            const state = await api.pullCordStart(nid, draft);
            setRun(state); setDraft(state.config);
          })}>{tr("启动 / 归零", "Start / home")}</Button>
        <Button danger loading={stopping} disabled={(!needsStop && !busy) || stopping} onClick={() => {
          setStopping(true);
          void action(tr("停机", "Stop"), () => api.pullCordStop()).finally(() => setStopping(false));
        }}>{tr("停机并关闭电机", "Stop and close motor")}</Button>
        <Button disabled={!controlActive || busy} onClick={() => void action(tr("重新归零", "Rehome"), () => api.pullCordRehome())}>
          {tr("重新归零", "Rehome")}
        </Button>
      </Space>
      <Typography.Paragraph type="secondary" style={{ marginTop: 8, marginBottom: 0 }}>
        {tr("磁吸手柄可从零点附近直接归零。确保手柄可自由收回到止挡；线速绝对值低于 0.02 m/s 持续 600 ms 后确认零点。方向错误时立即停机，修改收线方向后重试。", "The magnetic handle can home directly from near zero. Leave it free to reach the stop; homing completes after cord speed stays below 0.02 m/s in magnitude for 600 ms. Stop immediately if direction is wrong.")}
      </Typography.Paragraph>
      {(run?.fault || status?.fault) && <Alert type="error" showIcon style={{ marginTop: 12 }} message={run?.fault || status?.fault}
        description={tr("点击“停机并关闭电机”完成关闭确认后再启动。", "Confirm Stop and close motor before starting again.")} />}
      {run?.phase === "homing" && <Progress percent={Math.round(run.homing_progress * 100)} status="active" />}
      {run && <Descriptions bordered size="small" column={{ xs: 1, sm: 2, md: 3 }} style={{ marginTop: 12 }}>
        <Descriptions.Item label={tr("阶段", "Phase")}>{run.phase}</Descriptions.Item>
        <Descriptions.Item label={tr("线长", "Cord length")}>{fmt(run.length_m, 3)} m</Descriptions.Item>
        <Descriptions.Item label={tr("出线速度", "Payout speed")}>{fmt(run.cord_speed_m_s, 3)} m/s</Descriptions.Item>
        {currentMode ? <>
          <Descriptions.Item label={tr("A1 电流指令（原始值）", "A1 current command (raw)")}>{run.current_command_raw} raw</Descriptions.Item>
          <Descriptions.Item label={tr("已生效出线 / 静止 / 收线基础电流", "Applied payout / stationary / retract base current")}>
            {liveA1 ? `${liveA1.payout_current_raw} / ${liveA1.near_raw} / ${liveA1.retract_current_raw} raw` : "—"}
          </Descriptions.Item>
          <Descriptions.Item label={tr("指令幅值 / 满电流参考", "Command magnitude / full-current reference")}>
            {liveA1 ? `${fmt(Math.abs(run.current_command_raw) / liveA1.full_current_raw * 100, 1)}% (${liveA1.full_current_raw} raw)` : "—"}
          </Descriptions.Item>
          <Descriptions.Item label={tr("电流指令上限", "Current command ceiling")}>{liveA1?.max_raw ?? "—"} raw</Descriptions.Item>
        </> : <>
          <Descriptions.Item label={tr("A2 收线目标速度", "A2 retract target speed")}>
            {run.speed_command_dps == null ? "—"
              : `${fmt(Math.abs(run.speed_command_dps) / 360 * 2 * Math.PI * run.config.drum_radius_m, 3)} m/s`}
          </Descriptions.Item>
          <Descriptions.Item label={tr("驱动全局限矩（已确认原始值）", "Driver global limit (confirmed raw)")}>
            <Typography.Text strong>{run.global_torque_limit_raw ?? "—"} raw</Typography.Text>
          </Descriptions.Item>
          <Descriptions.Item label={tr("A2 帧内限矩目标（原始值）", "A2 frame limit target (raw)")}>{run.torque_limit_raw ?? "—"} raw</Descriptions.Item>
          <Descriptions.Item label={tr("全局限矩 / 最大力矩映射参考", "Global limit / full-torque reference")}>
            {liveA2 && run.global_torque_limit_raw != null ? `${fmt(run.global_torque_limit_raw / liveA2.full_torque_raw * 100, 1)}% (${liveA2.full_torque_raw} raw)` : "—"}
          </Descriptions.Item>
          <Descriptions.Item label={tr("总限矩封顶（配置值）", "Overall limit ceiling (configured)")}>{liveA2?.max_raw ?? "—"} raw</Descriptions.Item>
          <Descriptions.Item label={tr("已生效速度环 Kp / Ki / Kd", "Applied speed-loop Kp / Ki / Kd")}>
            {liveA2 ? `${liveA2.speed_kp} / ${liveA2.speed_ki} / ${liveA2.speed_kd}` : "—"}
          </Descriptions.Item>
          <Descriptions.Item label={tr("已生效加减速斜率", "Applied speed ramp")}>{liveA2?.speed_ramp_dps2 ?? "—"} °/s²</Descriptions.Item>
        </>}
        <Descriptions.Item label={tr("出线口相对中心偏角", "Outlet offset from centre")}>{fmt(run.encoder_offset_deg, 1)} °</Descriptions.Item>
        <Descriptions.Item label={tr("反馈电流原始值", "Current feedback counts")}>{run.measured_current_raw ?? "—"}</Descriptions.Item>
        <Descriptions.Item label={tr("力矩 / 线力", "Torque / cord force")}>{tr("未标定", "Uncalibrated")}</Descriptions.Item>
        <Descriptions.Item label={tr("电机转速", "Motor speed")}>{run.motor_speed_dps ?? "—"} °/s</Descriptions.Item>
        <Descriptions.Item label={tr("多圈角度", "Multi-turn angle")}>{fmt(run.position_angle_deg, 2)} °</Descriptions.Item>
        <Descriptions.Item label={tr("母线电压", "Bus voltage")}>{fmt(run.bus_voltage_v, 2)} V</Descriptions.Item>
        <Descriptions.Item label={tr("电机温度", "Motor temperature")}>{run.motor_temp_c ?? "—"} °C</Descriptions.Item>
        <Descriptions.Item label={currentMode ? tr("反馈年龄 A1 / 角度 / 状态", "Feedback age A1 / angle / status")
          : tr("反馈年龄 A2 / 角度 / 状态", "Feedback age A2 / angle / status")}>
          {(currentMode ? run.a1_feedback_age_ms : run.a2_feedback_age_ms) ?? "—"} / {run.angle_feedback_age_ms ?? "—"} / {run.motor_status_age_ms ?? "—"} ms
        </Descriptions.Item>
      </Descriptions>}
    </Card>

    <PullCordEncoder state={status?.encoder ?? null} canZero={!busy && !needsStop && !!status?.encoder?.angle_usable && !pollError}
      onZero={() => void action(tr("编码器零点", "Encoder zero"), async () => {
        const result = await api.encoderZero();
        if (!result.applied_live) message.warning(tr("零点已写入；尚未检测到新零点反馈，请重启编码器后核对。", "Zero was written; no fresh zero feedback yet. Restart the encoder and verify."));
      })} />

    <PullChassisPanel status={status?.chassis} unavailable={!status || !!pollError || stopping} cordMode={mode}
      retractTargetSpeedMps={running ? run.config.retract_speed_m_s : draft?.retract_speed_m_s ?? null}
      pullReady={controlActive && run?.phase === "running" && !!run.online && !!status?.encoder?.angle_usable
        && (status.encoder.position_age_ms ?? Infinity) <= 150 && !busy}
      liveInput={{ cordLengthM: run?.length_m ?? null, cordSpeedMps: run?.cord_speed_m_s ?? null,
        encoderAngleDeg: status?.encoder?.angle_usable ? status.encoder.angle_degrees : null }}
      refresh={async () => { const next = await api.status(); setStatus(next); setRun(next.run); setPollError(null); }} />

    <Card size="small" title={tr("响应曲线", "Response chart")}>
      <CordChart samples={samples} mode={chartMode} chartVersion={chartVersion} windowSec={20} />
      <Typography.Text type="secondary">{chartMode === "a1_current" ? tr(
        "A1 电流指令与电机反馈均以带符号原始值显示。实测线速以出线为正、收线为负。",
        "The A1 command and current feedback are signed raw counts. Measured cord speed is positive for payout and negative for retraction."
      ) : tr("已确认全局限矩显示驱动器回复确认的实时上限，A2 曲线显示帧内目标；反馈电流需单独观察。最大力矩映射仅用于限矩参考占比，不换算反馈电流或 N·m。实测线速以出线为正、收线为负。", "The confirmed global limit shows the live value acknowledged by the driver; the A2 curve shows the frame target. Observe current feedback separately. The full-torque reference gives limit percentages only, not feedback current or N·m. Measured cord speed is positive for payout and negative for retraction.")}</Typography.Text>
    </Card>

    {draft && <Card size="small" title={tr("拉线参数", "Pull cord settings")}>
      <Alert type="info" showIcon message={currentMode ? tr(
        "A1 电流始终朝收线方向。归零后，实测线速大于 0.005 m/s 时用出线电流，小于 −0.005 m/s 时用收线电流，其余用静止电流。进入末端过渡区后，三档按线长叠加相同的电流变化量。归零使用独立电流，接近归零速度上限时电流逐渐减小。",
        "A1 always pulls inward. After homing, measured cord speed above 0.005 m/s selects payout current, below −0.005 m/s selects retract current, and the middle range uses stationary current. All three share the same end-zone adjustment. Homing uses its own current, which tapers near the homing speed limit."
      ) : tr(
        "A2 速度模式按目标速度收线，根据实测线速选择出线、静止或收线上限，并实时写入驱动全局限矩。向外拉线时上限较低，收线时可使用较大上限。各阶段限矩均受“总限矩封顶”约束，单位为 0–2000 原始值。最大力矩映射参考只改变显示占比。每次启动会重新配置速度环 Kp/Ki/Kd 和加减速斜率。",
        "A2 speed mode retracts at the target speed. Measured cord speed selects the payout, stationary or retract limit, which updates the driver’s global limit in real time. Payout uses a lower limit; retraction can use a higher one. All phase limits are bounded by the overall ceiling, in raw values from 0 to 2000. The full-torque reference only changes displayed percentages. Every start reapplies speed-loop Kp/Ki/Kd and the speed ramp."
      )} />
      {!a1 && !a2 && <Alert type="warning" showIcon style={{ marginTop: 12 }} message={tr("缺少所选模式的参数，请加载默认参数。", "Settings for the selected mode are missing. Load the defaults.")} />}
      {invalidSpeed && <Alert type="warning" showIcon style={{ marginTop: 12 }} message={currentMode
        ? tr(`请将归零收线速度软上限设为当前鼓半径允许的 ${fmt(maxSpeed, 3)} m/s 以内。`, `Set the homing speed soft limit within ${fmt(maxSpeed, 3)} m/s for this drum radius.`)
        : tr(`请将 A2 归零和收线目标速度设为当前鼓半径允许的 ${fmt(maxSpeed, 3)} m/s 以内。`, `Set the A2 homing and retract speeds within ${fmt(maxSpeed, 3)} m/s for this drum radius.`)} />}
      {invalidLimits && <Alert type="warning" showIcon style={{ marginTop: 12 }} message={currentMode ? tr(
        "请保持出线电流 ≤ 静止电流 ≤ 收线电流。",
        "Keep payout current ≤ stationary current ≤ retract current."
      ) : tr(
        "请保持：出线力矩上限 ≤ 静止轻拉上限 ≤ 收线力矩上限。",
        "Keep the payout limit ≤ stationary limit ≤ retract limit."
      )} />}
      {profile && clippedTargets && <Alert type="info" showIcon style={{ marginTop: 12 }} message={currentMode ? tr(
        `部分电流目标超过指令上限，实际下发幅值最多为 ${profile.max_raw} raw。`,
        `Some current targets exceed the command ceiling. Their magnitude is capped at ${profile.max_raw} raw.`
      ) : tr(
        `部分阶段或末端叠加后的限矩超过总限矩封顶 ${profile.max_raw} raw，实际写入会被限幅。`,
        `Some phase limits, including end-zone adjustment, exceed the overall ceiling of ${profile.max_raw} raw and will be clipped.`
      )} />}
      <div className="pull-cord-settings">
        <Field label={tr("鼓半径", "Drum radius")} value={draft.drum_radius_m} set={setNumber("drum_radius_m")}
          min={0.001} step={0.001} unit="m" disabled={needsStop || busy} />
        <Field label={tr("总线长", "Total cord length")} value={draft.cord_total_length_m} set={setNumber("cord_total_length_m")}
          min={0.1} step={0.1} unit="m" />
        <label>{tr("收线方向", "Retract direction")}<br />
          <Segmented disabled={needsStop || busy} value={draft.retract_positive ? "positive" : "negative"}
            onChange={value => setDraft({ ...draft, retract_positive: value === "positive" })}
            options={[{ label: "+", value: "positive" }, { label: "−", value: "negative" }]} />
        </label>
        {a1 && <>
          <Field label={tr("归零电流", "Homing current")} value={a1.homing_raw} set={setCurrentConfig("homing_raw")}
            min={0} max={2048} step={1} precision={0} unit="raw" />
          <Field label={tr("出线电流", "Payout current")} value={a1.payout_current_raw} set={setCurrentConfig("payout_current_raw")}
            min={0} max={2048} step={1} precision={0} unit="raw" />
          <Field label={tr("静止电流", "Stationary current")} value={a1.near_raw} set={setCurrentConfig("near_raw")}
            min={0} max={2048} step={1} precision={0} unit="raw" />
          <Field label={tr("收线电流", "Retract current")} value={a1.retract_current_raw} set={setCurrentConfig("retract_current_raw")}
            min={0} max={2048} step={1} precision={0} unit="raw" />
          <Field label={tr("末端静止电流", "End-zone stationary current")} value={a1.alert_raw} set={setCurrentConfig("alert_raw")}
            min={0} max={2048} step={1} precision={0} unit="raw" />
          <Field label={tr("电流指令上限", "Current command ceiling")} value={a1.max_raw} set={setCurrentConfig("max_raw")}
            min={0} max={2048} step={1} precision={0} unit="raw" disabled={needsStop || busy} />
          <Field label={tr("满电流映射参考值", "Full-current reference")} value={a1.full_current_raw} set={setCurrentConfig("full_current_raw")}
            min={1} max={2048} step={1} precision={0} unit="raw" disabled={needsStop || busy} />
        </>}
        {a2 && <>
          <Field label={tr("出线力矩上限（原始值）", "Payout torque limit (raw)")} value={a2.payout_raw} set={setSpeedConfig("payout_raw")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("静止轻拉上限（原始值）", "Stationary pull limit (raw)")} value={a2.near_raw} set={setSpeedConfig("near_raw")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("收线力矩上限（原始值）", "Retract torque limit (raw)")} value={a2.retract_raw} set={setSpeedConfig("retract_raw")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("末端静止上限（原始值）", "End-zone stationary limit (raw)")} value={a2.alert_raw} set={setSpeedConfig("alert_raw")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("归零力矩上限（原始值）", "Homing torque limit (raw)")} value={a2.homing_raw} set={setSpeedConfig("homing_raw")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("总限矩封顶（原始值）", "Overall torque ceiling (raw)")} value={a2.max_raw} set={setSpeedConfig("max_raw")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("最大力矩映射参考值", "Full-torque reference")} value={a2.full_torque_raw} set={setSpeedConfig("full_torque_raw")}
            min={1} max={2048} step={1} precision={0} unit="raw" />
        </>}
        <Field label={currentMode ? tr("归零收线速度软上限", "Homing speed soft limit") : tr("归零目标速度", "Homing target speed")} value={draft.homing_speed_m_s} set={setNumber("homing_speed_m_s")}
          min={0.01} max={maxSpeed} step={0.05} unit="m/s" />
        <Field label={currentMode ? tr("底盘回收制动参考速度", "Chassis retraction brake reference") : tr("收线目标速度", "Retract target speed")}
          value={draft.retract_speed_m_s} set={setNumber("retract_speed_m_s")}
          min={0.01} max={currentMode ? undefined : maxSpeed} step={0.05} unit="m/s" />
        {a2 && <>
          <Field label={tr("速度环 Kp", "Speed-loop Kp")} value={a2.speed_kp} set={setSpeedConfig("speed_kp")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("速度环 Ki", "Speed-loop Ki")} value={a2.speed_ki} set={setSpeedConfig("speed_ki")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("速度环 Kd", "Speed-loop Kd")} value={a2.speed_kd} set={setSpeedConfig("speed_kd")}
            min={0} max={2000} step={1} precision={0} unit="raw" />
          <Field label={tr("加减速斜率", "Speed ramp")} value={a2.speed_ramp_dps2} set={setSpeedConfig("speed_ramp_dps2")}
            min={1} max={600000} step={100} precision={0} unit="°/s²" />
        </>}
        <Field label={tr("基础轻拉区结束线长", "Base-pull zone end")} value={draft.zone_near_end_m} set={setNumber("zone_near_end_m")}
          min={0.01} step={0.1} unit="m" />
        <Field label={tr("末端区起点", "End-zone start")} value={draft.zone_alert_start_m} set={setNumber("zone_alert_start_m")}
          min={0.02} step={0.1} unit="m" />
      </div>
      {a2 && <Typography.Paragraph type="secondary" style={{ marginTop: 12, marginBottom: 0 }}>
        {tr(
          "按实测线速判断：大于 0.005 m/s 为出线，小于 −0.005 m/s 为收线，中间为静止轻拉。过渡区按线长逐渐叠加“末端静止上限 − 静止轻拉上限”的差值，受总限矩封顶限制后写入驱动全局限矩。方向切换直接更新限矩；修改驱动速度上限、Kp/Ki/Kd 或斜率时会短暂降为零限矩、零速并重新配置。点击“应用到运行”后查看上方已确认全局限矩；“保存到本机”供下次启动使用。鼓半径和收线方向需停机修改。",
          "Measured cord speed above 0.005 m/s selects payout, below −0.005 m/s selects retraction, and the middle range selects stationary pull. Across the transition zone, the difference between end-zone and base stationary limits is added gradually, then bounded by the overall ceiling and written to the driver’s global limit. Direction changes update the limit directly. Changes to the driver speed cap, Kp/Ki/Kd or ramp briefly set zero torque and speed while reconfiguring. Apply to run, then check the confirmed global limit above; save locally for the next start. Stop before changing drum radius or retract direction."
        )}
      </Typography.Paragraph>}
      <Space wrap style={{ marginTop: 14 }}>
        <Button disabled={busy || !controlActive || !validProfile} onClick={() => void action(tr("参数已提交，生效值见运行状态", "Settings submitted; see applied values in run status"), async () => {
          await api.pullCordSetConfig(draft);
          setRun((await api.status()).run);
        })}>{tr("应用到运行", "Apply to run")}</Button>
        <Button disabled={busy || !validProfile} onClick={() => void action(tr("保存参数", "Save settings"), async () => {
          setDraft(await api.settingsSave(draft));
        })}>{tr("保存到本机", "Save locally")}</Button>
        <Button disabled={busy} onClick={() => void action(tr("恢复默认", "Reset defaults"), resetDraft)}>
          {tr("恢复默认", "Reset defaults")}
        </Button>
      </Space>
    </Card>}
    {!draft && <Button disabled={busy} onClick={() => void action(tr("加载默认参数", "Load defaults"), resetDraft)}>
      {tr("加载默认参数", "Load defaults")}
    </Button>}
  </Space>;
}

function fmt(value: number | null | undefined, digits: number): string {
  return value == null || !Number.isFinite(value) ? "—" : value.toFixed(digits);
}

function Field({ label, value, set, min, max, step, unit, disabled, precision }: {
  label: string; value: number; set: (value: number | null) => void;
  min?: number; max?: number; step: number; unit: string; disabled?: boolean; precision?: number;
}) {
  return <label style={{ display: "block" }}>{label}<br />
    <InputNumber aria-label={label} style={{ width: "100%" }} value={value} onChange={set}
      min={min} max={max} step={step} addonAfter={unit} disabled={disabled} precision={precision} />
  </label>;
}
