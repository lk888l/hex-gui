import { useEffect, useRef, useState } from "react";
import { Alert, App, Button, Card, Checkbox, Collapse, Descriptions, Input, InputNumber, Select, Space, Table, Tag, Typography } from "antd";
import { api, errMsg } from "../pullCordApi";
import type { FollowInput, FollowParameters, PullChassisConfig, PullChassisStatus } from "../pullChassisTypes";
import type { PullCordMode } from "../pullCordTypes";
import { useI18n } from "../i18n";
import { BasePoseViewer } from "./BasePoseViewer";
import { PaceFollowControls } from "./PaceFollowControls";
import { BehindSpeedControls } from "./BehindSpeedControls";

// Retain the reference controller's 200 RPM tuning envelope, with HopeA3 wheels.
const MAX_FOLLOW_MPS = 200 * Math.PI * 0.154 / 60;

export function PullChassisPanel({ status, liveInput, cordMode, retractTargetSpeedMps, pullReady, unavailable, refresh }: {
  status?: PullChassisStatus;
  liveInput: FollowInput;
  cordMode: PullCordMode;
  retractTargetSpeedMps: number | null;
  pullReady: boolean;
  unavailable: boolean;
  refresh: () => Promise<void>;
}) {
  const { lang } = useI18n();
  const { message } = App.useApp();
  const tr = (zh: string, en: string) => lang === "zh" ? zh : en;
  const [config, setConfig] = useState<PullChassisConfig | null>(null);
  const [iface, setIface] = useState(() => /linux/i.test(navigator.userAgent) ? "can0" : "gs_usb1");
  const [bitrate, setBitrate] = useState(5_000_000);
  const [busy, setBusy] = useState(false);
  const [stopping, setStopping] = useState(false);
  const [manual, setManual] = useState<FollowInput>({ cordLengthM: 0, cordSpeedMps: 0, encoderAngleDeg: 60 });
  const loaded = useRef(false);
  const refreshRef = useRef(refresh);
  refreshRef.current = refresh;
  const run = status?.run;
  const running = !!run?.running;
  const ownsSession = !!status?.needs_stop;
  const structuralLocked = ownsSession || busy;

  // Reattach by observation. Draft restoration never enables a motor or
  // writes parameters to an already-running session.
  useEffect(() => {
    if (!status || loaded.current) return;
    let alive = true;
    const restore = async () => {
      const restored = status.needs_stop && status.run ? status.run.config : await api.chassisSettingsGet();
      if (!alive) return;
      loaded.current = true;
      setConfig(restored);
      if (status.iface) setIface(status.iface);
      if (status.run) setManual(status.run.input);
      else setManual({ cordLengthM: 0, cordSpeedMps: 0, encoderAngleDeg: restored.follow.encoderCenterDeg });
    };
    void restore().catch(e => { if (alive) { loaded.current = false; message.error(errMsg(e)); } });
    return () => { alive = false; };
  }, [!!status, message]);

  const stop = async () => {
    setStopping(true);
    try { await api.chassisStop(); }
    catch (error) { message.error(errMsg(error)); }
    finally {
      await refreshRef.current().catch(error => message.error(errMsg(error)));
      setStopping(false);
    }
  };
  const stopRef = useRef(stop);
  stopRef.current = stop;
  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      if (event.code === "Escape") { event.preventDefault(); void stopRef.current(); }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  const action = async (operation: () => Promise<unknown>) => {
    setBusy(true);
    try { await operation(); }
    catch (error) { message.error(errMsg(error)); }
    finally { await refreshRef.current().catch(error => message.error(errMsg(error))); setBusy(false); }
  };
  const set = <K extends keyof PullChassisConfig>(key: K, value: PullChassisConfig[K]) =>
    setConfig(current => current && ({ ...current, [key]: value }));
  const setFollow = <K extends keyof FollowParameters>(key: K, value: FollowParameters[K]) =>
    setConfig(current => current && ({ ...current, follow: { ...current.follow, [key]: value } }));
  const field = (key: keyof FollowParameters, label: string, min: number, max: number, step = 0.01) =>
    <label className="pull-chassis-field" key={key}>{label}<InputNumber aria-label={label} disabled={busy}
      value={config?.follow[key] as number} min={min} max={max} step={step}
      onChange={value => { if (value != null) setFollow(key, value); }} /></label>;
  const input = config?.source === "manual" ? manual : running ? run.input : liveInput;
  const ready = config?.source === "manual" || pullReady;
  const feedback = run?.chassis;
  const follow = run?.follow_status;
  const phase = !running ? tr("已停止", "Stopped")
    : run?.retract_braking ? tr("回收超速制动", "Retraction speed braking")
    : follow?.braking === "ahead_switch" ? tr("切换制动", "Braking for ahead guidance")
    : follow?.braking ? tr("长线制动", "Long-cord braking")
    : follow?.payout_stop ? tr("猛拉停车", "Payout stop")
    : follow?.travel_mode === "ahead" ? tr("身前引导", "Ahead guidance")
    : follow?.waiting_for_pull_back ? tr("等待收线重启", "Waiting for retraction")
    : follow?.retract_resume ? tr("收线重启", "Retraction restart")
    : run?.lateral_pace_active ? tr("全向步速同步", "Omnidirectional pace matching")
    : run?.config.pace_motion === "omni" && follow?.pace_active && follow.pace_tracking && !follow.pace_rotation_guard
      ? tr("纵向同步 · 等待末端速度", "Matching forward pace · waiting for endpoint velocity")
    : follow?.pace_active ? tr("步速同步", "Pace matching")
    : tr("跟随运行", "Following");

  return <Card className="pull-chassis" size="small" title={tr("真实底盘 · HopeA3 三轮全向轮", "Physical chassis · HopeA3 omni base")}
    extra={<Space><Tag color="blue">HexMeow · 21 / 22 / 23</Tag><Tag color={running ? "green" : "default"}>{phase}</Tag></Space>}>
    <Space wrap>
      <label>{tr("底盘 CAN-FD", "Chassis CAN-FD")} <Input aria-label="Chassis CAN interface" style={{ width: 160 }}
        value={iface} onChange={e => setIface(e.target.value)} disabled={status?.connected || busy} placeholder="can0 / gs_usb1" /></label>
      <Select aria-label="Chassis data bitrate" value={bitrate} disabled={status?.connected || busy} onChange={setBitrate}
        options={[1_000_000, 2_000_000, 5_000_000].map(value => ({ value, label: `CAN FD 1M / ${value / 1e6}M` }))} />
      <Button disabled={busy || unavailable || !iface.trim()} loading={busy} onClick={() => void action(() =>
        status?.connected ? api.chassisDisconnect() : api.chassisConnect(iface, bitrate))}>
        {status?.connected ? tr("断开底盘", "Disconnect chassis") : tr("连接底盘", "Connect chassis")}
      </Button>
      <Button danger type="primary" loading={stopping} onClick={() => void stop()}>{tr("立即停机 / Esc", "STOP / Esc")}</Button>
    </Space>
    <Typography.Paragraph type="secondary" style={{ marginTop: 8 }}>
      {tr("底盘与拉线使用不同 CAN 接口。先启动拉线并完成归零，再启动真机跟随。",
        "Use separate CAN interfaces for chassis and cord. Home the cord before starting physical follow.")}
    </Typography.Paragraph>
    {unavailable && <Alert type="error" showIcon message={tr("运行状态未知，可按立即停机重试", "Run status unknown; STOP remains available")} />}
    {run?.fault && <Alert type="error" showIcon message={run.fault}
      description={tr("先点击立即停机确认，再重新启动；拉线重归零或反馈中断后不会自动恢复底盘。", "Confirm STOP before restarting. Rehoming or lost feedback never resumes the base automatically.")} />}
    {!config && <Button onClick={() => void action(async () => setConfig(await api.chassisDefaultConfig()))}>{tr("加载底盘默认参数", "Load chassis defaults")}</Button>}
    {config && <>
      <Space wrap style={{ margin: "12px 0" }}>
        <Select aria-label="Follow input source" value={config.source} disabled={structuralLocked}
          onChange={value => set("source", value)} options={[
            { value: "live", label: tr("实时设备", "Live device") }, { value: "manual", label: tr("手动测试", "Manual test") },
          ]} />
        <Select aria-label="Follow mode" value={config.mode} disabled={structuralLocked}
          onChange={value => set("mode", value)} options={[
            { value: "behind", label: tr("身后跟随", "Behind follow") }, { value: "ahead", label: tr("身前引导", "Ahead guidance") },
          ]} />
        {config.source === "live" && config.mode === "behind" &&
          <Select aria-label="Pace motion mode" value={config.pace_motion ?? "steering"} disabled={structuralLocked || !config.follow.pace.enabled}
            onChange={value => set("pace_motion", value)} options={[
              { value: "steering", label: tr("纵向步速同步（原模式）", "Longitudinal pace matching (original)") },
              { value: "omni", label: tr("全向步速同步", "Omnidirectional pace matching") },
            ]} />}
        <Select aria-label="Cord exit side" value={config.cord_exit_side} disabled={structuralLocked}
          onChange={value => set("cord_exit_side", value)} options={[
            { value: "driven_axle", label: tr("前端出线", "Front cord exit") }, { value: "passive_axle", label: tr("后端出线", "Rear cord exit") },
          ]} />
        <Checkbox checked={config.direction_inverted} disabled={structuralLocked}
          onChange={event => set("direction_inverted", event.target.checked)}>{tr("左右方向反相", "Invert left/right")}</Checkbox>
      </Space>
      {config.source === "live" && !pullReady && <Alert type="info" showIcon message={tr("等待拉线归零完成及有效方向编码器反馈", "Waiting for homed cord and valid direction feedback")} />}
      <div className="pull-chassis-grid">
        {([['cordLengthM', tr("线长 m", "Cord length m"), 0, 8, 0.01],
          ['cordSpeedMps', tr("线速 m/s（出线为正）", "Cord speed m/s (payout positive)"), -3, 3, 0.01],
          ['encoderAngleDeg', tr("编码器角度 °", "Encoder angle °"), 0, 359.99, 1]] as const).map(([key, label, min, max, step]) =>
          <label className="pull-chassis-field" key={key}>{label}<InputNumber aria-label={label} value={input[key]}
            disabled={config.source !== "manual" || busy} min={min} max={max} step={step}
            onChange={value => { if (value != null) setManual(current => ({ ...current, [key]: value })); }} /></label>)}
      </div>
      {config.source === "manual" && <Typography.Paragraph type="secondary">
        {tr("编辑输入只修改草稿。使能手动台架后，点击“应用手动输入”才执行，保持到下一次应用、释放或停机。", "Edits are drafts. Arm the bench, then explicitly apply manual input; values hold until updated, released or stopped.")}
      </Typography.Paragraph>}
      <Space wrap style={{ margin: "12px 0" }}>
        <Button type="primary" loading={busy} disabled={!status?.connected || ownsSession || !ready || unavailable || stopping}
          onClick={() => void action(() => api.chassisStart(config))}>
          {config.source === "manual" ? tr("使能手动台架", "Arm manual bench") : tr("启动真机跟随", "Start physical follow")}
        </Button>
        <Button disabled={busy || unavailable} onClick={() => void action(async () => {
          if (running) await api.chassisSetConfig(config);
          await api.chassisSettingsSave(config);
          message.success(tr("底盘参数已保存", "Chassis parameters saved"));
        })}>{running ? tr("应用并保存参数", "Apply and save parameters") : tr("保存配置", "Save configuration")}</Button>
        <Button disabled={!run || busy} onClick={() => { if (run) { setConfig(run.config); setManual(run.input); } }}>{tr("读取运行参数", "Read run parameters")}</Button>
        <Button disabled={ownsSession || busy} onClick={() => void action(async () => setConfig(await api.chassisDefaultConfig()))}>{tr("恢复默认草稿", "Reset draft defaults")}</Button>
        {config.source === "manual" && <>
          <Button disabled={!running || busy || unavailable} onClick={() => void action(() => api.chassisSetManual(true, manual))}>{tr("应用手动输入", "Apply manual input")}</Button>
          <Button disabled={!running || busy} onClick={() => void action(() => api.chassisSetManual(false, manual))}>{tr("释放手动输入", "Release manual input")}</Button>
        </>}
      </Space>
      <div className="pull-chassis-grid">
        {config.mode === "behind" ? <>
          {field("behindStartCordLengthM", tr("启动最短线长 m", "Minimum launch length m"), 0, config.follow.behindTargetCordLengthM - 0.01)}
          {field("behindTargetCordLengthM", tr("启动区间分界 m", "Launch band boundary m"), config.follow.behindStartCordLengthM + 0.01, config.follow.behindStopCordLengthM - 0.01)}
          {field("behindStopCordLengthM", tr("停车 / 回放门限 m", "Stop / retraction threshold m"), config.follow.behindTargetCordLengthM + 0.01, config.follow.behindAheadSwitchCordLengthM - 0.01)}
          {field("behindAheadSwitchCordLengthM", tr("切换身前线长 m", "Ahead switch length m"), config.follow.behindStopCordLengthM + 0.01, 8)}
        </> : <>
          {field("startCordLengthM", tr("身前启动线长 m", "Ahead start length m"), 0, 8)}
          {field("lengthGainPerS", tr("线长速度增益 /s", "Length gain /s"), 0, 5, 0.1)}
          {field("cordSpeedFeedforward", tr("线速前馈", "Cord speed feedforward"), 0, 2, 0.05)}
        </>}
        {field("maxSpeedMps", tr("最高跟随速度 m/s", "Maximum follow speed m/s"), 0, MAX_FOLLOW_MPS, 0.05)}
        {field("maxReverseSpeedMps", tr("最大负速度 m/s", "Maximum negative speed m/s"), 0, MAX_FOLLOW_MPS, 0.05)}
        {field("encoderCenterDeg", tr("编码器中心 °", "Encoder centre °"), 0, 359.99, 1)}
        {field("directionDeadbandDeg", tr("转向死区 °", "Turning deadband °"), 0, 45, 1)}
        {field("smoothingMs", tr("平滑时间 ms", "Smoothing ms"), 0, 2000, 10)}
      </div>
      <Card size="small" title={tr("回收超速制动", "Retraction speed braking")} style={{ marginBottom: 12 }}>
        <Space wrap align="end">
          <Checkbox aria-label={cordMode === "a1_current" ? "Use cord brake reference" : "Use retract target speed"} checked={config.retract_brake_speed_mps == null}
            disabled={busy || (config.retract_brake_speed_mps == null && retractTargetSpeedMps == null)}
            onChange={event => set("retract_brake_speed_mps", event.target.checked ? null : retractTargetSpeedMps)}>
            {cordMode === "a1_current" ? tr("使用拉线模块的制动参考速度", "Use cord brake reference") : tr("使用收线目标速度", "Use retract target speed")}
          </Checkbox>
          <label className="pull-chassis-field">{tr("回收制动速度 m/s", "Retraction brake threshold m/s")}
            <InputNumber aria-label="Retraction brake threshold m/s" min={0.01} step={0.05}
              disabled={busy || config.retract_brake_speed_mps == null}
              value={config.retract_brake_speed_mps ?? retractTargetSpeedMps}
              onChange={value => { if (value != null) set("retract_brake_speed_mps", value); }} />
          </label>
        </Space>
        <Typography.Paragraph type="secondary" style={{ marginTop: 8, marginBottom: 0 }}>
          {tr("仅实时设备跟随生效。实测收线速率达到阈值时，以当前各项减速度的最大值制动，并停止转向。停稳且收线速率降至阈值的 95% 以下后恢复跟随。默认采用拉线模块已应用的速度设置，取消勾选可自定义。",
            "During live follow, retraction at or above the threshold brakes the chassis at the largest configured deceleration and stops turning. Follow resumes once the chassis stops and retraction falls below 95% of the threshold. The default uses the cord module’s applied speed setting. Uncheck to set a separate threshold.")}
        </Typography.Paragraph>
      </Card>
      {config.mode === "behind" && <Space direction="vertical" style={{ width: "100%" }} size={12}>
        <PaceFollowControls value={config.follow.pace} omni={config.source === "live" && config.pace_motion === "omni"}
          stopLengthM={Math.max(config.follow.behindStopCordLengthM, config.follow.pace.targetLengthM + config.follow.pace.stopMarginM)}
          onChange={value => setFollow("pace", value)} />
        <BehindSpeedControls value={config.follow.behindSpeed} paceEnabled={config.follow.pace.enabled}
          maxSpeedMps={Math.min(config.max_linear_mps, config.follow.maxSpeedMps)} wheelGeometry={{ wheel_diameter_m: 0.154, gear_ratio: 1 }}
          onChange={value => setFollow("behindSpeed", value)} />
      </Space>}
      <Collapse style={{ marginTop: 12 }} items={[{ key: "base", label: tr("HopeA3 限幅与电机参数", "HopeA3 limits and motor parameters"), children: <>
        <div className="pull-chassis-grid">
          {([['max_linear_mps', tr("底盘线速度上限 m/s", "Base linear limit m/s"), 0, MAX_FOLLOW_MPS, 0.05],
            ['max_angular_rad_s', tr("底盘角速度上限 rad/s", "Base angular limit rad/s"), 0, 3, 0.1],
            ['acceleration_mps2', tr("基础加速度 m/s²", "Base acceleration m/s²"), 0.01, 4, 0.05],
            ['turn_gain', tr("转向增益 /m", "Turning gain /m"), 0, 10, 0.1]] as const).map(([key, label, min, max, step]) =>
            <label className="pull-chassis-field" key={key}>{label}<InputNumber aria-label={label} disabled={busy} value={config[key]}
              min={min} max={max} step={step} onChange={value => { if (value != null) set(key, value); }} /></label>)}
          {[0, 1, 2].map(index => <div key={index}>
            <label className="pull-chassis-field">{21 + index} · KD (Nm·s/rad)<InputNumber min={0} step={0.05} disabled={busy} value={config.kd_si[index]}
              onChange={value => { const next = [...config.kd_si] as [number, number, number]; next[index] = value ?? 0; set("kd_si", next); }} /></label>
            <label className="pull-chassis-field">{21 + index} · {tr("最大力矩 ‰", "Max torque ‰")}<InputNumber min={0} max={1000} precision={0} disabled={busy}
              value={config.max_torque_permille[index]} onChange={value => { const next = [...config.max_torque_permille] as [number, number, number]; next[index] = value ?? 0; set("max_torque_permille", next); }} /></label>
          </div>)}
        </div>
      </> }]} />
    </>}
    {run && <>
      <Descriptions size="small" bordered column={{ xs: 1, sm: 2, lg: 3 }} style={{ marginTop: 12 }}>
        <Descriptions.Item label={tr("运行阶段", "Phase")}>{phase}</Descriptions.Item>
        <Descriptions.Item label={tr("估计步速", "Estimated pace")}>{fmt(follow?.pace_forward_speed_mps)} m/s</Descriptions.Item>
        <Descriptions.Item label={tr("距离修正", "Gap correction")}>{fmt(follow?.pace_correction_mps)} m/s</Descriptions.Item>
        <Descriptions.Item label={tr("指令前进 / 左移 / 转向", "Forward / left / yaw command")}>{fmt(feedback?.cmd_vx)} m/s · {fmt(feedback?.cmd_vy)} m/s · {fmt(feedback?.cmd_wz)} rad/s</Descriptions.Item>
        <Descriptions.Item label={tr("实测前进 / 左移 / 转向", "Measured forward / left / yaw")}>{fmt(feedback?.meas_vx)} m/s · {fmt(feedback?.meas_vy)} m/s · {fmt(feedback?.meas_wz)} rad/s</Descriptions.Item>
        <Descriptions.Item label={tr("运行长线保护", "Running stop length")}>{fmt(follow?.pace_stop_length_m)} m</Descriptions.Item>
        <Descriptions.Item label={tr("运行回收制动阈值", "Running retraction brake threshold")}>{fmt(run.retract_brake_speed_mps)} m/s</Descriptions.Item>
        <Descriptions.Item label={tr("回收制动减速度", "Retraction brake deceleration")}>{fmt(run.retract_brake_speed_mps == null ? null : run.retract_brake_deceleration_mps2)} m/s²</Descriptions.Item>
      </Descriptions>
      <BasePoseViewer connected={running} poseX={feedback?.pose_x ?? 0} poseY={feedback?.pose_y ?? 0} theta={feedback?.pose_theta ?? 0}
        vx={feedback?.meas_vx ?? 0} vy={feedback?.meas_vy ?? 0} wz={feedback?.meas_wz ?? 0} />
      <Table size="small" pagination={false} rowKey="node_id" dataSource={feedback?.motors ?? []} scroll={{ x: 650 }} columns={[
        { title: "ID", dataIndex: "node_id" },
        { title: tr("状态", "Status"), render: (_, motor) => motor.error ?? (motor.enabled ? tr("已使能", "Enabled") : tr("失能", "Disabled")) },
        { title: tr("目标 rev/s", "Target rev/s"), render: (_, motor) => fmt(motor.target_rev_per_s) },
        { title: tr("反馈 rev/s", "Actual rev/s"), render: (_, motor) => fmt(motor.velocity_rev_per_s) },
        { title: tr("力矩 Nm", "Torque Nm"), render: (_, motor) => fmt(motor.torque_nm) },
      ]} />
    </>}
  </Card>;
}

function fmt(value: number | null | undefined) { return value == null || !Number.isFinite(value) ? "—" : value.toFixed(3); }
