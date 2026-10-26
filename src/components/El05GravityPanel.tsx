import { useState } from "react";
import { Alert, Button, Card, Descriptions, InputNumber, Popconfirm, Space, Tag, Typography } from "antd";
import { el05Api, type El05GravityCompensationConfig, type El05State } from "../el05Api";
import { pendulumModel } from "../el05Experiment";
import { gravityReadiness, type GravityBlocker } from "../el05Gravity";

type Settings = Omit<El05GravityCompensationConfig, "reference_id">;
const DEFAULT_SETTINGS: Settings = {
  mass_kg: 1,
  length_m: 0.18,
  arm_mass_kg: 0,
  compensation_scale: 1,
  damping_nm_s_rad: 0.1,
  torque_limit_nm: 1.8,
  max_velocity_rad_s: 3,
  max_temperature_c: 70,
  ramp_s: 1,
};

export function El05GravityPanel({ state, zh, busy, stopping, run, stop, setZero }: {
  state: El05State | null;
  zh: boolean;
  busy: boolean;
  stopping: boolean;
  run: (operation: () => Promise<unknown>, success?: string) => Promise<void>;
  stop: () => Promise<void>;
  setZero: () => Promise<void>;
}) {
  const tr = (cn: string, en: string) => zh ? cn : en;
  const [settings, setSettings] = useState<Settings>(DEFAULT_SETTINGS);
  const status = state?.gravity_compensation;
  const active = !!status?.active;
  const experimentActive = !!state?.experiment.active;
  const locked = busy || stopping || active || experimentActive;
  const reference = status?.reference;
  // The active config is frozen by the backend. Keep that config visible if
  // this panel is remounted while the controller is still running.
  const shown = active && status?.config ? status.config : settings;
  const model = pendulumModel(shown.mass_kg, shown.length_m, shown.arm_mass_kg);
  const peakTorque = model.gravityNm * shown.compensation_scale;
  const insufficientTorque = peakTorque > shown.torque_limit_nm + 1e-9;
  const validModel = model.gravityNm > 0;
  const { fresh, position, positionSaturated, canCapture, canStart, captureBlocker, startBlocker } = gravityReadiness(state,
    { busy: busy || stopping, validModel, insufficientTorque });
  const angle = fresh && reference && position != null && Number.isFinite(reference.position_rad)
    ? position - reference.position_rad : null;
  const blockerText = (blocker: GravityBlocker | null): string | null => {
    if (!blocker) return null;
    const messages: Record<GravityBlocker, [string, string]> = {
      connect: ["请先连接 CAN。", "Connect CAN first."],
      select: ["请先选择电机。", "Select a motor first."],
      feedback: ["等待新鲜反馈（需在 150 ms 内）。", "Waiting for fresh feedback (within 150 ms)."],
      position: ["尚未收到有效编码器位置。", "No valid encoder position has arrived."],
      position_saturated: ["位置量程已满；请先停止并重置位置零点，再记录下垂参考。", "Position range is saturated. Stop and reset the position zero, then capture the downward reference."],
      fault: ["请先处理电机故障或预警。", "Resolve the motor fault or warning first."],
      busy: ["正在处理操作，请稍候。", "An operation is in progress."],
      experiment: ["请先停止摆锤性能测试。", "Stop the pendulum experiment first."],
      active: ["零重力保持正在运行。", "Zero-gravity hold is running."],
      enabled: ["请先停止手动控制。", "Stop manual control first."],
      moving: ["等待摆锤静止（速度需不超过 0.1 rad/s）。", "Wait for the pendulum to rest (speed at most 0.1 rad/s)."],
      reference: ["请先记录竖直下垂参考。", "Capture the vertically downward reference first."],
      reference_motor: ["当前参考属于其他电机，请重新记录。", "The reference belongs to another motor; capture it again."],
      model: ["请输入非零末端质量或力臂质量。", "Enter a nonzero end mass or arm mass."],
      torque: ["所需补偿超过转矩上限，请调整负载参数或上限。", "Required compensation exceeds the torque limit; adjust the load or limit."],
    };
    return messages[blocker][zh ? 0 : 1];
  };
  const predictedTorque = angle == null ? null : peakTorque * Math.sin(angle);
  const field = (label: string, key: keyof Settings, min: number, max: number, step: number, factor = 1) => (
    <label className="el05-field">
      <Typography.Text type="secondary">{label}</Typography.Text>
      <InputNumber disabled={locked} value={shown[key] * factor} min={min} max={max} step={step}
        onChange={value => { if (value != null) setSettings(previous => ({ ...previous, [key]: value / factor })); }} />
    </label>
  );
  const capture = () => run(() => el05Api.captureGravityReference(), tr("下垂参考已记录", "Downward reference captured"));
  const start = () => {
    if (!reference || !canStart) return;
    return run(() => el05Api.startGravityCompensation({ ...settings, reference_id: reference.id }),
      tr("零重力保持已开启", "Zero-gravity hold started"));
  };

  return <div className="el05-gravity">
    <Card size="small" title={tr("摆锤零重力保持", "Pendulum zero-gravity hold")}
      extra={<Tag color={active ? "processing" : "default"}>{active ? tr("补偿中", "Compensating") : tr("已停止", "Stopped")}</Tag>}>
      <Typography.Paragraph>{tr(
        "根据实时摆角抵消摆锤自重，可手动拖动到其他角度，松手后通过阻尼与摩擦逐渐停留。补偿连续运行，点击停止结束。",
        "Continuously offsets the pendulum's weight using its measured angle. Move it by hand; damping and friction let it settle after release. Compensation runs until stopped.",
      )}</Typography.Paragraph>
      <Typography.Paragraph type="secondary">{tr(
        "先输入末端负重与力臂参数，将摆锤自然垂直下垂，等待静止后记录下垂参考，再开启保持。补偿比例 1 表示完全抵消模型中的重力；增大阻尼可减小松手后的滑动。模型误差可能导致缓慢漂移。",
        "Enter the load and arm geometry, let the pendulum hang vertically down, wait for rest, then capture the downward reference and start. A compensation scale of 1 cancels the model's gravity. Increase damping to reduce motion after release. Model errors can cause slow drift.",
      )}</Typography.Paragraph>
      {experimentActive && <Alert type="info" showIcon message={tr("请先停止摆锤性能测试，再开启零重力保持。", "Stop the pendulum experiment before starting zero-gravity hold.")} />}
      {positionSaturated && <Alert type="warning" showIcon
        message={tr("位置反馈已到 ±12.57 rad 量程端点", "Position feedback has reached the ±12.57 rad endpoint")}
        description={tr("端点读数不能用于计算真实摆角。先停止电机并重置位置零点，然后重新记录竖直下垂参考；此操作会修改电机零位并使旧参考失效。", "The saturated reading cannot establish the pendulum angle. Stop and reset the position zero, then capture vertically down again. This changes the motor zero and invalidates the previous reference.")} />}
      {state?.protocol === "StandardMit" && <Space style={{ margin: "12px 0" }} wrap>
        <Popconfirm title={tr("停止电机并将当前位置设为零点？", "Stop the motor and set the current position to zero?")}
          description={tr("这会修改电机位置零位，并使旧下垂参考失效；完成后需要重新记录参考。", "This changes the motor position zero and invalidates the previous downward reference. Capture a new reference afterward.")}
          okText={tr("停止并置零", "Stop & reset zero")} cancelText={tr("取消", "Cancel")}
          disabled={!state.attached || locked} onConfirm={setZero}>
          <Button disabled={!state.attached || locked} loading={busy}>{tr("停止并重置位置零点", "Stop & reset position zero")}</Button>
        </Popconfirm>
      </Space>}
      <div className="el05-target__fields">
        {field(tr("末端质量 (kg)", "End mass (kg)"), "mass_kg", 0, 20, 0.05)}
        {field(tr("轴至负重重心 (mm)", "Axis to load CoM (mm)"), "length_m", 1, 2000, 1, 1000)}
        {field(tr("均匀力臂质量 (kg)", "Uniform arm mass (kg)"), "arm_mass_kg", 0, 20, 0.01)}
        {field(tr("重力补偿比例", "Compensation scale"), "compensation_scale", 0, 1.5, 0.01)}
        {field(tr("阻尼 (Nm·s/rad)", "Damping (Nm·s/rad)"), "damping_nm_s_rad", 0, 5, 0.01)}
      </div>
      <div className="el05-target__fields">
        {field(tr("输出转矩上限 (Nm)", "Output torque limit (Nm)"), "torque_limit_nm", 0.01, 6, 0.1)}
        {field(tr("超速停机阈值 (rad/s)", "Overspeed stop (rad/s)"), "max_velocity_rad_s", 0.1, 20, 0.1)}
        {field(tr("超温停机阈值 (°C)", "Temperature stop (°C)"), "max_temperature_c", 20, 100, 1)}
        {field(tr("重力补偿渐入时间 (s)", "Gravity ramp-in (s)"), "ramp_s", 0.1, 10, 0.1)}
      </div>
      <Space wrap className="el05-gravity-model">
        <Tag color={insufficientTorque ? "error" : "blue"}>{tr("最大重力补偿", "Peak gravity compensation")} {peakTorque.toFixed(3)} Nm</Tag>
        <Typography.Text type="secondary">τ = α · (m + mₐ/2) · g · L · sin(θ) − b · ω</Typography.Text>
      </Space>
      {insufficientTorque && <Alert type="warning" showIcon message={tr(
        "负载所需的最大补偿超过转矩上限。请减小负载或补偿比例，或按电机和机构能力调整上限后再开启。",
        "Peak compensation exceeds the torque limit. Reduce the load or compensation scale, or adjust the limit within the motor and mechanism's capabilities before starting.",
      )} />}
      {!validModel && <Alert type="info" showIcon message={tr("请输入非零末端质量或力臂质量。", "Enter a nonzero end mass or arm mass.")} />}
      <div className="el05-gravity-reference">
        <Space wrap>
          <Button disabled={!canCapture} loading={busy} onClick={() => void capture()}>{tr("将当前角记为竖直下垂", "Capture current angle as vertically down")}</Button>
          <Tag color={reference ? "green" : "default"}>{reference ? tr("下垂参考已测定", "Downward reference captured") : tr("需要测定下垂参考", "Capture a downward reference")}</Tag>
          {reference && <Typography.Text code>θ₀ = {reference.position_rad.toFixed(4)} rad</Typography.Text>}
        </Space>
        {captureBlocker && <Typography.Paragraph type="secondary" style={{ margin: "8px 0" }}>{tr("暂不能记录：", "Cannot capture yet: ")}{blockerText(captureBlocker)}</Typography.Paragraph>}
        <Typography.Paragraph type="secondary" className="el05-experiment-notes">{tr(
          "停机且摆锤静止时才能记录参考；记录参考不修改电机机械零点。重连、更换电机、修改机械零位或反馈中断后需要重新记录。",
          "Capture requires a stopped motor and a stationary pendulum. Capturing the reference preserves the motor's mechanical zero. Capture again after reconnecting, changing motors, changing the mechanical zero, or losing feedback.",
        )}</Typography.Paragraph>
      </div>
      {state?.enabled && !active && !experimentActive && <Alert type="info" showIcon message={tr("请先停止手动控制，再记录参考或开启保持。", "Stop manual control before capturing the reference or starting hold.")} />}
      <div className="el05-experiment-actions">
        <Button type="primary" size="large" disabled={!canStart} loading={busy} onClick={() => void start()}>{tr("开启零重力保持", "Start zero-gravity hold")}</Button>
        <Button type="primary" danger size="large" disabled={!state?.attached} loading={stopping} onClick={() => void stop()}>{tr("停止补偿和电机", "Stop compensation & motor")}</Button>
        <Typography.Text type="secondary">{tr("停止后支撑转矩立即撤去，摆锤将受重力作用。", "Stopping removes support torque immediately; the pendulum will move under gravity.")}</Typography.Text>
      </div>
      {startBlocker && <Typography.Paragraph type="secondary" style={{ margin: "8px 0" }}>{tr("暂不能开启：", "Cannot start yet: ")}{blockerText(startBlocker)}</Typography.Paragraph>}
    </Card>
    <Card size="small" title={tr("实时补偿状态", "Live compensation")}>
      <div className="el05-gravity-live">
        <PendulumView angle={angle} active={active} zh={zh} />
        <div>
          <div className="el05-metrics">
            <Reading label={tr("编码器累计位置（本次连接）", "Unwrapped encoder position (this connection)")} value={fresh ? position : null} unit="rad" />
            <Reading label={tr("相对下垂角", "Angle from down")} value={angle == null ? null : angle * 180 / Math.PI} unit="°"
              placeholder={!reference ? tr("待记录下垂参考", "Capture downward reference") : undefined} />
            <Reading label={tr("当前速度", "Velocity")} value={fresh ? state?.velocity_rad_s : null} unit="rad/s" />
            <Reading label={tr("指令转矩", "Command torque")} value={active ? status.command_torque_nm : 0} unit="Nm" />
            <Reading label={tr("重力补偿分量", "Gravity component")} value={active ? status.gravity_torque_nm : predictedTorque} unit="Nm" />
            <Reading label={tr("阻尼分量", "Damping component")} value={active ? status.damping_torque_nm : 0} unit="Nm" />
            <Reading label={tr("电机温度", "Motor temperature")} value={fresh ? state?.temperature_c : null} unit="°C" />
          </div>
          <Descriptions size="small" column={1} className="el05-gravity-status">
            <Descriptions.Item label={tr("运行时间", "Elapsed")}>{(status?.elapsed_s ?? 0).toFixed(1)} s</Descriptions.Item>
            <Descriptions.Item label={tr("输出状态", "Output status")}>{active ? status.limited
              ? <Tag color="warning">{tr("转矩已限幅", "Torque limited")}</Tag>
              : tr("补偿中", "Compensating") : status?.reason ?? tr("尚未开启", "Not started")}</Descriptions.Item>
          </Descriptions>
        </div>
      </div>
    </Card>
  </div>;
}

function Reading({ label, value, unit, placeholder }: { label: string; value: number | null | undefined; unit: string; placeholder?: string }) {
  return <div className="el05-metric"><Typography.Text type="secondary">{label}</Typography.Text>
    <div className="el05-metric__value el05-number">{value == null || !Number.isFinite(value) ? placeholder ? <Typography.Text type="secondary">{placeholder}</Typography.Text> : "—" : value.toFixed(3)}{(!placeholder || value != null) && <span className="el05-metric__unit">{unit}</span>}</div>
  </div>;
}

function PendulumView({ angle, active, zh }: { angle: number | null; active: boolean; zh: boolean }) {
  const x = 120 + 80 * Math.sin(angle ?? 0);
  const y = 105 + 80 * Math.cos(angle ?? 0);
  return <svg className="el05-gravity-pendulum" viewBox="0 0 240 240" role="img"
    aria-label={zh ? "实时摆锤角度示意图" : "Live pendulum angle diagram"}>
    <circle cx="120" cy="105" r="80" fill="none" stroke="currentColor" opacity="0.12" />
    <path d="M120 105V193" stroke="currentColor" strokeDasharray="4 5" opacity="0.4" />
    <path d="M90 100H150" stroke="currentColor" opacity="0.5" strokeWidth="4" />
    <g opacity={angle == null ? 0.25 : 1}>
      <path d={`M120 105L${x} ${y}`} stroke={active ? "#52c41a" : "#4f8cff"} strokeWidth="5" strokeLinecap="round" />
      <circle cx={x} cy={y} r="12" fill={active ? "#52c41a" : "#4f8cff"} />
    </g>
    <circle cx="120" cy="105" r="6" fill="currentColor" />
    <text x="120" y="222" textAnchor="middle" fill="currentColor" fontSize="12" opacity="0.65">{zh ? "虚线为竖直下垂方向" : "Dashed line: vertically down"}</text>
  </svg>;
}
