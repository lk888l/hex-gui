import { useEffect, useMemo, useState } from "react";
import { Alert, Button, Card, Descriptions, InputNumber, Popconfirm, Select, Slider, Space, Tag, Typography } from "antd";
import { standardMitApi, type Axis3GravityConfig, type Axis3ModelInfo, type El05State } from "../el05Api";
import { errMsg } from "../api";
import { gravityReadiness } from "../el05Gravity";
import { AXIS3_GRAVITY, axis3Angle, axis3CalibrationMatches, axis3Torque } from "../armAxis3";
import { ArmViewer } from "./ArmViewer";

const PACKAGES = { "总装URDF": "/axis3" };
const DEGREES = 180 / Math.PI;
type Props = {
  state: El05State | null; torqueMaxNm: number; zh: boolean; busy: boolean; stopping: boolean;
  run: (operation: () => Promise<unknown>, success?: string) => Promise<void>;
  stop: () => Promise<void>; setZero: () => Promise<void>;
};

export function Axis3GravityPanel({ state, torqueMaxNm, zh, busy, stopping, run, stop, setZero }: Props) {
  const tr = (cn: string, en: string) => zh ? cn : en;
  const [assembly, setAssembly] = useState<Axis3GravityConfig>({
    gravity_base_m_s2: [0, 0, -AXIS3_GRAVITY], payload_mass_kg: 0, hang_length_m: 0, hook_offset_m: 0,
  });
  const [referenceDegrees, setReferenceDegrees] = useState(0);
  const [direction, setDirection] = useState<1 | -1>(1);
  const [previewDegrees, setPreviewDegrees] = useState(0);
  const [controls, setControls] = useState({
    compensation_scale: 1, damping_nm_s_rad: 0.1, torque_limit_nm: 1.5, max_temperature_c: 60, ramp_s: 2,
  });
  const status = state?.gravity_compensation;
  const reference = status?.reference;
  const active = !!status?.active;
  const running = active ? status.config : null;
  const { compensation_scale: scale, damping_nm_s_rad: damping, torque_limit_nm,
    max_temperature_c: temperatureLimit, ramp_s: ramp } = running ?? controls;
  const torqueLimit = Math.min(torque_limit_nm, torqueMaxNm);
  const { gravity_base_m_s2: [gx, gy, gz], payload_mass_kg, hang_length_m, hook_offset_m } = running?.axis3 ?? assembly;
  // Telemetry supplies a new snapshot each poll. Only request a new model when its inputs change.
  const config = useMemo<Axis3GravityConfig>(() => ({
    gravity_base_m_s2: [gx, gy, gz], payload_mass_kg, hang_length_m, hook_offset_m,
  }), [gx, gy, gz, payload_mass_kg, hang_length_m, hook_offset_m]);
  const gravity = config.gravity_base_m_s2;
  const updateAssembly = (patch: Partial<Axis3GravityConfig>) => setAssembly(previous => ({ ...previous, ...patch }));
  const updateControls = (patch: Partial<typeof controls>) => setControls(previous => ({ ...previous, ...patch }));
  const [loadedModel, setLoadedModel] = useState<{ config: Axis3GravityConfig; info: Axis3ModelInfo } | null>(null);
  const model = loadedModel?.config === config ? loadedModel.info : null;
  const [modelError, setModelError] = useState<string | null>(null);
  const locked = active || busy || stopping;
  const calibration = { joint_angle_rad: referenceDegrees / DEGREES, encoder_direction: direction };

  useEffect(() => {
    let disposed = false;
    setModelError(null);
    void standardMitApi.axis3Model(config).then(next => {
      if (!disposed) setLoadedModel({ config, info: next });
    }).catch(error => { if (!disposed) setModelError(errMsg(error)); });
    return () => { disposed = true; };
  }, [config]);

  const suspendedLoad = useMemo(() => model && payload_mass_kg > 0 ? {
    jointName: model.joint_name, tipJointM: model.gripper_tip_joint_m, hookJointM: model.hook_joint_m, hangLengthM: hang_length_m,
  } : undefined, [model, payload_mass_kg, hang_length_m]);

  const ready = gravityReadiness(state, { busy: locked, validModel: !!model, insufficientTorque: !!model && model.peak_nm * scale > torqueLimit + 1e-9 });
  const angle = ready.fresh ? axis3Angle(reference, ready.position) : null;
  const mapped = axis3CalibrationMatches(reference, calibration);
  const angleValid = model != null && angle != null && angle >= model.lower_rad && angle <= model.upper_rad;
  const referenceValid = model != null && calibration.joint_angle_rad >= model.lower_rad && calibration.joint_angle_rad <= model.upper_rad;
  const predicted = angle != null && model && mapped ? axis3Torque(model, angle, direction, scale) : null;
  const canStart = ready.canStart && mapped && angleValid;
  const number = (value: number | null | undefined, digits = 3) => value == null || !Number.isFinite(value) ? "—" : value.toFixed(digits);
  const start = () => run(() => standardMitApi.startGravityCompensation({
    ...controls, torque_limit_nm: torqueLimit, reference_id: reference!.id, axis3: config,
    // The shared IPC also carries pendulum geometry. J3 uses its assembly config above.
    mass_kg: 0, length_m: 0.18, arm_mass_kg: 0,
  }), tr("第 3 轴零重力已开启", "Axis-3 zero gravity started"));

  return <Space direction="vertical" size={12} style={{ width: "100%" }}>
    <Alert type="info" showIcon message={tr("第 3 轴单关节总成 · URDF 重力补偿", "Axis-3 assembly · URDF gravity compensation")}
      description={tr("模型仅有 Link1 一个转动关节，作为第 3 轴；Link2、Link3 固定随动。固定总成底座，保持其他机械臂轴不动。电机角度/转矩按关节输出端 rad/Nm 使用。", "The model has one moving joint, Link1, used as axis 3; Link2 and Link3 move rigidly with it. Fix the assembly base and hold the other arm axes stationary. Motor angle/torque must be joint-output rad/Nm.")} />
    {modelError && <Alert type="error" showIcon message={modelError} />}
    <Card size="small" title={tr("1 · 安装方向与悬吊负载", "1 · Mounting and suspended load")}>
      <Typography.Paragraph type="secondary">{tr("默认 base_link 的 +Z 竖直向上；斜装时填写底座坐标系中的重力向量，模长为 9.80665 m/s²。红色箭头表示重力方向。", "Default: base_link +Z points up. For a tilted mount, enter gravity in the base frame with magnitude 9.80665 m/s². The red arrow shows gravity.")}</Typography.Paragraph>
      <Space wrap>{gravity.map((value, i) => <Field key={i} label={`g${"XYZ"[i]} (m/s²)`} value={value} min={-AXIS3_GRAVITY} max={AXIS3_GRAVITY} step={0.01} disabled={locked}
        onChange={v => updateAssembly({ gravity_base_m_s2: gravity.map((g, j) => i === j ? v : g) as [number, number, number] })} />)}</Space>
      <div className="el05-target__fields" style={{ marginTop: 12 }}>
        <Field label={tr("砝码质量 (kg)", "Weight mass (kg)")} value={payload_mass_kg} min={0} max={20} step={0.05} disabled={locked}
          onChange={v => updateAssembly({ payload_mass_kg: v })} />
        <Field label={tr("吊点到砝码中心的垂距 (mm)", "Vertical drop to weight center (mm)")} value={hang_length_m * 1000} min={0} max={2000} step={1} disabled={locked}
          onChange={v => updateAssembly({ hang_length_m: v / 1000 })} />
        <Field label={tr("吊点相对指尖的轴向偏移 (mm)", "Hook offset from fingertip (mm)")} value={hook_offset_m * 1000} min={-2000} max={2000} step={1} disabled={locked}
          onChange={v => updateAssembly({ hook_offset_m: v / 1000 })} />
      </div>
      <Typography.Paragraph type="secondary">{tr("绳子系在指尖时，吊点偏移填 0。从指尖沿夹爪向外量为正，向内量为负。静止悬吊时，砝码的重力作用在吊点，绳长只改变砝码高度。质量填 0 为空载。", "For a cord tied at the fingertip, use zero hook offset. Measure outward along the gripper as positive and inward as negative. A freely hanging weight pulls down at the hook. Cord length changes the height of the weight. Set mass to zero for an unloaded test.")}</Typography.Paragraph>
      <Descriptions size="small" column={2} style={{ marginTop: 12 }}>
        <Descriptions.Item label={tr("总运动质量（含砝码）", "Total moving mass with load")}>{number(model?.mass_kg, 6)} kg</Descriptions.Item>
        <Descriptions.Item label={tr("关节", "Joint")}>{model?.joint_name ?? "—"}</Descriptions.Item>
        <Descriptions.Item label={tr("零位总质心（J3 坐标）", "Total COM at URDF zero (J3 frame)")}>{model?.com_joint_m.map(v => v.toFixed(5)).join(", ") ?? "—"} m</Descriptions.Item>
        <Descriptions.Item label={tr("角度范围", "Angle range")}>{number(model ? model.lower_rad * DEGREES : null, 2)}–{number(model ? model.upper_rad * DEGREES : null, 2)}°</Descriptions.Item>
      </Descriptions>
      <ArmViewer q={[(angle ?? previewDegrees / DEGREES)]} previewQ={angle == null || !reference?.axis3 ? null : [reference.axis3.joint_angle_rad]} gravity={gravity} jointNames={["Link1"]}
        urdfUrl="/axis3/assembly.urdf" packages={PACKAGES} suspendedLoad={suspendedLoad} fitToModel />
      {angle == null && <><Typography.Text>{tr("离线姿态预览（不发送电机指令）", "Offline pose preview (no motor commands)")}: {previewDegrees.toFixed(1)}°</Typography.Text>
        <Slider aria-label={tr("预览角度", "Preview angle")} min={0} max={(model?.upper_rad ?? 3.14) * DEGREES} step={0.1} value={previewDegrees} onChange={setPreviewDegrees} /></>}
      <Typography.Text type="secondary">{tr("记录参考前，将实物摆到下面填写的 URDF 角度；预览 0° 即模型零位。已有参考时，绿色半透明模型表示记录姿态。", "Before capture, place the assembly at the URDF angle entered below. Preview 0° shows the model zero. With a reference, the green ghost shows the capture pose.")}</Typography.Text>
    </Card>
    <Card size="small" title={tr("2 · 记录关节角度参考", "2 · Capture joint reference")}>
      <Typography.Paragraph>{tr("托住并停止电机，对照模型摆好姿态、保持静止，再记录参考。默认当前姿态对应 URDF 0°，无需修改编码器；轻推关节核对实物与模型方向，反向时修改方向并重新记录。", "Support and stop the motor, align it with the model and hold it still, then capture. By default the pose is URDF 0° without changing the encoder. Move it gently to check model direction; reverse the direction and recapture if needed.")}</Typography.Paragraph>
      <Space wrap>
        <Field label={tr("当前实物的 URDF 角度 (°)", "Physical URDF angle (°)")} value={referenceDegrees} min={0} max={(model?.upper_rad ?? 3.14) * DEGREES} step={1} disabled={locked} onChange={setReferenceDegrees} />
        <label className="el05-field"><Typography.Text type="secondary">{tr("编码器与关节正方向", "Encoder to joint direction")}</Typography.Text>
          <Select value={direction} disabled={locked} style={{ width: 180 }} onChange={setDirection} options={[{ value: 1, label: tr("同向 (+1)", "Same (+1)") }, { value: -1, label: tr("反向 (−1)", "Opposite (−1)") }]} /></label>
        <Button disabled={!ready.canCapture || !referenceValid} onClick={() => void run(() => standardMitApi.captureAxis3Reference(calibration), tr("关节角度参考已记录", "Joint reference captured"))}>{tr("记录角度参考", "Capture reference")}</Button>
        <Popconfirm title={tr("将当前编码器位置清零？", "Reset encoder position here?")} description={tr("会清除角度参考。清零读回完成后，需重新记录 URDF 角度。", "This invalidates the reference. Capture the URDF angle again after zero readback.")}
          onConfirm={() => setZero()} disabled={locked || !state?.attached || state.enabled || !ready.fresh || !ready.atRest}>
          <Button disabled={locked || !state?.attached || state.enabled || !ready.fresh || !ready.atRest}>{tr("可选：编码器清零", "Optional: zero encoder")}</Button>
        </Popconfirm>
      </Space>
      <Typography.Paragraph type="secondary" style={{ marginTop: 10 }}>{tr("参考仅在本次电机连接与坐标系有效；切换电机、清零或反馈中断后必须重新记录。停止会立即撤去支撑转矩，请先托住机构。", "The reference belongs to this motor connection and coordinate frame. Recapture after selection, zeroing or a feedback gap. Stop removes support torque immediately; support the mechanism first.")}</Typography.Paragraph>
      <Tag color={mapped ? "green" : "default"}>{mapped ? tr("参考已绑定", "Reference bound") : tr("待记录参考", "Capture required")}</Tag>
      <Typography.Text>q = {number(reference?.axis3?.joint_angle_rad)} + ({reference?.axis3?.encoder_direction ?? "±1"}) × (p − {number(reference?.position_rad)}) rad</Typography.Text>
    </Card>
    <Card size="small" title={tr("3 · 零重力测试", "3 · Zero-gravity test")}>
      <Typography.Paragraph type="secondary">{tr("Kp = Kd = 0；后端以实际角度计算支撑转矩并叠加速度阻尼。补偿比例 1 为模型全量补偿，0 仅保留阻尼。", "Kp = Kd = 0. The backend computes support from the measured angle and adds velocity damping. Scale 1 gives full model compensation; 0 gives damping only.")}</Typography.Paragraph>
      <div className="el05-target__fields">
        <Field label={tr("补偿比例", "Compensation scale")} value={scale} min={0} max={1.5} step={0.05} disabled={locked} onChange={v => updateControls({ compensation_scale: v })} />
        <Field label={tr("阻尼 (Nm·s/rad)", "Damping (Nm·s/rad)")} value={damping} min={0} max={5} step={0.01} disabled={locked} onChange={v => updateControls({ damping_nm_s_rad: v })} />
        <Field label={tr("转矩上限 (Nm)", "Torque limit (Nm)")} value={torqueLimit} min={0.01} max={torqueMaxNm} step={0.1} disabled={locked} onChange={v => updateControls({ torque_limit_nm: v })} />
        <Field label={tr("温度上限 (°C)", "Temperature limit (°C)")} value={temperatureLimit} min={20} max={100} step={1} disabled={locked} onChange={v => updateControls({ max_temperature_c: v })} />
        <Field label={tr("补偿渐入 (s)", "Ramp (s)")} value={ramp} min={0.1} max={10} step={0.1} disabled={locked} onChange={v => updateControls({ ramp_s: v })} />
      </div>
      <Typography.Paragraph>{tr("全角度最大重力补偿", "Peak gravity compensation")}: {number(model ? model.peak_nm * scale : null)} Nm</Typography.Paragraph>
      {model && model.peak_nm * scale > torqueLimit + 1e-9 && <Alert type="warning" showIcon message={tr("转矩上限不足，减小补偿比例或提高上限后再启动。", "Torque limit is too low. Reduce the scale or increase the limit before starting.")} />}
      <Space wrap style={{ marginTop: 12 }}>
        <Button type="primary" disabled={!canStart} loading={busy} onClick={() => void start()}>{tr("开启零重力", "Start zero gravity")}</Button>
        <Button danger disabled={!state?.attached} loading={stopping} onClick={() => void stop()}>{tr("停止 / 撤去支撑", "Stop / remove support")}</Button>
        <Tag color={active ? "green" : "default"}>{active ? tr("补偿运行中", "Compensating") : tr("已停止", "Stopped")}</Tag>
      </Space>
      {!canStart && !active && <Typography.Paragraph type="secondary" style={{ marginTop: 10 }}>{!model ? tr("等待有效模型与重力方向。", "Waiting for a valid model and gravity vector.")
        : !state?.attached ? tr("先连接并选择第 3 轴电机。", "Connect and select the axis-3 motor.")
        : !ready.fresh ? tr("等待新鲜反馈。", "Waiting for fresh feedback.")
        : ready.positionSaturated ? tr("编码器位置饱和，请先清零再记录参考。", "Encoder position is saturated; zero it and capture again.")
        : !mapped ? tr("请停止、保持静止并记录当前角度参考。", "Stop, hold still and capture the current angle reference.")
        : !angleValid ? tr("当前关节角度超出 URDF 范围，请检查姿态与零点。", "The joint angle is outside URDF limits; check the pose and reference.")
        : tr("请确认电机已停止、无故障且参数有效。", "Ensure the motor is stopped, fault-free and the parameters are valid.")}</Typography.Paragraph>}
    </Card>
    <Card size="small" title={tr("实时状态", "Live state")}>
      <Descriptions column={2} size="small">
        <Descriptions.Item label={tr("关节角度", "Joint angle")}>{number(angle == null ? null : angle * DEGREES)}°</Descriptions.Item>
        <Descriptions.Item label={tr("速度", "Speed")}>{number(ready.fresh ? state?.velocity_rad_s : null)} rad/s</Descriptions.Item>
        <Descriptions.Item label={tr("重力补偿", "Gravity support")}>{number(active ? status?.gravity_torque_nm : predicted)} Nm</Descriptions.Item>
        <Descriptions.Item label={tr("阻尼转矩", "Damping torque")}>{number(active ? status?.damping_torque_nm : 0)} Nm</Descriptions.Item>
        <Descriptions.Item label={tr("指令转矩", "Command torque")}>{number(active ? status?.command_torque_nm : 0)} Nm {active && status?.limited && <Tag color="warning">{tr("限幅", "Limited")}</Tag>}</Descriptions.Item>
        <Descriptions.Item label={tr("反馈转矩", "Feedback torque")}>{number(ready.fresh ? state?.torque_nm : null)} Nm</Descriptions.Item>
        <Descriptions.Item label={tr("温度", "Temperature")}>{number(ready.fresh ? state?.temperature_c : null, 1)} °C</Descriptions.Item>
        <Descriptions.Item label={tr("运行时长", "Elapsed")}>{number(status?.elapsed_s, 1)} s</Descriptions.Item>
      </Descriptions>
      {status?.reason && <Typography.Paragraph type="secondary">{status.reason}</Typography.Paragraph>}
    </Card>
  </Space>;
}

function Field({ label, onChange, ...props }: { label: string; value: number; min: number; max: number; step: number; disabled: boolean; onChange: (value: number) => void }) {
  return <label className="el05-field"><Typography.Text type="secondary">{label}</Typography.Text>
    <InputNumber aria-label={label} {...props} onChange={value => { if (value != null && Number.isFinite(value)) onChange(value); }} /></label>;
}
