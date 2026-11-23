import { Card, Checkbox, Collapse, InputNumber, Typography } from "antd";
import type { PaceFollowParameters } from "../pullChassisTypes";
import { useI18n } from "../i18n";

export function PaceFollowControls({ value, stopLengthM, omni, onChange }: {
  value: PaceFollowParameters;
  stopLengthM: number;
  omni: boolean;
  onChange: (next: PaceFollowParameters) => void;
}) {
  const { lang } = useI18n();
  const tr = (zh: string, en: string) => lang === "zh" ? zh : en;
  type Field = { key: Exclude<keyof PaceFollowParameters, "enabled">;
    label: string; min: number; max: number; step: number };
  const basic: Field[] = [
    { key: "targetLengthM", label: tr("保持线长 m", "Follow gap m"), min: 0.1, max: 8, step: 0.01 },
    { key: "lengthDeadbandM", label: tr("允许浮动 ±m", "Gap tolerance ±m"), min: 0, max: Math.min(0.5, value.targetLengthM, value.stopMarginM), step: 0.01 },
  ];
  const advanced: Field[] = [
    { key: "filterMs", label: tr("步速滤波 ms", "Pace filter ms"), min: 20, max: 2000, step: 10 },
    { key: "lengthGainPerS", label: tr("绳长回调增益 /s", "Gap correction gain /s"), min: 0, max: 2, step: 0.05 },
    { key: "maxCorrectionMps", label: tr("回调速度上限 m/s", "Gap correction limit m/s"), min: 0, max: 0.3, step: 0.01 },
    { key: "accelerationMps2", label: tr("同步加速度上限 m/s²", "Pace acceleration limit m/s²"), min: 0.01, max: 4, step: 0.05 },
    { key: "decelerationMps2", label: tr("同步减速度上限 m/s²", "Pace deceleration limit m/s²"), min: 0.01, max: 4, step: 0.05 },
    { key: "stopMarginM", label: tr("长线保护余量 m", "Long-cord guard margin m"), min: 0.1, max: 4, step: 0.05 },
  ];
  const controls = (fields: Field[]) => <div className="simulator-calibration-row">
    {fields.map(({ key, label, min, max, step }) => <label className="simulator-compact-control" key={key}>
      <span>{label}</span>
      <InputNumber value={value[key]} disabled={!value.enabled} min={min} max={max} step={step}
        onChange={next => { if (next != null) onChange({ ...value, [key]: next }); }} />
    </label>)}
  </div>;
  return <Card size="small" title={tr("绳长保持 · 步速同步", "Gap hold · Pace matching")}
    extra={<Checkbox checked={value.enabled} onChange={e => onChange({ ...value, enabled: e.target.checked })}>
      {tr("启用", "Enable")}
    </Checkbox>}>
    <Typography.Paragraph type="secondary">
      {tr("实时身后跟随沿用收线发车动作。发车后按轮速和收放线速度同步纵向步速。绳长偏短时车稍快，偏长时车稍慢。",
        "During live Behind follow, use the usual retraction gesture to launch. Wheel feedback and cord speed determine forward pace. A short gap adds a small speed correction and a long gap reduces speed.")}
      {" "}{omni
        ? tr("全向模式同时跟随手柄的左右步速，并以接入同步时的侧向位置为基准。车头方向保持，斜走时两个方向可同时跟随。",
            "Omnidirectional mode also matches the grip’s sideways pace and maintains its lateral offset at engagement. The base can move diagonally with zero commanded yaw.")
        : tr("纵向模式下，拉线偏角控制车身转向。", "In longitudinal mode, cord deflection steers the base.")}
    </Typography.Paragraph>
    {controls(basic)}
    <Typography.Paragraph type="secondary" className="simulator-card-hint">
      {tr(`发车后长线保护为 ${stopLengthM.toFixed(2)} m，取“保持线长＋保护余量”和原停止线长的较大值。运行中达到“切换身前线长”时，先制动至停稳，再按当前线长进入身前引导；该切换优先于步速同步和长线停车。停住时不为追求精确线长自行移动。修改后应用参数。`,
        `Running stop guard: ${stopLengthM.toFixed(2)} m, the larger of follow gap plus margin and the original stop length. Reaching the Ahead switch length during a run brakes to a confirmed stop, then enters Ahead guidance according to the current cord length. This takes priority over pace matching and long-cord stops. The cart does not move a stationary person to chase an exact gap. Apply edited parameters.`)}
    </Typography.Paragraph>
    <Collapse size="small" items={[{ key: "response", label: tr("跟随响应与保护", "Response and protection"), children: controls(advanced) }]} />
  </Card>;
}
