import { Card, Checkbox, Collapse, InputNumber, Typography } from "antd";
import type { PaceFollowParameters } from "../pullChassisTypes";
import { useI18n } from "../i18n";

export function PaceFollowControls({ value, stopLengthM, onChange }: {
  value: PaceFollowParameters;
  stopLengthM: number;
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
      {tr("保留原发车动作。发车后让绳长回到设定值附近：偏短时车稍快，偏长时车稍慢；进入允许浮动范围后维持步速同步。步速由动力轴实测速率和收放线计算，转向轴独立跟随拉线角度。人加快、收线就提速，人落后、出线就减速，人停止则停车。",
        "Keep the existing launch gesture. A short gap adds a small speed correction and a long gap reduces speed. Within tolerance, match pace using drive wheel speed and cord motion. The steering axis follows cord angle independently. Retraction speeds up, payout slows down, and a stationary person stops the cart.")}
    </Typography.Paragraph>
    {controls(basic)}
    <Typography.Paragraph type="secondary" className="simulator-card-hint">
      {tr("动力轴跟随步速请求，转向轴围绕 180° 安装中心跟随拉线角差，支持连续多圈；转向限速按实测行进速度在舵轮面板设置的两个 rad/s 端点之间线性映射。",
        "The drive axis follows the pace request. Steering follows cord angle around its 180° mounting center with continuous rotation. Its speed limit follows measured travel speed between the two rad/s endpoints in the module panel.")}
    </Typography.Paragraph>
    <Typography.Paragraph type="secondary" className="simulator-card-hint">
      {tr(`发车后长线保护为 ${stopLengthM.toFixed(2)} m，取“保持线长＋保护余量”和原停止线长的较大值。运行中达到“切换身前线长”时，先制动至停稳，再按当前线长进入身前引导；该切换优先于步速同步和长线停车。停住时不为追求精确线长自行移动。修改后应用参数。`,
        `Running stop guard: ${stopLengthM.toFixed(2)} m, the larger of follow gap plus margin and the original stop length. Reaching the Ahead switch length during a run brakes to a confirmed stop, then enters Ahead guidance according to the current cord length. This takes priority over pace matching and long-cord stops. The cart does not move a stationary person to chase an exact gap. Apply edited parameters.`)}
    </Typography.Paragraph>
    <Collapse size="small" items={[{ key: "response", label: tr("跟随响应与保护", "Response and protection"), children: controls(advanced) }]} />
  </Card>;
}
