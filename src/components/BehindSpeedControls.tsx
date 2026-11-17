import { Card, Checkbox, InputNumber, Typography } from "antd";
import { outputSpeedRadS, type BehindSpeedParameters, type WheelGeometry } from "../pullChassisTypes";
import { useI18n } from "../i18n";

export function BehindSpeedControls({ value, maxSpeedMps, wheelGeometry, paceEnabled, onChange }: {
  value: BehindSpeedParameters;
  paceEnabled: boolean;
  maxSpeedMps: number;
  wheelGeometry: WheelGeometry;
  onChange: (next: BehindSpeedParameters) => void;
}) {
  const { lang } = useI18n();
  const tr = (zh: string, en: string) => lang === "zh" ? zh : en;
  const maxOutputRadS = outputSpeedRadS(maxSpeedMps, wheelGeometry);
  const fields: {
    key: Exclude<keyof BehindSpeedParameters, "enabled" | "adaptiveEnabled" | "payoutStopEnabled">;
    label: string; min: number; max: number; step: number; scale?: number;
  }[] = [
    { key: "payoutSpeedRatio", label: tr("出线巡航 / 最高速度 %", "Payout cruise / speed limit %"), min: 0, max: 100, step: 5, scale: 100 },
    { key: "neutralBoostRatio", label: tr("零线速增加 / 最高速度 %", "Neutral boost / speed limit %"), min: 0, max: 100, step: 5, scale: 100 },
    { key: "retractGain", label: tr("回放增益 (m/s)/(m/s)", "Retraction gain (m/s)/(m/s)"), min: 0, max: 10, step: 0.05 },
    { key: "speedDeadbandMps", label: tr("零线速死区 ±m/s", "Neutral deadband ±m/s"), min: 0, max: 3, step: 0.01 },
    { key: "payoutBlendMps", label: tr("出线过渡宽度 m/s", "Payout transition width m/s"), min: 0.001, max: 3, step: 0.01 },
    { key: "filterMs", label: tr("线速滤波时间 ms", "Cord speed filter ms"), min: 0, max: 5000, step: 10 },
    { key: "payoutAccelerationMps2", label: tr("出线基础加速度 m/s²", "Base payout acceleration m/s²"), min: 0.01, max: 4, step: 0.01 },
    { key: "responseAccelerationMps2", label: tr("回放 / 零线速基础加速度 m/s²", "Base retraction / neutral acceleration m/s²"), min: 0.01, max: 4, step: 0.01 },
    { key: "decelerationMps2", label: tr("普通跟随基础减速度 m/s²", "Cruise base deceleration m/s²"), min: 0.01, max: 4, step: 0.01 },
    { key: "stopDecelerationMps2", label: tr("停止线减速度 m/s²", "Stop-line deceleration m/s²"), min: 0.01, max: 4, step: 0.05 },
    { key: "switchDecelerationMps2", label: tr("切换制动减速度 m/s²", "Ahead switch deceleration m/s²"), min: 0.01, max: 4, step: 0.05 },
    { key: "adaptiveGain", label: tr("跟手响应增益 /s", "Adaptive response gain /s"), min: 0, max: 10, step: 0.1 },
    { key: "adaptiveAccelerationMps2", label: tr("自适应加速度上限 m/s²", "Adaptive acceleration ceiling m/s²"), min: 0.01, max: 4, step: 0.05 },
    { key: "adaptiveDecelerationMps2", label: tr("自适应减速度上限 m/s²", "Adaptive braking ceiling m/s²"), min: 0.01, max: 4, step: 0.05 },
    { key: "payoutStopSpeedMps", label: tr("猛拉停车出线速度 m/s", "Tug stop payout threshold m/s"), min: 0.01, max: 3, step: 0.05 },
    { key: "payoutStopConfirmMs", label: tr("猛拉确认时间 ms", "Tug confirmation ms"), min: 10, max: 1000, step: 10 },
    { key: "payoutStopMinPayoutM", label: tr("猛拉最小出线量 m", "Minimum tug payout m"), min: 0.001, max: 0.5, step: 0.005 },
    { key: "payoutStopDecelerationMps2", label: tr("猛拉停车减速度 m/s²", "Tug stop braking m/s²"), min: 0.01, max: 4, step: 0.05 },
    { key: "pullBackAccelerationMps2", label: tr("停稳回放重启加速度上限 m/s²", "Retraction restart acceleration ceiling m/s²"), min: 0.01, max: 4, step: 0.05 },
    { key: "pullBackAccelerationGain", label: tr("回放速度→重启加速度增益 /s", "Retraction speed → restart acceleration gain /s"), min: 0, max: 10, step: 0.1 },
    { key: "pullBackMinRetractM", label: tr("重启最小回放量 m", "Minimum restart retraction m"), min: 0.001, max: 0.5, step: 0.005 },
    { key: "pullBackLengthGain", label: tr("回放长度→重启速度增益 /s", "Retraction distance → restart speed gain /s"), min: 0, max: 10, step: 0.1 },
    { key: "pullBackDecelerationMps2", label: tr("回放重启制动减速度 m/s²", "Retraction restart braking m/s²"), min: 0.01, max: 4, step: 0.05 },
  ];
  return <Card size="small" title={paceEnabled ? tr("身后跟随 · 停车与回放重启", "Behind follow · Braking and restart") : tr("身后跟随 · 旧版线速调速", "Behind follow · Legacy cord response")}
    extra={!paceEnabled && <Checkbox checked={value.enabled} onChange={e => onChange({ ...value, enabled: e.target.checked })}>
      {tr("启用巡航线速调速", "Enable cruise speed response")}
    </Checkbox>}>
    {!paceEnabled && <Typography.Paragraph>
      <Checkbox checked={value.adaptiveEnabled} disabled={!value.enabled}
        onChange={e => onChange({ ...value, adaptiveEnabled: e.target.checked })}>
        {tr("线速自适应加减速", "Adaptive ramps")}
      </Checkbox>
    </Typography.Paragraph>}
    {paceEnabled ? <Typography.Paragraph type="secondary">
      {tr("步速同步按人在车后计算：收线加速、出线减速，不启用旧版猛拉停车。长线停车仍生效。",
        "Pace matching assumes the person is behind the cart: retraction requests acceleration, payout requests slowing. The legacy tug gesture is inactive; long-cord stops remain active.")}
    </Typography.Paragraph> : <Typography.Paragraph>
      <Checkbox checked={value.payoutStopEnabled}
        onChange={e => onChange({ ...value, payoutStopEnabled: e.target.checked })}>
        {tr("行进中猛拉出线停车", "Stop on a fast payout tug while moving")}
      </Checkbox>
    </Typography.Paragraph>}
    <div className="simulator-calibration-row">
      {fields.filter(({ key }) => !paceEnabled || key.startsWith("pullBack")
        || key === "stopDecelerationMps2" || key === "switchDecelerationMps2" || key === "speedDeadbandMps"
        || key === "responseAccelerationMps2").map(({ key, label, min, max, step, scale = 1 }) =>
        <label className="simulator-compact-control" key={key}>
          <span>{label}</span>
          <InputNumber value={value[key] * scale} disabled={(!value.enabled && !key.startsWith("pullBack") && !key.startsWith("payoutStop")
            && key !== "responseAccelerationMps2" && key !== "speedDeadbandMps"
            && key !== "decelerationMps2" && key !== "stopDecelerationMps2" && key !== "switchDecelerationMps2")
            || (key.startsWith("payoutStop") && !value.payoutStopEnabled)
            || (key.startsWith("adaptive") && !value.adaptiveEnabled)}
            min={min} max={max} step={step}
            onChange={next => { if (next != null) onChange({ ...value, [key]: next / scale }); }} />
        </label>)}
    </div>
    {!paceEnabled && <Typography.Paragraph type="secondary" className="simulator-card-hint">
      {value.enabled ? tr(
        `启动至目标线长内，持续出线时动力输出轴巡航 ${(maxOutputRadS * value.payoutSpeedRatio).toFixed(3)} rad/s；零线速时目标 ${Math.min(maxOutputRadS, maxOutputRadS * (value.payoutSpeedRatio + value.neutralBoostRatio)).toFixed(3)} rad/s；回放越快越提速，上限 ${maxOutputRadS.toFixed(3)} rad/s。`,
        `Between start and desired lengths: sustained payout requests drive output-shaft cruise at ${(maxOutputRadS * value.payoutSpeedRatio).toFixed(3)} rad/s; neutral targets ${Math.min(maxOutputRadS, maxOutputRadS * (value.payoutSpeedRatio + value.neutralBoostRatio)).toFixed(3)} rad/s; faster retraction raises speed up to ${maxOutputRadS.toFixed(3)} rad/s.`,
      ) : tr("已关闭线速调速，使用纯线长速度模型和直行起步加速度，独立制动仍生效。", "Disabled: uses length-only speed targets and linear launch acceleration; independent braking remains active.")}
    </Typography.Paragraph>}
    {!paceEnabled && value.enabled && value.payoutSpeedRatio + value.neutralBoostRatio >= 1 &&
      <Typography.Paragraph type="warning">{tr("巡航比例与零线速增量之和已达到最高速度，零线速附近将饱和。", "Cruise plus neutral boost reaches the speed limit; the legacy response saturates near zero cord speed.")}</Typography.Paragraph>}
    {!paceEnabled && <Typography.Paragraph type="secondary" className="simulator-card-hint">
      {tr(
        "线速正值为出线，负值为回放。超过目标线长后整体减速。自适应加速跟随线速，普通制动还结合速度差。“直行加速度”只限制基础加速，不限制减速；自适应减速度上限低于基础减速度时，以基础减速度为准。降速目标立即响应，提速仍经过滤波。关闭自适应后使用固定加减速。滤波设为 0 可关闭；停止区保留转向。预览显示稳态目标；运行中点击“应用参数”，停机时点击“保存参数”。",
        "Positive cord speed means payout; negative means retraction. Speed tapers beyond the desired length. Adaptive acceleration follows cord motion; cruise braking also uses speed error. Linear acceleration limits base acceleration only. An adaptive braking ceiling below base deceleration cannot weaken that base. Lower targets respond immediately; increases remain filtered. Disable adaptation for fixed ramps; filter 0 disables filtering. Stop zones retain steering. Preview shows steady targets. Apply while running or Save while stopped.",
      )}
    </Typography.Paragraph>}
    <Typography.Paragraph type="secondary" className="simulator-card-hint">
      {tr(
        "停止线与切换制动独立于巡航、自适应开关，默认分别为 1.5、3.0 m/s²。达到停止线后保持制动至停稳；达到身前切换线时，若仍沿身后方向运动，使用切换减速度，且不低于停止线或已触发的猛拉制动。线长回摆不提前解除；指令归零且实测纵向速度不超过 ±0.02 m/s 后，才按当前线长和正常起步加速度恢复运动。缺少有效反馈时继续保持。",
        "Stop-line and switch brakes remain active independently of cruise and adaptive settings; defaults are 1.5 and 3.0 m/s². At the stop line, brake until stopped. At the Ahead switch, remaining Behind motion uses switch braking, at least as strong as stop-line or active tug braking. Cord bounce cannot release it. Motion resumes according to current length and normal launch acceleration only after the command is zero and measured linear speed is within ±0.02 m/s. Missing feedback keeps the brake active.",
      )}
    </Typography.Paragraph>
    {!paceEnabled && <Typography.Paragraph type="secondary" className="simulator-card-hint">
      {tr(
        "旧版猛拉停车独立于巡航调速开关：正常前进时，正线速达到门限、持续达到确认时间且实际净出线达到最小量后制动，保留角度转向。默认 0.5 m/s、40 ms、2 cm，停车减速度 1.2 m/s²。线速归零后仍保持停止；放回目标线长并停稳后恢复巡航。如果动作起点短于目标线长，需放回本次起点。达到身前切换线长后先完成制动，停稳再进入身前引导。运行状态显示实际恢复线长；静态预览不判定猛拉。",
        "Tug stop is independent of cruise speed response. During forward cruise, positive cord speed must meet the threshold for the confirmation time with enough net payout. Defaults: 0.5 m/s, 40 ms, 2 cm, braking at 1.2 m/s². Heading control remains active. Neutral cord speed keeps the stop latched; return to the target length and confirm a physical stop to resume. For a tug starting below the target length, return to its starting length. At the Ahead switch, finish braking before starting Ahead motion. Runtime shows the return length; static preview does not detect tugs.",
      )}
    </Typography.Paragraph>}
    <Typography.Paragraph type="secondary" className="simulator-card-hint">
      {tr(
        "停稳后，负线速回放需连续确认 80 ms 且达到最小回放量，才恢复前进。重启目标同时受本次净回放长度和当前回放速度限制，不再叠加长线误差。回放停止或改为出线时立即请求制动；下一次动作重新计算回放量并确认停稳。默认最小回放量 0.01 m、长度增益 2 /s、重启制动 0.8 m/s²。降低长度增益可减小短动作的速度，增大最小回放量可忽略小抖动；实际停车距离仍受负载影响。回到目标线长恢复普通巡航，超过远端切换线长仍按原身前规则运行。原重启加速度上限与线速增益继续生效，巡航开关不关闭本分支。",
        "Restart requires a measured stop, 80 ms of negative cord speed and minimum net retraction. Speed is bounded by both this gesture's shortening and current retraction speed, without the old long-cord position error. Release or payout requests braking immediately; each new gesture requires a fresh stopped confirmation and stroke. Defaults: minimum 0.01 m, distance gain 2 /s, braking 0.8 m/s². Lower distance gain softens short gestures; higher minimum rejects small movements. Real stopping distance depends on load. Normal cruise resumes at the target length; the far Ahead rule remains. Existing restart acceleration caps and gain remain active, independently of the cruise switch.",
      )}
    </Typography.Paragraph>
  </Card>;
}
