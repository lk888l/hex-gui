import { useState } from "react";
import { Alert, Button, Card, Collapse, Descriptions, InputNumber, Progress, Select, Space, Switch, Table, Tag, Typography } from "antd";
import { useI18n } from "../i18n";
import { hpmImuApi as api } from "../hpmImuApi";
import { testSummary } from "../hpmImu";
import type { HpmImuTestConfig, HpmImuTestRow, HpmImuTestSnapshot } from "../hpmImu";

export function HpmImuTestPanel({ node, ready, busy, state, run, onStart }: {
  node: number; ready: boolean; busy: boolean; state: HpmImuTestSnapshot | null;
  run: (action: () => Promise<void>) => Promise<void>;
  onStart: (state: HpmImuTestSnapshot) => void;
}) {
  const { lang } = useI18n();
  const tr = (zh: string, en: string) => lang === "zh" ? zh : en;
  const [config, setConfig] = useState<HpmImuTestConfig>({
    imu_node: node, motor_node: 1, host_node: 126, angle_deg: 90, repeats: 10,
    axis: 2, direction: 1, zero_each_round: true, speed_deg_s: 30,
    settle_ms: 500, samples: 20, tolerance_deg: 0.1, timeout_s: 60,
  });
  const locked = busy || !!state?.active || state?.stop_confirmed === false;
  const patch = (value: Partial<HpmImuTestConfig>) => setConfig(old => ({ ...old, ...value }));
  const summary = testSummary(state?.rows ?? []);
  const fmt = (v: number | null | undefined, unit = "°") => v == null ? "—" : `${v.toFixed(4)}${unit}`;
  const phase: Record<string, string> = {
    initializing: tr("初始化电机", "Initializing motor"), homing: tr("回零 / 稳定采样", "Homing / settling"),
    rotating: tr("转动 / 稳定采样", "Rotating / settling"), returning: tr("返回零位 / 稳定采样", "Returning / settling"),
    stopping: tr("电机失能确认中", "Confirming motor disable"), cancelled: tr("已停止", "Stopped"),
    completed: tr("测试完成", "Completed"), failed: tr("测试失败", "Failed"),
  };
  const number = (key: keyof HpmImuTestConfig, label: string, min: number, max: number, unit = "", precision = 0) =>
    <label className="hpm-imu-test-field" key={key}><span>{label}</span>
      <InputNumber aria-label={label} value={Number(config[key])} min={min} max={max} precision={precision}
        addonAfter={unit || undefined} disabled={locked} onChange={v => v != null && patch({ [key]: v })} />
    </label>;
  const columns = [
    { title: tr("次数", "Round"), dataIndex: "round", width: 70 },
    { title: tr("目标角", "Target"), dataIndex: "target_deg" },
    { title: tr("电机实际角", "Motor"), dataIndex: "motor_deg" },
    { title: tr("出发前残余角", "Home residual"), dataIndex: "home_raw_deg" },
    { title: tr("Raw 实测角", "Raw angle"), dataIndex: "raw_deg" },
    { title: tr("误差", "Error"), dataIndex: "error_deg" },
    { title: tr("有符号误差 %", "Signed error %"), dataIndex: "error_percent" },
    { title: tr("绝对误差 %", "Absolute error %"), dataIndex: "absolute_error_percent" },
    { title: tr("回零残余角", "Return residual"), dataIndex: "return_raw_deg" },
  ].map(c => ({ ...c, render: c.dataIndex === "round" ? undefined : (v: number | null) => fmt(v, c.dataIndex.includes("percent") ? "%" : "°") }));

  return <Space direction="vertical" size={16} style={{ width: "100%" }}>
    <Card size="small" title={tr("Raw 角度精度测试 · 电机基准", "Raw angle accuracy · Motor reference")}>
      <Typography.Paragraph type="secondary">{tr(
        "将 IMU 固定在电机输出轴上，所选传感器轴与转轴对齐。IMU 和电机接入同一 CAN 总线（1M/4M），节点 ID 不同。自动识别 hexmeow-4310 / 4342，角度使用输出轴单位。",
        "Mount the IMU with the selected sensor axis aligned to the motor shaft. Connect both devices to the same CAN bus (1M/4M), with distinct node IDs. hexmeow-4310 / 4342 are identified automatically; angles use output-shaft units.")}</Typography.Paragraph>
      <div className="hpm-imu-test-grid">
        {number("motor_node", tr("电机 Node-ID", "Motor Node-ID"), 1, 127)}
        {number("angle_deg", tr("旋转角度（负数反转）", "Rotation (negative reverses)"), -360, 360, "°", 2)}
        {number("repeats", tr("测试次数", "Repeat count"), 1, 10000)}
        {number("speed_deg_s", tr("转动速度", "Speed"), 1, 180, "°/s", 1)}
        <label className="hpm-imu-test-field"><span>{tr("IMU 测量轴", "IMU axis")}</span><Select aria-label="IMU axis" value={config.axis} disabled={locked}
          options={["X", "Y", "Z"].map((label, value) => ({ label, value }))} onChange={axis => patch({ axis })} /></label>
        <label className="hpm-imu-test-field"><span>{tr("IMU 与电机正方向", "IMU vs motor direction")}</span><Select aria-label="IMU direction" value={config.direction} disabled={locked}
          options={[{ value: 1, label: tr("同向", "Same") }, { value: -1, label: tr("反向", "Opposite") }]} onChange={direction => patch({ direction })} /></label>
      </div>
      <Space wrap className="hpm-imu-actions">
        {[90, 180].map(angle => <Button key={angle} size="small" disabled={locked} onClick={() => patch({ angle_deg: angle })}>{angle}°</Button>)}
        <Switch aria-label="Zero IMU at each home" checked={config.zero_each_round} disabled={locked} onChange={zero_each_round => patch({ zero_each_round })} />
        <span>{tr("每次电机回零后 IMU 置零", "Zero IMU at each home")}</span>
      </Space>
      <Typography.Paragraph type="secondary">{tr(
        "每轮：电机回到已有的 0° → 稳定 → 可选 IMU 置零 → 转到设定角度 → 稳定采样 → 回 0°。测试开始时建立一次角度参考；开关关闭后一直保留该参考，可测累计漂移。置零仅重设测试参考，使用原始 SFLP 四元数，不写入设备。",
        "Each round: return to the motor’s existing 0° → settle → optional IMU zero → rotate to target → settle and sample → return to 0°. One reference is established at test start; disabling per-round zero preserves it to measure drift. Zeroing resets the test reference using raw SFLP quaternions; it does not write to the IMU.")}</Typography.Paragraph>
      <Collapse size="small" items={[{ key: "timing", label: tr("采样与到位设置", "Sampling and arrival settings"), children: <div className="hpm-imu-test-grid">
        {number("settle_ms", tr("到位后稳定时间", "Settling time"), 100, 10000, "ms")}
        {number("samples", tr("每点采样数", "Samples per point"), 1, 200)}
        {number("tolerance_deg", tr("电机到位容差", "Motor arrival tolerance"), 0.001, 1, "°", 3)}
        {number("timeout_s", tr("每段运动超时", "Timeout per leg"), 2, 600, "s")}
        {number("host_node", tr("主机心跳 Node-ID（空闲 ID）", "Host heartbeat Node-ID (unused)"), 1, 127)}
      </div> }]} />
      <Space wrap className="hpm-imu-actions">
        <Button type="primary" disabled={!ready || locked || Math.abs(config.angle_deg) < 0.1} onClick={() => run(async () => {
          onStart(await api.testStart({ ...config, imu_node: node }));
        })}>{tr("开始测试", "Start test")}</Button>
        <Button danger disabled={busy || !state || (!state.active && state.stop_confirmed)} onClick={() => run(() => api.testStop())}>
          {tr("停止 / 电机失能", "Stop / disable motor")}</Button>
        {!ready && <Typography.Text type="secondary">{tr("先连接并初始化 IMU，等待有效姿态。", "Connect and initialize the IMU, then wait for valid attitude.")}</Typography.Text>}
      </Space>
      <Typography.Paragraph type="secondary">{tr(
        "电机到位并低速稳定后，取多个新样本的平均角度。误差 = Raw 实测角 − 设定角；误差百分比 = 误差 ÷ |设定角| × 100%。汇总使用绝对误差，避免正负抵消。",
        "Samples are averaged after fresh motor feedback confirms arrival and low speed. Error = raw angle − target; error % = error / |target| × 100%. Summary uses absolute errors to avoid cancellation.")}</Typography.Paragraph>
    </Card>
    {state && <Card size="small" title={tr("测试结果", "Test results")} extra={<Tag color={state.active ? "blue" : state.phase === "completed" ? "green" : "default"}>{phase[state.phase] ?? state.phase}</Tag>}>
      {state.error && <Alert type="error" showIcon message={state.error} />}
      {!state.active && !state.stop_confirmed && <Alert type="error" message={tr("电机失能未确认，请重试停止。", "Motor disable is unconfirmed; retry Stop.")} />}
      <Progress percent={Math.round(state.completed / state.config.repeats * 100)} status={state.phase === "failed" ? "exception" : state.active ? "active" : undefined}
        format={() => `${state.completed} / ${state.config.repeats}`} />
      <Descriptions size="small" column={{ xs: 1, sm: 2, lg: 3 }} items={[
        { key: "motor", label: tr("电机", "Motor"), children: `${state.motor_model ?? "—"} · ID ${state.config.motor_node}` },
        { key: "angle", label: tr("本次参数", "Run settings"), children: `${state.config.angle_deg}° · ${["X", "Y", "Z"][state.config.axis]} · ${state.config.zero_each_round ? tr("逐次置零", "Zero each round") : tr("保留参考", "Fixed reference")}` },
        { key: "pos", label: tr("电机实时角度", "Motor position"), children: fmt(state.motor_position_deg) },
        { key: "raw", label: tr("Raw 实时角度", "Raw angle"), children: fmt(state.imu_angle_deg) },
        { key: "mean", label: tr("平均绝对误差", "Mean absolute error"), children: fmt(summary?.meanAbsolutePercent, "%") },
        { key: "max", label: tr("最大绝对误差", "Max absolute error"), children: fmt(summary?.maxAbsolutePercent, "%") },
        { key: "bias", label: tr("平均有符号误差", "Mean signed error"), children: fmt(summary?.meanError) },
        { key: "rms", label: "RMSE", children: fmt(summary?.rmsDegrees) },
      ]} />
      <Table<HpmImuTestRow> size="small" rowKey="round" columns={columns} dataSource={state.rows} pagination={{ pageSize: 20 }} scroll={{ x: 1100 }} />
    </Card>}
  </Space>;
}
