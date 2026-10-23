import { useState } from "react";
import { Alert, Button, InputNumber, Progress, Select, Space, Table, Tag, Typography } from "antd";
import { useI18n } from "../i18n";
import { unitreeApi, type UnitreeState } from "../unitreeApi";
import { DEFAULT_UNITREE_SETTINGS, DEFAULT_UNITREE_TARGET, validateUnitreeBusTest, type UnitreeTarget } from "../unitreeServo";

export function UnitreeBusTestPanel({ state, disabled, settingsReady, target, run, stop }: {
  state: UnitreeState | null;
  disabled: boolean;
  settingsReady: boolean;
  target: UnitreeTarget;
  run: (operation: () => Promise<unknown>, success?: string) => Promise<void>;
  stop: () => Promise<void>;
}) {
  const { lang } = useI18n();
  const tr = (cn: string, en: string) => lang === "zh" ? cn : en;
  const [ids, setIds] = useState<number[]>(() => state?.bus_test?.nodes.map(node => node.id) ?? state?.devices.map(device => device.id) ?? []);
  const [duration, setDuration] = useState(state?.bus_test?.duration_s ?? 10);
  const [targets, setTargets] = useState<Record<number, UnitreeTarget>>(() => Object.fromEntries(state?.bus_test?.nodes.map(node => [node.id, node.target]) ?? []));
  const report = state?.bus_test;
  const running = !!report?.running;
  const settings = state?.settings ?? DEFAULT_UNITREE_SETTINGS;
  const nodeTarget = (id: number) => targets[id] ?? { ...DEFAULT_UNITREE_TARGET, position_rad: state?.devices.find(device => device.id === id)?.feedback.position_rad ?? 0 };
  const setValue = (id: number, key: keyof UnitreeTarget, value: number | null) => {
    if (value != null) setTargets(previous => ({ ...previous, [id]: { ...nodeTarget(id), [key]: value } }));
  };
  const ready = !!state?.connected && settingsReady && ids.length >= 2 && !disabled && !running && state.stop_confirmed && !state.bootloader_ids?.length;
  const motionReady = ready && ids.every(id => state?.devices.some(device => device.id === id && device.online && !device.feedback.error_bits && !device.feedback.timed_out));
  const start = (kind: "communication" | "motion") => void run(() => unitreeApi.startBusTest(validateUnitreeBusTest({
    kind, duration_s: duration, nodes: ids.map(id => ({ id, target: nodeTarget(id) })),
  }, settings)), tr("测试已启动；结束时会锁定全部参与舵机", "Test started; all participating servos will be locked at the end"));
  const exportReport = () => {
    if (!report) return;
    const url = URL.createObjectURL(new Blob([JSON.stringify({ schema_version: 1, exported_at: new Date().toISOString(), port: state?.port, baud_rate: state?.baud_rate, command_hz: settings.command_hz, report }, null, 2)], { type: "application/json" }));
    const anchor = document.createElement("a");
    anchor.href = url; anchor.download = `unitree-bus-test-${report.nodes.map(node => node.id).join("-")}.json`; anchor.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  const input = (id: number, key: keyof UnitreeTarget, min: number, max: number) => <InputNumber
    aria-label={`ID ${id} ${key}`} value={nodeTarget(id)[key]} min={min} max={max} step={key === "kp" ? 0.1 : 0.01}
    disabled={disabled || running} onChange={value => setValue(id, key, value)} style={{ width: 100 }} />;
  return <div className="unitree-bus-test">
    <Typography.Paragraph type="secondary">{tr("多台舵机共用这一条 USB 串口总线，每台必须有不同的 ID。先扫描并运行锁定模式通信测试，再按需运行 FOC 测试。程序逐 ID 发送并等待各自应答，结束或故障时逐台锁定。", "Servos share this USB serial bus and must have distinct IDs. Scan and run the lock-mode communication test first, then run FOC testing as needed. Requests and replies are addressed individually; all members are locked on completion or fault.")}</Typography.Paragraph>
    <div className="unitree-form-row">
      <label className="unitree-field"><span>{tr("参与测试的 ID（2–15 台）", "Test IDs (2–15 servos)")}</span><Select
        aria-label="Bus test IDs" mode="multiple" value={ids} onChange={setIds} disabled={disabled || running} style={{ width: 320 }}
        options={Array.from({ length: 15 }, (_, id) => ({ value: id, label: `ID ${id}${state?.devices.some(device => device.id === id && device.online) ? tr(" · 在线", " · online") : ""}` }))} /></label>
      <Button disabled={disabled || running || !state?.devices.length} onClick={() => setIds(state?.devices.map(device => device.id) ?? [])}>{tr("选择扫描结果", "Use scanned IDs")}</Button>
      <label className="unitree-field"><span>{tr("时长（秒）", "Duration (seconds)")}</span><InputNumber aria-label="Bus test duration" min={1} max={120} precision={0} value={duration} disabled={disabled || running} onChange={value => value != null && setDuration(value)} /></label>
      <Tag>{tr("目标每舵机频率", "Requested per-servo rate")}: {settings.command_hz} Hz</Tag>
    </div>
    <Space wrap>
      <Button type="primary" aria-label="Start bus communication test" disabled={!ready} onClick={() => start("communication")}>{tr("开始通信测试（锁定）", "Start communication test (lock)")}</Button>
      <Button aria-label="Start bus motion test" disabled={!motionReady} onClick={() => start("motion")}>{tr("按各 ID 目标开始 FOC 测试", "Start FOC test with per-ID targets")}</Button>
      <Button danger disabled={!running} onClick={() => void stop()}>{tr("停止全部测试舵机", "Stop all test servos")}</Button>
      <Button disabled={!report || running} onClick={exportReport}>{tr("导出测试报告 JSON", "Export test report JSON")}</Button>
    </Space>
    <Typography.Paragraph type="secondary" className="unitree-note">{tr("下表参数仅在 FOC 测试时生效。FOC 要求所选 ID 全部在线且无故障，先以零力矩、零增益确认使能，再发送各自目标，并始终启用舵机通信超时保护。实际频率受设备数量和串口往返时间影响，见结果表。", "Targets below apply to FOC tests. All selected IDs must be online without faults. Servos first acknowledge FOC with zero torque and gains, then receive their own targets. The servo communication watchdog stays enabled. Device count and serial round-trip time affect the achieved rate shown in the results.")}</Typography.Paragraph>
    <Space wrap style={{ marginBottom: 10 }}>
      <Button disabled={disabled || running || !ids.length} onClick={() => setTargets(Object.fromEntries(ids.map(id => [id, { ...target }])))}>{tr("套用当前单机目标到所选 ID", "Copy single-servo target to selected IDs")}</Button>
      <Button disabled={disabled || running || !ids.length} onClick={() => setTargets(previous => ({ ...previous, ...Object.fromEntries(ids.map(id => [id, { ...nodeTarget(id), position_rad: state?.devices.find(device => device.id === id)?.feedback.position_rad ?? nodeTarget(id).position_rad }])) }))}>{tr("各自使用当前位置", "Use each servo's current position")}</Button>
    </Space>
    <Table size="small" pagination={false} rowKey="id" scroll={{ x: 760 }} dataSource={ids.map(id => ({ id }))} columns={[
      { title: "ID", dataIndex: "id", width: 55 },
      { title: tr("位置 rad", "Position rad"), render: (_, row) => input(row.id, "position_rad", settings.position_min_rad, settings.position_max_rad) },
      { title: tr("速度 rad/s", "Velocity rad/s"), render: (_, row) => input(row.id, "velocity_rad_s", -settings.velocity_limit_rad_s, settings.velocity_limit_rad_s) },
      { title: tr("前馈 Nm", "Feedforward Nm"), render: (_, row) => input(row.id, "torque_nm", -settings.torque_limit_nm, settings.torque_limit_nm) },
      { title: "Kp", render: (_, row) => input(row.id, "kp", 0, 2128.523254) },
      { title: "Kd", render: (_, row) => input(row.id, "kd", 0, 21.285233) },
    ]} />
    {report && <>
      <div className="unitree-form-row"><Tag color={running ? "processing" : report.phase === "failed" ? "error" : "success"}>{({ starting: tr("准备中", "Preparing"), running: tr("测试中", "Running"), completed: tr("已完成", "Completed"), stopped: tr("已停止", "Stopped"), failed: tr("异常停止", "Failed") })[report.phase]}</Tag><span>{report.elapsed_s.toFixed(1)} / {report.duration_s} s · {tr("轮询周期", "Cycles")} {report.cycles} · {tr("超出目标周期", "Overruns")} {report.overruns}</span></div>
      <Progress percent={Math.min(100, Math.round(report.elapsed_s / report.duration_s * 100))} status={report.phase === "failed" ? "exception" : running ? "active" : "normal"} />
      {report.error && <Alert type="warning" showIcon message={report.error} />}
      <Table className="unitree-bus-results" size="small" rowKey="id" pagination={false} scroll={{ x: 980 }} dataSource={report.nodes} columns={[
        { title: "ID", dataIndex: "id", width: 45 },
        { title: "TX / RX", render: (_, row) => `${row.tx_count} / ${row.rx_count}` },
        { title: tr("丢包率", "Loss"), render: (_, row) => `${row.tx_count ? ((row.tx_count - row.rx_count) / row.tx_count * 100).toFixed(1) : "0.0"}%` },
        { title: "Hz", render: (_, row) => (row.rx_count / Math.max(0.001, report.elapsed_s)).toFixed(1) },
        { title: tr("超时 / 无效帧", "Timeout / invalid"), render: (_, row) => `${row.timeout_count} / ${row.crc_error_count}` },
        { title: tr("平均 / 最大 ms", "Mean / max ms"), render: (_, row) => `${row.mean_rtt_ms.toFixed(2)} / ${row.max_rtt_ms.toFixed(2)}` },
        { title: tr("反馈模式", "Feedback mode"), render: (_, row) => row.feedback?.mode ?? "—" },
        { title: tr("位置 rad", "Position rad"), render: (_, row) => row.feedback?.position_rad.toFixed(3) ?? "—" },
        { title: tr("故障码", "Fault bits"), render: (_, row) => row.feedback ? <Tag color={row.feedback.error_bits ? "red" : "default"}>{`0x${row.feedback.error_bits.toString(16).toUpperCase()}`}</Tag> : "—" },
        { title: tr("结束锁定", "End lock"), render: (_, row) => running ? "—" : <Tag color={row.stop_confirmed ? "green" : "red"}>{row.stop_confirmed ? tr("已确认", "Confirmed") : tr("未确认", "Unconfirmed")}</Tag> },
      ]} />
    </>}
  </div>;
}
