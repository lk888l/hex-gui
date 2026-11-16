import { useEffect, useMemo, useRef, useState } from "react";
import { Alert, App, Button, Card, Col, Collapse, Descriptions, Input, InputNumber, Row, Select, Space, Steps, Switch, Table, Tabs, Tag, Typography } from "antd";
import ReactECharts from "echarts-for-react";
import { errMsg } from "../api";
import { useI18n } from "../i18n";
import { hpmImuApi as api } from "../hpmImuApi";
import { chipAttitude, defaultPdos, eulerDegrees, field, fresh, hex32, mapObject, odKey, pdoBytes, relativeQuaternion, wireBytes } from "../hpmImu";
import type { HpmImuObject, HpmImuPdo, HpmImuSnapshot, HpmImuValue, Quaternion } from "../hpmImu";
import type { ConnectionInfo } from "../types";
import { ImuViewer } from "./ImuViewer";
import { HpmImuTestPanel } from "./HpmImuTestPanel";
import "./HpmImuPanel.css";

const nmtName = (state?: number) => ({ 0: "Boot-up", 4: "Stopped", 5: "Operational", 127: "Pre-operational" }[state ?? -1] ?? "—");
const axes = ["X", "Y", "Z"];
type ChartSample = { t: number; values: (number | null)[] };

export function HpmImuPanel() {
  const { lang } = useI18n();
  const tr = (zh: string, en: string) => lang === "zh" ? zh : en;
  const { message } = App.useApp();
  const [spec, setSpec] = useState(() => /linux/i.test(navigator.userAgent) ? "can2" : "gs_usb0");
  const [connected, setConnected] = useState(false);
  const [operationBusy, setBusy] = useState(false);
  const working = useRef(false);
  const [node, setNode] = useState(31);
  const [period, setPeriod] = useState(8);
  const [link, setLink] = useState<ConnectionInfo | null>(null);
  const [catalog, setCatalog] = useState<HpmImuObject[]>([]);
  const [snapshot, setSnapshot] = useState<HpmImuSnapshot | null>(null);
  const busy = operationBusy || !!snapshot?.angle_test?.active || snapshot?.angle_test?.stop_confirmed === false;
  const [draft, setDraft] = useState<HpmImuPdo[]>(() => defaultPdos(31));
  const [diagnostics, setDiagnostics] = useState<HpmImuValue[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [pollError, setPollError] = useState<string | null>(null);
  const [csvPath, setCsvPath] = useState<string | null>(null);
  const [reference, setReference] = useState<Quaternion | null>(null);
  const lastQ = useRef<Quaternion>([1, 0, 0, 0]);
  const receivedAt = useRef(0);
  const [now, setNow] = useState(performance.now());
  const samples = useRef<ChartSample[]>([]);
  const [chartVersion, setChartVersion] = useState(0);
  const [chartIndex, setChartIndex] = useState(0x3003);
  const alive = useRef(true);

  useEffect(() => { alive.current = true; return () => { alive.current = false; }; }, []);
  useEffect(() => {
    if (!connected) return;
    let cancelled = false;
    let timer: number;
    const poll = async () => {
      try {
        const s = await api.state();
        if (cancelled) return;
        receivedAt.current = performance.now();
        setSnapshot(s);
        setPollError(null);
      } catch (e) { if (!cancelled) setPollError(errMsg(e)); }
      if (!cancelled) timer = window.setTimeout(poll, 50);
    };
    void poll();
    const clock = window.setInterval(() => setNow(performance.now()), 100);
    return () => { cancelled = true; window.clearTimeout(timer); window.clearInterval(clock); };
  }, [connected]);

  const elapsed = Math.max(0, now - receivedAt.current);
  const selected = snapshot?.identity?.node === node;
  const values = selected ? snapshot?.values ?? [] : [];
  const attitude = chipAttitude(values, elapsed);
  if (attitude) lastQ.current = attitude;
  const displayQ = relativeQuaternion(lastQ.current, reference);
  const euler = eulerDegrees(displayQ);
  const heartbeat = snapshot?.heartbeat.find(h => h.node === node);
  const heartbeatFresh = heartbeat != null && heartbeat.age_ms + elapsed < 1800;
  const fault = field(values, 0x3004, 4);
  const sensorStatus = field(values, 0x3004, 3);
  const chipStatus = field(values, 0x3008, 5);

  useEffect(() => {
    if (!connected || !selected) return;
    const t = performance.now();
    if (samples.current.length && t - samples.current[samples.current.length - 1].t < 180) return;
    const data = [0x3002, 0x3003, 0x3005].flatMap(index => [1, 2, 3].map(sub => {
      const v = field(values, index, sub);
      const counter = field(values, 0x3004, 2);
      const good = fresh(fault, elapsed) && !(fault!.value & (index === 0x3005 ? 2 : 1));
      return fresh(v, elapsed) && good && (index === 0x3005 || (fresh(counter, elapsed) && counter!.changed_age_ms + elapsed < 250)) ? v!.value : null;
    }));
    samples.current.push({ t, values: data });
    samples.current = samples.current.filter(s => s.t > t - 20_000);
    setChartVersion(v => v + 1);
  }, [snapshot, now, connected, selected]);

  const chart = useMemo(() => {
    const offset = [0x3002, 0x3003, 0x3005].indexOf(chartIndex) * 3;
    const t = performance.now();
    return {
      animation: false, tooltip: { trigger: "axis" },
      legend: { textStyle: { color: "#aaa" } }, grid: { left: 65, right: 20, top: 30, bottom: 30 },
      xAxis: { type: "value", min: -20, max: 0, axisLabel: { formatter: "{value}s" } },
      yAxis: { type: "value", scale: true, name: chartIndex === 0x3002 ? "g" : chartIndex === 0x3003 ? "°/s" : "µT" },
      series: axes.map((name, i) => ({ name, type: "line", showSymbol: false, connectNulls: false,
        lineStyle: { width: 1.5 }, color: ["#ff6b6b", "#2ecc71", "#4f8cff"][i],
        data: samples.current.map(s => [(s.t - t) / 1000, s.values[offset + i]]) })),
    };
  }, [chartVersion, chartIndex]);

  const resetView = () => {
    lastQ.current = [1, 0, 0, 0]; setReference(null); samples.current = []; setDiagnostics([]); setChartVersion(v => v + 1);
  };
  const accept = (s: HpmImuSnapshot) => {
    setSnapshot(s); setDraft(s.pdos); receivedAt.current = performance.now(); resetView();
  };
  const run = async (fn: () => Promise<void>) => {
    if (working.current) return;
    working.current = true; setBusy(true); setError(null);
    try { await fn(); }
    catch (e) { if (alive.current) { setError(errMsg(e)); message.error(errMsg(e)); } }
    finally { working.current = false; if (alive.current) setBusy(false); }
  };
  const disconnect = async () => {
    await api.disconnect(); setConnected(false); setSnapshot(null); setLink(null); setPollError(null); resetView();
  };
  const exportCsv = async () => {
    setCsvPath(await api.exportCsv());
    message.success(tr("CSV 已保存", "CSV saved"));
  };
  const readValue = (index: number, sub: number, digits = 3) => {
    const v = field(values, index, sub);
    return <span className={fresh(v, elapsed) ? "" : "hpm-imu-stale"} title={v ? `${Math.round(v.age_ms + elapsed)} ms` : "—"}>
      {v ? v.value.toFixed(digits) : "—"}{v && !fresh(v, elapsed) ? " *" : ""}
    </span>;
  };
  const wrongLink = link && (link.fd_enabled === false || (link.nominal?.bitrate != null && link.nominal.bitrate !== 1_000_000) || (link.data?.bitrate != null && link.data.bitrate !== 4_000_000));

  return <div className="hpm-imu">
    <Card size="small">
      <Space wrap>
        <Typography.Text>{tr("CAN 接口", "CAN interface")}</Typography.Text>
        <Input aria-label="CAN interface" style={{ width: 160 }} value={spec} onChange={e => setSpec(e.target.value)} disabled={connected || busy} />
        <Button type={connected ? "default" : "primary"} loading={operationBusy} onClick={() => run(connected ? disconnect : async () => {
          const result = await api.connect(spec.trim()); setCatalog(result.objects); setLink(result.link); setConnected(true);
        })}>{connected ? tr("断开", "Disconnect") : tr("连接", "Connect")}</Button>
        <Tag color={connected ? "green" : "default"}>{connected ? tr("已连接", "Connected") : tr("未连接", "Disconnected")}</Tag>
        <Tag>CAN FD · 1M / 4M · SP 80%</Tag>
        {link && <Tag>{link.backend} · N {link.nominal?.bitrate ?? "?"} / D {link.data?.bitrate ?? "?"}</Tag>}
      </Space>
      <Typography.Paragraph type="secondary" className="hpm-imu-note">
        {tr("SocketCAN 使用系统已有配置；gs_usb 自动使用 1M/4M。仅查看 IMU 时被动监听；启动电机测试后发送主机心跳，直到断开连接。", "SocketCAN uses system timing; gs_usb opens at 1M/4M. IMU viewing is passive; starting a motor test enables host heartbeats until disconnect.")}
      </Typography.Paragraph>
      {wrongLink && <Alert type="warning" showIcon message={tr("接口速率与 IMU 不匹配，请在系统中配置 CAN FD 1M/4M 后重新连接。", "Interface timing differs from the IMU. Configure CAN FD 1M/4M in the system and reconnect.")} />}
      {link?.inspection_error && <Alert type="info" message={link.inspection_error} />}
    </Card>

    <Card size="small" title={tr("初始化与设备状态", "Initialization & device state")}>
      <Steps size="small" current={!connected ? 0 : !selected ? 1 : heartbeat?.state === 5 && attitude ? 3 : 2} items={[
        { title: tr("连接总线", "Connect") }, { title: tr("确认设备", "Identify") },
        { title: "Pre-op → PDO → Start" }, { title: tr("实时数据", "Live data") },
      ]} />
      <Space wrap className="hpm-imu-actions">
        <span>Node-ID</span>
        <InputNumber aria-label="Node-ID" min={1} max={127} precision={0} value={node} disabled={busy} onChange={v => {
          if (v == null) return; setNode(v); setDraft(defaultPdos(v, period)); resetView();
        }} />
        {!!snapshot?.heartbeat.length && <Select aria-label="Discovered nodes" placeholder={tr("心跳发现", "Discovered nodes")} style={{ minWidth: 190 }} disabled={busy}
          value={snapshot.heartbeat.some(h => h.node === node) ? node : undefined}
          onChange={v => { setNode(v); setDraft(defaultPdos(v, period)); resetView(); }}
          options={snapshot.heartbeat.map(h => ({ value: h.node, label: `${h.node} · ${nmtName(h.state)}${h.age_ms + elapsed > 1800 ? " · offline" : ""}` }))} />}
        <span>{tr("默认 PDO 周期", "Default PDO period")}</span>
        <InputNumber aria-label="PDO period" min={1} max={65535} precision={0} addonAfter="ms" value={period} disabled={busy} onChange={v => v != null && setPeriod(v)} />
        <Button type="primary" disabled={!connected || busy} onClick={() => run(async () => {
          accept(await api.configure(node, null, period)); message.success(tr("初始化完成，等待 PDO 数据", "Initialized; waiting for PDO data"));
        })}>{tr("一键初始化", "Initialize")}</Button>
        <Button disabled={!connected || busy} onClick={() => run(async () => accept(await api.read(node)))}>{tr("读取当前配置", "Read configuration")}</Button>
      </Space>
      <Typography.Paragraph type="secondary" className="hpm-imu-note">
        {tr("初始化：读取 bimu 身份 → 定向 Pre-op → 配置并回读 TPDO1/2（各 29 字节，FD 补齐 32 字节），禁用 TPDO3/4 → Start。配置只保存在 RAM，复位恢复固件默认值。", "Initialize: read bimu identity → directed Pre-op → configure/read back TPDO1/2 (29 bytes each, padded to 32), disable TPDO3/4 → Start. Configuration lives in RAM and resets with firmware defaults.")}
      </Typography.Paragraph>
      <Space wrap>
        <Tag color={heartbeatFresh ? "green" : "default"}>{nmtName(heartbeat?.state)} · {heartbeat ? `${Math.round(heartbeat.age_ms + elapsed)} ms` : "—"}</Tag>
        {[[1, "Start"], [0x80, "Pre-op"], [2, "Stop"], [0x81, tr("复位节点", "Reset node")], [0x82, tr("复位通信", "Reset comm")]].map(([command, title]) =>
          <Button key={command} size="small" disabled={!connected || busy} onClick={() => run(async () => { await api.nmt(node, command as number); if (Number(command) >= 0x81) resetView(); })}>{title}</Button>)}
        <Button size="small" disabled={!connected || busy} onClick={() => run(() => api.sync(node))}>{tr("发送一次 SYNC", "Send SYNC once")}</Button>
      </Space>
      {selected && snapshot?.identity && <Descriptions size="small" column={{ xs: 1, sm: 2, lg: 3 }} className="hpm-imu-actions" items={[
        { key: "name", label: tr("设备", "Device"), children: snapshot.identity.name || "HPM bimu" },
        { key: "fw", label: tr("固件", "Firmware"), children: snapshot.identity.firmware || "—" },
        { key: "hw", label: tr("硬件", "Hardware"), children: snapshot.identity.hardware == null ? "—" : hex32(snapshot.identity.hardware) },
        { key: "serial", label: "Serial", children: hex32(snapshot.identity.serial) },
        { key: "product", label: "Vendor / Product", children: `${hex32(snapshot.identity.vendor)} / ${hex32(snapshot.identity.product)}` },
        { key: "revision", label: "Revision", children: hex32(snapshot.identity.revision) },
      ]} />}
    </Card>
    {(error || pollError || snapshot?.error) && <Alert type="error" showIcon message={error || pollError || snapshot?.error} />}

    <Tabs defaultActiveKey="live" items={[
      { key: "test", label: tr("Raw 角度测试", "Raw angle test"), children: <HpmImuTestPanel node={node}
        ready={connected && selected && !!attitude} busy={operationBusy} state={snapshot?.angle_test ?? null} run={run}
        onStart={angle_test => setSnapshot(old => old ? { ...old, angle_test } : old)} /> },
      { key: "live", label: tr("实时测量", "Live measurements"), children: <>
        <Space wrap className="hpm-imu-actions">
          {(selected ? snapshot?.stats : [])?.map((s, i) => <Tag key={i} color={s.age_ms != null && s.age_ms + elapsed < 250 ? "green" : "default"}>
            TPDO{i + 1} · {s.hz.toFixed(1)} Hz · {s.frames} {tr("帧", "frames")} · {s.bytes} B {s.fd ? `FD${s.brs ? "+BRS" : ""}` : "Classic"} · {s.age_ms == null ? "—" : `${Math.round(s.age_ms + elapsed)} ms`}
          </Tag>)}
          <Button disabled={!connected || busy || !selected || !snapshot?.recorded_frames} onClick={() => run(exportCsv)}>{tr("导出最近 60 秒 CSV", "Export last 60 s CSV")}</Button>
        </Space>
        {csvPath && <Typography.Paragraph copyable={{ text: csvPath }} type="secondary">{csvPath}</Typography.Paragraph>}
        <Row gutter={[16, 16]}>
          <Col xs={24} xl={12}><Card size="small" title={tr("片上 SFLP 姿态", "On-chip SFLP attitude")} extra={<Tag color={attitude ? "green" : "orange"}>{attitude ? tr("有效", "Fresh") : tr("等待 / 已冻结", "Waiting / frozen")}</Tag>}>
            <ImuViewer quaternion={displayQ} />
            <Space wrap className="hpm-imu-actions">
              <Button disabled={!attitude} onClick={() => setReference(attitude)}>{tr("显示归零", "Zero display")}</Button>
              <Button disabled={!reference} onClick={() => setReference(null)}>{tr("恢复原始姿态", "Clear reference")}</Button>
              {euler.map((v, i) => <Tag key={i}>{["Roll", "Pitch", "Yaw"][i]} {v.toFixed(1)}°</Tag>)}
            </Space>
            <Typography.Text code>WXYZ [{displayQ.map(v => v.toFixed(4)).join(", ")}]</Typography.Text>
            <Typography.Paragraph type="secondary" className="hpm-imu-note">{tr("六轴片上融合，Yaw 是相对航向。红 X / 绿 Y / 蓝 Z 为传感器原生坐标；模型为示意。显示归零只改变视图，CSV 保留原始值。", "Six-axis on-chip fusion; yaw is relative. Red X / green Y / blue Z use native sensor axes; the board is schematic. Zeroing only changes the view; CSV retains raw values.")}</Typography.Paragraph>
          </Card></Col>
          <Col xs={24} xl={12}><Card size="small" title={tr("测量与健康状态", "Measurements & health")}>
            <table className="hpm-imu-values"><thead><tr><th></th>{axes.map(a => <th key={a}>{a}</th>)}</tr></thead><tbody>
              {[[0x3002, tr("加速度 / g", "Acceleration / g"), 1], [0x3003, tr("角速度 / °/s", "Angular rate / °/s"), 1],
                [0x3005, tr("磁场 / µT", "Magnetic field / µT"), 1], [0x3006, tr("重力 / g", "Gravity / g"), 1],
                [0x3006, tr("片上零偏 / °/s", "On-chip bias / °/s"), 4]].map(([index, label, start]) => <tr key={String(label)}><td>{label}</td>{[0, 1, 2].map(a => <td key={a}>{readValue(Number(index), Number(start) + a)}</td>)}</tr>)}
            </tbody></table>
            <Descriptions size="small" column={2} className="hpm-imu-actions" items={[
              { key: "temp", label: tr("温度 / °C", "Temperature / °C"), children: readValue(0x3004, 1, 2) },
              { key: "count", label: tr("IMU 样本", "IMU samples"), children: readValue(0x3004, 2, 0) },
              { key: "chipCount", label: tr("SFLP 样本", "SFLP samples"), children: readValue(0x3008, 1, 0) },
              { key: "age", label: tr("SFLP 年龄 / ms", "SFLP age / ms"), children: readValue(0x3008, 2, 0) },
              { key: "status", label: tr("传感器状态", "Sensor status"), children: sensorStatus ? hex32(sensorStatus.value) : "—" },
              { key: "chipStatus", label: tr("SFLP 状态", "SFLP status"), children: chipStatus ? hex32(chipStatus.value) : "—" },
              { key: "fault", label: tr("故障码", "Fault"), children: fault ? <Tag color={fault.value ? "red" : fresh(fault, elapsed) ? "green" : "default"}>{hex32(fault.value)}</Tag> : "—" },
              { key: "invalid", label: tr("无效 PDO", "Invalid PDOs"), children: snapshot?.invalid_frames ?? 0 },
            ]} />
            {!!fault?.value && <Alert type="warning" message={tr("故障位：bit0 IMU 数据失效；bit1 磁场失效。读数保留上次值。", "Fault bits: bit0 invalid IMU data; bit1 invalid magnetic data. Last values are retained.")} />}
            <Typography.Paragraph type="secondary" className="hpm-imu-note">{tr("* 表示超过 250 ms 未更新；— 表示未映射或未收到。片上姿态失效会冻结视图。两路 PDO 独立采样，CSV 每行保留该帧接收时间和该帧字段。", "* marks values older than 250 ms; — means unmapped or absent. Invalid attitude freezes the view. PDO streams are independent; each CSV row contains that frame’s receive time and fields.")}</Typography.Paragraph>
          </Card></Col>
        </Row>
        <Card size="small" className="hpm-imu-actions" title={tr("最近 20 秒", "Last 20 seconds")} extra={<Select value={chartIndex} style={{ width: 160 }} onChange={setChartIndex} options={[
          { value: 0x3002, label: tr("加速度", "Acceleration") }, { value: 0x3003, label: tr("角速度", "Angular rate") }, { value: 0x3005, label: tr("磁场", "Magnetic field") },
        ]} />}><ReactECharts option={chart} notMerge style={{ height: 240 }} /></Card>
      </> },
      { key: "pdo", label: tr("PDO 配置", "PDO configuration"), children: <Space direction="vertical" style={{ width: "100%" }} size={12}>
        <Typography.Paragraph type="secondary">{tr("可编辑四路 TPDO 的 COB-ID、传输类型、周期、抑制时间和映射顺序。每路最多 24 项 / 64 字节。≤8 字节为 Classic，>8 字节自动 FD+BRS。应用会进入 Pre-op，写入并回读后 Start；失败时尝试恢复原配置。", "Edit all four TPDOs: COB-ID, transmission type, period, inhibit time and ordered mapping. Maximum 24 entries / 64 bytes per PDO. ≤8 bytes uses Classic; >8 uses FD+BRS. Apply enters Pre-op, writes and verifies, then starts; errors trigger restoration of the previous configuration.")}</Typography.Paragraph>
        <Space wrap>
          <Button disabled={busy} onClick={() => setDraft(defaultPdos(node, period))}>{tr("载入默认映射", "Load default mapping")}</Button>
          <Button disabled={!connected || busy} onClick={() => run(async () => accept(await api.read(node)))}>{tr("读取设备映射", "Read device mapping")}</Button>
          <Button type="primary" disabled={!connected || busy} onClick={() => run(async () => {
            accept(await api.configure(node, draft, period)); message.success(tr("PDO 已应用并回读确认", "PDOs applied and verified"));
          })}>{tr("应用配置并启动", "Apply & start")}</Button>
        </Space>
        <Collapse defaultActiveKey={["0", "1"]} items={draft.map((p, i) => ({ key: String(i),
          label: <Space>TPDO{i + 1}<Tag>{odKey(0x1800 + i, 0).slice(0, 4)} / {odKey(0x1a00 + i, 0).slice(0, 4)}</Tag><Tag color={pdoBytes(p) > 64 ? "red" : "blue"}>{p.mapping.length}/24 · {pdoBytes(p)}/64 B → {wireBytes(pdoBytes(p))} B</Tag></Space>,
          children: <PdoEditor pdo={p} catalog={catalog} disabled={busy} zh={lang === "zh"} onChange={next => setDraft(prev => prev.map((p, j) => i === j ? next : p))} />,
        }))} />
      </Space> },
      { key: "diagnostics", label: tr("诊断与对象", "Diagnostics & objects"), children: <>
        <Space wrap className="hpm-imu-actions">
          <Button disabled={!connected || busy} onClick={() => run(async () => setDiagnostics(await api.diagnostics(node)))}>{tr("读取诊断与传感器配置", "Read diagnostics & sensor configuration")}</Button>
          <Typography.Text type="secondary">{tr("0x3100 / 0x3102 为只读配置；此固件没有保存参数或片上校零命令。", "0x3100 / 0x3102 are read-only. This firmware has no parameter-save or on-chip zeroing command.")}</Typography.Text>
        </Space>
        <Table size="small" pagination={false} rowKey={v => odKey(v.index, v.sub)} dataSource={diagnostics} columns={[
          { title: tr("对象", "Object"), render: (_, v) => odKey(v.index, v.sub) },
          { title: tr("名称", "Name"), render: (_, v) => catalog.find(o => o.index === v.index && o.sub === v.sub)?.name ?? (v.index === 0x3100 ? ["Accel range", "Accel ODR", "Gyro range", "Gyro ODR"][v.sub - 1] : ["Mag ODR", "Mag range", "Mag OSR1", "Mag OSR2"][v.sub - 1]) },
          { title: tr("值", "Value"), dataIndex: "value", render: v => v === 0xffffffff ? "UINT32_MAX (no sample)" : v },
          { title: tr("读取时间", "Read at"), render: (_, v) => new Date(v.rx_unix_ms).toLocaleTimeString() },
        ]} />
        <Typography.Title level={5}>{tr("当前映射对象值", "Live mapped objects")}</Typography.Title>
        <Table size="small" pagination={{ pageSize: 12 }} rowKey={v => odKey(v.index, v.sub)} dataSource={values} columns={[
          { title: tr("对象", "Object"), render: (_, v) => odKey(v.index, v.sub) },
          { title: tr("名称", "Name"), render: (_, v) => catalog.find(o => o.index === v.index && o.sub === v.sub)?.name },
          { title: tr("值", "Value"), render: (_, v) => `${v.value} ${catalog.find(o => o.index === v.index && o.sub === v.sub)?.unit ?? ""}` },
          { title: tr("接收年龄", "Receive age"), render: (_, v) => `${Math.round(v.age_ms + elapsed)} ms` },
        ]} />
      </> },
    ]} />
  </div>;
}

function PdoEditor({ pdo: p, catalog, disabled, zh, onChange }: {
  pdo: HpmImuPdo; catalog: HpmImuObject[]; disabled: boolean; zh: boolean; onChange: (next: HpmImuPdo) => void;
}) {
  const tr = (cn: string, en: string) => zh ? cn : en;
  const patch = (value: Partial<HpmImuPdo>) => onChange({ ...p, ...value });
  const mapping = (value: number[]) => patch({ mapping: value });
  const shift = (from: number, to: number) => { const next = [...p.mapping]; [next[from], next[to]] = [next[to], next[from]]; mapping(next); };
  let offset = 0;
  return <div>
    <Space wrap>
      <Switch checked={!(p.cob_id & 0x80000000)} disabled={disabled} checkedChildren="ON" unCheckedChildren="OFF" onChange={enabled => patch({ cob_id: ((p.cob_id & 0x7fffffff) | (enabled ? 0 : 0x80000000)) >>> 0 })} />
      <span>COB-ID</span><InputNumber aria-label="COB-ID" min={1} max={0x7ff} precision={0} disabled={disabled} value={p.cob_id & 0x7ff}
        formatter={n => `0x${Number(n).toString(16).toUpperCase()}`} parser={s => Number(s ?? 0)} onChange={v => v != null && patch({ cob_id: ((p.cob_id & 0xc0000000) | v) >>> 0 })} />
      <span>{tr("传输类型", "Type")}</span><InputNumber aria-label="Transmission type" min={0} max={255} precision={0} disabled={disabled} value={p.transmission_type} onChange={v => v != null && patch({ transmission_type: v })} />
      <span>{tr("事件周期 ms", "Event ms")}</span><InputNumber min={0} max={65535} precision={0} disabled={disabled} value={p.event_ms} onChange={v => v != null && patch({ event_ms: v })} />
      <span>{tr("抑制 ×100µs", "Inhibit ×100µs")}</span><InputNumber min={0} max={65535} precision={0} disabled={disabled} value={p.inhibit_100us} onChange={v => v != null && patch({ inhibit_100us: v })} />
      <span>SYNC start</span><InputNumber min={0} max={240} precision={0} disabled={disabled} value={p.sync_start} onChange={v => v != null && patch({ sync_start: v })} />
    </Space>
    <Typography.Paragraph type="secondary" className="hpm-imu-note">{tr("255 / 254：事件驱动；0～240：同步 PDO，需要总线上的 SYNC。周期 0 关闭事件定时器。", "255 / 254: event driven; 0–240: synchronous PDOs, requiring SYNC on the bus. Event period 0 disables the event timer.")}</Typography.Paragraph>
    {p.mapping.map((entry, i) => {
      const start = offset; offset += (entry & 255) / 8;
      return <div className="hpm-imu-map-row" key={i}>
        <Typography.Text code>{i + 1} · +{start}B</Typography.Text>
        <Select showSearch optionFilterProp="label" disabled={disabled} value={entry} style={{ flex: 1, minWidth: 200 }}
          options={catalog.map(o => ({ value: mapObject(o), label: `${odKey(o.index, o.sub)} · ${o.name} · ${o.bits} bit ${o.unit}` }))}
          onChange={v => mapping(p.mapping.map((m, j) => j === i ? v : m))} />
        <Button size="small" aria-label="Move mapping up" disabled={disabled || i === 0} onClick={() => shift(i, i - 1)}>↑</Button>
        <Button size="small" aria-label="Move mapping down" disabled={disabled || i === p.mapping.length - 1} onClick={() => shift(i, i + 1)}>↓</Button>
        <Button size="small" disabled={disabled} onClick={() => mapping(p.mapping.filter((_, j) => j !== i))}>{tr("删除", "Remove")}</Button>
      </div>;
    })}
    <Button size="small" disabled={disabled || p.mapping.length >= 24 || !catalog.length} onClick={() => mapping([...p.mapping, mapObject(catalog[0])])}>+ {tr("映射对象", "Map object")}</Button>
  </div>;
}
