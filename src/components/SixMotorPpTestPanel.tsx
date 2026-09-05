import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import {
  Alert,
  App,
  Button,
  Card,
  Descriptions,
  InputNumber,
  Space,
  Table,
  Tag,
  Tooltip,
  Typography,
} from "antd";
import type { ColumnsType } from "antd/es/table";
import { api, errMsg } from "../api";
import { nid2hex } from "../format";
import { useI18n } from "../i18n";
import { formatPdoRateHz } from "../meowMotorUi";
import {
  SIX_MOTOR_PP_IDS,
  cloneSixMotorPpRequest,
  createSixMotorPpDefaultRequest,
  validateSixMotorPpRequest,
} from "../sixMotorPpTest";
import { OnlineTag } from "../tags";
import type {
  MotorInfo,
  SixMotorPpMotorView,
  SixMotorPpRequest,
  SixMotorPpView,
} from "../types";
import "./SixMotorPpTestPanel.css";

const STATE_POLL_MS = 50;

interface MotorRow {
  nid: number;
  device: MotorInfo | null;
  live: SixMotorPpMotorView | null;
}

interface WaypointRow {
  key: number;
  label: string;
  positions: number[];
}

export function SixMotorPpTestPanel({
  connected,
  devices,
  onBusyChange,
  onRunningChange,
}: {
  connected: boolean;
  devices: MotorInfo[];
  onBusyChange?: (busy: boolean) => void;
  onRunningChange?: (running: boolean) => void;
}) {
  const { message } = App.useApp();
  const { lang } = useI18n();
  const text = useCallback((en: string, zh: string) => (lang === "zh" ? zh : en), [lang]);
  const [request, setRequest] = useState<SixMotorPpRequest>(() =>
    createSixMotorPpDefaultRequest(),
  );
  const [view, setView] = useState<SixMotorPpView | null>(null);
  const [busy, setBusyState] = useState(false);
  const [running, setRunningState] = useState(false);
  const [localError, setLocalError] = useState<string | null>(null);
  const runningRef = useRef(false);
  const pollPendingRef = useRef(false);

  const setBusy = useCallback(
    (value: boolean) => {
      setBusyState(value);
      onBusyChange?.(value);
    },
    [onBusyChange],
  );
  const setRunning = useCallback(
    (value: boolean) => {
      runningRef.current = value;
      setRunningState(value);
      onRunningChange?.(value);
    },
    [onRunningChange],
  );
  const applyView = useCallback(
    (next: SixMotorPpView) => {
      setView(next);
      setRunning(next.running);
    },
    [setRunning],
  );

  const deviceByNid = useMemo(
    () => new Map(devices.map((device) => [device.node_id, device])),
    [devices],
  );
  const liveByNid = useMemo(
    () => new Map((view?.motors ?? []).map((motor) => [motor.node_id, motor])),
    [view],
  );
  const motorRows = useMemo<MotorRow[]>(
    () =>
      SIX_MOTOR_PP_IDS.map((nid) => ({
        nid,
        device: deviceByNid.get(nid) ?? null,
        live: liveByNid.get(nid) ?? null,
      })),
    [deviceByNid, liveByNid],
  );
  const unavailableReasons = useMemo(() => {
    const reasons: string[] = [];
    for (const nid of SIX_MOTOR_PP_IDS) {
      const device = deviceByNid.get(nid);
      if (device == null) reasons.push(text(`ID ${nid} not discovered`, `未发现 ID ${nid}`));
      else if (device.device_type !== "meow_motor") {
        reasons.push(text(`ID ${nid} is not a Meow Motor`, `ID ${nid} 不是 Meow Motor`));
      } else if (!device.online) reasons.push(text(`ID ${nid} is offline`, `ID ${nid} 已离线`));
    }
    return reasons;
  }, [deviceByNid, text]);

  useEffect(() => {
    if (!connected) {
      setView(null);
      setRunning(false);
      return;
    }
    let alive = true;
    const poll = () => {
      if (!alive || pollPendingRef.current) return;
      pollPendingRef.current = true;
      void api
        .sixMotorPpGetState()
        .then((next) => {
          if (alive) applyView(next);
        })
        .catch((error) => {
          if (alive && runningRef.current) setLocalError(errMsg(error));
        })
        .finally(() => {
          pollPendingRef.current = false;
        });
    };
    poll();
    const timer = window.setInterval(poll, STATE_POLL_MS);
    return () => {
      alive = false;
      window.clearInterval(timer);
    };
  }, [applyView, connected, setRunning]);

  useEffect(
    () => () => {
      onBusyChange?.(false);
      onRunningChange?.(false);
      if (runningRef.current) void api.sixMotorPpStop().catch(() => {});
    },
    [onBusyChange, onRunningChange],
  );

  const start = async () => {
    const nextRequest = cloneSixMotorPpRequest(request);
    try {
      validateSixMotorPpRequest(nextRequest);
    } catch (error) {
      message.error(errMsg(error));
      return;
    }
    setBusy(true);
    setLocalError(null);
    try {
      const next = await api.sixMotorPpStart(nextRequest);
      applyView(next);
      message.success(text("Sequential PP test started", "逐轴 PP 测试已启动"));
    } catch (error) {
      const detail = errMsg(error);
      setLocalError(detail);
      setRunning(false);
      message.error(`${text("Could not start PP test", "PP 测试启动失败")}: ${detail}`);
    } finally {
      setBusy(false);
    }
  };

  const stop = async () => {
    setBusy(true);
    setLocalError(null);
    try {
      await api.sixMotorPpStop();
      applyView(await api.sixMotorPpGetState());
      message.success(text("All six motors are disabled", "6 台电机均已失能"));
    } catch (error) {
      const detail = errMsg(error);
      setLocalError(detail);
      message.error(`${text("Stop/disable failed", "停止/失能失败")}: ${detail}`);
    } finally {
      setBusy(false);
    }
  };

  const changeWaypoint = (waypointIndex: number, motorIndex: number, value: number) => {
    setRequest((current) => ({
      ...current,
      waypoints: current.waypoints.map((waypoint, row) =>
        row === waypointIndex
          ? waypoint.map((position, column) => (column === motorIndex ? value : position))
          : waypoint,
      ),
    }));
  };
  const changeRequest = <K extends keyof SixMotorPpRequest>(
    key: K,
    value: SixMotorPpRequest[K],
  ) => setRequest((current) => ({ ...current, [key]: value }));

  const waypointLabels = [
    text("Initial", "初始位置"),
    text("Position 1", "位置 1"),
    text("Position 2", "位置 2"),
    text("Position 3", "位置 3"),
  ];
  const activeWaypoint = view?.active_waypoint_index ?? 0;
  const activeMotorId =
    view?.active_motor_index == null ? null : SIX_MOTOR_PP_IDS[view.active_motor_index];
  const hasSession = (view?.motors.length ?? 0) === SIX_MOTOR_PP_IDS.length;
  const canStart = connected && unavailableReasons.length === 0 && !busy && !running;
  const canStop = connected && hasSession && view?.phase !== "idle" && view?.phase !== "stopped";

  const motorColumns: ColumnsType<MotorRow> = [
    {
      title: text("Motor", "电机"),
      key: "motor",
      width: 130,
      render: (_, row) => (
        <Space direction="vertical" size={0}>
          <Typography.Text strong>ID {row.nid}</Typography.Text>
          <Typography.Text code type="secondary">
            {nid2hex(row.nid)}{row.device ? ` · ${row.device.friendly_name}` : ""}
          </Typography.Text>
        </Space>
      ),
    },
    {
      title: text("Online", "在线"),
      key: "online",
      width: 80,
      render: (_, row) => <OnlineTag online={row.live?.online ?? row.device?.online === true} />,
    },
    {
      title: text("Sequence", "顺序状态"),
      key: "sequence",
      width: 135,
      render: (_, row) => renderSequenceState(row, text),
    },
    {
      title: text("Expected (Rev)", "预期位置（Rev）"),
      key: "expected",
      align: "right",
      width: 160,
      render: (_, row) => (
        <Typography.Text className="six-motor-pp__expected" code>
          {formatPosition(row.live?.target_position_rev ?? request.waypoints[activeWaypoint]?.[row.nid - 1])}
        </Typography.Text>
      ),
    },
    {
      title: text("Encoder (Rev)", "编码器位置（Rev）"),
      key: "actual",
      align: "right",
      width: 190,
      render: (_, row) => (
        <Typography.Text className="six-motor-pp__encoder" code>
          {formatPosition(row.live?.actual_position_rev)}
        </Typography.Text>
      ),
    },
    {
      title: text("Actual − expected (Rev)", "实际 − 预期（Rev）"),
      key: "error",
      align: "right",
      width: 190,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.position_error_rev)}</Typography.Text>,
    },
    {
      title: text("Velocity (Rev/s)", "速度（Rev/s）"),
      key: "velocity",
      align: "right",
      width: 150,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.actual_velocity_rev_per_s)}</Typography.Text>,
    },
    {
      title: text("Feedback age", "反馈帧龄"),
      key: "age",
      align: "right",
      width: 110,
      render: (_, row) => row.live?.feedback_age_ms == null ? "—" : `${row.live.feedback_age_ms} ms`,
    },
  ];

  const waypointRows: WaypointRow[] = request.waypoints.map((positions, index) => ({
    key: index,
    label: waypointLabels[index],
    positions,
  }));
  const waypointColumns: ColumnsType<WaypointRow> = [
    {
      title: text("Waypoint", "轨迹点"),
      dataIndex: "label",
      key: "label",
      fixed: "left",
      width: 120,
      render: (label: string, row) => (
        <Space>
          <Typography.Text strong>{label}</Typography.Text>
          {hasSession && activeWaypoint === row.key && (
            <Tag color="processing">{text("Active", "当前")}</Tag>
          )}
        </Space>
      ),
    },
    ...SIX_MOTOR_PP_IDS.map(
      (nid, motorIndex): ColumnsType<WaypointRow>[number] => ({
        title: `ID ${nid} (Rev)`,
        key: `motor-${nid}`,
        align: "right",
        width: 145,
        render: (_, row) => (
          <InputNumber
            aria-label={`${row.label} ID ${nid}`}
            value={row.positions[motorIndex]}
            min={-128}
            max={128 - 2 ** -24}
            step={0.01}
            precision={9}
            disabled={busy || running}
            onChange={(value) => changeWaypoint(row.key, motorIndex, value ?? 0)}
          />
        ),
      }),
    ),
  ];

  const error = localError ?? view?.last_error ?? null;
  return (
    <Space className="six-motor-pp" direction="vertical" size={12}>
      <Alert
        type={unavailableReasons.length === 0 ? "info" : "warning"}
        showIcon
        message={text(
          "IDs 1–6 · Profile Position · strictly sequential motion",
          "ID 1–6 · PP 位置模式 · 严格逐电机运动",
        )}
        description={
          unavailableReasons.length > 0
            ? unavailableReasons.join("；")
            : text(
                "All axes enter PP while holding their measured startup positions. Only motor 1 receives a moving target; motor 2 starts after motor 1 has reached and settled, continuing through motor 6.",
                "所有轴先在实时编码器位置进入 PP 保持；仅 1 号收到运动目标，1 号到位并稳定后才启动 2 号，依次执行到 6 号。",
              )
        }
      />

      {error && (
        <Alert
          type="error"
          closable
          showIcon
          message={text("PP test error", "PP 测试错误")}
          description={error}
          onClose={() => setLocalError(null)}
        />
      )}

      <Card
        size="small"
        title={text("Six encoder positions", "六电机编码器位置")}
        extra={
          <Space wrap>
            <Tag color={running ? "processing" : view?.phase === "error" ? "error" : "default"}>
              {phaseLabel(view?.phase, text)}
            </Tag>
            <Typography.Text type="secondary">
              {activeMotorId == null
                ? text("All axes holding", "全部轴保持")
                : text(`Active: motor ${activeMotorId}`, `当前：${activeMotorId} 号电机`)}
            </Typography.Text>
          </Space>
        }
      >
        <Table
          rowKey="nid"
          columns={motorColumns}
          dataSource={motorRows}
          pagination={false}
          size="middle"
          scroll={{ x: 1_245 }}
          rowClassName={(row) => row.live?.active ? "six-motor-pp__motor--active" : ""}
        />
        <Descriptions size="small" bordered column={3} style={{ marginTop: 12 }}>
          <Descriptions.Item label={text("Waypoint", "当前轨迹点")}>
            {waypointLabels[activeWaypoint] ?? "—"}
          </Descriptions.Item>
          <Descriptions.Item label={text("Completed cycles", "完整循环")}>
            {view?.completed_cycles ?? 0}
          </Descriptions.Item>
          <Descriptions.Item label={text("TPDO rate", "TPDO 频率")}>
            {view?.pdo_rate_hz ? `${view.pdo_rate_hz.toFixed(1)} Hz` : formatPdoRateHz(request.tpdo_event_timer_ms) + " Hz"}
          </Descriptions.Item>
        </Descriptions>
      </Card>

      <Card size="small" title={text("Position sequence", "位置序列（按 ID）")}>
        <Table
          rowKey="key"
          columns={waypointColumns}
          dataSource={waypointRows}
          pagination={false}
          size="small"
          scroll={{ x: 990 }}
          rowClassName={(row) =>
            hasSession && row.key === activeWaypoint ? "six-motor-pp__waypoint--active" : ""
          }
        />
        <Typography.Text type="secondary" style={{ display: "block", marginTop: 10 }}>
          {text(
            "The latest committed four-waypoint sequence is retained. At every waypoint the backend commands motors 1 → 6 one at a time.",
            "保留最新提交中的四组位置；每个轨迹点都由后端按 1 → 6 逐台下发并等待到位。",
          )}
        </Typography.Text>
      </Card>

      <Card size="small" title={text("PP parameters", "PP 参数与到位判定")}>
        <Space wrap align="end">
          <IntegerField
            label={text("TPDO period (ms)", "TPDO 周期（ms）")}
            value={request.tpdo_event_timer_ms}
            min={1}
            max={100}
            disabled={busy || running}
            set={(value) => changeRequest("tpdo_event_timer_ms", value)}
          />
          <IntegerField
            label={text("Max torque (‰)", "最大力矩（‰）")}
            value={request.max_torque_permille}
            min={0}
            max={1_000}
            disabled={busy || running}
            set={(value) => changeRequest("max_torque_permille", value)}
          />
          <NumericField
            label={text("Profile velocity (Rev/s)", "规划速度（Rev/s）")}
            value={request.profile.velocity_rev_per_s}
            min={0.001}
            max={5}
            step={0.01}
            disabled={busy || running}
            set={(value) => setRequest((current) => ({
              ...current,
              profile: { ...current.profile, velocity_rev_per_s: value },
            }))}
          />
          <NumericField
            label={text("Acceleration (Rev/s²)", "加速度（Rev/s²）")}
            value={request.profile.acceleration_rev_per_s2}
            min={0.001}
            max={50}
            step={0.01}
            disabled={busy || running}
            set={(value) => setRequest((current) => ({
              ...current,
              profile: { ...current.profile, acceleration_rev_per_s2: value },
            }))}
          />
          <NumericField
            label={text("Deceleration (Rev/s²)", "减速度（Rev/s²）")}
            value={request.profile.deceleration_rev_per_s2}
            min={0.001}
            max={50}
            step={0.01}
            disabled={busy || running}
            set={(value) => setRequest((current) => ({
              ...current,
              profile: { ...current.profile, deceleration_rev_per_s2: value },
            }))}
          />
          <NumericField
            label={text("Position tolerance (Rev)", "到位位置容差（Rev）")}
            value={request.arrival_position_tolerance_rev}
            min={0.000001}
            max={1}
            step={0.001}
            precision={6}
            disabled={busy || running}
            set={(value) => changeRequest("arrival_position_tolerance_rev", value)}
          />
          <NumericField
            label={text("Velocity tolerance (Rev/s)", "到位速度容差（Rev/s）")}
            value={request.arrival_velocity_tolerance_rev_per_s}
            min={0.000001}
            max={5}
            step={0.01}
            precision={6}
            disabled={busy || running}
            set={(value) => changeRequest("arrival_velocity_tolerance_rev_per_s", value)}
          />
          <IntegerField
            label={text("Settle (ms)", "稳定时间（ms）")}
            value={request.settle_time_ms}
            min={20}
            max={5_000}
            disabled={busy || running}
            set={(value) => changeRequest("settle_time_ms", value)}
          />
          <IntegerField
            label={text("Axis timeout (ms)", "单轴超时（ms）")}
            value={request.movement_timeout_ms}
            min={1_000}
            max={600_000}
            disabled={busy || running}
            set={(value) => changeRequest("movement_timeout_ms", value)}
          />
          <IntegerField
            label={text("Waypoint dwell (ms)", "轨迹点停留（ms）")}
            value={request.waypoint_dwell_ms}
            min={0}
            max={60_000}
            disabled={busy || running}
            set={(value) => changeRequest("waypoint_dwell_ms", value)}
          />
        </Space>
      </Card>

      <Card size="small" className="app-command-card">
        <Space wrap>
          <Button type="primary" size="large" disabled={!canStart} loading={busy && !running} onClick={() => void start()}>
            {text("Start sequential PP test", "启动逐轴 PP 测试")}
          </Button>
          <Button danger size="large" disabled={!canStop || busy} onClick={() => void stop()}>
            {text("Stop and disable all", "停止并全部失能")}
          </Button>
          {!connected && <Typography.Text type="warning">{text("Connect first", "请先连接")}</Typography.Text>}
        </Space>
      </Card>
    </Space>
  );
}

function renderSequenceState(
  row: MotorRow,
  text: (en: string, zh: string) => string,
) {
  if (row.device == null) return <Tag>{text("Not discovered", "未发现")}</Tag>;
  if (row.device.device_type !== "meow_motor") return <Tag color="error">{text("Unsupported", "不支持")}</Tag>;
  if (!row.device.online) return <Tag color="warning">{text("Offline", "离线")}</Tag>;
  if (row.live?.error) {
    return (
      <Tooltip title={row.live.error}>
        <Tag color="error">{text("Fault", "故障")}</Tag>
      </Tooltip>
    );
  }
  const labels: Record<string, [string, string, string]> = {
    commanding: ["Commanding", "下发目标", "processing"],
    moving: ["PP moving", "PP 运动中", "processing"],
    settling: ["Settling", "到位确认", "processing"],
    waiting: ["Waiting", "等待前一轴", "default"],
    reached: ["Reached", "已到位", "success"],
    holding: ["Holding", "保持", "success"],
    disabled: ["Disabled", "已失能", "default"],
    "disable-failed": ["Disable failed", "失能失败", "error"],
  };
  const [en, zh, color] = labels[row.live?.sequence_state ?? "waiting"] ?? ["Standby", "待机", "default"];
  return <Tag color={color}>{text(en, zh)}</Tag>;
}

function phaseLabel(phase: string | undefined, text: (en: string, zh: string) => string) {
  const labels: Record<string, [string, string]> = {
    idle: ["Idle", "待机"],
    moving: ["Sequential PP running", "逐轴 PP 运行中"],
    dwell: ["Waypoint dwell", "轨迹点停留"],
    stopping: ["Stopping", "停止中"],
    stopped: ["Stopped", "已停止"],
    error: ["Safety stop", "安全停机"],
  };
  const [en, zh] = labels[phase ?? "idle"] ?? [phase ?? "Idle", phase ?? "待机"];
  return text(en, zh);
}

function formatPosition(value: number | null | undefined) {
  return value == null || !Number.isFinite(value) ? "—" : value.toFixed(6);
}

function formatSigned(value: number | null | undefined) {
  if (value == null || !Number.isFinite(value)) return "—";
  return `${value >= 0 ? "+" : ""}${value.toFixed(6)}`;
}

function Field({ label, children }: { label: ReactNode; children: ReactNode }) {
  return (
    <div>
      <div style={{ color: "#8a93a3", fontSize: 12, marginBottom: 3 }}>{label}</div>
      {children}
    </div>
  );
}

function NumericField({
  label,
  value,
  min,
  max,
  step,
  precision = 6,
  disabled,
  set,
}: {
  label: ReactNode;
  value: number;
  min: number;
  max: number;
  step: number;
  precision?: number;
  disabled: boolean;
  set: (value: number) => void;
}) {
  return (
    <Field label={label}>
      <InputNumber
        value={value}
        min={min}
        max={max}
        step={step}
        precision={precision}
        disabled={disabled}
        onChange={(next) => set(next ?? 0)}
      />
    </Field>
  );
}

function IntegerField({
  label,
  value,
  min,
  max,
  disabled,
  set,
}: {
  label: ReactNode;
  value: number;
  min: number;
  max: number;
  disabled: boolean;
  set: (value: number) => void;
}) {
  return (
    <Field label={label}>
      <InputNumber
        value={value}
        min={min}
        max={max}
        step={1}
        precision={0}
        disabled={disabled}
        onChange={(next) => set(Math.round(next ?? 0))}
      />
    </Field>
  );
}
