import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import {
  Alert,
  App,
  Button,
  Card,
  Descriptions,
  InputNumber,
  Select,
  Space,
  Switch,
  Table,
  Tag,
  Tooltip,
  Typography,
} from "antd";
import type { ColumnsType } from "antd/es/table";
import { api, errMsg } from "../api";
import { nid2hex } from "../format";
import { useI18n } from "../i18n";
import {
  SIX_MOTOR_MIT_CONTROL_RATE_HZ,
  SIX_MOTOR_MIT_IDS,
  cloneSixMotorMitRequest,
  createSixMotorMitDefaultRequest,
  sixMotorMitSessionNeedsStop,
  validateSixMotorMitRequest,
} from "../sixMotorMitTest";
import { OnlineTag } from "../tags";
import type {
  MotorInfo,
  SixMotorMitMotorView,
  SixMotorMitRequest,
  SixMotorMitTuning,
  SixMotorMitView,
} from "../types";
import "./SixMotorMitTestPanel.css";

const STATE_POLL_MS = 50;

interface MotorRow {
  nid: number;
  device: MotorInfo | null;
  live: SixMotorMitMotorView | null;
}

interface WaypointRow {
  key: number;
  label: string;
  positions: number[];
}

interface TuningRow {
  key: number;
  nid: number;
  tuning: SixMotorMitTuning;
  live: SixMotorMitMotorView | null;
}

type TuningNumberKey = keyof SixMotorMitTuning;
type MitOperation = "idle" | "starting" | "updating" | "stopping";

export function SixMotorMitTestPanel({
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
  const { message, modal } = App.useApp();
  const { lang } = useI18n();
  const text = useCallback((en: string, zh: string) => (lang === "zh" ? zh : en), [lang]);
  const [busy, setBusyState] = useState(false);
  const busyDepthRef = useRef(0);
  const [operation, setOperationState] = useState<MitOperation>("idle");
  const operationRef = useRef<MitOperation>("idle");
  const [running, setRunningState] = useState(false);
  const [sessionActive, setSessionActiveState] = useState(false);
  // A recovery/error session can have no trajectory loop while still owning
  // a 500 Hz safe-hold stream for an axis that did not confirm Disabled.
  const sessionActiveRef = useRef(false);
  const pollPendingRef = useRef(false);
  const [request, setRequest] = useState<SixMotorMitRequest>(() =>
    createSixMotorMitDefaultRequest(),
  );
  const [view, setView] = useState<SixMotorMitView | null>(null);
  const [dirty, setDirty] = useState(false);
  const [localError, setLocalError] = useState<string | null>(null);

  const setBusy = useCallback(
    (value: boolean) => {
      busyDepthRef.current = value
        ? busyDepthRef.current + 1
        : Math.max(0, busyDepthRef.current - 1);
      const next = busyDepthRef.current > 0;
      setBusyState(next);
      onBusyChange?.(next);
    },
    [onBusyChange],
  );

  const setRunning = useCallback(
    (value: boolean, sessionActive = value) => {
      sessionActiveRef.current = sessionActive;
      setRunningState(value);
      setSessionActiveState(sessionActive);
      onRunningChange?.(sessionActive);
    },
    [onRunningChange],
  );

  const setOperation = useCallback((value: MitOperation) => {
    operationRef.current = value;
    setOperationState(value);
  }, []);

  const applyView = useCallback(
    (next: SixMotorMitView) => {
      setView(next);
      const sessionActive = sixMotorMitSessionNeedsStop(next);
      setRunning(next.running, sessionActive);
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
      SIX_MOTOR_MIT_IDS.map((nid) => ({
        nid,
        device: deviceByNid.get(nid) ?? null,
        live: liveByNid.get(nid) ?? null,
      })),
    [deviceByNid, liveByNid],
  );

  const unavailableReasons = useMemo(() => {
    const reasons: string[] = [];
    for (const nid of SIX_MOTOR_MIT_IDS) {
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
        .sixMotorMitGetState()
        .then((next) => {
          if (alive) applyView(next);
        })
        .catch((error) => {
          if (alive && sessionActiveRef.current) setLocalError(errMsg(error));
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
      if (sessionActiveRef.current) void api.sixMotorMitStop().catch(() => {});
    },
    [onBusyChange, onRunningChange],
  );

  const validate = useCallback(() => {
    const copy = cloneSixMotorMitRequest(request);
    validateSixMotorMitRequest(copy);
    return copy;
  }, [request]);

  const start = async () => {
    let nextRequest: SixMotorMitRequest;
    try {
      nextRequest = validate();
    } catch (error) {
      message.error(errMsg(error));
      return;
    }
    setOperation("starting");
    setBusy(true);
    setLocalError(null);
    try {
      const next = await api.sixMotorMitStart(nextRequest);
      applyView(next);
      if (nextRequest.capture_gravity_park_on_start) {
        const capturedByNid = new Map(
          next.motors.map((motor) => [motor.node_id, motor.gravity_park_position_rev]),
        );
        setRequest((current) => ({
          ...current,
          tuning: current.tuning.map((tuning, index) => {
            const captured = capturedByNid.get(SIX_MOTOR_MIT_IDS[index]);
            return captured != null && Number.isFinite(captured)
              ? { ...tuning, gravity_park_position_rev: captured }
              : tuning;
          }),
        }));
      }
      setDirty(false);
      message.success(
        text(
          "500 Hz uncompressed-MIT sequence started",
          "500 Hz 非压缩 MIT 顺序运动已启动",
        ),
      );
    } catch (error) {
      const detail = errMsg(error);
      setLocalError(detail);
      // A failed start may deliberately leave a registered recovery session
      // holding one or more axes until Stop confirms disable. Fetch it now so
      // navigation/unmount cleanup cannot mistake that session for idle.
      const latest = await api.sixMotorMitGetState().catch(() => null);
      if (latest) applyView(latest);
      // If the follow-up read itself fails, conservatively keep lifecycle
      // ownership active. A later poll can prove the backend idle; until then
      // navigation/unmount must still issue Stop.
      else setRunning(false, true);
      message.error(`${text("Could not start test", "测试启动失败")}: ${detail}`);
    } finally {
      setBusy(false);
      if (operationRef.current === "starting") setOperation("idle");
    }
  };

  const updateRuntime = async () => {
    let nextRequest: SixMotorMitRequest;
    try {
      nextRequest = validate();
    } catch (error) {
      message.error(errMsg(error));
      return;
    }
    setOperation("updating");
    setBusy(true);
    setLocalError(null);
    try {
      const next = await api.sixMotorMitUpdate(nextRequest);
      if (operationRef.current === "stopping") return;
      applyView(next);
      setDirty(false);
      message.success(
        text(
          "Runtime parameters applied; changed SDO fields were read back and verified",
          "运行参数已应用；发生变化的 SDO 字段已回读校验",
        ),
      );
    } catch (error) {
      if (operationRef.current === "stopping") return;
      const detail = errMsg(error);
      setLocalError(detail);
      const latest = await api.sixMotorMitGetState().catch(() => null);
      if (latest) applyView(latest);
      message.error(`${text("Runtime update failed", "运行时更新失败")}: ${detail}`);
    } finally {
      setBusy(false);
      if (operationRef.current === "updating") setOperation("idle");
    }
  };

  const stop = async () => {
    if (operationRef.current === "starting" || operationRef.current === "stopping") return;
    setOperation("stopping");
    setBusy(true);
    setLocalError(null);
    try {
      await api.sixMotorMitStop();
      const latest = await api.sixMotorMitGetState();
      applyView(latest);
      message.success(text("All six motors are disabled", "6 台电机均已失能"));
    } catch (error) {
      const detail = errMsg(error);
      setLocalError(detail);
      message.error(`${text("Stop/disable failed", "停止/失能失败")}: ${detail}`);
    } finally {
      setBusy(false);
      setOperation("idle");
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
    setDirty(true);
  };

  const changeTuning = (motorIndex: number, key: TuningNumberKey, value: number) => {
    setRequest((current) => ({
      ...current,
      tuning: current.tuning.map((tuning, index) =>
        index === motorIndex ? { ...tuning, [key]: value } : tuning,
      ),
    }));
    setDirty(true);
  };

  const changeRequest = <K extends keyof SixMotorMitRequest>(
    key: K,
    value: SixMotorMitRequest[K],
  ) => {
    setRequest((current) => ({ ...current, [key]: value }));
    setDirty(true);
  };

  const requestGravityEnable = (enabled: boolean) => {
    if (!enabled) {
      changeRequest("gravity_enabled", false);
      return;
    }
    modal.confirm({
      title: text(
        "Enable gravity compensation with these motor directions?",
        "按当前电机方向启用重力补偿？",
      ),
      content: text(
        "The ROS2 repository contains no validated real-hardware direction profile. A wrong sign can push a loaded joint downward. Verify one axis at a time with a low scale and clamp, keep physical power removal ready, then increase toward the prefilled values.",
        "ROS2 仓库中没有已验证的真机方向配置；方向错误会将带载关节向下助推。请先用较低比例和夹紧值逐轴验证，保持可随时物理断电，再逐步提高到预填值。",
      ),
      okText: text("I understand — enable", "我已知晓，启用"),
      okButtonProps: { danger: true },
      cancelText: text("Keep disabled", "保持关闭"),
      onOk: () => changeRequest("gravity_enabled", true),
    });
  };

  const changeVectorRequest = (
    key: "gravity_vector_m_per_s2" | "payload_center_of_mass_m",
    index: number,
    value: number,
  ) => {
    setRequest((current) => {
      const vector: [number, number, number] = [...current[key]];
      vector[index] = value;
      return { ...current, [key]: vector };
    });
    setDirty(true);
  };

  const waypointLabels = [
    text("Initial", "初始位置"),
    text("Position 1", "位置 1"),
    text("Position 2", "位置 2"),
    text("Position 3", "位置 3"),
  ];
  const activeWaypoint = view?.active_waypoint_index ?? 0;
  const activeMotorId =
    view?.active_motor_index == null ? null : SIX_MOTOR_MIT_IDS[view.active_motor_index];
  const hasSession = (view?.motors.length ?? 0) === SIX_MOTOR_MIT_IDS.length;
  const appliedGravityEnabled = view?.gravity_enabled ?? false;
  const gravityDraftPending =
    running && request.gravity_enabled !== appliedGravityEnabled;
  const canStart =
    connected && unavailableReasons.length === 0 && !busy && !running && !sessionActive;
  const canStop = connected && sessionActive;

  const motorColumns: ColumnsType<MotorRow> = [
    {
      title: text("Motor", "电机"),
      key: "motor",
      width: 120,
      render: (_, row) => (
        <Space direction="vertical" size={0}>
          <Typography.Text strong>ID {row.nid}</Typography.Text>
          <Typography.Text code type="secondary">{nid2hex(row.nid)}</Typography.Text>
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
      width: 130,
      render: (_, row) => renderSequenceState(row, text),
    },
    {
      title: "Pdes (Rev)",
      key: "pdes",
      align: "right",
      width: 130,
      render: (_, row) => (
        <Typography.Text className="six-motor-mit__expected" code>
          {formatNumber(row.live?.command_position_rev)}
        </Typography.Text>
      ),
    },
    {
      title: "Vdes (Rev/s)",
      key: "vdes",
      align: "right",
      width: 130,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.command_velocity_rev_per_s)}</Typography.Text>,
    },
    {
      title: text("Encoder (Rev)", "编码器（Rev）"),
      key: "encoder",
      align: "right",
      width: 170,
      render: (_, row) => (
        <Typography.Text className="six-motor-mit__encoder" code>
          {formatNumber(row.live?.actual_position_rev)}
        </Typography.Text>
      ),
    },
    {
      title: text("Actual − waypoint (Rev)", "实际 − 轨迹点（Rev）"),
      key: "error",
      align: "right",
      width: 190,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.position_error_rev)}</Typography.Text>,
    },
    {
      title: text("Measured velocity", "实测速度"),
      key: "velocity",
      align: "right",
      width: 150,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.actual_velocity_rev_per_s)}</Typography.Text>,
    },
    {
      title: text("URDF q (rad)", "URDF q（rad）"),
      key: "joint-position",
      align: "right",
      width: 145,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.joint_position_rad)}</Typography.Text>,
    },
    {
      title: text("URDF G(q) (Nm)", "URDF G(q)（Nm）"),
      key: "model-gravity",
      align: "right",
      width: 155,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.model_gravity_torque_nm)}</Typography.Text>,
    },
    {
      title: text("Motor gravity target", "电机重力目标"),
      key: "gravity-target",
      align: "right",
      width: 160,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.gravity_target_torque_nm)}</Typography.Text>,
    },
    {
      title: text("Applied gravity Tff", "已应用重力 Tff"),
      key: "gravity-applied",
      align: "right",
      width: 155,
      render: (_, row) => (
        <Space size={4}>
          <Typography.Text code>{formatSigned(row.live?.gravity_torque_nm)}</Typography.Text>
          {row.live?.gravity_limited && <Tag color="warning">{text("clamped", "已夹紧")}</Tag>}
        </Space>
      ),
    },
    {
      title: text("Requested physical Tff", "请求物理 Tff"),
      key: "total-tff",
      align: "right",
      width: 150,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.command_tff_nm)}</Typography.Text>,
    },
    {
      title: text("Wire Tff (Nm)", "线值 Tff（Nm）"),
      key: "wire-tff",
      align: "right",
      width: 145,
      render: (_, row) => <Typography.Text code>{formatSigned(row.live?.wire_tff_nm)}</Typography.Text>,
    },
    {
      title: text("Tff ceiling / peak", "Tff 上限 / 峰值"),
      key: "tff-ceiling",
      align: "right",
      width: 175,
      render: (_, row) => row.live == null ? "—" : (
        <Typography.Text code>
          {formatNumber(row.live.tff_limit_nm, 3)} / {formatNumber(row.live.peak_torque_nm, 3)} Nm
        </Typography.Text>
      ),
    },
    {
      title: text("Tff utilization", "Tff 使用率"),
      key: "tff-utilization",
      align: "right",
      width: 190,
      render: (_, row) => row.live == null ? "—" : (
        <Space size={4} wrap>
          <Typography.Text code>{formatNumber(row.live.tff_utilization_percent, 1)}%</Typography.Text>
          <Tooltip
            title={text(
              `Recommended minimum for Tff headroom plus full P/D reserve: ${row.live.recommended_min_torque_permille}‰`,
              `Tff 余量加完整 P/D 预算的建议最小值：${row.live.recommended_min_torque_permille}‰`,
            )}
          >
            <Tag color={row.live.tff_limited ? "error" : "default"}>
              {row.live.tff_limited
                ? text("SATURATED", "已饱和")
                : `${text("min", "建议")} ${row.live.recommended_min_torque_permille}‰`}
            </Tag>
          </Tooltip>
        </Space>
      ),
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
          {activeWaypoint === row.key && hasSession && <Tag color="processing">{text("Active", "当前")}</Tag>}
        </Space>
      ),
    },
    ...SIX_MOTOR_MIT_IDS.map(
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
            disabled={busy}
            onChange={(value) => changeWaypoint(row.key, motorIndex, value ?? 0)}
          />
        ),
      }),
    ),
  ];

  const tuningRows: TuningRow[] = request.tuning.map((tuning, index) => ({
    key: index,
    nid: SIX_MOTOR_MIT_IDS[index],
    tuning,
    live: liveByNid.get(SIX_MOTOR_MIT_IDS[index]) ?? null,
  }));
  const tuningColumns: ColumnsType<TuningRow> = [
    {
      title: text("Motor", "电机"),
      dataIndex: "nid",
      key: "nid",
      width: 75,
      render: (nid: number) => <Typography.Text strong>ID {nid}</Typography.Text>,
    },
    tuningInputColumn("Tff (Nm)", "tff_nm", -1_000, 1_000, 0.01, 4),
    {
      title: text("Direction", "方向"),
      key: "gravity_direction",
      align: "center",
      width: 105,
      render: (_, row) => (
        <Select<number>
          aria-label={`ID ${row.nid} gravity_direction`}
          value={row.tuning.gravity_direction}
          disabled={busy}
          style={{ width: 82 }}
          options={[
            { value: 1, label: "+1" },
            { value: -1, label: "−1" },
          ]}
          onChange={(value) => changeTuning(row.key, "gravity_direction", value)}
        />
      ),
    },
    tuningInputColumn(
      text("Park encoder (Rev)", "断电位编码器（Rev）"),
      "gravity_park_position_rev",
      -128,
      128 - 2 ** -24,
      0.01,
      6,
    ),
    tuningInputColumn(
      text("Park joint q (rad)", "断电位关节 q（rad）"),
      "gravity_park_angle_rad",
      -2 * Math.PI,
      2 * Math.PI,
      0.01,
      6,
    ),
    tuningInputColumn(text("Gravity scale", "重力比例"), "gravity_scale", 0, 1.5, 0.05, 3),
    tuningInputColumn(
      text("Gravity clamp (Nm)", "重力夹紧（Nm）"),
      "gravity_max_torque_nm",
      0,
      1_000,
      0.1,
      3,
    ),
    tuningInputColumn("Kp (Nm/rad)", "kp_nm_per_rad", 0, 10_000, 0.1, 4),
    tuningInputColumn("Kd (Nm·s/rad)", "kd_nm_s_per_rad", 0, 10_000, 0.1, 4),
    tuningInputColumn(text("P/D limit (‰)", "P/D 限幅（‰）"), "kp_kd_limit_permille", 0, 1_000, 10, 0),
    tuningInputColumn(text("Max torque (‰)", "最大力矩（‰）"), "max_torque_permille", 0, 1_000, 10, 0),
    {
      title: "0x4102:07",
      key: "factor",
      align: "right",
      width: 125,
      render: (_, row) => <Typography.Text code>{formatNumber(row.live?.kp_kd_factor, 8)}</Typography.Text>,
    },
    {
      title: text("Wire Kp / Kd", "线值 Kp / Kd"),
      key: "wire",
      align: "right",
      width: 135,
      render: (_, row) => row.live == null ? "—" : <Typography.Text code>{row.live.kp_raw} / {row.live.kd_raw}</Typography.Text>,
    },
  ];

  function tuningInputColumn(
    title: string,
    key: TuningNumberKey,
    min: number,
    max: number,
    step: number,
    precision: number,
  ): ColumnsType<TuningRow>[number] {
    return {
      title,
      key,
      align: "right",
      width: 155,
      render: (_, row) => (
        <InputNumber
          aria-label={`ID ${row.nid} ${key}`}
          value={row.tuning[key]}
          min={min}
          max={max}
          step={step}
          precision={precision}
          disabled={busy}
          onChange={(value) => changeTuning(row.key, key, value ?? 0)}
        />
      ),
    };
  }

  const error = localError ?? view?.last_error ?? null;

  return (
    <Space className="six-motor-mit" direction="vertical" size={12}>
      <Alert
        type={unavailableReasons.length === 0 ? "info" : "warning"}
        showIcon
        message={text(
          "IDs 1–6 · canonical uncompressed MIT · sequential motion",
          "ID 1–6 · 标准非压缩 MIT · 逐电机运动",
        )}
        description={
          unavailableReasons.length > 0
            ? unavailableReasons.join("；")
            : text(
                "The backend starts from each live encoder position. Motor 2 moves only after motor 1 reaches and settles at its waypoint; the same rule continues through motor 6.",
                "后端从各电机实时编码器位置平滑起步；1 号到位并稳定后才启动 2 号，依次执行到 6 号。",
              )
        }
      />

      {error && (
        <Alert
          type="error"
          closable
          showIcon
          message={text("MIT test error", "MIT 测试错误")}
          description={error}
          onClose={() => setLocalError(null)}
        />
      )}

      <Card
        size="small"
        title={text("URDF zero-gravity compensation", "URDF 零重力补偿")}
        extra={
          <Space size={4} wrap>
            <Tag color={(running ? appliedGravityEnabled : request.gravity_enabled) ? "success" : "default"}>
              {running
                ? appliedGravityEnabled
                  ? text("Applied: On", "已生效：开")
                  : text("Applied: Off", "已生效：关")
                : request.gravity_enabled
                  ? text("Start setting: On", "启动设置：开")
                  : text("Start setting: Off", "启动设置：关")}
            </Tag>
            {gravityDraftPending && (
              <Tag color="warning">
                {request.gravity_enabled
                  ? text("Pending: On", "待应用：开")
                  : text("Pending: Off", "待应用：关")}
              </Tag>
            )}
          </Space>
        }
      >
        <Alert
          type="warning"
          showIcon
          message={text(
            "Place the unpowered arm in the defined park pose before Start",
            "启动前请将断电机械臂摆到指定基准姿态",
          )}
          description={text(
            "Required URDF pose: q = [0, −1.570, 3.140, 0, 0, 0] rad. When capture is enabled, Start samples all six live encoders before any motor is enabled and writes those values back into the Park encoder fields. The public ROS2 repository has no validated real-hardware direction profile, so gravity starts OFF. Verify each configured direction with a low scale/clamp before enabling it: a wrong sign can assist the arm falling.",
            "要求的 URDF 姿态：q = [0, −1.570, 3.140, 0, 0, 0] rad。启用捕获后，“启动”会在任何电机使能前采集 6 轴实时编码器，并将结果回填到“断电位编码器”字段。公开 ROS2 仓库没有已验证的真机方向配置，因此重力补偿默认关闭；请先用较低比例/夹紧值逐轴确认方向再开启，符号错误会助推机械臂下坠。",
          )}
          style={{ marginBottom: 12 }}
        />
        <Space wrap align="end" size={[14, 10]}>
          <Field label={text("Gravity compensation", "重力补偿")}>
            <Switch
              checked={request.gravity_enabled}
              checkedChildren={text("On", "开")}
              unCheckedChildren={text("Off", "关")}
              disabled={busy}
              onChange={requestGravityEnable}
            />
          </Field>
          <Field label={text("Capture park encoders on Start", "启动时捕获断电位编码器")}>
            <Switch
              checked={request.capture_gravity_park_on_start}
              checkedChildren={text("Capture", "捕获")}
              unCheckedChildren={text("Use table", "使用表值")}
              disabled={busy}
              onChange={(value) => changeRequest("capture_gravity_park_on_start", value)}
            />
          </Field>
          {(["X", "Y", "Z"] as const).map((axis, index) => (
            <NumericField
              key={`gravity-${axis}`}
              label={`g${axis} · base (m/s²)`}
              value={request.gravity_vector_m_per_s2[index]}
              min={-20}
              max={20}
              step={0.01}
              precision={3}
              disabled={busy}
              set={(value) => changeVectorRequest("gravity_vector_m_per_s2", index, value)}
            />
          ))}
          <NumericField
            label={text("Tff transition rate (Nm/s)", "Tff 过渡速率（Nm/s）")}
            value={request.tff_slew_rate_nm_per_s}
            min={0.001}
            max={1_000}
            step={0.5}
            precision={3}
            disabled={busy}
            set={(value) => changeRequest("tff_slew_rate_nm_per_s", value)}
          />
          <NumericField
            label={text("Payload mass (kg)", "末端负载质量（kg）")}
            value={request.payload_mass_kg}
            min={0}
            max={100}
            step={0.05}
            precision={3}
            disabled={busy}
            set={(value) => changeRequest("payload_mass_kg", value)}
          />
          {(["X", "Y", "Z"] as const).map((axis, index) => (
            <NumericField
              key={`payload-com-${axis}`}
              label={`${text("Payload COM", "负载质心")} ${axis} · tip (m)`}
              value={request.payload_center_of_mass_m[index]}
              min={-10}
              max={10}
              step={0.005}
              precision={4}
              disabled={busy}
              set={(value) => changeVectorRequest("payload_center_of_mass_m", index, value)}
            />
          ))}
        </Space>
        <Descriptions size="small" bordered column={2} style={{ marginTop: 12 }}>
          <Descriptions.Item label={text("Dynamics model", "动力学模型")}>
            <Typography.Text code>{view?.gravity_model || "firefly_y6 · hex-arm-dynamics"}</Typography.Text>
          </Descriptions.Item>
          <Descriptions.Item label={text("Joint mapping", "关节映射")}>
            <Typography.Text code>q = qPark + direction × 2π × (encoder − parkEncoder)</Typography.Text>
          </Descriptions.Item>
          <Descriptions.Item label={text("Gravity command", "重力命令")} span={2}>
            <Typography.Text code>
              motor target = clamp(direction × scale × G(q), ±gravity clamp)
            </Typography.Text>
          </Descriptions.Item>
        </Descriptions>
        <Typography.Text type="secondary" style={{ display: "block", marginTop: 10 }}>
          {text(
            "Gravity, payload, mapping, clamps and MIT tuning remain editable while running; press Apply to blend toward the new Tff without a torque step. Capture itself occurs only on Start.",
            "运行中仍可修改重力、负载、映射、夹紧值和 MIT 参数；点击“应用运行参数”后，Tff 会按渐变速率过渡，避免力矩阶跃。编码器捕获仅在“启动”时执行。",
          )}
        </Typography.Text>
      </Card>

      <Card
        size="small"
        title={text("Live sequence and encoder feedback", "顺序运动与编码器实时反馈")}
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
          scroll={{ x: 2_650 }}
          rowClassName={(row) => row.live?.active ? "six-motor-mit__motor--active" : ""}
        />
        <Descriptions size="small" bordered column={4} style={{ marginTop: 12 }}>
          <Descriptions.Item label={text("Waypoint", "当前轨迹点")}>
            {waypointLabels[activeWaypoint] ?? "—"}
          </Descriptions.Item>
          <Descriptions.Item label={text("Completed cycles", "完整循环")}>
            {view?.completed_cycles ?? 0}
          </Descriptions.Item>
          <Descriptions.Item label="Pdes/Vdes RPDO1">
            {view?.rpdo_cob_id ? `0x${view.rpdo_cob_id.toString(16).toUpperCase()}` : "—"}
          </Descriptions.Item>
          <Descriptions.Item label="Tff RPDO2">
            {view?.tff_rpdo_cob_id ? `0x${view.tff_rpdo_cob_id.toString(16).toUpperCase()}` : "—"}
          </Descriptions.Item>
          <Descriptions.Item label={text("Control rate", "控制频率")}>
            {view?.pdo_rate_hz ?? SIX_MOTOR_MIT_CONTROL_RATE_HZ} Hz
          </Descriptions.Item>
        </Descriptions>
      </Card>

      <Card size="small" title={text("Position sequence (Pdes)", "位置序列（Pdes）")}>
        <Table
          rowKey="key"
          columns={waypointColumns}
          dataSource={waypointRows}
          pagination={false}
          size="small"
          scroll={{ x: 990 }}
          rowClassName={(row) => activeWaypoint === row.key && hasSession ? "six-motor-mit__waypoint--active" : ""}
        />
        <Typography.Text type="secondary" style={{ display: "block", marginTop: 10 }}>
          {text(
            "Pdes follows a 500 Hz trapezoidal trajectory instead of jumping to the next waypoint. Runtime edits take effect only after Apply.",
            "Pdes 以 500 Hz 梯形轨迹平滑变化，不会阶跃到下一位置；运行中修改后需点击“应用运行参数”。",
          )}
        </Typography.Text>
      </Card>

      <Card size="small" title={text("500 Hz Pdes/Vdes trajectory", "500 Hz Pdes/Vdes 轨迹参数")}>
        <Space wrap align="end">
          <NumericField
            label={text("Max Vdes (Rev/s)", "最大 Vdes（Rev/s）")}
            value={request.max_velocity_rev_per_s}
            min={0.001}
            max={5}
            step={0.01}
            disabled={busy}
            set={(value) => changeRequest("max_velocity_rev_per_s", value)}
          />
          <NumericField
            label={text("Acceleration (Rev/s²)", "加速度（Rev/s²）")}
            value={request.acceleration_rev_per_s2}
            min={0.001}
            max={50}
            step={0.01}
            disabled={busy}
            set={(value) => changeRequest("acceleration_rev_per_s2", value)}
          />
          <NumericField
            label={text("Deceleration (Rev/s²)", "减速度（Rev/s²）")}
            value={request.deceleration_rev_per_s2}
            min={0.001}
            max={50}
            step={0.01}
            disabled={busy}
            set={(value) => changeRequest("deceleration_rev_per_s2", value)}
          />
          <NumericField
            label={text("Position tolerance (Rev)", "到位位置容差（Rev）")}
            value={request.arrival_position_tolerance_rev}
            min={0.000001}
            max={1}
            step={0.001}
            precision={6}
            disabled={busy}
            set={(value) => changeRequest("arrival_position_tolerance_rev", value)}
          />
          <NumericField
            label={text("Velocity tolerance (Rev/s)", "到位速度容差（Rev/s）")}
            value={request.arrival_velocity_tolerance_rev_per_s}
            min={0.000001}
            max={5}
            step={0.01}
            precision={6}
            disabled={busy}
            set={(value) => changeRequest("arrival_velocity_tolerance_rev_per_s", value)}
          />
          <IntegerField
            label={text("Settle (ms)", "稳定时间（ms）")}
            value={request.settle_time_ms}
            min={20}
            max={5_000}
            disabled={busy}
            set={(value) => changeRequest("settle_time_ms", value)}
          />
          <IntegerField
            label={text("Axis timeout (ms)", "单轴超时（ms）")}
            value={request.movement_timeout_ms}
            min={1_000}
            max={600_000}
            disabled={busy}
            set={(value) => changeRequest("movement_timeout_ms", value)}
          />
          <IntegerField
            label={text("Waypoint dwell (ms)", "轨迹点停留（ms）")}
            value={request.waypoint_dwell_ms}
            min={0}
            max={60_000}
            disabled={busy}
            set={(value) => changeRequest("waypoint_dwell_ms", value)}
          />
        </Space>
      </Card>

      <Card size="small" title={text("Per-motor MIT and gravity tuning", "逐电机 MIT 与重力补偿参数")}>
        <Table
          rowKey="key"
          columns={tuningColumns}
          dataSource={tuningRows}
          pagination={false}
          size="small"
          scroll={{ x: 2_350 }}
        />
        <Typography.Text type="secondary" style={{ display: "block", marginTop: 10 }}>
          {text(
            "The URDF model computes G(q) from all six measured joint positions. Gravity uses the URDF base frame; payload COM uses the tip-link frame. Direction maps each joint torque into motor coordinates; scale and the per-axis gravity clamp set the target. Start and configuration changes blend physical Tff at the transition rate; after the blend settles, G(q) follows fresh feedback at 500 Hz without transition lag. The factory torque factor then produces wire Tff and 0x4572 enforces the final ceiling; when SATURATED, delivered torque is lower than requested. Gravity-only edits avoid SDO writes. Every accepted Apply restarts safely from fresh feedback at waypoint 1 / motor 1; an SDO tuning change also holds and rechecks all axes. Lowering a safety ceiling may clamp immediately.",
            "URDF 模型根据 6 轴实测关节位置计算 G(q)；重力向量使用 URDF base 坐标系，负载质心使用末端 tip-link 坐标系。方向用于映射到电机坐标，比例与逐轴重力夹紧值共同确定目标。启动及配置变更按过渡速率平滑切换物理 Tff；过渡稳定后，G(q) 以 500 Hz 跟随最新反馈，不再受过渡速率拖慢。随后乘出厂力矩系数得到线值 Tff，并由 0x4572 最终限幅；显示“已饱和”时，实际输出会低于请求值。仅修改重力参数时不会写 SDO。每次成功应用都会从最新反馈安全重启第 1 轨迹点/1 号电机；修改 SDO 参数时还会保持并重新检查全部轴。下调安全上限时可能立即夹紧。",
          )}
        </Typography.Text>
      </Card>

      <Card size="small" title={text("Protocol allocation", "协议分配")}>
        <Descriptions size="small" bordered column={2}>
          <Descriptions.Item label={text("Uncompressed selector", "非压缩选择器")}>
            <Typography.Text code>0x4103:01 = 0</Typography.Text>
          </Descriptions.Item>
          <Descriptions.Item label={text("Shared RPDO1 · 500 Hz", "共享 RPDO1 · 500 Hz")}>
            <Typography.Text code>0x4102:01 Pdes + 0x4102:02 Vdes</Typography.Text>
          </Descriptions.Item>
          <Descriptions.Item label={text("Shared RPDO2 · 500 Hz", "共享 RPDO2 · 500 Hz")}>
            <Typography.Text code>
              0x4102:03 Tff = slew(constant + URDF G(q)) × factory factor
            </Typography.Text>
          </Descriptions.Item>
          <Descriptions.Item label={text("Runtime SDO write/read", "运行时 SDO 读写")} span={2}>
            <Typography.Text code>
              0x4102:04 Kp · :05 Kd · :06 P/D limit · :07 factor (R) · 0x4572 max torque
            </Typography.Text>
          </Descriptions.Item>
        </Descriptions>
      </Card>

      <Card size="small" className="app-command-card">
        <Space wrap>
          <Button type="primary" size="large" disabled={!canStart} loading={operation === "starting"} onClick={() => void start()}>
            {text("Start sequential MIT test", "启动顺序 MIT 测试")}
          </Button>
          <Button
            size="large"
            disabled={!running || busy || !dirty}
            loading={operation === "updating"}
            onClick={() => void updateRuntime()}
          >
            {text("Apply runtime parameters", "应用运行参数")}
          </Button>
          <Button
            danger
            size="large"
            disabled={!canStop || operation === "starting" || operation === "stopping"}
            loading={operation === "stopping"}
            onClick={() => void stop()}
          >
            {text("Stop and disable all", "停止并全部失能")}
          </Button>
          {running && dirty && <Tag color="warning">{text("Unapplied edits", "有未应用修改")}</Tag>}
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
    moving: ["Moving", "运动中", "processing"],
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
    "compensation-ramp": ["Transitioning Tff · holding", "Tff 过渡 · 保持中"],
    moving: ["500 Hz running", "500 Hz 运行中"],
    updating: ["Applying SDO · holding", "应用 SDO · 保持中"],
    dwell: ["Waypoint dwell", "轨迹点停留"],
    stopping: ["Stopping", "停止中"],
    stopped: ["Stopped", "已停止"],
    error: ["Safety stop", "安全停机"],
  };
  const [en, zh] = labels[phase ?? "idle"] ?? [phase ?? "Idle", phase ?? "待机"];
  return text(en, zh);
}

function formatNumber(value: number | null | undefined, digits = 6) {
  return value == null || !Number.isFinite(value) ? "—" : value.toFixed(digits);
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
  precision = 4,
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
        step={10}
        precision={0}
        disabled={disabled}
        onChange={(next) => set(Math.round(next ?? 0))}
      />
    </Field>
  );
}
