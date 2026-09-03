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
import {
  MAX_PDO_EVENT_TIMER_MS,
  MIN_PDO_EVENT_TIMER_MS,
  clampToRange,
  formatPdoRateHz,
} from "../meowMotorUi";
import {
  PP_POSITION_MAX_REV_EXCLUSIVE,
  PP_POSITION_MIN_REV,
  positionErrorRev,
} from "../ppAccuracy";
import {
  SIX_MOTOR_PP_DEFAULT_INTERVAL_MS,
  SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS,
  SIX_MOTOR_PP_IDS,
  cloneSixMotorPpDefaultWaypoints,
  validateSixMotorPpWaypoints,
} from "../sixMotorPpTest";
import { OnlineTag } from "../tags";
import type { MeowMotorSnapshot, MeowProfileLimits, MotorInfo } from "../types";
import "./SixMotorPpTestPanel.css";

const POSITION_POLL_MS = 50;
const MIN_INTERVAL_MS = 10;
const MAX_INTERVAL_MS = 600_000;
const DEFAULT_MAX_TORQUE_PERMILLE = 200;
const DEFAULT_PROFILE: MeowProfileLimits = {
  velocity_rev_per_s: 0.2,
  acceleration_rev_per_s2: 0.2,
  deceleration_rev_per_s2: 0.2,
};

type MotorPhase = "idle" | "initializing" | "ready" | "running" | "disabled" | "error";

interface MotorRunState {
  phase: MotorPhase;
  error: string | null;
}

interface SnapshotReading {
  snapshot: MeowMotorSnapshot | null;
  error: string | null;
}

interface MotorRow {
  nid: number;
  device: MotorInfo | null;
}

interface WaypointRow {
  key: number;
  label: string;
  positions: number[];
}

interface RunConfig {
  waypoints: number[][];
  intervalMs: number;
  tpdoEventTimerMs: number;
  maxTorquePermille: number;
  profile: MeowProfileLimits;
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
  const [busy, setBusyState] = useState(false);
  const [running, setRunningState] = useState(false);
  const runningRef = useRef(false);
  const runTokenRef = useRef(0);
  const timerRef = useRef<number | null>(null);
  const pollPendingRef = useRef(false);
  const activeWaypointRef = useRef(0);
  const generationAtCommandRef = useRef<Record<number, number>>({});

  const [waypoints, setWaypoints] = useState<number[][]>(() =>
    cloneSixMotorPpDefaultWaypoints(),
  );
  const [intervalMs, setIntervalMs] = useState(SIX_MOTOR_PP_DEFAULT_INTERVAL_MS);
  const [tpdoEventTimerMs, setTpdoEventTimerMs] = useState<number | null>(
    SIX_MOTOR_PP_DEFAULT_TPDO_EVENT_TIMER_MS,
  );
  const [maxTorquePermille, setMaxTorquePermille] = useState(
    DEFAULT_MAX_TORQUE_PERMILLE,
  );
  const [profile, setProfile] = useState<MeowProfileLimits>(DEFAULT_PROFILE);
  const [activeWaypoint, setActiveWaypoint] = useState(0);
  const [nextCommandAt, setNextCommandAt] = useState<number | null>(null);
  const [completedCycles, setCompletedCycles] = useState(0);
  const [lastRunError, setLastRunError] = useState<string | null>(null);
  const [motorStates, setMotorStates] = useState<Record<number, MotorRunState>>({});
  const [snapshots, setSnapshots] = useState<Record<number, SnapshotReading>>({});
  const snapshotsRef = useRef<Record<number, SnapshotReading>>({});

  const deviceByNid = useMemo(
    () => new Map(devices.map((device) => [device.node_id, device])),
    [devices],
  );
  const motorRows = useMemo<MotorRow[]>(
    () =>
      SIX_MOTOR_PP_IDS.map((nid) => ({
        nid,
        device: deviceByNid.get(nid) ?? null,
      })),
    [deviceByNid],
  );

  const unavailableReasons = useMemo(() => {
    const reasons: string[] = [];
    for (const nid of SIX_MOTOR_PP_IDS) {
      const device = deviceByNid.get(nid);
      if (device == null) {
        reasons.push(text(`ID ${nid} not discovered`, `未发现 ID ${nid}`));
      } else if (device.device_type !== "meow_motor") {
        reasons.push(text(`ID ${nid} is not a Meow Motor`, `ID ${nid} 不是 Meow Motor`));
      } else if (!device.online) {
        reasons.push(text(`ID ${nid} is offline`, `ID ${nid} 已离线`));
      }
    }
    return reasons;
  }, [deviceByNid, text]);

  const setBusy = useCallback(
    (next: boolean) => {
      setBusyState(next);
      onBusyChange?.(next);
    },
    [onBusyChange],
  );

  const setRunning = useCallback(
    (next: boolean) => {
      runningRef.current = next;
      setRunningState(next);
      onRunningChange?.(next);
    },
    [onRunningChange],
  );

  const clearTimer = useCallback(() => {
    if (timerRef.current != null) {
      window.clearTimeout(timerRef.current);
      timerRef.current = null;
    }
  }, []);

  const updateMotorState = useCallback(
    (nid: number, phase: MotorPhase, error: string | null = null) => {
      setMotorStates((current) => ({ ...current, [nid]: { phase, error } }));
    },
    [],
  );

  const publishReadings = useCallback((readings: Array<[number, SnapshotReading]>) => {
    const next = { ...snapshotsRef.current };
    for (const [nid, reading] of readings) next[nid] = reading;
    snapshotsRef.current = next;
    setSnapshots(next);
  }, []);

  const readSnapshots = useCallback(
    async (nids: readonly number[]) => {
      const readings = await Promise.all(
        nids.map(async (nid): Promise<[number, SnapshotReading]> => {
          try {
            return [nid, { snapshot: await api.meowGetStatus(nid), error: null }];
          } catch (error) {
            return [nid, { snapshot: null, error: errMsg(error) }];
          }
        }),
      );
      publishReadings(readings);
      return readings;
    },
    [publishReadings],
  );

  useEffect(() => {
    if (connected) return;
    snapshotsRef.current = {};
    setSnapshots({});
    setMotorStates({});
  }, [connected]);

  useEffect(() => {
    if (!connected || busy) return;
    const pollNids = SIX_MOTOR_PP_IDS.filter((nid) => {
      const device = deviceByNid.get(nid);
      return device?.device_type === "meow_motor" && device.online;
    });
    if (pollNids.length === 0) return;

    let alive = true;
    const tick = () => {
      if (!alive || pollPendingRef.current) return;
      pollPendingRef.current = true;
      void readSnapshots(pollNids).finally(() => {
        pollPendingRef.current = false;
      });
    };
    tick();
    const timer = window.setInterval(tick, POSITION_POLL_MS);
    return () => {
      alive = false;
      window.clearInterval(timer);
    };
  }, [busy, connected, deviceByNid, readSnapshots]);

  useEffect(() => {
    return () => {
      runTokenRef.current += 1;
      clearTimer();
      onBusyChange?.(false);
      onRunningChange?.(false);
      if (runningRef.current) {
        for (const nid of SIX_MOTOR_PP_IDS) void api.meowDisable(nid).catch(() => {});
      }
    };
  }, [clearTimer, onBusyChange, onRunningChange]);

  const disableAll = useCallback(async () => {
    const failures: string[] = [];
    for (const nid of SIX_MOTOR_PP_IDS) {
      try {
        await api.meowDisable(nid);
        updateMotorState(nid, "disabled");
      } catch (error) {
        const detail = errMsg(error);
        failures.push(`ID ${nid}: ${detail}`);
        updateMotorState(nid, "error", detail);
      }
    }
    return failures;
  }, [updateMotorState]);

  const validateConfig = (): RunConfig => {
    if (tpdoEventTimerMs == null) {
      throw new RangeError(text("Enter a TPDO period", "请输入 TPDO 周期"));
    }
    validateSixMotorPpWaypoints(waypoints);
    if (
      !Number.isInteger(intervalMs) ||
      intervalMs < MIN_INTERVAL_MS ||
      intervalMs > MAX_INTERVAL_MS
    ) {
      throw new RangeError(
        text(
          `Command interval must be ${MIN_INTERVAL_MS}–${MAX_INTERVAL_MS} ms`,
          `指令间隔必须为 ${MIN_INTERVAL_MS}–${MAX_INTERVAL_MS} ms`,
        ),
      );
    }
    if (
      !Number.isInteger(maxTorquePermille) ||
      maxTorquePermille < 0 ||
      maxTorquePermille > 1000
    ) {
      throw new RangeError(text("Max torque must be 0–1000‰", "最大力矩必须为 0–1000‰"));
    }
    for (const value of [
      profile.velocity_rev_per_s,
      profile.acceleration_rev_per_s2,
      profile.deceleration_rev_per_s2,
    ]) {
      if (!Number.isFinite(value) || value <= 0) {
        throw new RangeError(text("Profile limits must be positive", "规划参数必须大于 0"));
      }
    }
    return {
      waypoints: waypoints.map((positions) => [...positions]),
      intervalMs,
      tpdoEventTimerMs,
      maxTorquePermille,
      profile: { ...profile },
    };
  };

  const assertAllMotorsAvailable = async () => {
    const latest = await api.listDevices();
    const latestByNid = new Map(latest.map((device) => [device.node_id, device]));
    const problems: string[] = [];
    for (const nid of SIX_MOTOR_PP_IDS) {
      const device = latestByNid.get(nid);
      if (device == null) problems.push(text(`ID ${nid} not discovered`, `未发现 ID ${nid}`));
      else if (device.device_type !== "meow_motor") {
        problems.push(text(`ID ${nid} is not a Meow Motor`, `ID ${nid} 不是 Meow Motor`));
      } else if (!device.online) problems.push(text(`ID ${nid} is offline`, `ID ${nid} 已离线`));
    }
    if (problems.length > 0) throw new Error(problems.join("; "));
  };

  const scheduleNext = useCallback(
    (token: number, config: RunConfig) => {
      clearTimer();
      setNextCommandAt(Date.now() + config.intervalMs);
      timerRef.current = window.setTimeout(() => {
        void advance(token, config);
      }, config.intervalMs);
    },
    [clearTimer],
  );

  const stopAfterError = async (token: number, error: unknown) => {
    if (runTokenRef.current !== token) return;
    runTokenRef.current += 1;
    clearTimer();
    setNextCommandAt(null);
    setRunning(false);
    const detail = errMsg(error);
    setLastRunError(detail);
    const disableFailures = await disableAll();
    await readSnapshots(SIX_MOTOR_PP_IDS).catch(() => []);
    setBusy(false);
    const suffix = disableFailures.length === 0 ? "" : `; ${disableFailures.join("; ")}`;
    message.error(`${text("Six-motor PP test stopped", "六电机 PP 测试已停止")}: ${detail}${suffix}`);
  };

  const advance = async (token: number, config: RunConfig) => {
    if (runTokenRef.current !== token || !runningRef.current) return;
    timerRef.current = null;
    setBusy(true);
    try {
      const beforeCommand = await readSnapshots(SIX_MOTOR_PP_IDS);
      for (const [nid, reading] of beforeCommand) {
        const snapshot = reading.snapshot;
        const baseline = generationAtCommandRef.current[nid];
        const healthy =
          snapshot != null &&
          reading.error == null &&
          snapshot.online &&
          snapshot.logic?.state === "Enabled" &&
          snapshot.logic.mode === "ProfilePosition" &&
          snapshot.lifecycle.kind !== "NeedsRestart" &&
          baseline != null &&
          snapshot.measurements.tpdo1_generation > baseline &&
          snapshot.measurements.position_rev != null &&
          Number.isFinite(snapshot.measurements.position_rev);
        if (!healthy) {
          const detail = reading.error ?? text("feedback is stale or unhealthy", "反馈陈旧或状态异常");
          updateMotorState(nid, "error", detail);
          throw new Error(`ID ${nid}: ${detail}`);
        }
      }

      const nextWaypoint = (activeWaypointRef.current + 1) % config.waypoints.length;
      for (let motorIndex = 0; motorIndex < SIX_MOTOR_PP_IDS.length; motorIndex += 1) {
        const nid = SIX_MOTOR_PP_IDS[motorIndex];
        try {
          await api.meowSetTarget(nid, {
            kind: "ProfilePosition",
            position_rev: config.waypoints[nextWaypoint][motorIndex],
          });
        } catch (error) {
          const detail = errMsg(error);
          updateMotorState(nid, "error", detail);
          throw new Error(`ID ${nid}: ${detail}`);
        }
      }

      const afterCommand = await readSnapshots(SIX_MOTOR_PP_IDS);
      generationAtCommandRef.current = Object.fromEntries(
        afterCommand.map(([nid, reading]) => [
          nid,
          reading.snapshot?.measurements.tpdo1_generation ?? 0,
        ]),
      );
      if (runTokenRef.current !== token || !runningRef.current) return;
      activeWaypointRef.current = nextWaypoint;
      setActiveWaypoint(nextWaypoint);
      if (nextWaypoint === 0) setCompletedCycles((count) => count + 1);
      scheduleNext(token, config);
    } catch (error) {
      await stopAfterError(token, error);
    } finally {
      if (runTokenRef.current === token) setBusy(false);
    }
  };

  const start = async () => {
    let config: RunConfig;
    try {
      config = validateConfig();
    } catch (error) {
      message.error(errMsg(error));
      return;
    }

    const token = runTokenRef.current + 1;
    runTokenRef.current = token;
    clearTimer();
    setLastRunError(null);
    setCompletedCycles(0);
    activeWaypointRef.current = 0;
    setActiveWaypoint(0);
    setNextCommandAt(null);
    setMotorStates({});
    setBusy(true);
    try {
      await assertAllMotorsAvailable();
      for (const nid of SIX_MOTOR_PP_IDS) {
        updateMotorState(nid, "initializing");
        try {
          const snapshot = await api.meowInitialize(nid, config.tpdoEventTimerMs);
          publishReadings([[nid, { snapshot, error: null }]]);
          await api.meowSetMaxTorque(nid, config.maxTorquePermille);
          await api.meowSetProfileLimits(nid, config.profile);
          updateMotorState(nid, "ready");
        } catch (error) {
          const detail = errMsg(error);
          updateMotorState(nid, "error", detail);
          throw new Error(`ID ${nid}: ${detail}`);
        }
      }

      for (let motorIndex = 0; motorIndex < SIX_MOTOR_PP_IDS.length; motorIndex += 1) {
        const nid = SIX_MOTOR_PP_IDS[motorIndex];
        try {
          await api.meowActivateTarget(nid, {
            kind: "ProfilePosition",
            position_rev: config.waypoints[0][motorIndex],
          });
          updateMotorState(nid, "running");
        } catch (error) {
          const detail = errMsg(error);
          updateMotorState(nid, "error", detail);
          throw new Error(`ID ${nid}: ${detail}`);
        }
      }

      const afterCommand = await readSnapshots(SIX_MOTOR_PP_IDS);
      generationAtCommandRef.current = Object.fromEntries(
        afterCommand.map(([nid, reading]) => [
          nid,
          reading.snapshot?.measurements.tpdo1_generation ?? 0,
        ]),
      );
      if (runTokenRef.current !== token) return;
      setRunning(true);
      scheduleNext(token, config);
      message.success(text("Six-motor PP test started", "六电机 PP 测试已启动"));
    } catch (error) {
      const detail = errMsg(error);
      setLastRunError(detail);
      setRunning(false);
      const disableFailures = await disableAll();
      const suffix = disableFailures.length === 0 ? "" : `; ${disableFailures.join("; ")}`;
      message.error(`${text("Could not start test", "测试启动失败")}: ${detail}${suffix}`);
    } finally {
      setBusy(false);
    }
  };

  const stop = async () => {
    runTokenRef.current += 1;
    clearTimer();
    setNextCommandAt(null);
    setRunning(false);
    setBusy(true);
    try {
      const failures = await disableAll();
      await readSnapshots(SIX_MOTOR_PP_IDS).catch(() => []);
      if (failures.length > 0) throw new Error(failures.join("; "));
      message.success(text("Test stopped; all six motors disabled", "测试已停止，6 台电机均已失能"));
    } catch (error) {
      const detail = errMsg(error);
      setLastRunError(detail);
      message.error(`${text("Stop/disable failed", "停止/失能失败")}: ${detail}`);
    } finally {
      setBusy(false);
    }
  };

  const controlsDisabled = busy || running;
  const canStart = connected && unavailableReasons.length === 0 && !busy && !running;
  const nextInMs = nextCommandAt == null ? null : Math.max(0, nextCommandAt - Date.now());
  const waypointLabels = [
    text("Initial", "初始位置"),
    text("Position 1", "位置 1"),
    text("Position 2", "位置 2"),
    text("Position 3", "位置 3"),
  ];

  const motorColumns: ColumnsType<MotorRow> = [
    {
      title: text("Motor", "电机"),
      key: "motor",
      width: 150,
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
      width: 90,
      render: (_, row) => <OnlineTag online={row.device?.online === true} />,
    },
    {
      title: text("Test status", "测试状态"),
      key: "status",
      width: 150,
      render: (_, row) => renderMotorStatus(row, motorStates[row.nid], snapshots[row.nid], text),
    },
    {
      title: text("Expected position (Rev)", "预期位置（Rev）"),
      key: "expected",
      align: "right",
      width: 180,
      render: (_, row) => (
        <Typography.Text className="six-motor-pp__expected" code>
          {formatPosition(waypoints[activeWaypoint]?.[row.nid - 1])}
        </Typography.Text>
      ),
    },
    {
      title: text("Actual encoder position (Rev)", "实际编码器位置（Rev）"),
      key: "actual",
      align: "right",
      width: 230,
      render: (_, row) => {
        const reading = validReading(row.device, snapshots[row.nid]);
        if (reading?.error) {
          return (
            <Tooltip title={reading.error}>
              <Typography.Text type="danger">{text("Read failed", "读取失败")}</Typography.Text>
            </Tooltip>
          );
        }
        const actual = reading?.snapshot?.measurements.position_rev;
        return (
          <Typography.Text className="six-motor-pp__encoder" code>
            {formatPosition(actual)}
          </Typography.Text>
        );
      },
    },
    {
      title: text("Error: actual − expected (Rev)", "误差：实际 − 预期（Rev）"),
      key: "error",
      align: "right",
      width: 220,
      render: (_, row) => {
        const actual = validReading(row.device, snapshots[row.nid])?.snapshot?.measurements.position_rev;
        const expected = waypoints[activeWaypoint]?.[row.nid - 1];
        return (
          <Typography.Text code>{formatSigned(positionErrorRev(actual, expected))}</Typography.Text>
        );
      },
    },
    {
      title: "TPDO1",
      key: "tpdo",
      align: "right",
      width: 110,
      render: (_, row) =>
        validReading(row.device, snapshots[row.nid])?.snapshot?.measurements.tpdo1_generation ?? "—",
    },
  ];

  const waypointRows: WaypointRow[] = waypoints.map((positions, index) => ({
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
          {activeWaypoint === row.key && <Tag color="processing">{text("Active", "当前")}</Tag>}
        </Space>
      ),
    },
    ...SIX_MOTOR_PP_IDS.map(
      (nid): ColumnsType<WaypointRow>[number] => ({
        title: `ID ${nid} (Rev)`,
        key: `motor-${nid}`,
        align: "right",
        width: 145,
        render: (_, row) => (
          <InputNumber
            aria-label={`${row.label} ID ${nid}`}
            value={row.positions[nid - 1]}
            min={PP_POSITION_MIN_REV}
            max={PP_POSITION_MAX_REV_EXCLUSIVE - 2 ** -24}
            step={0.01}
            precision={9}
            disabled={controlsDisabled}
            onChange={(value) => {
              setWaypoints((current) =>
                current.map((positions, waypointIndex) =>
                  waypointIndex === row.key
                    ? positions.map((position, motorIndex) =>
                        motorIndex === nid - 1 ? (value ?? 0) : position,
                      )
                    : positions,
                ),
              );
            }}
          />
        ),
      }),
    ),
  ];

  return (
    <Space className="six-motor-pp" direction="vertical" size={12}>
      <Alert
        type={unavailableReasons.length === 0 ? "info" : "warning"}
        showIcon
        message={text(
          "Fixed six-motor PP sequence · Node IDs 1–6",
          "固定六电机 PP 序列 · 节点 ID 1–6",
        )}
        description={
          unavailableReasons.length === 0
            ? text(
                "All six latest-firmware Meow Motors are available. Starting will initialize and enable every motor.",
                "6 台最新固件 Meow Motor 均已就绪；启动后会逐台初始化并使能。",
              )
            : unavailableReasons.join("；")
        }
      />

      {lastRunError && (
        <Alert
          type="error"
          closable
          showIcon
          message={text("Last test error", "上次测试错误")}
          description={lastRunError}
          onClose={() => setLastRunError(null)}
        />
      )}

      <Card
        size="small"
        title={text("Six encoder positions", "六电机编码器位置重点显示")}
        extra={
          <Space wrap>
            <Tag color={running ? "processing" : "default"}>
              {running ? text("Sequence running", "序列运行中") : text("Stopped", "已停止")}
            </Tag>
            <Typography.Text type="secondary">
              {text("Current", "当前")}: {waypointLabels[activeWaypoint]}
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
          scroll={{ x: 1_120 }}
        />
        <Descriptions size="small" bordered column={3} style={{ marginTop: 12 }}>
          <Descriptions.Item label={text("Completed cycles", "完整循环")}>{completedCycles}</Descriptions.Item>
          <Descriptions.Item label={text("Next command in", "距下次指令")}>
            {nextInMs == null ? "—" : `${nextInMs} ms`}
          </Descriptions.Item>
          <Descriptions.Item label={text("Encoder display refresh", "编码器显示刷新")}>
            {1000 / POSITION_POLL_MS} Hz
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
            row.key === activeWaypoint ? "six-motor-pp__waypoint--active" : ""
          }
        />
        <Typography.Text type="secondary" style={{ display: "block", marginTop: 10 }}>
          {text(
            "The sequence repeats: Initial → Position 1 → Position 2 → Position 3 → Initial.",
            "序列循环执行：初始位置 → 位置 1 → 位置 2 → 位置 3 → 初始位置。",
          )}
        </Typography.Text>
      </Card>

      <Card size="small" title={text("PP parameters", "PP 参数")}>
        <Space direction="vertical" size={12} style={{ width: "100%" }}>
          <Space wrap align="end">
            <Field label={text("TPDO period", "TPDO 周期")}>
              <InputNumber
                min={MIN_PDO_EVENT_TIMER_MS}
                max={MAX_PDO_EVENT_TIMER_MS}
                step={1}
                precision={0}
                value={tpdoEventTimerMs}
                addonAfter="ms"
                disabled={controlsDisabled}
                onChange={(value) => {
                  if (value == null) {
                    setTpdoEventTimerMs(null);
                    return;
                  }
                  setTpdoEventTimerMs(
                    clampToRange(
                      Math.round(value),
                      MIN_PDO_EVENT_TIMER_MS,
                      MAX_PDO_EVENT_TIMER_MS,
                    ),
                  );
                }}
              />
            </Field>
            <Typography.Text type="secondary">
              {text("PDO rate", "PDO 频率")}: {tpdoEventTimerMs == null ? "—" : formatPdoRateHz(tpdoEventTimerMs)} Hz
            </Typography.Text>
            <Field label={text("Command interval", "位置切换间隔")}>
              <InputNumber
                min={MIN_INTERVAL_MS}
                max={MAX_INTERVAL_MS}
                step={100}
                precision={0}
                value={intervalMs}
                addonAfter="ms"
                disabled={controlsDisabled}
                onChange={(value) => setIntervalMs(value ?? 0)}
              />
            </Field>
            <Field label={text("Max torque (‰)", "最大力矩（‰）")}>
              <InputNumber
                min={0}
                max={1000}
                step={10}
                precision={0}
                value={maxTorquePermille}
                disabled={controlsDisabled}
                onChange={(value) => setMaxTorquePermille(value ?? 0)}
              />
            </Field>
          </Space>
          <Space wrap align="end">
            <NumericField
              label={text("Profile velocity (Rev/s)", "规划速度（Rev/s）")}
              value={profile.velocity_rev_per_s}
              disabled={controlsDisabled}
              set={(value) => setProfile({ ...profile, velocity_rev_per_s: value })}
            />
            <NumericField
              label={text("Acceleration (Rev/s²)", "加速度（Rev/s²）")}
              value={profile.acceleration_rev_per_s2}
              disabled={controlsDisabled}
              set={(value) => setProfile({ ...profile, acceleration_rev_per_s2: value })}
            />
            <NumericField
              label={text("Deceleration (Rev/s²)", "减速度（Rev/s²）")}
              value={profile.deceleration_rev_per_s2}
              disabled={controlsDisabled}
              set={(value) => setProfile({ ...profile, deceleration_rev_per_s2: value })}
            />
          </Space>
          <Typography.Text type="secondary">
            {text(
              "Defaults: PDO 100 Hz, interval 2.5 s, max torque 200‰, and all profile limits 0.2.",
              "默认值：PDO 100 Hz、间隔 2.5 s、最大力矩 200‰，三个规划参数均为 0.2。",
            )}
          </Typography.Text>
        </Space>
      </Card>

      <Card size="small" className="app-command-card">
        <Space wrap>
          <Button
            type="primary"
            size="large"
            disabled={!canStart}
            loading={busy && !running}
            onClick={() => void start()}
          >
            {text("Start six-motor sequence", "启动六电机序列")}
          </Button>
          <Button
            danger
            size="large"
            disabled={!running || busy}
            loading={busy && running}
            onClick={() => void stop()}
          >
            {text("Stop and disable all", "停止并全部失能")}
          </Button>
          {!connected && (
            <Typography.Text type="warning">{text("Connect first", "请先连接")}</Typography.Text>
          )}
        </Space>
      </Card>
    </Space>
  );
}

function validReading(
  device: MotorInfo | null,
  reading: SnapshotReading | undefined,
): SnapshotReading | null {
  if (device == null || reading == null) return null;
  if (reading.snapshot != null && reading.snapshot.session_epoch !== device.session_epoch) return null;
  return reading;
}

function renderMotorStatus(
  row: MotorRow,
  runState: MotorRunState | undefined,
  rawReading: SnapshotReading | undefined,
  text: (en: string, zh: string) => string,
) {
  if (row.device == null) return <Tag>{text("Not discovered", "未发现")}</Tag>;
  if (row.device.device_type !== "meow_motor") return <Tag color="error">{text("Unsupported", "不支持")}</Tag>;
  if (!row.device.online) return <Tag color="warning">{text("Offline", "离线")}</Tag>;
  const reading = validReading(row.device, rawReading);
  if (reading?.error) {
    return (
      <Tooltip title={reading.error}>
        <Tag color="error">{text("Read failed", "读取失败")}</Tag>
      </Tooltip>
    );
  }
  if (runState?.phase === "error") {
    return (
      <Tooltip title={runState.error}>
        <Tag color="error">{text("Failed", "失败")}</Tag>
      </Tooltip>
    );
  }
  const phaseLabel: Record<Exclude<MotorPhase, "error">, [string, string, string]> = {
    idle: ["Standby", "待机", "default"],
    initializing: ["Initializing", "初始化中", "processing"],
    ready: ["Ready", "就绪", "success"],
    running: ["PP running", "PP 运行中", "processing"],
    disabled: ["Disabled", "已失能", "default"],
  };
  const phase = runState?.phase ?? "idle";
  const [en, zh, color] = phaseLabel[phase];
  return <Tag color={color}>{text(en, zh)}</Tag>;
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
  disabled,
  set,
}: {
  label: ReactNode;
  value: number;
  disabled: boolean;
  set: (value: number) => void;
}) {
  return (
    <Field label={label}>
      <InputNumber
        value={value}
        min={0.000001}
        step={0.1}
        disabled={disabled}
        onChange={(next) => set(next ?? 0)}
      />
    </Field>
  );
}
