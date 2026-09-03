import { QuestionCircleOutlined } from "@ant-design/icons";
import { useCallback, useEffect, useRef, useState, type ReactNode } from "react";
import {
  Alert,
  App,
  Button,
  Card,
  Descriptions,
  InputNumber,
  Segmented,
  Space,
  Switch,
  Tag,
  Tooltip,
  Typography,
} from "antd";
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
  emptyEndpointAccuracyStats,
  isRepresentablePpPosition,
  positionErrorRev,
  recordEndpointAccuracy,
  recordMissedEndpointSample,
  type EndpointAccuracyStats,
  type PpEndpoint,
} from "../ppAccuracy";
import type { MeowMotorSnapshot, MeowProfileLimits, MotorInfo } from "../types";
import "./PpAccuracyPanel.css";

const DEFAULT_PROFILE: MeowProfileLimits = {
  velocity_rev_per_s: 0.2,
  acceleration_rev_per_s2: 0.2,
  deceleration_rev_per_s2: 0.2,
};
const MIN_DWELL_MS = 10;
const MAX_DWELL_MS = 600_000;

interface RunConfig {
  positionA: number;
  positionB: number;
  dwellMs: number;
}

interface AccuracyByEndpoint {
  A: EndpointAccuracyStats;
  B: EndpointAccuracyStats;
}

export function PpAccuracyPanel({
  info,
  connected,
  logging,
  logPath,
  onToggleLog,
  onBusyChange,
  onRunningChange,
}: {
  info: MotorInfo;
  connected: boolean;
  logging: boolean;
  logPath: string | null;
  onToggleLog: (on: boolean) => void;
  onBusyChange?: (busy: boolean) => void;
  onRunningChange?: (running: boolean) => void;
}) {
  const { message } = App.useApp();
  const { t, lang } = useI18n();
  const text = (en: string, zh: string) => (lang === "zh" ? zh : en);
  const [snapshot, setSnapshot] = useState<MeowMotorSnapshot | null>(null);
  const snapshotRef = useRef<MeowMotorSnapshot | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [busy, setBusyState] = useState(false);
  const [running, setRunningState] = useState(false);
  const runningRef = useRef(false);
  const runTokenRef = useRef(0);
  const timerRef = useRef<number | null>(null);

  const [refreshHz, setRefreshHz] = useState(100);
  const [tpdoEventTimerMs, setTpdoEventTimerMs] = useState<number | null>(1);
  const [maxTorquePermille, setMaxTorquePermille] = useState(200);
  const [profile, setProfile] = useState<MeowProfileLimits>(DEFAULT_PROFILE);
  const [positionA, setPositionA] = useState(-0.1);
  const [positionB, setPositionB] = useState(0.1);
  const [dwellMs, setDwellMs] = useState(2_000);

  const [activeEndpoint, setActiveEndpoint] = useState<PpEndpoint | null>(null);
  const [activeTargetRev, setActiveTargetRev] = useState<number | null>(null);
  const [nextCommandAt, setNextCommandAt] = useState<number | null>(null);
  const [roundTrips, setRoundTrips] = useState(0);
  const [lastRunError, setLastRunError] = useState<string | null>(null);
  const [stats, setStats] = useState<AccuracyByEndpoint>(() => ({
    A: emptyEndpointAccuracyStats(),
    B: emptyEndpointAccuracyStats(),
  }));

  const setBusy = (value: boolean) => {
    setBusyState(value);
    onBusyChange?.(value);
  };

  const setRunning = (value: boolean) => {
    runningRef.current = value;
    setRunningState(value);
    onRunningChange?.(value);
  };

  const clearTimer = () => {
    if (timerRef.current != null) {
      window.clearTimeout(timerRef.current);
      timerRef.current = null;
    }
  };

  const acceptSnapshot = useCallback((next: MeowMotorSnapshot) => {
    snapshotRef.current = next;
    setSnapshot(next);
    setLoadError(null);
  }, []);

  const refresh = useCallback(async () => {
    const next = await api.meowGetStatus(info.node_id);
    acceptSnapshot(next);
    return next;
  }, [acceptSnapshot, info.node_id]);

  useEffect(() => {
    if (!connected) return;
    let alive = true;
    snapshotRef.current = null;
    setSnapshot(null);
    setLoadError(null);

    const load = async () => {
      try {
        const current = await api.meowGetStatus(info.node_id);
        if (
          current.lifecycle.kind !== "Unknown" &&
          current.lifecycle.kind !== "UnsupportedIdentity"
        ) {
          return current;
        }
      } catch {
        // Identification creates the dedicated latest-firmware manager entry.
      }
      return api.meowIdentify(info.node_id);
    };

    void load()
      .then((next) => {
        if (alive) acceptSnapshot(next);
      })
      .catch((error) => {
        if (alive) setLoadError(errMsg(error));
      });
    return () => {
      alive = false;
    };
  }, [acceptSnapshot, connected, info.node_id]);

  useEffect(() => {
    if (!connected) return;
    let alive = true;
    let requestPending = false;
    const timer = window.setInterval(() => {
      if (requestPending) return;
      requestPending = true;
      void api
        .meowGetStatus(info.node_id)
        .then((next) => {
          if (alive) acceptSnapshot(next);
        })
        .catch((error) => {
          if (alive) setLoadError(errMsg(error));
        })
        .finally(() => {
          requestPending = false;
        });
    }, Math.max(1, Math.round(1000 / refreshHz)));
    return () => {
      alive = false;
      window.clearInterval(timer);
    };
  }, [acceptSnapshot, connected, info.node_id, refreshHz]);

  useEffect(() => {
    return () => {
      runTokenRef.current += 1;
      clearTimer();
      onBusyChange?.(false);
      onRunningChange?.(false);
      if (runningRef.current) {
        // App-level tool switching performs and awaits the same disable before
        // disconnect. This remains a best-effort guard for other unmounts.
        void api.meowDisable(info.node_id).catch(() => {});
      }
    };
  }, [info.node_id, onBusyChange, onRunningChange]);

  const runSingle = async (label: string, action: () => Promise<unknown>) => {
    setBusy(true);
    try {
      await action();
      await refresh();
      message.success(label);
    } catch (error) {
      message.error(`${label} ${t("failed")}: ${errMsg(error)}`);
    } finally {
      setBusy(false);
    }
  };

  const stopAfterError = async (token: number, error: unknown) => {
    if (runTokenRef.current !== token) return;
    runTokenRef.current += 1;
    clearTimer();
    setNextCommandAt(null);
    setRunning(false);
    const detail = errMsg(error);
    setLastRunError(detail);
    setBusy(true);
    try {
      await api.meowDisable(info.node_id);
      await refresh();
    } catch (disableError) {
      setLastRunError(`${detail}; ${text("disable failed", "失能失败")}: ${errMsg(disableError)}`);
    } finally {
      setBusy(false);
    }
    message.error(`${text("PP accuracy test stopped", "PP 精度测试已停止")}: ${detail}`);
  };

  const scheduleNext = (
    token: number,
    endpoint: PpEndpoint,
    config: RunConfig,
    generationAtCommand: number,
  ) => {
    clearTimer();
    setNextCommandAt(Date.now() + config.dwellMs);
    timerRef.current = window.setTimeout(() => {
      void advance(token, endpoint, config, generationAtCommand);
    }, config.dwellMs);
  };

  const advance = async (
    token: number,
    endpoint: PpEndpoint,
    config: RunConfig,
    generationAtCommand: number,
  ) => {
    if (runTokenRef.current !== token || !runningRef.current) return;
    timerRef.current = null;

    const current = snapshotRef.current;
    const encoderRev = current?.measurements.position_rev;
    const ppModeActive =
      current?.logic?.state === "Enabled" &&
      current.logic.mode === "ProfilePosition";
    const hasFreshEncoder =
      current != null &&
      current.online &&
      ppModeActive &&
      current.measurements.tpdo1_generation > generationAtCommand &&
      encoderRev != null &&
      Number.isFinite(encoderRev);
    const sampledTarget = endpoint === "A" ? config.positionA : config.positionB;
    setStats((previous) => ({
      ...previous,
      [endpoint]: hasFreshEncoder
        ? recordEndpointAccuracy(previous[endpoint], encoderRev, sampledTarget)
        : recordMissedEndpointSample(previous[endpoint]),
    }));
    if (endpoint === "B") setRoundTrips((count) => count + 1);

    if (
      current == null ||
      !current.online ||
      !ppModeActive ||
      current.logic?.state === "Error" ||
      current.lifecycle.kind === "NeedsRestart"
    ) {
      await stopAfterError(token, text("motor feedback is not healthy", "电机反馈状态异常"));
      return;
    }

    const nextEndpoint: PpEndpoint = endpoint === "A" ? "B" : "A";
    const nextTarget = nextEndpoint === "A" ? config.positionA : config.positionB;
    setBusy(true);
    try {
      await api.meowSetTarget(info.node_id, {
        kind: "ProfilePosition",
        position_rev: nextTarget,
      });
      if (runTokenRef.current !== token || !runningRef.current) return;
      const afterCommand = await refresh();
      if (runTokenRef.current !== token || !runningRef.current) return;
      setActiveEndpoint(nextEndpoint);
      setActiveTargetRev(nextTarget);
      scheduleNext(
        token,
        nextEndpoint,
        config,
        afterCommand.measurements.tpdo1_generation,
      );
    } catch (error) {
      await stopAfterError(token, error);
    } finally {
      if (runTokenRef.current === token) setBusy(false);
    }
  };

  const validateConfig = (): RunConfig => {
    if (tpdoEventTimerMs == null) {
      throw new RangeError(text("Enter a TPDO period", "请输入 TPDO 周期"));
    }
    if (!isRepresentablePpPosition(positionA) || !isRepresentablePpPosition(positionB)) {
      throw new RangeError(
        text(
          `Both positions must be in [${PP_POSITION_MIN_REV}, ${PP_POSITION_MAX_REV_EXCLUSIVE}) rev`,
          `两个位置都必须位于 [${PP_POSITION_MIN_REV}, ${PP_POSITION_MAX_REV_EXCLUSIVE}) 圈`,
        ),
      );
    }
    if (positionA === positionB) {
      throw new RangeError(text("Position A and B must differ", "位置 A 与 B 不能相同"));
    }
    if (!Number.isInteger(dwellMs) || dwellMs < MIN_DWELL_MS || dwellMs > MAX_DWELL_MS) {
      throw new RangeError(
        text(
          `Command interval must be ${MIN_DWELL_MS}–${MAX_DWELL_MS} ms`,
          `指令间隔必须为 ${MIN_DWELL_MS}–${MAX_DWELL_MS} ms`,
        ),
      );
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
    if (
      !Number.isInteger(maxTorquePermille) ||
      maxTorquePermille < 0 ||
      maxTorquePermille > 1000
    ) {
      throw new RangeError(text("Max torque must be 0–1000‰", "最大力矩必须为 0–1000‰"));
    }
    return { positionA, positionB, dwellMs };
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
    setStats({ A: emptyEndpointAccuracyStats(), B: emptyEndpointAccuracyStats() });
    setRoundTrips(0);
    setActiveEndpoint(null);
    setActiveTargetRev(null);
    setNextCommandAt(null);
    setBusy(true);
    try {
      const initialized = await api.meowInitialize(info.node_id, tpdoEventTimerMs!);
      acceptSnapshot(initialized);
      await api.meowSetMaxTorque(info.node_id, maxTorquePermille);
      await api.meowSetProfileLimits(info.node_id, profile);
      await api.meowActivateTarget(info.node_id, {
        kind: "ProfilePosition",
        position_rev: config.positionA,
      });
      const afterCommand = await refresh();
      if (runTokenRef.current !== token) return;
      setRunning(true);
      setActiveEndpoint("A");
      setActiveTargetRev(config.positionA);
      scheduleNext(token, "A", config, afterCommand.measurements.tpdo1_generation);
      message.success(text("PP round-trip test started", "PP 往返测试已启动"));
    } catch (error) {
      const detail = errMsg(error);
      setLastRunError(detail);
      await api.meowDisable(info.node_id).catch(() => {});
      await refresh().catch(() => null);
      message.error(`${text("Could not start test", "测试启动失败")}: ${detail}`);
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
      await api.meowDisable(info.node_id);
      await refresh();
      message.success(text("Test stopped and motor disabled", "测试已停止，电机已失能"));
    } catch (error) {
      const detail = errMsg(error);
      setLastRunError(detail);
      message.error(`${text("Stop/disable failed", "停止/失能失败")}: ${detail}`);
    } finally {
      setBusy(false);
    }
  };

  const measurement = snapshot?.measurements;
  const encoderRev = measurement?.position_rev ?? null;
  const accumulatedRev =
    measurement?.accumulation_valid === true
      ? measurement.accumulated_position_rev
      : null;
  const currentErrorRev = positionErrorRev(encoderRev, activeTargetRev);
  const nextInMs = nextCommandAt == null ? null : Math.max(0, nextCommandAt - Date.now());
  const controlsDisabled = busy || running;
  const canStart = connected && snapshot?.online === true && !busy && !running;

  return (
    <Space className="pp-accuracy" direction="vertical" size={12}>
      <Card size="small">
        <Space style={{ justifyContent: "space-between", width: "100%" }} align="start" wrap>
          <Space direction="vertical" size={2}>
            <Space wrap>
              <Typography.Title level={4} style={{ margin: 0 }}>
                {snapshot?.friendly_name ?? info.friendly_name}
              </Typography.Title>
              <Typography.Text code>{nid2hex(info.node_id)}</Typography.Text>
              <Tag color={snapshot?.online ? "green" : "default"}>
                {snapshot?.online ? t("online") : text("Offline", "离线")}
              </Tag>
              <Tag color={running ? "processing" : "default"}>
                {running ? text("Round-trip running", "往返运行中") : text("Stopped", "已停止")}
              </Tag>
            </Space>
            <Typography.Text type="secondary">
              {t("meowLifecycle")}: {snapshot?.lifecycle.kind ?? "Identifying"} · NMT{" "}
              {snapshot?.nmt_state ?? "—"} · {text("Mode", "模式")}{" "}
              {snapshot?.logic?.state === "Enabled" ? snapshot.logic.mode : snapshot?.logic?.state ?? "—"}
            </Typography.Text>
          </Space>
          <Space size={16} wrap>
            <Space size={4}>
              <Typography.Text type="secondary">{t("refresh")}</Typography.Text>
              <Tooltip title={t("meowRefreshHint")}>
                <QuestionCircleOutlined style={{ color: "#8a93a3" }} />
              </Tooltip>
              <Segmented
                size="small"
                value={refreshHz}
                onChange={(value) => setRefreshHz(value as number)}
                options={[
                  { label: "50 Hz", value: 50 },
                  { label: "100 Hz", value: 100 },
                ]}
              />
            </Space>
            <Space size={4}>
              <Typography.Text type="secondary">{t("recordCsv")}</Typography.Text>
              <Switch checked={logging} onChange={onToggleLog} />
            </Space>
          </Space>
        </Space>
        {logging && logPath && (
          <Typography.Text type="secondary" style={{ display: "block", fontSize: 12 }} copyable>
            {logPath}
          </Typography.Text>
        )}
      </Card>

      {loadError && (
        <Alert type="error" showIcon message={t("meowIdentifyFailed")} description={loadError} />
      )}
      {snapshot?.logic?.state === "Error" && (
        <Alert
          type="error"
          showIcon
          message={`${t("motorFault")}: ${snapshot.logic.kind} @ 0x${snapshot.logic.raw_code
            .toString(16)
            .toUpperCase()
            .padStart(4, "0")}`}
          description={t("faultDesc")}
        />
      )}
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
        title={text("Encoder focus", "编码器重点显示")}
        extra={
          <Typography.Text type="secondary">
            TPDO1 #{measurement?.tpdo1_generation ?? "—"}
          </Typography.Text>
        }
      >
        <div className="pp-accuracy__readouts">
          <Readout
            primary
            label={text("Raw Q8.24 encoder position", "原始 Q8.24 编码器位置")}
            value={formatRev(encoderRev)}
            unit="Rev"
            detail={encoderRev == null ? text("Waiting for TPDO1", "等待 TPDO1") : `${(encoderRev * 360).toFixed(6)}°`}
          />
          <Readout
            label={text("Host accumulated encoder", "上位机累计编码器")}
            value={formatRev(accumulatedRev)}
            unit="Rev"
            detail={
              measurement?.accumulation_valid
                ? text("Continuous accumulation valid", "连续累计有效")
                : text("Waiting for continuous accumulation", "等待连续累计建立")
            }
          />
          <Readout
            label={text("Live target error (encoder − target)", "实时目标误差（编码器 − 目标）")}
            value={formatSigned(currentErrorRev)}
            unit="Rev"
            detail={currentErrorRev == null ? "—" : `${formatSigned(currentErrorRev * 360)}°`}
          />
        </div>
        <Descriptions size="small" bordered column={4} style={{ marginTop: 12 }}>
          <Descriptions.Item label={text("Active endpoint", "当前端点")}>{activeEndpoint ?? "—"}</Descriptions.Item>
          <Descriptions.Item label={text("Active target", "当前目标")}>{formatRev(activeTargetRev)} Rev</Descriptions.Item>
          <Descriptions.Item label={text("Velocity", "速度")}>{formatNumber(measurement?.velocity_rev_per_s, 6)} Rev/s</Descriptions.Item>
          <Descriptions.Item label={text("Raw torque feedback", "原始力矩反馈")}>{measurement?.torque_permille ?? "—"} ‰</Descriptions.Item>
          <Descriptions.Item label={text("Round trips", "完整往返")}>{roundTrips}</Descriptions.Item>
          <Descriptions.Item label={text("Next command in", "距下次指令")}>{nextInMs == null ? "—" : `${nextInMs} ms`}</Descriptions.Item>
          <Descriptions.Item label={t("driverTemp")}>{formatNumber(measurement?.driver_temp_c, 1)}</Descriptions.Item>
          <Descriptions.Item label={t("motorTemp")}>{formatNumber(measurement?.motor_temp_c, 1)}</Descriptions.Item>
        </Descriptions>
      </Card>

      <Card
        size="small"
        title={text("Endpoint accuracy samples", "端点精度采样")}
        extra={
          <Tooltip
            title={text(
              "Each endpoint is sampled immediately before the next command. A sample is skipped unless a newer TPDO1 arrived after that endpoint command.",
              "每个端点都在发送下一条指令前采样；只有该端点指令后收到更新的 TPDO1 才计入，否则记为漏采。",
            )}
          >
            <Typography.Text type="secondary">
              <QuestionCircleOutlined /> {text("Sampling rule", "采样规则")}
            </Typography.Text>
          </Tooltip>
        }
      >
        <div className="pp-accuracy__stats">
          <EndpointStatsCard endpoint="A" targetRev={positionA} stats={stats.A} />
          <EndpointStatsCard endpoint="B" targetRev={positionB} stats={stats.B} />
        </div>
      </Card>

      <Card size="small" title={text("Round-trip settings", "往返设置")}>
        <Space wrap align="end">
          <Field label={text("Position A (rev)", "位置 A（圈）")}>
            <InputNumber
              value={positionA}
              min={PP_POSITION_MIN_REV}
              max={PP_POSITION_MAX_REV_EXCLUSIVE - 2 ** -24}
              step={0.01}
              precision={9}
              disabled={controlsDisabled}
              onChange={(value) => setPositionA(value ?? 0)}
            />
          </Field>
          <Button
            size="small"
            disabled={controlsDisabled || encoderRev == null}
            onClick={() => setPositionA(encoderRev ?? positionA)}
          >
            {text("A = encoder", "A = 当前编码器")}
          </Button>
          <Field label={text("Position B (rev)", "位置 B（圈）")}>
            <InputNumber
              value={positionB}
              min={PP_POSITION_MIN_REV}
              max={PP_POSITION_MAX_REV_EXCLUSIVE - 2 ** -24}
              step={0.01}
              precision={9}
              disabled={controlsDisabled}
              onChange={(value) => setPositionB(value ?? 0)}
            />
          </Field>
          <Button
            size="small"
            disabled={controlsDisabled || encoderRev == null}
            onClick={() => setPositionB(encoderRev ?? positionB)}
          >
            {text("B = encoder", "B = 当前编码器")}
          </Button>
          <Field label={text("Command interval / endpoint dwell", "指令间隔 / 端点停留")}>
            <InputNumber
              value={dwellMs}
              min={MIN_DWELL_MS}
              max={MAX_DWELL_MS}
              step={100}
              precision={0}
              addonAfter="ms"
              disabled={controlsDisabled}
              onChange={(value) => setDwellMs(value ?? 0)}
            />
          </Field>
        </Space>
        <Alert
          type="info"
          showIcon
          style={{ marginTop: 12 }}
          message={text(
            "The interval starts when a target command is confirmed; it includes travel time, and the encoder is sampled at the end of the interval.",
            "间隔从目标指令确认后开始，包含运动时间；间隔结束时采集编码器，再发送另一端点。",
          )}
        />
      </Card>

      <Card size="small" title={text("PP parameters", "PP 对应参数")}>
        <Space direction="vertical" size={12} style={{ width: "100%" }}>
          <Space wrap align="end">
            <Field label={t("meowTpdoPeriod")}>
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
              {t("meowTpdoEffectiveRate")}: {tpdoEventTimerMs == null ? "—" : formatPdoRateHz(tpdoEventTimerMs)} Hz
            </Typography.Text>
            <Button
              disabled={controlsDisabled || !snapshot?.online || tpdoEventTimerMs == null}
              loading={busy}
              onClick={() => {
                if (tpdoEventTimerMs != null) {
                  void runSingle(
                    snapshot?.lifecycle.kind === "Initialized" ? t("reinitialize") : t("initialize"),
                    () => api.meowInitialize(info.node_id, tpdoEventTimerMs),
                  );
                }
              }}
            >
              {snapshot?.lifecycle.kind === "Initialized" ? t("reinitialize") : t("initialize")}
            </Button>
            <Button
              disabled={controlsDisabled || snapshot == null}
              loading={busy}
              onClick={() => void runSingle(t("clearError"), () => api.meowClearError(info.node_id))}
            >
              {t("clearError")}
            </Button>
          </Space>
          <Space wrap align="end">
            <Field label={t("meowMaxTorque")}>
              <InputNumber
                value={maxTorquePermille}
                min={0}
                max={1000}
                step={10}
                precision={0}
                disabled={controlsDisabled}
                onChange={(value) => setMaxTorquePermille(value ?? 0)}
              />
            </Field>
            <Button
              disabled={controlsDisabled || !snapshot?.is_ready}
              loading={busy}
              onClick={() =>
                void runSingle(t("limitMaxTorque"), () =>
                  api.meowSetMaxTorque(info.node_id, maxTorquePermille),
                )
              }
            >
              {t("apply")}
            </Button>
            <NumericField
              label={t("meowProfileVelocity")}
              value={profile.velocity_rev_per_s}
              disabled={controlsDisabled}
              set={(value) => setProfile({ ...profile, velocity_rev_per_s: value })}
            />
            <NumericField
              label={t("meowProfileAcceleration")}
              value={profile.acceleration_rev_per_s2}
              disabled={controlsDisabled}
              set={(value) => setProfile({ ...profile, acceleration_rev_per_s2: value })}
            />
            <NumericField
              label={t("meowProfileDeceleration")}
              value={profile.deceleration_rev_per_s2}
              disabled={controlsDisabled}
              set={(value) => setProfile({ ...profile, deceleration_rev_per_s2: value })}
            />
            <Button
              disabled={controlsDisabled || !snapshot?.is_ready}
              loading={busy}
              onClick={() =>
                void runSingle(t("meowApplyProfile"), () =>
                  api.meowSetProfileLimits(info.node_id, profile),
                )
              }
            >
              {t("meowApplyProfile")}
            </Button>
          </Space>
          <Typography.Text type="secondary" style={{ fontSize: 12 }}>
            {text(
              "Start re-initializes with the TPDO period above, applies max torque and all three profile limits, then activates PP at endpoint A.",
              "启动时会先按上方 TPDO 周期重新初始化，再应用最大力矩和三个规划参数，最后以端点 A 激活 PP。",
            )}
          </Typography.Text>
        </Space>
      </Card>

      <Card size="small" className="app-command-card">
        <Space wrap>
          <Button type="primary" size="large" disabled={!canStart} loading={busy} onClick={() => void start()}>
            {text("Start round trip", "启动往返")}
          </Button>
          <Button danger size="large" disabled={!running || busy} loading={busy} onClick={() => void stop()}>
            {text("Stop and disable", "停止并失能")}
          </Button>
          <Button
            disabled={running || busy || snapshot == null}
            onClick={() => void runSingle(t("disableAction"), () => api.meowDisable(info.node_id))}
          >
            {t("disableAction")}
          </Button>
          {!connected && <Typography.Text type="warning">{t("connectFirst")}</Typography.Text>}
        </Space>
      </Card>
    </Space>
  );
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

function Readout({
  label,
  value,
  unit,
  detail,
  primary = false,
}: {
  label: string;
  value: string;
  unit: string;
  detail: string;
  primary?: boolean;
}) {
  return (
    <div className={`pp-accuracy__readout${primary ? " pp-accuracy__readout--primary" : ""}`}>
      <div className="pp-accuracy__readout-label">{label}</div>
      <div className="pp-accuracy__readout-value">{value}</div>
      <div className="pp-accuracy__readout-unit">{unit}</div>
      <div className="pp-accuracy__readout-detail">{detail}</div>
    </div>
  );
}

function EndpointStatsCard({
  endpoint,
  targetRev,
  stats,
}: {
  endpoint: PpEndpoint;
  targetRev: number;
  stats: EndpointAccuracyStats;
}) {
  const { lang } = useI18n();
  const text = (en: string, zh: string) => (lang === "zh" ? zh : en);
  const entries = [
    [text("Target", "目标"), `${formatRev(targetRev)} Rev`],
    [text("Last encoder", "末次编码器"), `${formatRev(stats.last_encoder_rev)} Rev`],
    [text("Last signed error", "末次有符号误差"), `${formatSigned(stats.last_error_rev)} Rev`],
    [text("Mean absolute error", "平均绝对误差"), `${formatUnsigned(stats.mean_abs_error_rev)} Rev`],
    [text("Max absolute error", "最大绝对误差"), `${formatUnsigned(stats.max_abs_error_rev)} Rev`],
    [text("Valid / missed", "有效 / 漏采"), `${stats.samples} / ${stats.missed}`],
  ];
  return (
    <Card size="small" title={`${text("Endpoint", "端点")} ${endpoint}`}>
      <div className="pp-accuracy__stat-grid">
        {entries.map(([label, value]) => (
          <div key={label}>
            <div className="pp-accuracy__stat-label">{label}</div>
            <div className="pp-accuracy__stat-value">{value}</div>
          </div>
        ))}
      </div>
    </Card>
  );
}

function formatNumber(value: number | null | undefined, digits: number): string {
  return value == null || !Number.isFinite(value) ? "—" : value.toFixed(digits);
}

function formatRev(value: number | null | undefined): string {
  return formatNumber(value, 9);
}

function formatSigned(value: number | null | undefined): string {
  if (value == null || !Number.isFinite(value)) return "—";
  return `${value >= 0 ? "+" : ""}${value.toFixed(9)}`;
}

function formatUnsigned(value: number | null | undefined): string {
  return value == null || !Number.isFinite(value) ? "—" : Math.abs(value).toFixed(9);
}
