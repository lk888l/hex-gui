import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Alert, App, Button, Card, Space, Table, Tag, Tooltip, Typography } from "antd";
import type { ColumnsType } from "antd/es/table";
import { api, errMsg } from "../api";
import { nid2hex } from "../format";
import { useI18n } from "../i18n";
import { OnlineTag } from "../tags";
import type { MotorInfo } from "../types";

const POSITION_POLL_MS = 250;

type InitStatus = "running" | "success" | "failed";

interface InitResult {
  sessionEpoch: number;
  status: InitStatus;
  error: string | null;
}

interface PositionResult {
  sessionEpoch: number;
  positionRev: number | null;
  error: string | null;
}

function isMotor(device: MotorInfo) {
  return device.device_type === "cia402_motor" || device.device_type === "meow_motor";
}

async function readEncoderPosition(device: MotorInfo): Promise<number | null> {
  if (device.device_type === "meow_motor") {
    const snapshot = await api.meowGetStatus(device.node_id);
    return snapshot.measurements.position_rev;
  }
  const snapshot = await api.getStatus(device.node_id);
  return snapshot.measurements.position_rev;
}

export function AllMotorPositionsPanel({
  connected,
  devices,
  onBusyChange,
}: {
  connected: boolean;
  devices: MotorInfo[];
  onBusyChange?: (busy: boolean) => void;
}) {
  const { message } = App.useApp();
  const { t } = useI18n();
  const motors = useMemo(() => devices.filter(isMotor), [devices]);
  const [busy, setBusyState] = useState(false);
  const [hasRun, setHasRun] = useState(false);
  const [initResults, setInitResults] = useState<Record<number, InitResult>>({});
  const [positions, setPositions] = useState<Record<number, PositionResult>>({});
  const pollPending = useRef(false);

  const setBusy = useCallback(
    (next: boolean) => {
      setBusyState(next);
      onBusyChange?.(next);
    },
    [onBusyChange],
  );

  const refreshPositions = useCallback(async (targets: MotorInfo[]) => {
    if (pollPending.current) return;
    pollPending.current = true;
    try {
      const readings = await Promise.all(
        targets.map(async (device) => {
          if (!device.online) {
            return {
              nid: device.node_id,
              result: {
                sessionEpoch: device.session_epoch,
                positionRev: null,
                error: null,
              },
            };
          }
          try {
            return {
              nid: device.node_id,
              result: {
                sessionEpoch: device.session_epoch,
                positionRev: await readEncoderPosition(device),
                error: null,
              },
            };
          } catch (error) {
            return {
              nid: device.node_id,
              result: {
                sessionEpoch: device.session_epoch,
                positionRev: null,
                error: errMsg(error),
              },
            };
          }
        }),
      );
      setPositions((current) => {
        const next = { ...current };
        for (const { nid, result } of readings) next[nid] = result;
        return next;
      });
    } finally {
      pollPending.current = false;
    }
  }, []);

  useEffect(() => {
    if (connected) return;
    setHasRun(false);
    setInitResults({});
    setPositions({});
  }, [connected]);

  useEffect(() => {
    return () => onBusyChange?.(false);
  }, [onBusyChange]);

  useEffect(() => {
    if (!connected || !hasRun || busy) return;
    let alive = true;
    const tick = () => {
      if (!alive) return;
      void refreshPositions(motors);
    };
    tick();
    const timer = window.setInterval(tick, POSITION_POLL_MS);
    return () => {
      alive = false;
      window.clearInterval(timer);
    };
  }, [busy, connected, hasRun, motors, refreshPositions]);

  const initializeAll = async () => {
    setHasRun(true);
    setBusy(true);
    setPositions({});
    setInitResults((current) => {
      const next = { ...current };
      for (const motor of motors) {
        next[motor.node_id] = {
          sessionEpoch: motor.session_epoch,
          status: "running",
          error: null,
        };
      }
      return next;
    });

    try {
      const results = await api.initializeAll();
      const latestDevices = await api.listDevices();
      const latestMotors = latestDevices.filter(isMotor);
      const resultByNid = new Map(results);
      setInitResults((current) => {
        const next = { ...current };
        for (const motor of latestMotors) {
          const error = resultByNid.get(motor.node_id);
          if (error === undefined) continue;
          next[motor.node_id] = {
            sessionEpoch: motor.session_epoch,
            status: error == null ? "success" : "failed",
            error,
          };
        }
        return next;
      });
      await refreshPositions(latestMotors);

      const failed = results.filter(([, error]) => error != null);
      if (results.length === 0) message.warning(t("allMotorPositionsNoMotors"));
      else if (failed.length === 0) message.success(t("initAllDone"));
      else {
        message.warning(
          `${t("initAllPartial")} ${failed.map(([nid]) => nid2hex(nid)).join(", ")}`,
        );
      }
    } catch (error) {
      const detail = errMsg(error);
      setInitResults((current) => {
        const next = { ...current };
        for (const motor of motors) {
          const previous = next[motor.node_id];
          if (previous?.status === "running") {
            next[motor.node_id] = {
              sessionEpoch: motor.session_epoch,
              status: "failed",
              error: detail,
            };
          }
        }
        return next;
      });
      message.error(`${t("initFailed")}: ${detail}`);
    } finally {
      setBusy(false);
    }
  };

  const columns: ColumnsType<MotorInfo> = [
    {
      title: t("allMotorPositionsMotor"),
      key: "motor",
      render: (_, motor) => (
        <Space direction="vertical" size={0}>
          <Typography.Text strong>{motor.friendly_name}</Typography.Text>
          <Typography.Text type="secondary" code>
            {nid2hex(motor.node_id)}
          </Typography.Text>
        </Space>
      ),
    },
    {
      title: t("allMotorPositionsProtocol"),
      key: "protocol",
      render: (_, motor) => (
        <Tag color={motor.device_type === "meow_motor" ? "purple" : "blue"}>
          {motor.device_type === "meow_motor" ? "Meow Motor" : "CiA 402"}
        </Tag>
      ),
    },
    {
      title: t("online"),
      key: "online",
      render: (_, motor) => <OnlineTag online={motor.online} />,
    },
    {
      title: t("allMotorPositionsInitStatus"),
      key: "init",
      render: (_, motor) => {
        const result = initResults[motor.node_id];
        if (!result || result.sessionEpoch !== motor.session_epoch) {
          return <Tag>{t("allMotorPositionsNotRun")}</Tag>;
        }
        if (result.status === "running") {
          return <Tag color="processing">{t("allMotorPositionsInitializing")}</Tag>;
        }
        if (result.status === "success") {
          return <Tag color="success">{t("allMotorPositionsInitialized")}</Tag>;
        }
        return (
          <Tooltip title={result.error}>
            <Tag color="error">{t("allMotorPositionsInitError")}</Tag>
          </Tooltip>
        );
      },
    },
    {
      title: `${t("allMotorPositionsEncoder")} (Rev)`,
      key: "position",
      align: "right",
      render: (_, motor) => {
        const reading = positions[motor.node_id];
        if (!reading || reading.sessionEpoch !== motor.session_epoch || !hasRun) return "—";
        if (reading.error) {
          return (
            <Tooltip title={reading.error}>
              <Typography.Text type="danger">{t("allMotorPositionsReadError")}</Typography.Text>
            </Tooltip>
          );
        }
        if (reading.positionRev == null) return t("allMotorPositionsWaiting");
        return (
          <Typography.Text code style={{ fontVariantNumeric: "tabular-nums" }}>
            {reading.positionRev.toFixed(6)}
          </Typography.Text>
        );
      },
    },
  ];

  return (
    <Space direction="vertical" size={12} style={{ width: "100%" }}>
      <Alert
        type="info"
        showIcon
        message={t("allMotorPositionsHint")}
        description={t("allMotorPositionsHintDetail")}
      />
      <Card
        size="small"
        title={t("toolAllMotorPositions")}
        extra={
          <Button
            type="primary"
            loading={busy}
            disabled={!connected || motors.length === 0}
            onClick={() => void initializeAll()}
          >
            {t("initAll")}
          </Button>
        }
      >
        <Table
          rowKey={(motor) => `${motor.node_id}:${motor.session_epoch}`}
          columns={columns}
          dataSource={motors}
          pagination={false}
          size="small"
          scroll={{ x: 720 }}
          locale={{ emptyText: connected ? t("allMotorPositionsNoMotors") : t("connectFirst") }}
        />
      </Card>
    </Space>
  );
}
