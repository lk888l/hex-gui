import { Alert, Button, Card, Descriptions, Popconfirm, Space, Tag } from "antd";
import { useI18n } from "../i18n";
import type { PullCordEncoderState } from "../pullCordApi";

export function PullCordEncoder({ state, canZero, onZero }: {
  state: PullCordEncoderState | null;
  canZero: boolean;
  onZero: () => void;
}) {
  const { lang } = useI18n();
  const tr = (zh: string, en: string) => lang === "zh" ? zh : en;
  const sensor: Record<string, string> = {
    Ok: tr("磁场正常", "Magnet OK"),
    MagnetTooWeak: tr("磁场偏弱", "Magnet too weak"),
    MagnetTooStrong: tr("磁场偏强", "Magnet too strong"),
    MagnetNotDetected: tr("未检测到磁铁", "Magnet not detected"),
    SensorIoError: tr("传感器读取错误", "Sensor I/O error"),
  };
  return <Card size="small" title={tr("拉线方向编码器", "Cord direction encoder")}>
    <Space wrap>
      <Tag color={state?.angle_usable ? "green" : "default"}>
        {state?.angle_usable ? tr("角度有效", "Angle valid") : tr("等待有效反馈", "Waiting for valid feedback")}
      </Tag>
      <Popconfirm title={tr("将当前位置写入编码器固件零点？", "Write the current position as the encoder firmware zero?")}
        description={tr("写入后读取核对，并检查新角度反馈。", "Verifies readback and fresh angle feedback after writing.")}
        onConfirm={onZero} disabled={!canZero}>
        <Button disabled={!canZero}>{tr("设为编码器零点", "Set encoder zero")}</Button>
      </Popconfirm>
    </Space>
    <Descriptions size="small" column={{ xs: 1, sm: 2, md: 3 }} style={{ marginTop: 12 }}>
      <Descriptions.Item label={tr("节点", "Node")}>{state ? `0x${state.node_id.toString(16).toUpperCase().padStart(2, "0")}` : "—"}</Descriptions.Item>
      <Descriptions.Item label={tr("角度", "Angle")}>{state?.angle_usable && state.angle_degrees != null ? `${state.angle_degrees.toFixed(2)}°` : "—"}</Descriptions.Item>
      <Descriptions.Item label={tr("磁场状态", "Magnet status")}>{state?.sensor_status ? sensor[state.sensor_status] ?? state.sensor_status : "—"}</Descriptions.Item>
      <Descriptions.Item label={tr("位置 / 状态年龄", "Position / status age")}>{state?.position_age_ms ?? "—"} / {state?.status_age_ms ?? "—"} ms</Descriptions.Item>
      <Descriptions.Item label={tr("位置帧 / 无效帧", "Position / invalid frames")}>{state?.position_frames ?? 0} / {state?.invalid_frames ?? 0}</Descriptions.Item>
      <Descriptions.Item label={tr("总线状态", "Bus state")}>{state?.bus_state ?? "—"}</Descriptions.Item>
    </Descriptions>
    {state?.receive_error && <Alert type="error" message={state.receive_error} />}
  </Card>;
}
