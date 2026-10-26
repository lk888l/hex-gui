import { Alert, Button, Card, Descriptions, Input, Popconfirm, Space, Tag, Typography } from "antd";
import { standardMitApi, type El05State } from "../el05Api";

interface Props {
  state: El05State | null;
  zh: boolean;
  extensions: boolean;
  disabled: boolean;
  run: (operation: () => Promise<unknown>, success?: string) => Promise<void>;
  setZero: () => Promise<void>;
  newMotorId: string;
  setNewMotorId: (id: string) => void;
  canChangeId: boolean;
  changeMotorId: () => Promise<void>;
}

export function StandardMitSettings({ state, zh, extensions, disabled, run, setZero, newMotorId, setNewMotorId, canChangeId, changeMotorId }: Props) {
  const tr = (cn: string, en: string) => zh ? cn : en;
  const stopped = !disabled && !state?.enabled;
  const canZero = !disabled && state?.mode !== "PositionCsp";
  const hex = (value: number | null | undefined) => value == null ? "—" : `0x${value.toString(16).toUpperCase()}`;
  return <Card size="small" title={tr("MIT 设备设置", "MIT device settings")}>
    <Space direction="vertical" size={16} style={{ width: "100%" }}>
      <Descriptions size="small" column={{ xs: 1, sm: 2 }}>
        <Descriptions.Item label={tr("电机 CAN ID", "Motor CAN ID")}>{hex(state?.motor_id)}</Descriptions.Item>
        <Descriptions.Item label={tr("回复 CAN ID", "Reply CAN ID")}>{hex(state?.reply_id)}</Descriptions.Item>
        <Descriptions.Item label={tr("指令配置", "Command profile")}><Tag>{extensions ? tr("EL05 手册扩展", "EL05 manual extensions") : tr("基础 MIT", "Basic MIT")}</Tag></Descriptions.Item>
        <Descriptions.Item label="MCU UID">{state?.identity?.uid ? <Typography.Text code copyable>{state.identity.uid}</Typography.Text> : tr("未取得配置应答", "No configuration acknowledgement yet")}</Descriptions.Item>
      </Descriptions>
      <Typography.Paragraph type="secondary" style={{ margin: 0 }}>{tr("机械置零会先停机，等待新的近零位置反馈，并清除旧坐标曲线。位置模式下请先切回 MIT。重连、换电机、修改 ID 或机械零位后，需重新记录摆锤下垂参考。", "Zeroing stops the motor, waits for fresh near-zero feedback and clears old coordinate traces. Switch from position mode to MIT first. Reconnects, motor changes, ID changes and zeroing require a new downward reference.")}</Typography.Paragraph>
      <Popconfirm title={tr("停止并将当前位置设置为零点？", "Stop and set this position as zero?")}
        description={tr("旧下垂参考将失效，完成后请重新记录。", "The previous downward reference becomes invalid; capture it again afterward.")}
        disabled={!canZero} onConfirm={setZero} okText={tr("停止并置零", "Stop & reset zero")} cancelText={tr("取消", "Cancel")}>
        <Button disabled={!canZero}>{tr("停止并重置位置零点", "Stop & reset position zero")}</Button>
      </Popconfirm>
      {extensions ? <>
        <div className="el05-settings-row">
          <label className="el05-field"><Typography.Text type="secondary">{tr("新电机 ID（0–127）", "New motor ID (0–127)")}</Typography.Text>
            <Input aria-label={tr("新电机 ID", "New motor ID")} value={newMotorId} disabled={!stopped} onChange={event => setNewMotorId(event.target.value)} placeholder={tr("十进制或 0x", "Decimal or 0x")} />
          </label>
          <Popconfirm title={tr("修改电机 CAN ID？", "Change motor CAN ID?")}
            description={tr("先用停止帧检查新地址；通过配置应答和新地址反馈确认后重新选择。", "Probes the new address with Stop, then confirms the configuration acknowledgement and feedback at the new address.")}
            disabled={!canChangeId} onConfirm={changeMotorId} okText={tr("修改 ID", "Change ID")} cancelText={tr("取消", "Cancel")}>
            <Button disabled={!canChangeId}>{tr("修改 ID", "Change ID")}</Button>
          </Popconfirm>
        </div>
        <Space wrap>
          <Button disabled={disabled} onClick={() => void run(() => standardMitApi.clearFault(), tr("清故障已应答", "Fault reset acknowledged"))}>{tr("停止并清除故障", "Stop & clear fault")}</Button>
          <Popconfirm title={tr("将当前参数保存到电机？", "Save current parameters to the motor?")}
            onConfirm={() => run(() => standardMitApi.saveParameters(), tr("参数保存已应答", "Parameter save acknowledged"))} disabled={!stopped}
            okText={tr("保存", "Save")} cancelText={tr("取消", "Cancel")}>
            <Button disabled={!stopped}>{tr("保存参数", "Save parameters")}</Button>
          </Popconfirm>
          <Button disabled={!stopped} onClick={() => void run(() => standardMitApi.setReporting(!state?.reporting), tr("主动上报设置已应答", "Reporting setting acknowledged"))}>
            {state?.reporting ? tr("关闭主动上报", "Disable reporting") : tr("开启主动上报", "Enable reporting")}
          </Button>
        </Space>
        <Alert type="info" showIcon message={tr("手册扩展与固件版本", "Manual extensions and firmware")}
          description={tr("保存、主动上报和参数读写需要固件 1.0.5.0.4 或兼容实现。不支持的指令会显示超时或拒绝。版本读取未在 MIT 协议中定义；配置应答可返回 MCU UID。", "Save, reporting and parameter access require firmware 1.0.5.0.4 or a compatible implementation. Unsupported commands report timeouts or rejections. MIT has no documented version query; configuration replies can return the MCU UID.")} />
      </> : <Typography.Paragraph type="secondary" style={{ margin: 0 }}>{tr("基础配置支持五参数 MIT 控制、位置/速度/转矩/温度反馈、置零、全部摆锤测试和零重力保持。设备支持手册扩展时，停止并断开后选择「EL05 手册扩展」重新连接。", "The basic profile supports five-parameter MIT control, position/velocity/torque/temperature feedback, zeroing, all pendulum tests and zero-gravity hold. If the device supports the manual extensions, stop and disconnect, then reconnect with EL05 manual extensions selected.")}</Typography.Paragraph>}
    </Space>
  </Card>;
}
