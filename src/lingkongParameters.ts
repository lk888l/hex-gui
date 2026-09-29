// CAN V2.36 sections 18/19 (RAM) and 26–28 (setting/ROM).
// The protocol leaves some setting units unspecified; those rows expose the
// documented integer wire value rather than guessing a physical conversion.
export interface LingkongParameterDef {
  key: string;
  group: "ram" | "setting";
  zh: string;
  en: string;
  address: string;
  max: number;
  scale?: number;
  fields?: string[];
  options?: { value: number; zh: string; en: string }[];
  noteZh?: string;
  noteEn?: string;
}

const pidFields = ["Kp", "Ki", "Kd"];

export const lingkongParameters: LingkongParameterDef[] = [
  { key: "ram_position_pid", group: "ram", zh: "位置环 PID", en: "Position loop PID", address: "0xC0/0xC1 · 0x0A", max: 2000, fields: pidFields },
  { key: "ram_speed_pid", group: "ram", zh: "速度环 PID", en: "Speed loop PID", address: "0xC0/0xC1 · 0x0B", max: 2000, fields: pidFields },
  { key: "ram_current_pid", group: "ram", zh: "电流环 PID", en: "Current loop PID", address: "0xC0/0xC1 · 0x0C", max: 2000, fields: pidFields },
  { key: "ram_torque_limit", group: "ram", zh: "转矩电流限制", en: "Torque current limit", address: "0xC0/0xC1 · 0x1E", max: 2000, noteZh: "MHF 比值；0xA1 直发电流不受此限值约束", noteEn: "MHF ratio; direct 0xA1 current bypasses this limit" },
  { key: "ram_speed_limit", group: "ram", zh: "速度限制 (°/s)", en: "Speed limit (°/s)", address: "0xC0/0xC1 · 0x20", max: 600_000, scale: 100 },
  { key: "ram_angle_limit", group: "ram", zh: "角度限制 (°)", en: "Angle limit (°)", address: "0xC0/0xC1 · 0x22", max: 2_147_483_647, scale: 100 },
  { key: "ram_current_ramp", group: "ram", zh: "电流斜率 (协议值)", en: "Current ramp (raw)", address: "0xC0/0xC1 · 0x24", max: 30_000, noteZh: "MHF 使用手册标为暂未开放", noteEn: "The MHF user manual marks this unavailable" },
  { key: "ram_speed_ramp", group: "ram", zh: "速度斜率 (°/s²)", en: "Speed ramp (°/s²)", address: "0xC0/0xC1 · 0x26", max: 600_000 },
  { key: "setting_driver_id", group: "setting", zh: "电机 ID", en: "Driver ID", address: "0x40/0x42 · 05 0A", max: 32, noteZh: "0 使用拨码开关 ID；保存并重启后生效", noteEn: "0 selects DIP-switch ID; takes effect after save and reboot" },
  { key: "setting_bus_type", group: "setting", zh: "总线类型", en: "Bus type", address: "0x40/0x42 · 05 0B", max: 2,
    options: [{ value: 0, zh: "无", en: "None" }, { value: 1, zh: "RS485", en: "RS485" }, { value: 2, zh: "CAN", en: "CAN" }],
    noteZh: "修改为非 CAN 后，此页面将无法通过 CAN 连接", noteEn: "Changing away from CAN makes this page unreachable over CAN" },
  { key: "setting_rs485_baudrate", group: "setting", zh: "RS485 波特率", en: "RS485 baudrate", address: "0x40/0x42 · 05 0C", max: 10,
    options: [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 2000000, 4000000].map((rate, value) => ({ value, zh: `${rate} bps`, en: `${rate} bps` })) },
  { key: "setting_can_baudrate", group: "setting", zh: "电机 CAN 波特率", en: "Motor CAN bitrate", address: "0x40/0x42 · 05 0D", max: 4,
    options: [100, 125, 250, 500, 1000].map((rate, value) => ({ value, zh: `${rate} kbit/s`, en: `${rate} kbit/s` })),
    noteZh: "保存并重启后，主机 CAN 口也须设为相同波特率；gs_usb 当前仅支持 1 Mbps", noteEn: "After save and reboot, match the host bitrate; gs_usb currently supports only 1 Mbps" },
  { key: "setting_max_power", group: "setting", zh: "最大功率/转矩比值", en: "Max power / torque ratio", address: "0x40/0x42 · 05 E0", max: 2000, noteZh: "MHF 范围 0–2000；协议字段名为 Max Power", noteEn: "MHF range 0–2000; protocol field is named Max Power" },
  { key: "setting_max_speed", group: "setting", zh: "最大速度 (协议值)", en: "Max speed (raw)", address: "0x40/0x42 · 05 E2", max: 600_000 },
  { key: "setting_max_angle", group: "setting", zh: "最大角度 (协议值)", en: "Max angle (raw)", address: "0x40/0x42 · 05 E4", max: 2_147_483_647 },
  { key: "setting_current_ramp", group: "setting", zh: "电流斜率 (协议值)", en: "Current ramp (raw)", address: "0x40/0x42 · 05 EA", max: 30_000, noteZh: "MHF 使用手册标为暂未开放", noteEn: "The MHF user manual marks this unavailable" },
  { key: "setting_speed_ramp", group: "setting", zh: "速度斜率 (°/s²)", en: "Speed ramp (°/s²)", address: "0x40/0x42 · 05 EC", max: 600_000 },
  { key: "setting_position_pid", group: "setting", zh: "位置环 PID", en: "Position loop PID", address: "0x40/0x42 · A0", max: 2000, fields: pidFields },
  { key: "setting_speed_pid", group: "setting", zh: "速度环 PID", en: "Speed loop PID", address: "0x40/0x42 · A4", max: 2000, fields: pidFields },
  { key: "setting_current_pid", group: "setting", zh: "电流环 PID", en: "Current loop PID", address: "0x40/0x42 · A8", max: 2000, fields: pidFields },
];
