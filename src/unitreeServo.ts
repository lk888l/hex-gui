// Host-side settings. Saving this profile never claims to write servo Flash.
export interface UnitreeTarget {
  position_rad: number;
  velocity_rad_s: number;
  torque_nm: number;
  kp: number;
  kd: number;
}

export interface UnitreeSettings {
  command_hz: number;
  response_timeout_ms: number;
  timeout_protection: boolean;
  position_min_rad: number;
  position_max_rad: number;
  velocity_limit_rad_s: number;
  torque_limit_nm: number;
}

export interface UnitreeProfile {
  schema_version: 1;
  port: string;
  selected_id: number;
  settings: UnitreeSettings;
  target: UnitreeTarget;
}

export interface UnitreeBusTestNode { id: number; target: UnitreeTarget }
export interface UnitreeBusTestRequest {
  kind: "communication" | "motion";
  duration_s: number;
  nodes: UnitreeBusTestNode[];
}

export function validateUnitreeBusTest(request: UnitreeBusTestRequest, settings: UnitreeSettings): UnitreeBusTestRequest {
  validateUnitreeSettings(settings);
  if (!["communication", "motion"].includes(request.kind)) throw new Error("无效的测试类型 / Invalid test kind");
  if (request.nodes.length < 2 || request.nodes.length > 15) throw new Error("请选择 2–15 台舵机 / Select 2–15 servos");
  const unique = new Set<number>();
  return {
    kind: request.kind,
    duration_s: range("测试时长 / Test seconds", request.duration_s, 1, 120, true),
    nodes: request.nodes.map(node => {
      const id = validateUnitreeId(node.id);
      if (unique.has(id)) throw new Error(`重复 ID / Duplicate ID: ${id}`);
      unique.add(id);
      const target = request.kind === "motion" ? validateUnitreeTarget(node.target, settings)
        : { position_rad: 0, velocity_rad_s: 0, torque_nm: 0, kp: 0, kd: 0 };
      return { id, target };
    }),
  };
}

export const DEFAULT_UNITREE_SETTINGS: UnitreeSettings = {
  command_hz: 100,
  response_timeout_ms: 20,
  timeout_protection: true,
  position_min_rad: -2 * Math.PI,
  position_max_rad: 2 * Math.PI,
  velocity_limit_rad_s: 1,
  torque_limit_nm: 0.5,
};

export const DEFAULT_UNITREE_TARGET: UnitreeTarget = {
  position_rad: 0, velocity_rad_s: 0, torque_nm: 0, kp: 0, kd: 0.1,
};

export const UNITREE_PROFILE_KEY = "hex-gui.unitree-j288s288.profile.v1";

export function validateUnitreeId(value: number): number {
  if (!Number.isInteger(value) || value < 0 || value > 14) {
    throw new Error("ID 必须为 0–14（15 是广播地址） / ID must be 0–14 (15 is broadcast)");
  }
  return value;
}

export function validateUnitreeIdChange(oldId: number, newId: number): { oldId: number; newId: number } {
  validateUnitreeId(oldId);
  validateUnitreeId(newId);
  if (oldId === newId) throw new Error("新 ID 必须与当前 ID 不同 / New ID must differ from current ID");
  return { oldId, newId };
}

function range(name: string, value: unknown, min: number, max: number, integer = false): number {
  if (typeof value !== "number" || !Number.isFinite(value) || value < min || value > max || (integer && !Number.isInteger(value))) {
    throw new Error(`${name}: ${min} … ${max}`);
  }
  return value;
}

export function validateUnitreeSettings(settings: UnitreeSettings): UnitreeSettings {
  const checked: UnitreeSettings = {
    command_hz: range("发送频率 / Command Hz", settings.command_hz, 65, 500, true),
    response_timeout_ms: range("响应超时 / Response timeout ms", settings.response_timeout_ms, 5, 100, true),
    timeout_protection: settings.timeout_protection,
    position_min_rad: range("位置下限 / Position minimum", settings.position_min_rad, -1428.018898, 1428.018898),
    position_max_rad: range("位置上限 / Position maximum", settings.position_max_rad, -1428.018898, 1428.018898),
    velocity_limit_rad_s: range("速度限制 / Velocity limit", settings.velocity_limit_rad_s, Number.MIN_VALUE, 278.90143),
    torque_limit_nm: range("前馈力矩限制 / Feedforward torque limit", settings.torque_limit_nm, Number.MIN_VALUE, 36.908174),
  };
  if (typeof checked.timeout_protection !== "boolean") throw new Error("Invalid timeout protection setting");
  if (checked.position_min_rad >= checked.position_max_rad) throw new Error("位置下限必须小于上限 / Position minimum must be below maximum");
  return checked;
}

export function validateUnitreeTarget(target: UnitreeTarget, settings: UnitreeSettings): UnitreeTarget {
  validateUnitreeSettings(settings);
  return {
    position_rad: range("目标位置 / Position rad", target.position_rad, settings.position_min_rad, settings.position_max_rad),
    velocity_rad_s: range("目标速度 / Velocity rad/s", target.velocity_rad_s, -settings.velocity_limit_rad_s, settings.velocity_limit_rad_s),
    torque_nm: range("前馈力矩 / Feedforward torque Nm", target.torque_nm, -settings.torque_limit_nm, settings.torque_limit_nm),
    kp: range("Kp", target.kp, 0, 2128.523254),
    kd: range("Kd", target.kd, 0, 21.285233),
  };
}

export function parseUnitreeProfile(text: string): UnitreeProfile {
  const value = JSON.parse(text);
  if (!value || value.schema_version !== 1 || typeof value.port !== "string" || value.port.length > 256 || !value.settings || !value.target) {
    throw new Error("无效的 J288/S288 本机配置 / Invalid J288/S288 local profile");
  }
  const settings = validateUnitreeSettings(value.settings);
  return {
    schema_version: 1,
    port: value.port,
    selected_id: validateUnitreeId(value.selected_id),
    settings,
    target: validateUnitreeTarget(value.target, settings),
  };
}
