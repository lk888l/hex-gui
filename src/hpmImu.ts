// HPM bimu only. Units match hpm-imu/docs/can.md, not the legacy STM32 IMU.
export type Quaternion = [number, number, number, number];
export interface HpmImuObject {
  index: number; sub: number; bits: number; name: string; unit: string; signed: boolean; scale: number;
}
export interface HpmImuPdo {
  cob_id: number; transmission_type: number; inhibit_100us: number;
  event_ms: number; sync_start: number; mapping: number[];
}
export interface HpmImuValue {
  index: number; sub: number; value: number; age_ms: number; changed_age_ms: number; rx_unix_ms: number;
}
export interface HpmImuSnapshot {
  identity: { node: number; vendor: number; product: number; revision: number; serial: number;
    hardware: number | null; name: string; firmware: string } | null;
  heartbeat: { node: number; state: number; age_ms: number }[];
  pdos: HpmImuPdo[];
  stats: { frames: number; age_ms: number | null; hz: number; bytes: number; fd: boolean; brs: boolean }[];
  values: HpmImuValue[];
  invalid_frames: number; error: string | null; recorded_frames: number;
  angle_test: HpmImuTestSnapshot | null;
}

export interface HpmImuTestConfig {
  imu_node: number; motor_node: number; host_node: number;
  angle_deg: number; repeats: number; axis: number; direction: number; zero_each_round: boolean;
  speed_deg_s: number; settle_ms: number; samples: number; tolerance_deg: number; timeout_s: number;
}
export interface HpmImuTestRow {
  round: number; target_deg: number; motor_deg: number; home_raw_deg: number; raw_deg: number;
  error_deg: number; error_percent: number; absolute_error_percent: number; return_raw_deg: number | null;
}
export interface HpmImuTestSnapshot {
  active: boolean; phase: string; completed: number; config: HpmImuTestConfig;
  motor_model: string | null; motor_position_deg: number | null; imu_angle_deg: number | null;
  stop_confirmed: boolean; error: string | null; rows: HpmImuTestRow[];
}

export function testSummary(rows: HpmImuTestRow[]) {
  if (!rows.length) return null;
  return {
    count: rows.length,
    meanError: rows.reduce((sum, row) => sum + row.error_deg, 0) / rows.length,
    meanAbsolutePercent: rows.reduce((sum, row) => sum + row.absolute_error_percent, 0) / rows.length,
    maxAbsolutePercent: Math.max(...rows.map(row => row.absolute_error_percent)),
    rmsDegrees: Math.sqrt(rows.reduce((sum, row) => sum + row.error_deg ** 2, 0) / rows.length),
  };
}
export const odKey = (index: number, sub: number) => `${index.toString(16).toUpperCase().padStart(4, "0")}:${sub.toString(16).toUpperCase().padStart(2, "0")}`;
export const hex32 = (n: number) => `0x${(n >>> 0).toString(16).toUpperCase().padStart(8, "0")}`;
export const mapObject = (o: HpmImuObject) => ((o.index << 16) | (o.sub << 8) | o.bits) >>> 0;
export const pdoBytes = (p: HpmImuPdo) => p.mapping.reduce((n, m) => n + (m & 255) / 8, 0);
export const wireBytes = (bytes: number) => [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64].find(n => n >= bytes) ?? bytes;

export function defaultPdos(node: number, period = 8): HpmImuPdo[] {
  const map = (index: number, sub: number, bits: number) => ((index << 16) | (sub << 8) | bits) >>> 0;
  const raw = [0x3002, 0x3003, 0x3005].flatMap(i => [1, 2, 3].map(s => map(i, s, 16)));
  raw.push(map(0x3004, 1, 16), map(0x3004, 2, 32), map(0x3004, 3, 8), map(0x3004, 4, 32));
  const chip = [1, 2, 3, 4].map(s => map(0x3000, s, 16));
  chip.push(...[1, 2, 3, 4, 5, 6].map(s => map(0x3006, s, 16)), map(0x3008, 1, 32), map(0x3008, 2, 32), map(0x3008, 5, 8));
  return [raw, chip, [], []].map((mapping, i) => ({
    cob_id: ((0x180 + i * 0x100 + node) | (i > 1 ? 0x80000000 : 0)) >>> 0,
    transmission_type: 255, inhibit_100us: 0, event_ms: period, sync_start: 0, mapping,
  }));
}

export function field(values: HpmImuValue[], index: number, sub: number) {
  return values.find(v => v.index === index && v.sub === sub);
}
export function fresh(v: HpmImuValue | undefined, elapsed = 0) {
  return !!v && Number.isFinite(v.value) && v.age_ms + elapsed < 250;
}
export function chipAttitude(values: HpmImuValue[], elapsed = 0): Quaternion | null {
  const status = field(values, 0x3008, 5);
  const age = field(values, 0x3008, 2);
  const counter = field(values, 0x3008, 1);
  const q = [1, 2, 3, 4].map(sub => field(values, 0x3000, sub));
  if (!fresh(status, elapsed) || !(status!.value & 1) || !fresh(age, elapsed) ||
      age!.value + age!.age_ms + elapsed >= 250 || !fresh(counter, elapsed) ||
      counter!.changed_age_ms + elapsed >= 250 || !q.every(v => fresh(v, elapsed))) return null;
  const components = q.map(v => v!.value) as Quaternion;
  const norm = Math.hypot(...components);
  if (norm < 0.5 || norm > 1.5) return null;
  return components.map(v => v / norm) as Quaternion;
}
export function relativeQuaternion(q: Quaternion, reference: Quaternion | null): Quaternion {
  if (!reference) return q;
  const [a, b, c, d] = [reference[0], -reference[1], -reference[2], -reference[3]];
  const [w, x, y, z] = q;
  return [a*w-b*x-c*y-d*z, a*x+b*w+c*z-d*y, a*y-b*z+c*w+d*x, a*z+b*y-c*x+d*w];
}
export function eulerDegrees([w, x, y, z]: Quaternion) {
  return [Math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)), Math.asin(Math.max(-1, Math.min(1, 2*(w*y-z*x)))),
    Math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))].map(v => v * 180 / Math.PI);
}
