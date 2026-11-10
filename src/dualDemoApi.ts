import { invoke } from "@tauri-apps/api/core";

export interface DemoConnection {
  meow_iface: string;
  meow_bitrate: number;
  dm_iface: string;
  dm_bitrate: number;
  host_node_id: number;
  meow_node_id: number | null;
  dm_motor_id: number | null;
}

export interface DemoMotion {
  amplitude_deg: number;
  period_s: number;
  speed_variation: number;
  meow_kp: number;
  meow_kd: number;
  dm_kp: number;
  dm_kd: number;
  dm_reverse: boolean;
}

export interface DemoMotorView {
  node_id: number | null;
  online: boolean;
  enabled: boolean;
  target_rpm: number;
  actual_rpm: number | null;
  center_rad: number | null;
  target_deg: number;
  actual_deg: number | null;
  temperature_c: number | null;
  feedback_age_ms: number | null;
}

export interface DemoView {
  connected: boolean;
  running: boolean;
  phase: "disconnected" | "ready" | "starting" | "running" | "stopping" | "error";
  elapsed_s: number;
  connection: DemoConnection | null;
  motion: DemoMotion;
  meow: DemoMotorView;
  dm: DemoMotorView;
  error: string | null;
  stop_error: string | null;
}

export const dualDemoApi = {
  connect: (config: DemoConnection) => invoke<DemoView>("dual_demo_connect", { config }),
  start: (motion: DemoMotion) => invoke<DemoView>("dual_demo_start", { motion }),
  get: () => invoke<DemoView>("dual_demo_get"),
  stop: () => invoke<DemoView>("dual_demo_stop"),
  disconnect: () => invoke<void>("dual_demo_disconnect"),
};
