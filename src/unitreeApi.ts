import { invoke } from "@tauri-apps/api/core";
import type { UnitreeBusTestRequest, UnitreeSettings, UnitreeTarget } from "./unitreeServo";

export interface UnitreeBusTestReport {
  kind: "communication" | "motion";
  phase: "starting" | "running" | "completed" | "stopped" | "failed";
  running: boolean;
  duration_s: number;
  started_at_s: number;
  elapsed_s: number;
  cycles: number;
  overruns: number;
  error: string | null;
  nodes: {
    id: number; target: UnitreeTarget;
    tx_count: number; rx_count: number; timeout_count: number; crc_error_count: number;
    consecutive_failures: number; mean_rtt_ms: number; max_rtt_ms: number;
    stop_confirmed: boolean; last_error: string | null; feedback: UnitreeFeedback | null;
  }[];
}

export interface UnitreeFeedback {
  id: number;
  mode: number;
  timed_out: boolean;
  shell_temperature_c: number;
  winding_temperature_c: number;
  voltage_v: number;
  torque_nm: number;
  velocity_rad_s: number;
  position_rad: number;
  output_angle_rad: number;
  error_bits: number;
  warning_bits: number;
  errors: string[];
  warnings: string[];
}

export interface UnitreeState {
  connected: boolean;
  port: string;
  baud_rate: number;
  selected_id: number | null;
  online: boolean;
  streaming: boolean;
  motion_active: boolean;
  stop_confirmed: boolean;
  id_change: {
    old_id: number;
    new_id: number;
    phase: "checking" | "entering_boot" | "unlocking" | "writing" | "saving" | "verifying" | "restoring" | "complete" | "failed";
    write_attempted: boolean;
    saved: boolean;
    error: string | null;
  } | null;
  bootloader_ids: number[];
  bus_test: UnitreeBusTestReport | null;
  pending_stop_ids: number[];
  settings: UnitreeSettings;
  target: UnitreeTarget | null;
  feedback: UnitreeFeedback | null;
  feedback_age_ms: number | null;
  feedback_sequence: number;
  feedback_time_s: number | null;
  devices: { id: number; online: boolean; feedback: UnitreeFeedback }[];
  tx_count: number;
  rx_count: number;
  timeout_count: number;
  crc_error_count: number;
  last_tx_hex: string | null;
  last_rx_hex: string | null;
  last_error: string | null;
}

export const unitreeApi = {
  listPorts: () => invoke<{ name: string; description: string }[]>("unitree_list_ports"),
  connect: (port: string, id: number, settings: UnitreeSettings) => invoke<UnitreeState>("unitree_connect", { port, id, settings }),
  disconnect: () => invoke<void>("unitree_disconnect"),
  getState: () => invoke<UnitreeState | null>("unitree_get_state"),
  scan: () => invoke<UnitreeState>("unitree_scan"),
  select: (id: number) => invoke<UnitreeState>("unitree_select", { id }),
  sendTarget: (target: UnitreeTarget, repeat: boolean) => invoke<UnitreeState>("unitree_send_target", { target, repeat }),
  stop: () => invoke<UnitreeState>("unitree_stop"),
  configure: (settings: UnitreeSettings) => invoke<UnitreeState>("unitree_configure", { settings }),
  changeId: (oldId: number, newId: number) => invoke<UnitreeState>("unitree_change_id", { oldId, newId }),
  restoreApp: (id: number) => invoke<UnitreeState>("unitree_restore_app", { id }),
  setMode: (mode: number) => invoke<UnitreeState>("unitree_set_mode", { mode }),
  startBusTest: (request: UnitreeBusTestRequest) => invoke<UnitreeState>("unitree_start_bus_test", { request }),
};
