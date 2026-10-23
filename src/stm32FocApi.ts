import { invoke } from "@tauri-apps/api/core";

export type FocMode = "SingleTurn" | "Position" | "Velocity" | "Torque" | "Mit";

export interface FocConfig {
  mode: FocMode;
  current_limit_permille: number;
  profile_velocity: number;
  acceleration: number;
  deceleration: number;
  quick_stop_deceleration: number;
  following_error_window: number;
  following_error_timeout_ms: number;
}

export interface FocTarget {
  position_rev: number;
  velocity_rev_s: number;
  torque_permille: number;
  torque_ff_nm: number;
  kp: number;
  kd: number;
  relative: boolean;
}

export interface FocState {
  node_id: number;
  initialized: boolean;
  streaming: boolean;
  enabled: boolean;
  mode: FocMode;
  actual_mode: number | null;
  statusword: number | null;
  error_code: number | null;
  calibration_state: number | null;
  position_rev: number | null;
  velocity_rev_s: number | null;
  torque_permille: number | null;
  voltage_v: number | null;
  temperature_c: number | null;
  nmt_state: number | null;
  feedback_age_ms: number | null;
  tx_count: number;
  rx_count: number;
  phase: string;
  last_error: string | null;
  identity: [number, number, number, number] | null;
}

export interface FocConnection {
  connected: boolean;
  selected_node_id: number | null;
}

export interface FocDevice {
  node_id: number;
  online: boolean;
  nmt_state: number;
  heartbeat_age_ms: number;
  heartbeat_count: number;
  selected: boolean;
  initialized: boolean;
  enabled: boolean;
  identity: [number, number, number, number] | null;
  error_code: number | null;
}

export const stm32FocApi = {
  connect: (iface: string, dataBitrate: number) =>
    invoke<FocConnection>("stm32_foc_connect", { iface, dataBitrate }),
  disconnect: () => invoke<void>("stm32_foc_disconnect"),
  listDevices: () => invoke<FocDevice[]>("stm32_foc_list_devices"),
  selectNode: (nodeId: number) => invoke<FocState>("stm32_foc_select_node", { nodeId }),
  forgetOffline: () => invoke<void>("stm32_foc_forget_offline"),
  getState: () => invoke<FocState>("stm32_foc_get_state"),
  initialize: (config: FocConfig) => invoke<FocState>("stm32_foc_initialize", { config }),
  calibrate: () => invoke<FocState>("stm32_foc_calibrate"),
  enable: () => invoke<void>("stm32_foc_enable"),
  setTarget: (target: FocTarget) => invoke<void>("stm32_foc_set_target", { target }),
  halt: (halted: boolean) => invoke<void>("stm32_foc_halt", { halted }),
  quickStop: () => invoke<void>("stm32_foc_quick_stop"),
  disable: () => invoke<void>("stm32_foc_disable"),
  resetFault: () => invoke<FocState>("stm32_foc_reset_fault"),
  save: () => invoke<void>("stm32_foc_save"),
};
