import { invoke } from "@tauri-apps/api/core";
import type { CordEndpoint, PullCordConfig, PullCordMode, PullCordState, TrajectoryCursor, TrajectoryUpdate } from "./pullCordTypes";
import type { FollowInput, PullChassisConfig, PullChassisStatus } from "./pullChassisTypes";
export { errMsg } from "./api";

export interface PullCordEncoderState {
  node_id: number;
  angle_degrees: number | null;
  position_counts: number | null;
  angle_usable: boolean;
  sensor_status: string | null;
  communication: string;
  position_age_ms: number | null;
  status_age_ms: number | null;
  position_frames: number;
  invalid_frames: number;
  bus_state: string | null;
  receive_error: string | null;
}

export interface PullCordStatus {
  chassis: PullChassisStatus;
  connected: boolean;
  iface: string | null;
  pending_node: number | null;
  pending_mode: PullCordMode | null;
  fault: string | null;
  run: PullCordState | null;
  encoder: PullCordEncoderState | null;
  encoder_center_deg: number;
  trajectory: TrajectoryUpdate | null;
}

export const api = {
  chassisConnect: (iface: string, dataBitrate: number) => invoke<void>("pull_chassis_connect", { iface, dataBitrate }),
  chassisDisconnect: () => invoke<void>("pull_chassis_disconnect"),
  chassisStart: (config: PullChassisConfig) => invoke<void>("pull_chassis_start", { config }),
  chassisStop: () => invoke<void>("pull_chassis_stop"),
  chassisSetManual: (enabled: boolean, input: FollowInput) => invoke<void>("pull_chassis_set_manual", { enabled, input }),
  chassisSetConfig: (config: PullChassisConfig) => invoke<void>("pull_chassis_set_config", { config }),
  chassisDefaultConfig: () => invoke<PullChassisConfig>("pull_chassis_default_config"),
  chassisSettingsGet: () => invoke<PullChassisConfig>("pull_chassis_settings_get"),
  chassisSettingsSave: (config: PullChassisConfig) => invoke<void>("pull_chassis_settings_save", { config }),
  status: (trajectoryAfter?: TrajectoryCursor) =>
    invoke<PullCordStatus>("pull_cord_status", { trajectoryAfter }),
  pullCordEndpoint: () => invoke<CordEndpoint | null>("pull_cord_endpoint"),
  pullCordClearTrajectory: (cursor: TrajectoryCursor) =>
    invoke<TrajectoryCursor | null>("pull_cord_clear_trajectory", { cursor }),
  pullMotorConnect: (iface: string, encoderNode: number, encoderCenterDeg: number) =>
    invoke<void>("pull_cord_connect", { iface, encoderNode, encoderCenterDeg }),
  pullMotorDisconnect: () => invoke<void>("pull_cord_disconnect"),
  pullCordStart: (nodeId: number, config: PullCordConfig) =>
    invoke<PullCordState>("pull_cord_start", { nodeId, config }),
  pullCordStop: () => invoke<void>("pull_cord_stop"),
  pullCordRehome: () => invoke<void>("pull_cord_rehome"),
  pullCordSetConfig: (config: PullCordConfig) => invoke<void>("pull_cord_set_config", { config }),
  pullCordDefaultConfig: () => invoke<PullCordConfig>("pull_cord_default_config"),
  settingsGet: () => invoke<PullCordConfig>("pull_cord_settings_get"),
  settingsSave: (config: PullCordConfig) => invoke<PullCordConfig>("pull_cord_settings_save", { config }),
  encoderZero: () => invoke<{ zero_counts: number; applied_live: boolean }>("pull_cord_encoder_zero"),
};
