import { invoke } from "@tauri-apps/api/core";
import type { ConnectionInfo } from "./types";
import type { HpmImuObject, HpmImuPdo, HpmImuSnapshot, HpmImuValue, HpmImuTestConfig, HpmImuTestSnapshot } from "./hpmImu";

export const hpmImuApi = {
  connect: (spec: string) => invoke<{ link: ConnectionInfo; objects: HpmImuObject[] }>("hpm_imu_connect", { spec }),
  disconnect: () => invoke<void>("hpm_imu_disconnect"),
  state: () => invoke<HpmImuSnapshot>("hpm_imu_state"),
  read: (node: number) => invoke<HpmImuSnapshot>("hpm_imu_read", { node }),
  configure: (node: number, pdos: HpmImuPdo[] | null, periodMs: number) => invoke<HpmImuSnapshot>("hpm_imu_configure", { node, pdos, periodMs }),
  nmt: (node: number, command: number) => invoke<void>("hpm_imu_nmt", { node, command }),
  sync: (node: number) => invoke<void>("hpm_imu_sync", { node }),
  diagnostics: (node: number) => invoke<HpmImuValue[]>("hpm_imu_diagnostics", { node }),
  exportCsv: () => invoke<string>("hpm_imu_export_csv"),
  testStart: (config: HpmImuTestConfig) => invoke<HpmImuTestSnapshot>("hpm_imu_test_start", { config }),
  testStop: () => invoke<void>("hpm_imu_test_stop"),
};
