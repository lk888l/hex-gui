import type { El05Protocol } from "./el05Api.ts";

export const LEGACY_HISTORY_KEY = "hexmeow.el05.pendulum-history.v2";
export const MIT_HISTORY_KEY = "hexmeow.standard-mit.pendulum-history.v1";
type StorageAccess = Pick<Storage, "getItem" | "setItem">;
type StoredRun = { data?: { protocol?: El05Protocol } };

export function historyKey(protocol: El05Protocol): string {
  return protocol === "StandardMit" ? MIT_HISTORY_KEY : LEGACY_HISTORY_KEY;
}

function readArray(storage: StorageAccess, key: string): unknown[] {
  const raw = storage.getItem(key);
  if (!raw || raw.length > 1_351_000) return [];
  const parsed: unknown = JSON.parse(raw);
  return Array.isArray(parsed) ? parsed : [];
}

function matches(record: unknown, protocol: El05Protocol): boolean {
  return ((record as StoredRun | null)?.data?.protocol ?? "Native") === protocol;
}

// Copy the old MIT runs before either app can overwrite the shared key. Keep
// the source intact; if storage is full, the caller can still export it.
function migrateMitHistory(storage: StorageAccess): void {
  if (storage.getItem(MIT_HISTORY_KEY) != null) return;
  const legacy = readArray(storage, LEGACY_HISTORY_KEY).filter(record => matches(record, "StandardMit"));
  if (legacy.length) storage.setItem(MIT_HISTORY_KEY, JSON.stringify(legacy));
}

export function readWorkspaceHistory(protocol: El05Protocol, storage: StorageAccess): unknown[] {
  try { migrateMitHistory(storage); } catch { /* Read the untouched legacy source below. */ }
  const key = protocol === "StandardMit" && storage.getItem(MIT_HISTORY_KEY) == null ? LEGACY_HISTORY_KEY : historyKey(protocol);
  return readArray(storage, key).filter(record => matches(record, protocol));
}

export function writeWorkspaceHistory(protocol: El05Protocol, records: StoredRun[], storage: StorageAccess): void {
  // A failed migration must prevent an overwrite that would lose old MIT runs.
  migrateMitHistory(storage);
  storage.setItem(historyKey(protocol), JSON.stringify(records.filter(record => matches(record, protocol))));
}
