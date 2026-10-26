import test from "node:test";
import assert from "node:assert/strict";
import { LEGACY_HISTORY_KEY, MIT_HISTORY_KEY, readWorkspaceHistory, writeWorkspaceHistory } from "../src/motorWorkspaceHistory.ts";

function storage() {
  const values = new Map<string, string>();
  return { values, getItem: (key: string) => values.get(key) ?? null, setItem: (key: string, value: string) => { values.set(key, value); } };
}
const native = { id: "native", data: {} };
const mit = { id: "mit", data: { protocol: "StandardMit" as const } };

test("opening native history migrates existing MIT records before native can overwrite them", () => {
  const store = storage();
  store.setItem(LEGACY_HISTORY_KEY, JSON.stringify([native, mit]));
  assert.deepEqual(readWorkspaceHistory("Native", store), [native]);
  writeWorkspaceHistory("Native", [], store);
  assert.deepEqual(readWorkspaceHistory("StandardMit", store), [mit]);
  assert.deepEqual(JSON.parse(store.getItem(LEGACY_HISTORY_KEY)!), []);
});

test("saving or deleting MIT history does not change native runs or resurrect migrated records", () => {
  const store = storage();
  store.setItem(LEGACY_HISTORY_KEY, JSON.stringify([mit, native]));
  assert.deepEqual(readWorkspaceHistory("StandardMit", store), [mit]);
  writeWorkspaceHistory("StandardMit", [], store);
  assert.deepEqual(readWorkspaceHistory("StandardMit", store), []);
  assert.deepEqual(readWorkspaceHistory("Native", store), [native]);
  assert.deepEqual(JSON.parse(store.getItem(LEGACY_HISTORY_KEY)!), [mit, native]);
});

test("storage quota failure preserves the shared source and permits legacy MIT export", () => {
  const backing = storage();
  const original = JSON.stringify([native, mit]);
  backing.setItem(LEGACY_HISTORY_KEY, original);
  const full = { getItem: backing.getItem, setItem: () => { throw new Error("quota exceeded"); } };
  assert.deepEqual(readWorkspaceHistory("StandardMit", full), [mit]);
  assert.throws(() => writeWorkspaceHistory("Native", [], full), /quota/);
  assert.equal(backing.getItem(LEGACY_HISTORY_KEY), original);
  assert.equal(backing.getItem(MIT_HISTORY_KEY), null);
});

test("a workspace never saves records from the other wire protocol", () => {
  const store = storage();
  writeWorkspaceHistory("StandardMit", [native, mit], store);
  assert.deepEqual(readWorkspaceHistory("StandardMit", store), [mit]);
  writeWorkspaceHistory("Native", [native, mit], store);
  assert.deepEqual(readWorkspaceHistory("Native", store), [native]);
});
