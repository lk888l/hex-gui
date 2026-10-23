import test from "node:test";
import assert from "node:assert/strict";
import {
  DEFAULT_UNITREE_SETTINGS, DEFAULT_UNITREE_TARGET, parseUnitreeProfile,
  validateUnitreeId, validateUnitreeIdChange, validateUnitreeSettings, validateUnitreeTarget, validateUnitreeBusTest,
} from "../src/unitreeServo.ts";

test("servo ID never accepts the broadcast address or fractional IDs", () => {
  assert.equal(validateUnitreeId(0), 0);
  assert.equal(validateUnitreeId(14), 14);
  for (const value of [15, -1, 1.5, NaN, Infinity]) assert.throws(() => validateUnitreeId(value));
});

test("ID writes require an explicit different valid destination", () => {
  assert.deepEqual(validateUnitreeIdChange(0, 14), { oldId: 0, newId: 14 });
  assert.deepEqual(validateUnitreeIdChange(14, 0), { oldId: 14, newId: 0 });
  for (const [oldId, newId] of [[3, 3], [0, 15], [15, 0], [0, 1.5], [0, NaN]]) {
    assert.throws(() => validateUnitreeIdChange(oldId, newId));
  }
});

test("multi-servo tests require distinct unicast IDs and a bounded duration", () => {
  const request = { kind: "motion" as const, duration_s: 10, nodes: [0, 3, 14].map(id => ({ id, target: { ...DEFAULT_UNITREE_TARGET } })) };
  assert.equal(validateUnitreeBusTest(request, DEFAULT_UNITREE_SETTINGS).nodes.length, 3);
  for (const ids of [[0], [0, 0], [0, 15], [0, -1], [0, 1.5]]) {
    assert.throws(() => validateUnitreeBusTest({ ...request, nodes: ids.map(id => ({ id, target: DEFAULT_UNITREE_TARGET })) }, DEFAULT_UNITREE_SETTINGS));
  }
  for (const duration_s of [0, 121, 1.5, NaN]) assert.throws(() => validateUnitreeBusTest({ ...request, duration_s }, DEFAULT_UNITREE_SETTINGS));
});

test("communication tests always send zero targets and motion tests validate every servo target", () => {
  const nodes = [0, 1].map(id => ({ id, target: { ...DEFAULT_UNITREE_TARGET, torque_nm: 0.6 } }));
  assert.throws(() => validateUnitreeBusTest({ kind: "motion", duration_s: 5, nodes }, DEFAULT_UNITREE_SETTINGS));
  const checked = validateUnitreeBusTest({ kind: "communication", duration_s: 5, nodes }, DEFAULT_UNITREE_SETTINGS);
  assert.ok(checked.nodes.every(node => Object.values(node.target).every(value => value === 0)));
  assert.equal(nodes[0].target.torque_nm, 0.6);
});

test("profiles restore only validated input values without auto-start flags", () => {
  const source = {
    schema_version: 1, port: "/dev/ttyACM0", selected_id: 3,
    settings: DEFAULT_UNITREE_SETTINGS, target: DEFAULT_UNITREE_TARGET,
    streaming: true, connected: true,
  };
  const profile = parseUnitreeProfile(JSON.stringify(source));
  assert.equal(profile.port, "/dev/ttyACM0");
  assert.equal(profile.selected_id, 3);
  assert.ok(!("streaming" in profile) && !("connected" in profile));
  assert.throws(() => parseUnitreeProfile(JSON.stringify({ ...source, schema_version: 2 })));
  assert.throws(() => parseUnitreeProfile(JSON.stringify({ ...source, selected_id: 15 })));
  assert.throws(() => parseUnitreeProfile(JSON.stringify({ ...source, target: { ...source.target, torque_nm: 1 } })));
});

test("motion validates host limits, finite values and physical gain ranges", () => {
  assert.deepEqual(validateUnitreeTarget(DEFAULT_UNITREE_TARGET, DEFAULT_UNITREE_SETTINGS), DEFAULT_UNITREE_TARGET);
  for (const target of [
    { ...DEFAULT_UNITREE_TARGET, velocity_rad_s: 1.01 },
    { ...DEFAULT_UNITREE_TARGET, torque_nm: -0.51 },
    { ...DEFAULT_UNITREE_TARGET, position_rad: 7 },
    { ...DEFAULT_UNITREE_TARGET, kp: -1 },
    { ...DEFAULT_UNITREE_TARGET, kd: 22 },
    { ...DEFAULT_UNITREE_TARGET, position_rad: NaN },
  ]) assert.throws(() => validateUnitreeTarget(target, DEFAULT_UNITREE_SETTINGS));
});

test("session settings reject incompatible timing and invalid position limits", () => {
  for (const settings of [
    { ...DEFAULT_UNITREE_SETTINGS, command_hz: 60 },
    { ...DEFAULT_UNITREE_SETTINGS, command_hz: 100.1 },
    { ...DEFAULT_UNITREE_SETTINGS, response_timeout_ms: 101 },
    { ...DEFAULT_UNITREE_SETTINGS, position_min_rad: 7 },
    { ...DEFAULT_UNITREE_SETTINGS, torque_limit_nm: 0 },
    { ...DEFAULT_UNITREE_SETTINGS, velocity_limit_rad_s: Infinity },
    { ...DEFAULT_UNITREE_SETTINGS, timeout_protection: "yes" },
  ]) assert.throws(() => validateUnitreeSettings(settings as typeof DEFAULT_UNITREE_SETTINGS));
});
