import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import test, { type TestContext } from "node:test";
import ts from "typescript";
import type { Hopea3State } from "../src/types.ts";

const require = createRequire(import.meta.url);
const React = require("react");
const { act, create } = require("react-test-renderer");

async function mount(t: TestContext) {
  const oldGlobals = ["window", "document", "navigator"].map(key => [key, Object.getOwnPropertyDescriptor(globalThis, key)] as const);
  const timers = new Map<number, { fn: () => unknown; ms: number }>();
  let nextTimer = 0;
  for (const [key, value] of Object.entries({
    window: {
      setInterval(fn: () => unknown, ms: number) { timers.set(++nextTimer, { fn, ms }); return nextTimer; },
      clearInterval(id: number) { timers.delete(id); }, addEventListener() {}, removeEventListener() {},
    },
    document: { activeElement: { tagName: "DIV" } },
    navigator: { getGamepads: () => [] },
  })) Object.defineProperty(globalThis, key, { configurable: true, value });
  const host = (name: string) => (props: any) => React.createElement(name, props, props.children);
  const errors: string[] = [];
  const message = { success() {}, error(error: string) { errors.push(error); } };
  const antd = {
    ...Object.fromEntries(["Alert", "Button", "InputNumber", "Space", "Switch", "Table", "Tag"].map(name => [name, host(name)])),
    Typography: { Text: host("Text") }, App: { useApp: () => ({ message }) },
  };
  const translate = (key: string) => key;
  let status: Hopea3State = {
    pose_x: 0, pose_y: 0, pose_theta: 0, meas_vx: 0, meas_vy: 0, meas_wz: 0,
    cmd_vx: 0, cmd_vy: 0, cmd_wz: 0, max_linear: 3, max_angular: 3,
    running: false, fault: null, motors: [],
  };
  const calls: [string, ...unknown[]][] = [];
  let failStop = false;
  const api = {
    async hopea3GetState() { return structuredClone(status); },
    async hopea3InitProgress() { return { active: false }; },
    async hopea3Start() {
      calls.push(["start"]);
      status = { ...status, running: true, fault: null, motors: [21, 22, 23].map(node_id => ({
        node_id, online: true, enabled: true, target_rev_per_s: 0, velocity_rev_per_s: 0,
        torque_nm: 0, max_torque_permille: 800, driver_temp_c: 25, motor_temp_c: 25, error: null,
      })) };
    },
    async hopea3Stop() {
      calls.push(["stop"]);
      status.running = false;
      if (failStop) { status.fault = "HexMeow disable not confirmed"; throw new Error(status.fault); }
      status.fault = null; status.motors = [];
    },
    async hopea3SetCmd(...values: number[]) { calls.push(["drive", ...values]); },
    async hopea3ReinitMotor(nid: number) { calls.push(["reinit", nid]); status.fault = null; },
  };
  const mocks: Record<string, unknown> = {
    antd, "../api": { api, errMsg: String }, "../format": { nid2hex: String },
    "../i18n": { useI18n: () => ({ t: translate }) },
    "./BasePoseViewer": { BasePoseViewer: host("PoseViewer") },
  };
  const source = new URL("../src/components/Hopea3Panel.tsx", import.meta.url);
  const compiled = ts.transpileModule(readFileSync(source, "utf8"), {
    fileName: source.pathname,
    compilerOptions: { module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX, target: ts.ScriptTarget.ES2021 },
  });
  const module = { exports: {} as any };
  new Function("require", "module", "exports", compiled.outputText)(
    (id: string) => Object.hasOwn(mocks, id) ? mocks[id] : id.endsWith(".css") ? {} : require(id), module, module.exports,
  );
  let renderer: any;
  await act(async () => { renderer = create(React.createElement(module.exports.Hopea3Panel, { connected: true })); });
  t.after(() => {
    act(() => renderer.unmount());
    for (const [key, descriptor] of oldGlobals) {
      if (descriptor) Object.defineProperty(globalThis, key, descriptor); else Reflect.deleteProperty(globalThis, key);
    }
  });
  const button = (label: string) => renderer.root.findAllByType("Button").find((node: any) => node.props.children === label);
  return {
    calls, errors, button,
    get status() { return status; },
    failStop(value: boolean) { failStop = value; },
    reinitButton() { return renderer.root.findByType("Table").props.columns.at(-1).render(null, status.motors[1]); },
    async click(label: string) {
      assert.ok(button(label), `Missing ${label}`);
      assert.ok(!button(label).props.disabled, `${label} is disabled`);
      await act(async () => { await button(label).props.onClick(); });
    },
    async poll() { await act(async () => { for (const timer of [...timers.values()]) if (timer.ms === 50) await timer.fn(); }); },
  };
}

test("HopeA3 opening only observes; drive controls unlock after confirmed startup", async t => {
  const h = await mount(t);
  assert.deepEqual(h.calls, []);
  assert.equal(h.button("▲").props.disabled, true);
  await h.click("hopeStart");
  await h.poll();
  assert.equal(h.button("▲").props.disabled, false);
  assert.deepEqual(h.calls, [["start"]]);
});

test("HexMeow feedback faults inhibit driving but leave motor reinitialization available", async t => {
  const h = await mount(t);
  await h.click("hopeStart");
  h.status.fault = "HexMeow feedback stale";
  await h.poll();
  assert.equal(h.button("▲").props.disabled, true);
  assert.equal(h.reinitButton().props.disabled, false);
  await act(async () => { await h.reinitButton().props.onClick(); });
  await h.poll();
  assert.deepEqual(h.calls.at(-1), ["reinit", 22]);
  assert.equal(h.button("▲").props.disabled, false);
});

test("failed HexMeow disable keeps Stop available even with the control loop stopped", async t => {
  const h = await mount(t);
  await h.click("hopeStart");
  await h.poll();
  h.failStop(true);
  await h.click("hopeStop");
  assert.equal(h.button("▲").props.disabled, true);
  await h.poll();
  assert.equal(h.button("hopeStart"), undefined);
  assert.ok(!h.button("hopeStop").props.disabled);
  assert.equal(h.reinitButton().props.disabled, true);
  assert.ok(h.errors[0].includes("disable not confirmed"));
  h.failStop(false);
  await h.click("hopeStop");
  await h.poll();
  assert.ok(h.button("hopeStart"));
  assert.deepEqual(h.calls, [["start"], ["stop"], ["stop"]]);
});
