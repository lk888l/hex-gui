import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import test, { type TestContext } from "node:test";
import ts from "typescript";

const require = createRequire(import.meta.url);
const React = require("react");
const { act, create } = require("react-test-renderer");
const defaults = JSON.parse(readFileSync(new URL("../src-tauri/defaults/pull-chassis.json", import.meta.url), "utf8"));

async function mount(t: TestContext) {
  const oldWindow = Object.getOwnPropertyDescriptor(globalThis, "window");
  const oldNavigator = Object.getOwnPropertyDescriptor(globalThis, "navigator");
  let keydown: (event: any) => void = () => {};
  Object.defineProperty(globalThis, "window", { configurable: true, value: {
    addEventListener(_: string, callback: typeof keydown) { keydown = callback; }, removeEventListener() {},
  } });
  Object.defineProperty(globalThis, "navigator", { configurable: true, value: { userAgent: "Linux" } });
  const host = (name: string) => (props: any) => React.createElement(name, props, props.children);
  const antd: any = Object.fromEntries(["Alert", "Button", "Card", "Checkbox", "Collapse", "Descriptions", "Input", "InputNumber", "Select", "Space", "Table", "Tag"].map(name => [name, host(name)]));
  antd.Descriptions.Item = host("Description");
  antd.Typography = { Paragraph: host("Paragraph") };
  const errors: string[] = [];
  const message = { success() {}, error(error: string) { errors.push(error); } };
  antd.App = { useApp: () => ({ message }) };
  const calls: [string, ...any[]][] = [];
  const status: any = { connected: false, iface: null, needs_stop: false, run: null };
  let failStop = false;
  let pendingStart: Promise<never> | null = null;
  const api = {
    async chassisSettingsGet() { return structuredClone(defaults); },
    async chassisSettingsSave(config: any) { calls.push(["save", config]); },
    async chassisConnect(iface: string, rate: number) { calls.push(["connect", iface, rate]); status.connected = true; status.iface = iface; },
    async chassisStart(config: any) {
      calls.push(["start", config]); status.needs_stop = true;
      status.run = { running: true, config, input: { cordLengthM: 0, cordSpeedMps: 0, encoderAngleDeg: config.follow.encoderCenterDeg }, manual_enabled: false, follow_status: {}, chassis: { motors: [] } };
      if (pendingStart) return pendingStart;
    },
    async chassisSetManual(enabled: boolean, input: any) {
      calls.push(["manual", enabled, structuredClone(input)]); status.run.manual_enabled = enabled; status.run.input = structuredClone(input);
    },
    async chassisStop() {
      calls.push(["stop"]);
      if (failStop) { status.run.running = false; status.run.fault = "disable unconfirmed"; throw new Error(status.run.fault); }
      status.needs_stop = false; status.run = null;
    },
  };
  const mocks: Record<string, unknown> = {
    antd, "../pullCordApi": { api, errMsg: String }, "../i18n": { useI18n: () => ({ lang: "en" }) },
    "./BasePoseViewer": { BasePoseViewer: host("Pose") }, "./PaceFollowControls": { PaceFollowControls: host("Pace") },
    "./BehindSpeedControls": { BehindSpeedControls: host("Behind") },
  };
  const { outputText } = ts.transpileModule(readFileSync(new URL("../src/components/PullChassisPanel.tsx", import.meta.url), "utf8"), {
    compilerOptions: { module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX, target: ts.ScriptTarget.ES2021 },
  });
  const module = { exports: {} as any };
  new Function("require", "module", "exports", outputText)(
    (id: string) => Object.hasOwn(mocks, id) ? mocks[id] : require(id), module, module.exports,
  );
  let renderer: any;
  const render = () => React.createElement(module.exports.PullChassisPanel, {
    status: structuredClone(status), liveInput: { cordLengthM: 0.9, cordSpeedMps: 0, encoderAngleDeg: 60 },
    pullReady: true, unavailable: false, async refresh() { renderer.update(render()); },
  });
  await act(async () => { renderer = create(render()); });
  t.after(() => {
    act(() => renderer.unmount());
    for (const [key, descriptor] of [["window", oldWindow], ["navigator", oldNavigator]] as const) {
      if (descriptor) Object.defineProperty(globalThis, key, descriptor); else Reflect.deleteProperty(globalThis, key);
    }
  });
  const button = (label: string) => renderer.root.findAllByType("Button").find((node: any) => node.props.children === label);
  return {
    calls, errors, status, button,
    failStop() { failStop = true; }, recoverStop() { failStop = false; },
    deferStart(promise: Promise<never>) { pendingStart = promise; },
    async click(label: string) {
      assert.ok(button(label), `Missing ${label}`);
      assert.ok(!button(label).props.disabled, `${label} is disabled`);
      await act(async () => { button(label).props.onClick(); });
    },
    edit(type: string, label: string, value: unknown) {
      act(() => renderer.root.findAllByType(type).find((node: any) => node.props["aria-label"] === label).props.onChange(value));
    },
    async escape() { await act(async () => { keydown({ code: "Escape", preventDefault() {} }); }); },
  };
}

test("chassis drafts and connection never arm; manual motion requires explicit apply and failed stop is retryable", async t => {
  const h = await mount(t);
  assert.deepEqual(h.calls, []);
  h.edit("Select", "Follow input source", "manual");
  h.edit("InputNumber", "Cord length m", 0.75);
  await h.click("Save configuration");
  await h.click("Connect chassis");
  assert.deepEqual(h.calls.map(call => call[0]), ["save", "connect"]);
  assert.deepEqual(h.calls.at(-1), ["connect", "can0", 5_000_000]);
  await h.click("Arm manual bench");
  assert.equal(h.status.run.manual_enabled, false);
  assert.equal(h.status.run.input.cordLengthM, 0);
  await h.click("Apply manual input");
  assert.equal(h.status.run.input.cordLengthM, 0.75);
  h.edit("InputNumber", "Cord length m", 0.9);
  assert.equal(h.status.run.input.cordLengthM, 0.75);
  await h.click("Release manual input");
  assert.equal(h.status.run.manual_enabled, false);
  h.failStop();
  await h.click("STOP / Esc");
  assert.equal(h.button("Arm manual bench").props.disabled, true);
  assert.ok(!h.button("STOP / Esc").props.disabled);
  assert.ok(h.errors.some(error => error.includes("disable unconfirmed")));
  h.recoverStop();
  await h.escape();
  assert.equal(h.status.needs_stop, false);
});

test("Escape can stop the chassis while initialization is pending", async t => {
  const h = await mount(t);
  await h.click("Connect chassis");
  let reject!: (error: Error) => void;
  h.deferStart(new Promise<never>((_, no) => { reject = no; }));
  await h.click("Start physical follow");
  assert.ok(!h.button("STOP / Esc").props.disabled);
  await h.escape();
  assert.equal(h.calls.at(-1)?.[0], "stop");
  await act(async () => { reject(new Error("start cancelled")); });
  assert.ok(h.errors.some(error => error.includes("start cancelled")));
});
