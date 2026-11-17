import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import test, { type TestContext } from "node:test";
import ts from "typescript";
import type { PullCordConfig } from "../src/pullCordTypes.ts";

const require = createRequire(import.meta.url);
const React = require("react");
const { act, create } = require("react-test-renderer");
const defaults: PullCordConfig = JSON.parse(readFileSync(new URL("../src-tauri/defaults/pull-cord.json", import.meta.url), "utf8"));
const chassisDefaults = JSON.parse(readFileSync(new URL("../src-tauri/defaults/pull-chassis.json", import.meta.url), "utf8"));

function component(name: string, mocks: Record<string, unknown>) {
  const url = new URL(`../src/components/${name}.tsx`, import.meta.url);
  const { outputText } = ts.transpileModule(readFileSync(url, "utf8"), {
    compilerOptions: { module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX, target: ts.ScriptTarget.ES2021 },
    fileName: url.pathname,
  });
  const module = { exports: {} as any };
  new Function("require", "module", "exports", outputText)(
    (id: string) => Object.hasOwn(mocks, id) ? mocks[id] : id.endsWith(".css") ? {} : require(id), module, module.exports,
  );
  return module.exports[name];
}

async function mount(t: TestContext) {
  const oldWindow = Object.getOwnPropertyDescriptor(globalThis, "window");
  const oldNavigator = Object.getOwnPropertyDescriptor(globalThis, "navigator");
  let tick: (() => Promise<void>) | undefined;
  Object.defineProperty(globalThis, "window", { configurable: true, value: {
    setInterval(fn: () => Promise<void>) { tick = fn; return 1; }, clearInterval() { tick = undefined; },
  } });
  Object.defineProperty(globalThis, "navigator", { configurable: true, value: { userAgent: "Windows" } });
  const host = (name: string) => (props: any) => React.createElement(name, props, props.children);
  const antd: any = Object.fromEntries(["Alert", "Button", "Card", "Descriptions", "Input", "InputNumber", "Progress", "Segmented", "Space", "Tag"].map(name => [name, host(name)]));
  antd.Descriptions.Item = host("Description");
  antd.Typography = { Text: host("Text"), Paragraph: host("Paragraph") };
  const errors: string[] = [];
  const message = { success() {}, warning() {}, error(error: string) { errors.push(error); } };
  antd.App = { useApp: () => ({ message }) };
  let saved = structuredClone(defaults);
  let status: any = { connected: false, iface: null, pending_node: null, fault: null, run: null, encoder: null };
  const calls: [string, ...unknown[]][] = [];
  let failStop = false;
  let startWait: Promise<never> | null = null;
  const api = {
    async status() { return structuredClone(status); },
    async settingsGet() { return structuredClone(saved); },
    async chassisSettingsGet() { return structuredClone(chassisDefaults); },
    async settingsSave(config: PullCordConfig) { calls.push(["save", config]); saved = structuredClone(config); return structuredClone(saved); },
    async pullCordDefaultConfig() { calls.push(["defaults"]); return structuredClone(defaults); },
    async pullMotorConnect(iface: string, encoderNode: number, centre: number) {
      calls.push(["connect", iface, encoderNode, centre]); status.connected = true; status.iface = iface;
    },
    async pullMotorDisconnect() { calls.push(["disconnect"]); status = { ...status, connected: false, pending_node: null, run: null }; },
    async pullCordStart(node: number, config: PullCordConfig) {
      calls.push(["start", node, config]); status.pending_node = node;
      if (startWait) return startWait;
      status.run = { running: true, phase: "running", node_id: node, config: structuredClone(config), homing_progress: 1 };
      return structuredClone(status.run);
    },
    async pullCordStop() {
      calls.push(["stop"]);
      if (failStop) {
        if (status.run) { status.run.running = false; status.run.phase = "fault"; }
        status.fault = "Close not acknowledged";
        throw new Error(status.fault);
      }
      status.run = null; status.pending_node = null; status.fault = null;
    },
    async pullCordSetConfig(config: PullCordConfig) { calls.push(["apply", config]); status.run.config = structuredClone(config); },
    async pullCordRehome() { calls.push(["rehome"]); },
  };
  const Panel = component("PullCordPanel", {
    antd, "../pullCordApi": { api, errMsg: String },
    "../i18n": { useI18n: () => ({ lang: "en" }) },
    "./PullCordChart": { PullCordChart: host("Chart") },
    "./PullCordEncoder": { PullCordEncoder: host("Encoder") },
    "./PullChassisPanel": { PullChassisPanel: host("Chassis") },
  });
  let renderer: any;
  await act(async () => { renderer = create(React.createElement(Panel)); });
  t.after(() => {
    act(() => renderer.unmount());
    for (const [key, descriptor] of [["window", oldWindow], ["navigator", oldNavigator]] as const) {
      if (descriptor) Object.defineProperty(globalThis, key, descriptor); else Reflect.deleteProperty(globalThis, key);
    }
  });
  const button = (label: string) => renderer.root.findAllByType("Button").find((node: any) => node.props.children === label);
  return {
    calls, errors, button, api,
    get status() { return status; },
    get chassisProps() { return renderer.root.findByType("Chassis").props; },
    failStop(value: boolean) { failStop = value; },
    deferStart(promise: Promise<never>) { startWait = promise; },
    async click(label: string) {
      assert.ok(button(label), `Missing ${label}`);
      assert.ok(!button(label).props.disabled, `${label} is disabled`);
      await act(async () => { button(label).props.onClick(); });
    },
    edit(label: string, value: number) {
      act(() => renderer.root.findAllByType("InputNumber").find((node: any) => node.props["aria-label"] === label).props.onChange(value));
    },
    value(label: string) { return renderer.root.findAllByType("InputNumber").find((node: any) => node.props["aria-label"] === label).props.value; },
    async poll() { await act(async () => { await tick?.(); }); },
  };
}

test("opening and saving the pull-cord app never connects or enables a motor", async t => {
  const h = await mount(t);
  assert.deepEqual(h.calls, []);
  assert.equal(h.value("Drum radius"), 0.026);
  assert.equal(h.value("Encoder centre"), chassisDefaults.follow.encoderCenterDeg);
  assert.equal(h.button("Start / home").props.disabled, true);
  await h.click("Save locally");
  assert.deepEqual(h.calls.map(c => c[0]), ["save"]);
  await h.click("Connect Classic CAN 1 Mbps");
  assert.deepEqual(h.calls.at(-1), ["connect", "gs_usb0", 16, chassisDefaults.follow.encoderCenterDeg]);
  assert.ok(!h.calls.some(c => c[0] === "start"));
  await h.click("Start / home");
  assert.deepEqual(h.calls.at(-1), ["start", 1, defaults]);
});

test("save, apply and reset keep the draft, saved template and running settings separate", async t => {
  const h = await mount(t);
  await h.click("Connect Classic CAN 1 Mbps");
  await h.click("Start / home");
  h.edit("Speed-loop Kp", 25);
  await h.click("Save locally");
  assert.equal(h.status.run.config.a2_speed.speed_kp, 20);
  await h.click("Apply to run");
  assert.equal(h.status.run.config.a2_speed.speed_kp, 25);
  h.edit("Speed-loop Kp", 30);
  await h.click("Reset defaults");
  assert.equal(h.value("Speed-loop Kp"), 20);
  assert.equal(h.status.run.config.a2_speed.speed_kp, 25);
  assert.deepEqual(h.calls.map(c => c[0]), ["connect", "start", "save", "apply", "defaults"]);
});

test("the automatic chassis brake threshold displays the applied cord target while running", async t => {
  const h = await mount(t);
  assert.equal(h.chassisProps.retractTargetSpeedMps, defaults.retract_speed_m_s);
  await h.click("Connect Classic CAN 1 Mbps");
  await h.click("Start / home");
  h.edit("Retract target speed", 1.5);
  assert.equal(h.chassisProps.retractTargetSpeedMps, defaults.retract_speed_m_s);
  await h.click("Save locally");
  assert.equal(h.chassisProps.retractTargetSpeedMps, defaults.retract_speed_m_s);
  await h.click("Apply to run");
  assert.equal(h.chassisProps.retractTargetSpeedMps, 1.5);
  await h.click("Reset defaults");
  assert.equal(h.chassisProps.retractTargetSpeedMps, 1.5);
});

test("a failed stop remains retryable even after the control task is no longer running", async t => {
  const h = await mount(t);
  await h.click("Connect Classic CAN 1 Mbps");
  await h.click("Start / home");
  h.failStop(true);
  await h.click("Stop and close motor");
  assert.equal(h.status.run.running, false);
  assert.equal(h.button("Start / home").props.disabled, true);
  assert.equal(h.button("Stop and close motor").props.disabled, false);
  assert.ok(h.errors.some(error => error.includes("Close not acknowledged")));
  h.failStop(false);
  await h.click("Stop and close motor");
  assert.equal(h.status.pending_node, null);
  assert.equal(h.button("Start / home").props.disabled, false);
});

test("Stop remains available while Start is waiting for motor initialization", async t => {
  const h = await mount(t);
  await h.click("Connect Classic CAN 1 Mbps");
  let rejectStart!: (error: Error) => void;
  h.deferStart(new Promise<never>((_, reject) => { rejectStart = reject; }));
  await h.click("Start / home");
  assert.equal(h.button("Stop and close motor").props.disabled, false);
  await h.click("Stop and close motor");
  assert.equal(h.button("Start / home").props.disabled, true, "pending startup must still keep actions busy");
  await act(async () => { rejectStart(new Error("start cancelled")); });
  assert.equal(h.button("Start / home").props.disabled, false);
});

test("homing chart samples without length feedback do not produce infinite axis bounds", () => {
  const Chart = component("PullCordChart", {
    "../i18n": { useI18n: () => ({ t: (key: string) => key }) },
    "echarts-for-react": { __esModule: true, default: (props: any) => React.createElement("chart", props) },
  });
  let renderer: any;
  act(() => { renderer = create(React.createElement(Chart, {
    samples: { current: [{ t: 1, length_m: null, torque_limit_raw: null, global_torque_limit_raw: null, feedback_raw: null, speed_m_s: null, target_speed_m_s: null }] },
    chartVersion: 1, windowSec: 20,
  })); });
  for (const axis of renderer.root.findByType("chart").props.option.yAxis) {
    assert.ok(axis.min == null || Number.isFinite(axis.min));
    assert.ok(axis.max == null || Number.isFinite(axis.max));
  }
  act(() => renderer.unmount());
});
