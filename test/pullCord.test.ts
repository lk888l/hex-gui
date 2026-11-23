import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import test, { type TestContext } from "node:test";
import ts from "typescript";
import type { CordEndpoint, PullCordConfig, PullCordMode, TrajectoryCursor } from "../src/pullCordTypes.ts";

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
  antd.Card = ({ extra, children, ...props }: any) => React.createElement("Card", props, extra, children);
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
    async status(_after?: TrajectoryCursor) { return structuredClone(status); },
    async pullCordClearTrajectory(displayed: TrajectoryCursor): Promise<TrajectoryCursor | null> {
      calls.push(["clearTrail", displayed]);
      const trace = status.trajectory;
      if (trace?.cursor.frame_id !== displayed.frame_id) return null;
      const cursor = { frame_id: displayed.frame_id, seq: trace.cursor.seq + 1 };
      status.trajectory = { ...trace, cursor, points: [], retained_points: 0 };
      return cursor;
    },
    async settingsGet() { return structuredClone(saved); },
    async chassisSettingsGet() { return structuredClone(chassisDefaults); },
    async settingsSave(config: PullCordConfig) { calls.push(["save", config]); saved = structuredClone(config); return structuredClone(saved); },
    async pullCordDefaultConfig() { calls.push(["defaults"]); return structuredClone(defaults); },
    async pullMotorConnect(iface: string, encoderNode: number, centre: number) {
      calls.push(["connect", iface, encoderNode, centre]); status.connected = true; status.iface = iface;
    },
    async pullMotorDisconnect() { calls.push(["disconnect"]); status = { ...status, connected: false, pending_node: null, pending_mode: null, run: null }; },
    async pullCordStart(node: number, config: PullCordConfig) {
      calls.push(["start", node, config]); status.pending_node = node; status.pending_mode = config.control_mode;
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
      status.run = null; status.pending_node = null; status.pending_mode = null; status.fault = null;
    },
    async pullCordSetConfig(config: PullCordConfig) { calls.push(["apply", config]); status.run.config = structuredClone(config); },
    async pullCordRehome() { calls.push(["rehome"]); },
  };
  const i18n = { useI18n: () => ({ lang: "en" }) };
  const Trajectory = component("PullCordTrajectory", {
    antd, "../i18n": i18n, "echarts-for-react": { __esModule: true, default: host("XYChart") },
  });
  const Panel = component("PullCordPanel", {
    antd, "../pullCordApi": { api, errMsg: String }, "../i18n": i18n,
    "./PullCordChart": { PullCordChart: host("Chart") },
    "./PullCordEncoder": { PullCordEncoder: host("Encoder") },
    "./PullCordTrajectory": { PullCordTrajectory: Trajectory },
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
  const modeSelector = () => renderer.root.findAllByType("Segmented").find((node: any) => node.props["aria-label"] === "Cord control mode").props;
  const field = (label: string) => renderer.root.findAllByType("InputNumber").find((node: any) => node.props["aria-label"] === label)?.props;
  return {
    calls, errors, button, api, field, modeSelector,
    chooseMode(mode: PullCordMode) {
      assert.ok(!modeSelector().disabled, "mode selector is locked");
      act(() => modeSelector().onChange(mode));
    },
    get chartProps() { return renderer.root.findByType("Chart").props; },
    get trajectoryProps() { return renderer.root.findByType(Trajectory).props; },
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
    max(label: string) { return renderer.root.findAllByType("InputNumber").find((node: any) => node.props["aria-label"] === label).props.max; },
    async poll() { await act(async () => { void tick?.(); }); },
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

test("homing and retraction speed inputs allow 1000 rpm for the configured drum", async t => {
  const h = await mount(t);
  for (const [radius, limit] of [[0.026, 2.7227136331111543], [0.005, 0.5235987755982989]]) {
    h.edit("Drum radius", radius);
    for (const label of ["Homing target speed", "Retract target speed"]) {
      assert.ok(Math.abs(h.max(label) - limit) < 1e-12, `${label} at radius ${radius}`);
    }
  }
  assert.deepEqual(h.calls, []);
});

test("select, tune and start A1, then switch to the retained A2 profile after a confirmed stop", async t => {
  const h = await mount(t);
  assert.equal(h.modeSelector().value, "a2_speed");
  h.edit("Speed-loop Kp", 30);
  h.chooseMode("a1_current");
  assert.deepEqual(["Payout current", "Stationary current", "Retract current"].map(label => h.value(label)),
    [100, 150, 200]);
  h.edit("Payout current", 80);
  h.edit("Stationary current", 127);
  h.edit("Current command ceiling", 500);
  assert.equal(h.field("Speed-loop Kp"), undefined);
  assert.equal(h.field("Retract target speed"), undefined);
  assert.deepEqual(h.calls, []);
  await h.click("Connect Classic CAN 1 Mbps");
  assert.equal(h.modeSelector().disabled, false);
  h.edit("Retract current", 126);
  assert.equal(h.button("Start / home").props.disabled, true);
  assert.equal(h.button("Save locally").props.disabled, true);
  h.edit("Retract current", 190);
  await h.click("Save locally");
  const saved = h.calls.at(-1)?.[1] as PullCordConfig;
  assert.equal(saved.control_mode, "a1_current");
  assert.equal(saved.a1_current?.payout_current_raw, 80);
  assert.equal(saved.a1_current?.retract_current_raw, 190);
  await h.click("Start / home");
  assert.equal(h.status.run.config.control_mode, "a1_current");
  assert.equal(h.status.run.config.a1_current.near_raw, 127);
  assert.equal(h.modeSelector().disabled, true);
  assert.equal(h.field("Current command ceiling").disabled, true);
  h.status.run.current_command_raw = -127;
  h.status.run.measured_current_raw = -89;
  await h.poll();
  const sample = h.chartProps.samples.current.at(-1);
  assert.equal(h.chartProps.mode, "a1_current");
  assert.equal(sample.current_command_raw, -127);
  assert.equal(sample.feedback_raw, -89);
  assert.equal(sample.target_speed_m_s, null);
  assert.equal(sample.torque_limit_raw, null);
  h.edit("Payout current", 130);
  assert.equal(h.button("Apply to run").props.disabled, true);
  h.edit("Payout current", 73);
  h.edit("Stationary current", 103);
  h.edit("Retract current", 215);
  await h.click("Apply to run");
  assert.equal(h.status.run.config.a1_current.near_raw, 103);
  assert.equal(h.status.run.config.a1_current.payout_current_raw, 73);
  assert.equal(h.status.run.config.a1_current.retract_current_raw, 215);
  h.failStop(true);
  await h.click("Stop and close motor");
  assert.equal(h.modeSelector().disabled, true);
  h.failStop(false);
  await h.click("Stop and close motor");
  h.chooseMode("a2_speed");
  assert.equal(h.field("Payout current"), undefined);
  assert.equal(h.field("Retract current"), undefined);
  assert.equal(h.value("Payout torque limit (raw)"), defaults.a2_speed!.payout_raw);
  assert.equal(h.value("Retract torque limit (raw)"), defaults.a2_speed!.retract_raw);
  assert.equal(h.value("Speed-loop Kp"), 30);
  await h.click("Start / home");
  assert.equal(h.status.run.config.control_mode, "a2_speed");
  assert.equal(h.status.run.config.a1_current.near_raw, 103);
});

test("resetting an A1 draft during a run preserves the mode and setup-only current ceiling", async t => {
  const h = await mount(t);
  h.chooseMode("a1_current");
  h.edit("Current command ceiling", 200);
  await h.click("Connect Classic CAN 1 Mbps");
  await h.click("Start / home");
  h.edit("Stationary current", 171);
  await h.click("Reset defaults");
  assert.equal(h.modeSelector().value, "a1_current");
  assert.equal(h.value("Current command ceiling"), 200);
  assert.equal(h.value("Stationary current"), defaults.a1_current!.near_raw);
  await h.click("Apply to run");
  assert.equal(h.status.run.config.control_mode, "a1_current");
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
  assert.equal(h.modeSelector().disabled, true);
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
    samples: { current: [{ t: 1, control_mode: "a1_current", length_m: null, current_command_raw: -150, torque_limit_raw: null, global_torque_limit_raw: null, feedback_raw: null, speed_m_s: null, target_speed_m_s: null }] },
    mode: "a1_current", chartVersion: 1, windowSec: 20,
  })); });
  const { legend, series, yAxis } = renderer.root.findByType("chart").props.option;
  assert.ok(legend.data.includes("chartCurrentCommand"));
  assert.ok(!legend.data.includes("chartTorqueLimit"));
  assert.ok(!legend.data.includes("chartTargetSpeed"));
  assert.deepEqual(series.find((s: any) => s.name === "chartCurrentCommand").data, [[0, -150]]);
  for (const axis of yAxis) {
    assert.ok(axis.min == null || Number.isFinite(axis.min));
    assert.ok(axis.max == null || Number.isFinite(axis.max));
  }
  act(() => renderer.unmount());
});

function endpoint(seq: number, segment = 1, frame_id = 1): CordEndpoint {
  return { frame_id, segment, time_us: seq * 40_000, center_deg: 275, x_m: seq / 10, y_m: 0.25,
    length_m: Math.hypot(seq / 10, 0.25), bearing_rad: Math.atan2(0.25, seq / 10),
    radial_speed_m_s: 0, velocity_m_s: null };
}

test("trajectory polling retains missed plot points, expires the marker and replaces a rehomed trace", async t => {
  const h = await mount(t);
  await h.click("Connect Classic CAN 1 Mbps");
  await h.click("Start / home");
  const [a, b, c] = [endpoint(1), endpoint(2), endpoint(3)];
  h.status.trajectory = { cursor: { frame_id: 1, seq: 2 }, retained_points: 2, points: [a, b], latest: b };
  await h.poll();
  assert.deepEqual(h.trajectoryProps.points, [a, b]);
  assert.deepEqual(h.trajectoryProps.latest, b);
  h.status.trajectory = { cursor: { frame_id: 1, seq: 3 }, retained_points: 2, points: [c], latest: null };
  await h.poll();
  assert.deepEqual(h.trajectoryProps.points, [b, c]);
  assert.equal(h.trajectoryProps.latest, null);
  const newFrame = endpoint(4, 1, 2);
  h.status.trajectory = { cursor: { frame_id: 2, seq: 1 }, retained_points: 1, points: [newFrame], latest: newFrame };
  await h.poll();
  assert.deepEqual(h.trajectoryProps.points, [newFrame]);
  await h.click("Stop and close motor");
  assert.equal(h.trajectoryProps.latest, null);
  assert.deepEqual(h.trajectoryProps.points, [newFrame]);
  assert.deepEqual(h.calls.map(c => c[0]), ["connect", "start", "stop"]);
});

test("Clear trail keeps live feedback, rejects a delayed poll and can clear a stopped run", async t => {
  const h = await mount(t);
  assert.equal(h.button("Clear trail").props.disabled, true);
  await h.click("Connect Classic CAN 1 Mbps");
  await h.click("Start / home");
  const [a, b, c] = [endpoint(1), endpoint(2), endpoint(3)];
  const cursor = { frame_id: 1, seq: 2 };
  h.status.trajectory = { cursor, retained_points: 2, points: [a, b], latest: b };
  await h.poll();

  const readStatus = h.api.status;
  const oldReply = await readStatus();
  let release!: (status: typeof oldReply) => void;
  h.api.status = () => new Promise(resolve => { release = resolve; });
  await h.poll();
  await h.click("Clear trail");
  assert.deepEqual(h.calls.at(-1), ["clearTrail", cursor]);
  assert.deepEqual(h.trajectoryProps.points, []);
  assert.deepEqual(h.trajectoryProps.latest, b);
  await act(async () => { release(oldReply); });
  assert.deepEqual(h.trajectoryProps.points, []);

  const cleared = h.status.trajectory.cursor;
  h.api.status = async after => {
    assert.deepEqual(after, cleared);
    return readStatus(after);
  };
  h.status.trajectory = { cursor: { frame_id: 1, seq: 4 }, retained_points: 1, points: [c], latest: c };
  await h.poll();
  assert.deepEqual(h.trajectoryProps.points, [c]);
  h.api.status = readStatus;
  await h.click("Stop and close motor");
  await h.click("Clear trail");
  await h.poll();
  assert.deepEqual(h.trajectoryProps.points, []);
  assert.equal(h.trajectoryProps.latest, null);
  assert.equal(h.button("Clear trail").props.disabled, true);
  assert.deepEqual(h.calls.map(call => call[0]), ["connect", "start", "clearTrail", "stop", "clearTrail"]);
});

test("a failed clear keeps the path and an external clear is observed even without new samples", async t => {
  const h = await mount(t);
  const p = endpoint(1);
  h.status.trajectory = { cursor: { frame_id: 1, seq: 1 }, retained_points: 1, points: [p], latest: null };
  await h.poll();
  h.api.pullCordClearTrajectory = async () => { throw new Error("unavailable"); };
  await h.click("Clear trail");
  assert.deepEqual(h.trajectoryProps.points, [p]);
  assert.equal(h.button("Clear trail").props.disabled, false);
  assert.ok(h.errors.some(e => e.includes("unavailable")));
  h.status.trajectory = { cursor: { frame_id: 1, seq: 2 }, retained_points: 0, points: [], latest: null };
  await h.poll();
  assert.deepEqual(h.trajectoryProps.points, []);
});

test("the XY view breaks lines at feedback gaps and draws the cord only for a live endpoint", () => {
  const host = (name: string) => (props: any) => React.createElement(name, props, props.children);
  const Descriptions = Object.assign(host("dl"), { Item: host("dd") });
  const View = component("PullCordTrajectory", {
    antd: { Button: host("button"), Card: host("section"), Descriptions, Space: host("div"), Tag: host("span"), Typography: { Text: host("p") } },
    "../i18n": { useI18n: () => ({ lang: "en" }) },
    "echarts-for-react": { __esModule: true, default: host("chart") },
  });
  const points = [endpoint(1), endpoint(2), endpoint(3, 2)];
  const latest = endpoint(4, 2);
  let renderer: any;
  act(() => { renderer = create(React.createElement(View, { points, latest, phase: "running" })); });
  const option = () => renderer.root.findByType("chart").props.option;
  assert.equal(option().xAxis.max, option().yAxis.max);
  assert.deepEqual(option().series[0].data, [[0.1, 0.25], [0.2, 0.25], null, [0.3, 0.25], [0.4, 0.25]]);
  assert.deepEqual(option().series[1].data, [[0, 0], [0.4, 0.25]]);
  act(() => renderer.update(React.createElement(View, { points, latest: null, phase: "idle" })));
  assert.deepEqual(option().series[1].data, []);
  assert.deepEqual(option().series[3].data, []);
  act(() => renderer.unmount());
});
