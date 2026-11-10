import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import test, { type TestContext } from "node:test";
import ts from "typescript";
import type { DemoConnection, DemoView } from "../src/dualDemoApi.ts";

const require = createRequire(import.meta.url);
const React = require("react");
const { act, create } = require("react-test-renderer");

function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>((done) => { resolve = done; });
  return { promise, resolve };
}

function snapshot(overrides: Partial<DemoView> = {}): DemoView {
  const motor = { node_id: 1, online: true, enabled: false, target_rpm: 0,
    actual_rpm: 0, temperature_c: 25, feedback_age_ms: 10, center_rad: 0, target_deg: 0, actual_deg: 0 };
  return { connected: false, running: false, phase: "disconnected", elapsed_s: 0, connection: null,
    motion: { amplitude_deg: 180, period_s: 5, speed_variation: 0.35, meow_kp: 3, meow_kd: 0.15, dm_kp: 3, dm_kd: 0.15, dm_reverse: false },
    meow: { ...motor }, dm: { ...motor }, error: null, stop_error: null, ...overrides };
}

function mount(t: TestContext) {
  const previous = ["navigator", "window", "localStorage"].map((name) => [name, Object.getOwnPropertyDescriptor(globalThis, name)] as const);
  const events: Record<string, (event: any) => void> = {};
  Object.defineProperty(globalThis, "navigator", { configurable: true, value: { userAgent: "Windows" } });
  Object.defineProperty(globalThis, "localStorage", { configurable: true, value: { getItem: () => null, setItem() {} } });
  Object.defineProperty(globalThis, "window", { configurable: true, value: {
    addEventListener: (key: string, listener: any) => { events[key] = listener; },
    removeEventListener: (key: string) => { delete events[key]; },
  } });
  const host = (name: string) => (props: any) => React.createElement(name, props, props.children);
  const antd: any = Object.fromEntries(["Alert", "Button", "Card", "Input", "InputNumber", "Select", "Space", "Switch", "Tag"].map((name) => [name, host(name)]));
  antd.App = { useApp: () => ({ message: { success() {} } }) };
  antd.Typography = { Text: host("Text"), Paragraph: host("Paragraph") };
  let current = snapshot();
  const initialPoll = deferred<DemoView>();
  let polls = 0;
  let starts = 0;
  let stops = 0;
  let failDisconnect = false;
  let startPending: ReturnType<typeof deferred<DemoView>> | null = null;
  const configs: DemoConnection[] = [];
  const busy: boolean[] = [];
  const mocks: Record<string, unknown> = {
    antd, "echarts-for-react": { __esModule: true, default: host("Chart") },
    "../api": { errMsg: String },
    "../i18n": { useI18n: () => ({ lang: "en" }) },
    "./DualMotorDemoPanel.css": {},
    "../dualDemoApi": { dualDemoApi: {
      get: async () => ++polls === 1 ? initialPoll.promise : current,
      connect: async (config: DemoConnection) => {
        configs.push(config); current = snapshot({ connected: true, phase: "ready", connection: config }); return current;
      },
      start: async () => {
        starts += 1;
        if (startPending) return startPending.promise;
        current = { ...current, running: true, phase: "running" }; return current;
      },
      stop: async () => { stops += 1; current = { ...current, running: false, phase: "ready" }; return current; },
      disconnect: async () => {
        if (failDisconnect) throw new Error("Disable not confirmed");
        current = snapshot();
      },
    } },
  };
  const url = new URL("../src/components/DualMotorDemoPanel.tsx", import.meta.url);
  const { outputText } = ts.transpileModule(readFileSync(url, "utf8"), {
    compilerOptions: { module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX, target: ts.ScriptTarget.ES2021 }, fileName: url.pathname,
  });
  const module = { exports: {} as any };
  new Function("require", "module", "exports", outputText)((name: string) => Object.hasOwn(mocks, name) ? mocks[name] : require(name), module, module.exports);
  let renderer: any;
  act(() => { renderer = create(React.createElement(module.exports.DualMotorDemoPanel, { onBusyChange: (b: boolean) => busy.push(b) })); });
  t.after(() => {
    act(() => renderer.unmount());
    for (const [name, descriptor] of previous) {
      if (descriptor) Object.defineProperty(globalThis, name, descriptor);
      else Reflect.deleteProperty(globalThis, name);
    }
  });
  const text = (value: any): string => Array.isArray(value) ? value.map(text).join("") : String(value ?? "");
  const button = (label: string) => renderer.root.findAllByType("Button").find((n: any) => text(n.props.children).startsWith(label));
  return {
    configs, busy, button, initialPoll, events,
    get starts() { return starts; }, get stops() { return stops; },
    get alerts() { return renderer.root.findAllByType("Alert"); },
    get inputs() { return renderer.root.findAllByType("Input"); },
    input(label: string, value: string) { act(() => renderer.root.findAllByType("Input").find((n: any) => n.props["aria-label"] === label).props.onChange({ target: { value } })); },
    rate(label: string, value: number) { act(() => renderer.root.findAllByType("Select").find((n: any) => n.props["aria-label"] === label).props.onChange(value)); },
    async click(label: string) { const b = button(label); assert.ok(b); assert.equal(b.props.disabled, false); await act(async () => { await b.props.onClick(); }); },
    delayStart() { startPending = deferred<DemoView>(); return startPending; },
    failDisconnect() { failDisconnect = true; },
  };
}

test("dual demo preserves independent channel and bitrate selections", async (t) => {
  const h = mount(t);
  assert.deepEqual(h.inputs.map((n: any) => n.props.value), ["gs_usb0", "gs_usb1"]);
  h.input("hexmeow CAN", "gs_usb:2"); h.input("DAMIAO CAN", "gs_usb3");
  h.rate("hexmeow bitrate", 4_000_000); h.rate("DAMIAO bitrate", 500_000);
  await h.click("Connect & discover");
  assert.deepEqual(h.configs[0], { meow_iface: "gs_usb:2", meow_bitrate: 4_000_000,
    dm_iface: "gs_usb3", dm_bitrate: 500_000, host_node_id: 10, meow_node_id: null, dm_motor_id: null });
  assert.ok(h.inputs.every((n: any) => n.props.disabled));
  assert.deepEqual(h.busy, [true, false]);
});

test("late telemetry cannot undo a successful connection or Start", async (t) => {
  const h = mount(t);
  await h.click("Connect & discover"); await h.click("Start demo");
  await act(async () => { h.initialPoll.resolve(snapshot()); });
  assert.equal(h.button("Start demo").props.disabled, true);
  assert.equal(h.button("Stop both motors").props.disabled, false);
  await act(async () => { h.events.keydown({ key: "Escape" }); });
  assert.equal(h.stops, 1);
  assert.equal(h.button("Start demo").props.disabled, false);
});

test("Stop fences a delayed Start and duplicate clicks submit only once", async (t) => {
  const h = mount(t);
  await h.click("Connect & discover");
  const pending = h.delayStart();
  const start = h.button("Start demo").props.onClick;
  let starting!: Promise<void>;
  act(() => { starting = start(); void start(); });
  assert.equal(h.starts, 1);
  const stopping = h.button("Stop both motors").props.onClick();
  await act(async () => { pending.resolve(snapshot({ connected: true, running: true, phase: "running" })); await starting; await stopping; });
  assert.equal(h.stops, 2, "Stop must be repeated after an in-flight Start settles");
  assert.equal(h.button("Start demo").props.disabled, false);
});

test("failed disconnect preserves access to Stop and shows the failure", async (t) => {
  const h = mount(t);
  await h.click("Connect & discover"); h.failDisconnect();
  await h.click("Stop & disconnect");
  assert.equal(h.button("Stop both motors").props.disabled, false);
  assert.match(h.alerts[0].props.message, /Disable not confirmed/);
});
