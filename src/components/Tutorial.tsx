import { useState } from "react";
import { Carousel, Modal, Typography, theme } from "antd";
import { useI18n } from "../i18n";

// A swipe-through getting-started guide shown from the tool picker.
//
// To customize: drop screenshots or short screen-recordings into
// `public/tutorial/` and point each slide's `media` at them, e.g.
//   media: { type: "image", src: "/tutorial/01-connect.png" }
//   media: { type: "video", src: "/tutorial/02-drive.mp4" }
// Files in `public/` are served from the site root, so the leading "/" is the
// project's `public/` folder. Slides with no `media` just render their text.
// Edit, reorder, or add to SLIDES freely — both languages live inline.
interface Slide {
  media?: { type: "image" | "video"; src: string };
  title: { en: string; zh: string };
  body: { en: string; zh: string };
}

const HOME_SLIDES: Slide[] = [
  {
    media: { type: "image", src: "/tutorial/01-connect.png" },
    title: { en: "1 · Connect", zh: "1 · 连接" },
    body: {
      en: "Pick your CAN interface in the top bar, then press Connect. Linux defaults to can0; macOS / Windows default to gs_usb0 (a USB candleLight adapter).",
      zh: "在顶部选择 CAN 接口，然后点击「连接」。Linux 默认 can0；macOS / Windows 默认 gs_usb0（USB candleLight 适配器）。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/02-select.png" },
    title: { en: "2 · Pick a motor", zh: "2 · 选择电机" },
    body: {
      en: "Discovered motors appear in the left sidebar. Click one to open its live panel, charts, and controls.",
      zh: "发现的电机会出现在左侧栏。点击任意一个即可打开它的实时面板、图表和控制区。",
    },
  },
  {
    media: { type: "video", src: "/tutorial/03-drive.mp4" },
    title: { en: "3 · Drive & chart", zh: "3 · 控制与绘图" },
    body: {
      en: "Set a mode and target to drive the motor. Switch the display to Chart to watch position, velocity and torque. Use the Refresh High/Low toggle if the chart feels heavy.",
      zh: "设置模式和目标值来驱动电机。把显示切到「图表」即可观察位置、速度和力矩。如果图表卡顿，用「刷新率 高/低」开关调节。",
    },
  },
  {
    title: { en: "4 · Record full-rate data", zh: "4 · 记录全速率数据" },
    body: {
      en: "The on-screen chart is downsampled for smoothness. For the full ~1000 Hz stream, flip the Record CSV switch — it logs every frame to a file untouched by the UI.",
      zh: "屏幕上的图表为了流畅做了降采样。需要完整的 ~1000 Hz 数据流时，打开「记录 CSV」开关——它会把每一帧原样写入文件，不受界面影响。",
    },
  },
];

// Placeholder slides for a per-app tutorial that hasn't been written yet.
// Each slide's media already points at `public/tutorial/<tool>/0N.png`, so
// dropping a screenshot (or renaming to .mp4 and adjusting the type) there is
// all it takes to fill a step in — then replace the step's body text. Add or
// remove steps by changing `count`.
function placeholderSlides(tool: string, count = 3): Slide[] {
  return Array.from({ length: count }, (_, i) => {
    const n = i + 1;
    return {
      media: { type: "image", src: `/tutorial/${tool}/0${n}.png` },
      title: { en: `Step ${n}`, zh: `步骤 ${n}` },
      body: {
        en: "(Describe this step, then drop a screenshot into public/tutorial/ to replace this placeholder.)",
        zh: "（在此描述该步骤，并把截图放到 public/tutorial/ 目录以替换此占位。）",
      },
    };
  });
}

const SETTINGS_SLIDES: Slide[] = [
  {
    title: { en: "1 · Connect and select", zh: "1 · 连接并选择设备" },
    body: {
      en: "Open Device Settings, connect the CAN bus, and select a device from the sidebar. All discovered nodes are shown, but write operations are available only for an exact known Vendor-ID + Product-code identity.",
      zh: "打开「设备设置」，连接 CAN 总线，再从侧栏选择设备。这里会显示所有发现的节点，但只有 Vendor-ID + Product-code 精确匹配的已知设备才开放写操作。",
    },
  },
  {
    title: { en: "2 · Configure communication", zh: "2 · 配置通信参数" },
    body: {
      en: "The current Node-ID comes from the selected device and cannot be typed manually. Nominal timing is fixed at 1 Mbit/s, SP 0.80. CAN-FD devices offer 1/2/4 Mbit/s at SP 0.80 or 5 Mbit/s at SP 0.75, plus TPDO BRS. Classic-CAN devices hide the FD fields.",
      zh: "当前 Node-ID 来自所选设备，不能手动填写。仲裁段固定为 1 Mbit/s、SP 0.80。CAN-FD 设备可选 1/2/4 Mbit/s（SP 0.80）或 5 Mbit/s（SP 0.75），并可设置 TPDO BRS；Classic CAN 设备会隐藏 FD 字段。",
    },
  },
  {
    title: { en: "3 · Apply and follow the result", zh: "3 · 应用并按结果处理" },
    body: {
      en: "Communication settings can be applied only while the device is Pre-Operational or Stopped. A 0x2001 change requires a later power cycle. A pending 0x2100 change must be allowed to persist; do not reset the device automatically. A BRS-only change may take effect immediately.",
      zh: "只有设备处于 Pre-Operational 或 Stopped 时才能应用通信设置。修改 0x2001 后需要稍后重新上电；0x2100 显示等待持久化时，应等待完成，不要自动复位设备；仅修改 BRS 时可以立即生效。",
    },
  },
  {
    title: { en: "4 · Motor position preset", zh: "4 · 电机位置预设" },
    body: {
      en: "Known motors have a separate Position Preset card. Position is read once after each online edge and can always be refreshed manually. Saving a preset is its own transaction and performs one confirmation read; a failed read is never retried in a loop.",
      zh: "已知电机另有独立的「位置预设」卡片。每次设备上线后只自动读取一次位置，也可以随时手动刷新。保存预设是独立事务，之后只确认读取一次；读取失败不会进入自动重试循环。",
    },
  },
];

const CONTROL_SLIDES: Slide[] = [
  {
    media: { type: "image", src: "/tutorial/control/01.png" },
    title: { en: "1 · Open Motor Control", zh: "1 · 打开 Motor Control" },
    body: {
      en: "Once wiring is done, open the Motor Control app from the tool picker.",
      zh: "接好线后，在工具选择界面点击 Motor Control 方框进入。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/control/02.png" },
    title: { en: "2 · Connect", zh: "2 · 连接" },
    body: {
      en: "In the Motor Control view, press Connect.",
      zh: "进入 Motor Control 界面后，点击 Connect 连接总线。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/control/03.png" },
    title: { en: "3 · Motors appear", zh: "3 · 显示电机" },
    body: {
      en: "On a successful connection, the detected motors and their info show in the left list.",
      zh: "连接成功后，左侧列表会显示电机及其信息。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/control/04.png" },
    title: { en: "4 · Select & initialize", zh: "4 · 选择并初始化" },
    body: {
      en: "Select the motor you want to drive, then press the Initialize button.",
      zh: "选择你要控制的电机，然后点击初始化按钮。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/control/05.png" },
    title: { en: "5 · Choose a mode & send", zh: "5 · 选择模式并发送" },
    body: {
      en: "Pick a control mode — here velocity mode (0.5 rev/s, 30% max torque). In order: choose the mode, enable the motor, limit peak torque to 30%, set speed to 0.5 rev/s, then send. The motor starts turning.",
      zh: "选择控制方式，这里以速度模式（0.5 rev/s、30% 峰值力矩）为例。按顺序操作：选择控制模式 → 使能电机 → 将峰值力矩限制到 30% → 设置速度 0.5 rev/s → 发送速度。成功后即可看到电机转动。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/control/06.png" },
    title: { en: "6 · Live readout", zh: "6 · 实时数据" },
    body: {
      en: "While running, the Display panel streams live motor data. Press Chart to switch to a graph view.",
      zh: "电机运行时，Display 窗口会实时返回运行数据。点击 Chart 按钮可切换为图表模式。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/control/07.png" },
    title: { en: "7 · Chart view", zh: "7 · 图表模式" },
    body: {
      en: "The chart plots position, velocity and torque over time.",
      zh: "图表模式下可实时观察位置、速度和力矩曲线。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/control/08.png" },
    title: { en: "8 · Record CSV", zh: "8 · 记录 CSV" },
    body: {
      en: "Press Record CSV to save the run. The highlighted field (2) shows the path of the saved data file.",
      zh: "按下 Record CSV 按钮即可保存运行数据。图中标记的 2 号方框就是数据文件的存储路径。",
    },
  },
];

const CALIBRATION_SLIDES: Slide[] = [
  {
    title: { en: "1 · Prepare the unloaded motor", zh: "1 · 准备空载电机" },
    body: {
      en: "Place the motor case stably on a flat surface. Remove every shaft load, clear the rotation envelope, and keep physical power removal ready.",
      zh: "将电机外壳平稳放在水平桌面上，卸除输出轴负载并清空旋转范围，同时准备好物理断电。",
    },
  },
  {
    title: { en: "2 · Run bounded static passes", zh: "2 · 执行有限静摩擦测试" },
    body: {
      en: "The Rust worker increases the SDO torque target one bounded step at a time in each direction. Effective motion requires both displacement and velocity thresholds; detection immediately commands zero and Disabled.",
      zh: "Rust 后端沿正反方向逐步增加 SDO 力矩目标。位移和速度必须同时超过阈值才算有效运动；检测后立即归零并失能。",
    },
  },
  {
    title: { en: "3 · Measure kinetic drag at 1 rad/s", zh: "3 · 在 1 rad/s 测量动摩擦" },
    body: {
      en: "The app runs unloaded Profile Velocity at +1 and -1 rad/s, waits for settled speed, then robustly averages raw actual torque and motor temperature. No friction or torque-factor compensation is active.",
      zh: "APP 以 +1、-1 rad/s 执行空载 Profile Velocity，速度稳定后对原始实际力矩和电机温度做稳健平均。过程中不启用摩擦或力矩系数补偿。",
    },
  },
  {
    title: { en: "4 · Review and export", zh: "4 · 验收并导出" },
    body: {
      en: "Review the four raw command-domain Nm values and copy the JSON audit record. This measurement app does not write 0x4001; torque factor, CRC and HMAC are issued together in the later provisioning flow.",
      zh: "检查四个修正前原始命令域 Nm 值并复制 JSON 审计记录。本测量 APP 不写 0x4001；力矩系数、CRC 和 HMAC 将在后续写入流程中统一签发。",
    },
  },
];

const TORQUE_CALIBRATION_SLIDES: Slide[] = [
  {
    title: { en: "1 · Mount and clear the fixture", zh: "1 · 安装并清空工装" },
    body: {
      en: "Lay the motor on its side. The light lever and 1 kg mass at 0.27 m must pass both ±65° endpoints freely. Keep immediate physical power removal ready.",
      zh: "电机侧放；轻质杆与中心距 0.27 m 的 1 kg 负重必须能自由通过两侧 ±65° 端点；保持可立即物理断电。",
    },
  },
  {
    title: { en: "2 · Capture the low point", zh: "2 · 采集最低点" },
    body: {
      en: "With the motor Disabled, the backend waits for the lever to hang still and records that encoder value as local zero. No position object is rewritten.",
      zh: "电机保持 Disabled，后端等待杆自然静止并把该编码器位置作为本地零点，不改写任何位置对象。",
    },
  },
  {
    title: { en: "3 · Pair host-controlled MIT traversals", zh: "3 · 配对主机 MIT 原始数据" },
    body: {
      en: "Rust generates a speed/acceleration-limited trajectory and streams a host PD plus nominal-gravity Tff over compressed MIT at 1000 Hz. At the same signed angle, forward and reverse raw 0x4577 values are averaged before the through-origin fit. Only ±60° is fitted.",
      zh: "Rust 生成限速、限加速度轨迹，并以 1000 Hz compressed MIT 发送主机 PD 与名义重力 Tff；在相同有符号角度对正向与反向的原始 0x4577 取平均后进行过原点拟合；只拟合 ±60°。",
    },
  },
  {
    title: { en: "4 · Acceptance and silence", zh: "4 · 验收与静默" },
    body: {
      en: "Acceptance streams 100 Hz compressed-MIT gravity torque with kp=kd=0 and no friction term. Ending either operation zeros and Disables the motor, restores settings, then stops PDO and host heartbeat traffic.",
      zh: "验收以 100 Hz compressed-MIT 发送纯重力矩，kp=kd=0 且不含摩擦项。结束任一操作都会归零、失能、恢复参数，并停止 PDO 与主机心跳。",
    },
  },
];

const DFU_SLIDES: Slide[] = [
  {
    title: { en: "1 · Choose transport", zh: "1 · 选择传输方式" },
    body: {
      en: "For the current HPM product, enter USB Bootloader mode and select USB. The STM32 CAN page currently provides read-only Classic-CAN discovery; all known product profiles remain write-locked.",
      zh: "当前 HPM 产品请进入 USB Bootloader 并选择 USB。STM32 CAN 页面目前只提供 Classic CAN 只读发现；所有已知产品 profile 仍保持写锁定。",
    },
  },
  {
    title: { en: "2 · Validate firmware", zh: "2 · 校验固件" },
    body: {
      en: "USB accepts a local artifact after strict validation. STM32 .meowpkg selection appears only after an exact product profile is qualified; manual selection never bypasses validation.",
      zh: "USB 可在严格校验后选择本地制品。STM32 只有在精确产品 profile 完成资格确认后才开放 .meowpkg；手动选择不会绕过校验。",
    },
  },
  {
    title: { en: "3 · Upgrade and check", zh: "3 · 升级并检查" },
    body: {
      en: "Keep power connected during USB update. USB cannot prove application health, so check the HPM device's actual function afterward. This build does not start an STM32 CAN write.",
      zh: "USB 升级期间请保持供电。USB 无法证明 APP 健康，因此结束后还需检查 HPM 设备的实际功能。此构建不会启动 STM32 CAN 写入。",
    },
  },
];

const SMARTKNOB_SLIDES: Slide[] = [
  {
    media: { type: "image", src: "/tutorial/smartknob/01.png" },
    title: { en: "1 · Open SmartKnob", zh: "1 · 打开 SmartKnob" },
    body: {
      en: "Open the SmartKnob app to experience the motor's haptic force-feedback knob.",
      zh: "选择 SmartKnob 模式，体验 hex 电机的智能力反馈旋钮功能。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/smartknob/02.png" },
    title: { en: "2 · Connect & pick a mode", zh: "2 · 连接并选择模式" },
    body: {
      en: "After connecting, select the motor to operate, then pick a feel mode on the right.",
      zh: "连接电机后，选择你想操作的电机，再在右边选择想要的模式。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/smartknob/03.png" },
    title: { en: "3 · Custom mode", zh: "3 · 自定义模式" },
    body: {
      en: "The first option is Custom — tune the haptic feel parameters below the dial.",
      zh: "第一个是自定义模式，你可以在仪表盘下方自定义手感参数。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/smartknob/04.png" },
    title: { en: "4 · Adjust strength", zh: "4 · 调整强度" },
    body: {
      en: "You can also adjust the feel strength of each mode below the mode selector.",
      zh: "你也可以在模式下方调整不同模式的手感强度。",
    },
  },
];

const LIFT_SLIDES: Slide[] = [
  {
    title: { en: "1 · Connect and attach", zh: "1 · 连接并绑定" },
    body: {
      en: "Connect can0 (or the selected adapter), then attach the Lift worker to Node-ID 20 by default. Attach only reads identity, nameplate, configuration and telemetry; it never makes the node Operational.",
      zh: "连接 can0（或所选适配器），再把 Lift worker 绑定到默认 Node-ID 20。绑定只读取身份、铭牌、配置与遥测，不会让节点进入 Operational。",
    },
  },
  {
    title: { en: "2 · Read every safety gate", zh: "2 · 检查全部安全门控" },
    body: {
      en: "Motion stays locked until heartbeat and both TPDOs are fresh, the encoder/INA sample is independently healthy, NMT is Operational, CONFIG_VALID is set, no fault is latched, and Homing has completed where required. Never bypass a red or amber blocker.",
      zh: "只有 heartbeat 与两路 TPDO 都新鲜、编码器/INA 联合样本也独立确认健康、NMT 为 Operational、CONFIG_VALID 有效、无锁存故障，并在需要时完成 Homing，运动才会解锁。不要绕过任何红色或黄色阻塞提示。",
    },
  },
  {
    title: { en: "3 · Velocity is hold-to-jog", zh: "3 · 速度只允许按住点动" },
    body: {
      en: "Press and hold Up or Down. Rust owns RPDO timing, while the WebView renews a short operator lease; release, blur or stale telemetry stops it. DISABLE OUTPUT is the always-available directed NMT Stop path.",
      zh: "按住“上升”或“下降”才会点动。RPDO 时序由 Rust 管理，WebView 续租短时人机租约；松手、失焦或遥测过期都会停止。DISABLE OUTPUT 是始终可用的定向 NMT Stop 路径。",
    },
  },
  {
    title: { en: "4 · Homing and Position are autonomous", zh: "4 · Homing 与 Position 是自主运动" },
    body: {
      en: "Use a current-limited supply and keep physical power removal ready. A confirmed Detach/Disconnect cancels autonomous motion, but process crash or host power loss cannot guarantee cancellation. Commission free motor and Homing before Position.",
      zh: "使用限流电源并随时准备物理断电。经确认的 Detach/Disconnect 会取消自主运动，但进程崩溃或主机掉电无法保证。应先完成自由电机与 Homing 调试，最后才测试 Position。",
    },
  },
];

const CANALYZER_SLIDES: Slide[] = [
  {
    media: { type: "image", src: "/tutorial/canalyzer/01.png" },
    title: { en: "1 · Open CAN Analyzer", zh: "1 · 打开 CAN 分析仪" },
    body: {
      en: "Open the CAN Analyzer app to inspect the messages on the connected CAN bus.",
      zh: "选择 CAN Analyzer 模式，查看所连接 CAN 总线上的消息。",
    },
  },
  {
    media: { type: "image", src: "/tutorial/canalyzer/02.png" },
    title: { en: "2 · Connect & operate", zh: "2 · 连接与操作" },
    body: {
      en: "Press Connect first, then use the various buttons to filter and interact with the traffic.",
      zh: "同样先点击连接按钮，之后可以点击不同的按钮进行操作。",
    },
  },
];

const AUTHENTICITY_SLIDES: Slide[] = [
  {
    title: { en: "1 · Connect and wait for heartbeats", zh: "1 · 连接并等待心跳" },
    body: {
      en: "The app identifies heartbeat nodes through their complete 0x1018 record. Unknown products remain visible to discovery but never receive proprietary proof reads.",
      zh: "APP 通过完整 0x1018 识别心跳节点。未知产品仍可被发现，但不会收到任何专用证明读取。",
    },
  },
  {
    title: { en: "2 · Read source and registration separately", zh: "2 · 分开查看来源与注册状态" },
    body: {
      en: "Signed hexmeow devices are verified offline with a product-pinned P-256 key. Meow Motor uses its complete 0x4001 v1 envelope and the online issuance ledger. Network failure is never displayed as a counterfeit verdict.",
      zh: "自有 hexmeow 设备使用产品固定的 P-256 公钥离线验签；Meow Motor 使用完整 0x4001 v1 envelope 与在线签发账本。网络故障不会被显示成盗版结论。",
    },
  },
  {
    title: { en: "3 · First-register the selected set", zh: "3 · 首次注册所选集合" },
    body: {
      en: "Review every identity, then register all selected new devices atomically. Registration is normally irreversible and reveals no customer, order or registration-time information.",
      zh: "核对每个身份后，将所选新设备作为一个原子事务注册。注册通常不可撤销，公开接口不会透露客户、订单或注册时间。",
    },
  },
];

// Slide sets keyed by tool id (matching App's Tool union, plus "home" for the
// landing-page guide). Tools without a written guide yet fall back to
// placeholder steps.
export const TUTORIALS: Record<string, Slide[]> = {
  home: HOME_SLIDES,
  control: CONTROL_SLIDES,
  settings: SETTINGS_SLIDES,
  hopea3: placeholderSlides("hopea3"),
  lift: LIFT_SLIDES,
  smartknob: SMARTKNOB_SLIDES,
  zenoh: placeholderSlides("zenoh"),
  arm: placeholderSlides("arm"),
  config: placeholderSlides("config"),
  canalyzer: CANALYZER_SLIDES,
  dfu: DFU_SLIDES,
  authenticity: AUTHENTICITY_SLIDES,
  calibration: CALIBRATION_SLIDES,
  torqueCalibration: TORQUE_CALIBRATION_SLIDES,
  calibrationUpdate: placeholderSlides("calibrationUpdate"),
  damiao: [
    {
      title: { en: "1 · Discover and select a motor", zh: "1 · 自动发现并选择电机" },
      body: {
        en: "Hard prerequisite: this 1 Mbps bus must contain DAMIAO motors only; do not mix CANopen or other motor families on the same bus. IDs 0x0–0xF are scanned automatically with status requests and motion-safe Disable probes; select a discovered motor in the left list. Use manual configuration only for an exceptional ID.",
        zh: "硬性前置条件：这条 1 Mbps 总线只能连接达妙电机，严禁与 CANopen 或其他类型电机混接。系统会通过状态查询和安全失能探测自动扫描 ID 0x0–0xF；在左侧列表选择电机即可。仅在使用特殊 ID 时才需要手动配置。",
      },
    },
    {
      title: { en: "2 · Match mapping and attach", zh: "2 · 匹配映射并挂载" },
      body: {
        en: "For each motor, verify its feedback Master ID, control mode, and exact PMAX/VMAX/TMAX values from DAMIAO Assistant before attaching. Several motors may remain attached on the same CAN bus, each with independent feedback and target streaming.",
        zh: "逐台确认反馈 Master ID、控制模式，以及达妙调试助手中完全一致的 PMAX/VMAX/TMAX 后再挂载。同一 CAN 总线可同时保留多台已挂载电机，各自维护反馈与目标流。",
      },
    },
    {
      title: { en: "3 · Verify feedback and stop safely", zh: "3 · 确认反馈并安全停止" },
      body: {
        en: "Confirm fresh telemetry before enabling and start with conservative targets. Use Disable All for a bus-wide stop. Detach, Disconnect, tool switching, and window close safely disable every attached motor before releasing the adapter.",
        zh: "使能前确认遥测持续刷新，并从保守目标开始。需要整条总线停止时使用“全部失能”。卸载、断开连接、切换工具与关闭窗口都会先安全失能所有已挂载电机，再释放适配器。",
      },
    },
  ],
  unitreeServo: [
    {
      title: { en: "1 · Open USB serial", zh: "1 · 打开 USB 串口" },
      body: {
        en: "Choose a USB single-wire TTL adapter (Linux defaults to /dev/ttyACM0) and an ID from 0 to 14. Communication is fixed at 6 Mbps, 8N1. Serial open and Servo online are separate indicators. An unpowered servo cannot reply; opening the port does not enable FOC.",
        zh: "选择 USB 单总线 TTL 适配器（Linux 默认 /dev/ttyACM0）和 0–14 的 ID，通信固定为 6 Mbps、8N1。「串口已打开」与「舵机在线」分别显示。未上电的舵机不会应答；打开串口不会开启 FOC。",
      },
    },
    {
      title: { en: "2 · Scan and send targets", zh: "2 · 扫描并发送目标" },
      body: {
        en: "Scan IDs 0–14 and select a discovered servo. Select Hybrid FOC (1) and use Enable FOC to confirm the mode with zero torque and gains, then send output-side position, velocity, torque, Kp and Kd. Send also confirms FOC automatically when needed. Position, velocity and torque presets all use hybrid FOC. Delayed first-frame mode feedback is handled automatically. Single targets are followed by lock after about 200 ms.",
        zh: "扫描 0–14 并选择舵机。工作模式选择「混合 FOC（1）」，可先点击「使能 FOC」以零力矩、零增益确认，再发送输出端位置、速度、力矩、Kp 和 Kd。直接发送目标时也会自动确认 FOC。位置、速度、力矩预设均使用混合 FOC，首帧反馈模式滞后由后台处理。单次目标约 200 ms 后自动锁定。",
      },
    },
    {
      title: { en: "3 · Monitor and stop", zh: "3 · 监控与停止" },
      body: {
        en: "Watch numeric feedback or the position, velocity and torque charts. Stop / clear timeout, or Ctrl+Enter, stops streaming and requests lock mode. The backend stops motion on lost client heartbeat, missing replies or fault feedback. Stop before changing the selected servo or communication settings.",
        zh: "查看数值反馈或位置、速度、力矩曲线。点击「停止 / 清除超时」或按 Ctrl+Enter 可停止发送目标并请求锁定。后台会在界面心跳丢失、连续应答失败或收到故障反馈后停止控制。修改通信对象和通信设置前先停机。",
      },
    },
    {
      title: { en: "4 · Session profiles and servo configuration", zh: "4 · 会话配置与舵机配置" },
      body: {
        en: "Communication & limits and JSON profiles retain host settings on this computer. To change the physical servo ID, connect one powered servo, stop motion, open Servo settings, enter a different ID (0–14) and click Change ID and save to servo. The app saves to Flash, verifies the new ID and restores APP without starting motion. Failed or interrupted configuration shows whether Flash saving was acknowledged; Restore APP can recover communication at the old or new ID.",
        zh: "「通信与限制」和 JSON 配置保存电脑上的会话设置。修改舵机本体 ID 时，总线上只接一台已上电舵机，先停止运动，再到「舵机设置」输入不同的新 ID（0–14），点击「修改 ID 并保存到舵机」。程序会保存 Flash、回读新 ID 并恢复 APP，不会启动运动。异常时会明确显示保存是否已确认，可在旧或新 ID 上尝试「恢复 APP」。",
      },
    },
    {
      title: { en: "5 · Test several servos on one wire", zh: "5 · 一条总线上的一拖多测试" },
      body: {
        en: "Give each servo a distinct ID, connect them to the same single-wire bus, scan, and open Multi-servo test. Select 2–15 IDs and a duration. Start with the lock-mode communication test; inspect per-ID loss, rate and response time. For motion, edit each ID's target and start the FOC test explicitly. Stop, completion, a member fault or lost client heartbeat locks all members. Export the JSON report to retain results.",
        zh: "先分别设置不同的 ID，再将多台舵机接到同一单总线并扫描。打开「一拖多测试」，选择 2–15 个 ID 和时长，先运行锁定模式通信测试，查看各 ID 的丢包率、实际频率和延迟。需要运动时，分别编辑各 ID 目标，再明确启动 FOC 测试。停止、到时、成员故障或界面心跳丢失时逐台锁定；可导出 JSON 测试报告。",
      },
    },
  ],
  standardMit: [
    {
      title: { en: "1 · Connect a standard MIT motor", zh: "1 · 连接标准 MIT 电机" },
      body: {
        en: "Use the independent Standard MIT Motor Control app for 11-bit classic CAN at 1 Mbps. Open can0 / socketcan:can0 or gs_usb0. The Basic MIT profile supports drives with the EL05-compatible p16/v12/Kp12/Kd12/torque12 format and ranges ±12.57 rad, ±50 rad/s and ±6 Nm. Select EL05 manual extensions only for drives that implement manual 260922.",
        zh: "本应用使用 1 Mbps、11 位标准帧 Classic CAN。接口支持 can0 / socketcan:can0 或 gs_usb0。「基础 MIT」适用于 EL05 兼容的 p16/v12/Kp12/Kd12/torque12 格式，量程为 ±12.57 rad、±50 rad/s 和 ±6 Nm。电机明确支持 260922 手册时再选择「EL05 手册扩展」。",
      },
    },
    {
      title: { en: "2 · Discover, select and control", zh: "2 · 扫描、选择与控制" },
      body: {
        en: "Scan sends Stop to IDs 0–255. Select a discovered motor or enter its decimal/hex ID. The reply CAN ID is learned automatically. Enable, edit the five MIT targets, then Send target. Velocity needs Kd > 0; fixed position with Kp > 0 resists continuous rotation. The host keeps commands streaming. Stop ends the stream and disables the motor.",
        zh: "扫描会向 ID 0–255 发送停止帧。选择发现的电机或手动输入十进制/十六进制 ID，回复 CAN ID 自动识别。使能后编辑 MIT 五参数，再点击「发送目标」。速度目标需 Kd > 0；固定位置且 Kp > 0 会阻碍连续旋转。主机会持续发送目标；停止会结束连续发送并失能。",
      },
    },
    {
      title: { en: "3 · Monitor and test", zh: "3 · 监视与测试" },
      body: {
        en: "Use numeric feedback or position, speed and torque charts. Packed velocity has 12-bit resolution, about 0.02442 rad/s. Pendulum tests include passive decay, bidirectional friction calibration and performance measurements. Enter geometry and capture the downward reference before gravity-based tests or zero-gravity hold. MIT history is stored separately from native EL05 runs; CSV, JSON and reports can be exported.",
        zh: "数值与曲线显示位置、速度、转矩和温度，速度反馈为 12 位，分辨率约 0.02442 rad/s。摆锤测试包括被动衰减、双向静摩擦标定及性能测试。重力相关测试和零重力保持需填写几何参数并测定下垂参考。MIT 历史与原生 EL05 记录分开保存，可导出 CSV、JSON 和报告。",
      },
    },
    {
      title: { en: "4 · Zero and optional settings", zh: "4 · 置零与扩展设置" },
      body: {
        en: "If position saturates at ±12.57 rad, stop and reset the mechanical zero, then capture a new reference. The extended profile adds mode 1 position CSP, mode 2 velocity, fault reset and ID changes. Parameter read/write, saving and reporting require firmware 1.0.5.0.4 or a compatible implementation. Writes require a stopped motor and verify readback; save parameters explicitly for persistence. Leaving the app stops its motor before releasing CAN.",
        zh: "位置反馈停在 ±12.57 rad 端点时，先停机并重置机械零点，再重新记录下垂参考。扩展配置提供模式 1 位置 CSP、模式 2 速度、清故障与修改 ID。参数读写、保存和主动上报需固件 1.0.5.0.4 或兼容实现。参数写入需先停机并读回校验，掉电保存需手动点击「保存参数」。离开应用时会先停机再释放 CAN。",
      },
    },
  ],
  el05: [
    {
      title: { en: "1 · Connect and select", zh: "1 · 连接并选择" },
      body: {
        en: "Connect an EL05 using RobStride's native protocol at 1 Mbps. Linux defaults to can0; USB adapters use gs_usb0. Host ID defaults to 0xFD. Scan, then select a discovered motor, or enter its ID manually. Selecting another motor stops the previous one.",
        zh: "使用 RobStride 私有协议，以 1 Mbps 连接 EL05。Linux 默认 can0，USB 适配器使用 gs_usb0，主机 ID 默认为 0xFD。扫描后选择发现的电机，或手动输入 ID。切换电机时会先停止上一台。",
      },
    },
    {
      title: { en: "2 · Pendulum performance tests", zh: "2 · 摆锤性能测试" },
      body: {
        en: "The default Pendulum tests tab provides eleven presets, live traces and automatic metrics. Static friction calibration waits for rest, slowly ramps torque, then returns after detecting a small displacement; its maximum angle is configurable. Enter load geometry and capture the downward reference for gravity-based analysis. Start prepares the mode and drive automatically. Compare completed runs, export CSV/JSON/reports, or import earlier JSON. Manual control and settings have separate tabs.",
        zh: "默认「摆锤性能测试」提供十一种工况、实时曲线与自动指标。「静摩擦标定」先确认静止、缓慢加转矩，检测到小角度起动后回落，最大偏转角可设。输入负载几何并测定下垂参考后，可进行重力修正。「一键开始测试」自动准备模式并使能。完成后可比较历史，导出 CSV、JSON 或报告，也可导入 JSON 复查。手动控制与设置位于独立分页。",
      },
    },
    {
      title: { en: "3 · Zero-gravity hold", zh: "3 · 零重力保持" },
      body: {
        en: "Open Zero-gravity hold, enter the load and arm geometry, then capture the vertically downward reference with the motor stopped and stationary. Start enables continuous gravity support with adjustable scale, damping, torque limit, startup ramp and speed/temperature limits. Move the pendulum by hand; damping and friction help it settle after release. It does not lock an angle, and model error may cause drift. Stop removes support. Reconnect, reselect, zero/ID changes or lost feedback require a new reference.",
        zh: "进入「零重力保持」，填写负载与摆杆几何，在电机停机且摆锤静止垂直下垂时测定参考。启动后持续补偿重力，可预设补偿比例、阻尼、转矩上限、缓启时间及速度/温度限制。手动拖动摆锤，松手后靠阻尼与摩擦逐渐静止；此模式不锁定角度，模型误差可能造成缓慢漂移。停止会撤去支撑。重连、重选电机、修改零位/ID 或反馈中断后需重新测定参考。",
      },
    },
    {
      title: { en: "4 · Manual control modes", zh: "4 · 手动控制模式" },
      body: {
        en: "Motion/MIT combines position, velocity, torque and Kp/Kd. PP plans a position trajectory with speed and acceleration limits; CSP uses position and speed limits. Velocity and Current control their respective targets. Changing mode stops the motor; enable it again, then send the target.",
        zh: "运控/MIT 同时设置位置、速度、转矩及 Kp/Kd；PP 使用速度和加速度规划位置轨迹，CSP 使用目标位置与速度限制；速度和电流模式分别控制对应目标。切换模式会停止电机，请重新使能后发送目标。",
      },
    },
    {
      title: { en: "5 · Watch feedback and stop", zh: "5 · 观察反馈并停止" },
      body: {
        en: "Switch between numeric feedback and position/velocity/torque charts. Use Stop / disable to end motion and target streaming. Parameter access includes current, voltage and multi-turn position; writes require a stopped motor. Disconnect and leaving this workspace stop the selected motor before releasing the bus.",
        zh: "在数值反馈与位置、速度、转矩曲线之间切换。「停止 / 失能」结束运动和连续发送。参数区可读取电流、电压和多圈位置，写参数前需停止电机。断开连接和离开工作区都会先停止所选电机，再释放总线。",
      },
    },
    {
      title: { en: "6 · Device settings and identity", zh: "6 · 设备设置与身份信息" },
      body: {
        en: "Device settings & information reads CAN ID, MCU UID and the documented motor-version field. Changing ID accepts 0–127, stops the motor, checks address conflicts and verifies its UID at the new address. Version reads stop the motor too. Mechanical zero works in MIT / CSP and refreshes the position target. Firmware strings such as AppCodeVersion have no documented native-protocol read format in this manual.",
        zh: "「设备设置与信息」可读取 CAN ID、MCU UID 和手册定义的电机版本字段。修改 ID 支持 0–127，会先停机、检查地址冲突，再核对新地址的 UID。读取版本也会停机。机械零点支持 MIT / CSP，设置后会更新位置目标。手册未提供 AppCodeVersion 等固件字符串的私有协议读取格式。",
      },
    },
  ],
  rollercanControl: [
    {
      title: { en: "1 · Connect and discover", zh: "1 · 连接并发现" },
      body: {
        en: "Hard prerequisite: this 1 Mbps bus must contain stock-firmware Unit RollerCAN motors only; do not mix CANopen or other motor families on the same bus. The workspace scans 8-bit RollerCAN IDs with harmless extended-frame pings; the stock default is 0xA8.",
        zh: "硬性前置条件：这条 1 Mbps 总线只能连接使用原厂控制固件的 Unit RollerCAN 电机，严禁与 CANopen 或其他类型电机混接。工作区会用无动作的扩展帧 ping 扫描 8 位 RollerCAN ID；原厂默认 ID 为 0xA8。",
      },
    },
    {
      title: { en: "2 · Attach and verify feedback", zh: "2 · 挂载并确认反馈" },
      body: {
        en: "Attach reads the existing enable state and run mode without changing either. Confirm fresh position, speed, current and voltage feedback before operating the motor.",
        zh: "挂载只读取已有的使能状态和运行模式，不会更改它们。操作前请确认位置、速度、电流和电压反馈持续刷新。",
      },
    },
    {
      title: { en: "3 · Disable before changing mode", zh: "3 · 先失能再切换模式" },
      body: {
        en: "Choose Speed, Position, Current or Encoder while disabled, set a conservative current limit where available, enable, then send a target. Detach and Disconnect require a confirmed disable.",
        zh: "失能状态下选择速度、位置、电流或编码器模式；在支持的模式中先设置保守的最大电流，再使能并发送目标。卸载与断开连接都会要求确认失能。",
      },
    },
  ],
};

// Renders the slide's image/video, falling back to the placeholder caption if
// the file is missing (so it looks intentional before real media is dropped in).
function SlideMedia({ media }: { media?: Slide["media"] }) {
  const { t } = useI18n();
  const [failed, setFailed] = useState(false);

  if (media && !failed) {
    if (media.type === "image") {
      return (
        <img
          src={media.src}
          alt=""
          onError={() => setFailed(true)}
          style={{ maxWidth: "100%", maxHeight: "100%", objectFit: "contain" }}
        />
      );
    }
    return (
      <video
        src={media.src}
        controls
        onError={() => setFailed(true)}
        style={{ maxWidth: "100%", maxHeight: "100%" }}
      />
    );
  }
  return (
    <Typography.Text type="secondary">{t("tutorialMediaPlaceholder")}</Typography.Text>
  );
}

export function TutorialModal({
  open,
  onClose,
  title,
  slides,
}: {
  open: boolean;
  onClose: () => void;
  // Defaults to the landing-page "Getting started" guide when omitted.
  title?: string;
  slides?: Slide[];
}) {
  const { t, lang } = useI18n();
  const { token } = theme.useToken();
  const list = slides ?? HOME_SLIDES;

  return (
    <Modal
      open={open}
      onCancel={onClose}
      footer={null}
      width={640}
      centered
      title={title ?? t("tutorialTitle")}
    >
      <Carousel arrows draggable adaptiveHeight style={{ paddingBottom: 24 }}>
        {list.map((s, i) => (
          <div key={i}>
            <div style={{ padding: "8px 32px 0" }}>
              <div
                style={{
                  height: 280,
                  borderRadius: token.borderRadiusLG,
                  overflow: "hidden",
                  background: token.colorFillTertiary,
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "center",
                  marginBottom: 16,
                }}
              >
                <SlideMedia media={s.media} />
              </div>
              <Typography.Title level={5} style={{ marginTop: 0 }}>
                {s.title[lang]}
              </Typography.Title>
              <Typography.Paragraph type="secondary" style={{ marginBottom: 0 }}>
                {s.body[lang]}
              </Typography.Paragraph>
            </div>
          </div>
        ))}
      </Carousel>
    </Modal>
  );
}
