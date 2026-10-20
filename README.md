# hexmeow GUI (Tauri)

A Tauri 2.x desktop service GUI for hexmeow devices. It includes the original
CiA402 motor host plus robot/controller discovery, arm/base/lift/end-effector
tools, commissioning and configuration flows, firmware updates, CAN analysis,
diagnostics, and device-authenticity checks.

Frontend: **Vite + React + TypeScript + Ant Design + ECharts**.
Backend: **pure Rust** (Tauri commands over `hex-motor`).

## Layout

```
tauri-test/
├── index.html              # Vite entry
├── package.json            # frontend deps + scripts
├── vite.config.ts
├── src/                    # React frontend (TypeScript)
│   ├── main.tsx / App.tsx
│   ├── api.ts              # typed invoke() wrappers
│   ├── types.ts            # TS mirrors of the Rust DTOs
│   ├── useTelemetry.ts     # 20 Hz get_status poll + rolling buffer
│   └── components/         # ConnectBar / Sidebar / MotorDetail / LivePanel / LiveChart / ControlPanel
└── src-tauri/
    ├── tauri.conf.json
    └── src/
        ├── main.rs / lib.rs
        ├── backend.rs      # CanBus factory (per-OS / per-backend)
        ├── state.rs        # AppState: Cia402Manager + CSV log handles
        ├── dto.rs          # serde DTOs mirroring hex-motor
        ├── commands.rs     # #[tauri::command]s
        └── logging.rs      # full-rate CSV recorder task
```

## Prerequisites

### 1. System libraries (Linux)

Tauri 2.x on Linux links WebKit2GTK + libsoup-3. On Debian/Ubuntu:

```bash
sudo apt install -y \
    libwebkit2gtk-4.1-dev libsoup-3.0-dev libjavascriptcoregtk-4.1-dev \
    build-essential pkg-config libssl-dev \
    libayatana-appindicator3-dev librsvg2-dev
```

### 2. Node.js (for the frontend)

The frontend needs Node 18+ (developed on Node 24). Easiest is
[nvm](https://github.com/nvm-sh/nvm):

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
# reopen the shell, then:
nvm install 24
```

Install JS dependencies once (and after any `package.json` change):

```bash
cd tauri-test
npm install
```

### 3. A CAN interface

Three options, selected by the **interface** string in the Connect bar:

- **SocketCAN** (Linux): real hardware on `can0`, or a virtual bus to
  smoke-test without it:
  ```bash
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan
  sudo ip link set up vcan0
  ```
- **gs_usb / candleLight** (Linux / macOS / Windows): type `gs_usb`
  (or `gs_usb0`, `gs_usb1` for a specific channel) — a CAN-FD adapter
  driven directly over USB. On Linux this needs usbfs access; add a udev
  rule so the GUI can open it without running as root:
  ```bash
  # adjust idVendor/idProduct for your adapter (here: candleLight 1209:2323)
  echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="2323", MODE="0660", GROUP="plugdev"' \
    | sudo tee /etc/udev/rules.d/70-gs-usb.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
  On macOS no setup is needed (no sudo, no driver install).

### 4. HPM USB Bootloader access (Firmware Update tool)

The Firmware Update app currently enables the hardware-tested HPM USB v2
transport for the exact legacy `gs_can` Bootloader profile. On Linux, install
a udev rule for `34b7:beef` so the GUI can claim interface 0 without root:

```bash
sudo install -m 0644 /dev/stdin /etc/udev/rules.d/99-hpm-bl.rules <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="beef", MODE="0660", GROUP="plugdev", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Windows uses the Bootloader's WinUSB descriptor; macOS needs no driver setup.
For the initial MVP, connect exactly one matching Bootloader at a time.

## Run

### Dev (hot-reload, recommended)

Uses `tauri-cli`, which runs `npm run dev` (Vite at `:1420`) and the Rust app
together:

```bash
cargo install tauri-cli --version "^2" --locked   # once
cd hex-gui/src-tauri
cargo tauri dev
```

### Quick run (no tauri-cli)

Build the frontend, then run the Rust binary directly (it embeds `dist/`):

```bash
cd hex-gui
npm run build
cd src-tauri && cargo run
```

(Repeat `npm run build` after frontend changes, since `cargo run` embeds the
built `dist/` rather than talking to the Vite dev server.)

### Laggy on Linux? Use a Wayland session

If the UI feels sluggish on Linux — especially when the window is large, with
the lag getting worse the bigger the window — **log into a Wayland session**
(GDM login screen → gear icon → **"Ubuntu on Wayland"** → log in). This is by
far the biggest fix and needs no change to the app.

The cause is **WebKitGTK** — the webview Tauri uses on Linux — not this app.
On the **NVIDIA proprietary driver under X11** (and worse with fractional
display scaling), WebKitGTK's per-frame window presentation is slow, so cost
scales with window pixel area. Chromium-based apps (Chrome, VS Code) don't hit
this; WebKitGTK does. It reproduces in `cargo tauri dev` and in the prebuilt
binary alike, and none of the usual `WEBKIT_DISABLE_DMABUF_RENDERER` /
`WEBKIT_DISABLE_COMPOSITING_MODE` / Skia-CPU env toggles help — but Wayland
does.

**Quick confirmation:** install GNOME Web (`sudo apt install epiphany-browser`),
maximize it, and scroll a long page. If Epiphany is *also* laggy when large,
it's this WebKitGTK/X11 limitation (not hexmeow-gui), and switching to Wayland
is the fix.

## Packaging (Ubuntu x64)

Prebuilt packages target **Ubuntu 22.04+ / x86-64**. Other distros: build from
source (see prerequisites above). `cargo tauri build` produces both a `.deb`
and an `.AppImage`:

```bash
cd tauri-test/src-tauri
cargo tauri build                      # both deb + appimage (see bundle.targets)
# or just one:
cargo tauri build --bundles deb
cargo tauri build --bundles appimage
```

Outputs land in `src-tauri/target/release/bundle/{deb,appimage}/`.

- **`.deb`** (~5 MB) — `sudo apt install ./hexmeow-gui_*.deb`. It declares
  `libwebkit2gtk-4.1-0` + `libgtk-3-0` as dependencies, so apt pulls the
  **WebKitGTK 4.1** runtime automatically. Recommended for Ubuntu.
- **`.AppImage`** (~77 MB) — bundles WebKitGTK, so it runs without installing
  anything: `chmod +x hexmeow-gui_*.AppImage && ./hexmeow-gui_*.AppImage`.
  On Ubuntu 22.04+ you may need FUSE: `sudo apt install libfuse2` (or run with
  `--appimage-extract-and-run`).

> **Rename compatibility (v1.3.0):** the user-facing product is now
> `hexmeow-gui`, while the bundle identifier, WebView data identity, internal
> executable name, and legacy local-storage keys remain stable. The Windows MSI
> UpgradeCode is pinned to the previous value, the NSIS installer migrates the
> old registration in place, and the Debian package replaces `hex-motor-gui`.
> On macOS the new `.app` has a different filename; after copying it, remove the
> old `hex-motor-gui.app` manually. Preferences remain under the same bundle ID.
>
> **Duplicate Linux launcher.** The desktop entry is named after `productName`,
> so it moved from `hex-motor-gui.desktop` to `hexmeow-gui.desktop` — while
> `Exec`/`Icon` still point at the unchanged `hex-motor-gui` binary. `Conflicts`
> + `Replaces` clean up an old *dpkg-installed* entry, but not one that AppImage
> desktop integration or a manual `cp` left behind: that one survives, still
> launches, and shows up as a second app in the GNOME grid.
> `src-tauri/linux/deb-postinst.sh` removes such orphans under
> `/usr/{,local/}share/applications` on install and reports (but never deletes)
> per-user copies under `~/.local/share/applications`. To check by hand:
>
> ```bash
> ls /usr/share/applications/*hex* /usr/local/share/applications/*hex* \
>    ~/.local/share/applications/*hex* 2>/dev/null
> ```

> **glibc / build host:** an AppImage links against the build machine's glibc
> and is **not** forward-compatible. Build releases on the **oldest** target
> (Ubuntu 22.04) — e.g. a CI job in an `ubuntu:22.04` Docker image — so they run
> on 22.04 and up. (The `.deb` has the same constraint via its dependencies.)
>
> **Runtime dependency:** all builds need **WebKitGTK 4.1**
> (`libwebkit2gtk-4.1-0`). The `.deb` installs it for you; for the bare binary
> or other distros, install it manually (Ubuntu/Debian:
> `sudo apt install libwebkit2gtk-4.1-0`).

### CI

`.github/workflows/release.yml` builds all three desktop platforms in a matrix:

| Runner          | Bundles                                    |
| --------------- | ------------------------------------------ |
| `ubuntu-22.04`  | `.deb` + `.AppImage` (x86-64)              |
| `windows-latest`| `.msi` + NSIS `.exe` installer (x86-64)    |
| `macos-latest`  | `.dmg` + `.app`, universal (Intel + ARM)   |

The workflow runs on pushes to `main`, PRs, `v*` tags, and manual dispatch. What
it does depends on the trigger:

- Every run first reads the application version from
  `src-tauri/Cargo.toml`, verifies the lock file and runs the frontend/Rust
  test gate. Tag builds additionally require an exact `v<app-version>` match.
- **push / PR / manual** — build every platform and upload the bundles as
  **run artifacts** (Actions → the run → Artifacts). Nothing is released.
- **`v*` tag** — build every platform and create a **draft GitHub Release**
  named from the verified Cargo application version, with every bundle attached.

#### Cutting a draft release

The draft Release is driven entirely by pushing a tag that matches `v*`
(handled by [`tauri-action`](https://github.com/tauri-apps/tauri-action) with
`releaseDraft: true`). There is no button to click — just tag and push:

```bash
# src-tauri/Cargo.toml is the only manually maintained application version.
# Bump it, refresh src-tauri/Cargo.lock with cargo metadata, commit, then:
git tag v1.4.0
git push origin v1.4.0
```

Each platform's job appends its bundles to the same Release. When all three
finish, open **Releases** on GitHub — the draft is waiting there. Review it, then
**Publish** manually (drafts are never public until you publish). Do not move or
reuse a published tag. If a draft and tag have never become an external release,
a maintainer may explicitly delete and recreate both.

> Bundles are **unsigned**: macOS users right-click → Open past Gatekeeper,
> Windows users click through SmartScreen. Add signing later via `tauri-action`
> env vars.
>
> **Fresh-checkout inputs** (see the header comment in the workflow):
> Rust library dependencies are resolved from crates.io. CI checks out the verified
> shared contract from pinned `hex-meow/hex-robot-proto` commit `6848962` via `ROBOT_PROTO_DIR`.

## Usage

1. Top bar: pick the CAN interface (default `can0`; also accepts
   `socketcan:vcan0`-style prefixed specs) and your own NID (1..127, must
   differ from every motor), then **连接 (Connect)**.
2. Discovered motors appear in the left **sidebar**. Click one to open its
   detail view.
3. Click **初始化 (Initialize)** in the control card (runs
   `NMT PreOp → TPDO → fault-clear → NMT Op`). The init also brute-forces the
   firmware's flaky heartbeat-fault clear, so a freshly power-cycled or
   reconnected motor comes up clean.
4. **显示面板**: toggle between **数值** (numeric) and **图表** (a rolling
   2-D chart of position / velocity / torque; window defaults to 10 s, 1–60 s
   adjustable).
5. **记录 CSV**: flip the switch to record this motor's *full TPDO-rate*
   stream to `logs/motor_0xNN_<localtime>.csv`. Each toggle-on opens a fresh
   file; the path is shown and copyable.
6. **控制**: pick a mode (locked once enabled), **使能 (Enable)**, then send a
   mode-specific target (**发送目标**). Adjust the `0x6072` **最大力矩** limit
   (permille, with the ≈Nm equivalent shown) in any mode. After init, faults
   are **not** auto-cleared — the panel surfaces them so you can decide
   (清除错误 + 重新初始化).

The numeric panel / chart poll `get_status` at ~20 Hz (velocity is already
filtered in Rust); CSV logging subscribes to the full TPDO stream separately.

> **MIT mode units are SI** (`pos` rad, `vel` rad/s, `kp` Nm/rad, `kd`
> Nm·s/rad, `tor` Nm). The GUI converts to the motor's native Rev internally
> (±2π); `kp`/`kd` are then mapped to integers via the cached `0x2003:07`
> factor by `hex-motor`.

## Tools

On launch you pick a tool (extensible for future utilities like zero-point
setting). The choice is made *before* connecting, which lets each tool open the
bus with the right settings:

- **Motor Control** — everything above. Broadcasts our heartbeat (the motor's
  `0x1016` consumer needs it).
- **Other → EL05 Motor Control** — independent RobStride EDULITE 05 workspace
  based on the EL05 manual dated 2026-09-22. Open `can0` / `socketcan:can0`
  on Linux or `gs_usb` with Classic CAN at 1 Mbps, scan, and select the motor.
  The default host ID is `253` (`0xFD`). This app uses RobStride's native
  29-bit extended CAN protocol and its own connection. SocketCAN timing is
  configured by the operating system.

  All five motor modes are available: motion control / MIT (`0`, position,
  velocity, Kp, Kd, torque feedforward), profile position PP (`1`, position,
  speed and acceleration), velocity (`2`, speed, acceleration and current
  limit), current (`3`, Iq), and cyclic position CSP (`5`, position and speed
  limit). Values use rad, rad/s, Nm and A. Mode changes stop the motor before
  writing and reading back `run_mode`; enable starts with a neutral target.
  Send a target after enabling. Stop, fault reset, mechanical zero, parameter
  read/write, live feedback and telemetry charts are provided in the workspace.

  **Device settings & information** provides fresh CAN ID / MCU UID queries
  (type `0`), ID changes (type `7`, new IDs `0..127`), and the motor version
  query (type `4`, payload `00 C4 00 00 00 00 00 00`). ID changes stop the
  motor, probe for address conflicts, verify the same UID at the new address,
  and move the selected device. Lost acknowledgements trigger an ID query;
  an unverified address clears the selection. Version reads also stop the
  motor, since the query uses a Stop frame. Neither operation resumes motion.
  The version is displayed as the four-byte, high-byte-first value documented
  on manual p.31, alongside the complete raw reply. The manual lists firmware
  strings `AppCodeVersion` / `BootCodeVersion` at `0x1003` / `0x1000` but does
  not specify their native-protocol string transport or a separate hardware
  revision. These fields are not guessed from the motor-version reply.
  Mechanical zero is available in MIT / CSP, stops first, reads back `mechPos`,
  and refreshes the position target and chart for the new coordinate system.

  The backend owns command streaming and stops on stale feedback or a missing
  UI heartbeat. Enabling sets the volatile `canTimeout` to one second; leaving
  the workspace or closing the window stops the selected motor. Firmware
  parameter-read failures are shown explicitly, since older EL05 firmware may
  not implement every optional parameter in the current manual. This workspace
  requires the native RobStride protocol; a motor configured for CANopen or
  the alternative standard-frame MIT protocol must first be switched back.
  An attended hardware check is available with
  `cargo run --manifest-path src-tauri/Cargo.toml --example el05_smoke -- can0 1 --motion`.
  It deliberately moves motor ID 1 with small targets in all five modes,
  checks the UI timeout stop, and restores the original mode and volatile limits.
  To check identity and version without sending motion targets, run
  `cargo run --manifest-path src-tauri/Cargo.toml --example el05_settings -- can0 1 --read`.
  Use `--roundtrip-id 2` instead of `--read` to verify a temporary ID change
  and restore ID 1. Both checks stop the motor; they leave its zero unchanged.

  MIT uses `tau = Kp*(p_ref-p) + Kd*(v_ref-v) + tau_ff`; its velocity input
  is in output-shaft rad/s, not rpm, and does not create an integral velocity
  loop. With Kp=0, friction produces speed error; with Kd=0, the velocity
  input has no effect. The EL05 native frame maps ±50 rad/s over 16 bits
  (100/65535 ≈ 0.001526 rad/s per count). The display and chart use fast
  type-2/type-24 speed feedback exclusively. Following the reported hardware
  direction, the shared decoder now uses its signed value directly (+1),
  reversing the previous -1 multiplier for live monitoring, experiments and
  zero-gravity damping. Velocity targets keep their existing direction.
  The asynchronous `0x701B` `mechVel` float is displayed separately with its
  sample age. A wrapped-position
  derivative over roughly 150 ms provides a second, encoder-based check.
  Both estimates use the same motor encoder and do not establish absolute
  accuracy against an external reference. Chart points use feedback sequence
  and monotonic receive time, so dictionary responses do not duplicate points.

  **Pendulum test bench** provides eleven experiments inside the EL05 workspace:
  free decay, bidirectional velocity steps, signed torque ramps, static
  multi-angle torque checks, position steps, sine sweeps, impedance disturbance
  pulses, zero-feedforward backdrive, thermal holding, and slow triangular
  position motion, plus a dedicated small-angle static-friction calibration.
  Starting a motion experiment selects MIT and enables the
  motor automatically; free-decay recording stops the drive automatically.
  The downward reference can be captured directly without a disable or
  speed-threshold prerequisite. Position centers are relative to that reference
  when measured, otherwise relative to the position captured at run start.
  Optional gravity compensation uses the measured reference and actual angle.
  The general torque ramp continues after motion begins. Its timed experiments
  do not use position or velocity to gate start or stop. Duration is configurable
  from 2 to 3600 seconds, and a
  temperature bound of zero disables the host temperature cutoff. Duration,
  communication loss (750 ms), a drive fault, and a missing client heartbeat
  still stop the run. The service loop runs at 50 Hz; sine sweeps are limited
  to 10 Hz. Stop removes active support.

  **Zero-gravity hold** is an independent tab for moving the pendulum by hand
  and supporting it after release. Enter the end mass, arm length and uniform
  arm mass, then place the pendulum vertically down and capture its reference
  while the motor is stopped and stationary. This reference is captured by the
  backend and is invalidated after motor reselection, reconnection, a mechanical
  zero or CAN ID change, or lost feedback. It is independent of the test bench's
  reference and does not change the motor's mechanical zero.

  Start automatically prepares MIT and enables continuous support without a
  fixed duration. At 50 Hz, the backend computes
  `tau = scale * (m + arm_mass/2) * 9.80665 * L * sin(q - q_down) - damping * velocity`
  using current feedback, with MIT Kp and Kd both zero. There is no commanded
  holding angle: damping and friction let the pendulum settle after release,
  while geometry or reference errors may cause slow drift. Compensation scale,
  damping, torque limit, startup ramp, speed limit and temperature limit are
  configurable before starting; active settings are frozen. Startup ramps the
  gravity component while damping acts immediately. Start is rejected when
  the configured peak gravity compensation exceeds the torque limit; the
  combined gravity/damping command is also limited during operation. Speed or
  temperature limits, feedback older than 150 ms, a missing UI heartbeat for
  three seconds and drive faults stop support. Stop disables the motor and
  removes its supporting torque.

  **Static friction calibration** tests both directions in one run by default
  (positive → return → rest → negative → return → stop), with optional
  positive-only and negative-only modes. Place the
  pendulum naturally down, then start. It confirms 0.5 seconds of rest, records
  that starting position, and ramps a signed torque at 0.02 Nm/s by default.
  At a 0.15° displacement it freezes the measurement torque while confirming
  motion for 40 ms, then ends the ramp and returns toward the starting position.
  Maximum displacement is configurable (default 3°); an approaching boundary
  triggers return before the configured angle, and a measured exceedance
  invalidates the calibration. Software braking has finite delay, so the actual
  peak displacement is retained rather than promising zero overshoot. Return
  torque is bounded by the configured maximum test torque and lasts at most
  three seconds. Maximum torque, time limit, failure to settle, and interruption
  are reported separately for each direction. The configured timeout applies
  independently to each direction's settling and ramp, plus up to three seconds
  for its return. The second direction starts from zero torque only after a
  stable return and a new rest confirmation. An angle guard, unstable return,
  fault or interruption stops further directions; a torque/time limit followed
  by a stable return still allows testing the other direction.

  The last stationary, first qualifying motion, and confirmation frames are
  captured before telemetry thinning. Reports use the first motion frame's
  drive-estimated torque and distinguish it from the later confirmation frame.
  A captured resting position is only a displacement reference: gravity
  correction still requires the independently measured downward reference.
  The result is an estimated breakaway friction torque in Nm, not a dimensionless
  contact friction coefficient; small-angle onset still includes detection,
  inertia and drive torque estimation errors. Both directions use the configured
  torque magnitude and ramp rate, retaining signed results and independent event
  frames even if the other direction fails. Charts mark both onsets and returns;
  reports compare torque magnitudes only when both directions have valid onset
  evidence. CSV includes `static_friction_direction` and `static_friction_phase`;
  JSON retains the current pass and `completed_directions` before plot thinning.
  Legacy single-direction recordings remain single-direction results on import.

  The bench displays position/velocity references and feedback, effort,
  temperature, and derived pendulum traces alongside automatic metrics.
  Depending on the experiment and available excitation, analysis includes
  static plateaus and torque/current regression, Coulomb/viscous friction,
  free-decay period and damping, step response and tracking errors, sweep
  gain/phase, disturbance recovery, low-speed behavior, and thermal drift.
  Missing references, incomplete tests, stale current samples, and insufficient
  excitation are reported rather than converted into fabricated metrics.
  These are estimates for the combined drive and mechanical structure, using
  drive-estimated torque; they do not independently measure dynamic shaft
  torque, backlash, or efficiency. The manual's 0.94 Nm/Arms torque constant
  must not be treated as an independently calibrated conversion from `iqf`.
  All torque traces, static plateau results and exports use the native drive's
  **Nm (N·m)** unit. The EL05 260922 manual (printed pp.30, 34) maps type-2
  and type-24 feedback torque words from `0..65535` to `-6..6 Nm`. Theoretical
  holding torque `(m + arm_mass/2) * g * L * sin(theta)`, with mass in kg and
  length in m, already uses this same unit; no gear-ratio or current-constant
  conversion is applied. CSV torque columns retain their `_nm` suffix and JSON
  explicitly records `torque_unit: "Nm"`. A difference between theory and
  feedback can include friction, geometry errors and drive-estimation error.
  Static calibration previews the peak theoretical demand at its target angles;
  the torque chart and analysis show the captured `0x700B` drive limit, falling
  back to the native 6 Nm maximum when no readback is available. Theory above
  the limit remains visible and is flagged. For example, 9.96 kg at 180 mm with
  a 0.06 kg uniform arm requires 9.957 Nm at 0.6 rad, exceeding the 6 Nm peak.

  Receive-time data is captured independently of UI refresh and transferred
  incrementally. Long runs retain original CAN bytes at a duration-dependent
  interval, bounded to 120,000 samples; that interval is included in the status.
  The UI keeps recent runs for comparison, imports saved JSON, and exports
  CSV, JSON with analysis, and Markdown reports. Browser history has an
  explicit storage budget: its plot samples are reduced over the whole run
  while the metrics calculated before reduction are retained. Export the live
  recording for full retained telemetry before leaving the workspace.
  New JSON recordings use `feedback_velocity_sign=1`. Old JSON and history
  marked `-1` have their feedback and calibration-event velocities reversed
  once on load; unmarked recordings require matching original type-2/type-24
  CAN bytes. Converted recordings are marked `1` and their analyses recomputed.
  Reduced history can only be recomputed from its retained samples; the
  interface identifies this case.

  EL05 firmware `0x0A050001` was observed to emit periodic telemetry as type
  `24` with the type-2 payload layout. Its measured `EPScan_time=0` period is
  about 10 ms and `1` is about 15 ms, differing from the generic SDK manual's
  starting index. Zero is accepted for restoring this firmware's original
  setting. The displayed receive rate combines periodic reports and replies
  to commands; it is not the periodic reporting rate alone.

  CSV rows include received position, unwrapped position, feedback velocity,
  position-derived velocity, drive-estimated torque, asynchronous Iq/mechVel
  values with ages, original CAN ID/payload bytes, status, faults and host MIT targets. The target columns
  are the latest successfully sent host requests before wire quantization,
  with sequence and age. JSON also contains each MIT send's start/completion
  times and exact wire bytes; CAN send completion does not prove when the
  drive applied a command. Times are host monotonic receive
  times, not drive timestamps; held current/parameter values are not synchronized
  to each feedback frame. JSON also includes configuration and the stop reason.
  The geometry calculator uses a point load and a uniform arm. With a 1 kg
  point load at 0.18 m, gravity peaks at 1.765197 Nm and load inertia is
  0.0324 kg·m² before adding arm, fixture and motor inertia. Additional rotating
  inertia can be entered separately. Gravity compensation is opt-in and uses
  the measured downward reference. Fit friction only after subtracting gravity
  and inertial torque; reported torque is an estimate, not a torque sensor.
  Measure the downward encoder reference before fitting gravity. Geometry,
  reference angle and its measured/unmeasured flag are frozen when recording
  starts, alongside available read-back torque limit, reporting setting and CAN timeout.
  A receive queue overflow aborts the run and preserves a dropped-frame count.

  An unloaded motor speed check uses the same production experiment engine:
  `cargo run --manifest-path src-tauri/Cargo.toml --example el05_bench -- can3 1 --motion /tmp/el05-bench`.
  It compares positive and negative steady-window speed feedback with a
  regression of received position at several speeds and Kd values, saves raw
  JSON and a report, and restores the original mode, torque limit, feedback
  period and CAN timeout. It leaves the motor stopped and does not change zero.
  The check also exercises both signed timed torque ramps and a stopped passive
  recording. `el05_probe can0 can1 can2 can3` compares explicitly selected
  interfaces using only UID queries, with timing, scan coverage and error
  counters; it never selects/enables a motor or changes its parameters.
  `python3 scripts/analyze-el05-bench.py /tmp/el05-bench --png` creates an
  offline JSON report and speed/effort SVG plots (PNG with local Chrome).
- **Product Authenticity** — a public, always-visible verification and
  first-registration workspace. It passively discovers heartbeat sessions,
  but reads a proprietary proof object only after a complete `0x1018:00..04`
  re-read exactly matches a known local product profile. Meow Motor uses its
  locally strict `0x4001` v1 envelope plus the fixed
  `https://product-auth.hexmeow.com` issuance ledger. Lift G0B1 uses its
  read-only `0x2113` raw low-S P-256 proof for offline source verification;
  registration/revocation remains a separate online result. Other heartbeat
  nodes are deliberately not probed. A CAN or HTTPS failure is shown as
  unavailable, never as an invalid/counterfeit proof. Selected new devices are
  re-read and registered as one atomic mixed-device batch after the server's
  three-second cooldown.
- **Firmware Update** — an independent DFU workspace with a CAN/USB selector.
  The current USB path recognizes only exact Bootloader version `0x0100`,
  mapped locally to product code `0x6763616E` (ASCII `gcan`). Protected devices
  accept only strict `.hpmota` v2; development devices accept only structural
  plaintext APP0 `.bin`. Local selection does not bypass validation. HPM CAN is
  visible but disabled until hardware validation exists. A matching JUMP ACK
  means transfer completion, not confirmed application health; check the
  device's actual function after upgrading. The STM32 CAN page is currently a
  read-only safety preview: it passively discovers valid CANopen heartbeat
  nodes and strictly reads/classifies complete `0x1018` identities. Every known
  product profile remains disabled, so no proprietary object or CAN update
  write is sent. The common streaming engine is present behind the final
  same-transport identity gate, ready for a separately qualified product
  profile without duplicating the CLI protocol logic.
- **Lift (Raw CAN)** — direct CANopen commissioning for one `lift-driver`
  node (default `0x14`) on the already-open bus. Attach is observation-only:
  it reads identity, nameplate/CRC, effective limits, heartbeat, TPDOs and SDO
  diagnostics, including `0x4601:08..0B` sensor status, INA `DIAG_ALRT`, sample
  age and failure count, without changing NMT or sending motion. TPDO2 frame
  freshness and INA sample freshness are displayed and gated independently:
  stale V/I remain visible only as explicitly marked last-successful values.
  QEI readiness and the separately bench-qualified encoder direction are also
  distinct status bits; an initialized QEI never implies that “up counts
  positive” has been verified on the mechanism.
  A separate low-duty commissioning card is shown **only** for the exact pair
  0x1008:00 = "hexmeow-lift-commission" and 0x4700:01 U16 = 2. ABI1 and
  production images never expose these controls. ABI2 uses the frozen 0x4700
  record and exact 8-byte 0x4701:00 RPDO3
  (active_session:u32 + pulse_id:u16 + signed duty:i16).

  The device owns the anti-replay boot epoch and one-shot challenge. ARM echoes
  the currently displayed non-zero challenge with kind=Arm; that echoed value
  becomes the active session only after ArmedIdle + flags.ARMED confirmation.
  The active session is always an echoed device challenge. Clear-fault is a
  separate kind=ClearFault challenge path, enabled only while NMT is Operational
  and FaultLatched; after CAN E-stop the operator must explicitly return the
  node to Operational before clearing. The GUI displays boot epoch,
  challenge/kind, expected and active pulse IDs, qualified encoder sign, and
  the INA238 configuration-fingerprint mismatch bitmap.

  Stage A epoch establishment is offered only for MissingOrUnreadable or
  Corrupt continuity. The write is enabled only in NMT Pre-operational while
  the commissioning state is Disarmed, active_session=0, ARMED/OUTPUT flags
  are clear, boot_epoch=0, and the operator separately confirms that the motor
  is physically disconnected from the driver PCB. The backend rechecks that
  boolean, obtains a fresh non-zero u32 provisioning salt from the operating
  system random source, and writes that value to EPOCH_SERVICE; there is no
  fixed service magic. The salt is anti-stale framing rather than a secret or
  CAN authentication credential. Exhausted and WriteFailed are warning-only
  terminal states: the GUI deliberately exposes no service or retry button
  for them.

  Stage A may be performed only with the motor physically disconnected.
  Connecting the motor for Stage B remains blocked until the shared `hstd`
  persistence paths (`0x1010`, `0x1011`, and write-through Flash mutations)
  reject or defer work while the lift is Operational, armed, or output-capable;
  otherwise blocking Flash work can pause the cooperative supervisor.

  Rust owns the 20 ms RPDO3 stream. ArmedIdle sends zero keepalives; A/B
  hold-to-run repeats only the device-issued expected pulse ID and duty—the
  WebView never predicts a sequence number. Pointer release, window blur, or
  loss of the operator lease sends zero. The host mirrors the firmware-reported
  100 ms lease from 0x4700:08 instead of maintaining a longer timeout.
  Repeated frames cannot extend the firmware absolute pulse deadline.
  Firmware hard-cap, lease, and maximum-pulse values remain read-only in the
  UI; no host sensor gate assumes SAMPLE_VALID.

  TPDO3 (0x380 + node) and TPDO4 (0x480 + node) are paired by their u16
  firmware tick before display/recording. The latest 2,000 paired samples are
  kept in a bounded backend buffer (cleared by the next ARM) and can be copied
  as CSV; this is intentionally not durable file logging yet. Commissioning
  E-stop sends directed NMT Stop before waiting for SDO, then enters Pre-op
  and confirms active_session=0, state=Disarmed or FaultLatched, and clear
  ARMED/OUTPUT flags. It is a CAN
  software stop, not a safety-rated substitute for physical power removal.
  Generic Homing/Velocity/Position and the production Clear Fault command are
  disabled for every commissioning image; a latched commissioning fault can
  only use the dedicated kind=ClearFault challenge path above.
  Homing, velocity and position remain locked until heartbeat and both TPDOs
  are fresh, the encoder/INA sample is healthy, `CONFIG_VALID` is set, NMT is
  Operational, no fault is latched, and Homing has completed where required.
  Velocity is hold-to-jog: Rust owns the RPDO timing while the WebView renews a
  250 ms operator lease. Lease loss sends a
  directed NMT Stop. Detach/Disconnect and normal window close report success
  only after a Pre-operational heartbeat and Disabled-command readback; a
  failed close keeps the window open with `STOP UNCONFIRMED`. Position is an
  autonomous goal: confirmed shutdown cancels it, but a process crash cannot,
  so commissioning still requires a physical power-removal path. This tool
  does not broadcast a host heartbeat.
- **Device Settings** — the merged identity-gated Node-ID, CAN profile and
  motor-zero workspace. The sidebar shows every heartbeat-discovered node, but
  unknown `(vendor_id, product_code)` tuples have no actions. A known motor can
  change Node-ID, 1/2/4/5 Mbit/s data rate, TPDO BRS and its `0x3001` position
  preset; known non-motor products expose only the communication fields their
  actual object shape supports. Classic-only `0x2100:00 == 1` images never get
  data-rate/BRS controls. Communication Apply is accepted only while the last
  device heartbeat reports Pre-operational or Stopped; the tool does not issue
  NMT just to force that state.

  Every write button force-reads `0x1018` again and verifies the exact expected
  tuple in the same per-node exclusive SDO transaction. Motor `0x2001` changes
  are stored with one final `0x1010:01 = "save"` and take effect after a
  physical power cycle. `0x2100/0x2101` changes are write-through, are not
  followed by `0x1010`, and are never auto-reset; wait for persistence before
  restarting and verify after the next heartbeat. Position preset is a
  separate button transaction that confirms Switch On Disabled before writing
  `0x3001`.

  This workspace does **not** broadcast a host heartbeat. It sends CAN traffic
  only after receiving a device heartbeat or after a user click. Position is
  read once per online edge, on explicit Read, and once after preset; failed
  edge reads are not retried by the UI refresh. This avoids accumulating TX
  errors while a device is unplugged or the bus is empty.

Use **Switch tool** in the header to go back to the picker (it disconnects
first).

## CAN backend extension point

The GUI ships two backends: `socketcan` (Linux) and `gs_usb` (candleLight
over USB, CAN-FD, cross-platform), selected by the interface string. Adding
another backend is contained to `src-tauri/src/backend.rs` — add an arm to
`open_bus` returning an `Arc<dyn CanBus>`; nothing else in the GUI changes.
