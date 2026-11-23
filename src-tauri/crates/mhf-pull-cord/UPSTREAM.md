# Upstream source

Source: `/home/kk/kk_data/rust_project/string_chassis`, branch `main`, commit
`20133accb224eed665c9308de05181e9a653c15e` (MIT). `upstream-source.json` records
the original file hashes. No sibling path or network dependency is required.

| Local source | Upstream source | Adaptation |
|---|---|---|
| `src/pull_cord.rs`, `src/pull_cord/{a1,a2,friction}.rs` | `crates/control-core/src/pull_cord*` | Desktop mode selection, live-edit guards and shared payout/stationary/retract selection for A1 and A2 |
| `src/lingkong_pull.rs` | `crates/control-runtime/src/lingkong_pull.rs` | Selectable A1/A2 transactions, shared homing and feedback supervision, applied retraction speed for chassis braking |
| `src/motion.rs` | `crates/control-runtime/src/pull_cord/motion.rs` | Includes the applied retract target for the desktop chassis brake |
| `src/runtime_support.rs` | Diagnostic struct/default in `crates/control-runtime/src/pull_cord.rs` | Extracted without the historical HEX4310 runtime |
| `defaults.json` | `crates/control-server/defaults/settings.json`, `pull_cord` field | Same JSON object |
| `src/chassis_model.rs`, `src/chassis_model/pace.rs`, associated `*_tests.rs` and fixtures | `crates/control-core/src/chassis_model*`, follow tests and `test/fixtures` | Ahead switching also applies during pace tracking; fixture include paths adjusted |
| `chassis-defaults.json` | `crates/control-server/defaults/settings.json`, `chassis` field | Same JSON object; hardware-specific fields are not executed |
| `../magnetic-encoder` | `crates/magnetic-encoder` | Exposes position-frame arrival time and the magnet status freshness deadline |

`lib.rs` provides the transport-independent desktop entry points, validates the
embedded deployed defaults, and exposes the existing confirmed-close operation
for recovery after failed initialization. Its signed-angle helper is the exact
upstream `(value + 180).rem_euclid(360) - 180` expression.

`src-tauri/src/pull_cord_app.rs` owns the desktop CAN lease, cancellation,
confirmed disconnect, encoder DTO/zero adapter, and atomic local settings file.
The desktop-only `src/trajectory.rs` computes outlet-relative endpoint feedback
from homed drum length, measured payout speed and the calibrated direction.
The desktop uses the existing SocketCAN/gs_usb backend, checks Classic 1 Mbps,
and does not include the Linux HTTP server or board power switching.
The desktop also supports A1 using the length profile and host command ceiling
from `crates/control-core/src/pull_cord/a1.rs` at main commit
`5b3e7581da6ab6e792726a8ec7fe2d9ba8cbf6d5`. That revision runs A2 upstream.
Its A1 profile varies with cord length only. The desktop extends it with the
A2 measured-speed thresholds and end-zone adjustment, using independent A1 current settings.
The A1 frame layout, homing speed taper and confirmed shutdown follow its parent
`ebaa3bf`. The selected mode is fixed from startup through shutdown retries.

`src-tauri/src/pull_chassis.rs` runs the adapted follow state machine at 100 Hz with
native cord/encoder samples and measured HopeA3 motion. The default maps steering
to body yaw. During launched Behind pace matching, the optional omni mode combines
the longitudinal target with lateral pace from the live endpoint and wheel feedback.
It drives the existing HexMeow HopeA3 controller at 500 Hz on a separate CAN-FD interface.
Live sessions retain the exact cord publication and invalidation epoch, reject
stale feedback, and require explicit restart after stopping. A native command
deadline prevents a stalled producer from retaining wheel motion.

`src-tauri/src/pull_chassis_control.rs` adds a desktop-only live retraction-speed
brake ahead of all shared follow modes. Its adjustable threshold defaults to
the cord run's applied target, carried in the same motion publication. Braking
uses the largest configured deceleration, holds until measured translation and
yaw stop, and releases below 95% of the threshold. Manual bench behavior remains
the shared state machine's behavior. The optional override is persisted in the
desktop chassis settings; missing legacy fields select the automatic target.

The desktop follow model applies `behindAheadSwitchCordLengthM` during an
established pace session as well as before launch. Reaching it takes priority
over pace matching and the running stop guard, but still requires a zero
command and a measured stop before Ahead motion. The omni adapter includes lateral
command and feedback in this stop condition. Switch braking retains the
pace stop rate when stronger, remains latched through cord bounce, and clears
the previous pace estimate on Ahead takeover. Original fixture values remain
unchanged; distance-hold tests place the switch beyond their working range,
and `pace_switch_tests.rs` covers the runtime transition at 1.0 m.

The frontend `PullCordPanel`, `PullCordChart` and `pullCordTypes.ts` derive from
the same main-branch UI. The HTTP calls become Tauri IPC and the save target is
the local application configuration directory. Encoder controls are included in
the standalone page. Stop remains available during startup and after failed
shutdown. Chart axes ignore absent/nonfinite samples during homing.
`PaceFollowControls`, `BehindSpeedControls` and the follow parameter types derive
from the same main-branch frontend. `PullChassisPanel` retains explicit start,
manual apply/release, draft-only editing and parameter save/apply interactions;
HopeA3 motor limits and telemetry replace the single-steering hardware controls.

Desktop first-run and reset defaults are now supplied by
`src-tauri/defaults/pull-cord.json` and `src-tauri/defaults/pull-chassis.json`,
captured from the user's saved configuration on 2026-11-17 and embedded at build
time. These deployment settings override the original upstream defaults only
at the desktop boundary; the source files and hashes recorded here remain intact.

When updating, compare against the next **main** commit, keep the protocol tests,
refresh defaults and source hashes together, and rerun the commands in
`docs/pull-cord.md`. Do not substitute the DM-H3510 feature branch.
