# Upstream source

Source: `/home/kk/kk_data/rust_project/string_chassis`, branch `main`, commit
`20133accb224eed665c9308de05181e9a653c15e` (MIT). `upstream-source.json` records
the original file hashes. No sibling path or network dependency is required.

| Local source | Upstream source | Adaptation |
|---|---|---|
| `src/pull_cord.rs`, `src/pull_cord/{a1,a2,friction}.rs` | `crates/control-core/src/pull_cord*` | Copied unchanged, including tests and legacy configuration compatibility |
| `src/lingkong_pull.rs` | `crates/control-runtime/src/lingkong_pull.rs` | Import paths and adapter visibility; historical default helper is test-only |
| `src/motion.rs` | `crates/control-runtime/src/pull_cord/motion.rs` | Copied unchanged |
| `src/runtime_support.rs` | Diagnostic struct/default in `crates/control-runtime/src/pull_cord.rs` | Extracted without the historical HEX4310 runtime |
| `defaults.json` | `crates/control-server/defaults/settings.json`, `pull_cord` field | Same JSON object |
| `src/chassis_model.rs`, `src/chassis_model/pace.rs`, associated `*_tests.rs` and fixtures | `crates/control-core/src/chassis_model*`, follow tests and `test/fixtures` | Algorithms unchanged; fixture include paths adjusted |
| `chassis-defaults.json` | `crates/control-server/defaults/settings.json`, `chassis` field | Same JSON object; hardware-specific fields are not executed |
| `../magnetic-encoder` | `crates/magnetic-encoder` | Copied unchanged |

`lib.rs` provides the transport-independent desktop entry points, validates the
embedded deployed defaults, and exposes the existing confirmed-close operation
for recovery after failed initialization. Its signed-angle helper is the exact
upstream `(value + 180).rem_euclid(360) - 180` expression.

`src-tauri/src/pull_cord_app.rs` owns the desktop CAN lease, cancellation,
confirmed disconnect, encoder DTO/zero adapter, and atomic local settings file.
The desktop uses the existing SocketCAN/gs_usb backend, checks Classic 1 Mbps,
and does not include the Linux HTTP server or board power switching.
Motor wire commands, control loop, homing and live torque policy are unchanged.

`src-tauri/src/pull_chassis.rs` runs the same follow state machine at 100 Hz with
native cord/encoder samples and measured HopeA3 motion. It adapts steering to
body yaw with zero strafe, and drives the existing HexMeow HopeA3 controller at
500 Hz on a separate CAN-FD interface. No DAMIAO hardware driver is imported.
Live sessions retain the exact cord publication and invalidation epoch, reject
stale feedback, and require explicit restart after stopping. A native command
deadline prevents a stalled producer from retaining wheel motion.

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
