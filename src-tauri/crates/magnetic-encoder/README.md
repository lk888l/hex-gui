# magnetic-encoder

Reusable host-side driver for the magnetic encoder used by the `gello-joint`
firmware. The crate has no Tauri, React, or application-state dependencies.

The implementation is split into two layers:

- `protocol` owns the frozen wire contract and pure frame decoding.
- `monitor` owns CAN subscriptions, freshness, drop/error accounting, bus-health
  sampling, and task shutdown.

## Current wire contract

- CANopen node ID: configurable, firmware default `0x10`
- CAN 2.0 profile: Classic CAN, 1 Mbit/s, FD and BRS disabled
- TPDO1 (`0x180 + node`), 8 bytes every 1 ms:
  - bytes `0..4`: zero-adjusted AS5600 position, little-endian `u32`
  - bytes `4..8`: CANopen object `0x1013:00`, little-endian `u32`
- TPDO2 (`0x280 + node`), 2 bytes every 100 ms:
  - bytes `0..2`: sensor status, little-endian `u16`
- position scale: 4096 counts per turn
- heartbeat (`0x700 + node`), 1 byte every 500 ms: `7F` Pre-Operational,
  `05` Operational, `04` Stopped
- NMT: standard ID `0x000`, payload `01 N` to start the selected node;
  this firmware does not respond to node-zero broadcast NMT
- SDO request/response: standard `0x600 + node` / `0x580 + node`
- sensor status: 0 OK, 1 weak field, 2 strong field, 3 missing magnet,
  4 I²C failure; values 3 and 4 invalidate the retained angle

This matches the `joint` firmware's `ClassicCanOnly` configuration. Position,
timestamp, status, and object dictionary entries retain their original layout.
The saved FD data bitrate (`0x2100:02`) and BRS flag (`0x2100:03`) have no effect
on the current firmware's wire format. Extended status/node announcements
(`0xA9` / `0xAB`) are separate from the standard CANopen identifiers.

The service defaults to `can1`, shared with the MHF5010 motor. Motor replies
(`0x141`–`0x160`) do not overlap the default encoder node's TPDOs, heartbeat,
or SDOs. Exact standard-ID subscriptions keep both devices separate.

## Firmware zero calibration

The firmware exposes the unadjusted AS5600 count at `0x3001:01` and a
flash-backed zero at `0x3000:02`. To capture the physical straight position,
read the former and write that value to the latter. The application validates
the gello-joint identity and sensor health, verifies the SDO read-back, and
then waits for a fresh near-zero TPDO before reporting immediate success.
The current `joint` sensor task reads the saved offset at startup, so changing
it normally requires a module restart (`applied_live=false`).

The firmware boots in CANopen Pre-Operational and suppresses TPDOs until an
NMT Start Remote Node command arrives. `EncoderMonitor` therefore starts its
configured node by default, watches that node's heartbeat, and rate-limits a
retry if a reset returns it to Boot-up, Stopped, or Pre-Operational. This can
be disabled in `EncoderConfig` when another process owns NMT supervision.

The default decoder accepts only Classic CAN data frames, validates the exact
standard identifier and payload length, and rejects counts outside 0–4095.
FD frames, including eight-byte frames without BRS, are reported as invalid
instead of becoming a usable angle. Old FD firmware is supported only by
explicitly using `GelloJointProtocol::with_can_fd(true)`; the service selects
this from `canFd=true` and requires an interface separate from the MHF bus.
