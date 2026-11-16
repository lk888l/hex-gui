# HPM IMU workspace

The **HPM IMU** card in the public Raw CAN apps section supports the `hpm-imu`
firmware's bimu protocol. It uses a separate raw CAN session and decoder from
the older STM32 IMU panel, whose 26-byte TPDO and `0x3200` commands do not apply.
Protocol source: `hpm-imu/docs/can.md` and `hpm-imu/src/can/OD.c`.

## Connect and initialize

1. Configure SocketCAN for arbitration **1 Mbit/s**, data **4 Mbit/s**, CAN FD,
   with both sample points at **80%**, then connect. Linux defaults to `can2`;
   `gs_usb0` is the default elsewhere and configures its own adapter timing.
   SocketCAN timing remains system-managed. For example:

   ```sh
   sudo ip link set can2 down
   sudo ip link set can2 type can bitrate 1000000 sample-point 0.8 \
     dbitrate 4000000 dsample-point 0.8 fd on
   sudo ip link set can2 up
   ```

2. Select a heartbeat-discovered node or enter its Node-ID (firmware default
   **31**, hexadecimal `0x1F`). Connecting only listens; it does not scan by
   sending requests or broadcast a host heartbeat.
3. Click **Initialize**. The native transaction reads identity (`0x1018`),
   verifies vendor `0x6865786D` / product `0x62696D75`, snapshots all four PDO
   configurations including unused mapping slots, sends directed NMT Pre-op,
   writes the standard TPDO1/2 mappings and disables TPDO3/4, reads back the
   configuration, and sends directed NMT Start. Default event period is 8 ms.
   Hardware version, serial number and descriptive strings are displayed;
   they do not unlock or prohibit controls.
4. Alternatively, **Read configuration** attaches to the currently configured
   streams without changing NMT or PDO configuration. Start, Pre-op, Stop,
   Reset Node and Reset Communication are direct buttons. When a node is
   Stopped, use **Pre-op** before SDO reads or initialization; SDO is inactive
   in Stopped. Reset clears the host decoder until configuration is read again.

Disconnect and switching tools close the host session and release the bus.
The device keeps its current NMT state and continues transmitting. Use Pre-op
or Stop explicitly when you want to suppress PDOs.

## PDO editor

All four TPDOs expose enabled state, 11-bit COB-ID, transmission type, inhibit
time (100 µs units), event timer (ms), SYNC start and ordered object mappings.
Each has a maximum of **24 entries / 64 bytes**. Objects are selected from the
firmware's measurement/diagnostic dictionary with fixed integer widths.
The editor shows byte offsets, logical length and padded wire length.

Default streams:

| Stream | Default COB-ID | Objects / payload | Contents |
| --- | --- | --- | --- |
| TPDO1 | `0x180 + node` | 13 / 29 bytes → 32 | Accel, gyro, magnetic field, temperature, IMU count, status, fault |
| TPDO2 | `0x280 + node` | 13 / 29 bytes → 32 | SFLP WXYZ quaternion, gravity, gyro bias, SFLP count, age, flags |
| TPDO3/4 | `0x380/0x480 + node` | Disabled, empty | User mapping |

Payloads through 8 bytes are Classic CAN; longer ones are CAN FD with BRS and
zero padding to a legal DLC. The firmware does not expose a BRS setting.
NMT, heartbeats and SDO remain Classic CAN.

Apply automatically performs Pre-op → disable current COB-ID → clear mapping
count → write mapping → commit count → write communication parameters →
enable/disable requested COB-ID → readback → Start. If configuration fails,
the backend attempts to restore **every PDO and all 24 mapping slots**, checks
the result and restores the previously observed NMT state. An incomplete
restore is reported explicitly; read or initialize again to recover.

Types 254/255 are event driven; types 0–240 require SYNC. A zero event period
disables the event timer. **Send SYNC once** reads the actual `0x1005` COB-ID
and transmits one Classic SYNC frame; this workspace does not run a periodic
SYNC producer. There are no RPDOs or parameter persistence objects. Reset or
power cycling restores firmware defaults; no Save button is offered.

## Measurements and diagnostics

Display units are g, °/s, µT and °C; scaling is applied from the mapped object
type, regardless of which PDO carries it. The 3D view uses the chip's six-axis
SFLP quaternion in WXYZ order, body to world. Yaw is relative and can drift.
The schematic board and XYZ axes use sensor-native coordinates.

The view freezes if quaternion fields, flags, age or counter are stale/invalid.
A repeated sample counter does not refresh sample validity. Default freshness
is 250 ms. Raw values remain visible with their receive age; missing/unmapped
values show `—`. The charts show gaps during invalid/stale samples. Display
zero is a local quaternion reference and does not write to the device or alter
exported data. Sensor fault bit0 means invalid IMU data; bit1 means invalid
magnetic data. SFLP status bit0/1/2 indicate fresh quaternion/gravity/bias,
bit6/7 indicate historical error/overflow.

The diagnostics tab reads `0x3007`, `0x3008` and read-only `0x3100/0x3102` on
request. It shows sample ages/counters, SPI/I²C errors, sensor errors, FIFO
overflows, UART log drops and sensor configuration codes. It does not invent
unsupported calibration, yaw-reset, sample-rate or range writes.

## CSV

The native receiver retains up to **60 seconds / 30,000 PDO frames**. Export
writes one CSV row per received PDO, with host receive Unix time (milliseconds),
PDO number, COB-ID, FD/BRS, raw payload and decoded fields with units. Only
fields actually carried by that frame are filled; the other columns are blank.
The two default streams are asynchronous and are not combined into a falsely
synchronized sample. UI polling/chart downsampling does not affect recording.
Reading/replacing the mapping or device reset clears the buffer. It is a
bounded recent-history export, not durable disk logging.

## Raw angle accuracy test

Open **Raw angle test** after connecting and initializing the IMU. This tests
the **unmodified on-chip SFLP quaternion**, without host sensor fusion or raw
gyro integration. Attach the IMU rigidly to the motor output shaft and align
the selected sensor-native X/Y/Z axis with the shaft. Select whether positive
IMU rotation matches or opposes positive motor rotation.

Connect the IMU and motor to the same CAN FD 1M/4M bus, with different Node-IDs.
Enter the motor ID; the same `MeowMotorManager` used by Motor Control identifies
both **hexmeow-4310 and hexmeow-4342**. Commands and feedback are already in
output-shaft revolutions: 90° = 0.25 rev, 180° = 0.5 rev, without gear-ratio
scaling. Initialization configures motor feedback and its heartbeat consumer.
The test owns a host heartbeat producer (default free Node-ID 126) until the
test session is replaced or the IMU workspace disconnects. Choose an unused
host ID distinct from both devices. No encoder preset or persistent save is
performed; motor zero means its **existing absolute 0°**.

Settings include signed rotation angle (±0.1° through ±360°, with 90°/180°
shortcuts), repetition count (1–10000), speed, settling time, sample count,
motor arrival tolerance and per-leg timeout. Defaults: 90°, 10 rounds,
30°/s, 500 ms settling, 20 fresh samples, 0.1° arrival tolerance, 60 s timeout.
Each run initializes the motor and sets its profile speed/acceleration limits,
then moves to zero. Each round moves to the requested absolute angle, records
the settled measurement and returns to zero. The next round begins there.

One quaternion reference is averaged at the initial motor zero. **Zero IMU
at each home** replaces that reference from the settled home samples before
each rotation. With the switch off, the initial reference persists across all
rounds, so drift remains visible. This is a test-local reference; the firmware
has no device attitude-zero command. The 3D display reference is independent.
The result table records home residuals before resetting and return residuals
before the next round, so per-round zero does not hide return drift.

Rust owns motion, sampling and cancellation. Fresh motor feedback must remain
within the arrival tolerance and below 0.5°/s for the settling interval. Then
distinct SFLP samples are averaged; repeated sample counters are not new
measurements. Quaternion, counter, age and status must be mapped in the same
PDO (the default TPDO2 meets this requirement). Invalid/stale IMU data, motor
faults, telemetry loss and motion/sampling timeout fail the run and request
confirmed motor disable. Stop, disconnect, tool switching and normal window
close also cancel the run and disable the motor. A failed disable is shown
explicitly and can be retried with Stop; disconnect retains the session on
that failure. IMU configuration/NMT changes are blocked during a run.

Rotation is the signed twist about the selected axis relative to the home
quaternion, continuously unwrapped. This avoids Euler pitch folding and
preserves overshoot across ±180° and ±360°; it does not choose the sign or
angle branch from the expected motor target. A wrong axis or mounting
direction therefore cannot silently turn into a passing result.

- Signed error (degrees) = measured raw angle − configured motor angle.
- Signed error (%) = signed error / |configured motor angle| × 100.
- Absolute error (%) = |signed error| / |configured motor angle| × 100.
- Summary: mean/max absolute percentage, mean signed degrees and RMSE degrees.

The target is the reference, consistent with treating the motor as precise;
actual motor angle is also displayed. Zero-return residuals are in degrees,
because relative error against a zero target is undefined. A target row is
retained if its return leg fails; its return residual stays blank and the
completed-round count advances only after the return leg succeeds.
The backend saves each export as `logs/hpm_imu_0xNN_<timestamp>.csv` under the
working directory and returns its absolute, copyable path to the UI. It does
not depend on browser download handling.

## Validation

```sh
npx tsc --noEmit
npm test
npm run build
cd src-tauri
cargo test --locked --lib hpm_imu::tests
cargo test --locked --lib hpm_imu::angle_test
# Explicit hardware exercise; leaves default 8 ms mappings Operational:
HPM_IMU_IFACE=can2 HPM_IMU_NODE=31 \
  HPM_IMU_REPORT=/tmp/hpm-imu-report.json \
  cargo test --locked --lib hpm_imu::tests::hardware_can -- --ignored --nocapture
```

Protocol tests cover signed/scaled values, padded and short frames, arbitrary
mapping order, mapping limits, duplicate IDs, duplicate sample counters,
reboot invalidation, per-stream CSV, and abort recovery through a simulated
SDO server. The hardware test exercises the same native session used by the
GUI: identity, 20 ms and 8 ms streams, a 6-byte Classic TPDO, a 64-byte FD TPDO,
Stop/Pre-op, diagnostics, CSV and disconnect/reconnect.

Verified on **2026-11-06**, `can2`, node 31, hardware 1.1, firmware
`0.2.0-sflp`: both default streams were about 115 Hz at 8 ms and 48 Hz at
20 ms. The 6-byte Classic and 64-byte FD mappings, Stop/Pre-op and reconnect
passed. Invalid PDOs, sensor/SFLP errors, FIFO overflows, SPI/I²C errors and
CAN TX/RX error counters were zero. UART log drops were present (a separate
firmware logging counter). The board was left Operational with default 8 ms
TPDO1/2 mappings.

The angle test has automated coverage for all three axes, quaternion sign
flips, ±180° overshoot, 360° unwrapping, fixed/per-round references, duplicate
and reset counters, stale/incoherent samples, cancellation and failed cleanup.
A simulated CAN/SDO motor exercises the production driver with both 4310 and
4342 identities, including initialization, signed Q8.24 targets, feedback and
confirmed disable. The motor-mounted IMU accuracy test has not yet been
validated on physical hardware; the telemetry verification above predates it.
