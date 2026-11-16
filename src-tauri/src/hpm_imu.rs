//! HPM bimu telemetry workspace. Contract: hpm-imu/src/can/OD.c and docs/can.md.
//! This is deliberately separate from the older STM32 IMU's 26-byte TPDO.
use std::collections::{BTreeMap, VecDeque};
use std::fmt::Write;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{bail, ensure, Context, Result};
use can_transport::{CanBus, CanFilter, CanFrame, CanId, FrameKind};
use canopen_sdo::asynch::{download_bytes_retry, upload_bytes_retry};
use serde::{Deserialize, Serialize};
use tokio::task::JoinHandle;

const SDO_TIMEOUT: Duration = Duration::from_millis(600);
const DISABLED: u32 = 1 << 31;
const MAX_RECORDS: usize = 30_000;

#[path = "hpm_imu_test.rs"]
pub mod angle_test;

#[derive(Clone, Debug, Serialize)]
pub struct Object {
    pub index: u16,
    pub sub: u8,
    pub bits: u8,
    pub name: String,
    pub unit: &'static str,
    pub signed: bool,
    pub scale: f64,
}

impl Object {
    fn mapping(&self) -> u32 {
        (u32::from(self.index) << 16) | (u32::from(self.sub) << 8) | u32::from(self.bits)
    }
}

pub fn objects() -> Vec<Object> {
    let mut out = Vec::new();
    let mut add = |index, sub, bits, name: String, unit, signed, scale| {
        out.push(Object {
            index,
            sub,
            bits,
            name,
            unit,
            signed,
            scale,
        });
    };
    for (i, axis) in ["W", "X", "Y", "Z"].iter().enumerate() {
        add(
            0x3000,
            i as u8 + 1,
            16,
            format!("SFLP quaternion {axis}"),
            "",
            true,
            0.0001,
        );
    }
    for (index, name, unit, scale) in [
        (0x3002, "Acceleration", "g", 0.001),
        (0x3003, "Angular rate", "°/s", 0.1),
        (0x3005, "Magnetic field", "µT", 0.1),
        (0x3006, "SFLP gravity", "g", 0.001),
    ] {
        for (i, axis) in ["X", "Y", "Z"].iter().enumerate() {
            add(
                index,
                i as u8 + 1,
                16,
                format!("{name} {axis}"),
                unit,
                true,
                scale,
            );
        }
    }
    for (i, axis) in ["X", "Y", "Z"].iter().enumerate() {
        add(
            0x3006,
            i as u8 + 4,
            16,
            format!("SFLP gyro bias {axis}"),
            "°/s",
            true,
            0.1,
        );
    }
    add(0x3004, 1, 16, "Temperature".into(), "°C", true, 0.01);
    add(0x3004, 2, 32, "IMU counter".into(), "", false, 1.0);
    add(0x3004, 3, 8, "Sensor status".into(), "", false, 1.0);
    add(0x3004, 4, 32, "Sensor fault".into(), "", false, 1.0);
    for (i, name) in [
        "IMU samples",
        "Mag samples",
        "IMU age (ms)",
        "Mag age (ms)",
        "IMU errors",
        "Mag errors",
        "Mag overflows",
        "Uptime (ms)",
        "SPI error",
        "I²C error",
        "UART log drops",
        "Sensor state",
    ]
    .iter()
    .enumerate()
    {
        add(0x3007, i as u8 + 1, 32, name.to_string(), "", false, 1.0);
    }
    for (i, name) in [
        "SFLP counter",
        "SFLP age (ms)",
        "SFLP errors",
        "FIFO overflows",
        "SFLP status",
    ]
    .iter()
    .enumerate()
    {
        add(
            0x3008,
            i as u8 + 1,
            if i == 4 { 8 } else { 32 },
            name.to_string(),
            "",
            false,
            1.0,
        );
    }
    out.sort_by_key(|o| (o.index, o.sub));
    out
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PdoConfig {
    /// Includes the CANopen disable / RTR bits, as read from 0x180x:01.
    pub cob_id: u32,
    pub transmission_type: u8,
    pub inhibit_100us: u16,
    pub event_ms: u16,
    pub sync_start: u8,
    pub mapping: Vec<u32>,
}

pub fn defaults(node: u8, period: u16) -> Vec<PdoConfig> {
    let map = |index: u32, sub: u32, bits: u32| (index << 16) | (sub << 8) | bits;
    let mut raw = Vec::new();
    for index in [0x3002, 0x3003, 0x3005] {
        raw.extend((1..=3).map(|s| map(index, s, 16)));
    }
    raw.extend([
        map(0x3004, 1, 16),
        map(0x3004, 2, 32),
        map(0x3004, 3, 8),
        map(0x3004, 4, 32),
    ]);
    let mut chip: Vec<_> = (1..=4).map(|s| map(0x3000, s, 16)).collect();
    chip.extend((1..=6).map(|s| map(0x3006, s, 16)));
    chip.extend([map(0x3008, 1, 32), map(0x3008, 2, 32), map(0x3008, 5, 8)]);
    [raw, chip, vec![], vec![]]
        .into_iter()
        .enumerate()
        .map(|(i, mapping)| PdoConfig {
            cob_id: 0x180 + 0x100 * i as u32 + u32::from(node) | if i > 1 { DISABLED } else { 0 },
            transmission_type: 255,
            inhibit_100us: 0,
            event_ms: period,
            sync_start: 0,
            mapping,
        })
        .collect()
}

fn validate(pdos: &[PdoConfig]) -> Result<()> {
    ensure!(pdos.len() == 4, "Expected four TPDO configurations");
    let catalog = objects();
    let mut ids = Vec::new();
    for (i, p) in pdos.iter().enumerate() {
        // Bits 31/30 are CANopen flags; bit 29 selects unsupported extended IDs.
        ensure!(
            p.cob_id & 0x3fff_f800 == 0 && p.cob_id & 0x7ff != 0,
            "TPDO{} requires a non-zero standard CAN ID",
            i + 1
        );
        ensure!(
            p.transmission_type <= 240 || p.transmission_type >= 254,
            "TPDO{} transmission type must be 0..240, 254 or 255",
            i + 1
        );
        ensure!(p.sync_start <= 240, "SYNC start must be 0..240");
        ensure!(
            p.mapping.len() <= 24,
            "TPDO{} exceeds 24 mapped objects",
            i + 1
        );
        let mut bits = 0usize;
        for entry in &p.mapping {
            ensure!(
                catalog.iter().any(|o| o.mapping() == *entry),
                "Unsupported mapping 0x{entry:08X}"
            );
            bits += (entry & 0xff) as usize;
        }
        ensure!(bits <= 512, "TPDO{} exceeds 64 bytes", i + 1);
        if p.cob_id & DISABLED == 0 {
            ensure!(
                !p.mapping.is_empty(),
                "TPDO{} is enabled with no mapped objects",
                i + 1
            );
            let id = p.cob_id & 0x7ff;
            ensure!(
                !ids.contains(&id),
                "Enabled TPDOs have duplicate COB-ID 0x{id:03X}"
            );
            ids.push(id);
        }
    }
    Ok(())
}

#[derive(Clone, Serialize)]
pub struct Identity {
    pub node: u8,
    pub vendor: u32,
    pub product: u32,
    pub revision: u32,
    pub serial: u32,
    pub hardware: Option<u32>,
    pub name: String,
    pub firmware: String,
}

#[derive(Clone, Serialize)]
pub struct Value {
    pub index: u16,
    pub sub: u8,
    pub value: f64,
    pub age_ms: u64,
    pub changed_age_ms: u64,
    pub rx_unix_ms: f64,
}

#[derive(Clone, Default, Serialize)]
pub struct PdoStats {
    pub frames: u64,
    pub age_ms: Option<u64>,
    pub hz: f64,
    pub bytes: usize,
    pub fd: bool,
    pub brs: bool,
}

#[derive(Serialize)]
pub struct Heartbeat {
    pub node: u8,
    pub state: u8,
    pub age_ms: u64,
}

#[derive(Serialize)]
pub struct Snapshot {
    pub identity: Option<Identity>,
    pub heartbeat: Vec<Heartbeat>,
    pub pdos: Vec<PdoConfig>,
    pub stats: Vec<PdoStats>,
    pub values: Vec<Value>,
    pub invalid_frames: u64,
    pub error: Option<String>,
    pub recorded_frames: usize,
    pub angle_test: Option<angle_test::TestSnapshot>,
}

struct TimedValue {
    value: f64,
    received: Instant,
    changed: Instant,
    unix_ms: f64,
}

struct Record {
    received: Instant,
    unix_ms: f64,
    pdo: usize,
    cob_id: u16,
    fd: bool,
    brs: bool,
    data: Vec<u8>,
    values: Vec<(u16, u8, f64)>,
}

#[derive(Default)]
struct Live {
    identity: Option<Identity>,
    heartbeat: BTreeMap<u8, (u8, Instant)>,
    pdos: Vec<PdoConfig>,
    stats: [PdoStats; 4],
    arrivals: [VecDeque<Instant>; 4],
    last: [Option<Instant>; 4],
    values: BTreeMap<(u16, u8), TimedValue>,
    invalid: u64,
    error: Option<String>,
    recording: VecDeque<Record>,
}

fn decode(pdo: &PdoConfig, data: &[u8]) -> Result<Vec<(u16, u8, f64)>> {
    let size: usize = pdo.mapping.iter().map(|m| (m & 0xff) as usize / 8).sum();
    ensure!(
        data.len() >= size,
        "Short TPDO: {} bytes, mapping needs {size}",
        data.len()
    );
    let catalog = objects();
    let mut offset = 0;
    pdo.mapping
        .iter()
        .map(|entry| {
            let o = catalog
                .iter()
                .find(|o| o.mapping() == *entry)
                .context("Unsupported received mapping")?;
            let size = o.bits as usize / 8;
            let d = &data[offset..offset + size];
            offset += size;
            let raw = match (o.signed, size) {
                (true, 2) => f64::from(i16::from_le_bytes(d.try_into()?)),
                (_, 1) => f64::from(d[0]),
                (_, 4) => f64::from(u32::from_le_bytes(d.try_into()?)),
                _ => bail!("Unsupported scalar width"),
            };
            Ok((o.index, o.sub, raw * o.scale))
        })
        .collect()
}

impl Live {
    fn clear_samples(&mut self) {
        self.values.clear();
        self.stats = Default::default();
        self.last = Default::default();
        self.arrivals = Default::default();
        self.recording.clear();
        self.invalid = 0;
        self.error = None;
    }

    fn receive(&mut self, frame: CanFrame) {
        let CanId::Standard(id) = frame.id() else {
            return;
        };
        let now = Instant::now();
        if frame.kind() == FrameKind::Data
            && (0x701..=0x77f).contains(&id)
            && frame.data().len() == 1
        {
            let node = (id - 0x700) as u8;
            let state = frame.data()[0];
            self.heartbeat.insert(node, (state, now));
            if state == 0 && self.identity.as_ref().is_some_and(|i| i.node == node) {
                // Reset restores firmware defaults, so an old mapping must never decode new frames.
                self.pdos.clear();
                self.clear_samples();
                self.error =
                    Some("Device rebooted; read PDO configuration or initialize again".into());
            }
            return;
        }
        if matches!(frame.kind(), FrameKind::Remote) {
            return;
        }
        let Some(i) = self
            .pdos
            .iter()
            .position(|p| p.cob_id & DISABLED == 0 && p.cob_id & 0x7ff == u32::from(id))
        else {
            return;
        };
        let values = match decode(&self.pdos[i], frame.data()) {
            Ok(v) => v,
            Err(e) => {
                self.invalid += 1;
                self.error = Some(e.to_string());
                return;
            }
        };
        let unix_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64()
            * 1000.0;
        for &(index, sub, value) in &values {
            let changed = self
                .values
                .get(&(index, sub))
                .filter(|old| old.value == value)
                .map_or(now, |v| v.changed);
            self.values.insert(
                (index, sub),
                TimedValue {
                    value,
                    received: now,
                    changed,
                    unix_ms,
                },
            );
        }
        let fd = matches!(frame.kind(), FrameKind::Fd { .. });
        let brs = matches!(frame.kind(), FrameKind::Fd { brs: true });
        self.stats[i].frames += 1;
        self.stats[i].bytes = frame.data().len();
        self.stats[i].fd = fd;
        self.stats[i].brs = brs;
        self.last[i] = Some(now);
        self.arrivals[i].push_back(now);
        while self.arrivals[i]
            .front()
            .is_some_and(|t| now.duration_since(*t) > Duration::from_secs(1))
        {
            self.arrivals[i].pop_front();
        }
        self.recording.push_back(Record {
            received: now,
            unix_ms,
            pdo: i + 1,
            cob_id: id,
            fd,
            brs,
            data: frame.data().to_vec(),
            values,
        });
        while self.recording.len() > MAX_RECORDS
            || self
                .recording
                .front()
                .is_some_and(|r| now.duration_since(r.received) > Duration::from_secs(60))
        {
            self.recording.pop_front();
        }
    }

    fn snapshot(&self) -> Snapshot {
        let now = Instant::now();
        Snapshot {
            identity: self.identity.clone(),
            heartbeat: self
                .heartbeat
                .iter()
                .map(|(&node, &(state, time))| Heartbeat {
                    node,
                    state,
                    age_ms: now.duration_since(time).as_millis() as u64,
                })
                .collect(),
            pdos: self.pdos.clone(),
            stats: self
                .stats
                .iter()
                .enumerate()
                .map(|(i, s)| {
                    let mut s = s.clone();
                    s.age_ms = self.last[i].map(|t| now.duration_since(t).as_millis() as u64);
                    let recent: Vec<_> = self.arrivals[i]
                        .iter()
                        .filter(|t| now.duration_since(**t) <= Duration::from_secs(1))
                        .collect();
                    s.hz = if recent.len() > 1 {
                        (recent.len() - 1) as f64
                            / now.duration_since(**recent.first().unwrap()).as_secs_f64()
                    } else {
                        0.0
                    };
                    s
                })
                .collect(),
            values: self
                .values
                .iter()
                .map(|(&(index, sub), v)| Value {
                    index,
                    sub,
                    value: v.value,
                    age_ms: now.duration_since(v.received).as_millis() as u64,
                    changed_age_ms: now.duration_since(v.changed).as_millis() as u64,
                    rx_unix_ms: v.unix_ms,
                })
                .collect(),
            invalid_frames: self.invalid,
            error: self.error.clone(),
            recorded_frames: self.recording.len(),
            angle_test: None,
        }
    }
}

struct SavedPdo {
    config: PdoConfig,
    slots: Vec<u32>,
}

pub struct Session {
    bus: Arc<dyn CanBus>,
    live: Arc<Mutex<Live>>,
    task: JoinHandle<()>,
    angle_test: Option<angle_test::Run>,
}

impl Drop for Session {
    fn drop(&mut self) {
        self.task.abort();
    }
}

impl Session {
    pub async fn new(bus: Arc<dyn CanBus>) -> Result<Self> {
        // Keep one cancellation-safe send queue for IMU SDOs, motor commands
        // and heartbeats across repeated tests on this physical connection.
        let bus = crate::backend::protect_feature_bus(bus);
        let mut rx = bus
            .subscribe(CanFilter {
                id: 0,
                mask: 0,
                extended: false,
            })
            .await?;
        let live = Arc::new(Mutex::new(Live::default()));
        let shared = live.clone();
        let task = tokio::spawn(async move {
            loop {
                match rx.recv().await {
                    Ok(frame) => shared.lock().unwrap().receive(frame),
                    Err(e) => {
                        shared.lock().unwrap().error = Some(format!("CAN receive: {e}"));
                        break;
                    }
                }
            }
        });
        Ok(Self {
            bus,
            live,
            task,
            angle_test: None,
        })
    }

    pub fn snapshot(&self) -> Snapshot {
        let mut snapshot = self.live.lock().unwrap().snapshot();
        snapshot.angle_test = self.angle_test.as_ref().map(|run| run.snapshot());
        snapshot
    }

    fn ensure_test_idle(&self) -> Result<()> {
        ensure!(
            self.angle_test
                .as_ref()
                .is_none_or(|run| !run.blocks_changes()),
            "Stop the angle test and confirm motor disable before changing IMU configuration"
        );
        Ok(())
    }

    pub async fn shutdown(&mut self) -> Result<()> {
        if let Some(run) = &mut self.angle_test {
            run.stop().await?;
        }
        self.angle_test = None;
        self.task.abort();
        let _ = (&mut self.task).await;
        Ok(())
    }

    pub async fn stop(mut self) {
        if let Err(error) = self.shutdown().await {
            log::error!("IMU test shutdown: {error:#}");
        }
    }

    async fn read(&self, node: u8, index: u16, sub: u8) -> Result<Vec<u8>> {
        upload_bytes_retry(&*self.bus, node, index, sub, Some(SDO_TIMEOUT), 1)
            .await
            .map_err(|e| anyhow::anyhow!("SDO read {index:04X}:{sub:02X}: {e}"))
    }

    async fn number(&self, node: u8, index: u16, sub: u8, size: usize) -> Result<u32> {
        let data = self.read(node, index, sub).await?;
        ensure!(
            data.len() == size,
            "SDO {index:04X}:{sub:02X}: expected {size} bytes, got {}",
            data.len()
        );
        let mut bytes = [0; 4];
        bytes[..size].copy_from_slice(&data);
        Ok(u32::from_le_bytes(bytes))
    }

    async fn write(&self, node: u8, index: u16, sub: u8, value: u32, size: usize) -> Result<()> {
        download_bytes_retry(
            &*self.bus,
            node,
            index,
            sub,
            &value.to_le_bytes()[..size],
            Some(SDO_TIMEOUT),
            1,
        )
        .await
        .map_err(|e| anyhow::anyhow!("SDO write {index:04X}:{sub:02X}: {e}"))
    }

    async fn identify(&self, node: u8) -> Result<Identity> {
        ensure!((1..=127).contains(&node), "Node-ID must be 1..127");
        let vendor = self.number(node, 0x1018, 1, 4).await?;
        let product = self.number(node, 0x1018, 2, 4).await?;
        ensure!(
            vendor == 0x6865786d && product == 0x62696d75,
            "Node {node} is not HPM bimu (vendor=0x{vendor:08X}, product=0x{product:08X})"
        );
        Ok(Identity {
            node,
            vendor,
            product,
            revision: self.number(node, 0x1018, 3, 4).await?,
            serial: self.number(node, 0x1018, 4, 4).await?,
            // Hardware provisioning and descriptive strings are not compatibility gates.
            hardware: self.number(node, 0x2102, 0, 4).await.ok(),
            name: String::from_utf8_lossy(&self.read(node, 0x1008, 0).await.unwrap_or_default())
                .into(),
            firmware: String::from_utf8_lossy(
                &self.read(node, 0x100a, 0).await.unwrap_or_default(),
            )
            .into(),
        })
    }

    async fn read_pdos(&self, node: u8) -> Result<Vec<SavedPdo>> {
        let mut out = Vec::new();
        for i in 0..4 {
            let c = 0x1800 + i;
            let m = 0x1a00 + i;
            let cob_id = self.number(node, c, 1, 4).await?;
            let transmission_type = self.number(node, c, 2, 1).await? as u8;
            let inhibit_100us = self.number(node, c, 3, 2).await? as u16;
            let event_ms = self.number(node, c, 5, 2).await? as u16;
            let sync_start = self.number(node, c, 6, 1).await? as u8;
            let count = self.number(node, m, 0, 1).await? as usize;
            ensure!(
                count <= 24,
                "TPDO{} mapping count {count} exceeds 24",
                i + 1
            );
            let mut slots = Vec::new();
            for s in 1..=24 {
                slots.push(self.number(node, m, s, 4).await?);
            }
            out.push(SavedPdo {
                config: PdoConfig {
                    cob_id,
                    transmission_type,
                    inhibit_100us,
                    event_ms,
                    sync_start,
                    mapping: slots[..count].to_vec(),
                },
                slots,
            });
        }
        Ok(out)
    }

    pub async fn inspect(&self, node: u8) -> Result<()> {
        let identity = self.identify(node).await?;
        let configs = self
            .read_pdos(node)
            .await?
            .into_iter()
            .map(|p| p.config)
            .collect();
        let mut live = self.live.lock().unwrap();
        live.clear_samples();
        live.identity = Some(identity);
        live.pdos = configs;
        Ok(())
    }

    async fn nmt_raw(&self, node: u8, command: u8) -> Result<()> {
        self.bus
            .send(CanFrame::new_data(0u16, &[command, node])?)
            .await?;
        Ok(())
    }

    pub async fn nmt(&self, node: u8, command: u8) -> Result<()> {
        ensure!(
            [1, 2, 0x80, 0x81, 0x82].contains(&command),
            "Unsupported NMT command"
        );
        // NMT must also work in Stopped, where SDO is unavailable.
        ensure!((1..=127).contains(&node), "Node-ID must be 1..127");
        self.nmt_raw(node, command).await
    }

    async fn write_pdo(&self, node: u8, i: usize, p: &PdoConfig, slots: &[u32]) -> Result<()> {
        let c = 0x1800 + i as u16;
        let m = 0x1a00 + i as u16;
        // Disable the CURRENT COB-ID before changing it (CANopenNode requirement).
        let old_id = self.number(node, c, 1, 4).await?;
        self.write(node, c, 1, old_id | DISABLED, 4).await?;
        self.write(node, m, 0, 0, 1).await?;
        for (j, &value) in slots.iter().enumerate() {
            self.write(node, m, j as u8 + 1, value, 4).await?;
        }
        self.write(node, m, 0, p.mapping.len() as u32, 1).await?;
        self.write(node, c, 2, p.transmission_type.into(), 1)
            .await?;
        self.write(node, c, 3, p.inhibit_100us.into(), 2).await?;
        self.write(node, c, 5, p.event_ms.into(), 2).await?;
        self.write(node, c, 6, p.sync_start.into(), 1).await?;
        self.write(node, c, 1, p.cob_id, 4).await?;
        Ok(())
    }

    pub async fn configure(&self, node: u8, pdos: Vec<PdoConfig>) -> Result<()> {
        validate(&pdos)?;
        let identity = self.identify(node).await?;
        let saved = self.read_pdos(node).await?;
        let old_nmt = self.live.lock().unwrap().heartbeat.get(&node).map(|v| v.0);
        self.nmt_raw(node, 0x80).await?;
        self.live.lock().unwrap().pdos.clear();
        let result: Result<()> = async {
            for (i, p) in pdos.iter().enumerate() {
                self.write_pdo(node, i, p, &p.mapping).await?;
            }
            let actual: Vec<_> = self
                .read_pdos(node)
                .await?
                .into_iter()
                .map(|p| p.config)
                .collect();
            ensure!(
                actual == pdos,
                "PDO readback does not match requested configuration"
            );
            Ok(())
        }
        .await;
        if let Err(error) = result {
            // Best effort on every PDO, even when an earlier restore fails.
            let mut failures = Vec::new();
            for (i, p) in saved.iter().enumerate() {
                if let Err(e) = self.write_pdo(node, i, &p.config, &p.slots).await {
                    failures.push(e.to_string());
                }
            }
            match self.read_pdos(node).await {
                Ok(actual)
                    if actual
                        .iter()
                        .zip(&saved)
                        .all(|(a, b)| a.config == b.config && a.slots == b.slots) => {}
                Ok(_) => failures.push("restored PDO readback mismatch".into()),
                Err(e) => failures.push(e.to_string()),
            }
            {
                let mut live = self.live.lock().unwrap();
                live.clear_samples();
                live.identity = Some(identity);
                if failures.is_empty() {
                    live.pdos = saved.into_iter().map(|p| p.config).collect();
                }
            }
            if failures.is_empty() {
                if let Some(cmd) = match old_nmt {
                    Some(5) => Some(1),
                    Some(4) => Some(2),
                    _ => None,
                } {
                    if let Err(e) = self.nmt_raw(node, cmd).await {
                        failures.push(e.to_string());
                    }
                }
            }
            bail!(
                "{error:#}; {}",
                if failures.is_empty() {
                    "previous PDO configuration restored".into()
                } else {
                    format!(
                        "restore incomplete: {}. Read configuration or initialize again",
                        failures.join("; ")
                    )
                }
            );
        }
        {
            let mut live = self.live.lock().unwrap();
            live.clear_samples();
            live.identity = Some(identity);
            live.pdos = pdos;
        }
        self.nmt_raw(node, 1)
            .await
            .context("PDOs configured, but NMT Start failed")
    }

    pub async fn sync(&self, node: u8) -> Result<()> {
        let id = self.number(node, 0x1005, 0, 4).await? & 0x1fff_ffff;
        ensure!(id <= 0x7ff, "Extended SYNC IDs are unsupported");
        self.bus.send(CanFrame::new_data(id as u16, &[])?).await?;
        Ok(())
    }

    pub async fn diagnostics(&self, node: u8) -> Result<Vec<Value>> {
        self.identify(node).await?;
        let mut values = Vec::new();
        for (index, count, size) in [
            (0x3007, 12, 4),
            (0x3008, 5, 4),
            (0x3100, 4, 1),
            (0x3102, 4, 1),
        ] {
            for sub in 1..=count {
                let size = if index == 0x3008 && sub == 5 { 1 } else { size };
                values.push(Value {
                    index,
                    sub,
                    value: self.number(node, index, sub, size).await?.into(),
                    age_ms: 0,
                    changed_age_ms: 0,
                    rx_unix_ms: SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_secs_f64()
                        * 1000.0,
                });
            }
        }
        Ok(values)
    }

    pub fn csv(&self) -> String {
        let catalog = objects();
        let live = self.live.lock().unwrap();
        let mut csv = String::from("rx_unix_ms,pdo,cob_id,fd,brs,data_hex");
        for o in &catalog {
            let _ = write!(
                csv,
                ",{:04X}:{:02X} {} ({})",
                o.index, o.sub, o.name, o.unit
            );
        }
        csv.push('\n');
        for r in live
            .recording
            .iter()
            .filter(|r| r.received.elapsed() <= Duration::from_secs(60))
        {
            let _ = write!(
                csv,
                "{:.3},{},0x{:03X},{},{},{}",
                r.unix_ms,
                r.pdo,
                r.cob_id,
                r.fd,
                r.brs,
                hex::encode(&r.data)
            );
            for o in &catalog {
                csv.push(',');
                if let Some(v) = r.values.iter().find(|v| v.0 == o.index && v.1 == o.sub) {
                    let _ = write!(csv, "{}", v.2);
                }
            }
            csv.push('\n');
        }
        csv
    }
}

// All SDO/configuration commands serialize with disconnect via the session mutex.
// The receive task has its own short-lived data lock and never awaits under it.
use crate::{
    can_lease::{CanOwner, CanTransportGate},
    state::AppState,
};
use tauri::State;

#[derive(Serialize)]
pub struct Connection {
    link: crate::dto::ConnectionInfoDto,
    objects: Vec<Object>,
}

#[tauri::command]
pub async fn hpm_imu_connect(
    state: State<'_, AppState>,
    gate: State<'_, CanTransportGate>,
    spec: String,
) -> Result<Connection, String> {
    let mut session = state.hpm_imu.lock().await;
    if session.is_some() {
        return Err("HPM IMU already connected".into());
    }
    let lease = gate.try_acquire(CanOwner::HpmImu)?;
    let (bus, _) = crate::backend::open_bus(spec.trim(), 4_000_000, false, lease)
        .await
        .map_err(|e| format!("{e:#}"))?;
    let (config, error) = match bus.link_config().await {
        Ok(c) => (c, None),
        Err(e) => (None, Some(e.to_string())),
    };
    let link = crate::dto::ConnectionInfoDto::new(
        if spec.starts_with("gs_usb") {
            "gs_usb"
        } else {
            "socketcan"
        },
        config,
        error,
    );
    *session = Some(Session::new(bus).await.map_err(|e| format!("{e:#}"))?);
    Ok(Connection {
        link,
        objects: objects(),
    })
}

#[tauri::command]
pub async fn hpm_imu_disconnect(state: State<'_, AppState>) -> Result<(), String> {
    let _operation = state.extension_op.lock().await;
    disconnect_session(&state).await?;
    Ok(())
}

pub(crate) async fn disconnect_session(state: &AppState) -> Result<(), String> {
    let mut guard = state.hpm_imu.lock().await;
    if let Some(session) = guard.as_mut() {
        session.shutdown().await.map_err(|e| format!("{e:#}"))?;
    }
    *guard = None;
    Ok(())
}

#[tauri::command]
pub async fn hpm_imu_state(state: State<'_, AppState>) -> Result<Snapshot, String> {
    Ok(state
        .hpm_imu
        .lock()
        .await
        .as_ref()
        .ok_or("HPM IMU not connected")?
        .snapshot())
}

#[tauri::command]
pub async fn hpm_imu_read(state: State<'_, AppState>, node: u8) -> Result<Snapshot, String> {
    let guard = state.hpm_imu.lock().await;
    let session = guard.as_ref().ok_or("HPM IMU not connected")?;
    session.ensure_test_idle().map_err(|e| e.to_string())?;
    session.inspect(node).await.map_err(|e| format!("{e:#}"))?;
    Ok(session.snapshot())
}

#[tauri::command]
pub async fn hpm_imu_configure(
    state: State<'_, AppState>,
    node: u8,
    pdos: Option<Vec<PdoConfig>>,
    period_ms: u16,
) -> Result<Snapshot, String> {
    let guard = state.hpm_imu.lock().await;
    let session = guard.as_ref().ok_or("HPM IMU not connected")?;
    session.ensure_test_idle().map_err(|e| e.to_string())?;
    session
        .configure(node, pdos.unwrap_or_else(|| defaults(node, period_ms)))
        .await
        .map_err(|e| format!("{e:#}"))?;
    Ok(session.snapshot())
}

#[tauri::command]
pub async fn hpm_imu_nmt(state: State<'_, AppState>, node: u8, command: u8) -> Result<(), String> {
    let guard = state.hpm_imu.lock().await;
    guard
        .as_ref()
        .ok_or("HPM IMU not connected")?
        .ensure_test_idle()
        .map_err(|e| e.to_string())?;
    guard
        .as_ref()
        .ok_or("HPM IMU not connected")?
        .nmt(node, command)
        .await
        .map_err(|e| format!("{e:#}"))
}

#[tauri::command]
pub async fn hpm_imu_sync(state: State<'_, AppState>, node: u8) -> Result<(), String> {
    let guard = state.hpm_imu.lock().await;
    guard
        .as_ref()
        .ok_or("HPM IMU not connected")?
        .ensure_test_idle()
        .map_err(|e| e.to_string())?;
    guard
        .as_ref()
        .ok_or("HPM IMU not connected")?
        .sync(node)
        .await
        .map_err(|e| format!("{e:#}"))
}

#[tauri::command]
pub async fn hpm_imu_diagnostics(
    state: State<'_, AppState>,
    node: u8,
) -> Result<Vec<Value>, String> {
    let guard = state.hpm_imu.lock().await;
    guard
        .as_ref()
        .ok_or("HPM IMU not connected")?
        .ensure_test_idle()
        .map_err(|e| e.to_string())?;
    guard
        .as_ref()
        .ok_or("HPM IMU not connected")?
        .diagnostics(node)
        .await
        .map_err(|e| format!("{e:#}"))
}

#[tauri::command]
pub async fn hpm_imu_export_csv(state: State<'_, AppState>) -> Result<String, String> {
    let (node, csv) = {
        let guard = state.hpm_imu.lock().await;
        let session = guard.as_ref().ok_or("HPM IMU not connected")?;
        let node = session
            .snapshot()
            .identity
            .ok_or("Read or initialize the IMU first")?
            .node;
        (node, session.csv())
    };
    tokio::task::spawn_blocking(move || write_csv_file(std::path::Path::new("logs"), node, &csv))
        .await
        .map_err(|e| e.to_string())?
        .map_err(|e| format!("{e:#}"))
}

fn write_csv_file(directory: &std::path::Path, node: u8, csv: &str) -> Result<String> {
    use std::io::Write as IoWrite;
    std::fs::create_dir_all(directory).context("Create IMU export directory")?;
    let stamp = chrono::Local::now().format("%Y%m%d_%H%M%S_%9f");
    let path = directory.join(format!("hpm_imu_0x{node:02X}_{stamp}.csv"));
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&path)
        .with_context(|| format!("Create {}", path.display()))?;
    file.write_all(csv.as_bytes())?;
    file.flush()?;
    Ok(std::fs::canonicalize(&path)?.to_string_lossy().into_owned())
}

#[cfg(test)]
#[path = "hpm_imu_tests.rs"]
mod tests;
