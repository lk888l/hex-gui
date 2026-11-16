use super::*;
use async_trait::async_trait;
use can_transport::{CanCapabilities, CanIoError, CanRx};
use tokio::sync::mpsc;

#[test]
fn firmware_layout_units_padding_and_remapped_classic() {
    let mut pdos = defaults(31, 8);
    assert_eq!(pdos[0].mapping.len(), 13);
    assert_eq!(pdos[1].mapping.len(), 13);
    let mut raw = [0; 32];
    raw[0..2].copy_from_slice(&(-1234i16).to_le_bytes());
    raw[6..8].copy_from_slice(&456i16.to_le_bytes());
    raw[12..14].copy_from_slice(&(-98i16).to_le_bytes());
    raw[18..20].copy_from_slice(&2345i16.to_le_bytes());
    raw[20..24].copy_from_slice(&0xf1234567u32.to_le_bytes());
    raw[24] = 3;
    let values = decode(&pdos[0], &raw).unwrap();
    assert_eq!(values[0], (0x3002, 1, -1.234));
    assert!((values[3].2 - 45.6).abs() < 1e-9);
    assert!((values[6].2 + 9.8).abs() < 1e-9);
    assert!((values[9].2 - 23.45).abs() < 1e-9);
    assert_eq!(values[10].2, 0xf1234567u32 as f64);
    assert!(decode(&pdos[0], &raw[..28]).is_err());
    // Custom ordering and Classic payload are decoded from the mapping, not offsets.
    pdos[2].mapping = vec![0x30040110, 0x30030110];
    assert_eq!(
        decode(&pdos[2], &[0x10, 0x27, 0xf6, 0xff]).unwrap(),
        vec![(0x3004, 1, 100.0), (0x3003, 1, -1.0)]
    );
    let mut chip = [0; 32];
    chip[0..2].copy_from_slice(&10000i16.to_le_bytes());
    chip[20..24].copy_from_slice(&42u32.to_le_bytes());
    chip[24..28].copy_from_slice(&u32::MAX.to_le_bytes());
    let chip = decode(&pdos[1], &chip).unwrap();
    assert_eq!(chip[0].2, 1.0);
    assert_eq!(chip[10].2, 42.0);
    assert_eq!(chip[11].2, u32::MAX as f64);
}

#[test]
fn mapping_limits_and_cob_id_validation() {
    let mut p = defaults(31, 8);
    validate(&p).unwrap();
    p[2].mapping = vec![0x30040220; 16]; // 64 bytes
    p[2].cob_id &= !DISABLED;
    validate(&p).unwrap();
    p[2].mapping.push(0x30040308); // 65 bytes
    assert!(validate(&p).is_err());
    p[2].mapping = vec![0x30040308; 25];
    assert!(validate(&p).is_err());
    p[2].mapping.truncate(24);
    validate(&p).unwrap();
    p[2].cob_id = p[0].cob_id;
    assert!(validate(&p).is_err());
    p[2].cob_id = 0x800;
    assert!(validate(&p).is_err());
    p[2].cob_id = 0x39f;
    p[2].mapping = vec![0x30040210]; // Wrong width
    assert!(validate(&p).is_err());
}

#[test]
fn restart_discards_old_mapping_and_duplicate_counter_keeps_age() {
    let mut live = Live::default();
    live.identity = Some(Identity {
        node: 31,
        vendor: 0,
        product: 0,
        revision: 0,
        serial: 0,
        hardware: None,
        name: "".into(),
        firmware: "".into(),
    });
    live.pdos = defaults(31, 8);
    let frame = CanFrame::new_fd(0x19fu16, &[0; 32], true).unwrap();
    live.receive(frame);
    let old = Instant::now() - Duration::from_millis(400);
    live.values.get_mut(&(0x3004, 2)).unwrap().changed = old;
    live.receive(frame);
    assert_eq!(live.values[&(0x3004, 2)].changed, old);
    assert_eq!(live.stats[0].frames, 2);
    live.receive(CanFrame::new_data(0x71fu16, &[0]).unwrap());
    assert!(live.pdos.is_empty());
    assert!(live.values.is_empty());
    live.receive(frame);
    assert_eq!(live.stats[0].frames, 0);
    assert!(live.error.as_ref().unwrap().contains("rebooted"));
}

// A tiny expedited-SDO server to exercise the real client and transaction ordering.
// It enforces disabling a PDO and zeroing the map count before changing entries.
struct Device {
    od: Mutex<BTreeMap<(u16, u8), Vec<u8>>>,
    listeners: Mutex<Vec<(CanFilter, mpsc::UnboundedSender<CanFrame>)>>,
    writes: Mutex<Vec<(u16, u8)>>,
    fail_once: Mutex<Option<(u16, u8)>>,
}
impl Device {
    fn new() -> Arc<Self> {
        let mut od = BTreeMap::new();
        for (s, v) in [(1, 0x6865786du32), (2, 0x62696d75), (3, 0), (4, 42)] {
            od.insert((0x1018, s), v.to_le_bytes().to_vec());
        }
        od.insert((0x2102, 0), u32::MAX.to_le_bytes().to_vec()); // Unprovisioned hardware is allowed.
        od.insert((0x1008, 0), b"bimu".to_vec());
        od.insert((0x100a, 0), b"test".to_vec());
        for (i, p) in defaults(31, 8).iter().enumerate() {
            let c = 0x1800 + i as u16;
            let m = 0x1a00 + i as u16;
            od.insert((c, 1), p.cob_id.to_le_bytes().to_vec());
            od.insert((c, 2), vec![p.transmission_type]);
            od.insert((c, 3), p.inhibit_100us.to_le_bytes().to_vec());
            od.insert((c, 5), p.event_ms.to_le_bytes().to_vec());
            od.insert((c, 6), vec![p.sync_start]);
            od.insert((m, 0), vec![p.mapping.len() as u8]);
            for j in 1..=24 {
                od.insert(
                    (m, j),
                    p.mapping
                        .get(j as usize - 1)
                        .copied()
                        .unwrap_or(0)
                        .to_le_bytes()
                        .to_vec(),
                );
            }
        }
        Arc::new(Self {
            od: Mutex::new(od),
            listeners: Mutex::new(vec![]),
            writes: Mutex::new(vec![]),
            fail_once: Mutex::new(None),
        })
    }
    fn emit(&self, frame: CanFrame) {
        self.listeners.lock().unwrap().retain(|(filter, tx)| {
            !tx.is_closed() && (!filter.matches(&frame) || tx.send(frame).is_ok())
        });
    }
}
struct Receiver(mpsc::UnboundedReceiver<CanFrame>);
#[async_trait]
impl CanRx for Receiver {
    async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
        self.0.recv().await.ok_or(CanIoError::Disconnected)
    }
    fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
        Ok(self.0.try_recv().ok())
    }
}
#[async_trait]
impl CanBus for Device {
    async fn send(&self, frame: CanFrame) -> std::result::Result<(), CanIoError> {
        if frame.id() == CanId::Standard(0) {
            let s = match frame.data()[0] {
                1 => 5,
                2 => 4,
                0x80 => 127,
                _ => 0,
            };
            self.emit(CanFrame::new_data(0x71fu16, &[s])?);
            return Ok(());
        }
        assert_eq!(frame.kind(), FrameKind::Data, "SDO must remain Classic CAN");
        let d = frame.data();
        let index = u16::from_le_bytes([d[1], d[2]]);
        let sub = d[3];
        let mut reply = [0; 8];
        reply[1..4].copy_from_slice(&d[1..4]);
        let mut od = self.od.lock().unwrap();
        if d[0] == 0x40 {
            let value = od.get(&(index, sub)).unwrap();
            reply[0] = 0x43 | (((4 - value.len()) as u8) << 2);
            reply[4..4 + value.len()].copy_from_slice(value);
        } else {
            self.writes.lock().unwrap().push((index, sub));
            let mut failure = self.fail_once.lock().unwrap();
            if *failure == Some((index, sub)) {
                *failure = None;
                reply[0] = 0x80;
                reply[4..8].copy_from_slice(&0x06040042u32.to_le_bytes());
            } else {
                if (0x1a00..=0x1a03).contains(&index) {
                    let cob =
                        u32::from_le_bytes(od[&(index - 0x200, 1)].as_slice().try_into().unwrap());
                    assert_ne!(cob & DISABLED, 0, "mapping mutated before disabling PDO");
                    if sub != 0 {
                        assert_eq!(
                            od[&(index, 0)][0],
                            0,
                            "mapping mutated before clearing count"
                        );
                    }
                }
                let size = 4 - ((d[0] >> 2) & 3) as usize;
                od.insert((index, sub), d[4..4 + size].to_vec());
                reply[0] = 0x60;
            }
        }
        drop(od);
        self.emit(CanFrame::new_data(0x59fu16, &reply)?);
        Ok(())
    }
    async fn subscribe(
        &self,
        filter: CanFilter,
    ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
        let (tx, rx) = mpsc::unbounded_channel();
        self.listeners.lock().unwrap().push((filter, tx));
        Ok(Box::new(Receiver(rx)))
    }
    fn capabilities(&self) -> CanCapabilities {
        CanCapabilities {
            fd: true,
            max_dlen: 64,
        }
    }
}

#[tokio::test]
async fn configure_roundtrip_and_mid_write_abort_restores_all_slots() {
    let bus = Device::new();
    let session = Session::new(bus.clone()).await.unwrap();
    session.configure(31, defaults(31, 20)).await.unwrap();
    assert_eq!(session.snapshot().pdos, defaults(31, 20));
    assert_eq!(
        session.snapshot().identity.unwrap().hardware,
        Some(u32::MAX)
    );
    let original = bus.od.lock().unwrap().clone();
    *bus.fail_once.lock().unwrap() = Some((0x1a01, 5));
    let err = session
        .configure(31, defaults(31, 8))
        .await
        .err()
        .unwrap()
        .to_string();
    assert!(err.contains("previous PDO configuration restored"), "{err}");
    assert_eq!(*bus.od.lock().unwrap(), original);
    assert_eq!(session.snapshot().pdos, defaults(31, 20));
    // Foreign identity does not receive IMU-specific writes.
    bus.od
        .lock()
        .unwrap()
        .insert((0x1018, 2), 99u32.to_le_bytes().to_vec());
    bus.writes.lock().unwrap().clear();
    assert!(session.configure(31, defaults(31, 8)).await.is_err());
    assert!(bus.writes.lock().unwrap().is_empty());
    session.stop().await;
}

#[tokio::test]
async fn csv_keeps_separate_stream_times_and_empty_unmapped_columns() {
    let bus = Device::new();
    let session = Session::new(bus).await.unwrap();
    let mut live = session.live.lock().unwrap();
    live.pdos = defaults(31, 8);
    live.receive(CanFrame::new_fd(0x19fu16, &[0; 32], true).unwrap());
    live.receive(CanFrame::new_fd(0x29fu16, &[0; 32], true).unwrap());
    drop(live);
    let csv = session.csv();
    let lines: Vec<_> = csv.lines().collect();
    assert_eq!(lines.len(), 3);
    let header: Vec<_> = lines[0].split(',').collect();
    let q = header
        .iter()
        .position(|s| s.starts_with("3000:01"))
        .unwrap();
    let raw: Vec<_> = lines[1].split(',').collect();
    let chip: Vec<_> = lines[2].split(',').collect();
    assert_eq!(raw[q], "");
    assert_eq!(chip[q], "0");
    assert_eq!(raw[1], "1");
    assert_eq!(chip[1], "2");
    assert_eq!(raw.len(), header.len());
    assert_eq!(chip.len(), header.len());
    let directory = std::env::temp_dir().join(format!(
        "hex-gui-imu-export-{}-{}",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    let path = write_csv_file(&directory, 31, &csv).unwrap();
    assert!(std::path::Path::new(&path).is_absolute());
    assert_eq!(std::fs::read_to_string(&path).unwrap(), csv);
    let another = write_csv_file(&directory, 31, &csv).unwrap();
    assert_ne!(path, another);
    std::fs::remove_dir_all(directory).unwrap();
    session.stop().await;
}

/// Explicit hardware exercise using the same Session as the GUI.
/// HPM_IMU_IFACE=can2 cargo test hpm_imu::tests::hardware_can -- --ignored --nocapture
#[tokio::test]
#[ignore = "requires an attached HPM IMU and configured CAN FD interface"]
async fn hardware_can() -> Result<()> {
    let spec = std::env::var("HPM_IMU_IFACE").context("set HPM_IMU_IFACE")?;
    let node = std::env::var("HPM_IMU_NODE")
        .unwrap_or_else(|_| "31".into())
        .parse::<u8>()?;
    let gate = CanTransportGate::default();
    let (bus, _) = crate::backend::open_bus(
        &spec,
        4_000_000,
        false,
        gate.try_acquire(CanOwner::HpmImu).unwrap(),
    )
    .await?;
    let session = Session::new(bus.clone()).await?;
    tokio::time::sleep(Duration::from_millis(700)).await;
    ensure!(
        session.snapshot().heartbeat.iter().any(|h| h.node == node),
        "No heartbeat"
    );
    session.inspect(node).await?;
    println!(
        "identity: {}",
        serde_json::to_string(&session.snapshot().identity)?
    );
    session.configure(node, defaults(node, 20)).await?;
    tokio::time::sleep(Duration::from_secs(3)).await;
    let slow = session.snapshot();
    for s in &slow.stats[..2] {
        ensure!(
            s.frames > 80 && s.bytes == 32 && s.fd && s.brs,
            "Missing 20ms FD stream"
        );
    }
    // Verify short Classic and maximum 64-byte mappings through the live decoder.
    let mut custom = defaults(node, 20);
    custom[2].cob_id &= !DISABLED;
    custom[2].mapping = vec![0x30050110, 0x30050210, 0x30050310];
    custom[3].cob_id &= !DISABLED;
    custom[3].mapping = vec![0x30070120; 16];
    session.configure(node, custom).await?;
    tokio::time::sleep(Duration::from_secs(2)).await;
    let mapped = session.snapshot();
    ensure!(
        mapped.stats[2].frames > 20 && mapped.stats[2].bytes == 6 && !mapped.stats[2].fd,
        "Short Classic PDO failed"
    );
    ensure!(
        mapped.stats[3].frames > 20 && mapped.stats[3].bytes == 64 && mapped.stats[3].brs,
        "64-byte FD PDO failed"
    );
    // Exercise Stop → Pre-op without requiring SDO availability in Stopped.
    session.nmt(node, 2).await?;
    tokio::time::sleep(Duration::from_millis(700)).await;
    ensure!(
        session
            .snapshot()
            .heartbeat
            .iter()
            .any(|h| h.node == node && h.state == 4),
        "NMT Stop unconfirmed"
    );
    session.nmt(node, 0x80).await?;
    tokio::time::sleep(Duration::from_millis(100)).await;
    session.configure(node, defaults(node, 8)).await?;
    tokio::time::sleep(Duration::from_secs(5)).await;
    let fast = session.snapshot();
    let diagnostics = session.diagnostics(node).await?;
    for s in &fast.stats[..2] {
        ensure!(s.frames > 400 && s.bytes == 32, "Missing default FD stream");
    }
    ensure!(fast.invalid_frames == 0, "Invalid frames");
    ensure!(
        fast.values
            .iter()
            .any(|v| v.index == 0x3004 && v.sub == 4 && v.value == 0.0),
        "Sensor fault is nonzero"
    );
    ensure!(
        fast.values
            .iter()
            .any(|v| v.index == 0x3008 && v.sub == 5 && v.value as u32 & 1 != 0),
        "SFLP is not fresh"
    );
    let csv = session.csv();
    ensure!(csv.lines().count() > 800, "CSV is not full rate");
    let report = serde_json::json!({ "interface": spec, "20ms": slow, "custom": mapped, "8ms": fast,
        "diagnostics": diagnostics, "csv_rows": csv.lines().count() - 1 });
    if let Ok(path) = std::env::var("HPM_IMU_REPORT") {
        std::fs::write(path, serde_json::to_string_pretty(&report)?)?;
    }
    println!(
        "8ms frame rates: {:?}; CSV rows: {}",
        fast.stats.iter().map(|s| s.hz).collect::<Vec<_>>(),
        csv.lines().count() - 1
    );
    session.stop().await;
    drop(bus);
    // Reconnect proves the receiver and the physical bus lease were released.
    let (bus, _) = crate::backend::open_bus(
        &spec,
        4_000_000,
        false,
        gate.try_acquire(CanOwner::HpmImu).unwrap(),
    )
    .await?;
    let session = Session::new(bus).await?;
    session.inspect(node).await?;
    ensure!(
        session.snapshot().pdos == defaults(node, 8),
        "Reconnect configuration mismatch"
    );
    session.stop().await;
    Ok(())
}
