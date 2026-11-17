//! Exercise HopeA3 through the real HexMeow SDK, with three simulated CANopen
//! devices answering SDO/NMT and publishing the new TPDO1/2 wire layouts.
use super::*;
use std::collections::HashMap;

use async_trait::async_trait;
use can_transport::{CanCapabilities, CanFilter, CanIoError, CanRx};
use hex_motor::meow_motor::MeowMotorManagerOptions;
use tokio::sync::mpsc;

type ObjectKey = (u8, u16, u8);

struct WireState {
    objects: HashMap<ObjectKey, Vec<u8>>,
    nmt: [u8; 3],
    positions: [f64; 3],
    timestamp: u16,
    sent: Vec<CanFrame>,
    refuse_disable: Option<u8>,
    fail_calibration: Option<u8>,
    silence_tpdo: Option<u8>,
    fail_shared_send: bool,
}

impl Default for WireState {
    fn default() -> Self {
        let mut objects = HashMap::new();
        for nid in NODE_IDS {
            let entries = [
                (0x1018, 0, vec![4]),
                (0x1018, 1, 0x0068_6578u32.to_le_bytes().to_vec()),
                (0x1018, 2, 0x6C64_BC78u32.to_le_bytes().to_vec()),
                (0x1018, 3, 1u32.to_le_bytes().to_vec()),
                (0x1018, 4, u32::from(nid).to_le_bytes().to_vec()),
                (0x2001, 0, vec![4]),
                (0x2001, 1, vec![nid]),
                (0x2001, 2, 1_000_000u32.to_le_bytes().to_vec()),
                (0x2001, 3, 5_000_000u32.to_le_bytes().to_vec()),
                (0x2001, 4, vec![1]),
                (0x4402, 0, vec![0]),
                (0x453F, 0, 0u16.to_le_bytes().to_vec()),
                (0x4576, 0, 20f32.to_le_bytes().to_vec()),
                (0x4102, 7, 0.001f32.to_le_bytes().to_vec()),
                // Simulate leftovers from a previous compressed MIT application.
                (0x4103, 1, vec![1]),
                (0x4102, 1, 9f32.to_le_bytes().to_vec()),
                (0x4102, 2, 9f32.to_le_bytes().to_vec()),
                (0x4102, 3, 9f32.to_le_bytes().to_vec()),
                (0x4102, 4, 1000u16.to_le_bytes().to_vec()),
                (0x4102, 5, 1000u16.to_le_bytes().to_vec()),
                (0x4102, 6, 0u16.to_le_bytes().to_vec()),
                (0x4572, 0, 800u16.to_le_bytes().to_vec()),
                (
                    0x1800,
                    1,
                    u32::from(0x180 + u16::from(nid)).to_le_bytes().to_vec(),
                ),
                (
                    0x1801,
                    1,
                    u32::from(0x280 + u16::from(nid)).to_le_bytes().to_vec(),
                ),
            ];
            for (index, subindex, value) in entries {
                objects.insert((nid, index, subindex), value);
            }
        }
        Self {
            objects,
            nmt: [127; 3],
            positions: [0.0; 3],
            timestamp: 0,
            sent: Vec::new(),
            refuse_disable: None,
            fail_calibration: None,
            silence_tpdo: None,
            fail_shared_send: false,
        }
    }
}

#[derive(Default)]
struct SimBus {
    wire: StdMutex<WireState>,
    subscribers: StdMutex<Vec<(CanFilter, mpsc::UnboundedSender<CanFrame>)>>,
}

impl SimBus {
    fn emit(&self, frame: CanFrame) {
        self.subscribers.lock().unwrap().retain(|(filter, tx)| {
            !tx.is_closed() && (!filter.matches(&frame) || tx.send(frame.clone()).is_ok())
        });
    }

    fn publish(&self) {
        let mut wire = self.wire.lock().unwrap();
        wire.timestamp = wire.timestamp.wrapping_add(2000);
        for (slot, nid) in NODE_IDS.into_iter().enumerate() {
            self.emit(CanFrame::new_data(0x700 + u16::from(nid), &[wire.nmt[slot]]).unwrap());
            if wire.nmt[slot] != 5 || wire.silence_tpdo == Some(nid) {
                continue;
            }
            let mode = wire.objects[&(nid, 0x4402, 0)][0];
            let max = u16::from_le_bytes(wire.objects[&(nid, 0x4572, 0)][..].try_into().unwrap());
            let velocity = if mode == 4 && max > 0 {
                f32::from_le_bytes(wire.objects[&(nid, 0x4102, 2)][..].try_into().unwrap()) as f64
            } else {
                0.0
            };
            wire.positions[slot] += velocity * 0.002;
            let mut tpdo1 = [0u8; 8];
            tpdo1[..4]
                .copy_from_slice(&((wire.positions[slot] * 16_777_216.0) as i32).to_le_bytes());
            tpdo1[4..6].copy_from_slice(&wire.timestamp.to_le_bytes());
            tpdo1[6..].copy_from_slice(&300i16.to_le_bytes());
            self.emit(CanFrame::new_fd(0x180 + u16::from(nid), &tpdo1, true).unwrap());
            self.emit(
                CanFrame::new_fd(0x280 + u16::from(nid), &[mode, 0, 0, 250, 0, 250, 0], true)
                    .unwrap(),
            );
        }
    }

    fn object(&self, nid: u8, index: u16, sub: u8) -> Vec<u8> {
        self.wire.lock().unwrap().objects[&(nid, index, sub)].clone()
    }

    fn last_shared(&self) -> CanFrame {
        self.wire
            .lock()
            .unwrap()
            .sent
            .iter()
            .rev()
            .find(|frame| frame.id().raw() == 0x18A)
            .unwrap()
            .clone()
    }
}

struct SimRx(mpsc::UnboundedReceiver<CanFrame>);

#[async_trait]
impl CanRx for SimRx {
    async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
        self.0.recv().await.ok_or(CanIoError::Disconnected)
    }

    fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
        match self.0.try_recv() {
            Ok(frame) => Ok(Some(frame)),
            Err(mpsc::error::TryRecvError::Empty) => Ok(None),
            Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
        }
    }
}

#[async_trait]
impl CanBus for SimBus {
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
        let mut wire = self.wire.lock().unwrap();
        let id = frame.id().raw();
        let data = frame.data();
        wire.sent.push(frame.clone());
        if id == 0 {
            if let Some(slot) = NODE_IDS.iter().position(|nid| *nid == data[1]) {
                wire.nmt[slot] = match data[0] {
                    1 => 5,
                    128 => 127,
                    _ => wire.nmt[slot],
                };
            }
        } else if id == 0x18A {
            if wire.fail_shared_send {
                return Err(CanIoError::Disconnected);
            }
            for nid in NODE_IDS {
                let count = wire
                    .objects
                    .get(&(nid, 0x1600, 0))
                    .map_or(0, |value| value[0]);
                let mut offset = 0;
                for sub in 1..=count {
                    let packed = u32::from_le_bytes(
                        wire.objects[&(nid, 0x1600, sub)][..].try_into().unwrap(),
                    );
                    let (index, sub, len) = (
                        (packed >> 16) as u16,
                        (packed >> 8) as u8,
                        (packed & 255) as usize / 8,
                    );
                    if index != 0x3000 {
                        wire.objects
                            .insert((nid, index, sub), data[offset..offset + len].to_vec());
                    }
                    offset += len;
                }
            }
        } else if (0x601..=0x67F).contains(&id) {
            let nid = (id - 0x600) as u8;
            let index = u16::from_le_bytes([data[1], data[2]]);
            let sub = data[3];
            let mut reply = [0u8; 8];
            reply[1..4].copy_from_slice(&data[1..4]);
            let abort = if data[0] == 0x40 {
                if index == 0x4001 && wire.fail_calibration == Some(nid) {
                    Some(0x0504_0000u32)
                } else if let Some(value) = wire.objects.get(&(nid, index, sub)) {
                    reply[0] = 0x43 | ((4 - value.len()) as u8) << 2;
                    reply[4..4 + value.len()].copy_from_slice(value);
                    None
                } else {
                    Some(0x0602_0000)
                }
            } else if matches!(data[0], 0x23 | 0x2B | 0x2F) {
                let len = 4 - ((data[0] >> 2) & 3) as usize;
                let value = data[4..4 + len].to_vec();
                if index == 0x4401 && value == [0] && wire.refuse_disable == Some(nid) {
                    Some(0x0800_0020)
                } else {
                    if index == 0x4401 && value == [4] {
                        // Enabling must never expose old velocity/torque/gain values.
                        assert_eq!(wire.objects[&(nid, 0x4103, 1)], [0]);
                        for sub in [1, 2, 3] {
                            assert_eq!(wire.objects[&(nid, 0x4102, sub)], 0f32.to_le_bytes());
                        }
                        for sub in [4, 5] {
                            assert_eq!(wire.objects[&(nid, 0x4102, sub)], [0, 0]);
                        }
                        assert_eq!(wire.objects[&(nid, 0x4572, 0)], [0, 0]);
                    }
                    if index == 0x4401 {
                        wire.objects.insert(
                            (nid, 0x4402, 0),
                            vec![if value[0] == 255 { 0 } else { value[0] }],
                        );
                    }
                    wire.objects.insert((nid, index, sub), value);
                    reply[0] = 0x60;
                    None
                }
            } else {
                Some(0x0504_0001)
            };
            if let Some(code) = abort {
                reply[0] = 0x80;
                reply[4..].copy_from_slice(&code.to_le_bytes());
            }
            self.emit(CanFrame::new_data(0x580 + u16::from(nid), &reply).unwrap());
        }
        Ok(())
    }

    async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
        let (tx, rx) = mpsc::unbounded_channel();
        self.subscribers.lock().unwrap().push((filter, tx));
        Ok(Box::new(SimRx(rx)))
    }

    fn capabilities(&self) -> CanCapabilities {
        CanCapabilities {
            fd: true,
            max_dlen: 64,
        }
    }
}

struct Harness {
    bus: Arc<SimBus>,
    mgr: Arc<MeowMotorManager>,
    publisher: JoinHandle<()>,
}

impl Harness {
    async fn new() -> Self {
        let bus = Arc::new(SimBus::default());
        let mgr = Arc::new(
            MeowMotorManager::new(
                bus.clone(),
                MeowMotorManagerOptions {
                    auto_identify: false,
                    broadcast_heartbeat: false,
                    sdo_timeout: Duration::from_millis(100),
                    nmt_confirm_timeout: Duration::from_millis(500),
                    mode_confirm_timeout: Duration::from_millis(500),
                    ..Default::default()
                },
            )
            .unwrap(),
        );
        let publisher = {
            let bus = bus.clone();
            tokio::spawn(async move {
                let mut tick = tokio::time::interval(Duration::from_millis(2));
                loop {
                    tick.tick().await;
                    bus.publish();
                }
            })
        };
        wait_for(|| mgr.list().iter().filter(|info| info.online).count() == 3).await;
        Self {
            bus,
            mgr,
            publisher,
        }
    }

    async fn start(&self) -> anyhow::Result<Hopea3> {
        Hopea3::start(
            self.mgr.clone(),
            self.bus.clone(),
            Some(Duration::from_millis(100)),
            &StdMutex::new(InitProgress::default()),
            &AtomicBool::new(false),
        )
        .await
    }
}

impl Drop for Harness {
    fn drop(&mut self) {
        self.publisher.abort();
    }
}

async fn wait_for(predicate: impl Fn() -> bool) {
    tokio::time::timeout(Duration::from_secs(3), async {
        while !predicate() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .expect("simulated motor state did not settle");
}

fn assert_disabled(bus: &SimBus) {
    for nid in NODE_IDS {
        assert_eq!(bus.object(nid, 0x4402, 0), [0], "node {nid}");
    }
}

#[tokio::test]
async fn starts_streams_new_pdos_and_confirms_all_three_disabled() {
    let h = Harness::new().await;
    // Real SDK CRC decoding, with a synthetic v1 record for torque_factor=2.
    // Nodes 22/23 stay explicitly uncalibrated, so their factor must be 1.
    let identity = hex_motor::types::MotorIdentity {
        node_id: 21,
        vendor_id: 0x0068_6578,
        product_code: 0x6C64_BC78,
        revision_number: 1,
        serial_number: 21,
        product_name: None,
    };
    let mut words = [0x0106_0000, 1, 0, 0x1CCD_4000, 0, 0, 0];
    words[0] |= u32::from(hex_motor::meow_motor::calibration::factory_crc16(
        &identity, &words,
    ));
    {
        let mut wire = h.bus.wire.lock().unwrap();
        wire.objects.insert((21, 0x4001, 0), vec![7]);
        for (offset, word) in words.into_iter().enumerate() {
            wire.objects
                .insert((21, 0x4001, offset as u8 + 1), word.to_le_bytes().to_vec());
        }
    }
    let mut app = h.start().await.unwrap_or_else(|error| panic!("{error:#}"));
    wait_for(|| app.state().motors.len() == 3 && app.state().fault.is_none()).await;
    for nid in NODE_IDS {
        assert_eq!(h.bus.object(nid, 0x4103, 1), [0]);
        assert_eq!(h.bus.object(nid, 0x4102, 3), 0f32.to_le_bytes());
        assert_eq!(h.bus.object(nid, 0x4102, 4), 0u16.to_le_bytes());
        assert_eq!(h.bus.object(nid, 0x4102, 6), 1000u16.to_le_bytes());
        assert_eq!(h.bus.object(nid, 0x1800, 5), 2u16.to_le_bytes());
    }
    app.set_accel_limits(0.0, 0.0);
    app.set_cmd(0.5, -0.2, 0.3);
    wait_for(|| (app.state().meas_vx - 0.5).abs() < 0.01).await;
    let targets = Kinematics::new().twist_to_motor_rev_s(0.5, -0.2, 0.3);
    for (slot, nid) in NODE_IDS.into_iter().enumerate() {
        assert_eq!(
            h.bus.object(nid, 0x4102, 2),
            (targets[slot] as f32).to_le_bytes()
        );
        assert_eq!(h.bus.object(nid, 0x4102, 5), 6283u16.to_le_bytes());
        assert_eq!(h.bus.object(nid, 0x4572, 0), 800u16.to_le_bytes());
        assert_eq!(
            app.state().motors[slot].torque_nm,
            Some(if nid == 21 { 3.0 } else { 6.0 })
        );
    }
    app.stop().await.unwrap();
    assert_disabled(&h.bus);
    assert_eq!(h.bus.last_shared().data(), &[0; 24]);
    assert!(!app.state().running);
    for frame in &h.bus.wire.lock().unwrap().sent {
        if (0x601..=0x67F).contains(&frame.id().raw()) {
            let index = u16::from_le_bytes([frame.data()[1], frame.data()[2]]);
            assert!(
                !matches!(index, 0x6040 | 0x6041 | 0x6060 | 0x6061 | 0x6072 | 0x2003),
                "legacy OD {index:04X}"
            );
        }
    }
}

#[tokio::test]
async fn legacy_identity_is_rejected_and_preceding_motor_is_rolled_back() {
    let h = Harness::new().await;
    h.bus
        .wire
        .lock()
        .unwrap()
        .objects
        .insert((22, 0x1018, 1), 0x4859_444Cu32.to_le_bytes().to_vec());
    let error = h
        .start()
        .await
        .err()
        .expect("legacy firmware must be rejected");
    assert!(format!("{error:#}").contains("0x1018"));
    assert_disabled(&h.bus);
    assert_eq!(h.bus.object(21, 0x4572, 0), [0, 0]);
    assert!(!h
        .bus
        .wire
        .lock()
        .unwrap()
        .objects
        .contains_key(&(22, 0x4401, 0)));
}

#[tokio::test]
async fn unreadable_factory_calibration_does_not_fall_back_to_one() {
    let h = Harness::new().await;
    h.bus.wire.lock().unwrap().fail_calibration = Some(22);
    let error = h
        .start()
        .await
        .err()
        .expect("failed calibration must prevent start");
    assert!(format!("{error:#}").contains("factory torque factor unavailable"));
    assert_disabled(&h.bus);
}

#[tokio::test]
async fn stop_failure_keeps_the_session_retryable_and_still_disables_other_nodes() {
    let h = Harness::new().await;
    let mut app = h.start().await.unwrap();
    h.bus.wire.lock().unwrap().refuse_disable = Some(22);
    assert!(app.stop().await.is_err());
    assert!(!app.state().running);
    assert!(app.state().fault.unwrap().contains("0x16"));
    assert_eq!(h.bus.object(21, 0x4402, 0), [0]);
    assert_eq!(h.bus.object(23, 0x4402, 0), [0]);
    assert_eq!(h.bus.object(22, 0x4572, 0), [0, 0]);
    h.bus.wire.lock().unwrap().refuse_disable = None;
    app.stop().await.unwrap();
    assert_disabled(&h.bus);
    assert!(app.state().fault.is_none());
}

#[tokio::test]
async fn stale_motor_inhibits_all_wheels_and_reinit_refreshes_scaling_without_old_motion() {
    let h = Harness::new().await;
    let mut app = h.start().await.unwrap();
    wait_for(|| app.state().motors.len() == 3 && app.state().fault.is_none()).await;
    app.set_accel_limits(0.0, 0.0);
    app.set_cmd(0.5, 0.0, 0.0);
    wait_for(|| app.state().cmd_vx > 0.0).await;
    h.bus.wire.lock().unwrap().silence_tpdo = Some(22);
    wait_for(|| app.state().fault.is_some()).await;
    assert_eq!(h.bus.last_shared().data(), &[0; 24]);
    assert_eq!(app.state().cmd_vx, 0.0);
    {
        let mut wire = h.bus.wire.lock().unwrap();
        wire.silence_tpdo = None;
        wire.objects
            .insert((22, 0x4102, 7), 0.002f32.to_le_bytes().to_vec());
    }
    app.reinit_motor(22, &AtomicBool::new(false)).await.unwrap();
    wait_for(|| app.state().fault.is_none()).await;
    assert_eq!(app.state().cmd_vx, 0.0);
    assert_eq!(h.bus.object(22, 0x4102, 5), 3142u16.to_le_bytes());
    app.stop().await.unwrap();
}

#[tokio::test]
async fn shared_send_failure_stays_inhibited_after_transport_recovers() {
    let h = Harness::new().await;
    let mut app = h.start().await.unwrap();
    h.bus.wire.lock().unwrap().fail_shared_send = true;
    wait_for(|| {
        app.state()
            .fault
            .as_ref()
            .is_some_and(|fault| fault.contains("CAN send failed"))
    })
    .await;
    h.bus.wire.lock().unwrap().fail_shared_send = false;
    app.set_cmd(1.0, 0.0, 0.0);
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert_eq!(h.bus.last_shared().data(), &[0; 24]);
    assert_eq!(app.state().cmd_vx, 0.0);
    app.stop().await.unwrap();
}

#[tokio::test]
async fn follow_deadline_stops_all_wheels_and_rejects_late_commands() {
    let h = Harness::new().await;
    let mut app = h.start().await.unwrap();
    wait_for(|| app.state().motors.len() == 3 && app.state().fault.is_none()).await;
    app.set_accel_limits(0.0, 0.0);
    app.set_follow_cmd(0.4, 0.0, -0.2);
    wait_for(|| app.state().cmd_vx > 0.0).await;
    wait_for(|| {
        app.state()
            .fault
            .as_ref()
            .is_some_and(|fault| fault.contains("Follow controller expired"))
    })
    .await;
    assert_eq!(h.bus.last_shared().data(), &[0; 24]);
    app.set_follow_cmd(0.5, 0.0, 0.2);
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert_eq!(h.bus.last_shared().data(), &[0; 24]);
    assert_eq!(app.state().cmd_vx, 0.0);
    app.stop().await.unwrap();
    assert_disabled(&h.bus);
}
