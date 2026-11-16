use super::*;
use anyhow::bail;
use std::sync::atomic::AtomicU32;

fn config() -> Config {
    Config {
        imu_node: 31,
        motor_node: 1,
        host_node: 126,
        angle_deg: 90.0,
        repeats: 2,
        axis: 2,
        direction: 1,
        zero_each_round: true,
        speed_deg_s: 30.0,
        settle_ms: 100,
        samples: 2,
        tolerance_deg: 0.1,
        timeout_s: 2,
    }
}
fn sample(degrees: f64, axis: usize, counter: u32) -> Attitude {
    let a = degrees.to_radians() / 2.0;
    let mut q = [a.cos(), 0.0, 0.0, 0.0];
    q[axis + 1] = a.sin();
    Attitude {
        q,
        counter,
        received: Instant::now(),
    }
}
fn shared(config: &Config) -> Shared {
    Arc::new(Mutex::new(TestSnapshot {
        active: true,
        phase: "initializing".into(),
        completed: 0,
        config: config.clone(),
        motor_model: None,
        motor_position_deg: None,
        imu_angle_deg: None,
        stop_confirmed: false,
        error: None,
        rows: vec![],
    }))
}

#[test]
fn rejects_bad_inputs_before_motion() {
    config().validate().unwrap();
    for angle in [0.0, f64::NAN, f64::INFINITY, 361.0, -0.01] {
        assert!(Config {
            angle_deg: angle,
            ..config()
        }
        .validate()
        .is_err());
    }
    for node in [0, 31, 100, 101, 126, 128] {
        assert!(Config {
            motor_node: node,
            ..config()
        }
        .validate()
        .is_err());
    }
    assert!(Config {
        repeats: 0,
        ..config()
    }
    .validate()
    .is_err());
    assert!(Config {
        speed_deg_s: f64::NAN,
        ..config()
    }
    .validate()
    .is_err());
    assert!(Config {
        samples: 0,
        ..config()
    }
    .validate()
    .is_err());
    assert!(Config {
        axis: 3,
        ..config()
    }
    .validate()
    .is_err());
    assert!(Config {
        direction: 0,
        ..config()
    }
    .validate()
    .is_err());
}

#[test]
fn all_axes_keep_180_overshoot_sign_wrap_and_return_drift() {
    for axis in 0..3 {
        for direction in [-1, 1] {
            let cfg = Config {
                axis,
                direction,
                ..config()
            };
            // A non-zero initial orientation must not contaminate the test angle.
            let mut tracker = AngleTracker::new(sample(47.0, axis, 0), &cfg);
            for (i, angle) in [
                45.0, 90.0, 135.0, 181.8, 270.0, 361.2, 270.0, 180.0, 90.0, 1.3,
            ]
            .into_iter()
            .enumerate()
            {
                let mut s = sample(47.0 + angle * direction as f64, axis, i as u32 + 1);
                // A sensor may flip the quaternion representation between frames.
                if i % 2 == 1 {
                    s.q = s.q.map(|v| -v);
                }
                tracker.update(s).unwrap();
                assert!(
                    (tracker.angle - angle).abs() < 1e-8,
                    "axis {axis}, angle {angle}: {}",
                    tracker.angle
                );
            }
        }
    }
}

#[test]
fn counter_gap_reset_and_rollover_are_distinguished() {
    let cfg = config();
    let mut tracker = AngleTracker::new(sample(0.0, 2, u32::MAX), &cfg);
    assert!(tracker.update(sample(1.0, 2, 0)).unwrap());
    assert!(!tracker.update(sample(9.0, 2, 0)).unwrap());
    assert!((tracker.angle - 1.0).abs() < 1e-8);
    assert!(tracker.update(sample(1.0, 2, u32::MAX)).is_err());
    tracker.received -= Duration::from_millis(300);
    assert!(tracker.update(sample(2.0, 2, 1)).is_err());
}

fn live() -> Live {
    let mut live = Live::default();
    live.pdos = super::super::defaults(31, 8);
    let mut data = [0u8; 32];
    data[..2].copy_from_slice(&10000_i16.to_le_bytes());
    data[20..24].copy_from_slice(&1u32.to_le_bytes());
    data[28] = 1;
    live.receive(can_transport::CanFrame::new_fd(0x29fu16, &data, true).unwrap());
    live
}

#[test]
fn stale_invalid_incoherent_and_frozen_attitudes_are_rejected() {
    assert!(attitude(&live()).is_ok());
    let mut s = live();
    s.values.get_mut(&(0x3008, 1)).unwrap().changed -= Duration::from_secs(1);
    assert!(attitude(&s).is_err());
    let mut s = live();
    s.values.get_mut(&(0x3008, 5)).unwrap().value = 0.0;
    assert!(attitude(&s).is_err());
    let mut s = live();
    s.values.get_mut(&(0x3008, 2)).unwrap().value = 250.0;
    assert!(attitude(&s).is_err());
    let mut s = live();
    s.values.get_mut(&(0x3000, 1)).unwrap().received -= Duration::from_millis(1);
    assert!(attitude(&s).is_err());
    let mut s = live();
    s.values.get_mut(&(0x3000, 1)).unwrap().value = 0.0;
    assert!(attitude(&s).is_err());
}

struct Motion {
    position: f64,
    target: f64,
    drift: f64,
    commands: Vec<f64>,
}
struct FakeRig {
    model: &'static str,
    motion: Mutex<Motion>,
    count: AtomicU32,
    disabled: AtomicBool,
    fail_command: bool,
    stale: bool,
    stuck: bool,
    fail_disable: bool,
}
impl FakeRig {
    fn new(model: &'static str) -> Self {
        Self {
            model,
            motion: Mutex::new(Motion {
                position: 15.0,
                target: 15.0,
                drift: 0.0,
                commands: vec![],
            }),
            count: AtomicU32::new(0),
            disabled: AtomicBool::new(false),
            fail_command: false,
            stale: false,
            stuck: false,
            fail_disable: false,
        }
    }
}
#[async_trait]
impl Rig for FakeRig {
    async fn initialize(&self, _: &Config) -> Result<String> {
        Ok(self.model.into())
    }
    async fn command(&self, degrees: f64) -> Result<()> {
        let mut m = self.motion.lock().unwrap();
        m.commands.push(degrees);
        if self.fail_command && degrees != 0.0 {
            bail!("simulated SDO failure");
        }
        if degrees == 0.0 && m.target != 0.0 && m.commands.len() > 1 {
            m.drift += 1.0;
        }
        m.target = degrees;
        Ok(())
    }
    fn feedback(&self) -> Result<Feedback> {
        let mut m = self.motion.lock().unwrap();
        if !self.stuck {
            m.position += (m.target - m.position).clamp(-30.0, 30.0);
        }
        if self.stale && m.target != 0.0 {
            bail!("Motor feedback is stale");
        }
        Ok(Feedback {
            position: m.position,
            velocity: if m.position == m.target { 0.0 } else { 30.0 },
            received: Instant::now(),
        })
    }
    fn attitude(&self) -> Result<Attitude> {
        let m = self.motion.lock().unwrap();
        Ok(sample(
            m.position * 1.01 + m.drift,
            2,
            self.count.fetch_add(1, Ordering::Relaxed),
        ))
    }
    async fn disable(&self) -> Result<()> {
        self.disabled.store(true, Ordering::Release);
        ensure!(!self.fail_disable, "Motor disable unconfirmed");
        Ok(())
    }
}

#[tokio::test]
async fn cycles_and_zero_switch_preserve_measurable_drift() {
    for model in ["hexmeow 4310", "hexmeow 4342"] {
        for zero_each_round in [false, true] {
            let cfg = Config {
                zero_each_round,
                ..config()
            };
            let rig = FakeRig::new(model);
            let s = shared(&cfg);
            let (_tx, rx) = watch::channel(false);
            drive(&rig, &cfg, &s, rx).await;
            let result = s.lock().unwrap();
            assert_eq!(result.phase, "completed", "{:?}", result.error);
            assert_eq!(result.completed, 2);
            assert_eq!(result.motor_model.as_deref(), Some(model));
            assert!(result.stop_confirmed && rig.disabled.load(Ordering::Acquire));
            let rows = &result.rows;
            assert!((rows[0].raw_deg - 90.9).abs() < 1e-8);
            assert!((rows[0].absolute_error_percent - 1.0).abs() < 1e-8);
            assert!((rows[0].return_raw_deg.unwrap() - 1.0).abs() < 1e-8);
            assert!((rows[1].raw_deg - if zero_each_round { 90.9 } else { 91.9 }).abs() < 1e-8);
            assert_eq!(
                rig.motion.lock().unwrap().commands,
                vec![0.0, 90.0, 0.0, 90.0, 0.0]
            );
        }
    }
}

// An expedited SDO server plus actual TPDOs exercises the production driver,
// including exact identities, mode confirmation, Q8.24 targets and disable.
mod wire {
    use super::*;
    use can_transport::{CanBus, CanCapabilities, CanFilter, CanFrame, CanId, CanIoError, CanRx};
    use hex_motor::meow_motor::{PRODUCT_CODE_4310, PRODUCT_CODE_4342, VENDOR_ID};
    use std::collections::BTreeMap;
    use tokio::sync::mpsc;

    struct Bus {
        od: Mutex<BTreeMap<(u16, u8), Vec<u8>>>,
        listeners: Mutex<Vec<(CanFilter, mpsc::UnboundedSender<CanFrame>)>>,
        nmt: AtomicU32,
        writes: Mutex<Vec<(u16, Vec<u8>)>>,
    }
    impl Bus {
        fn new(product: u32) -> Arc<Self> {
            let mut od = BTreeMap::new();
            for (sub, value) in [(1, VENDOR_ID), (2, product), (3, 1), (4, 42)] {
                od.insert((0x1018, sub), value.to_le_bytes().to_vec());
            }
            od.insert((0x1008, 0), b"Meow".to_vec());
            od.insert((0x2001, 0), vec![4]);
            od.insert((0x2001, 1), vec![1]);
            od.insert((0x2001, 2), 1_000_000u32.to_le_bytes().to_vec());
            od.insert((0x2001, 3), 4_000_000u32.to_le_bytes().to_vec());
            od.insert((0x2001, 4), vec![1]);
            od.insert((0x4402, 0), vec![0]);
            od.insert((0x453f, 0), vec![0, 0]);
            od.insert((0x4576, 0), 10f32.to_le_bytes().to_vec());
            od.insert((0x4102, 7), 1f32.to_le_bytes().to_vec());
            od.insert((0x457a, 0), vec![0; 4]);
            od.insert((0x1800, 1), 0x181u32.to_le_bytes().to_vec());
            od.insert((0x1801, 1), 0x281u32.to_le_bytes().to_vec());
            Arc::new(Self {
                od: Mutex::new(od),
                listeners: Mutex::new(vec![]),
                nmt: AtomicU32::new(127),
                writes: Mutex::new(vec![]),
            })
        }
        fn emit(&self, frame: CanFrame) {
            self.listeners.lock().unwrap().retain(|(filter, tx)| {
                !tx.is_closed() && (!filter.matches(&frame) || tx.send(frame).is_ok())
            });
        }
        fn feedback(&self, timestamp: u16) {
            let nmt = self.nmt.load(Ordering::Acquire) as u8;
            self.emit(CanFrame::new_data(0x701u16, &[nmt]).unwrap());
            if nmt != 5 {
                return;
            }
            let od = self.od.lock().unwrap();
            let mut p1 = [0u8; 8];
            p1[..4].copy_from_slice(&od[&(0x457a, 0)]);
            p1[4..6].copy_from_slice(&timestamp.to_le_bytes());
            let mut p2 = [0u8; 7];
            p2[0] = od[&(0x4402, 0)][0];
            drop(od);
            self.emit(CanFrame::new_fd(0x181u16, &p1, true).unwrap());
            self.emit(CanFrame::new_fd(0x281u16, &p2, true).unwrap());
        }
    }
    struct Rx(mpsc::UnboundedReceiver<CanFrame>);
    #[async_trait]
    impl CanRx for Rx {
        async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
            self.0.recv().await.ok_or(CanIoError::Disconnected)
        }
        fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
            Ok(self.0.try_recv().ok())
        }
    }
    #[async_trait]
    impl CanBus for Bus {
        async fn send(&self, frame: CanFrame) -> std::result::Result<(), CanIoError> {
            if frame.id() == CanId::Standard(0) {
                assert_eq!(
                    frame.data()[1],
                    1,
                    "NMT must address only the selected motor"
                );
                let state = match frame.data()[0] {
                    1 => 5,
                    0x80 => 127,
                    2 => 4,
                    _ => panic!("Unexpected NMT"),
                };
                self.nmt.store(state, Ordering::Release);
                self.emit(CanFrame::new_data(0x701u16, &[state as u8])?);
                return Ok(());
            }
            if frame.id() != CanId::Standard(0x601) {
                return Ok(());
            }
            let d = frame.data();
            let index = u16::from_le_bytes([d[1], d[2]]);
            let sub = d[3];
            let mut reply = [0u8; 8];
            reply[1..4].copy_from_slice(&d[1..4]);
            let mut od = self.od.lock().unwrap();
            if d[0] == 0x40 {
                if let Some(value) = od.get(&(index, sub)) {
                    reply[0] = 0x43 | ((4 - value.len()) as u8) << 2;
                    reply[4..4 + value.len()].copy_from_slice(value);
                } else {
                    reply[0] = 0x80;
                    reply[4..8].copy_from_slice(&0x06020000u32.to_le_bytes());
                }
            } else {
                let size = 4 - ((d[0] >> 2) & 3) as usize;
                let value = d[4..4 + size].to_vec();
                self.writes.lock().unwrap().push((index, value.clone()));
                if index == 0x4401 {
                    od.insert((0x4402, 0), value.clone());
                }
                od.insert((index, sub), value);
                reply[0] = 0x60;
            }
            drop(od);
            self.emit(CanFrame::new_data(0x581u16, &reply)?);
            Ok(())
        }
        async fn subscribe(
            &self,
            filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            self.listeners.lock().unwrap().push((filter, tx));
            Ok(Box::new(Rx(rx)))
        }
        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    #[tokio::test]
    async fn production_adapter_initializes_and_positions_4310_and_4342_in_output_shaft_units() {
        for (product, name) in [
            (PRODUCT_CODE_4310, "hexmeow 4310"),
            (PRODUCT_CODE_4342, "hexmeow 4342"),
        ] {
            let bus = Bus::new(product);
            let sender = bus.clone();
            let stream = tokio::spawn(async move {
                let mut time = 0u16;
                loop {
                    sender.feedback(time);
                    time = time.wrapping_add(10_000);
                    tokio::time::sleep(TICK).await;
                }
            });
            let rig = HardwareRig {
                manager: MeowMotorManager::new(
                    bus.clone(),
                    MeowMotorManagerOptions {
                        heartbeat_node_id: 126,
                        auto_identify: false,
                        ..Default::default()
                    },
                )
                .unwrap(),
                live: Arc::new(Mutex::new(live())),
                node: 1,
                needs_disable: AtomicBool::new(false),
            };
            assert_eq!(rig.initialize(&config()).await.unwrap(), name);
            for angle in [0.0, 90.0, 180.0, -180.0, 0.0] {
                rig.command(angle).await.unwrap();
                tokio::time::sleep(Duration::from_millis(35)).await;
                assert!((rig.feedback().unwrap().position - angle).abs() < 1e-4);
            }
            rig.disable().await.unwrap();
            assert!(!rig.needs_disable.load(Ordering::Acquire));
            let writes = bus.writes.lock().unwrap();
            let targets: Vec<i32> = writes
                .iter()
                .filter(|(index, _)| *index == 0x457a)
                .map(|(_, bytes)| i32::from_le_bytes(bytes.as_slice().try_into().unwrap()))
                .collect();
            assert_eq!(targets, [0, 0x00400000, 0x00800000, -0x00800000, 0]);
            assert_eq!(writes.last().unwrap(), &(0x4401, vec![0]));
            // Motor zero is a destination, never an encoder preset or save.
            assert!(!writes
                .iter()
                .any(|(index, _)| [0x3001, 0x1010].contains(index)));
            stream.abort();
        }
    }
}

#[tokio::test]
async fn negative_180_measures_overshoot_without_wrapping_the_error() {
    let cfg = Config {
        angle_deg: -180.0,
        repeats: 1,
        ..config()
    };
    let rig = FakeRig::new("hexmeow 4342");
    let s = shared(&cfg);
    let (_tx, rx) = watch::channel(false);
    drive(&rig, &cfg, &s, rx).await;
    let s = s.lock().unwrap();
    assert_eq!(s.phase, "completed", "{:?}", s.error);
    assert!((s.rows[0].raw_deg + 181.8).abs() < 1e-8);
    assert!((s.rows[0].error_percent + 1.0).abs() < 1e-8);
}

#[tokio::test]
async fn cancellation_and_failures_always_disable_and_do_not_invent_results() {
    for kind in ["cancel", "sdo", "stale", "timeout", "disable"] {
        let cfg = Config {
            timeout_s: 0,
            ..config()
        };
        let cfg = if kind == "timeout" { cfg } else { config() };
        let mut rig = FakeRig::new("hexmeow 4310");
        rig.fail_command = kind == "sdo";
        rig.stale = kind == "stale";
        rig.stuck = kind == "timeout";
        rig.fail_disable = kind == "disable";
        let s = shared(&cfg);
        let (_tx, rx) = watch::channel(kind == "cancel" || kind == "disable");
        drive(&rig, &cfg, &s, rx).await;
        let result = s.lock().unwrap();
        assert!(!result.active);
        assert_eq!(
            result.phase,
            if kind == "cancel" {
                "cancelled"
            } else {
                "failed"
            }
        );
        assert!(result.rows.is_empty());
        assert_eq!(result.stop_confirmed, kind != "disable");
        assert!(rig.disabled.load(Ordering::Acquire));
    }
}

#[tokio::test]
async fn stop_during_motion_ends_the_sequence_before_return_command() {
    let cfg = config();
    let rig = FakeRig::new("hexmeow 4342");
    let s = shared(&cfg);
    let (tx, rx) = watch::channel(false);
    tokio::join!(drive(&rig, &cfg, &s, rx), async {
        loop {
            if rig.motion.lock().unwrap().commands.len() == 2 {
                break;
            }
            tokio::time::sleep(TICK).await;
        }
        tx.send(true).unwrap();
    });
    assert_eq!(s.lock().unwrap().phase, "cancelled");
    assert_eq!(rig.motion.lock().unwrap().commands, vec![0.0, 90.0]);
    assert!(rig.disabled.load(Ordering::Acquire));
}
