//! Session regression tests against a minimal standard-MIT-only drive.
//! This mock intentionally cannot answer UID, dictionary or native frames,
//! and reports raw status zero even while its power stage is enabled.
use super::*;
use can_transport::CanCapabilities;
use std::sync::atomic::{AtomicU16, AtomicU8};
use tokio::sync::broadcast;

const STOP: [u8; 8] = [255, 255, 255, 255, 255, 255, 255, 253];
const ENABLE: [u8; 8] = [255, 255, 255, 255, 255, 255, 255, 252];
const ZERO: [u8; 8] = [255, 255, 255, 255, 255, 255, 255, 254];

struct MitOnlyBus {
    tx: broadcast::Sender<CanFrame>,
    sent: StdMutex<Vec<CanFrame>>,
    physical_enabled: AtomicBool,
    motion_replies: AtomicBool,
    stop_replies: AtomicBool,
    reply_id: AtomicU16,
    flags: AtomicU8,
    position: StdMutex<f64>,
    zero_due: StdMutex<Option<Instant>>,
    zero_applies: AtomicBool,
}

impl MitOnlyBus {
    fn new(reply_id: u16) -> Arc<Self> {
        let (tx, _) = broadcast::channel(1024);
        Arc::new(Self {
            tx,
            sent: StdMutex::new(Vec::new()),
            physical_enabled: AtomicBool::new(false),
            motion_replies: AtomicBool::new(true),
            stop_replies: AtomicBool::new(true),
            reply_id: AtomicU16::new(reply_id),
            flags: AtomicU8::new(0),
            position: StdMutex::new(-2.9333),
            zero_due: StdMutex::new(None),
            zero_applies: AtomicBool::new(true),
        })
    }

    fn telemetry(&self) -> CanFrame {
        // Independent wire fixture: p16, v12=2048, tau12=2048, 30.7 C.
        let p = (((*self.position.lock().unwrap() + 12.57) / 25.14) * 65535.0).round() as u16;
        CanFrame::new_data(
            CanId::Standard(self.reply_id.load(Ordering::Acquire)),
            &[
                1,
                (p >> 8) as u8,
                p as u8,
                0x80,
                0x08,
                0,
                self.flags.load(Ordering::Acquire) | 1,
                0x33,
            ],
        )
        .unwrap()
    }

    fn feedback(&self) {
        let _ = self.tx.send(self.telemetry());
    }

    fn motion_count(&self) -> usize {
        self.sent
            .lock()
            .unwrap()
            .iter()
            .filter(|frame| {
                frame.id() == CanId::Standard(1)
                    && frame.data() != STOP
                    && frame.data() != ENABLE
                    && frame.data() != ZERO
            })
            .count()
    }

    fn stop_count(&self) -> usize {
        self.sent
            .lock()
            .unwrap()
            .iter()
            .filter(|frame| frame.id() == CanId::Standard(1) && frame.data() == STOP)
            .count()
    }
}

struct MitRx(broadcast::Receiver<CanFrame>);
#[async_trait::async_trait]
impl CanRx for MitRx {
    async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
        self.0.recv().await.map_err(|error| match error {
            broadcast::error::RecvError::Lagged(dropped) => CanIoError::Lagged { dropped },
            broadcast::error::RecvError::Closed => CanIoError::Disconnected,
        })
    }
    fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
        match self.0.try_recv() {
            Ok(frame) => Ok(Some(frame)),
            Err(broadcast::error::TryRecvError::Empty) => Ok(None),
            Err(broadcast::error::TryRecvError::Lagged(dropped)) => {
                Err(CanIoError::Lagged { dropped })
            }
            Err(broadcast::error::TryRecvError::Closed) => Err(CanIoError::Disconnected),
        }
    }
}

#[async_trait::async_trait]
impl CanBus for MitOnlyBus {
    async fn send(&self, request: CanFrame) -> std::result::Result<(), CanIoError> {
        self.sent.lock().unwrap().push(request.clone());
        let invalid = || {
            CanIoError::backend(std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                "MIT-only fixture rejects native/configuration commands",
            ))
        };
        let CanId::Standard(id) = request.id() else {
            return Err(invalid());
        };
        if request.is_fd()
            || request.kind() != FrameKind::Data
            || request.data().len() != 8
            || id > 255
        {
            return Err(invalid());
        }
        if id != 1 {
            // Only Stop scans may address another drive.
            return if request.data() == STOP {
                Ok(())
            } else {
                Err(invalid())
            };
        }
        match request.data() {
            data if data == STOP => {
                self.physical_enabled.store(false, Ordering::Release);
                let mut due = self.zero_due.lock().unwrap();
                if due.is_some_and(|when| Instant::now() >= when) {
                    *self.position.lock().unwrap() = 0.0;
                    *due = None;
                }
                drop(due);
                if self.stop_replies.load(Ordering::Acquire) {
                    self.feedback();
                }
            }
            data if data == ENABLE => {
                self.physical_enabled.store(true, Ordering::Release);
                self.feedback();
            }
            data if data == ZERO => {
                assert!(!self.physical_enabled.load(Ordering::Acquire));
                // Reproduce the real firmware: the immediate ACK has the old
                // angle; only a Stop at least 100 ms later sees the new zero.
                self.feedback();
                if self.zero_applies.load(Ordering::Acquire) {
                    *self.zero_due.lock().unwrap() =
                        Some(Instant::now() + Duration::from_millis(100));
                }
            }
            data if data.starts_with(&[255; 6]) => return Err(invalid()),
            _ => {
                if self.motion_replies.load(Ordering::Acquire) {
                    self.feedback();
                }
            }
        }
        Ok(())
    }
    async fn subscribe(
        &self,
        _filter: CanFilter,
    ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
        Ok(Box::new(MitRx(self.tx.subscribe())))
    }
    fn capabilities(&self) -> CanCapabilities {
        CanCapabilities {
            fd: false,
            max_dlen: 8,
        }
    }
}

async fn wait_until(app: &El05Session, predicate: impl Fn(&El05State) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let state = app.snapshot();
        if predicate(&state) {
            return;
        }
        assert!(Instant::now() < deadline, "condition timed out: {state:?}");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

async fn attached(reply_id: u16) -> (Arc<MitOnlyBus>, Arc<El05Session>) {
    let bus = MitOnlyBus::new(reply_id);
    let app = El05Session::start_with_protocol(bus.clone(), 253, El05Protocol::StandardMit)
        .await
        .unwrap();
    app.attach(1).await.unwrap();
    (bus, app)
}

fn experiment_config(kind: ExperimentKind) -> ExperimentConfig {
    ExperimentConfig {
        kind,
        duration_s: 2.0,
        velocity_rad_s: 0.3,
        torque_nm: 0.03,
        kd: 0.5,
        kp: 3.0,
        position_amplitude_rad: 0.05,
        max_temperature_c: 60.0,
        settle_time_s: 0.1,
        ..Default::default()
    }
}

async fn gravity_config(app: &El05Session) -> GravityCompensationConfig {
    let reference = app.capture_gravity_reference().await.unwrap();
    GravityCompensationConfig {
        reference_id: reference.id,
        mass_kg: 0.5,
        length_m: 0.18,
        arm_mass_kg: 0.0,
        compensation_scale: 1.0,
        damping_nm_s_rad: 0.1,
        torque_limit_nm: 1.0,
        max_velocity_rad_s: 3.0,
        max_temperature_c: 60.0,
        ramp_s: 0.1,
    }
}

#[tokio::test]
async fn standard_scan_and_manual_attach_need_no_native_uid_or_dictionary() {
    let bus = MitOnlyBus::new(1);
    let app = El05Session::start_with_protocol(bus.clone(), 253, El05Protocol::StandardMit)
        .await
        .unwrap();
    app.discover().await.unwrap();
    wait_until(&app, |s| s.devices.iter().any(|d| d.motor_id == 1)).await;
    assert_eq!(app.devices()[0].uid, None);
    let state = app.attach(1).await.unwrap();
    assert!(state.attached && state.online);
    assert_eq!(state.protocol, El05Protocol::StandardMit);
    assert_eq!(state.reply_id, Some(1));
    assert!(!state.status_reliable);
    assert!(state.identity.is_none() && state.current_a.is_none() && state.voltage_v.is_none());
    assert_eq!(state.mode, El05Mode::Mit);
    app.shutdown().await.unwrap();
    assert!(bus
        .sent
        .lock()
        .unwrap()
        .iter()
        .all(|f| matches!(f.id(), CanId::Standard(_))));
}

#[tokio::test]
async fn standard_enable_stream_and_stop_work_with_permanently_zero_status() {
    let (bus, app) = attached(0x321).await;
    app.enable().await.unwrap();
    assert!(app.snapshot().enabled && bus.physical_enabled.load(Ordering::Acquire));
    assert_eq!(app.snapshot().status_code, 0);
    app.send_target(
        El05Target::Mit {
            position_rad: 0.0,
            velocity_rad_s: 0.4,
            torque_nm: 0.0,
            kp: 0.0,
            kd: 0.5,
        },
        true,
    )
    .await
    .unwrap();
    tokio::time::sleep(Duration::from_millis(80)).await;
    let state = app.snapshot();
    assert!(state.enabled && state.streaming && state.online);
    assert_eq!(state.reply_id, Some(0x321));
    assert_eq!(state.status_code, 0);
    assert!(bus.motion_count() >= 3);
    app.stop().await.unwrap();
    assert!(!app.snapshot().enabled && !app.snapshot().streaming);
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_all_eleven_pendulum_modes_record_with_standard_wire_metadata() {
    let (bus, app) = attached(1).await;
    for kind in [
        ExperimentKind::Record,
        ExperimentKind::VelocitySteps,
        ExperimentKind::TorqueRamp,
        ExperimentKind::StaticTorque,
        ExperimentKind::PositionSteps,
        ExperimentKind::SineSweep,
        ExperimentKind::Impedance,
        ExperimentKind::Backdrive,
        ExperimentKind::ThermalHold,
        ExperimentKind::SlowTriangle,
        ExperimentKind::StaticFriction,
    ] {
        app.start_experiment(experiment_config(kind)).await.unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;
        app.touch_client();
        assert!(
            app.snapshot().experiment.active,
            "{kind:?}: {:?}",
            app.snapshot().last_error
        );
        let data = app.experiment_data();
        assert_eq!(data.protocol, El05Protocol::StandardMit);
        assert_eq!(data.feedback_velocity_bits, 12);
        assert!(data.samples.len() >= 2, "{kind:?}: no continuous samples");
        assert!(data.parameters.is_empty() && data.uid.is_none() && data.version.is_none());
        assert!(data.samples.iter().all(|s| s.status_code == 0
            && s.current_a.is_none()
            && s.dictionary_velocity_rad_s.is_none()
            && s.can_id == 1
            && s.payload[0] == 1));
        if kind != ExperimentKind::Record {
            assert!(app.snapshot().enabled);
            assert!(
                !data.commands.is_empty(),
                "{kind:?}: missing command history"
            );
            assert!(data.commands.iter().all(|c| c.can_id == 1));
        }
        app.stop().await.unwrap();
        assert!(!app.snapshot().experiment.active && !app.snapshot().enabled);
    }
    assert!(bus
        .sent
        .lock()
        .unwrap()
        .iter()
        .all(|f| matches!(f.id(), CanId::Standard(_))));
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_gravity_without_uid_uses_current_attachment_reference() {
    let (bus, app) = attached(1).await;
    let config = gravity_config(&app).await;
    assert!(app.snapshot().identity.is_none());
    app.start_gravity_compensation(config.clone())
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;
    *bus.position.lock().unwrap() += 0.03;
    bus.feedback();
    tokio::time::sleep(Duration::from_millis(100)).await;
    let state = app.snapshot();
    assert!(
        state.gravity_compensation.active && state.enabled,
        "{state:?}"
    );
    assert_eq!(state.status_code, 0);
    assert!(state.gravity_compensation.gravity_torque_nm > 0.0);
    assert!(bus.motion_count() > 3);
    assert!(app
        .start_experiment(experiment_config(ExperimentKind::Record))
        .await
        .is_err());
    app.stop_gravity_compensation().await.unwrap();
    assert!(!app.snapshot().gravity_compensation.active && !app.snapshot().enabled);
    app.attach(1).await.unwrap();
    assert!(app.snapshot().gravity_compensation.reference.is_none());
    assert!(app.start_gravity_compensation(config).await.is_err());
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_fault_and_ui_expiry_stop_the_same_session() {
    let (bus, app) = attached(1).await;
    app.enable().await.unwrap();
    bus.flags.store(0x20, Ordering::Release);
    bus.feedback();
    wait_until(&app, |s| !s.enabled && !s.streaming).await;
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
    assert!(app.snapshot().fault_bits != 0 || app.snapshot().fault_code != 0);
    bus.flags.store(0, Ordering::Release);
    bus.feedback();
    tokio::time::sleep(Duration::from_millis(20)).await;
    app.attach(1).await.unwrap();
    app.enable().await.unwrap();
    app.inner.state.lock().unwrap().client_seen =
        Instant::now() - CLIENT_TIMEOUT - Duration::from_secs(1);
    wait_until(&app, |s| !s.enabled && !s.streaming).await;
    assert!(app
        .snapshot()
        .last_error
        .unwrap_or_default()
        .contains("heartbeat"));
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_motion_and_gravity_feedback_loss_stop_and_invalidate_reference() {
    let (bus, app) = attached(1).await;
    app.enable().await.unwrap();
    bus.motion_replies.store(false, Ordering::Release);
    wait_until(&app, |s| !s.enabled && !s.streaming).await;
    assert!(app
        .snapshot()
        .last_error
        .unwrap_or_default()
        .contains("feedback"));
    bus.motion_replies.store(true, Ordering::Release);
    app.attach(1).await.unwrap();
    let config = gravity_config(&app).await;
    app.start_gravity_compensation(config).await.unwrap();
    bus.motion_replies.store(false, Ordering::Release);
    wait_until(&app, |s| !s.gravity_compensation.active && !s.enabled).await;
    assert!(app.snapshot().gravity_compensation.reference.is_none());
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
    bus.motion_replies.store(true, Ordering::Release);
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_late_feedback_does_not_hide_a_control_gap() {
    let (bus, app) = attached(1).await;
    app.enable().await.unwrap();
    {
        let mut s = app.inner.state.lock().unwrap();
        s.feedback_times.clear();
        s.feedback_times
            .push_back(Instant::now() - Duration::from_millis(60));
    }
    observe_frame(&app.inner, bus.telemetry());
    assert!(app
        .inner
        .state
        .lock()
        .unwrap()
        .receive_error
        .as_deref()
        .is_some_and(|e| e.contains("feedback gap")));
    wait_until(&app, |s| !s.enabled && !s.streaming).await;
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_failed_stop_never_keeps_streaming_and_retry_recovers() {
    let (bus, app) = attached(1).await;
    app.enable().await.unwrap();
    bus.stop_replies.store(false, Ordering::Release);
    assert!(app.stop().await.is_err());
    assert!(!app.snapshot().streaming);
    let motion_count = bus.motion_count();
    let stop_count = bus.stop_count();
    tokio::time::sleep(Duration::from_millis(320)).await;
    assert_eq!(motion_count, bus.motion_count());
    assert!(
        bus.stop_count() > stop_count,
        "unconfirmed Stop must be retried without a drive watchdog"
    );
    assert!(app.enable().await.is_err());
    bus.stop_replies.store(true, Ordering::Release);
    tokio::time::sleep(Duration::from_millis(280)).await;
    assert_eq!(motion_count, bus.motion_count());
    assert!(
        app.enable().await.is_err(),
        "background Stop replies must not silently unlock control"
    );
    app.stop().await.unwrap();
    app.enable().await.unwrap();
    app.stop().await.unwrap();
    assert!(bus.stop_count() >= 3);
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_native_only_operations_are_rejected_without_native_transmissions() {
    let (bus, app) = attached(1).await;
    assert!(app.set_mode(El05Mode::Velocity).await.is_err());
    assert!(app.read_parameter(0x7005).await.is_err());
    assert!(app.write_parameter(0x7028, 20_000.0).await.is_err());
    assert!(app.read_identity().await.is_err());
    assert!(app.read_version().await.is_err());
    assert!(app.change_id(2).await.is_err());
    assert!(app.clear_fault().await.is_err());
    assert!(bus
        .sent
        .lock()
        .unwrap()
        .iter()
        .all(|f| matches!(f.id(), CanId::Standard(_))));
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_zero_delayed_readback_rebases_saturated_position() {
    let (bus, app) = attached(1).await;
    let old_reference = gravity_config(&app).await;
    *bus.position.lock().unwrap() = 12.57;
    bus.feedback();
    tokio::time::sleep(Duration::from_millis(5)).await;
    assert!(app
        .capture_gravity_reference()
        .await
        .unwrap_err()
        .to_string()
        .contains("saturated"));
    assert!(app
        .start_gravity_compensation(old_reference.clone())
        .await
        .unwrap_err()
        .to_string()
        .contains("saturated"));
    {
        let mut state = app.inner.state.lock().unwrap();
        state.last_sent_target = Some(El05Target::Mit {
            position_rad: 1.0,
            velocity_rad_s: 0.0,
            torque_nm: 0.1,
            kp: 0.0,
            kd: 0.0,
        });
        state.last_target_sent = Some(Instant::now());
        state.target_sequence = 42;
        state.receive_error =
            Some("standard MIT control feedback gap exceeded 30 ms; motor stopped".into());
    }
    let started = Instant::now();
    app.set_zero().await.unwrap();
    assert!(started.elapsed() >= Duration::from_millis(100));
    let after = app.snapshot();
    assert!(after.position_rad.unwrap().abs() < 0.001);
    assert_eq!(after.unwrapped_position_rad, after.position_rad);
    assert!(after.position_velocity_rad_s.is_none());
    assert!(after.gravity_compensation.reference.is_none());
    assert!(!after.enabled && !after.streaming && after.fault_bits == 0);
    {
        let state = app.inner.state.lock().unwrap();
        assert!(state.mode_confirmed && state.receive_error.is_none());
        assert!(state.last_sent_target.is_none() && state.last_target_sent.is_none());
        assert_eq!(state.target_sequence, 0);
    }
    assert!(app.start_gravity_compensation(old_reference).await.is_err());
    let new_reference = gravity_config(&app).await;
    app.start_gravity_compensation(new_reference).await.unwrap();
    *bus.position.lock().unwrap() = -12.57;
    bus.feedback();
    wait_until(&app, |s| !s.gravity_compensation.active && !s.enabled).await;
    assert!(app.snapshot().gravity_compensation.reference.is_none());
    assert!(app
        .snapshot()
        .last_error
        .unwrap_or_default()
        .contains("saturated"));
    assert_eq!(app.snapshot().fault_bits, 0);
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_zero_failed_readback_requires_reselection() {
    let (bus, app) = attached(1).await;
    bus.zero_applies.store(false, Ordering::Release);
    let error = app.set_zero().await.unwrap_err().to_string();
    assert!(error.contains("expected near zero") && error.contains("Select the motor again"));
    assert!(!app.inner.state.lock().unwrap().mode_confirmed);
    assert!(!app.snapshot().enabled && !app.snapshot().streaming);
    assert!(app.enable().await.is_err());
    assert!(app.capture_gravity_reference().await.is_err());
    app.shutdown().await.unwrap();
}
