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
    motor_id: AtomicU8,
    position: StdMutex<f64>,
    velocity_raw: AtomicU16,
    zero_due: StdMutex<Option<Instant>>,
    zero_applies: AtomicBool,
    follow_position: AtomicBool,
    temperature_decic: AtomicU16,
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
            motor_id: AtomicU8::new(1),
            position: StdMutex::new(-2.9333),
            velocity_raw: AtomicU16::new(2048),
            zero_due: StdMutex::new(None),
            zero_applies: AtomicBool::new(true),
            follow_position: AtomicBool::new(false),
            temperature_decic: AtomicU16::new(307),
        })
    }

    fn telemetry(&self) -> CanFrame {
        // Independent wire fixture: p16, v12, tau12=2048, 30.7 C by default.
        let p = (((*self.position.lock().unwrap() + 12.57) / 25.14) * 65535.0).round() as u16;
        let velocity = self.velocity_raw.load(Ordering::Acquire);
        let temperature = self.temperature_decic.load(Ordering::Acquire);
        CanFrame::new_data(
            CanId::Standard(self.reply_id.load(Ordering::Acquire)),
            &[
                self.motor_id.load(Ordering::Acquire),
                (p >> 8) as u8,
                p as u8,
                (velocity >> 4) as u8,
                ((velocity as u8) << 4) | 0x08,
                0,
                self.flags.load(Ordering::Acquire) | ((temperature >> 8) as u8 & 0x0f),
                temperature as u8,
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
        if id != u16::from(self.motor_id.load(Ordering::Acquire)) {
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
                let kp_raw =
                    (u16::from(request.data()[3] & 0x0f) << 8) | u16::from(request.data()[4]);
                if self.follow_position.load(Ordering::Acquire) && kp_raw > 0 {
                    let p = u16::from_be_bytes([request.data()[0], request.data()[1]]);
                    *self.position.lock().unwrap() = f64::from(p) * 25.14 / 65535.0 - 12.57;
                }
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
        axis3: None,
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

async fn axis3_config(app: &El05Session) -> GravityCompensationConfig {
    let mut config = gravity_config(app).await;
    let reference = app
        .capture_axis3_reference(crate::arm_axis3::Axis3Calibration {
            joint_angle_rad: 1.,
            encoder_direction: 1,
        })
        .await
        .unwrap();
    config.reference_id = reference.id;
    config.axis3 = Some(crate::arm_axis3::Axis3GravityConfig::default());
    config.torque_limit_nm = 2.;
    config
}

#[tokio::test]
async fn axis3_reference_binding_rejects_wrong_model_zero_and_selection_changes() {
    let (_, app) = attached(1).await;
    let mut pendulum = gravity_config(&app).await;
    pendulum.axis3 = Some(crate::arm_axis3::Axis3GravityConfig::default());
    pendulum.torque_limit_nm = 2.;
    assert!(app
        .start_gravity_compensation(pendulum)
        .await
        .unwrap_err()
        .to_string()
        .contains("model mismatch"));
    let config = axis3_config(&app).await;
    let mut wrong = config.clone();
    wrong.axis3 = None;
    assert!(app
        .start_gravity_compensation(wrong)
        .await
        .unwrap_err()
        .to_string()
        .contains("model mismatch"));
    app.start_gravity_compensation(config.clone())
        .await
        .unwrap();
    assert!(app
        .capture_axis3_reference(crate::arm_axis3::Axis3Calibration {
            joint_angle_rad: 0.,
            encoder_direction: 1
        })
        .await
        .is_err());
    assert!(app
        .send_target(
            El05Target::Mit {
                position_rad: 0.,
                velocity_rad_s: 0.,
                torque_nm: 0.,
                kp: 0.,
                kd: 0.
            },
            true
        )
        .await
        .is_err());
    app.stop().await.unwrap();
    app.set_zero().await.unwrap();
    assert!(app.snapshot().gravity_compensation.reference.is_none());
    assert!(app.start_gravity_compensation(config).await.is_err());
    let config = axis3_config(&app).await;
    app.attach(1).await.unwrap();
    assert!(app.start_gravity_compensation(config).await.is_err());
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn loaded_axis3_keeps_sending_damped_torque_above_the_old_speed_limit() {
    let (bus, app) = attached(1).await;
    let mut config = axis3_config(&app).await;
    config.axis3.as_mut().unwrap().payload_mass_kg = 0.25;
    // A legacy client can still supply its former threshold.
    config.max_velocity_rad_s = 1.;
    app.start_gravity_compensation(config.clone())
        .await
        .unwrap();
    for raw in [1024, 3071] {
        bus.velocity_raw.store(raw, Ordering::Release);
        bus.feedback();
        let sign = if raw < 2048 { -1. } else { 1. };
        wait_until(&app, |s| {
            let status = &s.gravity_compensation;
            s.enabled
                && s.streaming
                && status.active
                && status.elapsed_s >= config.ramp_s
                && s.velocity_rad_s.is_some_and(|v| sign * v > 20.)
                && sign * status.damping_torque_nm < -2.
                && status.gravity_torque_nm > 0.
                && status.command_torque_nm.abs() <= config.torque_limit_nm
        })
        .await;
        assert!(bus.physical_enabled.load(Ordering::Acquire));
    }
    {
        let _gate = app.inner.gate.lock().await;
        let mut s = app.inner.state.lock().unwrap();
        s.view.velocity_rad_s = Some(0.);
        s.view.position_velocity_rad_s = Some(-25.);
        assert!(gravity_input_problem(&s, &config, true).is_none());
        s.view.position_velocity_rad_s = Some(f64::NAN);
        assert!(gravity_input_problem(&s, &config, true)
            .unwrap()
            .contains("finite"));
        s.view.position_velocity_rad_s = Some(0.);
    }
    app.stop().await.unwrap();
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn axis3_stops_on_joint_limit_fault_temperature_feedback_loss_and_heartbeat() {
    for condition in ["angle", "fault", "temperature", "feedback", "heartbeat"] {
        let (bus, app) = attached(1).await;
        let config = axis3_config(&app).await;
        app.start_gravity_compensation(config).await.unwrap();
        assert!(bus.physical_enabled.load(Ordering::Acquire));
        match condition {
            "angle" => {
                // Move the coordinate outside the model range while keeping fresh
                // feedback to exercise the angle guard.
                app.inner
                    .state
                    .lock()
                    .unwrap()
                    .gravity
                    .status
                    .reference
                    .as_mut()
                    .unwrap()
                    .position_rad += 2.;
            }
            "fault" => {
                bus.flags.store(0x20, Ordering::Release);
                bus.feedback();
            }
            "temperature" => {
                bus.temperature_decic.store(900, Ordering::Release);
                bus.feedback();
            }
            "feedback" => {
                bus.motion_replies.store(false, Ordering::Release);
            }
            "heartbeat" => {
                app.inner.state.lock().unwrap().client_seen =
                    Instant::now() - CLIENT_TIMEOUT - Duration::from_secs(1);
            }
            _ => unreachable!(),
        }
        wait_until(&app, |s| {
            !s.gravity_compensation.active && !s.enabled && !s.streaming
        })
        .await;
        assert!(!bus.physical_enabled.load(Ordering::Acquire), "{condition}");
        let state = app.snapshot();
        assert_eq!(state.gravity_compensation.command_torque_nm, 0.);
        if condition == "angle" {
            assert!(state
                .gravity_compensation
                .reason
                .unwrap()
                .contains("URDF limits"));
        }
        if condition == "feedback" {
            assert!(state.gravity_compensation.reference.is_none());
        }
        app.shutdown().await.unwrap();
    }
}

#[tokio::test]
async fn axis3_start_queued_before_stop_cannot_reenable_the_motor() {
    let (bus, app) = attached(1).await;
    let config = axis3_config(&app).await;
    let gate = app.start_gate.lock().await;
    let start_app = app.clone();
    let start = tokio::spawn(async move { start_app.start_gravity_compensation(config).await });
    tokio::time::sleep(Duration::from_millis(10)).await;
    app.stop().await.unwrap();
    drop(gate);
    assert!(start.await.unwrap().is_err());
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
    assert!(!app.snapshot().gravity_compensation.active);
    app.shutdown().await.unwrap();
}

async fn accuracy_config(app: &El05Session) -> ExperimentConfig {
    let reference = app.capture_gravity_reference().await.unwrap();
    ExperimentConfig {
        kind: ExperimentKind::TorqueAccuracy,
        kp: 80.0,
        kd: 2.0,
        max_temperature_c: 120.0,
        gravity_compensation: true,
        pendulum: Some(experiment::PendulumConfig {
            mass_kg: 3.0,
            length_m: 0.18,
            // The session must replace UI coordinates with the bound reference.
            downward_zero_rad: 123.0,
            zero_measured: true,
            ..Default::default()
        }),
        torque_accuracy: Some(experiment::torque_accuracy::TorqueAccuracyConfig {
            reference_id: reference.id,
            start_nm: 3.7,
            end_nm: 5.2,
            step_nm: 0.05,
            hold_s: 1.0,
            move_speed_rad_s: 0.3,
            direction: 1,
        }),
        ..Default::default()
    }
}

#[tokio::test]
async fn standard_accuracy_overtemperature_keeps_position_control_until_zero_then_stops() {
    let (bus, app) = attached(1).await;
    let config = accuracy_config(&app).await;
    let zero = app
        .snapshot()
        .gravity_compensation
        .reference
        .unwrap()
        .position_rad;
    bus.follow_position.store(true, Ordering::Release);
    app.start_experiment(config).await.unwrap();
    let stop_count = bus.stop_count();
    assert_eq!(
        app.experiment_data()
            .status
            .config
            .unwrap()
            .pendulum
            .unwrap()
            .downward_zero_rad,
        zero
    );
    tokio::time::sleep(Duration::from_millis(700)).await;
    assert!(app.snapshot().position_rad.unwrap() > zero + 0.01);
    bus.temperature_decic.store(1200, Ordering::Release);
    wait_until(&app, |s| {
        s.experiment
            .torque_accuracy
            .as_ref()
            .is_some_and(|a| a.phase == experiment::torque_accuracy::TorqueAccuracyPhase::Returning)
    })
    .await;
    let returning = app.snapshot();
    assert!(returning.enabled && returning.streaming && returning.experiment.active);
    assert_eq!(
        bus.stop_count(),
        stop_count,
        "temperature alone must not send immediate Stop"
    );
    wait_until(&app, |s| !s.experiment.active && !s.enabled).await;
    let data = app.experiment_data();
    assert!(
        data.status
            .torque_accuracy
            .as_ref()
            .unwrap()
            .returned_to_zero
    );
    assert!(data
        .status
        .reason
        .as_ref()
        .unwrap()
        .contains("temperature bound reached"));
    assert!((app.snapshot().position_rad.unwrap() - zero).abs() < 0.02);
    assert!(bus.stop_count() > stop_count);
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
    assert!(data.commands.iter().all(|c| matches!(
        c.target,
        El05Target::Mit {
            kp: 80.0,
            kd: 2.0,
            torque_nm,
            ..
        } if torque_nm.is_finite() && torque_nm.abs() <= 6.0
    )));
    assert!(data
        .commands
        .iter()
        .any(|c| matches!(c.target, El05Target::Mit { torque_nm, .. } if torque_nm > 0.01)));
    let return_start = data
        .status
        .torque_accuracy
        .as_ref()
        .unwrap()
        .return_started_s
        .unwrap();
    assert!(data
        .commands
        .iter()
        .filter(|c| c.t_s >= return_start)
        .any(|c| matches!(c.target, El05Target::Mit { torque_nm, .. } if torque_nm > 0.01)));
    assert!(
        matches!(data.commands.last().unwrap().target, El05Target::Mit { torque_nm, .. } if torque_nm.abs() < 0.02)
    );
    assert!(data.samples.iter().any(|s| s.temperature_c >= 120.0));
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn standard_accuracy_stale_zero_is_rejected_and_stop_fault_and_feedback_guards_interrupt_return(
) {
    for condition in ["stop", "fault", "feedback"] {
        let (bus, app) = attached(1).await;
        let mut config = accuracy_config(&app).await;
        let reference_id = config
            .torque_accuracy
            .as_ref()
            .unwrap()
            .reference_id
            .clone();
        config.torque_accuracy.as_mut().unwrap().reference_id = "stale".into();
        assert!(app.start_experiment(config.clone()).await.is_err());
        assert!(!bus.physical_enabled.load(Ordering::Acquire));
        config.torque_accuracy.as_mut().unwrap().reference_id = reference_id;
        bus.follow_position.store(true, Ordering::Release);
        app.start_experiment(config).await.unwrap();
        tokio::time::sleep(Duration::from_millis(700)).await;
        bus.temperature_decic.store(1200, Ordering::Release);
        wait_until(&app, |s| {
            s.experiment.torque_accuracy.as_ref().is_some_and(|a| {
                a.phase == experiment::torque_accuracy::TorqueAccuracyPhase::Returning
            })
        })
        .await;
        match condition {
            "stop" => app.stop().await.unwrap(),
            "fault" => bus.flags.store(0x20, Ordering::Release),
            _ => bus.motion_replies.store(false, Ordering::Release),
        }
        wait_until(&app, |s| !s.experiment.active && !s.enabled).await;
        assert!(!bus.physical_enabled.load(Ordering::Acquire), "{condition}");
        assert!(
            !app.experiment_data()
                .status
                .torque_accuracy
                .unwrap()
                .returned_to_zero,
            "{condition}"
        );
        app.shutdown().await.unwrap();
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

/// Independent manual-extension drive: parameter, configuration and float
/// target replies are deliberately distinct from packed motion telemetry.
struct ManualMitBus {
    basic: Arc<MitOnlyBus>,
    mode: AtomicU8,
    parameters: StdMutex<BTreeMap<u16, [u8; 4]>>,
    apply_writes: AtomicBool,
    acknowledge_id: AtomicBool,
    reporting: AtomicBool,
    occupied: AtomicU8,
}

impl ManualMitBus {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            basic: MitOnlyBus::new(0xfd),
            mode: AtomicU8::new(0),
            parameters: StdMutex::new(BTreeMap::from([(0x700b, 6.0f32.to_le_bytes())])),
            apply_writes: AtomicBool::new(true),
            acknowledge_id: AtomicBool::new(true),
            reporting: AtomicBool::new(false),
            occupied: AtomicU8::new(0xff),
        })
    }
    fn reply(&self, id: u16, data: [u8; 8]) {
        let _ = self
            .basic
            .tx
            .send(CanFrame::new_data(CanId::Standard(id), &data).unwrap());
    }
    fn uid(&self, id: u8) {
        self.reply(
            u16::from(id),
            [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0],
        );
    }
}

#[async_trait::async_trait]
impl CanBus for ManualMitBus {
    async fn send(&self, request: CanFrame) -> std::result::Result<(), CanIoError> {
        let CanId::Standard(id) = request.id() else {
            panic!("manual MIT sent an extended frame");
        };
        let source = id as u8;
        let motor = self.basic.motor_id.load(Ordering::Acquire);
        let data: [u8; 8] = request.data().try_into().unwrap();
        if source != motor {
            self.basic.sent.lock().unwrap().push(request);
            if source == self.occupied.load(Ordering::Acquire) && data == STOP {
                self.reply(0xfd, [source, 0x80, 0, 0x80, 8, 0, 1, 0x33]);
            }
            return Ok(());
        }
        let ty = id >> 8;
        if matches!(ty, 1..=4) {
            self.basic.sent.lock().unwrap().push(request);
            match ty {
                3 => {
                    let index = u16::from_le_bytes([data[0], data[1]]);
                    let value = match index {
                        0x7005 => [self.mode.load(Ordering::Acquire), 0, 0, 0],
                        0x7019 => (*self.basic.position.lock().unwrap() as f32).to_le_bytes(),
                        _ => self
                            .parameters
                            .lock()
                            .unwrap()
                            .get(&index)
                            .copied()
                            .unwrap_or([0; 4]),
                    };
                    let mut response = data;
                    response[4..].copy_from_slice(&value);
                    self.reply(id, response);
                }
                4 => {
                    let index = u16::from_le_bytes([data[0], data[1]]);
                    if self.apply_writes.load(Ordering::Acquire) {
                        self.parameters
                            .lock()
                            .unwrap()
                            .insert(index, data[4..].try_into().unwrap());
                    }
                    self.reply(id, data);
                }
                _ => self.basic.feedback(),
            }
            return Ok(());
        }
        if data[..6] == [0xff; 6] && data != STOP && data != ENABLE && data != ZERO {
            self.basic.sent.lock().unwrap().push(request);
            match data[7] {
                0xfc => {
                    self.mode.store(data[6], Ordering::Release);
                    self.basic.feedback();
                }
                0xfb => {
                    self.basic.flags.store(0, Ordering::Release);
                    self.basic.feedback();
                }
                0xfa => {
                    self.basic.motor_id.store(data[6], Ordering::Release);
                    if self.acknowledge_id.load(Ordering::Acquire) {
                        self.uid(data[6]);
                    }
                }
                0xf8 => self.uid(motor),
                0xf9 => {
                    self.reporting.store(data[6] != 0, Ordering::Release);
                    self.basic.feedback();
                }
                _ => panic!("unexpected MIT configuration"),
            }
            return Ok(());
        }
        self.basic.send(request).await
    }
    async fn subscribe(
        &self,
        filter: CanFilter,
    ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
        self.basic.subscribe(filter).await
    }
    fn capabilities(&self) -> CanCapabilities {
        self.basic.capabilities()
    }
}

async fn manual_attached() -> (Arc<ManualMitBus>, Arc<El05Session>) {
    let bus = ManualMitBus::new();
    let app = El05Session::start_with_options(bus.clone(), 0xfd, El05Protocol::StandardMit, true)
        .await
        .unwrap();
    app.attach(1).await.unwrap();
    (bus, app)
}

#[tokio::test]
async fn manual_modes_use_csp_one_and_float_targets_and_require_stopped_parameter_access() {
    let (bus, app) = manual_attached().await;
    assert!(app.snapshot().manual_extensions);
    assert!(app.set_mode(El05Mode::PositionPp).await.is_err());
    app.set_mode(El05Mode::PositionCsp).await.unwrap();
    assert_eq!(bus.mode.load(Ordering::Acquire), 1);
    app.enable().await.unwrap();
    app.send_target(
        El05Target::PositionCsp {
            position_rad: 5.0,
            velocity_rad_s: 5.0,
            current_limit_a: 1.0,
        },
        true,
    )
    .await
    .unwrap();
    assert!(app.read_parameter(0x7019).await.is_err());
    app.stop().await.unwrap();
    app.set_mode(El05Mode::Velocity).await.unwrap();
    app.enable().await.unwrap();
    app.send_target(
        El05Target::Velocity {
            velocity_rad_s: 5.0,
            current_limit_a: 2.0,
            acceleration_rad_s2: 1.0,
        },
        true,
    )
    .await
    .unwrap();
    app.stop().await.unwrap();
    app.write_parameter(0x7005, 1.0).await.unwrap();
    assert_eq!(app.snapshot().mode, El05Mode::PositionCsp);
    assert_eq!(app.read_parameter(0x7005).await.unwrap().value, 1.0);
    let sent = bus.basic.sent.lock().unwrap();
    assert!(sent.iter().any(|frame| frame.id() == CanId::Standard(0x101)
        && frame.data() == [0, 0, 0xa0, 0x40, 0, 0, 0xa0, 0x40]));
    assert!(sent.iter().any(|frame| frame.id() == CanId::Standard(0x201)
        && frame.data() == [0, 0, 0xa0, 0x40, 0, 0, 0, 0x40]));
    drop(sent);
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn manual_parameter_write_is_verified_and_extension_responses_never_renew_motion_feedback() {
    let (bus, app) = manual_attached().await;
    app.write_parameter(0x700b, 3.0).await.unwrap();
    assert_eq!(app.read_parameter(0x700b).await.unwrap().value, 3.0);
    bus.apply_writes.store(false, Ordering::Release);
    assert!(app
        .write_parameter(0x700b, 4.0)
        .await
        .unwrap_err()
        .to_string()
        .contains("verification failed"));
    assert!(app.write_parameter(0x7019, 0.0).await.is_err());
    {
        let _guard = app.inner.gate.lock().await;
        let sequence = app.snapshot().feedback_sequence;
        observe_frame(
            &app.inner,
            CanFrame::new_data(CanId::Standard(0x301), &[1, 0x70, 0, 0, 0, 0, 0, 0]).unwrap(),
        );
        observe_frame(
            &app.inner,
            CanFrame::new_data(CanId::Standard(0x401), &[1, 0x70, 0, 0, 0, 0, 0, 0]).unwrap(),
        );
        let identity = app
            .inner
            .standard_uid_command(1, 1, 0xff, 0xf8)
            .await
            .unwrap();
        assert_eq!(identity.uid, "123456789abcdef0");
        assert_eq!(sequence, app.snapshot().feedback_sequence);
    }
    bus.basic.stop_replies.store(false, Ordering::Release);
    assert!(app.stop().await.is_err());
    let before = bus.basic.sent.lock().unwrap().len();
    assert!(app.read_parameter(0x700b).await.is_err());
    assert!(app.write_parameter(0x700b, 2.0).await.is_err());
    assert!(bus.basic.sent.lock().unwrap()[before..]
        .iter()
        .all(|frame| frame.data() == STOP));
    bus.basic.stop_replies.store(true, Ordering::Release);
    app.stop().await.unwrap();
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn manual_configuration_confirms_fault_reset_save_and_reporting_and_cleans_up() {
    let (bus, app) = manual_attached().await;
    bus.basic.flags.store(0x20, Ordering::Release);
    bus.basic.feedback();
    wait_until(&app, |state| state.fault_bits != 0).await;
    app.clear_fault().await.unwrap();
    assert_eq!(app.snapshot().fault_bits, 0);
    let identity = app.save_parameters().await.unwrap();
    assert_eq!(identity.motor_id, 1);
    app.set_reporting(true).await.unwrap();
    assert!(app.snapshot().reporting && bus.reporting.load(Ordering::Acquire));
    app.enable().await.unwrap();
    assert!(app.save_parameters().await.is_err());
    assert!(app.set_reporting(false).await.is_err());
    app.shutdown().await.unwrap();
    assert!(!bus.reporting.load(Ordering::Acquire));
    assert!(!bus.basic.physical_enabled.load(Ordering::Acquire));
}

#[tokio::test]
async fn manual_id_change_rejects_collisions_then_selects_the_acknowledged_new_address() {
    let (bus, app) = manual_attached().await;
    bus.occupied.store(2, Ordering::Release);
    assert!(app
        .change_id(2)
        .await
        .unwrap_err()
        .to_string()
        .contains("already responds"));
    assert_eq!(bus.basic.motor_id.load(Ordering::Acquire), 1);
    bus.occupied.store(0xff, Ordering::Release);
    app.change_id(2).await.unwrap();
    let state = app.snapshot();
    assert_eq!(state.motor_id, Some(2));
    assert!(state.online && !state.enabled && !state.streaming);
    assert_eq!(state.identity.unwrap().uid, "123456789abcdef0");
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn manual_id_change_with_lost_acknowledgement_clears_the_old_selection() {
    let (bus, app) = manual_attached().await;
    bus.acknowledge_id.store(false, Ordering::Release);
    assert!(app.change_id(2).await.is_err());
    assert_eq!(bus.basic.motor_id.load(Ordering::Acquire), 2);
    assert_eq!(app.snapshot().motor_id, None);
    assert!(!app.snapshot().attached && !app.snapshot().enabled);
    assert!(app.enable().await.is_err());
    app.attach(2).await.unwrap();
    assert!(app.snapshot().online);
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn basic_profile_rejects_extended_save_and_reporting_before_transmitting() {
    let (bus, app) = attached(1).await;
    let _guard = app.inner.gate.lock().await;
    assert!(app.inner.require_manual_extensions().is_err());
    assert!(!app.snapshot().manual_extensions);
    let before = bus.sent.lock().unwrap().len();
    drop(_guard);
    assert!(app.save_parameters().await.is_err());
    assert!(app.set_reporting(true).await.is_err());
    assert!(bus.sent.lock().unwrap()[before..]
        .iter()
        .all(|frame| frame.data() == STOP));
    app.shutdown().await.unwrap();
}

#[tokio::test]
async fn independent_workspace_disconnect_cannot_stop_the_other_app_and_native_close_stops_mit() {
    let (bus, app) = attached(0xfd).await;
    let state = crate::state::AppState::default();
    *state.standard_mit.lock().await = Some(app.clone());
    app.enable().await.unwrap();
    crate::el05_commands::disconnect_workspace(&state, El05Protocol::Native)
        .await
        .unwrap();
    assert!(app.snapshot().connected && app.snapshot().enabled);
    assert!(state.standard_mit.lock().await.is_some());
    crate::el05_commands::stop_session(&state, false)
        .await
        .unwrap();
    assert!(state.standard_mit.lock().await.is_none());
    assert!(!app.snapshot().connected);
    assert!(!bus.physical_enabled.load(Ordering::Acquire));
}

#[tokio::test]
async fn independent_workspace_retains_a_failed_stop_for_retry_and_force_close_releases_it() {
    let (bus, app) = attached(0xfd).await;
    let state = crate::state::AppState::default();
    *state.standard_mit.lock().await = Some(app.clone());
    bus.stop_replies.store(false, Ordering::Release);
    let error = crate::el05_commands::stop_session(&state, false)
        .await
        .unwrap_err();
    assert!(error.contains("Standard MIT"));
    assert!(state.standard_mit.lock().await.is_some());
    assert!(app.snapshot().connected);
    assert!(crate::el05_commands::stop_session(&state, true)
        .await
        .is_err());
    assert!(state.standard_mit.lock().await.is_none());
    assert!(!app.snapshot().connected);
}
