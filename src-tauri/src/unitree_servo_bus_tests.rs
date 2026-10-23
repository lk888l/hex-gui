use super::bus::{BusTestKind, BusTestNode};
use super::*;
use crate::unitree_protocol::crc32;
use std::collections::VecDeque;

struct SimNode {
    mode: u8,
    respond: bool,
    fault: u32,
    corrupt_crc: bool,
    stuck_mode: Option<u8>,
}
struct Model {
    nodes: BTreeMap<u8, SimNode>,
    frames: Vec<[u8; 20]>,
    late_other_id: bool,
}
struct BusSerial {
    model: Arc<Mutex<Model>>,
    input: VecDeque<u8>,
}

fn feedback(id: u8, mode: u8, fault: u32) -> [u8; 26] {
    let mut packet = [0; 26];
    packet[..6].copy_from_slice(&[0xfc, 0xee, id | (mode << 4), 25, 30, 48]);
    packet[14..18].copy_from_slice(&fault.to_le_bytes());
    let checksum = crc32(&packet[2..22]);
    packet[22..].copy_from_slice(&checksum.to_le_bytes());
    packet
}

impl Read for BusSerial {
    fn read(&mut self, data: &mut [u8]) -> io::Result<usize> {
        if self.input.is_empty() {
            std::thread::sleep(Duration::from_millis(1));
            return Err(io::ErrorKind::TimedOut.into());
        }
        let count = data.len().min(7).min(self.input.len());
        for byte in &mut data[..count] {
            *byte = self.input.pop_front().unwrap();
        }
        Ok(count)
    }
}
impl Write for BusSerial {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        let packet: [u8; 20] = data.try_into().unwrap();
        assert_eq!(
            crc32(&packet[..16]),
            u32::from_le_bytes(packet[16..].try_into().unwrap())
        );
        let id = packet[2] & 15;
        assert!(id < 15, "multi-servo polling must not broadcast");
        let requested = (packet[2] >> 4) & 7;
        assert!(
            requested <= 1,
            "mode tests must not send Boot or calibration"
        );
        let mut model = self.model.lock().unwrap();
        model.frames.push(packet);
        let late = if model.late_other_id {
            model.nodes.keys().copied().find(|other| *other != id)
        } else {
            None
        };
        let Some(node) = model.nodes.get_mut(&id) else {
            return Ok(data.len());
        };
        // Reproduces real J288: reply carries the PREVIOUS command's mode.
        let reported = node.stuck_mode.unwrap_or(node.mode);
        if node.stuck_mode.is_none() {
            node.mode = requested;
        }
        if !node.respond {
            return Ok(data.len());
        }
        let mut response = feedback(id, reported, node.fault);
        if node.corrupt_crc {
            response[7] ^= 0x20;
        }
        self.input.extend(packet); // adapter TX echo
        if let Some(other) = late {
            self.input.extend(feedback(other, 0, 0));
        }
        self.input.extend(response);
        Ok(data.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}
impl SerialIo for BusSerial {
    fn clear_input(&mut self) -> io::Result<()> {
        self.input.clear();
        Ok(())
    }
}

async fn simulated(ids: &[u8]) -> (Arc<ServoSession>, Arc<Mutex<Model>>) {
    let model = Arc::new(Mutex::new(Model {
        nodes: ids
            .iter()
            .map(|id| {
                (
                    *id,
                    SimNode {
                        mode: 0,
                        respond: true,
                        fault: 0,
                        corrupt_crc: false,
                        stuck_mode: None,
                    },
                )
            })
            .collect(),
        frames: vec![],
        late_other_id: false,
    }));
    let app = ServoSession::start(
        "multi-servo simulation".into(),
        ids[0],
        ServoSettings {
            response_timeout_ms: 5,
            ..ServoSettings::default()
        },
        Box::new(BusSerial {
            model: model.clone(),
            input: VecDeque::new(),
        }),
    )
    .unwrap();
    app.execute(ServoOperation::Select(ids[0])).await.unwrap();
    (app, model)
}

fn request(ids: &[u8], kind: BusTestKind, duration_s: u16) -> BusTestRequest {
    BusTestRequest {
        kind,
        duration_s,
        nodes: ids
            .iter()
            .enumerate()
            .map(|(index, id)| BusTestNode {
                id: *id,
                target: ServoTarget {
                    position_rad: index as f64 * 0.1,
                    torque_nm: (index + 1) as f64 * 0.02,
                    ..ServoTarget::default()
                },
            })
            .collect(),
    }
}
async fn until(app: &ServoSession, condition: impl Fn() -> bool, budget: Duration) {
    let deadline = Instant::now() + budget;
    while !condition() {
        assert!(Instant::now() < deadline, "condition timed out");
        app.touch_client();
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

#[tokio::test]
async fn delayed_modes_enable_with_zero_effort_then_apply_target_and_lock() {
    let (app, model) = simulated(&[0]).await;
    let target = ServoTarget {
        torque_nm: 0.03,
        ..ServoTarget::default()
    };
    let state = app
        .execute(ServoOperation::Send {
            target,
            repeat: true,
        })
        .await
        .unwrap();
    assert!(state.motion_active && state.feedback.unwrap().mode == 1);
    {
        let model = model.lock().unwrap();
        let foc: Vec<_> = model
            .frames
            .iter()
            .filter(|frame| frame[2] & 0x70 == 0x10)
            .collect();
        assert!(foc.len() >= 3);
        assert!(foc[..2]
            .iter()
            .all(|frame| frame[4..16].iter().all(|byte| *byte == 0)));
        assert_eq!(*foc[2], control_packet(0, 1, true, target).unwrap());
    }
    let state = app.execute(ServoOperation::Stop).await.unwrap();
    assert!(state.stop_confirmed && state.pending_stop_ids.is_empty());
    assert_eq!(state.feedback.unwrap().mode, 0);
    assert_eq!(model.lock().unwrap().nodes[&0].mode, 0);
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn mode_that_never_changes_is_bounded_and_never_receives_nonzero_target() {
    let (app, model) = simulated(&[0]).await;
    model.lock().unwrap().nodes.get_mut(&0).unwrap().stuck_mode = Some(0);
    let started = Instant::now();
    let result = app
        .execute(ServoOperation::Send {
            target: ServoTarget {
                torque_nm: 0.2,
                ..ServoTarget::default()
            },
            repeat: true,
        })
        .await;
    assert!(result.unwrap_err().to_string().contains("mode transition"));
    assert!(started.elapsed() < Duration::from_millis(500));
    assert!(model
        .lock()
        .unwrap()
        .frames
        .iter()
        .all(|frame| frame[4..16].iter().all(|byte| *byte == 0)));
    assert!(app.snapshot().stop_confirmed);
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn explicit_enable_selects_foc_with_zero_gains_and_explicit_lock_clears_it() {
    let (app, _) = simulated(&[0]).await;
    let enabled = app.execute(ServoOperation::SetMode(1)).await.unwrap();
    assert_eq!(enabled.feedback.unwrap().mode, 1);
    assert_eq!(enabled.target.unwrap().kp, 0.0);
    assert_eq!(enabled.target.unwrap().kd, 0.0);
    assert!(app.execute(ServoOperation::SetMode(2)).await.is_err());
    assert!(
        app.execute(ServoOperation::SetMode(0))
            .await
            .unwrap()
            .stop_confirmed
    );
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn shared_bus_polls_distinct_ids_and_retains_per_id_results() {
    let (app, model) = simulated(&[0, 3, 14]).await;
    model.lock().unwrap().late_other_id = true;
    app.execute(ServoOperation::StartBusTest(request(
        &[0, 3, 14],
        BusTestKind::Communication,
        1,
    )))
    .await
    .unwrap();
    until(
        &app,
        || !app.snapshot().bus_test.unwrap().running,
        Duration::from_secs(2),
    )
    .await;
    let state = app.snapshot();
    let report = state.bus_test.unwrap();
    assert_eq!(report.phase, "completed");
    assert!(report.cycles > 10 && state.stop_confirmed);
    for node in report.nodes {
        assert_eq!(node.tx_count, node.rx_count);
        assert!(node.rx_count > 10 && node.stop_confirmed);
        assert_eq!(node.timeout_count, 0);
    }
    assert_eq!(state.devices.len(), 3);
    assert!(model
        .lock()
        .unwrap()
        .frames
        .iter()
        .all(|frame| frame[2] & 0x70 == 0));
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn multi_foc_routes_independent_targets_and_stops_every_member() {
    let (app, model) = simulated(&[0, 3, 14]).await;
    let request = request(&[0, 3, 14], BusTestKind::Motion, 10);
    app.execute(ServoOperation::StartBusTest(request.clone()))
        .await
        .unwrap();
    until(
        &app,
        || app.snapshot().bus_test.unwrap().cycles >= 3,
        Duration::from_secs(1),
    )
    .await;
    let before_selection = model.lock().unwrap().frames.len();
    app.execute(ServoOperation::Select(14)).await.unwrap();
    assert!(model.lock().unwrap().frames[before_selection..]
        .iter()
        .all(|frame| frame[2] & 0x70 == 0x10));
    assert!(app
        .execute(ServoOperation::Send {
            target: ServoTarget::default(),
            repeat: true
        })
        .await
        .is_err());
    assert!(app
        .execute(ServoOperation::ChangeId {
            old_id: 14,
            new_id: 5
        })
        .await
        .is_err());
    {
        let model = model.lock().unwrap();
        let targets: Vec<_> = model
            .frames
            .iter()
            .filter(|frame| frame[2] & 0x70 == 0x10 && frame[4..16].iter().any(|byte| *byte != 0))
            .collect();
        assert!(targets.len() >= 9);
        for (index, frame) in targets.iter().enumerate() {
            let node = &request.nodes[index % 3];
            assert_eq!(
                **frame,
                control_packet(node.id, 1, true, node.target).unwrap()
            );
        }
    }
    let stopped = app.execute(ServoOperation::Stop).await.unwrap();
    assert!(!stopped.bus_test.as_ref().unwrap().running && stopped.stop_confirmed);
    assert!(stopped
        .bus_test
        .unwrap()
        .nodes
        .iter()
        .all(|node| node.stop_confirmed));
    assert!(model
        .lock()
        .unwrap()
        .nodes
        .values()
        .all(|node| node.mode == 0));
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn any_member_fault_stops_the_whole_motion_test() {
    let (app, model) = simulated(&[0, 3, 14]).await;
    app.execute(ServoOperation::StartBusTest(request(
        &[0, 3, 14],
        BusTestKind::Motion,
        10,
    )))
    .await
    .unwrap();
    model.lock().unwrap().nodes.get_mut(&3).unwrap().fault = 8;
    until(
        &app,
        || !app.snapshot().bus_test.unwrap().running,
        Duration::from_secs(1),
    )
    .await;
    let state = app.snapshot();
    assert_eq!(state.bus_test.unwrap().phase, "failed");
    assert!(state.stop_confirmed && state.pending_stop_ids.is_empty());
    assert!(model
        .lock()
        .unwrap()
        .nodes
        .values()
        .all(|node| node.mode == 0));
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn lost_member_does_not_prevent_stopping_other_members_and_can_recover() {
    let (app, model) = simulated(&[0, 3, 14]).await;
    app.execute(ServoOperation::StartBusTest(request(
        &[0, 3, 14],
        BusTestKind::Motion,
        10,
    )))
    .await
    .unwrap();
    model.lock().unwrap().nodes.get_mut(&3).unwrap().corrupt_crc = true;
    until(
        &app,
        || !app.snapshot().bus_test.unwrap().running,
        Duration::from_secs(1),
    )
    .await;
    let state = app.snapshot();
    assert_eq!(state.pending_stop_ids, vec![3]);
    assert!(!state.stop_confirmed);
    let report = state.bus_test.unwrap();
    assert_eq!(report.phase, "failed");
    assert!(report
        .nodes
        .iter()
        .filter(|node| node.id != 3)
        .all(|node| node.stop_confirmed));
    assert!(
        report
            .nodes
            .iter()
            .find(|node| node.id == 3)
            .unwrap()
            .crc_error_count
            >= 3
    );
    assert!(app.shutdown(false).await.is_err());
    assert!(app.execute(ServoOperation::SetMode(1)).await.is_err());
    model.lock().unwrap().nodes.get_mut(&3).unwrap().corrupt_crc = false;
    assert!(app
        .execute(ServoOperation::Stop)
        .await
        .unwrap()
        .pending_stop_ids
        .is_empty());
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn preflight_rejects_missing_members_and_invalid_requests_without_enabling_any() {
    let (app, model) = simulated(&[0, 3]).await;
    for ids in [&[0, 0][..], &[0, 15], &[0], &[0, 14]] {
        assert!(app
            .execute(ServoOperation::StartBusTest(request(
                ids,
                BusTestKind::Motion,
                10
            )))
            .await
            .is_err());
    }
    let mut excessive = request(&[0, 3], BusTestKind::Motion, 10);
    excessive.nodes[1].target.torque_nm = 0.6;
    assert!(app
        .execute(ServoOperation::StartBusTest(excessive))
        .await
        .is_err());
    assert!(model
        .lock()
        .unwrap()
        .frames
        .iter()
        .all(|frame| frame[2] & 0x70 == 0));
    assert!(app.snapshot().pending_stop_ids.is_empty());
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn idle_polling_keeps_all_discovered_devices_online() {
    let (app, _) = simulated(&[0, 3, 14]).await;
    app.execute(ServoOperation::Scan).await.unwrap();
    tokio::time::sleep(Duration::from_millis(900)).await;
    let state = app.snapshot();
    assert_eq!(state.devices.len(), 3);
    assert!(state.devices.iter().all(|device| device.online));
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn lost_client_heartbeat_stops_all_test_servos_and_records_the_reason() {
    let (app, model) = simulated(&[0, 3]).await;
    app.execute(ServoOperation::StartBusTest(request(
        &[0, 3],
        BusTestKind::Motion,
        10,
    )))
    .await
    .unwrap();
    tokio::time::sleep(Duration::from_millis(1800)).await;
    let state = app.snapshot();
    let report = state.bus_test.unwrap();
    assert_eq!(report.phase, "failed");
    assert!(report.error.unwrap().contains("heartbeat"));
    assert!(state.stop_confirmed && !state.motion_active);
    assert!(model
        .lock()
        .unwrap()
        .nodes
        .values()
        .all(|node| node.mode == 0));
    app.shutdown(false).await.unwrap();
}

#[tokio::test]
async fn timed_motion_completion_locks_all_members_before_reporting_completed() {
    let (app, model) = simulated(&[0, 3, 14]).await;
    app.execute(ServoOperation::StartBusTest(request(
        &[0, 3, 14],
        BusTestKind::Motion,
        1,
    )))
    .await
    .unwrap();
    until(
        &app,
        || !app.snapshot().bus_test.unwrap().running,
        Duration::from_secs(2),
    )
    .await;
    let state = app.snapshot();
    let report = state.bus_test.unwrap();
    assert_eq!(report.phase, "completed");
    assert!(state.stop_confirmed && !state.motion_active && state.pending_stop_ids.is_empty());
    assert!(report
        .nodes
        .iter()
        .all(|node| node.stop_confirmed && node.rx_count > 0));
    assert!(model
        .lock()
        .unwrap()
        .nodes
        .values()
        .all(|node| node.mode == 0));
    app.shutdown(false).await.unwrap();
}
