//! Identity-only CAN diagnosis using the GUI's production EL05 discovery session.
//! cargo run --example el05_probe -- can0 can1 can2 can3
//! Emits one JSON object per interface. SocketCAN timing is system-managed;
//! this program never changes interface configuration or selects a motor.

#[path = "../src/backend.rs"]
mod backend;
#[path = "../src/can_lease.rs"]
mod can_lease;
#[path = "../src/el05.rs"]
mod el05;

use async_trait::async_trait;
use can_transport::{
    CanBus, CanBusState, CanCapabilities, CanFilter, CanFrame, CanId, CanIoError, CanLinkConfig,
    CanRx, FrameKind,
};
use serde_json::{json, Value};
use std::collections::BTreeSet;
use std::process::ExitCode;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const HOST_ID: u8 = 253;
const SCAN_DEADLINE: Duration = Duration::from_secs(8);
const REPLY_SETTLE: Duration = Duration::from_millis(300);

#[derive(Default)]
struct ProbeCounters {
    attempted_ids: BTreeSet<u8>,
    successful_ids: BTreeSet<u8>,
    attempts: u64,
    successes: u64,
    rejected: u64,
    send_errors: Vec<String>,
}

/// Enforce the identity-only contract even if the production service changes.
/// No stop/enable/parameter/motion frame can reach the physical transport.
struct IdentityOnlyBus {
    inner: Arc<dyn CanBus>,
    counters: Arc<Mutex<ProbeCounters>>,
}

#[async_trait]
impl CanBus for IdentityOnlyBus {
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
        let destination = match frame.id() {
            CanId::Extended(id)
                if id >> 24 == 0
                    && (id >> 8) & 0xFFFF == u32::from(HOST_ID)
                    && frame.kind() == FrameKind::Data
                    && !frame.is_fd()
                    && frame.data() == [0; 8] =>
            {
                id as u8
            }
            _ => {
                self.counters.lock().unwrap().rejected += 1;
                return Err(CanIoError::backend(std::io::Error::new(
                    std::io::ErrorKind::PermissionDenied,
                    "el05_probe permits only RobStride type-0 UID requests",
                )));
            }
        };
        {
            let mut counters = self.counters.lock().unwrap();
            counters.attempts += 1;
            counters.attempted_ids.insert(destination);
        }
        let result = self.inner.send(frame).await;
        let mut counters = self.counters.lock().unwrap();
        match &result {
            Ok(()) => {
                counters.successes += 1;
                counters.successful_ids.insert(destination);
            }
            Err(error) => {
                // Keep every failing ID without unbounded duplicate text.
                counters
                    .send_errors
                    .push(format!("motor ID {destination}: {error}"));
            }
        }
        result
    }

    async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
        self.inner.subscribe(filter).await
    }

    fn capabilities(&self) -> CanCapabilities {
        self.inner.capabilities()
    }

    async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
        self.inner.bus_state().await
    }

    async fn link_config(&self) -> Result<Option<CanLinkConfig>, CanIoError> {
        self.inner.link_config().await
    }
}

fn state_json(state: Option<CanBusState>) -> Value {
    state.map_or(Value::Null, |state| {
        json!({
            "state": state.state.map(|state| format!("{state:?}")),
            "tx_errors": state.tx_errors,
            "rx_errors": state.rx_errors,
        })
    })
}

fn config_json(config: Option<CanLinkConfig>) -> Value {
    config.map_or(Value::Null, |config| {
        let timing = |timing: can_transport::CanBitTiming| {
            json!({
                "bitrate": timing.bitrate,
                "sample_point_per_mille": timing.sample_point_per_mille,
            })
        };
        json!({
            "fd_enabled": config.fd_enabled,
            "nominal": config.nominal.map(timing),
            "data": config.data.map(timing),
        })
    })
}

fn record_error(report: &mut Value, stage: &str, error: impl std::fmt::Display) {
    report["errors"]
        .as_array_mut()
        .unwrap()
        .push(json!({ "stage": stage, "message": error.to_string() }));
}

async fn probe(interface: &str) -> Value {
    let started = Instant::now();
    let mut report = json!({
        "interface": interface,
        "backend": backend::backend_name(interface),
        "host_id": HOST_ID,
        "transmit_policy": "RobStride type-0 UID requests only; IDs 0..255",
        "socketcan_timing": "system-managed, not changed by this program",
        "config": null,
        "bus_state_start": null,
        "bus_state_end": null,
        "devices": [],
        "scan": null,
        "status": "error",
        "errors": [],
    });
    let gate = can_lease::CanTransportGate::default();
    let permit = match gate.try_acquire(can_lease::CanOwner::El05) {
        Ok(permit) => permit,
        Err(error) => {
            record_error(&mut report, "acquire", error);
            return report;
        }
    };
    let bus = match backend::open_classic_1m_bus(interface, permit).await {
        Ok(bus) => backend::protect_feature_bus(bus),
        Err(error) => {
            record_error(&mut report, "open", format!("{error:#}"));
            return report;
        }
    };
    match bus.link_config().await {
        Ok(config) => report["config"] = config_json(config),
        Err(error) => record_error(&mut report, "link_config", error),
    }
    match bus.bus_state().await {
        Ok(state) => report["bus_state_start"] = state_json(state),
        Err(error) => record_error(&mut report, "bus_state_start", error),
    }
    let counters = Arc::new(Mutex::new(ProbeCounters::default()));
    let guarded_bus = Arc::new(IdentityOnlyBus {
        inner: bus.clone(),
        counters: counters.clone(),
    });
    match el05::El05Session::start(guarded_bus, HOST_ID).await {
        Ok(session) => {
            if let Err(error) = session.discover().await {
                record_error(&mut report, "discover", format!("{error:#}"));
            } else {
                let deadline = Instant::now() + SCAN_DEADLINE;
                loop {
                    let all_attempted = {
                        let counters = counters.lock().unwrap();
                        counters.attempted_ids.len() == 256
                            && counters.attempts
                                == counters.successes + counters.send_errors.len() as u64
                    };
                    if all_attempted {
                        tokio::time::sleep(REPLY_SETTLE).await;
                        break;
                    }
                    if Instant::now() >= deadline {
                        record_error(
                            &mut report,
                            "scan",
                            "scan deadline reached before all 256 IDs completed",
                        );
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(25)).await;
                }
            }
            // No motor was selected. force_stop only cancels local tasks and
            // does not transmit a motor Stop or change periodic reporting.
            session.force_stop();
            let snapshot = session.snapshot();
            report["devices"] = json!(snapshot.devices);
            if let Some(error) = snapshot.last_error {
                record_error(&mut report, "session", error);
            }
        }
        Err(error) => record_error(&mut report, "start", format!("{error:#}")),
    }
    match bus.bus_state().await {
        Ok(state) => report["bus_state_end"] = state_json(state),
        Err(error) => record_error(&mut report, "bus_state_end", error),
    }
    let counters = counters.lock().unwrap();
    let complete = counters.successful_ids.len() == 256;
    report["scan"] = json!({
        "complete": complete,
        "elapsed_ms": started.elapsed().as_millis(),
        "requested_id_range": [0, 255],
        "unique_ids_attempted": counters.attempted_ids.len(),
        "unique_ids_sent": counters.successful_ids.len(),
        "query_frames_attempted": counters.attempts,
        "query_frames_sent": counters.successes,
        "rejected_non_query_frames": counters.rejected,
        "missing_ids": (0..=255u8).filter(|id| !counters.successful_ids.contains(id)).collect::<Vec<_>>(),
        "send_errors": counters.send_errors,
    });
    report["status"] = json!(if !report["errors"].as_array().unwrap().is_empty()
        || !counters.send_errors.is_empty()
        || counters.rejected > 0
    {
        "error"
    } else if !complete {
        "incomplete"
    } else if report["devices"].as_array().unwrap().is_empty() {
        "no_response"
    } else {
        "discovered"
    });
    report
}

#[tokio::main]
async fn main() -> ExitCode {
    let interfaces: Vec<String> = std::env::args().skip(1).collect();
    if interfaces.is_empty()
        || interfaces
            .iter()
            .any(|interface| interface.starts_with('-'))
    {
        eprintln!("Usage: el05_probe <interface> [interface ...]\nExample: el05_probe can0 can1 can2 can3\nOnly sends type-0 UID queries; does not attach, stop, enable, or write parameters.\nOutputs one JSON object per interface; exit 1 for scan/transport errors, 2 for usage errors.");
        return ExitCode::from(
            if interfaces.iter().any(|arg| arg == "--help" || arg == "-h") {
                0
            } else {
                2
            },
        );
    }
    let mut failed = false;
    // Serial scans avoid reopening sibling channels of one physical USB
    // adapter concurrently. A healthy full scan takes about 5.5 seconds.
    for interface in interfaces {
        let report = probe(&interface).await;
        failed |= report["status"] == "error" || report["status"] == "incomplete";
        println!("{report}");
    }
    ExitCode::from(u8::from(failed))
}
