//! Bounded, individually addressed tests on a shared single-wire serial bus.
use super::*;

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum BusTestKind {
    Communication,
    Motion,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BusTestNode {
    pub id: u8,
    pub target: ServoTarget,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BusTestRequest {
    pub kind: BusTestKind,
    pub duration_s: u16,
    pub nodes: Vec<BusTestNode>,
}

impl BusTestRequest {
    pub fn validate(&self, settings: ServoSettings) -> Result<()> {
        ensure!(
            (2..=15).contains(&self.nodes.len()),
            "Select 2–15 servos for a multi-servo test"
        );
        ensure!(
            (1..=120).contains(&self.duration_s),
            "Test duration must be 1–120 seconds"
        );
        let mut unique = BTreeSet::new();
        for node in &self.nodes {
            validate_id(node.id)?;
            ensure!(unique.insert(node.id), "Duplicate servo ID {}", node.id);
            node.target.validate()?;
            if self.kind == BusTestKind::Motion {
                settings.validate_target(node.target)?;
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct BusTestNodeResult {
    pub id: u8,
    pub target: ServoTarget,
    pub tx_count: u64,
    pub rx_count: u64,
    pub timeout_count: u64,
    pub crc_error_count: u64,
    pub consecutive_failures: u8,
    pub mean_rtt_ms: f64,
    pub max_rtt_ms: f64,
    pub stop_confirmed: bool,
    pub last_error: Option<String>,
    pub feedback: Option<ServoFeedback>,
}

#[derive(Clone, Debug, Serialize)]
pub struct BusTestReport {
    pub kind: BusTestKind,
    pub phase: String,
    pub running: bool,
    pub duration_s: u16,
    pub started_at_s: f64,
    pub elapsed_s: f64,
    pub cycles: u64,
    pub overruns: u64,
    pub nodes: Vec<BusTestNodeResult>,
    pub error: Option<String>,
}

pub(super) struct ActiveBusTest {
    pub request: BusTestRequest,
    pub started: Instant,
}

impl Worker {
    pub(super) fn start_bus_test(&mut self, request: BusTestRequest) -> Result<()> {
        self.require_idle()?;
        let settings = self.shared.lock().unwrap().state.settings;
        request.validate(settings)?;
        {
            let mut shared = self.shared.lock().unwrap();
            shared.state.bus_test = Some(BusTestReport {
                kind: request.kind,
                phase: "starting".into(),
                running: true,
                duration_s: request.duration_s,
                started_at_s: shared.started_at.elapsed().as_secs_f64(),
                elapsed_s: 0.0,
                cycles: 0,
                overruns: 0,
                error: None,
                nodes: request
                    .nodes
                    .iter()
                    .map(|node| BusTestNodeResult {
                        id: node.id,
                        target: node.target,
                        tx_count: 0,
                        rx_count: 0,
                        timeout_count: 0,
                        crc_error_count: 0,
                        consecutive_failures: 0,
                        mean_rtt_ms: 0.0,
                        max_rtt_ms: 0.0,
                        stop_confirmed: false,
                        last_error: None,
                        feedback: None,
                    })
                    .collect(),
            });
            shared.state.last_error = None;
        }
        self.bus_test = Some(ActiveBusTest {
            request: request.clone(),
            started: Instant::now(),
        });
        let prepared = (|| -> Result<()> {
            // Check every member before enabling any member. Each command has
            // exactly one intended responder; broadcast ID 15 is never used.
            for node in &request.nodes {
                let feedback = self
                    .transition_mode(node.id, 0, false)
                    .with_context(|| format!("ID {} preflight failed", node.id))?;
                if request.kind == BusTestKind::Motion {
                    Self::check_servo_health(&feedback)?;
                }
            }
            if request.kind == BusTestKind::Motion {
                let enabling_started = Instant::now();
                for node in &request.nodes {
                    ensure!(
                        enabling_started.elapsed() < Duration::from_millis(500),
                        "Bus enable sequence exceeded 500 ms"
                    );
                    self.pending_stop.insert(node.id);
                    {
                        let mut shared = self.shared.lock().unwrap();
                        shared.state.motion_active = true;
                        shared.state.stop_confirmed = false;
                        shared.state.pending_stop_ids = self.pending_stop.iter().copied().collect();
                    }
                    self.transition_mode(node.id, 1, true)
                        .with_context(|| format!("ID {} FOC enable failed", node.id))?;
                }
            }
            Ok(())
        })();
        if let Err(error) = prepared {
            let detail = format!("{error:#}");
            self.finish_bus_test("failed", Some(detail.clone()));
            bail!("{detail}");
        }
        let first_id = request.nodes[0].id;
        self.bus_test.as_mut().unwrap().started = Instant::now();
        let mut shared = self.shared.lock().unwrap();
        shared.state.selected_id = Some(first_id);
        shared.state.feedback = shared.devices.get(&first_id).map(|(_, f)| f.clone());
        shared.feedback_at = shared.devices.get(&first_id).map(|(at, _)| *at);
        shared.state.feedback_time_s = shared.devices.get(&first_id).map(|(at, _)| {
            at.saturating_duration_since(shared.started_at)
                .as_secs_f64()
        });
        shared.state.feedback_sequence += 1;
        shared.state.streaming = true;
        let started_at_s = shared.started_at.elapsed().as_secs_f64();
        let report = shared.state.bus_test.as_mut().unwrap();
        report.started_at_s = started_at_s;
        report.phase = "running".into();
        Ok(())
    }

    pub(super) fn tick_bus_test(&mut self) {
        let test = self.bus_test.as_ref().unwrap();
        if test.started.elapsed() >= Duration::from_secs(test.request.duration_s as u64) {
            self.finish_bus_test("completed", None);
            return;
        }
        let request = test.request.clone();
        let cycle_start = Instant::now();
        let period = self.period();
        for (index, node) in request.nodes.iter().enumerate() {
            let started = Instant::now();
            let (before_crc, before_tx) = {
                let shared = self.shared.lock().unwrap();
                (shared.state.crc_error_count, shared.state.tx_count)
            };
            let mode = u8::from(request.kind == BusTestKind::Motion);
            // The one-second hardware watchdog is always enabled in multi-FOC
            // tests, independently of the single-servo session preference.
            let result = self.exchange(
                node.id,
                mode,
                mode == 1,
                if mode == 1 {
                    node.target
                } else {
                    ServoTarget::default()
                },
            );
            let rtt = started.elapsed().as_secs_f64() * 1000.0;
            let mut fatal = None;
            {
                let mut shared = self.shared.lock().unwrap();
                let crc_delta = shared.state.crc_error_count - before_crc;
                let tx_delta = shared.state.tx_count - before_tx;
                let report = shared.state.bus_test.as_mut().unwrap();
                let stats = &mut report.nodes[index];
                stats.tx_count += tx_delta;
                stats.crc_error_count += crc_delta;
                match &result {
                    Ok(feedback) => {
                        stats.feedback = Some(feedback.clone());
                        stats.rx_count += 1;
                        stats.mean_rtt_ms += (rtt - stats.mean_rtt_ms) / stats.rx_count as f64;
                        stats.max_rtt_ms = stats.max_rtt_ms.max(rtt);
                        stats.consecutive_failures = 0;
                        stats.last_error = None;
                        if request.kind == BusTestKind::Motion {
                            if let Err(error) = Self::check_motion_feedback(feedback.clone()) {
                                fatal = Some(format!("ID {}: {error:#}", node.id));
                            }
                        } else if feedback.mode != 0 || feedback.timed_out {
                            fatal = Some(format!(
                                "ID {} did not remain locked (mode {}, timeout {})",
                                node.id, feedback.mode, feedback.timed_out
                            ));
                        }
                    }
                    Err(error) => {
                        stats.consecutive_failures += 1;
                        if error.downcast_ref::<NoServoResponse>().is_some() {
                            stats.timeout_count += 1;
                        }
                        stats.last_error = Some(format!("{error:#}"));
                        if stats.consecutive_failures >= 3
                            || self.stop_epoch.load(Ordering::Acquire) != self.epoch
                            || error.downcast_ref::<NoServoResponse>().is_none()
                        {
                            fatal = Some(format!("ID {}: {error:#}", node.id));
                        }
                    }
                }
            }
            if cycle_start.elapsed() >= Duration::from_millis(500) {
                fatal = Some(
                    "Bus cycle exceeded 500 ms; stopping all servos before watchdog expiry".into(),
                );
            }
            if let Some(error) = fatal {
                self.finish_bus_test("failed", Some(error));
                return;
            }
        }
        let mut shared = self.shared.lock().unwrap();
        let report = shared.state.bus_test.as_mut().unwrap();
        report.cycles += 1;
        if cycle_start.elapsed() > period {
            report.overruns += 1;
        }
    }

    pub(super) fn finish_bus_test(&mut self, phase: &str, error: Option<String>) {
        let stop_error = self.stop().err().map(|e| format!("{e:#}"));
        let combined = match (error, stop_error) {
            (Some(a), Some(b)) => Some(format!("{a}; {b}")),
            (Some(a), None) | (None, Some(a)) => Some(a),
            (None, None) => None,
        };
        let mut shared = self.shared.lock().unwrap();
        if let Some(report) = shared.state.bus_test.as_mut() {
            report.phase = if combined.is_some() {
                "failed".into()
            } else {
                phase.into()
            };
            report.error = combined.clone();
        }
        shared.state.last_error = combined;
    }
}
