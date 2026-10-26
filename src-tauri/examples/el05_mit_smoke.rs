//! Unloaded standard-MIT verification through the GUI's production session.
//! cargo run --example el05_mit_smoke -- can0 1 --motion /tmp/el05-mit-smoke
//! Append --focused to run only breakaway and offset-gravity verification.
//! Use --capture-only instead of --motion for idle position/reference checks.
//! Exercises manual control, all eleven pendulum modes and gravity lifecycle.
//! The gravity check uses a tiny simulated load; it is not loaded compensation
//! metrology. Does not set a mechanical zero, change IDs or write parameters.

#[path = "../src/backend.rs"]
mod backend;
#[path = "../src/can_lease.rs"]
mod can_lease;
#[path = "../src/el05.rs"]
mod el05;

use anyhow::{bail, ensure, Context, Result};
use can_transport::{
    CanBus, CanBusState, CanCapabilities, CanFilter, CanFrame, CanId, CanIoError, CanLinkConfig,
    CanRx,
};
use el05::experiment::{ExperimentConfig, ExperimentData, ExperimentKind};
use el05::gravity::GravityCompensationConfig;
use el05::{El05Protocol, El05Session, El05State, El05Target};
use serde_json::{json, Value};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

#[derive(Clone, serde::Serialize)]
struct WireCommand {
    t_s: f64,
    can_id: u32,
    extended: bool,
    payload: Vec<u8>,
}

struct RecordingBus {
    inner: Arc<dyn CanBus>,
    epoch: Instant,
    sent: Mutex<Vec<WireCommand>>,
}

impl RecordingBus {
    fn new(inner: Arc<dyn CanBus>) -> Arc<Self> {
        Arc::new(Self {
            inner,
            epoch: Instant::now(),
            sent: Mutex::new(Vec::new()),
        })
    }
    fn cursor(&self) -> usize {
        self.sent.lock().unwrap().len()
    }
    fn since(&self, cursor: usize) -> Vec<WireCommand> {
        self.sent.lock().unwrap()[cursor..].to_vec()
    }
}

#[async_trait::async_trait]
impl CanBus for RecordingBus {
    async fn send(&self, frame: CanFrame) -> std::result::Result<(), CanIoError> {
        self.inner.send(frame.clone()).await?;
        self.sent.lock().unwrap().push(WireCommand {
            t_s: self.epoch.elapsed().as_secs_f64(),
            can_id: frame.id().raw(),
            extended: matches!(frame.id(), CanId::Extended(_)),
            payload: frame.data().to_vec(),
        });
        Ok(())
    }
    async fn subscribe(
        &self,
        filter: CanFilter,
    ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
        self.inner.subscribe(filter).await
    }
    fn capabilities(&self) -> CanCapabilities {
        self.inner.capabilities()
    }
    async fn bus_state(&self) -> std::result::Result<Option<CanBusState>, CanIoError> {
        self.inner.bus_state().await
    }
    async fn link_config(&self) -> std::result::Result<Option<CanLinkConfig>, CanIoError> {
        self.inner.link_config().await
    }
}

fn save(output: &Path, filename: &str, data: &impl serde::Serialize) -> Result<()> {
    std::fs::write(output.join(filename), serde_json::to_vec_pretty(data)?)?;
    Ok(())
}

fn healthy(state: &El05State, origin: f64) -> Result<()> {
    ensure!(state.online, "feedback went stale");
    ensure!(
        state.fault_bits == 0 && state.fault_code == 0 && state.warning_code == 0,
        "drive reported fault/warning: {:?}",
        state.faults
    );
    let temperature = state.temperature_c.context("missing temperature")?;
    ensure!(
        temperature.is_finite() && temperature < 60.0,
        "temperature {temperature} C"
    );
    let speed = state.velocity_rad_s.context("missing velocity")?;
    ensure!(
        speed.is_finite() && speed.abs() <= 3.0,
        "speed {speed} rad/s exceeds smoke limit"
    );
    let position = state.unwrapped_position_rad.context("missing position")?;
    ensure!(
        position.is_finite() && (position - origin).abs() <= 2.0,
        "excursion {} rad exceeds smoke limit",
        position - origin
    );
    Ok(())
}

async fn observe(app: &El05Session, duration: Duration, origin: f64) -> Result<Vec<El05State>> {
    let deadline = Instant::now() + duration;
    let mut states = Vec::new();
    while Instant::now() < deadline {
        tokio::time::sleep(Duration::from_millis(20)).await;
        app.touch_client();
        let state = app.snapshot();
        healthy(&state, origin)?;
        states.push(state);
    }
    Ok(states)
}

fn target(position: f64, velocity: f64, kp: f64, kd: f64) -> El05Target {
    El05Target::Mit {
        position_rad: position,
        velocity_rad_s: velocity,
        torque_nm: 0.0,
        kp,
        kd,
    }
}

fn config(kind: ExperimentKind) -> ExperimentConfig {
    ExperimentConfig {
        kind,
        duration_s: 2.0,
        velocity_rad_s: 0.5,
        torque_nm: 0.03,
        kp: 3.0,
        kd: 0.5,
        position_amplitude_rad: 0.08,
        frequency_start_hz: 0.1,
        frequency_end_hz: 0.5,
        disturbance_torque_nm: 0.03,
        max_temperature_c: 60.0,
        torque_ramp_rate_nm_s: 0.02,
        settle_time_s: 0.15,
        ..Default::default()
    }
}

fn check_recording(data: &ExperimentData, kind: ExperimentKind, motor_id: u8) -> Result<Value> {
    ensure!(
        data.protocol == El05Protocol::StandardMit && data.feedback_velocity_bits == 12,
        "recording protocol metadata is incorrect"
    );
    ensure!(
        data.samples.len() >= 20,
        "{kind:?}: only {} feedback samples",
        data.samples.len()
    );
    // The observer samples snapshots every 20 ms; a faster control loop can
    // report a transient between those snapshots. Validate every recorded CAN
    // sample against the same bounds instead of hiding a peak in an average.
    if let Some(sample) = data.samples.iter().find(|sample| {
        !sample.velocity_rad_s.is_finite()
            || sample.velocity_rad_s.abs() > 3.0
            || !sample.temperature_c.is_finite()
            || sample.temperature_c >= 60.0
            || sample.fault_bits != 0
    }) {
        bail!(
            "{kind:?}: full-rate feedback at {:.6}s exceeds smoke bounds: speed {} rad/s, temperature {} C, faults 0x{:02X}",
            sample.t_s, sample.velocity_rad_s, sample.temperature_c, sample.fault_bits
        );
    }
    ensure!(
        data.uid.is_none() && data.parameters.is_empty(),
        "invented native metadata"
    );
    ensure!(
        data.samples.iter().all(|s| s.payload[0] == motor_id
            && s.can_id <= 0x7FF
            && s.current_a.is_none()
            && s.dictionary_velocity_rad_s.is_none()),
        "recording contains non-MIT telemetry"
    );
    if kind != ExperimentKind::Record {
        ensure!(!data.commands.is_empty(), "{kind:?}: no recorded commands");
        ensure!(
            data.commands
                .iter()
                .all(|c| c.can_id == u32::from(motor_id)),
            "{kind:?}: command identifier is not the standard motor address"
        );
    }
    let first = data.samples.first().unwrap();
    let last = data.samples.last().unwrap();
    let duration = last.t_s - first.t_s;
    Ok(json!({
        "kind": kind, "samples": data.samples.len(), "commands": data.commands.len(),
        "feedback_hz": if duration > 0.0 { (data.samples.len()-1) as f64 / duration } else { 0.0 },
        "position_change_rad": last.unwrapped_position_rad-first.unwrapped_position_rad,
        "max_speed_rad_s": data.samples.iter().map(|s| s.velocity_rad_s.abs()).fold(0.0, f64::max),
        "max_temperature_c": data.samples.iter().map(|s| s.temperature_c).fold(0.0, f64::max),
        "reason": data.status.reason,
        "static_friction": data.status.static_friction,
    }))
}

async fn exercise(
    app: &El05Session,
    motor_id: u8,
    output: &Path,
    focused: bool,
    gravity_only: bool,
    wire: &RecordingBus,
) -> Result<Vec<Value>> {
    let mut results = Vec::new();
    if !focused {
        let origin = app
            .snapshot()
            .unwrapped_position_rad
            .context("initial position")?;
        app.enable().await?;
        ensure!(
            app.snapshot().enabled,
            "host control state did not enter enabled"
        );
        let mut manual = Vec::new();
        for velocity in [1.0, 0.0, -1.0, 0.0] {
            app.send_target(target(0.0, velocity, 0.0, 0.5), true)
                .await?;
            let states = observe(
                app,
                Duration::from_millis(if velocity == 0.0 { 250 } else { 600 }),
                origin,
            )
            .await?;
            manual.push(json!({"target_velocity_rad_s": velocity, "states": states}));
        }
        app.stop().await?;
        save(output, "manual.json", &manual)?;
        ensure!(
            !app.snapshot().enabled && !app.snapshot().streaming,
            "manual Stop not acknowledged"
        );
        results.push(json!({"check": "manual_bidirectional_control", "passed": true}));
        println!("PASS standard MIT manual positive/negative velocity and stop");
    }

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
        if gravity_only || (focused && kind != ExperimentKind::StaticFriction) {
            continue;
        }
        let origin = app
            .snapshot()
            .unwrapped_position_rad
            .context("experiment position")?;
        observe(app, Duration::from_millis(200), origin).await?;
        let mut experiment_config = config(kind);
        if focused && kind == ExperimentKind::StaticFriction {
            experiment_config.duration_s = 4.0;
            experiment_config.torque_nm = 0.25;
            experiment_config.torque_ramp_rate_nm_s = 0.12;
            experiment_config.max_excursion_rad = 0.1;
        }
        app.start_experiment(experiment_config).await?;
        let deadline = Instant::now()
            + Duration::from_secs(if kind == ExperimentKind::StaticFriction {
                30
            } else {
                4
            });
        let run_result = async {
            loop {
                app.touch_client();
                let state = app.snapshot();
                healthy(&state, origin)?;
                if !state.experiment.active {
                    break;
                }
                ensure!(
                    Instant::now() < deadline,
                    "{kind:?}: did not finish by the smoke deadline"
                );
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
            Ok::<_, anyhow::Error>(())
        }
        .await;
        let data = app.experiment_data();
        save(output, &format!("{kind:?}.json"), &data)?;
        let stopped = app.stop().await;
        run_result?;
        stopped?;
        let summary = check_recording(&data, kind, motor_id)?;
        let reason = data.status.reason.as_deref().unwrap_or("");
        ensure!(
            reason == "duration completed"
                || (kind == ExperimentKind::StaticFriction
                    && reason.starts_with("static friction calibration:")),
            "{kind:?}: unexpected completion: {reason}"
        );
        ensure!(
            !app.snapshot().enabled && !app.snapshot().streaming,
            "{kind:?}: failed to stop"
        );
        println!("PASS {}", serde_json::to_string(&summary)?);
        results.push(summary);
    }

    let origin = app
        .snapshot()
        .unwrapped_position_rad
        .context("gravity initial position")?;
    observe(app, Duration::from_millis(350), origin).await?;
    let reference = app.capture_gravity_reference().await?;
    if focused {
        // Keep the captured reference while placing the shaft off that reference.
        // The position command is expressed in the current wrapped coordinates.
        let wrapped = app
            .snapshot()
            .position_rad
            .context("gravity offset position")?;
        ensure!(
            wrapped + 0.2 <= 12.57,
            "gravity offset would cross the position boundary"
        );
        app.enable().await?;
        app.send_target(target(wrapped + 0.2, 0.0, 3.0, 0.5), true)
            .await?;
        let positioning = observe(app, Duration::from_millis(800), origin).await?;
        app.stop().await?;
        let settled = observe(app, Duration::from_millis(300), origin).await?;
        save(
            output,
            "gravity-offset.json",
            &json!({"reference": reference,
            "target_offset_rad": 0.2, "positioning": positioning, "settled": settled}),
        )?;
        ensure!(
            app.snapshot()
                .gravity_compensation
                .reference
                .as_ref()
                .map(|r| &r.id)
                == Some(&reference.id),
            "manual offset invalidated the captured gravity reference"
        );
        let offset = app
            .snapshot()
            .unwrapped_position_rad
            .context("gravity offset feedback")?
            - reference.position_rad;
        ensure!(
            offset.abs() > 0.01,
            "shaft did not move far enough from gravity reference: {offset} rad"
        );
    }
    // Small simulated mass exercises the live algorithm on an unloaded shaft.
    // Functional lifecycle is verified; actual loaded holding accuracy is not.
    let gravity = GravityCompensationConfig {
        reference_id: reference.id.clone(),
        mass_kg: if gravity_only {
            0.5
        } else if focused {
            0.1
        } else {
            0.03
        },
        length_m: 0.1,
        arm_mass_kg: 0.0,
        compensation_scale: 1.0,
        damping_nm_s_rad: if gravity_only { 0.4 } else { 0.1 },
        torque_limit_nm: if gravity_only {
            0.5
        } else if focused {
            0.12
        } else {
            0.05
        },
        max_velocity_rad_s: 2.0,
        max_temperature_c: 60.0,
        ramp_s: 0.5,
    };
    let wire_cursor = wire.cursor();
    app.start_gravity_compensation(gravity.clone()).await?;
    let states = observe(app, Duration::from_secs(1), origin).await?;
    let gravity_commands = wire.since(wire_cursor);
    ensure!(
        states
            .iter()
            .all(|s| s.gravity_compensation.active && s.enabled && s.last_error.is_none()),
        "gravity control stopped unexpectedly"
    );
    // Independently decode the actual successful standard transmissions, rather
    // than treating the intended torque shown in the UI as proof of wire output.
    let nonzero_feedforward: Vec<_> = gravity_commands
        .iter()
        .filter_map(|command| {
            let d = &command.payload;
            if command.extended
                || command.can_id != u32::from(motor_id)
                || d.len() != 8
                || d[..6] == [255; 6]
            {
                return None;
            }
            let kp = ((u16::from(d[3]) & 15) << 8) | u16::from(d[4]);
            let kd = (u16::from(d[5]) << 4) | (u16::from(d[6]) >> 4);
            let torque_raw = ((u16::from(d[6]) & 15) << 8) | u16::from(d[7]);
            (kp == 0 && kd == 0 && !(2047..=2048).contains(&torque_raw))
                .then_some(f64::from(torque_raw) * 12.0 / 4095.0 - 6.0)
        })
        .collect();
    save(
        output,
        "gravity.json",
        &json!({"reference": reference, "config": gravity, "states": states,
        "wire_commands": gravity_commands, "nonzero_feedforward_nm": nonzero_feedforward,
        "scope": "unloaded functional control-path test, not loaded compensation accuracy"}),
    )?;
    app.stop_gravity_compensation().await?;
    if gravity_only {
        let angles: Vec<_> = states
            .iter()
            .filter_map(|s| s.gravity_compensation.angle_rad)
            .collect();
        let low = angles.iter().copied().fold(f64::INFINITY, f64::min);
        let high = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        // Several encoder counts prove a changing angle; do not demand an
        // arbitrary travel distance from a small, friction-limited perturbation.
        ensure!(
            high - low > 3.0 * 25.14 / 65535.0,
            "gravity angle stayed constant during the small torque perturbation"
        );
        results.push(
            json!({"check": "live_gravity_angle_changes", "passed": true,
            "min_angle_rad": low, "max_angle_rad": high}),
        );
    }
    if focused {
        ensure!(
            states.iter().any(|s| s
                .gravity_compensation
                .angle_rad
                .is_some_and(|a| a.abs() > 0.01)),
            "gravity controller did not observe the displaced reference"
        );
        ensure!(
            !nonzero_feedforward.is_empty(),
            "no nonzero pure feedforward torque reached CAN"
        );
        results.push(
            json!({"check": "gravity_nonzero_wire_feedforward", "passed": true,
            "command_count": nonzero_feedforward.len(),
            "peak_abs_torque_nm": nonzero_feedforward.iter().map(|v| v.abs()).fold(0.0, f64::max)}),
        );
    }
    ensure!(
        !app.snapshot().gravity_compensation.active && !app.snapshot().enabled,
        "gravity stop failed"
    );
    results.push(json!({"check": "gravity_capture_start_stream_stop", "passed": true}));
    println!("PASS standard MIT gravity reference/start/stream/stop without UID or dictionary");

    if gravity_only {
        return Ok(results);
    }

    // Reuse the same current-session reference, then expire the actual UI lease.
    app.start_gravity_compensation(gravity.clone()).await?;
    app.touch_client();
    let deadline = Instant::now() + Duration::from_secs(4);
    while app.snapshot().gravity_compensation.active {
        healthy(&app.snapshot(), origin)?;
        ensure!(
            Instant::now() < deadline,
            "gravity UI heartbeat expiry failed"
        );
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    app.stop().await?;
    ensure!(
        !app.snapshot().enabled && !app.snapshot().streaming,
        "heartbeat expiry did not stop control"
    );
    save(output, "heartbeat-expired.json", &app.snapshot())?;
    results.push(json!({"check": "gravity_ui_heartbeat_stop", "passed": true}));
    println!("PASS gravity UI heartbeat expiry stopped the standard-MIT drive");
    app.attach(motor_id).await?;
    ensure!(
        app.snapshot().gravity_compensation.reference.is_none(),
        "reattach kept the old reference"
    );
    ensure!(
        app.start_gravity_compensation(gravity).await.is_err(),
        "old reference accepted after reattach"
    );
    results.push(json!({"check": "reference_invalid_after_reattach", "passed": true}));
    Ok(results)
}

async fn capture_only(
    app: &El05Session,
    output: &Path,
    wire: &RecordingBus,
    reset_zero: bool,
) -> Result<Vec<Value>> {
    let before = app.snapshot();
    let origin = before
        .unwrapped_position_rad
        .context("idle initial position")?;
    let states = observe(app, Duration::from_millis(500), origin).await?;
    let after = app.snapshot();
    save(output, "idle-position.json", &states)?;
    ensure!(
        after.feedback_sequence >= before.feedback_sequence + 5,
        "idle Stop polling did not produce continuous feedback"
    );
    ensure!(
        states.iter().all(|s| !s.enabled
            && !s.streaming
            && s.position_rad.is_some()
            && s.unwrapped_position_rad.is_some()
            && s.feedback_age_ms.is_some_and(|age| age <= 150)),
        "idle feedback was missing, stale, or unexpectedly enabled"
    );
    let reference = app.capture_gravity_reference().await?;
    save(output, "captured-reference.json", &reference)?;
    ensure!(
        wire.since(0).iter().all(|command| !command.extended
            && (command.payload == [255, 255, 255, 255, 255, 255, 255, 253]
                || (reset_zero && command.payload == [255, 255, 255, 255, 255, 255, 255, 254]))),
        "capture-only check sent a command other than standard Stop or explicitly requested zero"
    );
    println!(
        "PASS idle position and reference capture without enable: {}",
        serde_json::to_string(&reference)?
    );
    Ok(vec![
        json!({"check": "idle_position_and_reference_capture", "passed": true,
        "feedback_frames": after.feedback_sequence - before.feedback_sequence,
        "position_rad": after.position_rad, "unwrapped_position_rad": after.unwrapped_position_rad,
        "reference": reference, "transmit_policy": if reset_zero { "standard Stop and explicit zero only; no enable or motion targets" } else { "standard Stop only; no enable or motion targets" }}),
    ])
}

#[tokio::main]
async fn main() -> Result<()> {
    let mut args: Vec<_> = std::env::args().collect();
    let reset_zero = args.last().is_some_and(|arg| arg == "--reset-zero");
    if reset_zero {
        args.pop();
    }
    if !(4..=6).contains(&args.len()) || !matches!(args[3].as_str(), "--motion" | "--capture-only")
    {
        bail!("Usage: el05_mit_smoke <interface> <motor-id> --motion [output-directory] [--focused], or --capture-only [output-directory]");
    }
    let capture_only_mode = args[3] == "--capture-only";
    ensure!(
        !reset_zero || capture_only_mode,
        "--reset-zero is only available with --capture-only"
    );
    let gravity_only = args.last().is_some_and(|arg| arg == "--gravity-only");
    let focused = gravity_only || args.last().is_some_and(|arg| arg == "--focused");
    ensure!(
        !capture_only_mode || (!focused && args.len() <= 5),
        "--capture-only accepts only an optional output directory"
    );
    ensure!(
        args.len() != 6 || focused,
        "last argument must be --focused"
    );
    ensure!(
        !args[4..]
            .iter()
            .take(args[4..].len().saturating_sub(usize::from(focused)))
            .any(|arg| arg.starts_with('-')),
        "unknown option"
    );
    let motor_id: u8 = args[2].parse().context("motor ID must be 0..255")?;
    let output = Path::new(
        args.get(4)
            .filter(|arg| !matches!(arg.as_str(), "--focused" | "--gravity-only"))
            .map(String::as_str)
            .unwrap_or(if capture_only_mode {
                "/tmp/el05-mit-capture"
            } else {
                "/tmp/el05-mit-smoke"
            }),
    );
    std::fs::create_dir_all(output)?;
    let gate = can_lease::CanTransportGate::default();
    let permit = gate
        .try_acquire(can_lease::CanOwner::El05)
        .map_err(anyhow::Error::msg)?;
    let bus = backend::open_classic_1m_bus(&args[1], permit).await?;
    let wire = RecordingBus::new(backend::protect_feature_bus(bus));
    let app =
        El05Session::start_with_protocol(wire.clone(), 253, El05Protocol::StandardMit).await?;
    let result = async {
        app.discover().await?;
        let deadline = Instant::now() + Duration::from_secs(7);
        while !app.devices().iter().any(|d| d.motor_id == motor_id) {
            ensure!(
                Instant::now() < deadline,
                "motor {motor_id} was not discovered"
            );
            app.touch_client();
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let state = app.attach(motor_id).await?;
        ensure!(
            state.protocol == El05Protocol::StandardMit
                && state.reply_id.is_some()
                && !state.status_reliable,
            "incorrect standard-MIT session metadata"
        );
        save(output, "attached.json", &state)?;
        println!("ATTACHED {}", serde_json::to_string(&state)?);
        if capture_only_mode {
            if reset_zero {
                app.set_zero().await?;
                save(output, "after-zero.json", &app.snapshot())?;
            }
            capture_only(&app, output, &wire, reset_zero).await
        } else {
            exercise(&app, motor_id, output, focused, gravity_only, &wire).await
        }
    }
    .await;
    // Always attempt Stop and shutdown, including failed discovery/preparation.
    let stopped = if app.snapshot().motor_id.is_some() {
        app.stop().await
    } else {
        Ok(())
    };
    let shutdown = app.shutdown().await;
    if shutdown.is_err() {
        app.force_stop();
    }
    save(output, "wire-commands.json", &wire.since(0))?;
    save(output, "final-state.json", &app.snapshot())?;
    save(
        output,
        "summary.json",
        &json!({
        "interface": args[1], "motor_id": motor_id, "protocol": "StandardMit", "focused": focused,
        "capture_only": capture_only_mode, "reset_zero": reset_zero, "gravity_only": gravity_only,
            "passed": result.is_ok() && stopped.is_ok() && shutdown.is_ok(),
            "checks": result.as_ref().ok(), "error": result.as_ref().err().map(ToString::to_string),
            "stop_error": stopped.as_ref().err().map(ToString::to_string),
            "shutdown_error": shutdown.as_ref().err().map(ToString::to_string),
            "scope": "unloaded protocol and control-path validation; actual pendulum compensation accuracy requires a mounted load"
        }),
    )?;
    result?;
    stopped?;
    shutdown?;
    println!(
        "PASS standard MIT {} and final shutdown",
        if capture_only_mode {
            "idle position and downward-reference capture"
        } else if gravity_only {
            "small-offset gravity feedback"
        } else if focused {
            "focused static-friction and offset-gravity control"
        } else {
            "manual control, all pendulum modes, gravity lifecycle"
        }
    );
    Ok(())
}
