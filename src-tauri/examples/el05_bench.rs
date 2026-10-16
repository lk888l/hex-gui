//! No-load, attended MIT speed verification using the GUI's production session.
//! el05_bench can3 1 --motion <output-directory>
//! Writes raw receive-frame JSON and speed consistency results, then restores
//! volatile settings and the original mode. Does not change the mechanical zero.
#[path = "../src/backend.rs"]
mod backend;
#[path = "../src/can_lease.rs"]
mod can_lease;
#[path = "../src/el05.rs"]
mod el05;

use anyhow::{bail, ensure, Context, Result};
use el05::experiment::{ExperimentConfig, ExperimentData, ExperimentKind};
use el05::{El05Mode, El05Session};
use serde_json::json;
use std::path::Path;
use std::time::Duration;

fn speed_checks(data: &ExperimentData) -> Vec<serde_json::Value> {
    let config = data.status.config.as_ref().unwrap();
    [(0.23, 0.39), (0.63, 0.79)].into_iter().filter_map(|(start, end)| {
        let points: Vec<_> = data.samples.iter().filter(|s| (config.duration_s * start..config.duration_s * end).contains(&s.t_s)).collect();
        if points.len() < 5 { return None; }
        let n = points.len() as f64;
        let t = points.iter().map(|s| s.t_s).sum::<f64>() / n;
        let p = points.iter().map(|s| s.unwrapped_position_rad).sum::<f64>() / n;
        let variance = points.iter().map(|s| (s.t_s - t).powi(2)).sum::<f64>();
        let slope = points.iter().map(|s| (s.t_s - t) * (s.unwrapped_position_rad - p)).sum::<f64>() / variance;
        let reported = points.iter().map(|s| s.velocity_rad_s).sum::<f64>() / n;
        let iq: Vec<_> = points.iter().filter(|s| s.current_age_ms.is_some_and(|age| age <= 100)).filter_map(|s| s.current_a).collect();
        let iq_mean = (!iq.is_empty()).then(|| iq.iter().sum::<f64>() / iq.len() as f64);
        // Use each dictionary reply once, rather than treating held values as
        // independently sampled at the faster motion feedback rate.
        let mut dictionary = Vec::new();
        let mut last_age = None;
        for sample in &points {
            if let (Some(age), Some(speed)) = (sample.dictionary_velocity_age_ms, sample.dictionary_velocity_rad_s) {
                if age <= 100 && last_age.is_none_or(|old| age < old) { dictionary.push(speed); }
                last_age = Some(age);
            }
        }
        let dictionary_mean = (!dictionary.is_empty()).then(|| dictionary.iter().sum::<f64>() / dictionary.len() as f64);
        let times: Vec<_> = data.commands.iter().filter(|command| (config.duration_s * start..config.duration_s * end).contains(&command.completed_t_s)).map(|command| command.completed_t_s).collect();
        let tx_rate = (times.len() >= 2).then(|| (times.len() - 1) as f64 / (times.last().unwrap() - times[0]));
        let receive_rate = (points.len() - 1) as f64 / (points.last().unwrap().t_s - points[0].t_s);
        Some(json!({ "window_s": [start * config.duration_s, end * config.duration_s], "frames": points.len(),
            "command_rad_s": if start < 0.5 { config.velocity_rad_s } else { -config.velocity_rad_s },
            "feedback_mean_rad_s": reported, "position_regression_rad_s": slope,
            "feedback_minus_position_rad_s": reported - slope, "fresh_iq_mean_a": iq_mean,
            "dictionary_velocity_mean_rad_s": dictionary_mean, "dictionary_samples": dictionary.len(),
            "received_frame_rate_hz": receive_rate, "mit_command_rate_hz": tx_rate,
            "speed_mapping_consistent": (reported - slope).abs() <= 0.02 + 0.05 * slope.abs(),
            "note": "Internal consistency against the same encoder, not independent metrology or velocity tracking accuracy" }))
    }).collect()
}

async fn bench(app: &El05Session, output: &Path) -> Result<()> {
    app.set_mode(El05Mode::Mit).await?;
    // Ten millisecond periodic feedback, bounded no-load torque. These are
    // volatile and restored by main even when a trial fails.
    app.write_parameter(0x7026, 1.0).await?;
    app.write_parameter(0x700B, 0.6).await?;
    let mut results = Vec::new();
    for (index, (velocity, kd)) in [(0.3, 0.1), (0.3, 0.5), (1.0, 0.5), (1.0, 1.0), (2.0, 1.0)]
        .into_iter()
        .enumerate()
    {
        tokio::time::sleep(Duration::from_millis(600)).await;
        app.touch_client();
        app.enable().await?;
        tokio::time::sleep(Duration::from_millis(100)).await;
        let config = ExperimentConfig {
            kind: ExperimentKind::VelocitySteps,
            duration_s: 6.0,
            velocity_rad_s: velocity,
            torque_nm: 0.0,
            kd,
            max_temperature_c: 60.0,
            pendulum: None,
            ..Default::default()
        };
        app.start_experiment(config).await?;
        loop {
            tokio::time::sleep(Duration::from_millis(40)).await;
            app.touch_client();
            let state = app.snapshot();
            if !state.experiment.active {
                break;
            }
        }
        app.stop().await?;
        let data = app.experiment_data();
        std::fs::write(
            output.join(format!("velocity-{index}.json")),
            serde_json::to_vec_pretty(&data)?,
        )?;
        ensure!(
            data.status.reason.as_deref() == Some("duration completed"),
            "trial ended early: {:?}",
            data.status.reason
        );
        let checks = speed_checks(&data);
        ensure!(
            checks.len() == 2,
            "not enough feedback for both velocity plateaus"
        );
        let trial = json!({ "velocity_rad_s": velocity, "kd": kd, "samples": data.samples.len(), "checks": checks });
        println!("TRIAL {}", serde_json::to_string(&trial)?);
        results.push(trial);
    }
    // Verify the same timed torque-ramp and passive recording paths exposed
    // by the GUI. Movement no longer ends a torque ramp early.
    for (name, kind, torque) in [
        ("ramp-positive", ExperimentKind::TorqueRamp, 0.15),
        ("ramp-negative", ExperimentKind::TorqueRamp, -0.15),
        ("passive", ExperimentKind::Record, 0.0),
    ] {
        tokio::time::sleep(Duration::from_millis(600)).await;
        app.touch_client();
        if kind != ExperimentKind::Record {
            app.enable().await?;
        }
        let duration_s = if kind == ExperimentKind::Record {
            2.0
        } else {
            6.0
        };
        app.start_experiment(ExperimentConfig {
            kind,
            duration_s,
            velocity_rad_s: 0.3,
            torque_nm: torque,
            kd: 0.0,
            max_temperature_c: 60.0,
            pendulum: None,
            ..Default::default()
        })
        .await?;
        let deadline = std::time::Instant::now() + Duration::from_secs_f64(duration_s + 2.0);
        loop {
            tokio::time::sleep(Duration::from_millis(40)).await;
            app.touch_client();
            if !app.snapshot().experiment.active {
                break;
            }
            ensure!(
                std::time::Instant::now() < deadline,
                "experiment service deadline exceeded"
            );
        }
        app.stop().await?;
        let data = app.experiment_data();
        std::fs::write(
            output.join(format!("{name}.json")),
            serde_json::to_vec_pretty(&data)?,
        )?;
        let reason = data.status.reason.as_deref().unwrap_or("");
        println!(
            "EXPERIMENT {}",
            json!({ "name": name, "status": data.status, "last_command": data.commands.last() })
        );
        ensure!(
            reason == "duration completed",
            "{name} ended unexpectedly: {reason}"
        );
        if kind == ExperimentKind::Record {
            ensure!(data.commands.is_empty(), "passive recording sent a target");
        }
    }
    let report = json!({ "identity": app.snapshot().identity, "results": results });
    std::fs::write(
        output.join("speed-verification.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    ensure!(
        results
            .iter()
            .flat_map(|r| r["checks"].as_array().unwrap())
            .all(|c| c["speed_mapping_consistent"] == true),
        "speed feedback disagrees with position; inspect raw traces"
    );
    ensure!(results.iter().flat_map(|r| r["checks"].as_array().unwrap()).filter(|c| c["position_regression_rad_s"].as_f64().unwrap().abs() > 0.5).count() >= 4,
        "insufficient motion to validate speed scaling; inspect torque limits, friction and the raw traces");
    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 5 || args[3] != "--motion" {
        bail!("Usage: el05_bench <interface> <motor-id> --motion <output-directory> (unloaded motor only)");
    }
    let id: u8 = args[2].parse().context("motor ID must be 0..255")?;
    let output = Path::new(&args[4]);
    std::fs::create_dir_all(output)?;
    let gate = can_lease::CanTransportGate::default();
    let permit = gate
        .try_acquire(can_lease::CanOwner::El05)
        .map_err(anyhow::Error::msg)?;
    let bus = backend::open_classic_1m_bus(&args[1], permit).await?;
    let app = El05Session::start(backend::protect_feature_bus(bus), 253).await?;
    let result = async {
        app.attach(id).await?;
        println!(
            "IDENTITY {}",
            serde_json::to_string(&app.read_identity().await?)?
        );
        let _ = app.read_version().await?;
        let mode = app.snapshot().mode;
        let mut saved = Vec::new();
        for index in [0x700B, 0x7026, 0x7028] {
            saved.push(app.read_parameter(index).await?);
        }
        std::fs::write(output.join("original-settings.json"), serde_json::to_vec_pretty(&json!({
            "mode": mode, "parameters": saved, "identity": app.snapshot().identity, "version": app.snapshot().version
        }))?)?;
        let result = bench(&app, output).await;
        let mut cleanup = app.stop().await;
        if cleanup.is_ok() {
            if let Err(error) = app.set_mode(mode).await {
                cleanup = Err(error);
            }
            for parameter in saved {
                if let Err(error) = app.write_parameter(parameter.index, parameter.value).await {
                    eprintln!("Restore 0x{:04X}: {error}", parameter.index);
                    cleanup = Err(error);
                }
            }
        }
        result?;
        cleanup?;
        std::fs::write(output.join("restored-state.json"), serde_json::to_vec_pretty(&app.snapshot())?)?;
        Ok::<_, anyhow::Error>(())
    }
    .await;
    let shutdown = app.shutdown().await;
    if shutdown.is_err() {
        app.force_stop();
    }
    result?;
    shutdown?;
    println!(
        "PASS MIT velocity consistency; motor stopped and original volatile settings restored"
    );
    Ok(())
}
