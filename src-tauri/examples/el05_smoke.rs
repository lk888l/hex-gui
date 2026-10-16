//! Attended hardware check using the same EL05 session as the GUI.
//! Run explicitly: cargo run --example el05_smoke -- can0 1 --motion
//! Sends small targets in all five modes, then stops and restores volatile settings.

#[path = "../src/backend.rs"]
mod backend;
#[path = "../src/can_lease.rs"]
mod can_lease;
#[path = "../src/el05.rs"]
mod el05;

use anyhow::{bail, ensure, Context, Result};
use el05::{El05Mode, El05Session, El05Target};
use std::sync::Arc;
use std::time::Duration;

async fn observe(app: &El05Session, milliseconds: u64) -> Result<()> {
    for _ in 0..milliseconds.div_ceil(50) {
        tokio::time::sleep(Duration::from_millis(50)).await;
        app.touch_client();
        let state = app.snapshot();
        ensure!(state.online, "EL05 feedback became stale");
        ensure!(
            state.fault_bits == 0 && state.fault_code == 0,
            "EL05 fault: {:?}",
            state.faults
        );
    }
    Ok(())
}

async fn exercise(app: &Arc<El05Session>) -> Result<()> {
    for (mode, code) in [
        (El05Mode::Mit, 0),
        (El05Mode::Current, 3),
        (El05Mode::Velocity, 2),
        (El05Mode::PositionPp, 1),
        (El05Mode::PositionCsp, 5),
    ] {
        app.set_mode(mode).await?;
        let actual = app.read_parameter(0x7005).await?;
        ensure!(actual.value == code as f64, "Mode readback mismatch");
        let position = app.read_parameter(0x7019).await?.value;
        app.enable().await?;
        let target = match mode {
            El05Mode::Mit => El05Target::Mit {
                position_rad: position,
                velocity_rad_s: 0.15,
                torque_nm: 0.0,
                kp: 0.0,
                kd: 0.1,
            },
            El05Mode::Current => El05Target::Current { current_a: 0.05 },
            El05Mode::Velocity => El05Target::Velocity {
                velocity_rad_s: 0.15,
                acceleration_rad_s2: 1.0,
                current_limit_a: 0.3,
            },
            El05Mode::PositionPp => El05Target::PositionPp {
                position_rad: position + 0.05,
                velocity_rad_s: 0.15,
                acceleration_rad_s2: 1.0,
                deceleration_rad_s2: None,
                current_limit_a: 0.3,
            },
            El05Mode::PositionCsp => El05Target::PositionCsp {
                position_rad: position - 0.05,
                velocity_rad_s: 0.15,
                current_limit_a: 0.3,
            },
        };
        let before = app.snapshot().rx_count;
        app.send_target(target, true).await?;
        observe(app, 500).await?;
        let running = app.snapshot();
        ensure!(
            running.enabled && running.rx_count > before,
            "No running feedback for {mode:?}"
        );
        println!("RUN {}", serde_json::to_string(&running)?);
        app.stop().await?;
        ensure!(!app.snapshot().enabled, "Stop was not confirmed");
        println!("PASS {mode:?}: mode readback, running feedback, confirmed stop");
    }
    // Exercise the UI lease with real firmware: no GUI snapshots for >3 seconds.
    app.set_mode(El05Mode::Mit).await?;
    app.enable().await?;
    app.touch_client();
    tokio::time::sleep(Duration::from_millis(3500)).await;
    let state = app.snapshot();
    ensure!(
        !state.enabled && !state.streaming,
        "UI heartbeat expiry did not stop the motor"
    );
    println!(
        "PASS UI heartbeat expiry: {}",
        serde_json::to_string(&state)?
    );
    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 4 || args[3] != "--motion" {
        bail!("Usage: el05_smoke <can-interface> <motor-id> --motion (moves the motor)");
    }
    let id: u8 = args[2].parse().context("motor-id must be 0..255")?;
    let gate = can_lease::CanTransportGate::default();
    let permit = gate
        .try_acquire(can_lease::CanOwner::El05)
        .map_err(anyhow::Error::msg)?;
    let bus = backend::open_classic_1m_bus(&args[1], permit).await?;
    let app = El05Session::start(backend::protect_feature_bus(bus), 253).await?;
    let result = async {
        app.discover().await?;
        app.attach(id).await?;
        println!("ATTACHED {}", serde_json::to_string(&app.snapshot())?);
        let original_mode = app.snapshot().mode;
        let mut saved = Vec::new();
        for index in [0x700B, 0x7017, 0x7018, 0x7022, 0x7024, 0x7025, 0x7028] {
            saved.push(app.read_parameter(index).await?);
        }
        let exercise_result = exercise(&app).await;
        // Attempt every cleanup action even if one fails.
        let stopped = app.stop().await;
        let mut cleanup_error = stopped.err();
        if cleanup_error.is_none() {
            if let Err(e) = app.set_mode(original_mode).await {
                cleanup_error = Some(e);
            }
            for value in saved {
                if let Err(e) = app.write_parameter(value.index, value.value).await {
                    eprintln!("Restore 0x{:04X} failed: {e}", value.index);
                    cleanup_error = Some(e);
                }
            }
        }
        exercise_result?;
        if let Some(e) = cleanup_error {
            return Err(e);
        }
        Ok::<_, anyhow::Error>(())
    }
    .await;
    let shutdown = app.shutdown().await;
    if shutdown.is_err() {
        app.force_stop();
    }
    result?;
    shutdown?;
    println!("PASS shutdown; motor stopped, original mode and volatile limits restored");
    Ok(())
}
