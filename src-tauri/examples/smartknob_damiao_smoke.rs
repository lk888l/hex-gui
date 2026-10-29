//! Exercise the actual H3510 SmartKnob adapter on SocketCAN.
//!
//! --discover scans the bus; --probe reads mapping and confirms Disabled. --run runs the shared haptic
//! engine for five seconds. --torque checks short positive/negative torque
//! responses, bounded by time and measured displacement. No flash writes.

#![allow(dead_code)]

#[path = "../src/calibration_transport.rs"]
mod calibration_transport;
#[path = "../src/damiao.rs"]
mod damiao;
#[path = "../src/damiao_smartknob.rs"]
mod damiao_smartknob;
#[path = "../src/meow_smartknob.rs"]
mod meow_smartknob;
#[path = "../src/smartknob.rs"]
mod smartknob;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{ensure, Result};
use can_transport::CanBus;
use damiao::{DamiaoConfig, DamiaoMode, DamiaoSession};

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    let mut args = std::env::args().skip(1);
    let iface = args.next().unwrap_or_else(|| "can0".into());
    let node = args.next().unwrap_or_else(|| "1".into());
    let node = match node.strip_prefix("0x").or_else(|| node.strip_prefix("0X")) {
        Some(hex) => u8::from_str_radix(hex, 16)?,
        None => node.parse()?,
    };
    ensure!(node <= 15, "H3510 feedback is ambiguous for IDs above 0x0F; independent state confirmation is required");
    let operation = args.next().unwrap_or_else(|| "--probe".into());
    ensure!(
        args.next().is_none(),
        "usage: smartknob_damiao_smoke [can0] [0x01] [--discover|--probe|--run|--torque]"
    );
    ensure!(
        matches!(
            operation.as_str(),
            "--discover" | "--probe" | "--run" | "--torque"
        ),
        "unknown operation"
    );
    let bus = open_bus(&iface)?;
    if operation == "--discover" {
        let discovery = damiao::DamiaoDiscovery::start(bus).await?;
        let result = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if let Some(device) = discovery
                    .snapshot()
                    .into_iter()
                    .find(|device| device.motor_id == u16::from(node) && device.online)
                {
                    break device;
                }
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
        })
        .await;
        discovery.stop_and_wait().await;
        println!("DISCOVERY {}", serde_json::to_string(&result?)?);
        return Ok(());
    }
    let session = DamiaoSession::start(
        bus.clone(),
        DamiaoConfig {
            motor_id: u16::from(node),
            master_id: 0,
            mode: DamiaoMode::Mit,
            // Replaced with live register reads before any targets are sent.
            p_max: 12.5,
            v_max: 30.0,
            t_max: 0.45,
        },
    )
    .await?;
    let result = match operation.as_str() {
        "--run" => run_knob(session.clone(), bus, node).await,
        "--torque" => torque_check(session.clone(), bus).await,
        _ => {
            let result = session.disable().await;
            println!("PROBE {}", serde_json::to_string(&session.snapshot())?);
            result
        }
    };
    // Always make an independent, fresh disable transaction before exit.
    let stopped = session.disable().await;
    session.force_stop();
    if let Err(error) = stopped {
        anyhow::bail!("operation result={result:?}; final disable failed: {error:#}");
    }
    println!("STOP confirmed Disabled");
    result
}

#[cfg(target_os = "linux")]
fn open_bus(iface: &str) -> Result<Arc<dyn CanBus>> {
    Ok(Arc::new(can_transport::socketcan::SocketCanBus::open(
        iface,
    )?))
}

#[cfg(not(target_os = "linux"))]
fn open_bus(_iface: &str) -> Result<Arc<dyn CanBus>> {
    anyhow::bail!("this hardware smoke example requires Linux SocketCAN")
}

async fn run_knob(session: Arc<DamiaoSession>, bus: Arc<dyn CanBus>, node: u8) -> Result<()> {
    let mut knob = smartknob::SmartKnob::start_damiao(
        session,
        bus,
        node,
        1,
        &AtomicBool::new(false),
        smartknob::InitialSettings {
            custom_config: None,
            tuning: Some(smartknob::Tuning {
                torque_limit_nm: 0.05,
                max_torque_permille: 150,
                friction_compensation: 0.0,
                ..Default::default()
            }),
        },
    )
    .await?;
    let result: Result<()> = async {
        for index in [1, 5, 8, 7, 1] {
            knob.set_config(index);
            tokio::time::sleep(Duration::from_secs(1)).await;
            let state = knob.state();
            println!("HAPTIC {}", serde_json::to_string(&state)?);
            ensure!(
                state.running && state.enabled && state.error.is_none(),
                "haptic loop stopped: {:?}",
                state.error
            );
            ensure!(
                state.torque_limit_nm <= 0.05 && state.max_torque_permille == 150,
                "mode change raised session limits"
            );
            ensure!(
                state.applied_torque_nm.abs() <= 0.05 + 0.005,
                "unexpected output torque"
            );
        }
        Ok(())
    }
    .await;
    knob.stop_damiao().await?;
    result
}

async fn torque_check(session: Arc<DamiaoSession>, bus: Arc<dyn CanBus>) -> Result<()> {
    for request in [0.02, -0.02] {
        let motor = damiao_smartknob::DamiaoKnobMotor::start(
            session.clone(),
            bus.clone(),
            &AtomicBool::new(false),
        )
        .await?;
        let result: Result<()> = async {
            let start = motor.feedback()?.raw_rev * std::f64::consts::TAU;
            let started = Instant::now();
            let mut ticker = tokio::time::interval(Duration::from_millis(1));
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            let mut delta = 0.0f64;
            let mut applied = 0.0f64;
            while started.elapsed() < Duration::from_millis(200) {
                ticker.tick().await;
                let feedback = motor.feedback()?;
                delta = feedback.raw_rev * std::f64::consts::TAU - start;
                if delta.abs() >= 0.05 {
                    break;
                }
                applied = motor.send_torque(request, 100).await?;
            }
            println!("TORQUE requested={request:.5}Nm applied={applied:.5}Nm duration_ms={} delta_rad={delta:.6}", started.elapsed().as_millis());
            // A stationary shaft is reported, not mistaken for a sign pass.
            if delta.abs() > 0.005 {
                ensure!(delta.signum() == request.signum(), "torque and encoder signs disagree");
            }
            Ok(())
        }
        .await;
        motor.stop().await?;
        result?;
        tokio::time::sleep(Duration::from_millis(300)).await;
    }
    Ok(())
}
