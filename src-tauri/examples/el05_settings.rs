//! Attended settings check using the production EL05 session. No motion targets.
//! el05_settings can0 1 --read
//! el05_settings can0 1 --roundtrip-id 2 (temporarily changes ID, then restores it)

#[path = "../src/backend.rs"]
mod backend;
#[path = "../src/can_lease.rs"]
mod can_lease;
#[path = "../src/el05.rs"]
mod el05;

use anyhow::{bail, ensure, Context, Result};
use el05::El05Session;

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    let roundtrip = args.get(3).is_some_and(|arg| arg == "--roundtrip-id");
    if !(args.len() == 4 && args[3] == "--read" || args.len() == 5 && roundtrip) {
        bail!("Usage: el05_settings <interface> <motor-id> --read | --roundtrip-id <temporary-id>");
    }
    let id: u8 = args[2].parse().context("motor ID must be 0..255")?;
    let temporary: Option<u8> = if roundtrip {
        let temporary = args[4].parse().context("temporary ID must be 0..127")?;
        ensure!(
            id <= 127 && temporary <= 127 && id != temporary,
            "both IDs must be distinct and within 0..127 so restoration is possible"
        );
        Some(temporary)
    } else {
        None
    };
    let gate = can_lease::CanTransportGate::default();
    let permit = gate
        .try_acquire(can_lease::CanOwner::El05)
        .map_err(anyhow::Error::msg)?;
    let bus = backend::open_classic_1m_bus(&args[1], permit).await?;
    let app = El05Session::start(backend::protect_feature_bus(bus), 253).await?;
    let result = async {
        app.attach(id).await?;
        let identity = app.read_identity().await?;
        let version = app.read_version().await?;
        println!("IDENTITY {}", serde_json::to_string(&identity)?);
        println!("VERSION {}", serde_json::to_string(&version)?);
        if let Some(temporary) = temporary {
            let changed = app.change_id(temporary).await;
            // Attempt restoration even when the write succeeds but a later
            // mode/feedback check fails. Never address an unverified motor.
            let restore = if app.snapshot().motor_id == Some(temporary) {
                app.change_id(id).await.map(|restored| {
                    println!("RESTORED {}", serde_json::to_string(&restored).unwrap());
                    restored
                })
            } else {
                app.read_identity().await
            };
            if let Err(error) = &restore {
                eprintln!(
                    "ID restoration failed: {error}; scan for MCU {}",
                    identity.uid
                );
            }
            let restored = restore?;
            ensure!(
                restored == identity,
                "restored identity differs from the original"
            );
            let changed = changed?;
            ensure!(
                changed.motor_id == temporary && changed.uid == identity.uid,
                "changed identity mismatch"
            );
            println!("PASS ID {id} -> {temporary} -> {id}, same MCU UID");
        }
        let final_identity = app.read_identity().await?;
        ensure!(final_identity == identity, "final identity mismatch");
        ensure!(
            !app.snapshot().enabled && !app.snapshot().streaming,
            "motor must remain stopped"
        );
        Ok::<_, anyhow::Error>(())
    }
    .await;
    let shutdown = app.shutdown().await;
    if shutdown.is_err() {
        app.force_stop();
    }
    result?;
    shutdown?;
    println!("PASS metadata checks; motor stopped");
    Ok(())
}
