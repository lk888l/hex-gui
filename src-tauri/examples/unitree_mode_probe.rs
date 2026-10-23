//! Brief zero-gain/zero-feedforward mode-transition diagnostic. No Flash writes.
#[path = "../src/unitree_boot.rs"]
mod unitree_boot;
#[path = "../src/unitree_protocol.rs"]
mod unitree_protocol;
#[path = "../src/unitree_servo.rs"]
mod unitree_servo;

use anyhow::Result;
use unitree_protocol::ServoTarget;
use unitree_servo::{ServoOperation, ServoSession, ServoSettings};

#[tokio::main]
async fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let port = args.next().unwrap_or_else(|| "/dev/ttyACM0".into());
    let id = args.next().unwrap_or_else(|| "0".into()).parse()?;
    let app = ServoSession::open(port, id, ServoSettings::default())?;
    let selected = app.execute(ServoOperation::Select(id)).await?;
    println!("Before: {}", serde_json::to_string(&selected)?);
    let zero = ServoTarget {
        position_rad: selected.feedback.as_ref().unwrap().position_rad,
        ..ServoTarget::default()
    };
    let result = app
        .execute(ServoOperation::Send {
            target: zero,
            repeat: false,
        })
        .await;
    println!("Zero-output FOC: {result:?}");
    println!("After: {}", serde_json::to_string(&app.snapshot())?);
    for attempt in 1..=3 {
        let stopped = app.execute(ServoOperation::Stop).await;
        println!("Stop {attempt}: {stopped:?}");
        if stopped.is_ok() {
            break;
        }
    }
    app.shutdown(true).await?;
    Ok(())
}
