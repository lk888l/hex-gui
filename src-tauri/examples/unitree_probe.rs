//! Lock-mode-only USB serial probe. Never enables FOC or writes persistent data.
//! cargo run --manifest-path src-tauri/Cargo.toml --example unitree_probe -- /dev/ttyACM0
#[path = "../src/unitree_boot.rs"]
mod unitree_boot;
#[path = "../src/unitree_protocol.rs"]
mod unitree_protocol;
#[path = "../src/unitree_servo.rs"]
mod unitree_servo;

use anyhow::Result;
use unitree_servo::{ServoOperation, ServoSession, ServoSettings};

#[tokio::main]
async fn main() -> Result<()> {
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/ttyACM0".into());
    let app = ServoSession::open(port, 0, ServoSettings::default())?;
    println!("Serial opened: {}", serde_json::to_string(&app.snapshot())?);
    let result = app.execute(ServoOperation::Scan).await;
    match &result {
        Ok(state) => println!("Scan: {}", serde_json::to_string(state)?),
        Err(error) => eprintln!(
            "Scan: {error:#}\nState: {}",
            serde_json::to_string(&app.snapshot())?
        ),
    }
    app.shutdown(true).await?;
    result.map(|_| ())
}
