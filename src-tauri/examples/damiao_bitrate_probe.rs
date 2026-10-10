//! SocketCAN probe using the same bitrate transactions as the standalone app.
//! Examples (the host interface is configured separately):
//! cargo run --example damiao_bitrate_probe -- can0 3 0x23 get
//! cargo run --example damiao_bitrate_probe -- can0 3 0x23 set 500000
//! cargo run --example damiao_bitrate_probe -- can0 3 0x23 save

#[allow(dead_code)]
#[path = "../src/damiao.rs"]
mod damiao;

use std::sync::Arc;

use anyhow::{anyhow, bail, Context, Result};
use can_transport::CanBus;
use damiao::{DamiaoConfig, DamiaoMode, DamiaoSession};

fn parse_id(text: &str) -> Result<u16> {
    if let Some(hex) = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16).context("invalid hexadecimal CAN ID")
    } else {
        text.parse().context("invalid decimal CAN ID")
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if !(args.len() == 4 || args.len() == 5) {
        bail!("usage: damiao_bitrate_probe <interface> <motor-id> <master-id> <get|current|config|set bitrate|save>");
    }
    #[cfg(target_os = "linux")]
    let bus: Arc<dyn CanBus> = Arc::new(can_transport::socketcan::SocketCanBus::open(&args[0])?);
    #[cfg(not(target_os = "linux"))]
    let bus: Arc<dyn CanBus> = {
        bail!("this probe uses Linux SocketCAN; use the standalone app for USB CAN adapters");
    };

    eprintln!("Host CAN configuration: {:?}", bus.link_config().await?);
    let operation = args[3].as_str();
    if operation == "config" {
        return Ok(());
    }
    if !matches!(operation, "get" | "current" | "set" | "save") {
        bail!("unknown probe operation {operation}");
    }
    let requested_bitrate = if operation == "set" {
        Some(
            args.get(4)
                .ok_or_else(|| anyhow!("set requires a bitrate in bit/s"))?
                .parse::<u32>()?,
        )
    } else {
        None
    };
    let session = DamiaoSession::start(
        bus,
        DamiaoConfig {
            motor_id: parse_id(&args[1])?,
            master_id: parse_id(&args[2])?,
            mode: DamiaoMode::Mit,
            p_max: 12.5,
            v_max: 30.0,
            t_max: 10.0,
        },
    )
    .await?;

    let outcome: Result<()> = async {
        match operation {
            "get" | "current" => {
                println!("{}", serde_json::to_string(&session.get_bitrate().await?)?)
            }
            "set" => println!(
                "{}",
                serde_json::to_string(&session.set_bitrate(requested_bitrate.unwrap()).await?)?
            ),
            "save" => {
                session.save_parameters().await?;
                println!("{{\"saved\":true}}");
            }
            _ => unreachable!(),
        }
        println!("{}", serde_json::to_string(&session.snapshot())?);
        Ok(())
    }
    .await;
    let teardown = session.shutdown().await;
    if teardown.is_err() {
        session.force_stop();
    }
    outcome?;
    teardown
}
