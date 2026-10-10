//! Read the same mapping and position state as the standalone motor app.
//! The session sends a safe disable, then only position/status read requests.
//! cargo run --example damiao_position_probe -- can0 3 0x23

#[allow(dead_code)]
#[path = "../src/damiao.rs"]
mod damiao;

use std::sync::Arc;

use anyhow::{bail, Context, Result};
use can_transport::CanBus;
use damiao::{DamiaoConfig, DamiaoMode, DamiaoSession};

fn parse_id(value: &str) -> Result<u16> {
    if let Some(hex) = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
    {
        u16::from_str_radix(hex, 16).context("invalid hexadecimal CAN ID")
    } else {
        value.parse().context("invalid decimal CAN ID")
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 3 {
        bail!("usage: damiao_position_probe <interface> <motor-id> <master-id>");
    }
    #[cfg(target_os = "linux")]
    let bus: Arc<dyn CanBus> = Arc::new(can_transport::socketcan::SocketCanBus::open(&args[0])?);
    #[cfg(not(target_os = "linux"))]
    let bus: Arc<dyn CanBus> = {
        bail!("this probe requires Linux SocketCAN");
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
    let result: Result<()> = async {
        for _ in 0..10 {
            session.read_positions().await?;
            let state = session.snapshot();
            println!("{}", serde_json::to_string(&serde_json::json!({
                "state": state,
                "output_position_deg": state.output_position_rad.map(|value| value.to_degrees()),
            }))?);
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
        }
        Ok(())
    }.await;
    let teardown = session.shutdown().await;
    if teardown.is_err() {
        session.force_stop();
    }
    result?;
    teardown
}
