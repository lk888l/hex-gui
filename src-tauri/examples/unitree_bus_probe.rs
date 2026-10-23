//! Lock-mode-only multi-servo communication test; no FOC or Flash writes.
//! cargo run --example unitree_bus_probe -- /dev/ttyACM0 0,1,2 5
//! Omit IDs to use all IDs discovered by a lock-mode scan.
#[path = "../src/unitree_boot.rs"]
mod unitree_boot;
#[path = "../src/unitree_protocol.rs"]
mod unitree_protocol;
#[path = "../src/unitree_servo.rs"]
mod unitree_servo;

use anyhow::{ensure, Result};
use unitree_protocol::ServoTarget;
use unitree_servo::bus::{BusTestKind, BusTestNode};
use unitree_servo::{BusTestRequest, ServoOperation, ServoSession, ServoSettings};

#[tokio::main]
async fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let port = args.next().unwrap_or_else(|| "/dev/ttyACM0".into());
    let ids: Option<Vec<u8>> = args
        .next()
        .map(|text| text.split(',').map(str::parse).collect())
        .transpose()?;
    let duration_s = args.next().unwrap_or_else(|| "5".into()).parse()?;
    let app = ServoSession::open(
        port,
        ids.as_ref()
            .and_then(|ids| ids.first().copied())
            .unwrap_or(0),
        ServoSettings::default(),
    )?;
    let result = async {
        let ids = if let Some(ids) = ids {
            ids
        } else {
            app.execute(ServoOperation::Scan)
                .await?
                .devices
                .iter()
                .map(|device| device.id)
                .collect()
        };
        println!("Discovered / requested IDs: {ids:?}");
        ensure!(
            ids.len() >= 2,
            "Need at least two powered servos with distinct IDs for a multi-servo hardware test"
        );
        app.execute(ServoOperation::StartBusTest(BusTestRequest {
            kind: BusTestKind::Communication,
            duration_s,
            nodes: ids
                .into_iter()
                .map(|id| BusTestNode {
                    id,
                    target: ServoTarget::default(),
                })
                .collect(),
        }))
        .await?;
        loop {
            app.touch_client();
            let report = app.snapshot().bus_test.unwrap();
            if !report.running {
                println!("{}", serde_json::to_string_pretty(&report)?);
                ensure!(
                    report.phase == "completed"
                        && report.nodes.iter().all(|node| node.tx_count > 0
                            && node.tx_count == node.rx_count
                            && node.stop_confirmed),
                    "Multi-servo communication test failed; inspect report"
                );
                break;
            }
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
        Ok(())
    }
    .await;
    app.shutdown(true).await?;
    result
}
