//! Attended, bounded hardware smoke test of the same dual-CAN runtime as the GUI.
//! cargo run --example dual_motor_bench -- 20 [mirror]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let seconds = std::env::args()
        .nth(1)
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or(20);
    let mirror = std::env::args().nth(2).as_deref() == Some("mirror");
    hex_motor_gui_lib::run_dual_motor_bench(seconds, mirror).await
}
