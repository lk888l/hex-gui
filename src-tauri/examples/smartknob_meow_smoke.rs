//! Read-only probe and bounded smoke test for the actual Meow SmartKnob path.
//!
//! `cargo run --example smartknob_meow_smoke -- can0 0x01 --probe`
//! only uploads identity, calibration and status. `--run` additionally enables
//! the motor for five seconds, switches haptic modes, and confirms Disabled.
//! `--friction` compares four short, displacement-limited torque passes with
//! factory friction disabled/enabled, using the same driver as the haptic loop.
//! No calibration, identity, or communication setting is saved to flash.

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

use anyhow::{bail, ensure, Context, Result};
use can_transport::CanBus;
use hex_motor::canopen::sdo;
use hex_motor::meow_motor::{
    calibration, lookup_model, MeowFactoryCalibration, MeowMotorLogic, MeowMotorManager,
    MeowMotorManagerOptions,
};
use hex_motor::types::MotorIdentity;

const HOST_NODE_ID: u8 = 10;
const RUN_DURATION: Duration = Duration::from_secs(5);
const TORQUE_LIMIT_NM: f64 = 0.06;
const MOTOR_LIMIT_PERMILLE: u16 = 40;
const FRICTION_CAP_NM: f64 = 0.03;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let mut args = std::env::args().skip(1);
    let iface = args.next().unwrap_or_else(|| "can0".into());
    let node_id = args
        .next()
        .map(|v| parse_node_id(&v))
        .transpose()?
        .unwrap_or(1);
    let operation = args.next().unwrap_or_else(|| "--probe".into());
    ensure!(
        args.next().is_none(),
        "usage: smartknob_meow_smoke [can0] [0x01] [--probe|--run|--friction]"
    );
    ensure!(
        matches!(operation.as_str(), "--probe" | "--run" | "--friction"),
        "operation must be --probe, --run, or --friction"
    );
    ensure!(
        node_id != HOST_NODE_ID,
        "motor node ID must differ from the host node ID ({HOST_NODE_ID})"
    );
    let bus = open_bus(&iface)?;
    probe(bus.as_ref(), node_id).await?;
    match operation.as_str() {
        "--run" => smoke(bus, node_id).await?,
        "--friction" => friction_comparison(bus, node_id).await?,
        _ => {}
    }
    Ok(())
}

#[cfg(target_os = "linux")]
fn open_bus(iface: &str) -> Result<Arc<dyn CanBus>> {
    Ok(Arc::new(can_transport::socketcan::SocketCanBus::open(
        iface,
    )?))
}

#[cfg(not(target_os = "linux"))]
fn open_bus(_iface: &str) -> Result<Arc<dyn CanBus>> {
    bail!("this hardware smoke example requires Linux SocketCAN")
}

fn parse_node_id(value: &str) -> Result<u8> {
    let id = match value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
    {
        Some(hex) => u8::from_str_radix(hex, 16)?,
        None => value.parse()?,
    };
    ensure!((1..=127).contains(&id), "motor node ID must be 1..=127");
    Ok(id)
}

async fn probe(bus: &dyn CanBus, nid: u8) -> Result<()> {
    let timeout = Some(Duration::from_millis(500));
    let identity = MotorIdentity {
        node_id: nid,
        vendor_id: sdo::upload_u32(bus, nid, 0x1018, 1, timeout).await?,
        product_code: sdo::upload_u32(bus, nid, 0x1018, 2, timeout).await?,
        revision_number: sdo::upload_u32(bus, nid, 0x1018, 3, timeout).await?,
        serial_number: sdo::upload_u32(bus, nid, 0x1018, 4, timeout).await?,
        product_name: None,
    };
    let model = lookup_model(identity.vendor_id, identity.product_code)
        .context("target is not a supported new-protocol Meow Motor")?;
    println!(
        "PROBE {} node=0x{nid:02X} identity={:08X}/{:08X}/{:08X}/{:08X}",
        model.display_name(),
        identity.vendor_id,
        identity.product_code,
        identity.revision_number,
        identity.serial_number,
    );
    let factory =
        calibration::read_from_device(bus, nid, &identity, Duration::from_millis(500)).await?;
    match factory {
        MeowFactoryCalibration::Valid(v1) => {
            println!(
                "CALIBRATION CRC=valid torque_factor={:.8} fit_rmse_nm={:.8}",
                v1.torque.factor, v1.torque.fit_rmse_nm
            );
            if let Some(f) = v1.friction {
                println!(
                    "FRICTION raw_nm static_pos={:.8} static_neg={:.8} kinetic_pos={:.8} kinetic_neg={:.8} reference_rad_s={:.5} temperature_c={:.2}",
                    f.static_pos_raw_nm, f.static_neg_raw_nm,
                    f.kinetic_pos_raw_nm, f.kinetic_neg_raw_nm,
                    f.kinetic_reference_speed_rad_per_s, f.calibration_temperature_c,
                );
            } else {
                println!("FRICTION unavailable: no factory friction calibration");
            }
        }
        MeowFactoryCalibration::Missing { reason, .. } => {
            println!("CALIBRATION unavailable: {reason}")
        }
    }
    let mode = sdo::upload_u8(bus, nid, 0x4402, 0, timeout).await?;
    let error = sdo::upload_u16(bus, nid, 0x453F, 0, timeout).await?;
    println!("STATUS mode=0x{mode:02X} error=0x{error:04X}");
    Ok(())
}

async fn smoke(bus: Arc<dyn CanBus>, nid: u8) -> Result<()> {
    println!("RUN duration=5s host_limit={TORQUE_LIMIT_NM}Nm motor_limit={MOTOR_LIMIT_PERMILLE}permille friction_cap={FRICTION_CAP_NM}Nm");
    let manager = manager_for(bus.clone(), nid).await?;
    let shutdown = AtomicBool::new(false);
    let mut knob = smartknob::SmartKnob::start_meow(
        manager.clone(),
        bus,
        nid,
        1,
        &shutdown,
        HOST_NODE_ID,
        smartknob::InitialSettings {
            custom_config: None,
            tuning: Some(test_tuning(false)),
        },
    )
    .await?;

    // Every normal error after startup passes through stop_meow before it is
    // returned. The firmware heartbeat watchdog also handles process loss.
    let result = observe(&knob).await;
    let stop_result = knob.stop_meow(&manager).await;
    if let Err(error) = stop_result {
        bail!("smoke result={result:?}; final disable failed: {error:#}");
    }
    confirm_disabled(&manager, nid)?;
    result
}

async fn manager_for(bus: Arc<dyn CanBus>, nid: u8) -> Result<Arc<MeowMotorManager>> {
    let manager = Arc::new(MeowMotorManager::new(
        bus,
        MeowMotorManagerOptions {
            heartbeat_node_id: HOST_NODE_ID,
            broadcast_heartbeat: false,
            auto_identify: false,
            sdo_timeout: Duration::from_millis(500),
            ..Default::default()
        },
    )?);
    let online_deadline = Instant::now() + Duration::from_secs(2);
    while !manager
        .list()
        .iter()
        .any(|info| info.node_id == nid && info.online)
    {
        ensure!(
            Instant::now() < online_deadline,
            "no motor heartbeat received within two seconds"
        );
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    manager.identify(nid).await?;
    Ok(manager)
}

fn confirm_disabled(manager: &MeowMotorManager, nid: u8) -> Result<()> {
    let status = manager.status(nid)?;
    ensure!(
        status.logic == Some(MeowMotorLogic::Disabled),
        "stop completed without Disabled feedback: {:?}",
        status.logic
    );
    println!("STOP confirmed Disabled");
    Ok(())
}

fn apply_test_tuning(knob: &smartknob::SmartKnob, restoring: bool) {
    let tuning = test_tuning(restoring);
    knob.set_tuning(
        tuning.p_gain,
        tuning.d_gain,
        tuning.strength_scale,
        tuning.torque_limit_nm,
        tuning.max_torque_permille,
        tuning.friction_compensation,
        tuning.click_torque_nm,
    );
}

fn test_tuning(restoring: bool) -> smartknob::Tuning {
    smartknob::Tuning {
        p_gain: if restoring { 0.8 } else { 0.0 },
        d_gain: if restoring { 0.02 } else { 0.0 },
        strength_scale: 0.08,
        torque_limit_nm: TORQUE_LIMIT_NM,
        max_torque_permille: MOTOR_LIMIT_PERMILLE,
        friction_compensation: FRICTION_CAP_NM,
        click_torque_nm: 0.0,
    }
}

async fn observe(knob: &smartknob::SmartKnob) -> Result<()> {
    let started = Instant::now();
    let mut switched = false;
    let mut samples = 0;
    let mut max_speed: f64 = 0.0;
    let mut max_torque: f64 = 0.0;
    let mut max_friction: f64 = 0.0;
    let mut next_report = Duration::ZERO;
    while started.elapsed() < RUN_DURATION {
        tokio::time::sleep(Duration::from_millis(10)).await;
        if !switched && started.elapsed() >= Duration::from_secs(2) {
            knob.set_config(5);
            apply_test_tuning(knob, true);
            switched = true;
            println!("MODE return-to-center; reference is the present shaft position");
        }
        let state = knob.state();
        ensure!(
            state.error.is_none(),
            "SmartKnob reported {:?}",
            state.error
        );
        ensure!(
            state.running && state.online && state.enabled,
            "SmartKnob lost running/online/enabled state"
        );
        ensure!(
            state.applied_torque_nm.abs() <= TORQUE_LIMIT_NM + 1e-6,
            "host torque exceeded smoke limit: {}",
            state.applied_torque_nm
        );
        ensure!(
            state.shaft_velocity_rev_per_s.abs() < 0.8,
            "shaft speed exceeded smoke limit: {} rev/s",
            state.shaft_velocity_rev_per_s
        );
        samples += 1;
        max_speed = max_speed.max(state.shaft_velocity_rev_per_s.abs());
        max_torque = max_torque.max(state.applied_torque_nm.abs());
        max_friction = max_friction.max(state.friction_torque_nm.abs());
        if started.elapsed() >= next_report {
            println!(
                "SAMPLE t={:.3} mode={} angle_rad={:.6} velocity_rev_s={:.6} torque_nm={:.6} friction_nm={:.6}",
                started.elapsed().as_secs_f64(), state.config_index, state.shaft_angle_rad,
                state.shaft_velocity_rev_per_s, state.applied_torque_nm, state.friction_torque_nm,
            );
            if samples == 1 {
                println!(
                    "APPLIED_CALIBRATION {}",
                    serde_json::to_string(&state.calibration)?
                );
            }
            next_report += Duration::from_millis(500);
        }
    }
    println!("PASS samples={samples} max_velocity_rev_s={max_speed:.6} max_torque_nm={max_torque:.6} max_friction_nm={max_friction:.6}");
    Ok(())
}

const FRICTION_TEST_BASE_NM: f64 = 0.04;
const FRICTION_TEST_CAP_NM: f64 = 0.13;
const FRICTION_TEST_LIMIT_NM: f64 = 0.18;
const FRICTION_TEST_MOTOR_PERMILLE: u16 = 80;
const FRICTION_TEST_MAX_TRAVEL_RAD: f64 = 0.05;
const FRICTION_TEST_MAX_SPEED_REV_S: f64 = 0.5;

async fn friction_comparison(bus: Arc<dyn CanBus>, nid: u8) -> Result<()> {
    println!("FRICTION_TEST passes=4 base=+/-{FRICTION_TEST_BASE_NM}Nm cap={FRICTION_TEST_CAP_NM}Nm host_limit={FRICTION_TEST_LIMIT_NM}Nm motor_limit={FRICTION_TEST_MOTOR_PERMILLE}permille max_pass=500ms max_travel={FRICTION_TEST_MAX_TRAVEL_RAD}rad");
    let manager = manager_for(bus.clone(), nid).await?;
    let driver = meow_smartknob::MeowKnobMotor::start(
        manager.clone(),
        bus,
        nid,
        FRICTION_TEST_MOTOR_PERMILLE,
        &AtomicBool::new(false),
        HOST_NODE_ID,
    )
    .await?;
    let result = async {
        let calibration = driver.calibration();
        ensure!(
            calibration.static_positive_nm.is_some() && calibration.static_negative_nm.is_some(),
            "friction comparison requires valid directional factory friction calibration"
        );
        println!(
            "APPLIED_CALIBRATION {}",
            serde_json::to_string(&calibration)?
        );
        for (direction, compensated) in [(1.0, false), (1.0, true), (-1.0, false), (-1.0, true)] {
            wait_still(&driver).await?;
            friction_pass(&driver, direction, compensated).await?;
        }
        wait_still(&driver).await?;
        Ok::<(), anyhow::Error>(())
    }
    .await;
    let cleanup = driver.stop().await;
    if let Err(error) = cleanup {
        bail!("friction comparison result={result:?}; final disable failed: {error:#}");
    }
    confirm_disabled(&manager, nid)?;
    result
}

async fn wait_still(driver: &meow_smartknob::MeowKnobMotor) -> Result<()> {
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut still_since = None;
    loop {
        driver
            .send_torque(0.0, FRICTION_TEST_MOTOR_PERMILLE)
            .await?;
        let feedback = driver.feedback()?;
        ensure!(
            feedback.velocity_rev_per_s.abs() < FRICTION_TEST_MAX_SPEED_REV_S,
            "friction comparison exceeded speed limit while stopping"
        );
        if feedback.velocity_rev_per_s.abs() < 0.005 {
            let since = still_since.get_or_insert_with(Instant::now);
            if since.elapsed() >= Duration::from_millis(200) {
                return Ok(());
            }
        } else {
            still_since = None;
        }
        ensure!(
            Instant::now() < deadline,
            "motor did not become still within two seconds"
        );
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

async fn friction_pass(
    driver: &meow_smartknob::MeowKnobMotor,
    direction: f64,
    compensated: bool,
) -> Result<()> {
    let initial_position = driver.feedback()?.raw_rev;
    let started = Instant::now();
    let base = direction * FRICTION_TEST_BASE_NM;
    let cap = if compensated {
        FRICTION_TEST_CAP_NM
    } else {
        0.0
    };
    let mut max_torque: f64 = 0.0;
    let mut max_friction: f64 = 0.0;
    let mut max_speed: f64 = 0.0;
    let mut samples = 0;
    let mut displacement;
    let static_request = driver.friction_torque(0.0, base, cap);
    loop {
        let feedback = driver.feedback()?;
        displacement = (feedback.raw_rev - initial_position) * std::f64::consts::TAU;
        ensure!(
            feedback.velocity_rev_per_s.abs() < FRICTION_TEST_MAX_SPEED_REV_S,
            "friction comparison exceeded speed limit"
        );
        ensure!(
            displacement * direction > -0.01,
            "motor moved against the commanded direction: {displacement:.6} rad"
        );
        if displacement.abs() >= FRICTION_TEST_MAX_TRAVEL_RAD
            || started.elapsed() >= Duration::from_millis(500)
        {
            break;
        }
        let compensation = driver.friction_torque(
            feedback.velocity_rev_per_s * std::f64::consts::TAU,
            base,
            cap,
        );
        let requested =
            (base + compensation).clamp(-FRICTION_TEST_LIMIT_NM, FRICTION_TEST_LIMIT_NM);
        let applied = driver
            .send_torque(requested, FRICTION_TEST_MOTOR_PERMILLE)
            .await?;
        max_torque = max_torque.max(applied.abs());
        max_friction = max_friction.max(compensation.abs());
        max_speed = max_speed.max(feedback.velocity_rev_per_s.abs());
        samples += 1;
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    driver
        .send_torque(0.0, FRICTION_TEST_MOTOR_PERMILLE)
        .await?;
    println!(
        "FRICTION_PASS direction={direction:+.0} compensated={compensated} elapsed_ms={} samples={samples} displacement_rad={displacement:.8} moved={} static_compensation_nm={static_request:.8} max_friction_nm={max_friction:.8} max_applied_nm={max_torque:.8} max_speed_rev_s={max_speed:.8}",
        started.elapsed().as_millis(), displacement.abs() >= 0.005,
    );
    Ok(())
}
