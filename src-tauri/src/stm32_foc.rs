//! Dedicated host for stm32g431-foc-canfd. Configuration is SDO; motion and
//! periodic feedback are PDO. This module never uses the HEX motor OD/scales.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use can_transport::{CanBus, CanFilter, CanFrame, CanRx, FrameKind};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;
use tokio::task::JoinHandle;

const PERIOD: Duration = Duration::from_millis(10);
const SDO_TIMEOUT: Duration = Duration::from_millis(250);
const STATE_TIMEOUT: Duration = Duration::from_millis(500);
// TPDO1 is explicitly configured at 20 ms. Losing ten successive reports
// terminates renewal of the last motion command, allowing the firmware's own
// 100 ms command deadline to remain effective even after a receive failure.
const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(200);

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
pub enum FocMode {
    SingleTurn,
    Position,
    #[default]
    Velocity,
    Torque,
    Mit,
}

impl FocMode {
    fn raw(self) -> i8 {
        match self {
            Self::SingleTurn => -1,
            Self::Position => 1,
            Self::Velocity => 3,
            Self::Torque => 4,
            Self::Mit => 5,
        }
    }

    fn position(self) -> bool {
        matches!(self, Self::SingleTurn | Self::Position)
    }

    fn rpdo(self) -> (u16, &'static [u32]) {
        match self {
            Self::Velocity => (0, &[0x60FF0020, 0x60400010, 0x60600008]),
            Self::SingleTurn | Self::Position => (1, &[0x607A0020, 0x60400010, 0x60600008]),
            Self::Torque => (2, &[0x60710010, 0x60400010, 0x60600008]),
            Self::Mit => (
                3,
                &[
                    0x20030120, 0x20030220, 0x20030320, 0x20030410, 0x20030510, 0x60400010,
                    0x60600008,
                ],
            ),
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FocConfig {
    pub mode: FocMode,
    pub current_limit_permille: u16,
    pub profile_velocity: f32,
    pub acceleration: f32,
    pub deceleration: f32,
    pub quick_stop_deceleration: f32,
    pub following_error_window: f32,
    pub following_error_timeout_ms: u16,
}

impl Default for FocConfig {
    fn default() -> Self {
        Self {
            mode: FocMode::Velocity,
            current_limit_permille: 1000,
            profile_velocity: 2.0,
            acceleration: 2.0,
            deceleration: 2.0,
            quick_stop_deceleration: 4.0,
            following_error_window: 0.25,
            following_error_timeout_ms: 1000,
        }
    }
}

impl FocConfig {
    fn validate(&self) -> Result<()> {
        if self.current_limit_permille > 1000 {
            bail!("Current limit must be 0..1000 permille");
        }
        positive(self.profile_velocity, 40.0, "Profile velocity")?;
        positive(self.acceleration, 100.0, "Acceleration")?;
        positive(self.deceleration, 100.0, "Deceleration")?;
        positive(
            self.quick_stop_deceleration,
            100.0,
            "Quick-stop deceleration",
        )?;
        positive(
            self.following_error_window,
            100000.0,
            "Following-error window",
        )
    }
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct FocTarget {
    pub position_rev: f32,
    pub velocity_rev_s: f32,
    pub torque_permille: i16,
    pub torque_ff_nm: f32,
    pub kp: f32,
    pub kd: f32,
    pub relative: bool,
}

impl FocTarget {
    fn validate(&self) -> Result<()> {
        bounded(self.position_rev, -100000.0, 100000.0, "Position (rev)")?;
        bounded(self.velocity_rev_s, -40.0, 40.0, "Velocity (rev/s)")?;
        if !(-1000..=1000).contains(&self.torque_permille) {
            bail!("Torque must be -1000..1000 permille");
        }
        bounded(self.torque_ff_nm, -1.0, 1.0, "MIT feed-forward torque (Nm)")?;
        bounded(self.kp, 0.0, 65.535, "MIT Kp (Nm/rev)")?;
        bounded(self.kd, 0.0, 20.0, "MIT Kd (Nm/(rev/s))")
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct FocPdoInfo {
    pub direction: String,
    pub number: u8,
    pub cob_id: u32,
    pub enabled: bool,
    pub mapping: Vec<u32>,
}

#[derive(Clone, Debug, Serialize)]
pub struct FocState {
    pub node_id: u8,
    pub initialized: bool,
    pub streaming: bool,
    pub enabled: bool,
    pub mode: FocMode,
    pub actual_mode: Option<i8>,
    pub statusword: Option<u16>,
    pub error_code: Option<u16>,
    pub calibration_state: Option<u8>,
    pub calibration_pole_pairs: Option<u16>,
    pub calibration_direction: Option<i8>,
    pub calibration_offset_rad: Option<f32>,
    pub position_rev: Option<f32>,
    pub velocity_rev_s: Option<f32>,
    pub torque_permille: Option<i16>,
    pub voltage_v: Option<f32>,
    pub temperature_c: Option<f32>,
    pub nmt_state: Option<u8>,
    pub feedback_age_ms: Option<u64>,
    pub tx_count: u64,
    pub rx_count: u64,
    pub phase: String,
    pub last_error: Option<String>,
    pub identity: Option<[u32; 4]>,
    pub pdo_layout: Vec<FocPdoInfo>,
}

struct Inner {
    view: FocState,
    target: FocTarget,
    controlword: u16,
    status_seen: Option<Instant>,
    status_sequence: u64,
    stream_started: Option<Instant>,
}

pub struct FocSession {
    bus: Arc<dyn CanBus>,
    node: u8,
    inner: StdMutex<Inner>,
    operation: Mutex<()>,
    stop_operation: Mutex<()>,
    stopping: AtomicBool,
    sdo: Mutex<()>,
    transmit: Mutex<()>,
    task: Mutex<Option<JoinHandle<()>>>,
    generation: AtomicU64,
    closing: AtomicBool,
}

impl FocSession {
    pub async fn start(bus: Arc<dyn CanBus>, node: u8) -> Result<Arc<Self>> {
        if !(1..=127).contains(&node) {
            bail!("Node-ID must be within 1..127");
        }
        let rx = bus.subscribe(CanFilter::pass_all_standard()).await?;
        let session = Arc::new(Self {
            bus,
            node,
            inner: StdMutex::new(Inner {
                view: FocState {
                    node_id: node,
                    initialized: false,
                    streaming: false,
                    enabled: false,
                    mode: FocMode::Velocity,
                    actual_mode: None,
                    statusword: None,
                    error_code: None,
                    calibration_state: None,
                    calibration_pole_pairs: None,
                    calibration_direction: None,
                    calibration_offset_rad: None,
                    position_rev: None,
                    velocity_rev_s: None,
                    torque_permille: None,
                    voltage_v: None,
                    temperature_c: None,
                    nmt_state: None,
                    feedback_age_ms: None,
                    tx_count: 0,
                    rx_count: 0,
                    phase: "connected".into(),
                    last_error: None,
                    identity: None,
                    pdo_layout: Vec::new(),
                },
                target: FocTarget::default(),
                controlword: 0,
                status_seen: None,
                status_sequence: 0,
                stream_started: None,
            }),
            operation: Mutex::new(()),
            stop_operation: Mutex::new(()),
            stopping: AtomicBool::new(false),
            sdo: Mutex::new(()),
            transmit: Mutex::new(()),
            task: Mutex::new(None),
            generation: AtomicU64::new(0),
            closing: AtomicBool::new(false),
        });
        let weak = Arc::downgrade(&session);
        *session.task.lock().await = Some(tokio::spawn(async move {
            run(weak, rx).await;
        }));
        Ok(session)
    }

    pub fn snapshot(&self) -> FocState {
        let inner = self.inner.lock().unwrap();
        let mut view = inner.view.clone();
        view.feedback_age_ms = inner.status_seen.map(|t| t.elapsed().as_millis() as u64);
        view
    }

    fn phase(&self, phase: &str) {
        self.inner.lock().unwrap().view.phase = phase.into();
    }

    fn begin(&self) -> Result<u64> {
        if self.closing.load(Ordering::Acquire) {
            bail!("Session is closing");
        }
        if self.stopping.load(Ordering::Acquire) {
            bail!("Drive is stopping; wait for disable to complete");
        }
        self.inner.lock().unwrap().view.last_error = None;
        Ok(self.generation.load(Ordering::Acquire))
    }

    fn check(&self, generation: u64) -> Result<()> {
        if self.closing.load(Ordering::Acquire)
            || self.generation.load(Ordering::Acquire) != generation
        {
            bail!("Operation cancelled by disable/disconnect");
        }
        Ok(())
    }

    fn failed(&self, error: &anyhow::Error) {
        let mut inner = self.inner.lock().unwrap();
        inner.view.last_error = Some(format!("{error:#}"));
        inner.view.phase = "error".into();
    }

    pub async fn initialize(&self, config: FocConfig) -> Result<FocState> {
        config.validate()?;
        let _operation = self.operation.lock().await;
        let generation = self.begin()?;
        let result = self.initialize_inner(config, generation).await;
        if let Err(error) = &result {
            let _ = self.nmt(0x80).await;
            let _ = self.write(0x6040, 0, &0u16.to_le_bytes()).await;
            self.failed(error);
            self.inner.lock().unwrap().view.initialized = false;
        }
        result
    }

    async fn initialize_inner(&self, config: FocConfig, generation: u64) -> Result<FocState> {
        self.phase("initializing: disable bridge");
        self.stop_stream().await;
        self.nmt(0x80).await?;
        self.write_checked(0x6040, 0, &0u16.to_le_bytes(), generation)
            .await?;
        tokio::time::sleep(Duration::from_millis(100)).await;
        self.check(generation)?;
        let device_type = self.u32(0x1000, 0).await?;
        if device_type != 0x00020192 {
            bail!("Unexpected device type 0x{device_type:08X}; expected 0x00020192");
        }
        let mut identity = [0; 4];
        for sub in 1..=4 {
            self.check(generation)?;
            identity[sub - 1] = self.u32(0x1018, sub as u8).await?;
        }
        self.inner.lock().unwrap().view.identity = Some(identity);
        self.refresh_initial().await?;
        self.wait_state_sdo(0x40, generation).await?;
        if self.snapshot().calibration_state != Some(4) {
            self.phase("calibration required");
            bail!("Electrical calibration required: run Calibrate and save, then initialize again");
        }

        self.phase("initializing: parameters and initial target");
        self.check(generation)?;
        self.write_checked(0x2004, 1, &[0], generation).await?;
        self.write_checked(
            0x6072,
            0,
            &config.current_limit_permille.to_le_bytes(),
            generation,
        )
        .await?;
        self.write_checked(0x2003, 6, &1000u16.to_le_bytes(), generation)
            .await?;
        for (index, value) in [
            (0x6081, config.profile_velocity),
            (0x6083, config.acceleration),
            (0x6084, config.deceleration),
            (0x6085, config.quick_stop_deceleration),
            (0x6065, config.following_error_window),
        ] {
            self.check(generation)?;
            self.write_checked(index, 0, &value.to_le_bytes(), generation)
                .await?;
        }
        self.write_checked(
            0x6066,
            0,
            &config.following_error_timeout_ms.to_le_bytes(),
            generation,
        )
        .await?;
        self.write_checked(0x6060, 0, &[config.mode.raw() as u8], generation)
            .await?;
        let position = self.f32(0x6064, 0).await?;
        bounded(position, -100000.0, 100000.0, "Initial actual position")?;
        let target = FocTarget {
            position_rev: position,
            ..FocTarget::default()
        };
        self.write_checked(0x607A, 0, &position.to_le_bytes(), generation)
            .await?;
        self.write_checked(0x60FF, 0, &0f32.to_le_bytes(), generation)
            .await?;
        self.write_checked(0x6071, 0, &0i16.to_le_bytes(), generation)
            .await?;
        // Initialize all volatile MIT fields even when another mode was selected.
        for (sub, value) in [(1, position), (2, 0.0), (3, 0.0)] {
            self.write_checked(0x2003, sub, &value.to_le_bytes(), generation)
                .await?;
        }
        self.write_checked(0x2003, 4, &0u16.to_le_bytes(), generation)
            .await?;
        self.write_checked(0x2003, 5, &0u16.to_le_bytes(), generation)
            .await?;

        self.phase("initializing: read and replace PDO configuration");
        // Saved mappings and enabled flags are real configuration. Read each
        // layout and disable every PDO before installing the host's layout.
        let mut existing = Vec::new();
        for (direction, base, mapbase) in [("RPDO", 0x1400, 0x1600), ("TPDO", 0x1800, 0x1A00)] {
            for number in 0..4 {
                self.check(generation)?;
                let cob = self.u32(base + number, 1).await?;
                let count = self.u8(mapbase + number, 0).await?;
                if count > 16 {
                    bail!("Invalid {direction}{} map count {count}", number + 1);
                }
                let mut mapping = Vec::new();
                for sub in 1..=count {
                    mapping.push(self.u32(mapbase + number, sub).await?);
                }
                existing.push(FocPdoInfo {
                    direction: direction.into(),
                    number: number as u8 + 1,
                    cob_id: cob & 0x7FF,
                    enabled: false,
                    mapping,
                });
                self.write_checked(
                    base + number,
                    1,
                    &(cob | 0x80000000).to_le_bytes(),
                    generation,
                )
                .await?;
            }
        }
        // Keep the observed old layouts visible until the complete new mapping
        // has been acknowledged, including after a mid-configuration abort.
        self.inner.lock().unwrap().view.pdo_layout = existing;
        let (rpdo, mapping) = config.mode.rpdo();
        let rpdo_id = 0x200 + rpdo * 0x100 + self.node as u16;
        self.map(false, rpdo, rpdo_id, mapping, generation).await?;
        self.map(
            true,
            0,
            0x180 + self.node as u16,
            &[0x603F0010, 0x60410010, 0x60610008],
            generation,
        )
        .await?;
        self.map(
            true,
            1,
            0x280 + self.node as u16,
            &[0x60640020, 0x606C0020],
            generation,
        )
        .await?;
        self.map(
            true,
            2,
            0x380 + self.node as u16,
            &[0x60770010, 0x22020020, 0x21000910],
            generation,
        )
        .await?;
        let status_before_start = {
            let mut inner = self.inner.lock().unwrap();
            inner.target = target;
            inner.controlword = 0;
            inner.view.mode = config.mode;
            inner.view.enabled = false;
            inner.view.initialized = false;
            inner.status_seen = None;
            inner.status_sequence
        };
        self.check(generation)?;
        self.nmt_checked(1, generation).await?;
        self.phase("initializing: waiting for TPDO feedback");
        self.wait_feedback(0x006F, 0x40, status_before_start, generation)
            .await?;
        self.check(generation)?;
        {
            let mut inner = self.inner.lock().unwrap();
            self.check(generation)?;
            inner.view.initialized = true;
            inner.view.phase = "ready".into();
        }
        Ok(self.snapshot())
    }

    async fn map(
        &self,
        tpdo: bool,
        number: u16,
        cob: u16,
        entries: &[u32],
        generation: u64,
    ) -> Result<()> {
        let comm = (if tpdo { 0x1800 } else { 0x1400 }) + number;
        let map = (if tpdo { 0x1A00 } else { 0x1600 }) + number;
        let old = self.u32(comm, 1).await?;
        // Preserve the applicable RTR-policy bit; the installed layout uses
        // standard 11-bit IDs and no saved extension-ID flag.
        let flags = old & 0x40000000;
        self.write_checked(
            comm,
            1,
            &(flags | 0x80000000 | cob as u32).to_le_bytes(),
            generation,
        )
        .await?;
        self.write_checked(map, 0, &[0], generation).await?;
        for (offset, entry) in entries.iter().enumerate() {
            self.check(generation)?;
            self.write_checked(map, offset as u8 + 1, &entry.to_le_bytes(), generation)
                .await?;
        }
        self.write_checked(map, 0, &[entries.len() as u8], generation)
            .await?;
        self.write_checked(comm, 2, &[255], generation).await?;
        if tpdo {
            self.write_checked(comm, 3, &0u16.to_le_bytes(), generation)
                .await?;
            self.write_checked(comm, 5, &20u16.to_le_bytes(), generation)
                .await?;
        } else {
            self.write_checked(comm, 5, &0u16.to_le_bytes(), generation)
                .await?;
        }
        self.check(generation)?;
        self.write_checked(comm, 1, &(flags | cob as u32).to_le_bytes(), generation)
            .await?;
        let mut inner = self.inner.lock().unwrap();
        if let Some(info) = inner.view.pdo_layout.iter_mut().find(|info| {
            info.direction == if tpdo { "TPDO" } else { "RPDO" } && info.number == number as u8 + 1
        }) {
            info.cob_id = cob as u32;
            info.enabled = true;
            info.mapping = entries.to_vec();
        }
        Ok(())
    }

    pub async fn calibrate(&self) -> Result<FocState> {
        let _operation = self.operation.lock().await;
        let generation = self.begin()?;
        let result = self.calibrate_inner(generation).await;
        if let Err(error) = &result {
            let _ = self.nmt(2).await;
            let _ = self.nmt(0x80).await;
            let _ = self.write(0x6040, 0, &0u16.to_le_bytes()).await;
            self.failed(error);
        }
        result
    }

    async fn calibrate_inner(&self, generation: u64) -> Result<FocState> {
        self.stop_stream().await;
        self.inner.lock().unwrap().view.initialized = false;
        self.phase("calibrating: prepare disabled bridge");
        self.nmt(0x80).await?;
        self.write_checked(0x6040, 0, &0u16.to_le_bytes(), generation)
            .await?;
        tokio::time::sleep(Duration::from_millis(100)).await;
        self.wait_state_sdo(0x40, generation).await?;
        self.refresh_initial().await?;
        self.check(generation)?;
        self.write_checked(0x2100, 1, &0x696C6163u32.to_le_bytes(), generation)
            .await?;
        let deadline = Instant::now() + Duration::from_secs(15);
        let mut seen_active = false;
        loop {
            self.check(generation)?;
            let state = self.u8(0x2100, 2).await?;
            self.inner.lock().unwrap().view.calibration_state = Some(state);
            self.phase(match state {
                1 => "calibrating: align",
                2 => "calibrating: sweep",
                3 => "calibrating: settle",
                4 => "calibrating: complete",
                5 => "calibrating: failed",
                _ => "calibrating: waiting",
            });
            if (1..=3).contains(&state) {
                seen_active = true;
            }
            // A previous calibration may still read 4 until the control task
            // consumes the new request. Observe this run before accepting 4.
            if state == 4 && seen_active {
                break;
            }
            if state == 5 {
                self.refresh_initial().await?;
                bail!(
                    "Electrical calibration failed; drive fault 0x{:04X}",
                    self.snapshot().error_code.unwrap_or(0)
                );
            }
            if Instant::now() >= deadline {
                self.nmt(2).await?;
                self.nmt(0x80).await?;
                bail!("Electrical calibration timed out after 15 seconds");
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        self.check(generation)?;
        let poles = self.u16(0x2100, 3).await?;
        let direction = self.u8(0x2100, 4).await? as i8;
        let offset = self.f32(0x2100, 5).await?;
        self.refresh_initial().await?;
        self.wait_state_sdo(0x40, generation).await?;
        self.check(generation)?;
        self.write_checked(0x1010, 1, &0x65766173u32.to_le_bytes(), generation)
            .await?;
        {
            let mut inner = self.inner.lock().unwrap();
            inner.view.calibration_pole_pairs = Some(poles);
            inner.view.calibration_direction = Some(direction);
            inner.view.calibration_offset_rad = Some(offset);
            inner.view.phase = "calibration saved; initialize next".into();
        }
        Ok(self.snapshot())
    }

    pub async fn enable(&self) -> Result<()> {
        let _operation = self.operation.lock().await;
        let generation = self.begin()?;
        if !self.snapshot().initialized {
            bail!("Initialize the PDO layout and targets before enabling");
        }
        let result = self.enable_inner(generation).await;
        if let Err(error) = &result {
            if self.check(generation).is_err() {
                return result;
            }
            self.stop_stream().await;
            let _ = self.nmt(0x80).await;
            let _ = self.write(0x6040, 0, &0u16.to_le_bytes()).await;
            self.failed(error);
        }
        result
    }

    async fn enable_inner(&self, generation: u64) -> Result<()> {
        // Enabling establishes a fresh neutral command rather than resuming
        // the target left behind by an earlier disable or Quick stop.
        let position = self.f32(0x6064, 0).await?;
        bounded(position, -100000.0, 100000.0, "Initial actual position")?;
        {
            let mut inner = self.inner.lock().unwrap();
            self.check(generation)?;
            inner.target = FocTarget {
                position_rev: position,
                ..FocTarget::default()
            };
            inner.status_seen = None;
        }
        self.phase("enabling: start PDO stream");
        self.nmt_checked(1, generation).await?;
        {
            let mut inner = self.inner.lock().unwrap();
            self.check(generation)?;
            inner.controlword = 0;
            inner.view.streaming = true;
            inner.stream_started = Some(Instant::now());
        }
        for (cw, expected) in [(6, 0x21), (7, 0x23), (15, 0x27)] {
            self.check(generation)?;
            let sequence = {
                let mut inner = self.inner.lock().unwrap();
                inner.controlword = cw;
                inner.status_sequence
            };
            self.wait_feedback(0x006F, expected, sequence, generation)
                .await?;
        }
        self.phase("running");
        Ok(())
    }

    pub async fn set_target(&self, target: FocTarget) -> Result<()> {
        target.validate()?;
        let _operation = self.operation.lock().await;
        let generation = self.begin()?;
        let view = self.snapshot();
        if !view.initialized {
            bail!("Initialize before setting a target");
        }
        if view.mode.position() && !view.enabled {
            bail!("Enable the drive before triggering a position target");
        }
        let result = self.apply_target(target, generation).await;
        if let Err(error) = &result {
            // A new Stop/Quick stop owns the command after cancellation; its
            // trajectory must not be overwritten by this older transaction.
            if self.check(generation).is_ok() {
                self.stop_stream().await;
                let _ = self.nmt(0x80).await;
                let _ = self.write(0x6040, 0, &0u16.to_le_bytes()).await;
                self.failed(error);
            }
        }
        result
    }

    async fn apply_target(&self, target: FocTarget, generation: u64) -> Result<()> {
        if self.snapshot().mode.position() {
            // Do not emit a low/high pair in one firmware processing cycle.
            let sequence = {
                let mut inner = self.inner.lock().unwrap();
                inner.controlword &= !0x0010;
                inner.status_sequence
            };
            self.wait_feedback(0x1000, 0, sequence, generation).await?;
            self.check(generation)?;
            let sequence = {
                let mut inner = self.inner.lock().unwrap();
                inner.target = target;
                inner.controlword = (inner.controlword & !0x0040)
                    | if inner.target.relative { 0x0040 } else { 0 }
                    | 0x0010;
                inner.status_sequence
            };
            self.wait_feedback(0x1000, 0x1000, sequence, generation)
                .await?;
        } else {
            self.check(generation)?;
            self.inner.lock().unwrap().target = target;
        }
        Ok(())
    }

    pub async fn halt(&self, halt: bool) -> Result<()> {
        let _operation = self.operation.lock().await;
        self.begin()?;
        let mut inner = self.inner.lock().unwrap();
        if !inner.view.streaming {
            bail!("The drive has no running PDO stream");
        }
        if halt {
            inner.controlword |= 0x100;
        } else {
            inner.controlword &= !0x100;
        }
        inner.view.phase = if halt { "halted" } else { "running" }.into();
        Ok(())
    }

    pub async fn quick_stop(&self) -> Result<()> {
        self.begin()?;
        self.generation.fetch_add(1, Ordering::AcqRel);
        let mut inner = self.inner.lock().unwrap();
        if !inner.view.streaming {
            bail!("The drive has no running PDO stream");
        }
        // Keep renewing during deceleration. Reception of disabled terminates
        // this stream; the next enable always repeats the complete handshake.
        inner.controlword = 0x000B;
        inner.view.phase = "quick stopping".into();
        Ok(())
    }

    pub async fn disable(&self) -> Result<()> {
        let _stop = self.stop_operation.lock().await;
        self.stopping.store(true, Ordering::Release);
        self.generation.fetch_add(1, Ordering::AcqRel);
        self.stop_stream().await;
        // NMT Stop also cancels an in-progress electrical calibration before
        // waiting for its operation guard. Queued starts cannot overtake this
        // final bridge-off sequence.
        let _ = self.nmt(2).await;
        let _operation = self.operation.lock().await;
        let result = self.disable_inner().await;
        self.stopping.store(false, Ordering::Release);
        if let Err(error) = &result {
            self.failed(error);
        }
        result
    }

    async fn disable_inner(&self) -> Result<()> {
        let stop = self.nmt(2).await;
        let preop = self.nmt(0x80).await;
        let zero = self.write(0x6040, 0, &0u16.to_le_bytes()).await;
        stop?;
        preop?;
        zero?;
        let deadline = Instant::now() + STATE_TIMEOUT;
        loop {
            let status = self.u16(0x6041, 0).await?;
            self.record_status(status);
            if matches!(status & 0x006F, 0x40 | 0x08) {
                break;
            }
            if Instant::now() >= deadline {
                bail!("Disable timeout: status 0x{status:04X}");
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        self.phase("disabled");
        Ok(())
    }

    pub async fn reset_fault(&self) -> Result<FocState> {
        // Cancellation precedes the operation lock, so a calibration is
        // interrupted promptly instead of delaying the user's stop request.
        self.disable().await?;
        let _operation = self.operation.lock().await;
        let generation = self.begin()?;
        self.write_checked(0x6040, 0, &0u16.to_le_bytes(), generation)
            .await?;
        self.write_checked(0x6040, 0, &0x0080u16.to_le_bytes(), generation)
            .await?;
        let deadline = Instant::now() + STATE_TIMEOUT;
        loop {
            self.check(generation)?;
            let fault = self.u16(0x603F, 0).await?;
            let status = self.u16(0x6041, 0).await?;
            self.record_status(status);
            self.inner.lock().unwrap().view.error_code = Some(fault);
            if fault == 0 && status & 8 == 0 {
                break;
            }
            if Instant::now() >= deadline {
                bail!("Fault reset did not clear drive fault 0x{fault:04X}, status 0x{status:04X}");
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        self.write_checked(0x6040, 0, &0u16.to_le_bytes(), generation)
            .await?;
        self.wait_state_sdo(0x40, generation).await?;
        self.refresh_initial().await?;
        self.inner.lock().unwrap().view.initialized = false;
        self.phase("fault reset; initialize next");
        Ok(self.snapshot())
    }

    pub async fn save(&self) -> Result<()> {
        self.disable().await?;
        let _operation = self.operation.lock().await;
        let generation = self.begin()?;
        self.wait_state_sdo(0x40, generation).await?;
        self.check(generation)?;
        self.write_checked(0x1010, 1, &0x65766173u32.to_le_bytes(), generation)
            .await?;
        self.phase("parameters saved");
        Ok(())
    }

    pub async fn shutdown(&self) -> Result<()> {
        self.closing.store(true, Ordering::Release);
        self.generation.fetch_add(1, Ordering::AcqRel);
        self.stop_stream().await;
        // Cancel local calibration immediately, then drain the cancelled
        // operation before releasing the session/physical adapter lease.
        let _ = self.nmt(2).await;
        let _stop = self.stop_operation.lock().await;
        let _operation = self.operation.lock().await;
        if let Some(task) = self.task.lock().await.take() {
            task.abort();
            let _ = task.await;
        }
        // This is the final CAN operation: no cancelled initializer can send
        // Start or calibration cleanup after disconnect has returned.
        self.disable_inner().await
    }

    async fn stop_stream(&self) {
        {
            let mut inner = self.inner.lock().unwrap();
            inner.view.streaming = false;
            inner.controlword = 0;
            inner.stream_started = None;
        }
        // A stream frame already being submitted must finish before NMT/CW0.
        let _transmit = self.transmit.lock().await;
    }

    async fn nmt(&self, command: u8) -> Result<()> {
        let _transmit = self.transmit.lock().await;
        self.bus
            .send(CanFrame::new_data(0u16, &[command, self.node])?)
            .await?;
        self.inner.lock().unwrap().view.tx_count += 1;
        Ok(())
    }

    async fn nmt_checked(&self, command: u8, generation: u64) -> Result<()> {
        let _transmit = self.transmit.lock().await;
        self.check(generation)?;
        self.bus
            .send(CanFrame::new_data(0u16, &[command, self.node])?)
            .await?;
        self.inner.lock().unwrap().view.tx_count += 1;
        Ok(())
    }

    async fn write_checked(&self, index: u16, sub: u8, data: &[u8], generation: u64) -> Result<()> {
        let _sdo = self.sdo.lock().await;
        self.check(generation)?;
        canopen_sdo::asynch::download_bytes(
            &*self.bus,
            self.node,
            index,
            sub,
            data,
            Some(SDO_TIMEOUT),
        )
        .await
        .with_context(|| format!("SDO write 0x{index:04X}:{sub:02X}"))
    }

    async fn read<const N: usize>(&self, index: u16, sub: u8) -> Result<[u8; N]> {
        let _sdo = self.sdo.lock().await;
        let data =
            canopen_sdo::asynch::upload_bytes(&*self.bus, self.node, index, sub, Some(SDO_TIMEOUT))
                .await
                .with_context(|| format!("SDO read 0x{index:04X}:{sub:02X}"))?;
        data.try_into().map_err(|data: Vec<u8>| {
            anyhow!(
                "SDO 0x{index:04X}:{sub:02X}: expected {N} bytes, received {}",
                data.len()
            )
        })
    }

    async fn write(&self, index: u16, sub: u8, data: &[u8]) -> Result<()> {
        let _sdo = self.sdo.lock().await;
        canopen_sdo::asynch::download_bytes(
            &*self.bus,
            self.node,
            index,
            sub,
            data,
            Some(SDO_TIMEOUT),
        )
        .await
        .with_context(|| format!("SDO write 0x{index:04X}:{sub:02X}"))
    }

    async fn u8(&self, index: u16, sub: u8) -> Result<u8> {
        Ok(self.read::<1>(index, sub).await?[0])
    }
    async fn u16(&self, index: u16, sub: u8) -> Result<u16> {
        Ok(u16::from_le_bytes(self.read(index, sub).await?))
    }
    async fn u32(&self, index: u16, sub: u8) -> Result<u32> {
        Ok(u32::from_le_bytes(self.read(index, sub).await?))
    }
    async fn f32(&self, index: u16, sub: u8) -> Result<f32> {
        let value = f32::from_le_bytes(self.read(index, sub).await?);
        if !value.is_finite() {
            bail!("SDO 0x{index:04X}:{sub:02X}: non-finite Float32");
        }
        Ok(value)
    }

    fn record_status(&self, status: u16) {
        let mut inner = self.inner.lock().unwrap();
        inner.view.statusword = Some(status);
        inner.view.enabled = status & 0x006F == 0x27;
    }

    async fn refresh_initial(&self) -> Result<()> {
        let error = self.u16(0x603F, 0).await?;
        let status = self.u16(0x6041, 0).await?;
        let calibration = self.u8(0x2100, 2).await?;
        let voltage = i32::from_le_bytes(self.read(0x2202, 0).await?) as f32 / 1000.0;
        let temperature = i16::from_le_bytes(self.read(0x2100, 9).await?) as f32 / 10.0;
        self.record_status(status);
        {
            let mut inner = self.inner.lock().unwrap();
            inner.view.error_code = Some(error);
            inner.view.calibration_state = Some(calibration);
            inner.view.voltage_v = Some(voltage);
            inner.view.temperature_c = Some(temperature);
        }
        if error != 0 || status & 8 != 0 {
            bail!("Drive fault 0x{error:04X}, status 0x{status:04X}; remove the cause and reset explicitly");
        }
        Ok(())
    }

    async fn wait_state_sdo(&self, expected: u16, generation: u64) -> Result<()> {
        let deadline = Instant::now() + STATE_TIMEOUT;
        loop {
            self.check(generation)?;
            let status = self.u16(0x6041, 0).await?;
            self.record_status(status);
            if status & 8 != 0 {
                bail!("Drive is faulted, status 0x{status:04X}");
            }
            if status & 0x006F == expected {
                return Ok(());
            }
            if Instant::now() >= deadline {
                bail!("State timeout: expected 0x{expected:04X}, got 0x{status:04X}");
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    async fn wait_feedback(
        &self,
        mask: u16,
        expected: u16,
        after: u64,
        generation: u64,
    ) -> Result<()> {
        let deadline = Instant::now() + STATE_TIMEOUT;
        loop {
            self.check(generation)?;
            {
                let inner = self.inner.lock().unwrap();
                if let Some(status) = inner.view.statusword {
                    if status & 8 != 0 || inner.view.error_code.is_some_and(|e| e != 0) {
                        bail!(
                            "Drive fault 0x{:04X}, status 0x{status:04X}",
                            inner.view.error_code.unwrap_or(0)
                        );
                    }
                    if inner.status_seen.is_some()
                        && inner.status_sequence > after
                        && status & mask == expected
                    {
                        return Ok(());
                    }
                }
                if let Some(error) = &inner.view.last_error {
                    bail!("{error}");
                }
            }
            if Instant::now() >= deadline {
                bail!("TPDO timeout waiting for status & 0x{mask:04X} = 0x{expected:04X}");
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }
}

fn positive(value: f32, max: f32, name: &str) -> Result<()> {
    if !value.is_finite() || value <= 0.0 || value > max {
        bail!("{name} must be finite and within (0, {max}]");
    }
    Ok(())
}

fn bounded(value: f32, min: f32, max: f32, name: &str) -> Result<()> {
    if !value.is_finite() || !(min..=max).contains(&value) {
        bail!("{name} must be finite and within {min}..{max}");
    }
    Ok(())
}

fn command_frame(
    node: u8,
    mode: FocMode,
    target: &FocTarget,
    controlword: u16,
) -> Result<CanFrame> {
    let (number, _) = mode.rpdo();
    let id = 0x200 + number * 0x100 + node as u16;
    let mut data = Vec::with_capacity(20);
    match mode {
        FocMode::SingleTurn | FocMode::Position => data.extend(target.position_rev.to_le_bytes()),
        FocMode::Velocity => data.extend(target.velocity_rev_s.to_le_bytes()),
        FocMode::Torque => data.extend(target.torque_permille.to_le_bytes()),
        FocMode::Mit => {
            data.extend(target.position_rev.to_le_bytes());
            data.extend(target.velocity_rev_s.to_le_bytes());
            data.extend(target.torque_ff_nm.to_le_bytes());
            data.extend(((target.kp * 1000.0).round() as u16).to_le_bytes());
            data.extend(((target.kd * 1000.0).round() as u16).to_le_bytes());
        }
    }
    data.extend(controlword.to_le_bytes());
    data.push(mode.raw() as u8);
    if data.len() > 8 {
        data.resize(20, 0); // 19 mapped bytes, exactly the required FD wire DLC.
        Ok(CanFrame::new_fd(id, &data, true)?)
    } else {
        Ok(CanFrame::new_data(id, &data)?)
    }
}

async fn run(weak: std::sync::Weak<FocSession>, mut rx: Box<dyn CanRx>) {
    let mut interval = tokio::time::interval(PERIOD);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            received = rx.recv() => {
                let Some(session) = weak.upgrade() else { break; };
                match received {
                    Ok(frame) => { if session.ingest(&frame) { session.stop_on_error().await; } }
                    Err(error) => {
                        session.failed(&anyhow!("CAN receive failed: {error}"));
                        session.stop_on_error().await;
                        break;
                    }
                }
            }
            _ = interval.tick() => {
                let Some(session) = weak.upgrade() else { break; };
                if session.closing.load(Ordering::Acquire) { break; }
                let _transmit = session.transmit.lock().await;
                let next = {
                    let inner = session.inner.lock().unwrap();
                    if inner.view.streaming {
                        let reference = inner.status_seen.or(inner.stream_started);
                        if reference.is_some_and(|time| time.elapsed() > FEEDBACK_TIMEOUT) {
                            Err(anyhow!("TPDO1 feedback missing for more than 200 ms; PDO stream stopped"))
                        } else {
                            command_frame(session.node, inner.view.mode, &inner.target, inner.controlword).map(Some)
                        }
                    } else { Ok(None) }
                };
                match next {
                    Ok(Some(frame)) => match session.bus.send(frame).await {
                        Ok(()) => session.inner.lock().unwrap().view.tx_count += 1,
                        Err(error) => {
                            session.failed(&anyhow!("PDO send failed: {error}"));
                            drop(_transmit);
                            session.stop_on_error().await;
                        }
                    },
                    Ok(None) => {},
                    Err(error) => {
                        session.failed(&error);
                        drop(_transmit);
                        session.stop_on_error().await;
                    }
                }
            }
        }
    }
}

impl FocSession {
    fn ingest(&self, frame: &CanFrame) -> bool {
        if frame.id().is_extended() || frame.kind() != FrameKind::Data {
            return false;
        }
        let id = frame.id().raw() as u16;
        let data = frame.data();
        let node = self.node as u16;
        let mut inner = self.inner.lock().unwrap();
        if id == 0x700 + node && data.len() == 1 {
            inner.view.nmt_state = Some(data[0]);
            inner.view.rx_count += 1;
            if data[0] == 0 {
                inner.view.initialized = false;
                if inner.view.streaming {
                    inner.view.last_error = Some("Node rebooted; initialize again".into());
                    return true;
                }
            }
        } else if id == 0x180 + node && data.len() == 5 {
            let error = u16::from_le_bytes(data[0..2].try_into().unwrap());
            let status = u16::from_le_bytes(data[2..4].try_into().unwrap());
            inner.view.error_code = Some(error);
            inner.view.statusword = Some(status);
            inner.view.actual_mode = Some(data[4] as i8);
            inner.view.enabled = status & 0x006F == 0x27;
            inner.status_seen = Some(Instant::now());
            inner.status_sequence += 1;
            inner.view.rx_count += 1;
            if error != 0 || status & 8 != 0 {
                inner.view.last_error =
                    Some(format!("Drive fault 0x{error:04X}, status 0x{status:04X}"));
                inner.view.phase = "fault".into();
                return inner.view.streaming;
            }
            if inner.controlword == 0x000B && status & 0x006F == 0x40 {
                inner.view.streaming = false;
                inner.controlword = 0;
                inner.view.phase = "quick stop complete".into();
            }
        } else if id == 0x280 + node && data.len() == 8 {
            let position = f32::from_le_bytes(data[..4].try_into().unwrap());
            let velocity = f32::from_le_bytes(data[4..].try_into().unwrap());
            if position.is_finite() && velocity.is_finite() {
                inner.view.position_rev = Some(position);
                inner.view.velocity_rev_s = Some(velocity);
                inner.view.rx_count += 1;
            }
        } else if id == 0x380 + node && data.len() == 8 {
            inner.view.torque_permille = Some(i16::from_le_bytes(data[..2].try_into().unwrap()));
            inner.view.voltage_v =
                Some(i32::from_le_bytes(data[2..6].try_into().unwrap()) as f32 / 1000.0);
            inner.view.temperature_c =
                Some(i16::from_le_bytes(data[6..].try_into().unwrap()) as f32 / 10.0);
            inner.view.rx_count += 1;
        }
        false
    }

    async fn stop_on_error(&self) {
        {
            let mut inner = self.inner.lock().unwrap();
            inner.view.streaming = false;
            inner.view.initialized = false;
            inner.controlword = 0;
        }
        self.generation.fetch_add(1, Ordering::AcqRel);
        let _ = self.nmt(0x80).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use can_transport::{CanCapabilities, CanIoError};
    use std::collections::{HashMap, VecDeque};
    use tokio::sync::mpsc;

    struct MockRx {
        filter: CanFilter,
        rx: mpsc::UnboundedReceiver<CanFrame>,
    }

    #[async_trait::async_trait]
    impl CanRx for MockRx {
        async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
            loop {
                let frame = self.rx.recv().await.ok_or(CanIoError::Disconnected)?;
                if self.filter.matches(&frame) {
                    return Ok(frame);
                }
            }
        }
        fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
            loop {
                match self.rx.try_recv() {
                    Ok(frame) if self.filter.matches(&frame) => return Ok(Some(frame)),
                    Ok(_) => {}
                    Err(mpsc::error::TryRecvError::Empty) => return Ok(None),
                    Err(_) => return Err(CanIoError::Disconnected),
                }
            }
        }
    }

    struct Model {
        od: HashMap<(u16, u8), Vec<u8>>,
        sent: Vec<CanFrame>,
        subscribers: Vec<mpsc::UnboundedSender<CanFrame>>,
        status: u16,
        calibration_reads: VecDeque<u8>,
        save_count: usize,
    }

    struct MockBus {
        model: StdMutex<Model>,
        feedback: AtomicBool,
        acknowledge_position: AtomicBool,
    }

    impl MockBus {
        fn new() -> Arc<Self> {
            let mut od = HashMap::new();
            od.insert((0x1000, 0), 0x00020192u32.to_le_bytes().to_vec());
            for sub in 1..=4 {
                od.insert((0x1018, sub), (sub as u32).to_le_bytes().to_vec());
            }
            od.insert((0x603F, 0), 0u16.to_le_bytes().to_vec());
            od.insert((0x2100, 2), vec![4]);
            od.insert((0x2100, 3), 7u16.to_le_bytes().to_vec());
            od.insert((0x2100, 4), vec![255]);
            od.insert((0x2100, 5), 0.3f32.to_le_bytes().to_vec());
            od.insert((0x2100, 9), 260i16.to_le_bytes().to_vec());
            od.insert((0x2202, 0), 12000i32.to_le_bytes().to_vec());
            od.insert((0x6064, 0), 0.25f32.to_le_bytes().to_vec());
            for number in 0..4u16 {
                od.insert(
                    (0x1400 + number, 1),
                    (0x250u32 + number as u32).to_le_bytes().to_vec(),
                );
                od.insert(
                    (0x1800 + number, 1),
                    (0x350u32 + number as u32).to_le_bytes().to_vec(),
                );
                od.insert((0x1600 + number, 0), vec![1]);
                od.insert((0x1A00 + number, 0), vec![1]);
                od.insert((0x1600 + number, 1), 0x60400010u32.to_le_bytes().to_vec());
                od.insert((0x1A00 + number, 1), 0x60410010u32.to_le_bytes().to_vec());
            }
            Arc::new(Self {
                model: StdMutex::new(Model {
                    od,
                    sent: Vec::new(),
                    subscribers: Vec::new(),
                    status: 0x0240,
                    calibration_reads: VecDeque::new(),
                    save_count: 0,
                }),
                feedback: AtomicBool::new(true),
                acknowledge_position: AtomicBool::new(true),
            })
        }

        fn reply(model: &mut Model, id: u16, data: &[u8]) {
            let frame = CanFrame::new_data(id, data).unwrap();
            model.subscribers.retain(|tx| tx.send(frame).is_ok());
        }

        fn tpdo(model: &mut Model, mode: u8) {
            let mut data = vec![0, 0];
            data.extend(model.status.to_le_bytes());
            data.push(mode);
            Self::reply(model, 0x181, &data);
            let mut motion = Vec::from(0.25f32.to_le_bytes());
            motion.extend(0f32.to_le_bytes());
            Self::reply(model, 0x281, &motion);
        }

        fn update_control(model: &mut Model, cw: u16) {
            model.status = match cw & 0x000F {
                6 => 0x0221,
                7 => 0x0233,
                15 => 0x0237,
                11 => 0x0240,
                _ => 0x0240,
            } | if cw & 0x10 != 0 { 0x1000 } else { 0 };
        }
    }

    #[async_trait::async_trait]
    impl CanBus for MockBus {
        async fn send(&self, frame: CanFrame) -> std::result::Result<(), CanIoError> {
            let mut model = self.model.lock().unwrap();
            model.sent.push(frame);
            let data = frame.data();
            match frame.id().raw() {
                0 if data.len() == 2 => {
                    if data[0] != 1 {
                        model.status = 0x0240;
                    }
                    if data[0] == 1 && self.feedback.load(Ordering::Acquire) {
                        Self::tpdo(&mut model, 0);
                    }
                }
                0x601 => {
                    let index = u16::from_le_bytes([data[1], data[2]]);
                    let sub = data[3];
                    let mut response = [0; 8];
                    response[1..4].copy_from_slice(&data[1..4]);
                    if data[0] == 0x40 {
                        let value = if index == 0x6041 {
                            model.status.to_le_bytes().to_vec()
                        } else if (index, sub) == (0x2100, 2) && !model.calibration_reads.is_empty()
                        {
                            let value = model.calibration_reads.pop_front().unwrap();
                            model.od.insert((index, sub), vec![value]);
                            vec![value]
                        } else {
                            model.od.get(&(index, sub)).cloned().unwrap_or(vec![0; 4])
                        };
                        response[0] = 0x43 | ((4 - value.len()) as u8) << 2;
                        response[4..4 + value.len()].copy_from_slice(&value);
                    } else {
                        let length = 4 - ((data[0] >> 2) & 3) as usize;
                        let value = data[4..4 + length].to_vec();
                        model.od.insert((index, sub), value.clone());
                        if index == 0x6040 {
                            Self::update_control(
                                &mut model,
                                u16::from_le_bytes([value[0], value[1]]),
                            );
                        }
                        if (index, sub) == (0x2100, 1) {
                            // Emulate the previous completed result remaining in
                            // the OD until the control task consumes the request.
                            model.calibration_reads = VecDeque::from([4, 1, 2, 4]);
                        }
                        if index == 0x1010 {
                            model.save_count += 1;
                        }
                        response[0] = 0x60;
                    }
                    Self::reply(&mut model, 0x581, &response);
                }
                id @ (0x201 | 0x301 | 0x401 | 0x501) => {
                    let offset = match id {
                        0x401 => 2,
                        0x501 => 16,
                        _ => 4,
                    };
                    Self::update_control(
                        &mut model,
                        u16::from_le_bytes([data[offset], data[offset + 1]]),
                    );
                    if !self.acknowledge_position.load(Ordering::Acquire) {
                        model.status &= !0x1000;
                    }
                    if self.feedback.load(Ordering::Acquire) {
                        Self::tpdo(&mut model, data[offset + 2]);
                    }
                }
                _ => {}
            }
            Ok(())
        }

        async fn subscribe(
            &self,
            filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            self.model.lock().unwrap().subscribers.push(tx);
            Ok(Box::new(MockRx { filter, rx }))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    #[test]
    fn wire_layouts_match_firmware_and_mit_padding() {
        let target = FocTarget {
            position_rev: 0.25,
            velocity_rev_s: 0.1,
            torque_permille: -123,
            torque_ff_nm: -0.02,
            kp: 0.05,
            kd: 0.005,
            relative: false,
        };
        let velocity = command_frame(1, FocMode::Velocity, &target, 15).unwrap();
        assert_eq!(velocity.kind(), FrameKind::Data);
        assert_eq!(velocity.id().raw(), 0x201);
        assert_eq!(velocity.data(), &[0xCD, 0xCC, 0xCC, 0x3D, 15, 0, 3]);
        let single = command_frame(1, FocMode::SingleTurn, &target, 0x5F).unwrap();
        assert_eq!(single.data()[6], 255);
        let torque = command_frame(1, FocMode::Torque, &target, 15).unwrap();
        assert_eq!(&torque.data()[..2], &(-123i16).to_le_bytes());
        assert_eq!(torque.data().len(), 5);
        let mit = command_frame(1, FocMode::Mit, &target, 15).unwrap();
        assert_eq!(mit.kind(), FrameKind::Fd { brs: true });
        assert_eq!(mit.id().raw(), 0x501);
        assert_eq!(mit.data().len(), 20);
        assert_eq!(&mit.data()[12..20], &[50, 0, 5, 0, 15, 0, 5, 0]);
        assert!(FocTarget {
            kp: f32::NAN,
            ..target.clone()
        }
        .validate()
        .is_err());
        assert!(FocTarget {
            velocity_rev_s: 40.01,
            ..target
        }
        .validate()
        .is_err());
    }

    #[tokio::test]
    async fn initializes_saved_maps_then_pdo_handshake_and_neutral_reenable() {
        let bus = MockBus::new();
        let session = FocSession::start(bus.clone(), 1).await.unwrap();
        let initialized = session.initialize(FocConfig::default()).await.unwrap();
        assert!(initialized.initialized);
        assert!(!initialized.enabled);
        assert_eq!(initialized.identity, Some([1, 2, 3, 4]));
        {
            let model = bus.model.lock().unwrap();
            assert_eq!(model.od[&(0x1400, 1)], 0x201u32.to_le_bytes());
            assert_eq!(model.od[&(0x1401, 1)], (0x80000251u32).to_le_bytes());
            assert_eq!(model.od[&(0x1600, 0)], [3]);
            assert_eq!(model.od[&(0x1600, 1)], 0x60FF0020u32.to_le_bytes());
            assert_eq!(model.od[&(0x1800, 2)], [255]);
            assert_eq!(model.od[&(0x1800, 5)], 20u16.to_le_bytes());
            assert_eq!(model.od[&(0x2004, 1)], [0]);
            assert_eq!(model.od[&(0x2003, 6)], 1000u16.to_le_bytes());
            assert!(model
                .sent
                .iter()
                .filter(|frame| frame.id().raw() == 0x601)
                .all(|frame| frame.kind() == FrameKind::Data));
        }
        session.enable().await.unwrap();
        assert!(session.snapshot().enabled);
        session
            .set_target(FocTarget {
                velocity_rev_s: 0.2,
                ..FocTarget::default()
            })
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(25)).await;
        session.disable().await.unwrap();
        assert!(!session.snapshot().streaming);
        session.enable().await.unwrap();
        {
            let model = bus.model.lock().unwrap();
            let frames: Vec<_> = model
                .sent
                .iter()
                .filter(|frame| frame.id().raw() == 0x201)
                .collect();
            assert!(frames.iter().any(|frame| frame.data()[4] == 6));
            assert!(frames.iter().any(|frame| frame.data()[4] == 7));
            assert!(frames.iter().any(|frame| frame.data()[4] == 15));
            assert_eq!(&frames.last().unwrap().data()[..4], &0f32.to_le_bytes());
        }
        session.shutdown().await.unwrap();
        assert!(session.enable().await.is_err());
    }

    #[tokio::test]
    async fn position_targets_wait_for_low_and_high_acknowledgements() {
        let bus = MockBus::new();
        let session = FocSession::start(bus.clone(), 1).await.unwrap();
        session
            .initialize(FocConfig {
                mode: FocMode::SingleTurn,
                ..FocConfig::default()
            })
            .await
            .unwrap();
        session.enable().await.unwrap();
        let target = FocTarget {
            position_rev: 0.1,
            relative: true,
            ..FocTarget::default()
        };
        session.set_target(target.clone()).await.unwrap();
        assert_eq!(session.snapshot().statusword.unwrap() & 0x1000, 0x1000);
        session.set_target(target).await.unwrap();
        {
            let model = bus.model.lock().unwrap();
            let frames: Vec<_> = model
                .sent
                .iter()
                .filter(|frame| frame.id().raw() == 0x301)
                .collect();
            let controls: Vec<_> = frames.iter().map(|frame| frame.data()[4]).collect();
            let first_high = controls.iter().position(|cw| cw & 0x10 != 0).unwrap();
            let second_low = controls
                .iter()
                .skip(first_high + 1)
                .position(|cw| cw & 0x10 == 0)
                .unwrap()
                + first_high
                + 1;
            assert!(controls.iter().skip(second_low + 1).any(|cw| *cw == 0x5F));
            assert!(frames.iter().all(|frame| frame.data()[6] == 255));
        }
        session.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn calibration_waits_for_new_run_before_saving() {
        let bus = MockBus::new();
        let session = FocSession::start(bus.clone(), 1).await.unwrap();
        let state = session.calibrate().await.unwrap();
        assert_eq!(state.calibration_state, Some(4));
        assert_eq!(state.calibration_direction, Some(-1));
        let model = bus.model.lock().unwrap();
        assert!(model.calibration_reads.is_empty());
        assert_eq!(model.save_count, 1);
        drop(model);
        session.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn disable_cancels_calibration_without_waiting_for_completion() {
        let bus = MockBus::new();
        let session = FocSession::start(bus.clone(), 1).await.unwrap();
        let cloned = session.clone();
        let task = tokio::spawn(async move { cloned.calibrate().await });
        tokio::time::sleep(Duration::from_millis(160)).await;
        tokio::time::timeout(Duration::from_millis(100), session.disable())
            .await
            .unwrap()
            .unwrap();
        assert!(task.await.unwrap().is_err());
        assert_eq!(bus.model.lock().unwrap().save_count, 0);
        session.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn missing_feedback_times_out_initialization_and_stops_running_stream() {
        let bus = MockBus::new();
        let session = FocSession::start(bus.clone(), 1).await.unwrap();
        session.initialize(FocConfig::default()).await.unwrap();
        session.enable().await.unwrap();
        bus.feedback.store(false, Ordering::Release);
        tokio::time::sleep(Duration::from_millis(240)).await;
        assert!(!session.snapshot().streaming);
        assert!(session.snapshot().last_error.unwrap().contains("200 ms"));
        assert!(session.initialize(FocConfig::default()).await.is_err());
        assert!(!session.snapshot().initialized);
        session.shutdown().await.unwrap();
    }
    #[tokio::test]
    async fn failed_position_ack_stops_stream_and_clears_bridge_permission() {
        let bus = MockBus::new();
        let session = FocSession::start(bus.clone(), 1).await.unwrap();
        session
            .initialize(FocConfig {
                mode: FocMode::Position,
                ..FocConfig::default()
            })
            .await
            .unwrap();
        session.enable().await.unwrap();
        bus.acknowledge_position.store(false, Ordering::Release);
        assert!(session
            .set_target(FocTarget {
                position_rev: 1.0,
                ..FocTarget::default()
            })
            .await
            .is_err());
        assert!(!session.snapshot().streaming);
        let before = bus
            .model
            .lock()
            .unwrap()
            .sent
            .iter()
            .filter(|f| f.id().raw() == 0x301)
            .count();
        tokio::time::sleep(Duration::from_millis(25)).await;
        let model = bus.model.lock().unwrap();
        assert_eq!(
            before,
            model.sent.iter().filter(|f| f.id().raw() == 0x301).count()
        );
        assert_eq!(model.od[&(0x6040, 0)], 0u16.to_le_bytes());
        drop(model);
        session.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn quick_stop_cancels_position_wait_without_old_error_cleanup() {
        let bus = MockBus::new();
        let session = FocSession::start(bus.clone(), 1).await.unwrap();
        session
            .initialize(FocConfig {
                mode: FocMode::Position,
                ..FocConfig::default()
            })
            .await
            .unwrap();
        session.enable().await.unwrap();
        bus.acknowledge_position.store(false, Ordering::Release);
        let cloned = session.clone();
        let target = tokio::spawn(async move {
            cloned
                .set_target(FocTarget {
                    position_rev: 1.0,
                    ..FocTarget::default()
                })
                .await
        });
        tokio::time::sleep(Duration::from_millis(50)).await;
        let preop_count = bus
            .model
            .lock()
            .unwrap()
            .sent
            .iter()
            .filter(|f| f.id().raw() == 0 && f.data()[0] == 0x80)
            .count();
        session.quick_stop().await.unwrap();
        assert!(target.await.unwrap().is_err());
        tokio::time::sleep(Duration::from_millis(25)).await;
        assert_eq!(session.snapshot().phase, "quick stop complete");
        let model = bus.model.lock().unwrap();
        assert_eq!(
            preop_count,
            model
                .sent
                .iter()
                .filter(|f| f.id().raw() == 0 && f.data()[0] == 0x80)
                .count()
        );
        drop(model);
        session.shutdown().await.unwrap();
    }
}
