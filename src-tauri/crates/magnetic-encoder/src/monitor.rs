//! Long-running CAN subscriptions and health/freshness accounting.

use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanBusState, CanFilter, CanIoError};
use thiserror::Error;
use tokio::sync::watch;
use tokio::task::JoinHandle;
use tokio::time::{self, MissedTickBehavior};

use crate::protocol::{
    GelloJointProtocol, HeartbeatSample, NmtState, PositionSample, ProtocolConfigError,
    SensorStatus, StatusSample,
};

#[derive(Debug, Clone, Copy)]
pub struct EncoderConfig {
    pub protocol: GelloJointProtocol,
    pub position_stale_after: Duration,
    pub status_stale_after: Duration,
    pub bus_health_period: Duration,
    pub bus_health_timeout: Duration,
    /// Take ownership of keeping this encoder node Operational.
    pub auto_start_node: bool,
    /// Minimum interval between NMT Start retries while heartbeats still report
    /// Boot-up, Stopped, or Pre-Operational.
    pub nmt_start_retry_after: Duration,
}

impl Default for EncoderConfig {
    fn default() -> Self {
        Self {
            protocol: GelloJointProtocol::default(),
            // Firmware publishes position every 1 ms. A 250 ms threshold
            // avoids false alarms during host scheduling stalls while still
            // making an unplug obvious to an operator.
            position_stale_after: Duration::from_millis(250),
            // Firmware publishes status every 100 ms.
            status_stale_after: Duration::from_millis(500),
            bus_health_period: Duration::from_millis(500),
            bus_health_timeout: Duration::from_millis(300),
            auto_start_node: true,
            nmt_start_retry_after: Duration::from_millis(500),
        }
    }
}

impl EncoderConfig {
    pub fn for_node(node_id: u8) -> Result<Self, ProtocolConfigError> {
        Ok(Self {
            protocol: GelloJointProtocol::new(node_id, crate::COUNTS_PER_TURN)?,
            ..Self::default()
        })
    }

    fn validate(self) -> Result<Self, MonitorError> {
        for (name, value) in [
            ("position_stale_after", self.position_stale_after),
            ("status_stale_after", self.status_stale_after),
            ("bus_health_period", self.bus_health_period),
            ("bus_health_timeout", self.bus_health_timeout),
            ("nmt_start_retry_after", self.nmt_start_retry_after),
        ] {
            if value.is_zero() {
                return Err(MonitorError::InvalidDuration(name));
            }
        }
        Ok(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommunicationState {
    Waiting,
    Online,
    Stale,
    ReceiveError,
}

#[derive(Debug, Clone)]
pub struct BusHealth {
    pub state: Option<can_transport::CanControllerState>,
    pub tx_errors: Option<u16>,
    pub rx_errors: Option<u16>,
    pub query_error: Option<String>,
    pub age: Option<Duration>,
}

#[derive(Debug, Clone)]
pub struct EncoderSnapshot {
    pub node_id: u8,
    pub counts_per_turn: u32,
    pub communication: CommunicationState,
    pub monitor_age: Duration,
    pub position: Option<PositionSample>,
    /// Monotonic host arrival time of this position frame.
    pub position_received_at: Option<Instant>,
    pub position_age: Option<Duration>,
    pub status: Option<StatusSample>,
    pub status_age: Option<Duration>,
    pub status_is_fresh: bool,
    pub status_valid_until: Option<Instant>,
    pub heartbeat: Option<HeartbeatSample>,
    pub heartbeat_age: Option<Duration>,
    pub heartbeat_frames: u64,
    pub nmt_start_attempts: u64,
    pub nmt_start_error: Option<String>,
    pub position_frames: u64,
    pub status_frames: u64,
    pub invalid_frames: u64,
    pub dropped_frames: u64,
    pub last_decode_error: Option<String>,
    pub receive_error: Option<String>,
    pub bus_health: BusHealth,
}

impl EncoderSnapshot {
    /// `false` when the firmware says the transmitted position is retained
    /// from before a missing-magnet or I/O failure.
    pub fn angle_is_usable(&self) -> bool {
        self.communication == CommunicationState::Online
            && self.position.is_some()
            && self
                .status
                .map(|sample| sample.status.angle_is_usable())
                .unwrap_or(false)
            && self.status_is_fresh
    }

    pub fn sensor_status(&self) -> Option<SensorStatus> {
        self.status.map(|sample| sample.status)
    }
}

/// Owns the background subscriptions. Dropping it aborts both tasks; callers
/// should prefer [`Self::shutdown`] so receivers unsubscribe cleanly.
pub struct EncoderMonitor {
    config: EncoderConfig,
    shared: Arc<RwLock<SharedState>>,
    shutdown_tx: watch::Sender<bool>,
    receive_task: Option<JoinHandle<()>>,
    health_task: Option<JoinHandle<()>>,
}

impl EncoderMonitor {
    pub async fn start(bus: Arc<dyn CanBus>, config: EncoderConfig) -> Result<Self, MonitorError> {
        let config = config.validate()?;
        let position_cob_id = config.protocol.position_cob_id();
        let status_cob_id = config.protocol.status_cob_id();
        let heartbeat_cob_id = config.protocol.heartbeat_cob_id();
        let position_rx = bus
            .subscribe(CanFilter::exact_standard(position_cob_id))
            .await
            .map_err(|source| MonitorError::Subscribe {
                stream: "TPDO1 position",
                cob_id: position_cob_id,
                source,
            })?;
        let status_rx = bus
            .subscribe(CanFilter::exact_standard(status_cob_id))
            .await
            .map_err(|source| MonitorError::Subscribe {
                stream: "TPDO2 status",
                cob_id: status_cob_id,
                source,
            })?;
        let heartbeat_rx = bus
            .subscribe(CanFilter::exact_standard(heartbeat_cob_id))
            .await
            .map_err(|source| MonitorError::Subscribe {
                stream: "CANopen heartbeat",
                cob_id: heartbeat_cob_id,
                source,
            })?;

        let shared = Arc::new(RwLock::new(SharedState::new()));
        let (shutdown_tx, shutdown_rx) = watch::channel(false);
        let receive_task = tokio::spawn(run_receivers(
            bus.clone(),
            config,
            position_rx,
            status_rx,
            heartbeat_rx,
            shared.clone(),
            shutdown_rx.clone(),
        ));
        let health_task = tokio::spawn(run_bus_health(
            bus,
            config.bus_health_period,
            config.bus_health_timeout,
            shared.clone(),
            shutdown_rx,
        ));

        Ok(Self {
            config,
            shared,
            shutdown_tx,
            receive_task: Some(receive_task),
            health_task: Some(health_task),
        })
    }

    pub fn snapshot(&self) -> EncoderSnapshot {
        let now = Instant::now();
        let state = self
            .shared
            .read()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        state.snapshot(self.config, now)
    }

    pub async fn shutdown(&mut self) {
        let _ = self.shutdown_tx.send(true);
        if let Some(task) = self.receive_task.take() {
            let _ = task.await;
        }
        if let Some(task) = self.health_task.take() {
            let _ = task.await;
        }
    }
}

impl Drop for EncoderMonitor {
    fn drop(&mut self) {
        let _ = self.shutdown_tx.send(true);
        if let Some(task) = self.receive_task.take() {
            task.abort();
        }
        if let Some(task) = self.health_task.take() {
            task.abort();
        }
    }
}

#[derive(Debug, Error)]
pub enum MonitorError {
    #[error("{0} must be greater than zero")]
    InvalidDuration(&'static str),
    #[error("could not subscribe to {stream} (CAN ID 0x{cob_id:03X}): {source}")]
    Subscribe {
        stream: &'static str,
        cob_id: u16,
        #[source]
        source: CanIoError,
    },
}

struct Timed<T> {
    value: T,
    received_at: Instant,
}

struct SharedState {
    started_at: Instant,
    position: Option<Timed<PositionSample>>,
    status: Option<Timed<StatusSample>>,
    heartbeat: Option<Timed<HeartbeatSample>>,
    position_frames: u64,
    status_frames: u64,
    heartbeat_frames: u64,
    invalid_frames: u64,
    dropped_frames: u64,
    last_decode_error: Option<String>,
    receive_error: Option<String>,
    nmt_start_attempts: u64,
    nmt_start_error: Option<String>,
    last_nmt_start_at: Option<Instant>,
    bus_state: Option<CanBusState>,
    bus_query_error: Option<String>,
    bus_updated_at: Option<Instant>,
}

impl SharedState {
    fn new() -> Self {
        Self {
            started_at: Instant::now(),
            position: None,
            status: None,
            heartbeat: None,
            position_frames: 0,
            status_frames: 0,
            heartbeat_frames: 0,
            invalid_frames: 0,
            dropped_frames: 0,
            last_decode_error: None,
            receive_error: None,
            nmt_start_attempts: 0,
            nmt_start_error: None,
            last_nmt_start_at: None,
            bus_state: None,
            bus_query_error: None,
            bus_updated_at: None,
        }
    }

    fn snapshot(&self, config: EncoderConfig, now: Instant) -> EncoderSnapshot {
        let position_age = self
            .position
            .as_ref()
            .map(|sample| now.saturating_duration_since(sample.received_at));
        let status_age = self
            .status
            .as_ref()
            .map(|sample| now.saturating_duration_since(sample.received_at));
        let status_is_fresh = status_age
            .map(|age| age <= config.status_stale_after)
            .unwrap_or(false);
        let heartbeat_age = self
            .heartbeat
            .as_ref()
            .map(|sample| now.saturating_duration_since(sample.received_at));
        let communication = classify_communication(
            self.receive_error.is_some(),
            position_age,
            config.position_stale_after,
        );
        EncoderSnapshot {
            node_id: config.protocol.node_id(),
            counts_per_turn: config.protocol.counts_per_turn(),
            communication,
            monitor_age: now.saturating_duration_since(self.started_at),
            position: self.position.as_ref().map(|sample| sample.value),
            position_received_at: self.position.as_ref().map(|sample| sample.received_at),
            position_age,
            status: self.status.as_ref().map(|sample| sample.value),
            status_age,
            status_is_fresh,
            status_valid_until: self
                .status
                .as_ref()
                .and_then(|s| s.received_at.checked_add(config.status_stale_after)),
            heartbeat: self.heartbeat.as_ref().map(|sample| sample.value),
            heartbeat_age,
            heartbeat_frames: self.heartbeat_frames,
            nmt_start_attempts: self.nmt_start_attempts,
            nmt_start_error: self.nmt_start_error.clone(),
            position_frames: self.position_frames,
            status_frames: self.status_frames,
            invalid_frames: self.invalid_frames,
            dropped_frames: self.dropped_frames,
            last_decode_error: self.last_decode_error.clone(),
            receive_error: self.receive_error.clone(),
            bus_health: BusHealth {
                state: self.bus_state.and_then(|state| state.state),
                tx_errors: self.bus_state.and_then(|state| state.tx_errors),
                rx_errors: self.bus_state.and_then(|state| state.rx_errors),
                query_error: self.bus_query_error.clone(),
                age: self
                    .bus_updated_at
                    .map(|updated| now.saturating_duration_since(updated)),
            },
        }
    }
}

fn classify_communication(
    receive_failed: bool,
    position_age: Option<Duration>,
    stale_after: Duration,
) -> CommunicationState {
    if receive_failed {
        CommunicationState::ReceiveError
    } else {
        match position_age {
            None => CommunicationState::Waiting,
            Some(age) if age <= stale_after => CommunicationState::Online,
            Some(_) => CommunicationState::Stale,
        }
    }
}

enum ReceiveEvent {
    Shutdown,
    Position(Result<can_transport::CanFrame, CanIoError>),
    Status(Result<can_transport::CanFrame, CanIoError>),
    Heartbeat(Result<can_transport::CanFrame, CanIoError>),
}

async fn run_receivers(
    bus: Arc<dyn CanBus>,
    config: EncoderConfig,
    mut position_rx: Box<dyn can_transport::CanRx>,
    mut status_rx: Box<dyn can_transport::CanRx>,
    mut heartbeat_rx: Box<dyn can_transport::CanRx>,
    shared: Arc<RwLock<SharedState>>,
    mut shutdown: watch::Receiver<bool>,
) {
    let protocol = config.protocol;
    if config.auto_start_node {
        request_node_start(&bus, protocol, &shared).await;
    }
    loop {
        let event = tokio::select! {
            changed = shutdown.changed() => {
                let _ = changed;
                ReceiveEvent::Shutdown
            }
            result = position_rx.recv() => ReceiveEvent::Position(result),
            result = status_rx.recv() => ReceiveEvent::Status(result),
            result = heartbeat_rx.recv() => ReceiveEvent::Heartbeat(result),
        };
        match event {
            ReceiveEvent::Shutdown => break,
            ReceiveEvent::Position(Ok(frame)) => {
                let now = Instant::now();
                let mut state = shared
                    .write()
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
                match protocol.decode_position(&frame) {
                    Ok(value) => {
                        state.position = Some(Timed {
                            value,
                            received_at: now,
                        });
                        state.position_frames = state.position_frames.saturating_add(1);
                    }
                    Err(error) => record_decode_error(&mut state, error.to_string()),
                }
            }
            ReceiveEvent::Status(Ok(frame)) => {
                let now = Instant::now();
                let mut state = shared
                    .write()
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
                match protocol.decode_status(&frame) {
                    Ok(value) => {
                        state.status = Some(Timed {
                            value,
                            received_at: now,
                        });
                        state.status_frames = state.status_frames.saturating_add(1);
                    }
                    Err(error) => record_decode_error(&mut state, error.to_string()),
                }
            }
            ReceiveEvent::Heartbeat(Ok(frame)) => {
                let now = Instant::now();
                let should_start = {
                    let mut state = shared
                        .write()
                        .unwrap_or_else(|poisoned| poisoned.into_inner());
                    match protocol.decode_heartbeat(&frame) {
                        Ok(value) => {
                            state.heartbeat = Some(Timed {
                                value,
                                received_at: now,
                            });
                            state.heartbeat_frames = state.heartbeat_frames.saturating_add(1);
                            if value.state == NmtState::Operational {
                                state.nmt_start_error = None;
                            }
                            config.auto_start_node
                                && value.state.needs_start_command()
                                && state
                                    .last_nmt_start_at
                                    .map(|last| {
                                        now.saturating_duration_since(last)
                                            >= config.nmt_start_retry_after
                                    })
                                    .unwrap_or(true)
                        }
                        Err(error) => {
                            record_decode_error(&mut state, error.to_string());
                            false
                        }
                    }
                };
                if should_start {
                    request_node_start(&bus, protocol, &shared).await;
                }
            }
            ReceiveEvent::Position(Err(CanIoError::Lagged { dropped }))
            | ReceiveEvent::Status(Err(CanIoError::Lagged { dropped }))
            | ReceiveEvent::Heartbeat(Err(CanIoError::Lagged { dropped })) => {
                let mut state = shared
                    .write()
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
                state.dropped_frames = state.dropped_frames.saturating_add(dropped);
            }
            ReceiveEvent::Position(Err(error))
            | ReceiveEvent::Status(Err(error))
            | ReceiveEvent::Heartbeat(Err(error)) => {
                let mut state = shared
                    .write()
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
                state.receive_error = Some(error.to_string());
                break;
            }
        }
    }
}

async fn request_node_start(
    bus: &Arc<dyn CanBus>,
    protocol: GelloJointProtocol,
    shared: &Arc<RwLock<SharedState>>,
) {
    let frame = match protocol.start_remote_node_frame() {
        Ok(frame) => frame,
        Err(error) => {
            let mut state = shared
                .write()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            state.nmt_start_error = Some(error.to_string());
            return;
        }
    };
    {
        let mut state = shared
            .write()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        state.nmt_start_attempts = state.nmt_start_attempts.saturating_add(1);
        state.last_nmt_start_at = Some(Instant::now());
    }
    let result = bus.send(frame).await;
    let mut state = shared
        .write()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    state.nmt_start_error = result.err().map(|error| error.to_string());
}

fn record_decode_error(state: &mut SharedState, error: String) {
    state.invalid_frames = state.invalid_frames.saturating_add(1);
    state.last_decode_error = Some(error);
}

async fn run_bus_health(
    bus: Arc<dyn CanBus>,
    period: Duration,
    timeout: Duration,
    shared: Arc<RwLock<SharedState>>,
    mut shutdown: watch::Receiver<bool>,
) {
    let mut interval = time::interval(period);
    interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            changed = shutdown.changed() => {
                let _ = changed;
                break;
            }
            _ = interval.tick() => {
                let result = time::timeout(timeout, bus.bus_state()).await;
                let now = Instant::now();
                let mut state = shared.write().unwrap_or_else(|poisoned| poisoned.into_inner());
                state.bus_updated_at = Some(now);
                match result {
                    Ok(Ok(bus_state)) => {
                        state.bus_state = bus_state;
                        state.bus_query_error = None;
                    }
                    Ok(Err(error)) => {
                        state.bus_query_error = Some(error.to_string());
                    }
                    Err(_) => {
                        state.bus_query_error = Some(format!(
                            "CAN controller health query timed out after {} ms",
                            timeout.as_millis()
                        ));
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use async_trait::async_trait;
    use can_transport::{CanCapabilities, CanControllerState, CanFrame, CanId, CanRx};
    use tokio::sync::{mpsc, Mutex};

    #[test]
    fn communication_state_is_conservative() {
        let stale_after = Duration::from_millis(250);
        assert_eq!(
            classify_communication(false, None, stale_after),
            CommunicationState::Waiting
        );
        assert_eq!(
            classify_communication(false, Some(Duration::from_millis(249)), stale_after),
            CommunicationState::Online
        );
        assert_eq!(
            classify_communication(false, Some(Duration::from_millis(251)), stale_after),
            CommunicationState::Stale
        );
        assert_eq!(
            classify_communication(true, Some(Duration::ZERO), stale_after),
            CommunicationState::ReceiveError
        );
    }

    #[test]
    fn zero_duration_config_is_rejected() {
        let config = EncoderConfig {
            bus_health_timeout: Duration::ZERO,
            ..EncoderConfig::default()
        };
        assert!(matches!(
            config.validate(),
            Err(MonitorError::InvalidDuration("bus_health_timeout"))
        ));
    }

    #[tokio::test]
    async fn monitor_starts_node_and_publishes_valid_angle_snapshot() {
        let bus = Arc::new(MockBus::default());
        let trait_bus: Arc<dyn CanBus> = bus.clone();
        let mut monitor = EncoderMonitor::start(trait_bus, EncoderConfig::default())
            .await
            .unwrap();

        // The monitor sends targeted NMT Start immediately, before waiting for
        // telemetry, so firmware that booted Pre-Operational can publish TPDOs.
        for _ in 0..20 {
            if !bus.sent.lock().await.is_empty() {
                break;
            }
            time::sleep(Duration::from_millis(2)).await;
        }
        let sent = bus.sent.lock().await;
        assert!(!sent.is_empty());
        assert_eq!(sent[0].id(), CanId::Standard(0));
        assert_eq!(sent[0].data(), &[0x01, crate::DEFAULT_NODE_ID]);
        drop(sent);

        let protocol = GelloJointProtocol::default();
        bus.emit(CanFrame::new_data(protocol.heartbeat_cob_id(), &[0x05]).unwrap())
            .await;
        let mut position = [0u8; 8];
        position[0..4].copy_from_slice(&2048u32.to_le_bytes());
        position[4..8].copy_from_slice(&123u32.to_le_bytes());
        bus.emit(CanFrame::new_data(protocol.position_cob_id(), &position).unwrap())
            .await;
        bus.emit(CanFrame::new_data(protocol.status_cob_id(), &[0, 0]).unwrap())
            .await;

        for _ in 0..20 {
            if monitor.snapshot().position_frames > 0 && monitor.snapshot().status_frames > 0 {
                break;
            }
            time::sleep(Duration::from_millis(2)).await;
        }
        let snapshot = monitor.snapshot();
        assert_eq!(snapshot.communication, CommunicationState::Online);
        assert_eq!(snapshot.position.unwrap().position_counts, 2048);
        assert_eq!(snapshot.position.unwrap().angle_degrees, 180.0);
        assert!(!snapshot.position.unwrap().format.is_can_fd);
        assert!(!snapshot.position.unwrap().format.bitrate_switch);
        assert_eq!(snapshot.sensor_status(), Some(SensorStatus::Ok));
        assert_eq!(snapshot.heartbeat.unwrap().state, NmtState::Operational);
        assert!(snapshot.angle_is_usable());
        assert!(snapshot.nmt_start_attempts >= 1);
        assert_eq!(
            snapshot.bus_health.state,
            Some(CanControllerState::ErrorActive)
        );

        monitor.shutdown().await;
    }

    #[derive(Default)]
    struct MockBus {
        sent: Mutex<Vec<CanFrame>>,
        subscribers: Mutex<Vec<(CanFilter, mpsc::Sender<CanFrame>)>>,
    }

    impl MockBus {
        async fn emit(&self, frame: CanFrame) {
            let senders: Vec<_> = self
                .subscribers
                .lock()
                .await
                .iter()
                .filter(|(filter, _)| filter.matches(&frame))
                .map(|(_, sender)| sender.clone())
                .collect();
            for sender in senders {
                sender.send(frame).await.unwrap();
            }
        }
    }

    #[async_trait]
    impl CanBus for MockBus {
        async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
            self.sent.lock().await.push(frame);
            Ok(())
        }

        async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (sender, receiver) = mpsc::channel(32);
            self.subscribers.lock().await.push((filter, sender));
            Ok(Box::new(MockRx { receiver }))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: false,
                max_dlen: 8,
            }
        }

        async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
            Ok(Some(CanBusState {
                state: Some(CanControllerState::ErrorActive),
                tx_errors: Some(0),
                rx_errors: Some(0),
            }))
        }
    }

    struct MockRx {
        receiver: mpsc::Receiver<CanFrame>,
    }

    #[async_trait]
    impl CanRx for MockRx {
        async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
            self.receiver.recv().await.ok_or(CanIoError::Disconnected)
        }

        fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
            match self.receiver.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
            }
        }
    }
}
