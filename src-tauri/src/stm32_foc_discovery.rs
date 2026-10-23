//! Passive CANopen heartbeat inventory for the independent G431 workspace.
//! Discovery does not send NMT, SDO probes, SYNC, or motion commands.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use can_transport::{CanBus, CanFilter, CanFrame, FrameKind};
use serde::Serialize;
use tokio::task::JoinHandle;

use crate::stm32_foc::FocState;

const HEARTBEAT_ONLINE_TIMEOUT: Duration = Duration::from_secs(3);

#[derive(Clone, Debug, Serialize)]
pub struct FocDevice {
    pub node_id: u8,
    pub online: bool,
    pub nmt_state: u8,
    pub heartbeat_age_ms: u64,
    pub heartbeat_count: u64,
    pub selected: bool,
    pub initialized: bool,
    pub enabled: bool,
    pub identity: Option<[u32; 4]>,
    pub error_code: Option<u16>,
}

#[derive(Default)]
struct Node {
    last_heartbeat: Option<Instant>,
    nmt_state: u8,
    heartbeat_count: u64,
    identity: Option<[u32; 4]>,
    identity_invalidated: bool,
    error_code: Option<u16>,
}

impl Node {
    fn online(&self, now: Instant) -> bool {
        self.last_heartbeat
            .is_some_and(|last| now.saturating_duration_since(last) <= HEARTBEAT_ONLINE_TIMEOUT)
    }
}

#[derive(Default)]
struct Inventory {
    nodes: BTreeMap<u8, Node>,
    receive_error: Option<String>,
}

impl Inventory {
    fn observe(&mut self, frame: &CanFrame, now: Instant) {
        if frame.id().is_extended() || frame.kind() != FrameKind::Data {
            return;
        }
        let id = frame.id().raw();
        if !(0x701..=0x77F).contains(&id) || frame.data().len() != 1 {
            return;
        }
        let state = frame.data()[0];
        if !matches!(state, 0 | 4 | 5 | 127) {
            return;
        }
        let node = self.nodes.entry((id - 0x700) as u8).or_default();
        node.last_heartbeat = Some(now);
        node.nmt_state = state;
        node.heartbeat_count = node.heartbeat_count.saturating_add(1);
        if state == 0 {
            node.identity = None;
            node.identity_invalidated = true;
            node.error_code = None;
        }
    }

    fn note_session(&mut self, state: &FocState) {
        let node = self.nodes.entry(state.node_id).or_default();
        if state.identity.is_some() && !node.identity_invalidated {
            node.identity = state.identity;
        }
        node.error_code = state.error_code;
    }

    fn snapshot(&self, selected: Option<&FocState>, now: Instant) -> Vec<FocDevice> {
        self.nodes
            .iter()
            .map(|(&node_id, node)| {
                let state = selected.filter(|state| state.node_id == node_id);
                FocDevice {
                    node_id,
                    online: node.online(now),
                    nmt_state: node.nmt_state,
                    // Manual selection may create a row without a heartbeat.
                    // heartbeat_count == 0 distinguishes unknown age/state.
                    heartbeat_age_ms: node.last_heartbeat.map_or(0, |last| {
                        now.saturating_duration_since(last).as_millis() as u64
                    }),
                    heartbeat_count: node.heartbeat_count,
                    selected: state.is_some(),
                    initialized: state.is_some_and(|state| state.initialized),
                    enabled: state.is_some_and(|state| state.enabled),
                    identity: node.identity,
                    error_code: state.and_then(|state| state.error_code).or(node.error_code),
                }
            })
            .collect()
    }

    fn forget_offline(&mut self, selected_node_id: Option<u8>, now: Instant) {
        self.nodes
            .retain(|&node_id, node| Some(node_id) == selected_node_id || node.online(now));
    }
}

pub struct FocDiscovery {
    inventory: Arc<Mutex<Inventory>>,
    task: Option<JoinHandle<()>>,
}

impl FocDiscovery {
    pub async fn start(bus: &Arc<dyn CanBus>) -> Result<Self> {
        let mut rx = bus.subscribe(CanFilter::pass_all_standard()).await?;
        let inventory = Arc::new(Mutex::new(Inventory::default()));
        let received = inventory.clone();
        let task = tokio::spawn(async move {
            loop {
                match rx.recv().await {
                    Ok(frame) => received.lock().unwrap().observe(&frame, Instant::now()),
                    Err(error) => {
                        received.lock().unwrap().receive_error =
                            Some(format!("CAN heartbeat receive failed: {error}"));
                        break;
                    }
                }
            }
        });
        Ok(Self {
            inventory,
            task: Some(task),
        })
    }

    pub fn devices(&self, selected: Option<&FocState>) -> Result<Vec<FocDevice>, String> {
        let mut inventory = self.inventory.lock().unwrap();
        if let Some(error) = &inventory.receive_error {
            return Err(error.clone());
        }
        if let Some(state) = selected {
            inventory.note_session(state);
        }
        Ok(inventory.snapshot(selected, Instant::now()))
    }

    pub fn note_session(&self, state: &FocState) {
        self.inventory.lock().unwrap().note_session(state);
    }

    pub fn initialized(&self, state: &FocState) {
        let mut inventory = self.inventory.lock().unwrap();
        // This is called by the initialization command on successful return,
        // rather than relying on the UI to poll an intermediate disabled state.
        inventory
            .nodes
            .entry(state.node_id)
            .or_default()
            .identity_invalidated = false;
        inventory.note_session(state);
    }

    pub fn forget_offline(&self, selected_node_id: Option<u8>) {
        self.inventory
            .lock()
            .unwrap()
            .forget_offline(selected_node_id, Instant::now());
    }

    pub async fn stop(mut self) {
        if let Some(task) = self.task.take() {
            task.abort();
            let _ = task.await;
        }
    }
}

impl Drop for FocDiscovery {
    fn drop(&mut self) {
        if let Some(task) = self.task.take() {
            task.abort();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use can_transport::{CanCapabilities, CanIoError, CanRx};
    use std::sync::atomic::{AtomicUsize, Ordering};
    use tokio::sync::mpsc;

    #[test]
    fn only_classic_one_byte_heartbeat_states_discover_valid_nodes() {
        let now = Instant::now();
        let mut inventory = Inventory::default();
        for frame in [
            CanFrame::new_data(0x700u16, &[127]).unwrap(),
            CanFrame::new_data(0x780u16, &[127]).unwrap(),
            CanFrame::new_data(0x701u16, &[]).unwrap(),
            CanFrame::new_data(0x701u16, &[127, 0]).unwrap(),
            CanFrame::new_data(0x701u16, &[9]).unwrap(),
            CanFrame::new_fd(0x701u16, &[127], true).unwrap(),
            CanFrame::new_data(0x181u16, &[127]).unwrap(),
        ] {
            inventory.observe(&frame, now);
        }
        assert!(inventory.nodes.is_empty());
        for (node, state) in [(127u16, 0), (2, 4), (1, 5), (3, 127)] {
            inventory.observe(&CanFrame::new_data(0x700 + node, &[state]).unwrap(), now);
        }
        let rows = inventory.snapshot(None, now);
        assert_eq!(
            rows.iter().map(|row| row.node_id).collect::<Vec<_>>(),
            [1, 2, 3, 127]
        );
        assert!(rows
            .iter()
            .all(|row| row.online && row.heartbeat_count == 1));
    }

    #[test]
    fn expiry_and_resume_follow_only_received_heartbeats_and_keep_selection() {
        let now = Instant::now();
        let mut inventory = Inventory::default();
        inventory.observe(&CanFrame::new_data(0x702u16, &[127]).unwrap(), now);
        inventory.observe(&CanFrame::new_data(0x701u16, &[5]).unwrap(), now);
        let later = now + Duration::from_millis(3001);
        assert!(inventory
            .snapshot(None, later)
            .iter()
            .all(|node| !node.online));
        inventory.observe(&CanFrame::new_data(0x701u16, &[0]).unwrap(), later);
        let rows = inventory.snapshot(None, later);
        assert!(rows[0].online);
        assert_eq!(rows[0].nmt_state, 0);
        assert_eq!(rows[0].heartbeat_count, 2);
        assert!(!rows[1].online);
        inventory.nodes.entry(4).or_default();
        inventory.forget_offline(Some(4), later);
        assert_eq!(inventory.nodes.keys().copied().collect::<Vec<_>>(), [1, 4]);
    }

    struct SilentBus {
        sends: AtomicUsize,
        sender: Mutex<Option<mpsc::UnboundedSender<CanFrame>>>,
    }

    struct TestRx(mpsc::UnboundedReceiver<CanFrame>);

    #[async_trait::async_trait]
    impl CanRx for TestRx {
        async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
            self.0.recv().await.ok_or(CanIoError::Disconnected)
        }
        fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
            match self.0.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(_) => Err(CanIoError::Disconnected),
            }
        }
    }

    #[async_trait::async_trait]
    impl CanBus for SilentBus {
        async fn send(&self, _: CanFrame) -> Result<(), CanIoError> {
            self.sends.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }
        async fn subscribe(&self, _: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (sender, receiver) = mpsc::unbounded_channel();
            *self.sender.lock().unwrap() = Some(sender);
            Ok(Box::new(TestRx(receiver)))
        }
        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    #[tokio::test]
    async fn monitor_discovers_without_transmitting_and_releases_receiver_on_stop() {
        let raw = Arc::new(SilentBus {
            sends: AtomicUsize::new(0),
            sender: Mutex::new(None),
        });
        let bus: Arc<dyn CanBus> = raw.clone();
        let discovery = FocDiscovery::start(&bus).await.unwrap();
        let sender = raw.sender.lock().unwrap().clone().unwrap();
        sender
            .send(CanFrame::new_data(0x705u16, &[127]).unwrap())
            .unwrap();
        tokio::task::yield_now().await;
        let rows = discovery.devices(None).unwrap();
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].node_id, 5);
        assert_eq!(raw.sends.load(Ordering::Relaxed), 0);
        discovery.stop().await;
        assert!(sender.is_closed());
    }
}
