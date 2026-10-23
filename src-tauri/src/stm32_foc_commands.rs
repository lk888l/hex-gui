//! IPC for the G431 firmware. This tool owns its transport directly: the
//! generic motor manager uses different OD units and must not control it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use can_transport::CanBus;
use serde::Serialize;
use tauri::State;
use tokio::sync::Mutex;

use crate::can_lease::{CanOwner, CanTransportGate};
use crate::stm32_foc::{FocConfig, FocSession, FocState, FocTarget};
use crate::stm32_foc_discovery::{FocDevice, FocDiscovery};

#[derive(Serialize)]
pub struct FocConnection {
    connected: bool,
    selected_node_id: Option<u8>,
}

struct Connected {
    bus: Arc<dyn CanBus>,
    discovery: FocDiscovery,
    session: Option<Arc<FocSession>>,
}

impl Connected {
    fn selected_node_id(&self) -> Option<u8> {
        self.session
            .as_ref()
            .map(|session| session.snapshot().node_id)
    }

    async fn select_node(&mut self, node_id: u8) -> Result<FocState, String> {
        if !(1..=127).contains(&node_id) {
            return Err("Node-ID must be within 1..127".into());
        }
        if let Some(session) = &self.session {
            let state = session.snapshot();
            if state.node_id == node_id {
                return Ok(state);
            }
        }
        if let Some(previous) = self.session.take() {
            self.discovery.note_session(&previous.snapshot());
            previous.shutdown().await.map_err(|error| {
                format!("Previous motor session was stopped with an error; no motor is selected: {error:#}")
            })?;
        }
        let session = FocSession::start(self.bus.clone(), node_id)
            .await
            .map_err(|error| format!("{error:#}"))?;
        let state = session.snapshot();
        self.discovery.note_session(&state);
        self.session = Some(session);
        Ok(state)
    }
}

#[derive(Default)]
pub struct FocRuntime {
    connection: Mutex<Option<Connected>>,
    active: AtomicBool,
}

impl FocRuntime {
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Acquire)
    }

    async fn session(&self) -> Result<Arc<FocSession>, String> {
        self.connection
            .lock()
            .await
            .as_ref()
            .and_then(|connection| connection.session.clone())
            .ok_or_else(|| "Select a CANopen node from the device list first".into())
    }

    pub async fn disconnect(&self) -> Result<(), String> {
        let mut guard = self.connection.lock().await;
        let result = if let Some(mut connection) = guard.take() {
            let stopped = if let Some(session) = connection.session.take() {
                session
                    .shutdown()
                    .await
                    .map_err(|error| format!("{error:#}"))
            } else {
                Ok(())
            };
            connection.discovery.stop().await;
            stopped
        } else {
            Ok(())
        };
        self.active.store(false, Ordering::Release);
        result
    }
}

#[tauri::command]
pub async fn stm32_foc_connect(
    runtime: State<'_, FocRuntime>,
    gate: State<'_, CanTransportGate>,
    iface: String,
    data_bitrate: u32,
) -> Result<FocConnection, String> {
    let mut guard = runtime.connection.lock().await;
    if guard.is_some() {
        return Err("G431 FOC is already connected".into());
    }
    let lease = gate.try_acquire(CanOwner::Stm32Foc)?;
    runtime.active.store(true, Ordering::Release);
    let opened = async {
        let (bus, _) = crate::backend::open_bus(iface.trim(), data_bitrate, false, lease).await?;
        let bus = crate::backend::protect_feature_bus(bus);
        let discovery = FocDiscovery::start(&bus).await?;
        Ok::<_, anyhow::Error>(Connected {
            bus,
            discovery,
            session: None,
        })
    }
    .await;
    match opened {
        Ok(connection) => {
            *guard = Some(connection);
            Ok(FocConnection {
                connected: true,
                selected_node_id: None,
            })
        }
        Err(error) => {
            runtime.active.store(false, Ordering::Release);
            Err(format!("{error:#}"))
        }
    }
}

#[tauri::command]
pub async fn stm32_foc_list_devices(
    runtime: State<'_, FocRuntime>,
) -> Result<Vec<FocDevice>, String> {
    let guard = runtime.connection.lock().await;
    let Some(connection) = guard.as_ref() else {
        return Ok(Vec::new());
    };
    let selected = connection
        .session
        .as_ref()
        .map(|session| session.snapshot());
    connection.discovery.devices(selected.as_ref())
}

#[tauri::command]
pub async fn stm32_foc_select_node(
    runtime: State<'_, FocRuntime>,
    node_id: u8,
) -> Result<FocState, String> {
    let mut guard = runtime.connection.lock().await;
    let connection = guard
        .as_mut()
        .ok_or_else(|| "G431 FOC is not connected".to_string())?;
    connection.select_node(node_id).await
}

#[tauri::command]
pub async fn stm32_foc_forget_offline(runtime: State<'_, FocRuntime>) -> Result<(), String> {
    let guard = runtime.connection.lock().await;
    if let Some(connection) = guard.as_ref() {
        connection
            .discovery
            .forget_offline(connection.selected_node_id());
    }
    Ok(())
}

#[tauri::command]
pub async fn stm32_foc_disconnect(runtime: State<'_, FocRuntime>) -> Result<(), String> {
    runtime.disconnect().await
}

#[tauri::command]
pub async fn stm32_foc_get_state(runtime: State<'_, FocRuntime>) -> Result<FocState, String> {
    Ok(runtime.session().await?.snapshot())
}

#[tauri::command]
pub async fn stm32_foc_initialize(
    runtime: State<'_, FocRuntime>,
    config: FocConfig,
) -> Result<FocState, String> {
    let session = runtime.session().await?;
    let state = session
        .initialize(config)
        .await
        .map_err(|error| format!("{error:#}"))?;
    let guard = runtime.connection.lock().await;
    if let Some(connection) = guard.as_ref() {
        if connection
            .session
            .as_ref()
            .is_some_and(|active| Arc::ptr_eq(active, &session))
        {
            connection.discovery.initialized(&state);
        }
    }
    Ok(state)
}

#[tauri::command]
pub async fn stm32_foc_calibrate(runtime: State<'_, FocRuntime>) -> Result<FocState, String> {
    runtime
        .session()
        .await?
        .calibrate()
        .await
        .map_err(|error| format!("{error:#}"))
}

#[tauri::command]
pub async fn stm32_foc_enable(runtime: State<'_, FocRuntime>) -> Result<(), String> {
    runtime
        .session()
        .await?
        .enable()
        .await
        .map_err(|error| format!("{error:#}"))
}

#[tauri::command]
pub async fn stm32_foc_set_target(
    runtime: State<'_, FocRuntime>,
    target: FocTarget,
) -> Result<(), String> {
    runtime
        .session()
        .await?
        .set_target(target)
        .await
        .map_err(|error| format!("{error:#}"))
}

#[tauri::command]
pub async fn stm32_foc_halt(runtime: State<'_, FocRuntime>, halted: bool) -> Result<(), String> {
    runtime
        .session()
        .await?
        .halt(halted)
        .await
        .map_err(|error| format!("{error:#}"))
}

#[tauri::command]
pub async fn stm32_foc_quick_stop(runtime: State<'_, FocRuntime>) -> Result<(), String> {
    runtime
        .session()
        .await?
        .quick_stop()
        .await
        .map_err(|error| format!("{error:#}"))
}

#[tauri::command]
pub async fn stm32_foc_disable(runtime: State<'_, FocRuntime>) -> Result<(), String> {
    runtime
        .session()
        .await?
        .disable()
        .await
        .map_err(|error| format!("{error:#}"))
}

#[tauri::command]
pub async fn stm32_foc_reset_fault(runtime: State<'_, FocRuntime>) -> Result<FocState, String> {
    runtime
        .session()
        .await?
        .reset_fault()
        .await
        .map_err(|error| format!("{error:#}"))
}

#[tauri::command]
pub async fn stm32_foc_save(runtime: State<'_, FocRuntime>) -> Result<(), String> {
    runtime
        .session()
        .await?
        .save()
        .await
        .map_err(|error| format!("{error:#}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;
    use std::sync::Mutex as StdMutex;

    use can_transport::{CanCapabilities, CanFilter, CanFrame, CanIoError, CanRx};
    use tokio::sync::mpsc;

    #[derive(Default)]
    struct SwitchBus {
        sent: StdMutex<Vec<CanFrame>>,
        subscribers: StdMutex<Vec<mpsc::UnboundedSender<CanFrame>>>,
        fail_sends: AtomicBool,
    }

    struct SwitchRx {
        filter: CanFilter,
        receive: mpsc::UnboundedReceiver<CanFrame>,
    }

    #[async_trait::async_trait]
    impl CanRx for SwitchRx {
        async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
            loop {
                let frame = self.receive.recv().await.ok_or(CanIoError::Disconnected)?;
                if self.filter.matches(&frame) {
                    return Ok(frame);
                }
            }
        }
        fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
            loop {
                match self.receive.try_recv() {
                    Ok(frame) if self.filter.matches(&frame) => return Ok(Some(frame)),
                    Ok(_) => {}
                    Err(mpsc::error::TryRecvError::Empty) => return Ok(None),
                    Err(_) => return Err(CanIoError::Disconnected),
                }
            }
        }
    }

    #[async_trait::async_trait]
    impl CanBus for SwitchBus {
        async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
            self.sent.lock().unwrap().push(frame);
            if self.fail_sends.load(Ordering::Acquire) {
                return Err(CanIoError::Disconnected);
            }
            let id = frame.id().raw();
            if (0x601..=0x67F).contains(&id) && frame.data().len() == 8 {
                let data = frame.data();
                let mut response = [0; 8];
                response[1..4].copy_from_slice(&data[1..4]);
                if data[0] == 0x40 {
                    response[0] = 0x4B;
                    response[4] = 0x40;
                } else {
                    response[0] = 0x60;
                }
                let response = CanFrame::new_data((id - 0x80) as u16, &response).unwrap();
                self.subscribers
                    .lock()
                    .unwrap()
                    .retain(|sender| sender.send(response).is_ok());
            }
            Ok(())
        }
        async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (sender, receive) = mpsc::unbounded_channel();
            self.subscribers.lock().unwrap().push(sender);
            Ok(Box::new(SwitchRx { filter, receive }))
        }
        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    async fn connected(bus: Arc<dyn CanBus>) -> Connected {
        let discovery = FocDiscovery::start(&bus).await.unwrap();
        Connected {
            bus,
            discovery,
            session: None,
        }
    }

    #[tokio::test]
    async fn selecting_nodes_is_passive_and_switching_finishes_old_bridge_off() {
        let bus = Arc::new(SwitchBus::default());
        let mut connection = connected(bus.clone()).await;
        assert_eq!(connection.select_node(7).await.unwrap().node_id, 7);
        let previous = connection.session.as_ref().unwrap().clone();
        assert_eq!(connection.select_node(7).await.unwrap().node_id, 7);
        assert!(Arc::ptr_eq(connection.session.as_ref().unwrap(), &previous));
        assert!(bus.sent.lock().unwrap().is_empty());
        assert_eq!(connection.select_node(12).await.unwrap().node_id, 12);
        assert!(previous.enable().await.is_err());
        {
            let sent = bus.sent.lock().unwrap();
            assert!(sent
                .iter()
                .any(|frame| frame.id().raw() == 0 && frame.data() == [0x80, 7]));
            assert!(sent.iter().any(
                |frame| frame.id().raw() == 0x607 && frame.data()[..4] == [0x2B, 0x40, 0x60, 0]
            ));
            assert!(sent
                .iter()
                .all(|frame| matches!(frame.id().raw(), 0 | 0x607)));
        }
        let selected = connection.session.as_ref().unwrap().snapshot();
        let rows = connection.discovery.devices(Some(&selected)).unwrap();
        assert_eq!(
            rows.iter().map(|row| row.node_id).collect::<Vec<_>>(),
            [7, 12]
        );
        assert!(!rows[0].selected && !rows[0].initialized && !rows[0].enabled);
        assert!(rows[1].selected && !rows[1].online && rows[1].heartbeat_count == 0);
        let runtime = FocRuntime {
            connection: Mutex::new(Some(connection)),
            active: AtomicBool::new(true),
        };
        runtime.disconnect().await.unwrap();
        assert!(!runtime.is_active());
        assert!(runtime.connection.lock().await.is_none());
        assert!(bus
            .subscribers
            .lock()
            .unwrap()
            .iter()
            .all(|sender| sender.is_closed()));
    }

    #[tokio::test]
    async fn failed_switch_clears_selection_and_disconnect_still_releases_discovery() {
        let bus = Arc::new(SwitchBus::default());
        let mut connection = connected(bus.clone()).await;
        connection.select_node(1).await.unwrap();
        bus.fail_sends.store(true, Ordering::Release);
        let error = connection.select_node(2).await.unwrap_err();
        assert!(error.contains("no motor is selected"));
        assert!(connection.session.is_none());
        // Selecting remains available after the failed stop; it does not
        // require heartbeat or create another protocol write.
        assert_eq!(connection.select_node(2).await.unwrap().node_id, 2);
        let runtime = FocRuntime {
            connection: Mutex::new(Some(connection)),
            active: AtomicBool::new(true),
        };
        assert!(runtime.disconnect().await.is_err());
        assert!(!runtime.is_active());
        assert!(runtime.connection.lock().await.is_none());
        assert!(bus
            .subscribers
            .lock()
            .unwrap()
            .iter()
            .all(|sender| sender.is_closed()));
    }
}
