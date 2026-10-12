use std::sync::atomic::Ordering;
use std::sync::Arc;

use tauri::State;

use crate::can_lease::{CanOwner, CanTransportGate};
use crate::host_can::{self, HostCanStatus};
use crate::lingkong::{
    Action, LingkongInitialization, LingkongParameterReadResult, LingkongProbe,
    LingkongRebootResult, LingkongSession, LingkongSnapshot,
};
use crate::lingkong_params::ParameterValue;
use crate::state::AppState;

async fn session(state: &AppState) -> Result<Arc<LingkongSession>, String> {
    state
        .lingkong
        .lock()
        .await
        .clone()
        .ok_or_else(|| "Lingkong CAN is not connected".into())
}

#[tauri::command]
pub async fn lingkong_start(
    state: State<'_, AppState>,
    can_gate: State<'_, CanTransportGate>,
    spec: String,
    bitrate: u32,
) -> Result<(), String> {
    let _extension = state.extension_op.lock().await;
    let mut guard = state.lingkong.lock().await;
    if guard.is_some() {
        return Err("Lingkong CAN session is already open".into());
    }
    let was_active = state.extension_runtime_active.swap(true, Ordering::AcqRel);
    let result = async {
        let lease = can_gate.try_acquire(CanOwner::Lingkong)?;
        LingkongSession::start(spec.trim(), bitrate, lease).await
    }
    .await;
    match result {
        Ok(app) => {
            *guard = Some(app);
            Ok(())
        }
        Err(error) => {
            state
                .extension_runtime_active
                .store(was_active, Ordering::Release);
            Err(error)
        }
    }
}

pub(crate) async fn stop_session(state: &AppState) -> Result<(), String> {
    let app = match session(state).await {
        Ok(app) => app,
        Err(_) => return Ok(()),
    };
    app.shutdown().await?;
    let mut guard = state.lingkong.lock().await;
    if guard
        .as_ref()
        .is_some_and(|current| Arc::ptr_eq(current, &app))
    {
        guard.take();
    }
    Ok(())
}

pub(crate) async fn force_stop_session(state: &AppState) {
    if let Some(app) = state.lingkong.lock().await.take() {
        app.force_shutdown().await;
    }
}

#[tauri::command]
pub async fn lingkong_stop(state: State<'_, AppState>) -> Result<(), String> {
    let _extension = state.extension_op.lock().await;
    stop_session(&state).await?;
    state
        .extension_runtime_active
        .store(false, Ordering::Release);
    Ok(())
}

#[tauri::command]
pub async fn lingkong_scan(state: State<'_, AppState>) -> Result<Vec<LingkongProbe>, String> {
    session(&state).await?.scan().await
}

#[tauri::command]
pub async fn lingkong_snapshot(
    state: State<'_, AppState>,
    id: u8,
) -> Result<LingkongSnapshot, String> {
    session(&state).await?.snapshot(id).await
}

#[tauri::command]
pub async fn lingkong_initialize(
    state: State<'_, AppState>,
    id: u8,
) -> Result<LingkongInitialization, String> {
    session(&state).await?.initialize(id).await
}

#[tauri::command]
pub async fn lingkong_read_parameters(
    state: State<'_, AppState>,
    id: u8,
) -> Result<LingkongParameterReadResult, String> {
    session(&state).await?.read_parameters(id).await
}

#[tauri::command]
pub async fn lingkong_write_parameter(
    state: State<'_, AppState>,
    id: u8,
    key: String,
    values: Vec<u32>,
) -> Result<ParameterValue, String> {
    session(&state)
        .await?
        .write_parameter(id, &key, values)
        .await
}

#[tauri::command]
pub async fn lingkong_save_settings(state: State<'_, AppState>, id: u8) -> Result<(), String> {
    session(&state).await?.save_settings(id).await
}

#[tauri::command]
pub async fn lingkong_reboot(
    state: State<'_, AppState>,
    id: u8,
) -> Result<LingkongRebootResult, String> {
    session(&state).await?.reboot(id).await
}

#[tauri::command]
pub async fn lingkong_action(
    state: State<'_, AppState>,
    id: u8,
    kind: String,
    value: f64,
    limit: u16,
) -> Result<LingkongSnapshot, String> {
    let action = Action::parse(&kind, value, limit)?;
    session(&state).await?.action(id, action).await
}

#[tauri::command]
pub async fn host_can_inspect(iface: String) -> Result<HostCanStatus, String> {
    tauri::async_runtime::spawn_blocking(move || host_can::inspect(iface.trim()))
        .await
        .map_err(|error| error.to_string())?
}

#[tauri::command]
pub async fn host_can_apply(
    state: State<'_, AppState>,
    can_gate: State<'_, CanTransportGate>,
    iface: String,
    bitrate: u32,
    password: String,
) -> Result<HostCanStatus, String> {
    // Native window close and normal disconnect wait until the down/configure/up
    // sequence has finished, so a password dialog cannot leave the link down.
    let _operation = state.device_settings_operation.acquire().await;
    let _lease = can_gate.try_acquire(CanOwner::HostConfig)?;
    tauri::async_runtime::spawn_blocking(move || host_can::apply(iface.trim(), bitrate, password))
        .await
        .map_err(|error| error.to_string())?
}
