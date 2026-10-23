//! IPC/lifecycle adapter for the independent USB serial servo workspace.
use std::sync::{atomic::Ordering, Arc};
use tauri::State;

use crate::state::AppState;
use crate::unitree_protocol::ServoTarget;
use crate::unitree_servo::{
    BusTestRequest, ServoOperation, ServoPort, ServoSession, ServoSettings, ServoState,
};

type CmdResult<T> = Result<T, String>;
fn error(e: impl std::fmt::Display) -> String {
    e.to_string()
}

async fn session(state: &AppState) -> CmdResult<Arc<ServoSession>> {
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("Application is shutting down".into());
    }
    state
        .unitree_servo
        .lock()
        .await
        .clone()
        .ok_or_else(|| "Unitree serial port is not connected".into())
}

#[tauri::command]
pub async fn unitree_list_ports() -> CmdResult<Vec<ServoPort>> {
    tokio::task::spawn_blocking(crate::unitree_servo::list_ports)
        .await
        .map_err(error)?
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_connect(
    state: State<'_, AppState>,
    port: String,
    id: u8,
    settings: ServoSettings,
) -> CmdResult<ServoState> {
    let _lifecycle = state.extension_op.lock().await;
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("Application is shutting down".into());
    }
    let mut guard = state.unitree_servo.lock().await;
    if guard.is_some() {
        return Err("Unitree serial port is already connected".into());
    }
    // Native close must wait for the lifecycle lock even while USB is opening.
    state
        .extension_runtime_active
        .store(true, Ordering::Release);
    let app = tokio::task::spawn_blocking(move || ServoSession::open(port, id, settings))
        .await
        .map_err(error)?
        .map_err(|e| format!("{e:#}"))?;
    if state.shutdown_requested.load(Ordering::Acquire) {
        let _ = app.shutdown(true).await;
        return Err("Application is shutting down".into());
    }
    let snapshot = app.snapshot();
    *guard = Some(app);
    Ok(snapshot)
}

pub(crate) async fn stop_session(state: &AppState, force: bool) -> CmdResult<()> {
    let mut guard = state.unitree_servo.lock().await;
    let Some(app) = guard.as_ref() else {
        return Ok(());
    };
    let result = app.shutdown(force).await.map_err(|e| format!("{e:#}"));
    if result.is_ok() || force {
        guard.take();
    }
    result
}

#[tauri::command]
pub async fn unitree_disconnect(state: State<'_, AppState>) -> CmdResult<()> {
    let _lifecycle = state.extension_op.lock().await;
    stop_session(&state, false).await
    // Keep the shared extension marker: serial and CAN runtimes may coexist.
}

#[tauri::command]
pub async fn unitree_get_state(state: State<'_, AppState>) -> CmdResult<Option<ServoState>> {
    Ok(state.unitree_servo.lock().await.as_ref().map(|app| {
        app.touch_client();
        app.snapshot()
    }))
}

#[tauri::command]
pub async fn unitree_scan(state: State<'_, AppState>) -> CmdResult<ServoState> {
    session(&state)
        .await?
        .execute(ServoOperation::Scan)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_select(state: State<'_, AppState>, id: u8) -> CmdResult<ServoState> {
    session(&state)
        .await?
        .execute(ServoOperation::Select(id))
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_send_target(
    state: State<'_, AppState>,
    target: ServoTarget,
    repeat: bool,
) -> CmdResult<ServoState> {
    session(&state)
        .await?
        .execute(ServoOperation::Send { target, repeat })
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_stop(state: State<'_, AppState>) -> CmdResult<ServoState> {
    session(&state)
        .await?
        .execute(ServoOperation::Stop)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_set_mode(state: State<'_, AppState>, mode: u8) -> CmdResult<ServoState> {
    session(&state)
        .await?
        .execute(ServoOperation::SetMode(mode))
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_start_bus_test(
    state: State<'_, AppState>,
    request: BusTestRequest,
) -> CmdResult<ServoState> {
    session(&state)
        .await?
        .execute(ServoOperation::StartBusTest(request))
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_configure(
    state: State<'_, AppState>,
    settings: ServoSettings,
) -> CmdResult<ServoState> {
    session(&state)
        .await?
        .execute(ServoOperation::Configure(settings))
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_change_id(
    state: State<'_, AppState>,
    old_id: u8,
    new_id: u8,
) -> CmdResult<ServoState> {
    // Native close/disconnect uses this same lock order. A Flash transaction
    // must complete (including APP recovery) before USB teardown can proceed.
    let _lifecycle = state.extension_op.lock().await;
    let _operation = state.device_settings_operation.acquire().await;
    session(&state)
        .await?
        .execute(ServoOperation::ChangeId { old_id, new_id })
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn unitree_restore_app(state: State<'_, AppState>, id: u8) -> CmdResult<ServoState> {
    let _lifecycle = state.extension_op.lock().await;
    let _operation = state.device_settings_operation.acquire().await;
    session(&state)
        .await?
        .execute(ServoOperation::RestoreApp(id))
        .await
        .map_err(error)
}
