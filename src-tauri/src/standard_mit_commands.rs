//! Independent IPC and lifecycle for standard-frame MIT motor control.
use std::sync::{atomic::Ordering, Arc};
use tauri::State;

use crate::can_lease::CanTransportGate;
use crate::el05::{
    experiment::{ExperimentConfig, ExperimentData},
    gravity::{GravityCompensationConfig, GravityReference},
    El05Device, El05Identity, El05Mode, El05ParameterValue, El05Protocol, El05Session, El05State,
    El05Target,
};
use crate::el05_commands::{connect_workspace, disconnect_workspace};
use crate::state::AppState;

type CmdResult<T> = Result<T, String>;
fn error(error: impl std::fmt::Display) -> String {
    error.to_string()
}

async fn session(state: &AppState) -> CmdResult<Arc<El05Session>> {
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("Application is shutting down".into());
    }
    state
        .standard_mit
        .lock()
        .await
        .clone()
        .ok_or_else(|| "Standard MIT is not connected".into())
}

#[tauri::command]
pub async fn mit_connect(
    state: State<'_, AppState>,
    can_gate: State<'_, CanTransportGate>,
    iface: String,
    host_id: u8,
    manual_extensions: bool,
) -> CmdResult<El05State> {
    connect_workspace(
        &state,
        &can_gate,
        iface,
        host_id,
        El05Protocol::StandardMit,
        manual_extensions,
    )
    .await
}

#[tauri::command]
pub async fn mit_disconnect(state: State<'_, AppState>) -> CmdResult<()> {
    disconnect_workspace(&state, El05Protocol::StandardMit).await
}

#[tauri::command]
pub async fn mit_get_state(state: State<'_, AppState>) -> CmdResult<Option<El05State>> {
    Ok(state.standard_mit.lock().await.as_ref().map(|app| {
        app.touch_client();
        app.snapshot()
    }))
}

#[tauri::command]
pub async fn mit_scan(state: State<'_, AppState>) -> CmdResult<Vec<El05Device>> {
    let app = session(&state).await?;
    app.discover().await.map_err(error)?;
    Ok(app.devices())
}

#[tauri::command]
pub async fn mit_select_motor(state: State<'_, AppState>, motor_id: u8) -> CmdResult<El05State> {
    session(&state).await?.attach(motor_id).await.map_err(error)
}

#[tauri::command]
pub async fn mit_set_mode(state: State<'_, AppState>, mode: El05Mode) -> CmdResult<El05State> {
    session(&state).await?.set_mode(mode).await.map_err(error)
}

#[tauri::command]
pub async fn mit_enable(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.enable().await.map_err(error)
}
#[tauri::command]
pub async fn mit_stop(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.stop().await.map_err(error)
}
#[tauri::command]
pub async fn mit_clear_fault(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.clear_fault().await.map_err(error)
}
#[tauri::command]
pub async fn mit_set_zero(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.set_zero().await.map_err(error)
}
#[tauri::command]
pub async fn mit_change_id(state: State<'_, AppState>, new_id: u8) -> CmdResult<El05Identity> {
    session(&state)
        .await?
        .change_id(new_id)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_send_target(
    state: State<'_, AppState>,
    target: El05Target,
    repeat: bool,
) -> CmdResult<()> {
    session(&state)
        .await?
        .send_target(target, repeat)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_read_parameter(
    state: State<'_, AppState>,
    index: u16,
) -> CmdResult<El05ParameterValue> {
    session(&state)
        .await?
        .read_parameter(index)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_write_parameter(
    state: State<'_, AppState>,
    index: u16,
    value: f64,
) -> CmdResult<()> {
    session(&state)
        .await?
        .write_parameter(index, value)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_save_parameters(state: State<'_, AppState>) -> CmdResult<El05Identity> {
    session(&state)
        .await?
        .save_parameters()
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_set_reporting(state: State<'_, AppState>, enabled: bool) -> CmdResult<()> {
    session(&state)
        .await?
        .set_reporting(enabled)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_start_experiment(
    state: State<'_, AppState>,
    config: ExperimentConfig,
) -> CmdResult<()> {
    session(&state)
        .await?
        .start_experiment(config)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_experiment_data(
    state: State<'_, AppState>,
    after_sequence: Option<u64>,
) -> CmdResult<ExperimentData> {
    Ok(session(&state).await?.experiment_data_since(after_sequence))
}

#[tauri::command]
pub async fn mit_capture_gravity_reference(
    state: State<'_, AppState>,
) -> CmdResult<GravityReference> {
    session(&state)
        .await?
        .capture_gravity_reference()
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_start_gravity_compensation(
    state: State<'_, AppState>,
    config: GravityCompensationConfig,
) -> CmdResult<()> {
    session(&state)
        .await?
        .start_gravity_compensation(config)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn mit_stop_gravity_compensation(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state)
        .await?
        .stop_gravity_compensation()
        .await
        .map_err(error)
}
