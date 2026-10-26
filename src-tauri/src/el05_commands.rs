//! IPC and physical-adapter lifecycle for the independent EL05 workspace.

use std::sync::{atomic::Ordering, Arc};

use tauri::State;

use crate::can_lease::{CanOwner, CanTransportGate};
use crate::el05::{
    El05Device, El05Identity, El05Mode, El05ParameterValue, El05Protocol, El05Session, El05State,
    El05Target, El05Version,
};
use crate::state::AppState;

type CmdResult<T> = Result<T, String>;

#[tauri::command]
pub async fn el05_start_experiment(
    state: State<'_, AppState>,
    config: crate::el05::experiment::ExperimentConfig,
) -> CmdResult<()> {
    session(&state)
        .await?
        .start_experiment(config)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn el05_capture_gravity_reference(
    state: State<'_, AppState>,
) -> CmdResult<crate::el05::gravity::GravityReference> {
    session(&state)
        .await?
        .capture_gravity_reference()
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn el05_start_gravity_compensation(
    state: State<'_, AppState>,
    config: crate::el05::gravity::GravityCompensationConfig,
) -> CmdResult<()> {
    session(&state)
        .await?
        .start_gravity_compensation(config)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn el05_stop_gravity_compensation(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state)
        .await?
        .stop_gravity_compensation()
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn el05_experiment_data(
    state: State<'_, AppState>,
    after_sequence: Option<u64>,
) -> CmdResult<crate::el05::experiment::ExperimentData> {
    Ok(session(&state).await?.experiment_data_since(after_sequence))
}

fn error(e: impl std::fmt::Display) -> String {
    e.to_string()
}

async fn session(state: &AppState) -> CmdResult<Arc<El05Session>> {
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("Application is shutting down".into());
    }
    state
        .el05
        .lock()
        .await
        .clone()
        .ok_or_else(|| "EL05 is not connected".into())
}

#[tauri::command]
pub async fn el05_connect(
    state: State<'_, AppState>,
    can_gate: State<'_, CanTransportGate>,
    iface: String,
    host_id: u8,
) -> CmdResult<El05State> {
    connect_workspace(
        &state,
        &can_gate,
        iface,
        host_id,
        El05Protocol::Native,
        false,
    )
    .await
}

pub(crate) fn workspace_slot(
    state: &AppState,
    protocol: El05Protocol,
) -> &tokio::sync::Mutex<Option<Arc<El05Session>>> {
    match protocol {
        El05Protocol::Native => &state.el05,
        El05Protocol::StandardMit => &state.standard_mit,
    }
}

pub(crate) async fn connect_workspace(
    state: &AppState,
    can_gate: &CanTransportGate,
    iface: String,
    host_id: u8,
    protocol: El05Protocol,
    manual_extensions: bool,
) -> CmdResult<El05State> {
    let _lifecycle = state.extension_op.lock().await;
    if state.shutdown_requested.load(Ordering::Acquire) {
        return Err("Application is shutting down".into());
    }
    let mut guard = workspace_slot(state, protocol).lock().await;
    if guard.is_some() {
        return Err("Motor workspace is already connected".into());
    }
    // Mark before opening: native close must wait for this lifecycle lock.
    state
        .extension_runtime_active
        .store(true, Ordering::Release);
    let permit = can_gate.try_acquire(match protocol {
        El05Protocol::Native => CanOwner::El05,
        El05Protocol::StandardMit => CanOwner::StandardMit,
    })?;
    let bus = crate::backend::open_classic_1m_bus(&iface, permit)
        .await
        .map_err(error)?;
    let bus = crate::backend::protect_feature_bus(bus);
    let app = El05Session::start_with_options(bus, host_id, protocol, manual_extensions)
        .await
        .map_err(error)?;
    if state.shutdown_requested.load(Ordering::Acquire) {
        let result = app.shutdown().await;
        if result.is_err() {
            app.force_stop();
        }
        return Err("Application is shutting down".into());
    }
    let snapshot = app.snapshot();
    *guard = Some(app);
    Ok(snapshot)
}

/// Retain the session on a failed manual stop so the operator can retry.
/// On native close, always terminate its workers after the bounded stop.
pub(crate) async fn stop_session(state: &AppState, force: bool) -> CmdResult<()> {
    let native = stop_workspace(state, El05Protocol::Native, force).await;
    let mit = stop_workspace(state, El05Protocol::StandardMit, force).await;
    let mut errors = Vec::new();
    if let Err(error) = native {
        errors.push(format!("EL05: {error}"));
    }
    if let Err(error) = mit {
        errors.push(format!("Standard MIT: {error}"));
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

pub(crate) async fn stop_workspace(
    state: &AppState,
    protocol: El05Protocol,
    force: bool,
) -> CmdResult<()> {
    let mut guard = workspace_slot(state, protocol).lock().await;
    let Some(app) = guard.as_ref() else {
        return Ok(());
    };
    let result = app.shutdown().await.map_err(error);
    if result.is_ok() || force {
        if force {
            app.force_stop();
        }
        guard.take();
    }
    result
}

#[tauri::command]
pub async fn el05_disconnect(state: State<'_, AppState>) -> CmdResult<()> {
    disconnect_workspace(&state, El05Protocol::Native).await
}

pub(crate) async fn disconnect_workspace(
    state: &AppState,
    protocol: El05Protocol,
) -> CmdResult<()> {
    let _lifecycle = state.extension_op.lock().await;
    let had_session = workspace_slot(state, protocol).lock().await.is_some();
    stop_workspace(state, protocol, false).await?;
    if had_session {
        // CAN ownership is exclusive, but the independent USB serial servo
        // may still need the native close cleanup path.
        state.extension_runtime_active.store(
            state.unitree_servo.lock().await.is_some()
                || state.el05.lock().await.is_some()
                || state.standard_mit.lock().await.is_some(),
            Ordering::Release,
        );
    }
    Ok(())
}

#[tauri::command]
pub async fn el05_get_state(state: State<'_, AppState>) -> CmdResult<Option<El05State>> {
    Ok(state.el05.lock().await.as_ref().map(|app| {
        app.touch_client();
        app.snapshot()
    }))
}

#[tauri::command]
pub async fn el05_scan(state: State<'_, AppState>) -> CmdResult<Vec<El05Device>> {
    let app = session(&state).await?;
    app.discover().await.map_err(error)?;
    Ok(app.devices())
}

#[tauri::command]
pub async fn el05_select_motor(state: State<'_, AppState>, motor_id: u8) -> CmdResult<El05State> {
    let app = session(&state).await?;
    app.attach(motor_id).await.map_err(error)?;
    Ok(app.snapshot())
}

#[tauri::command]
pub async fn el05_set_mode(state: State<'_, AppState>, mode: El05Mode) -> CmdResult<El05State> {
    let app = session(&state).await?;
    app.set_mode(mode).await.map_err(error)?;
    Ok(app.snapshot())
}

#[tauri::command]
pub async fn el05_enable(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.enable().await.map_err(error)
}

#[tauri::command]
pub async fn el05_stop(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.stop().await.map_err(error)
}

#[tauri::command]
pub async fn el05_clear_fault(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.clear_fault().await.map_err(error)
}

#[tauri::command]
pub async fn el05_set_zero(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.set_zero().await.map_err(error)
}

#[tauri::command]
pub async fn el05_read_identity(state: State<'_, AppState>) -> CmdResult<El05Identity> {
    session(&state).await?.read_identity().await.map_err(error)
}

#[tauri::command]
pub async fn el05_change_id(state: State<'_, AppState>, new_id: u8) -> CmdResult<El05Identity> {
    session(&state)
        .await?
        .change_id(new_id)
        .await
        .map_err(error)
}

#[tauri::command]
pub async fn el05_read_version(state: State<'_, AppState>) -> CmdResult<El05Version> {
    session(&state).await?.read_version().await.map_err(error)
}

#[tauri::command]
pub async fn el05_send_target(
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
pub async fn el05_stop_stream(state: State<'_, AppState>) -> CmdResult<()> {
    session(&state).await?.stop_stream().await.map_err(error)
}

#[tauri::command]
pub async fn el05_read_parameter(
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
pub async fn el05_write_parameter(
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
