//! Opt-in extensions from EL05 manual 260922, printed pp.52–55.
//! Basic MIT-only sessions never transmit these configuration/dictionary frames.
use super::*;

pub(super) fn mit_mode_raw(mode: El05Mode) -> Result<u8> {
    match mode {
        El05Mode::Mit => Ok(0),
        El05Mode::PositionCsp => Ok(1),
        El05Mode::Velocity => Ok(2),
        _ => bail!("standard MIT supports MIT (0), position CSP (1) and velocity (2)"),
    }
}

pub(super) fn mit_mode_from_raw(raw: u8) -> Option<El05Mode> {
    match raw {
        0 => Some(El05Mode::Mit),
        1 => Some(El05Mode::PositionCsp),
        2 => Some(El05Mode::Velocity),
        _ => None,
    }
}

impl Inner {
    pub(super) fn require_manual_extensions(&self) -> Result<()> {
        self.check_running()?;
        if self.protocol != El05Protocol::StandardMit || !self.manual_extensions {
            bail!("reconnect with EL05 manual extensions enabled to use this command");
        }
        Ok(())
    }

    fn require_stopped_configuration(&self) -> Result<()> {
        self.require_manual_extensions()?;
        let s = self.state.lock().unwrap();
        if s.armed
            || s.safety_hold
            || s.view.enabled
            || s.experiment.status.active
            || s.gravity.status.active
        {
            bail!("stop the motor before changing MIT device settings");
        }
        Ok(())
    }

    pub(super) async fn standard_uid_command(
        &self,
        id: u8,
        reply_id: u8,
        argument: u8,
        opcode: u8,
    ) -> Result<El05Identity> {
        // The manual's UID reply uses the motor ID, while motion uses the host
        // ID. Identical addresses make those two untagged payloads ambiguous.
        if self.state.lock().unwrap().view.reply_id == Some(u16::from(reply_id)) {
            bail!("UID acknowledgements require different motor and motion reply CAN IDs");
        }
        self.identity_request_locked(reply_id, mit::configuration_frame(id, argument, opcode)?)
            .await?
            .ok_or_else(|| anyhow!("MIT configuration acknowledgement timed out; the write may have reached the drive"))
    }
}

impl El05Session {
    pub async fn save_parameters(&self) -> Result<El05Identity> {
        let _guard = self.inner.gate.lock().await;
        self.inner.require_stopped_configuration()?;
        let (id, _, _) = self.inner.selected()?;
        self.inner.standard_uid_command(id, id, 0xff, 0xf8).await
    }

    pub async fn set_reporting(&self, enabled: bool) -> Result<()> {
        let _guard = self.inner.gate.lock().await;
        self.inner.require_stopped_configuration()?;
        self.inner.reporting_locked(enabled).await
    }

    pub(super) async fn change_standard_id(&self, new_id: u8) -> Result<El05Identity> {
        let _guard = self.inner.gate.lock().await;
        self.inner.require_stopped_configuration()?;
        let (old_id, host, _) = self.inner.selected()?;
        if new_id > 127
            || new_id == old_id
            || u16::from(new_id) == self.snapshot().reply_id.unwrap_or(u16::from(host))
        {
            bail!("new motor ID must be 0–127 and differ from current motor and reply IDs");
        }
        self.inner.stop_locked(false).await?;
        // Probe with Stop rather than a configuration write. A responding
        // address is occupied, so do not transmit an ID change to it.
        let probe_started = Instant::now();
        self.inner.send(mit::stop_frame(new_id)?).await?;
        tokio::time::sleep(READ_TIMEOUT).await;
        if self
            .inner
            .state
            .lock()
            .unwrap()
            .devices
            .get(&new_id)
            .is_some_and(|record| record.seen >= probe_started)
        {
            bail!("MIT ID {new_id} already responds; ID was not changed");
        }
        let result = self
            .inner
            .standard_uid_command(old_id, new_id, new_id, 0xfa)
            .await;
        {
            let mut s = self.inner.state.lock().unwrap();
            let tx_count = s.view.tx_count;
            s.view = El05State::with_protocol(host, true, El05Protocol::StandardMit);
            s.view.tx_count = tx_count;
            s.last_seen = None;
            s.feedback_times.clear();
            s.position_window.clear();
            s.current_seen = None;
            s.dictionary_velocity_seen = None;
            s.last_sent_target = None;
            s.last_target_sent = None;
            s.mode_confirmed = false;
            s.devices.remove(&old_id);
            s.mit_requests.clear();
            s.selection_generation = s.selection_generation.wrapping_add(1);
            s.gravity
                .invalidate_reference("motor ID changed; capture the downward reference again");
            if let Ok(identity) = &result {
                s.view.motor_id = Some(new_id);
                s.view.attached = true;
                s.view.identity = Some(identity.clone());
            } else {
                s.view.last_error = Some(format!(
                    "ID change unconfirmed. Scan and select ID {old_id} or {new_id} again."
                ));
            }
        }
        let identity = result?;
        self.inner.stop_locked(false).await?;
        // Explicitly select MIT mode instead of assuming a persisted mode.
        self.set_mode_locked(El05Mode::Mit).await?;
        Ok(identity)
    }
}

pub(super) fn observe_standard_extension(inner: &Inner, frame: &CanFrame) -> bool {
    if frame.kind() != FrameKind::Data || frame.is_fd() || frame.data().len() != 8 {
        return false;
    }
    let CanId::Standard(id) = frame.id() else {
        return false;
    };
    let ty = id >> 8;
    let source = id as u8;
    if !inner.manual_extensions {
        return false;
    }
    let data = frame.data();
    let mut s = inner.state.lock().unwrap();
    if matches!(ty, 3 | 4) {
        if ty == 3
            && data[2..4] == [0, 0]
            && s.pending.as_ref().is_some_and(|(motor, index, _)| {
                *motor == source && *index == u16::from_le_bytes([data[0], data[1]])
            })
        {
            let index = u16::from_le_bytes([data[0], data[1]]);
            let result = decode_parameter(index, &data[4..]);
            if let Ok(value) = &result {
                match index {
                    0x7005 => {
                        if let Some(mode) = mit_mode_from_raw(value.value as u8) {
                            if s.armed && s.view.mode != mode {
                                s.receive_error = Some(
                                    "MIT mode changed outside this session; motor stopped".into(),
                                );
                            }
                            s.view.mode = mode;
                        } else {
                            s.mode_confirmed = false;
                        }
                    }
                    0x7019 => s.view.mechanical_position_rad = Some(value.value),
                    0x701a => {
                        s.view.current_a = Some(value.value);
                        s.current_seen = Some(Instant::now());
                    }
                    0x701b => {
                        s.view.dictionary_velocity_rad_s = Some(value.value);
                        s.dictionary_velocity_seen = Some(Instant::now());
                    }
                    0x701c => s.view.voltage_v = Some(value.value),
                    _ => {}
                }
            }
            if let Some((_, _, sender)) = s.pending.take() {
                let _ = sender.send(result);
            }
        }
        return true;
    }
    if ty == 0
        && s.pending_identity
            .as_ref()
            .is_some_and(|(motor, _)| *motor == source)
        && data.iter().any(|byte| *byte != 0)
        && !(data[..6] == [0xff; 6] && matches!(data[7], 0xf8..=0xfe | 0x01))
    {
        let identity = El05Identity {
            motor_id: source,
            uid: hex::encode(data),
        };
        if s.view.motor_id == Some(source) {
            s.view.identity = Some(identity.clone());
        }
        if let Some((_, sender)) = s.pending_identity.take() {
            let _ = sender.send(identity);
        }
        return true;
    }
    false
}
