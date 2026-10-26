//! EL05-compatible, 11-bit standard-frame MIT wire format.
//!
//! The standard protocol packs p16/v12/Kp12/Kd12/torque12. It is separate
//! from RobStride's native 29-bit motion-control messages. The reply CAN ID
//! belongs to the host configuration; the motor address is payload byte 0.

use anyhow::{anyhow, bail, Result};
use can_transport::{CanFrame, CanId, FrameKind};

use super::El05Target;

const STOP: [u8; 8] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd];
const ENABLE: [u8; 8] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc];

const ZERO: [u8; 8] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe];

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Feedback {
    pub source: u8,
    pub reply_id: u16,
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub torque_nm: f64,
    pub temperature_c: f64,
    /// Raw firmware bits. Some compatible drives report zero while running.
    pub status: u8,
    pub fault: bool,
    pub warning: bool,
    pub payload: [u8; 8],
}

fn frame(id: u8, payload: [u8; 8]) -> Result<CanFrame> {
    CanFrame::new_data(CanId::Standard(u16::from(id)), &payload)
        .map_err(|error| anyhow!("build standard MIT frame: {error}"))
}

pub(super) fn stop_frame(id: u8) -> Result<CanFrame> {
    frame(id, STOP)
}

pub(super) fn enable_frame(id: u8) -> Result<CanFrame> {
    frame(id, ENABLE)
}

pub(super) fn zero_frame(id: u8) -> Result<CanFrame> {
    frame(id, ZERO)
}

fn pack(value: f64, minimum: f64, maximum: f64, bits: u32) -> u16 {
    ((value - minimum) * f64::from((1u32 << bits) - 1) / (maximum - minimum)) as u16
}

fn unpack(value: u16, minimum: f64, maximum: f64, bits: u32) -> f64 {
    f64::from(value) * (maximum - minimum) / f64::from((1u32 << bits) - 1) + minimum
}

pub(super) fn target_frame(id: u8, target: El05Target) -> Result<CanFrame> {
    let El05Target::Mit {
        position_rad,
        velocity_rad_s,
        torque_nm,
        kp,
        kd,
    } = target
    else {
        bail!("standard MIT compatibility supports MIT targets only");
    };
    super::validate_target(target)?;
    let p = pack(position_rad, -12.57, 12.57, 16);
    let v = pack(velocity_rad_s, -50.0, 50.0, 12);
    let kp = pack(kp, 0.0, 500.0, 12);
    let kd = pack(kd, 0.0, 5.0, 12);
    let t = pack(torque_nm, -6.0, 6.0, 12);
    frame(
        id,
        [
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            ((v & 0x0f) << 4 | (kp >> 8)) as u8,
            kp as u8,
            (kd >> 4) as u8,
            ((kd & 0x0f) << 4 | (t >> 8)) as u8,
            t as u8,
        ],
    )
}

/// Decode only the wire format; callers also bind source/reply IDs to their
/// selected device and exclude locally transmitted command echoes. Arbitrary
/// MIT command bytes cannot always be distinguished from telemetry by content.
/// Faults, warnings, high temperatures and raw state bits remain observable.
pub(super) fn decode_feedback(frame: &CanFrame) -> Option<Feedback> {
    if frame.kind() != FrameKind::Data || frame.is_fd() || frame.data().len() != 8 {
        return None;
    }
    let CanId::Standard(reply_id) = frame.id() else {
        return None;
    };
    if reply_id > CanId::STANDARD_MAX {
        return None;
    }
    let data: [u8; 8] = frame.data().try_into().ok()?;
    // The fixed enable/disable sentinels are never normal telemetry. Avoid
    // broad temperature/state plausibility filters: those hide real faults.
    if data == STOP || data == ENABLE || data == ZERO {
        return None;
    }
    Some(Feedback {
        source: data[0],
        reply_id,
        position_rad: unpack(u16::from_be_bytes([data[1], data[2]]), -12.57, 12.57, 16),
        velocity_rad_s: unpack(
            (u16::from(data[3]) << 4) | u16::from(data[4] >> 4),
            -50.0,
            50.0,
            12,
        ),
        torque_nm: unpack(
            (u16::from(data[4] & 0x0f) << 8) | u16::from(data[5]),
            -6.0,
            6.0,
            12,
        ),
        temperature_c: f64::from((u16::from(data[6] & 0x0f) << 8) | u16::from(data[7])) / 10.0,
        status: data[6] >> 6,
        fault: data[6] & 0x20 != 0,
        warning: data[6] & 0x10 != 0,
        payload: data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target(p: f64, v: f64, kp: f64, kd: f64, torque: f64) -> El05Target {
        El05Target::Mit {
            position_rad: p,
            velocity_rad_s: v,
            torque_nm: torque,
            kp,
            kd,
        }
    }

    #[test]
    fn command_vectors_use_standard_frames_and_twelve_bit_fields() {
        let neutral = target_frame(1, target(0.0, 0.0, 0.0, 0.0, 0.0)).unwrap();
        assert_eq!(neutral.id(), CanId::Standard(1));
        assert_eq!(neutral.kind(), FrameKind::Data);
        assert_eq!(neutral.data(), [0x7f, 0xff, 0x7f, 0xf0, 0, 0, 7, 0xff]);
        let speed = target_frame(1, target(0.0, 1.0, 0.0, 0.5, 0.0)).unwrap();
        assert_eq!(speed.data(), [0x7f, 0xff, 0x82, 0x80, 0, 0x19, 0x97, 0xff]);
        assert_eq!(stop_frame(1).unwrap().data(), STOP);
        assert_eq!(enable_frame(1).unwrap().data(), ENABLE);
        assert_eq!(zero_frame(1).unwrap().data(), ZERO);
        assert_eq!(stop_frame(255).unwrap().id(), CanId::Standard(255));
    }

    #[test]
    fn full_span_endpoints_and_invalid_targets_are_checked() {
        let minimum = target_frame(0, target(-12.57, -50.0, 0.0, 0.0, -6.0)).unwrap();
        let maximum = target_frame(255, target(12.57, 50.0, 500.0, 5.0, 6.0)).unwrap();
        assert_eq!(minimum.data(), [0; 8]);
        assert_eq!(maximum.data(), [0xff; 8]);
        for invalid in [
            target(f64::NAN, 0.0, 0.0, 0.0, 0.0),
            target(0.0, f64::INFINITY, 0.0, 0.0, 0.0),
            target(0.0, 50.01, 0.0, 0.0, 0.0),
            target(0.0, 0.0, -0.1, 0.0, 0.0),
            target(0.0, 0.0, 0.0, 5.01, 0.0),
            target(0.0, 0.0, 0.0, 0.0, 6.01),
            El05Target::Current { current_a: 0.0 },
        ] {
            assert!(target_frame(1, invalid).is_err());
        }
    }

    #[test]
    fn captured_compatible_motor_reply_preserves_raw_state_and_reply_address() {
        let payload = [0x01, 0x62, 0x21, 0x80, 0x08, 0x00, 0x01, 0x33];
        let decoded = decode_feedback(&frame(1, payload).unwrap()).unwrap();
        assert_eq!(
            (decoded.source, decoded.reply_id, decoded.status),
            (1, 1, 0)
        );
        assert!((decoded.position_rad - (-2.933_287_708_857_861_3)).abs() < 1e-12);
        assert!((decoded.velocity_rad_s - 0.012_210_012_210_012_167).abs() < 1e-12);
        assert!((decoded.torque_nm - 0.001_465_201_465_201_46).abs() < 1e-12);
        assert_eq!(decoded.temperature_c, 30.7);
        assert!(!decoded.fault && !decoded.warning);
        assert_eq!(decoded.payload, payload);
        let alternate_reply = CanFrame::new_data(CanId::Standard(0x7ff), &payload).unwrap();
        assert_eq!(decode_feedback(&alternate_reply).unwrap().reply_id, 0x7ff);
    }

    #[test]
    fn fault_warning_and_high_temperature_are_not_filtered_out() {
        let decoded =
            decode_feedback(&frame(1, [1, 0x80, 0, 0x80, 8, 0, 0xbf, 0xff]).unwrap()).unwrap();
        assert_eq!(decoded.status, 2);
        assert!(decoded.fault && decoded.warning);
        assert_eq!(decoded.temperature_c, 409.5);
    }

    #[test]
    fn unrelated_frame_formats_and_control_sentinels_are_rejected() {
        let payload = [1, 0x62, 0x21, 0x80, 8, 0, 1, 0x33];
        for invalid in [
            CanFrame::new_data(CanId::Extended(1), &payload).unwrap(),
            CanFrame::new_data(CanId::Standard(0x800), &payload).unwrap(),
            CanFrame::new_fd(CanId::Standard(1), &payload, false).unwrap(),
            CanFrame::new_remote(CanId::Standard(1), 8).unwrap(),
            CanFrame::new_data(CanId::Standard(1), &payload[..6]).unwrap(),
            stop_frame(1).unwrap(),
            enable_frame(1).unwrap(),
            zero_frame(1).unwrap(),
        ] {
            assert!(decode_feedback(&invalid).is_none());
        }
    }
}
