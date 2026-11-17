//! Pure decoder for the `gello-joint` CAN 2.0 / CANopen contract.

use std::f64::consts::TAU;

use can_transport::{CanFrame, CanId, FrameKind};
use thiserror::Error;

/// Node ID used when the firmware's persisted `0x2101:00` value is invalid.
pub const DEFAULT_NODE_ID: u8 = 0x10;
/// Current firmware uses Classic CAN at 1 Mbps, without FD or BRS.
pub const CAN_BITRATE: u32 = 1_000_000;
/// `GELLO_JOINT_V1_PRODUCT_CODE` from the firmware identity object.
pub const EXPECTED_PRODUCT_CODE: u32 = 0x0000_1145;
/// AS5600 resolution and the firmware's `0x3000:01 + 1` default.
pub const COUNTS_PER_TURN: u32 = 4096;
/// Persistent firmware zero offset (writable `u32`).
pub const ZERO_POSITION_INDEX: u16 = 0x3000;
pub const ZERO_POSITION_SUBINDEX: u8 = 0x02;
/// Unadjusted AS5600 position used when capturing the current physical zero.
pub const RAW_POSITION_INDEX: u16 = 0x3001;
pub const RAW_POSITION_SUBINDEX: u8 = 0x01;
pub const TPDO1_BASE: u16 = 0x180;
pub const TPDO2_BASE: u16 = 0x280;
pub const HEARTBEAT_BASE: u16 = 0x700;

const POSITION_PAYLOAD_LEN: usize = 8;
const STATUS_PAYLOAD_LEN: usize = 2;
const HEARTBEAT_PAYLOAD_LEN: usize = 1;

/// Validated parameters needed to decode one encoder node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GelloJointProtocol {
    node_id: u8,
    counts_per_turn: u32,
    allow_can_fd: bool,
}

impl GelloJointProtocol {
    pub fn new(node_id: u8, counts_per_turn: u32) -> Result<Self, ProtocolConfigError> {
        if !(1..=127).contains(&node_id) {
            return Err(ProtocolConfigError::InvalidNodeId(node_id));
        }
        if counts_per_turn == 0 {
            return Err(ProtocolConfigError::ZeroCountsPerTurn);
        }
        Ok(Self {
            node_id,
            counts_per_turn,
            allow_can_fd: false,
        })
    }

    /// Enable decoding of legacy FD firmware on an explicitly selected FD bus.
    /// Classic frames remain valid for its heartbeat and short TPDOs.
    pub const fn with_can_fd(mut self, enabled: bool) -> Self {
        self.allow_can_fd = enabled;
        self
    }

    pub const fn node_id(self) -> u8 {
        self.node_id
    }

    pub const fn counts_per_turn(self) -> u32 {
        self.counts_per_turn
    }

    pub const fn position_cob_id(self) -> u16 {
        TPDO1_BASE + self.node_id as u16
    }

    pub const fn status_cob_id(self) -> u16 {
        TPDO2_BASE + self.node_id as u16
    }

    pub const fn heartbeat_cob_id(self) -> u16 {
        HEARTBEAT_BASE + self.node_id as u16
    }

    /// Decode TPDO1: adjusted position followed by CANopen `0x1013:00`.
    pub fn decode_position(self, frame: &CanFrame) -> Result<PositionSample, DecodeError> {
        self.validate_frame(frame, self.position_cob_id(), POSITION_PAYLOAD_LEN)?;
        let data = frame.data();
        let position_counts = u32::from_le_bytes(data[0..4].try_into().unwrap());
        if position_counts >= self.counts_per_turn {
            return Err(DecodeError::PositionOutOfRange {
                value: position_counts,
                counts_per_turn: self.counts_per_turn,
            });
        }
        let turn_fraction = f64::from(position_counts) / f64::from(self.counts_per_turn);
        Ok(PositionSample {
            position_counts,
            angle_degrees: turn_fraction * 360.0,
            angle_radians: turn_fraction * TAU,
            device_timestamp_raw: u32::from_le_bytes(data[4..8].try_into().unwrap()),
            host_timestamp_us: frame.timestamp_us(),
            format: FrameFormat::from(frame.kind()),
        })
    }

    /// Decode TPDO2: the firmware's `0x3001:03` sensor status.
    pub fn decode_status(self, frame: &CanFrame) -> Result<StatusSample, DecodeError> {
        self.validate_frame(frame, self.status_cob_id(), STATUS_PAYLOAD_LEN)?;
        let raw = u16::from_le_bytes(frame.data().try_into().unwrap());
        Ok(StatusSample {
            raw,
            status: SensorStatus::from_raw(raw),
            host_timestamp_us: frame.timestamp_us(),
            format: FrameFormat::from(frame.kind()),
        })
    }

    /// Decode the CiA 301 heartbeat for this exact node.
    pub fn decode_heartbeat(self, frame: &CanFrame) -> Result<HeartbeatSample, DecodeError> {
        self.validate_frame(frame, self.heartbeat_cob_id(), HEARTBEAT_PAYLOAD_LEN)?;
        let raw = frame.data()[0];
        Ok(HeartbeatSample {
            raw,
            state: NmtState::from_heartbeat_byte(raw),
            host_timestamp_us: frame.timestamp_us(),
        })
    }

    /// Targeted CiA 301 NMT Start Remote Node command.
    pub fn start_remote_node_frame(self) -> Result<CanFrame, can_transport::CanIoError> {
        CanFrame::new_data(0u16, &[0x01, self.node_id])
    }

    fn validate_frame(
        self,
        frame: &CanFrame,
        expected_id: u16,
        expected_len: usize,
    ) -> Result<(), DecodeError> {
        validate_frame(frame, expected_id, expected_len)?;
        if frame.is_fd() && !self.allow_can_fd {
            return Err(DecodeError::UnexpectedCanFd);
        }
        Ok(())
    }
}

impl Default for GelloJointProtocol {
    fn default() -> Self {
        Self {
            node_id: DEFAULT_NODE_ID,
            counts_per_turn: COUNTS_PER_TURN,
            allow_can_fd: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameFormat {
    pub is_can_fd: bool,
    pub bitrate_switch: bool,
}

impl From<FrameKind> for FrameFormat {
    fn from(kind: FrameKind) -> Self {
        match kind {
            FrameKind::Fd { brs } => Self {
                is_can_fd: true,
                bitrate_switch: brs,
            },
            FrameKind::Data | FrameKind::Remote => Self {
                is_can_fd: false,
                bitrate_switch: false,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionSample {
    pub position_counts: u32,
    pub angle_degrees: f64,
    pub angle_radians: f64,
    /// Raw value of CANopen high-resolution timestamp object `0x1013:00`.
    pub device_timestamp_raw: u32,
    /// Optional receive timestamp supplied by the host adapter.
    pub host_timestamp_us: Option<u64>,
    pub format: FrameFormat,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusSample {
    pub raw: u16,
    pub status: SensorStatus,
    pub host_timestamp_us: Option<u64>,
    pub format: FrameFormat,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HeartbeatSample {
    pub raw: u8,
    pub state: NmtState,
    pub host_timestamp_us: Option<u64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NmtState {
    BootUp,
    Stopped,
    Operational,
    PreOperational,
    Unknown(u8),
}

impl NmtState {
    pub const fn from_heartbeat_byte(raw: u8) -> Self {
        match raw {
            0x00 => Self::BootUp,
            0x04 => Self::Stopped,
            0x05 => Self::Operational,
            0x7F => Self::PreOperational,
            other => Self::Unknown(other),
        }
    }

    pub const fn needs_start_command(self) -> bool {
        matches!(self, Self::BootUp | Self::Stopped | Self::PreOperational)
    }
}

/// Values written by the firmware to object `0x3001:03`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorStatus {
    Ok,
    MagnetTooWeak,
    MagnetTooStrong,
    MagnetNotDetected,
    SensorIoError,
    Unknown(u16),
}

impl SensorStatus {
    pub const fn from_raw(raw: u16) -> Self {
        match raw {
            0 => Self::Ok,
            1 => Self::MagnetTooWeak,
            2 => Self::MagnetTooStrong,
            3 => Self::MagnetNotDetected,
            4 => Self::SensorIoError,
            other => Self::Unknown(other),
        }
    }

    /// Weak/strong field readings are degraded but the firmware still updates
    /// the angle. Missing-magnet and I/O-error values retain an old position.
    pub const fn angle_is_usable(self) -> bool {
        matches!(self, Self::Ok | Self::MagnetTooWeak | Self::MagnetTooStrong)
    }
}

#[derive(Debug, Error, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolConfigError {
    #[error("CANopen node ID must be in 1..=127, got {0}")]
    InvalidNodeId(u8),
    #[error("counts_per_turn must be greater than zero")]
    ZeroCountsPerTurn,
}

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum DecodeError {
    #[error("expected standard CAN ID 0x{expected:03X}, got {actual}")]
    WrongId { expected: u16, actual: String },
    #[error("remote frames cannot carry encoder telemetry")]
    RemoteFrame,
    #[error("CAN 2.0 encoder requires Classic CAN data frames; received CAN FD")]
    UnexpectedCanFd,
    #[error("CAN ID 0x{cob_id:03X} payload must be exactly {expected} bytes, got {actual}")]
    WrongLength {
        cob_id: u16,
        expected: usize,
        actual: usize,
    },
    #[error("encoder position {value} is outside a {counts_per_turn}-count turn")]
    PositionOutOfRange { value: u32, counts_per_turn: u32 },
}

fn validate_frame(
    frame: &CanFrame,
    expected_id: u16,
    expected_len: usize,
) -> Result<(), DecodeError> {
    match frame.id() {
        CanId::Standard(actual) if actual == expected_id => {}
        CanId::Standard(actual) => {
            return Err(DecodeError::WrongId {
                expected: expected_id,
                actual: format!("standard 0x{actual:03X}"),
            });
        }
        CanId::Extended(actual) => {
            return Err(DecodeError::WrongId {
                expected: expected_id,
                actual: format!("extended 0x{actual:08X}"),
            });
        }
    }
    if frame.is_remote() {
        return Err(DecodeError::RemoteFrame);
    }
    if frame.data().len() != expected_len {
        return Err(DecodeError::WrongLength {
            cob_id: expected_id,
            expected: expected_len,
            actual: frame.data().len(),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_cob_ids_follow_canopen_tpdo_allocation() {
        let protocol = GelloJointProtocol::default();
        assert_eq!(protocol.position_cob_id(), 0x190);
        assert_eq!(protocol.status_cob_id(), 0x290);
        assert_eq!(protocol.heartbeat_cob_id(), 0x710);
    }

    #[test]
    fn heartbeat_and_targeted_nmt_start_follow_cia301() {
        let protocol = GelloJointProtocol::default();
        let heartbeat = CanFrame::new_data(protocol.heartbeat_cob_id(), &[0x7F]).unwrap();
        assert_eq!(
            protocol.decode_heartbeat(&heartbeat).unwrap().state,
            NmtState::PreOperational
        );
        assert!(NmtState::PreOperational.needs_start_command());
        assert!(!NmtState::Operational.needs_start_command());

        let command = protocol.start_remote_node_frame().unwrap();
        assert_eq!(command.id(), CanId::Standard(0));
        assert_eq!(command.data(), &[0x01, DEFAULT_NODE_ID]);
        assert_eq!(command.kind(), FrameKind::Data);
    }

    #[test]
    fn position_tpdo_decodes_little_endian_counts_and_angles() {
        let protocol = GelloJointProtocol::default();
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&1024u32.to_le_bytes());
        payload[4..8].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        let frame = CanFrame::new_data(protocol.position_cob_id(), &payload)
            .unwrap()
            .with_timestamp_us(77);
        let sample = protocol.decode_position(&frame).unwrap();
        assert_eq!(sample.position_counts, 1024);
        assert_eq!(sample.angle_degrees, 90.0);
        assert!((sample.angle_radians - std::f64::consts::FRAC_PI_2).abs() < f64::EPSILON);
        assert_eq!(sample.device_timestamp_raw, 0x1234_5678);
        assert_eq!(sample.host_timestamp_us, Some(77));
        assert_eq!(
            sample.format,
            FrameFormat {
                is_can_fd: false,
                bitrate_switch: false
            }
        );
    }

    #[test]
    fn fd_telemetry_requires_explicit_legacy_mode() {
        let classic = GelloJointProtocol::default();
        let legacy = classic.with_can_fd(true);
        for brs in [false, true] {
            let position = CanFrame::new_fd(classic.position_cob_id(), &[0; 8], brs).unwrap();
            let status = CanFrame::new_fd(classic.status_cob_id(), &[0; 2], brs).unwrap();
            let heartbeat = CanFrame::new_fd(classic.heartbeat_cob_id(), &[5], brs).unwrap();
            assert_eq!(
                classic.decode_position(&position),
                Err(DecodeError::UnexpectedCanFd)
            );
            assert_eq!(
                classic.decode_status(&status),
                Err(DecodeError::UnexpectedCanFd)
            );
            assert_eq!(
                classic.decode_heartbeat(&heartbeat),
                Err(DecodeError::UnexpectedCanFd)
            );
            let sample = legacy.decode_position(&position).unwrap();
            assert!(sample.format.is_can_fd);
            assert_eq!(sample.format.bitrate_switch, brs);
            assert_eq!(
                legacy.decode_status(&status).unwrap().status,
                SensorStatus::Ok
            );
            assert_eq!(
                legacy.decode_heartbeat(&heartbeat).unwrap().state,
                NmtState::Operational
            );
        }
        let status = CanFrame::new_data(classic.status_cob_id(), &[0; 2]).unwrap();
        assert!(legacy.decode_status(&status).is_ok());
        let position = CanFrame::new_fd(classic.position_cob_id(), &[0; 8], true).unwrap();
        assert_eq!(
            legacy.with_can_fd(false).decode_position(&position),
            Err(DecodeError::UnexpectedCanFd)
        );
    }

    #[test]
    fn status_tpdo_preserves_known_and_unknown_codes() {
        let protocol = GelloJointProtocol::default();
        for (raw, expected) in [
            (0u16, SensorStatus::Ok),
            (1u16, SensorStatus::MagnetTooWeak),
            (2u16, SensorStatus::MagnetTooStrong),
            (3u16, SensorStatus::MagnetNotDetected),
            (4u16, SensorStatus::SensorIoError),
            (99u16, SensorStatus::Unknown(99)),
        ] {
            let frame = CanFrame::new_data(protocol.status_cob_id(), &raw.to_le_bytes()).unwrap();
            assert_eq!(protocol.decode_status(&frame).unwrap().status, expected);
        }
    }

    #[test]
    fn decoder_rejects_lookalike_or_malformed_frames() {
        let protocol = GelloJointProtocol::default();
        let wrong_id = CanFrame::new_data(0x191u16, &[0; 8]).unwrap();
        assert!(matches!(
            protocol.decode_position(&wrong_id),
            Err(DecodeError::WrongId { .. })
        ));
        // Classic CAN 2.0B announcements and MHF replies share the wire but
        // must never be mistaken for this node's standard TPDO1.
        for id in [
            CanId::Extended(u32::from(protocol.position_cob_id())),
            CanId::Extended(0xA9),
            CanId::Extended(0xAB),
            CanId::Standard(0x141),
        ] {
            let frame = CanFrame::new_data(id, &[0; 8]).unwrap();
            assert!(matches!(
                protocol.decode_position(&frame),
                Err(DecodeError::WrongId { .. })
            ));
        }
        let remote = CanFrame::new_remote(protocol.position_cob_id(), 8).unwrap();
        assert_eq!(
            protocol.decode_position(&remote),
            Err(DecodeError::RemoteFrame)
        );
        let short = CanFrame::new_data(protocol.position_cob_id(), &[0; 7]).unwrap();
        assert!(matches!(
            protocol.decode_position(&short),
            Err(DecodeError::WrongLength { actual: 7, .. })
        ));
        let out_of_range = CanFrame::new_data(
            protocol.position_cob_id(),
            &[0x00, 0x10, 0x00, 0x00, 0, 0, 0, 0],
        )
        .unwrap();
        assert!(matches!(
            protocol.decode_position(&out_of_range),
            Err(DecodeError::PositionOutOfRange { value: 4096, .. })
        ));
    }

    #[test]
    fn config_rejects_reserved_node_ids_and_zero_scale() {
        assert_eq!(
            GelloJointProtocol::new(0, COUNTS_PER_TURN),
            Err(ProtocolConfigError::InvalidNodeId(0))
        );
        assert_eq!(
            GelloJointProtocol::new(128, COUNTS_PER_TURN),
            Err(ProtocolConfigError::InvalidNodeId(128))
        );
        assert_eq!(
            GelloJointProtocol::new(DEFAULT_NODE_ID, 0),
            Err(ProtocolConfigError::ZeroCountsPerTurn)
        );
    }
}
