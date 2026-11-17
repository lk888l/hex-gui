//! Host-side magnetic encoder protocol and runtime monitoring.
//!
//! This crate deliberately contains no GUI or Tauri types. Applications can
//! wrap [`EncoderSnapshot`] in their own DTOs without leaking UI concerns into
//! the wire protocol.

#![forbid(unsafe_code)]

mod monitor;
mod protocol;

pub use can_transport::CanControllerState;
pub use monitor::{
    BusHealth, CommunicationState, EncoderConfig, EncoderMonitor, EncoderSnapshot, MonitorError,
};
pub use protocol::{
    DecodeError, FrameFormat, GelloJointProtocol, HeartbeatSample, NmtState, PositionSample,
    ProtocolConfigError, SensorStatus, StatusSample, CAN_BITRATE, COUNTS_PER_TURN, DEFAULT_NODE_ID,
    EXPECTED_PRODUCT_CODE, HEARTBEAT_BASE, RAW_POSITION_INDEX, RAW_POSITION_SUBINDEX, TPDO1_BASE,
    TPDO2_BASE, ZERO_POSITION_INDEX, ZERO_POSITION_SUBINDEX,
};
