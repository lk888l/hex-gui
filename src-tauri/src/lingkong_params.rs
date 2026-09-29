//! Parameter layouts from Lingkong CAN protocol V2.36, sections 18/19/26–28.
//! Values stay in the protocol's integer units so reads and writes round-trip.

use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct ParameterValue {
    pub key: String,
    pub values: Vec<u32>,
}

#[derive(Clone, Copy)]
enum Layout {
    ControlPid(u8),
    ControlWord(u8),
    ControlLong(u8),
    SettingByte(u8),
    SettingWord(u8),
    SettingLong(u8),
    SettingPid(u8),
}

#[derive(Clone, Copy)]
pub struct ParameterSpec {
    pub key: &'static str,
    layout: Layout,
    max: u32,
}

impl ParameterSpec {
    pub fn is_setting(self) -> bool {
        matches!(
            self.layout,
            Layout::SettingByte(..)
                | Layout::SettingWord(..)
                | Layout::SettingLong(..)
                | Layout::SettingPid(..)
        )
    }

    pub fn read_frame(self) -> [u8; 8] {
        let mut frame = [0u8; 8];
        match self.layout {
            Layout::ControlPid(id) | Layout::ControlWord(id) | Layout::ControlLong(id) => {
                frame[0] = 0xC0;
                frame[1] = id;
            }
            Layout::SettingByte(id) | Layout::SettingWord(id) | Layout::SettingLong(id) => {
                frame[0] = 0x40;
                frame[1] = 0x05;
                frame[2] = id;
            }
            Layout::SettingPid(id) => {
                frame[0] = 0x40;
                frame[1] = id;
            }
        }
        frame
    }

    pub fn write_frame(self, values: &[u32]) -> Result<[u8; 8], String> {
        let count = if matches!(self.layout, Layout::ControlPid(_) | Layout::SettingPid(_)) {
            3
        } else {
            1
        };
        if values.len() != count || values.iter().any(|value| *value > self.max) {
            return Err(format!(
                "{} expects {count} integer value(s) in 0..={}",
                self.key, self.max
            ));
        }
        let mut frame = self.read_frame();
        frame[0] = if self.is_setting() { 0x42 } else { 0xC1 };
        match self.layout {
            Layout::ControlPid(_) | Layout::SettingPid(_) => {
                for (index, value) in values.iter().enumerate() {
                    frame[2 + index * 2..4 + index * 2]
                        .copy_from_slice(&(*value as u16).to_le_bytes());
                }
            }
            Layout::ControlWord(_) | Layout::SettingWord(_) => {
                frame[4..6].copy_from_slice(&(values[0] as u16).to_le_bytes());
            }
            Layout::ControlLong(_) | Layout::SettingLong(_) => {
                frame[4..8].copy_from_slice(&values[0].to_le_bytes());
            }
            Layout::SettingByte(..) => frame[4] = values[0] as u8,
        }
        Ok(frame)
    }

    pub fn decode(self, frame: [u8; 8], write: bool) -> Result<ParameterValue, String> {
        let expected = if write {
            self.write_frame_opcode()
        } else {
            self.read_frame()[0]
        };
        if frame[0] != expected || frame[1] != self.read_frame()[1] {
            return Err(format!(
                "{} returned the wrong parameter selector",
                self.key
            ));
        }
        if matches!(
            self.layout,
            Layout::SettingByte(..) | Layout::SettingWord(..) | Layout::SettingLong(..)
        ) && frame[2] != self.read_frame()[2]
        {
            return Err(format!("{} returned the wrong setting address", self.key));
        }
        let values = match self.layout {
            Layout::ControlPid(_) | Layout::SettingPid(_) => vec![
                u16::from_le_bytes([frame[2], frame[3]]) as u32,
                u16::from_le_bytes([frame[4], frame[5]]) as u32,
                u16::from_le_bytes([frame[6], frame[7]]) as u32,
            ],
            Layout::ControlWord(_) | Layout::SettingWord(_) => {
                vec![u16::from_le_bytes([frame[4], frame[5]]) as u32]
            }
            Layout::ControlLong(_) | Layout::SettingLong(_) => {
                vec![u32::from_le_bytes([frame[4], frame[5], frame[6], frame[7]])]
            }
            Layout::SettingByte(..) => vec![frame[4] as u32],
        };
        if values.iter().any(|value| *value > self.max) {
            return Err(format!(
                "{} returned a value outside the V2.36 range",
                self.key
            ));
        }
        Ok(ParameterValue {
            key: self.key.into(),
            values,
        })
    }

    fn write_frame_opcode(self) -> u8 {
        if self.is_setting() {
            0x42
        } else {
            0xC1
        }
    }
}

const fn spec(key: &'static str, layout: Layout, max: u32) -> ParameterSpec {
    ParameterSpec { key, layout, max }
}

pub const PARAMETERS: &[ParameterSpec] = &[
    spec("ram_position_pid", Layout::ControlPid(0x0A), 2000),
    spec("ram_speed_pid", Layout::ControlPid(0x0B), 2000),
    spec("ram_current_pid", Layout::ControlPid(0x0C), 2000),
    spec("ram_torque_limit", Layout::ControlWord(0x1E), 2000),
    spec("ram_speed_limit", Layout::ControlLong(0x20), 600_000),
    spec(
        "ram_angle_limit",
        Layout::ControlLong(0x22),
        i32::MAX as u32,
    ),
    spec("ram_current_ramp", Layout::ControlLong(0x24), 30_000),
    spec("ram_speed_ramp", Layout::ControlLong(0x26), 600_000),
    spec("setting_driver_id", Layout::SettingByte(0x0A), 32),
    spec("setting_bus_type", Layout::SettingByte(0x0B), 2),
    spec("setting_rs485_baudrate", Layout::SettingByte(0x0C), 10),
    spec("setting_can_baudrate", Layout::SettingByte(0x0D), 4),
    spec("setting_max_power", Layout::SettingWord(0xE0), 2000),
    spec("setting_max_speed", Layout::SettingLong(0xE2), 600_000),
    spec(
        "setting_max_angle",
        Layout::SettingLong(0xE4),
        i32::MAX as u32,
    ),
    spec("setting_current_ramp", Layout::SettingWord(0xEA), 30_000),
    spec("setting_speed_ramp", Layout::SettingLong(0xEC), 600_000),
    spec("setting_position_pid", Layout::SettingPid(0xA0), 2000),
    spec("setting_speed_pid", Layout::SettingPid(0xA4), 2000),
    spec("setting_current_pid", Layout::SettingPid(0xA8), 2000),
];

pub fn parameter(key: &str) -> Result<ParameterSpec, String> {
    PARAMETERS
        .iter()
        .find(|spec| spec.key == key)
        .copied()
        .ok_or_else(|| format!("unsupported Lingkong parameter: {key}"))
}
