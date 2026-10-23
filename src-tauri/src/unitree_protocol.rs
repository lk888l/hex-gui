//! J288/S288 wire protocol, independently implemented from the supplied manual
//! and digital_servo 1.0.1's packed C structures. In particular, voltage is ONE
//! unsigned byte; specs/protocol.md and the Python feedback example have typos.
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

pub const BAUD_RATE: u32 = 6_000_000;
pub const RATIO: f64 = 70070.0 / 243.0;
pub const FEEDBACK_LEN: usize = 26;

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq)]
pub struct ServoTarget {
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub torque_nm: f64,
    pub kp: f64,
    pub kd: f64,
}

impl ServoTarget {
    pub fn validate(&self) -> Result<()> {
        for (name, value, min, max) in [
            (
                "Position (rad)",
                self.position_rad,
                -1428.018898,
                1428.018898,
            ),
            (
                "Velocity (rad/s)",
                self.velocity_rad_s,
                -278.90143,
                278.90143,
            ),
            ("Torque (Nm)", self.torque_nm, -36.9093, 36.908174),
            ("Kp", self.kp, 0.0, 2128.523254),
            ("Kd", self.kd, 0.0, 21.285233),
        ] {
            ensure!(
                value.is_finite() && value >= min && value <= max,
                "{name} must be finite and between {min} and {max}"
            );
        }
        Ok(())
    }
}

pub fn validate_id(id: u8) -> Result<()> {
    ensure!(
        id <= 14,
        "Servo ID must be 0–14; 15 is broadcast and cannot be selected"
    );
    Ok(())
}

/// STM32-style CRC: consume little-endian u32 words, MSB first, no final XOR.
/// This is deliberately not the common reflected Ethernet/zlib CRC32.
pub fn crc32(data: &[u8]) -> u32 {
    assert_eq!(data.len() % 4, 0);
    let mut crc = u32::MAX;
    for word in data.chunks_exact(4) {
        crc ^= u32::from_le_bytes(word.try_into().unwrap());
        for _ in 0..32 {
            crc = (crc << 1)
                ^ if crc & 0x8000_0000 != 0 {
                    0x04c1_1db7
                } else {
                    0
                };
        }
    }
    crc
}

pub fn control_packet(id: u8, mode: u8, timeout: bool, target: ServoTarget) -> Result<[u8; 20]> {
    validate_id(id)?;
    ensure!(
        mode <= 1,
        "Only lock (0) and hybrid FOC (1) modes are supported"
    );
    target.validate()?;
    let mut packet = [0u8; 20];
    packet[..4].copy_from_slice(&[0xfe, 0xee, id | (mode << 4) | (u8::from(timeout) << 7), 0]);
    let scaled_i16 = |value: f64| value.round().clamp(i16::MIN as f64, i16::MAX as f64) as i16;
    packet[4..6].copy_from_slice(&scaled_i16(target.torque_nm / RATIO * 256000.0).to_le_bytes());
    packet[6..8]
        .copy_from_slice(&scaled_i16(target.velocity_rad_s * RATIO * 2.56 / TAU).to_le_bytes());
    packet[8..12].copy_from_slice(
        &((target.position_rad * RATIO * 32768.0 / TAU).round() as i32).to_le_bytes(),
    );
    packet[12..14]
        .copy_from_slice(&scaled_i16(target.kp / RATIO.powi(2) * 1280000.0).to_le_bytes());
    packet[14..16]
        .copy_from_slice(&scaled_i16(target.kd / RATIO.powi(2) * 128000000.0).to_le_bytes());
    let crc = crc32(&packet[..16]);
    packet[16..20].copy_from_slice(&crc.to_le_bytes());
    Ok(packet)
}

#[derive(Clone, Debug, Serialize)]
pub struct ServoFeedback {
    pub id: u8,
    pub mode: u8,
    pub timed_out: bool,
    pub shell_temperature_c: i8,
    pub winding_temperature_c: u8,
    pub voltage_v: f64,
    pub torque_nm: f64,
    pub velocity_rad_s: f64,
    pub position_rad: f64,
    pub output_angle_rad: f64,
    pub error_bits: u32,
    pub warning_bits: u8,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

pub fn parse_feedback(packet: &[u8]) -> Result<ServoFeedback> {
    ensure!(
        packet.len() == FEEDBACK_LEN && packet[..2] == [0xfc, 0xee],
        "Invalid servo feedback header/length"
    );
    ensure!(
        crc32(&packet[2..22]) == u32::from_le_bytes(packet[22..26].try_into()?),
        "Servo feedback CRC mismatch"
    );
    let id = packet[2] & 15;
    validate_id(id)?;
    let mode = (packet[2] >> 4) & 7;
    let signed16 = |offset| i16::from_le_bytes([packet[offset], packet[offset + 1]]) as f64;
    let error_bits = u32::from_le_bytes(packet[14..18].try_into()?);
    let out = u16::from_le_bytes(packet[18..20].try_into()?);
    let warning_bits = (out >> 13) as u8;
    Ok(ServoFeedback {
        id,
        mode,
        timed_out: packet[2] & 0x80 != 0,
        shell_temperature_c: packet[3] as i8,
        winding_temperature_c: packet[4],
        voltage_v: packet[5] as f64 / 2.0,
        torque_nm: signed16(6) / 256000.0 * RATIO,
        velocity_rad_s: signed16(8) / 2.56 * TAU / RATIO,
        position_rad: i32::from_le_bytes(packet[10..14].try_into()?) as f64 * TAU / 32768.0 / RATIO,
        output_angle_rad: (out & 0x1fff) as f64 * TAU / 8192.0,
        error_bits,
        warning_bits,
        errors: describe_bits(error_bits, &ERROR_NAMES),
        warnings: describe_bits(
            warning_bits as u32,
            &["Low-voltage supply abnormal / 低压电源异常"],
        ),
    })
}

const ERROR_NAMES: [&str; 22] = [
    "Overcurrent / 过流",
    "Transient overvoltage / 瞬态过压",
    "Persistent overvoltage / 持续过压",
    "Undervoltage / 欠压",
    "Chip overtemperature / 芯片过热",
    "MOS hot/cold / MOS 过热或过冷",
    "MOS temperature sensor / MOS 温度异常",
    "Shell hot/cold / 壳体过热或过冷",
    "Shell temperature sensor / 壳体温度异常",
    "Winding overtemperature / 绕组过热",
    "Rotor encoder 1 / 转子编码器 1 错误",
    "Rotor encoder 2 / 转子编码器 2 错误",
    "Output encoder / 输出编码器错误",
    "Stored data / 存储数据错误",
    "Abnormal reset / 异常复位",
    "Waiting for unlock / 等待解锁",
    "Driver verification / 驱动验证错误",
    "Calibration mode / 标定模式",
    "Communication CRC / 通信校验错误",
    "Driver version too old / 驱动版本过低",
    "Wrong firmware model / 固件型号错误",
    "Encoder angle abnormal / 编码器角度异常",
];

fn describe_bits(bits: u32, names: &[&str]) -> Vec<String> {
    (0..32)
        .filter(|bit| bits & (1 << bit) != 0)
        .map(|bit| {
            names
                .get(bit)
                .map(|s| s.to_string())
                .unwrap_or_else(|| format!("Unknown bit {bit} / 未知位 {bit}"))
        })
        .collect()
}

/// Keep incomplete packets across reads, skip TX echo/noise, and recover after
/// corrupt packets instead of assuming every USB read returns one whole frame.
#[derive(Default)]
pub struct FeedbackDecoder {
    buffer: Vec<u8>,
    pub crc_errors: u64,
}

impl FeedbackDecoder {
    pub fn push(&mut self, bytes: &[u8]) {
        self.buffer.extend_from_slice(bytes);
        if self.buffer.len() > 4096 {
            self.buffer.drain(..self.buffer.len() - 4096);
        }
    }

    pub fn next(&mut self) -> Option<(ServoFeedback, Vec<u8>)> {
        loop {
            let Some(start) = self.buffer.windows(2).position(|w| w == [0xfc, 0xee]) else {
                // Preserve a split header, including a single 0xFC byte.
                let keep = self.buffer.last() == Some(&0xfc);
                self.buffer.clear();
                if keep {
                    self.buffer.push(0xfc);
                }
                return None;
            };
            self.buffer.drain(..start);
            if self.buffer.len() < FEEDBACK_LEN {
                return None;
            }
            match parse_feedback(&self.buffer[..FEEDBACK_LEN]) {
                Ok(feedback) => {
                    let raw = self.buffer.drain(..FEEDBACK_LEN).collect();
                    return Some((feedback, raw));
                }
                Err(_) => {
                    self.crc_errors += 1;
                    self.buffer.remove(0);
                }
            }
        }
    }
}

pub fn hex_bytes(data: &[u8]) -> String {
    data.iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    // CRC/control vectors generated independently with the provided official
    // Python SDK; feedback layout taken from its packed C header and manual.
    const FEEDBACK: [u8; 26] = [
        0xfc, 0xee, 0x93, 0xf6, 0xdc, 0xf0, 0x00, 0xff, 0x80, 0x00, 0x00, 0x80, 0xff, 0xff, 0x08,
        0x00, 0x10, 0x00, 0x23, 0xa1, 0x12, 0x34, 0x3b, 0x02, 0x82, 0x36,
    ];

    #[test]
    fn official_control_and_crc_vectors() {
        assert_eq!(
            hex_bytes(&control_packet(0, 0, true, ServoTarget::default()).unwrap()),
            "FE EE 80 00 00 00 00 00 00 00 00 00 00 00 00 00 1D 99 FB 6E"
        );
        let target = ServoTarget {
            position_rad: 0.25,
            velocity_rad_s: -0.5,
            torque_nm: 0.1,
            kp: 2.0,
            kd: 0.2,
        };
        assert_eq!(
            hex_bytes(&control_packet(3, 1, true, target).unwrap()),
            "FE EE 93 00 59 00 C5 FF 93 BC 05 00 1F 00 34 01 61 CE C3 B9"
        );
        let negative = control_packet(
            3,
            1,
            true,
            ServoTarget {
                position_rad: -0.25,
                ..target
            },
        )
        .unwrap();
        assert_eq!(
            i32::from_le_bytes(negative[8..12].try_into().unwrap()),
            -375955
        );
    }

    #[test]
    fn feedback_unsigned_voltage_and_winding_signed_position_and_flags() {
        let f = parse_feedback(&FEEDBACK).unwrap();
        assert_eq!((f.id, f.mode, f.timed_out), (3, 1, true));
        assert_eq!((f.shell_temperature_c, f.winding_temperature_c), (-10, 220));
        assert_eq!(f.voltage_v, 120.0);
        assert!((f.position_rad + TAU / RATIO).abs() < 1e-12);
        assert!((f.torque_nm + 0.001 * RATIO).abs() < 1e-12);
        assert_eq!(f.warning_bits, 5);
        assert_eq!(f.errors.len(), 2);
        assert_eq!(f.warnings.len(), 2);
    }

    #[test]
    fn decoder_recovers_after_echo_fragmentation_noise_and_bad_crc() {
        let mut d = FeedbackDecoder::default();
        d.push(&[0xfe, 0xee, 0x80, 0, 0xfc]);
        assert!(d.next().is_none());
        d.push(&FEEDBACK[1..8]);
        assert!(d.next().is_none());
        d.push(&FEEDBACK[8..]);
        assert_eq!(d.next().unwrap().0.id, 3);
        let mut corrupt = FEEDBACK;
        corrupt[12] ^= 1;
        d.push(&corrupt);
        d.push(&FEEDBACK);
        d.push(&FEEDBACK);
        assert!(d.next().is_some());
        assert!(d.next().is_some());
        assert!(d.next().is_none());
        assert_eq!(d.crc_errors, 1);
    }

    #[test]
    fn invalid_targets_modes_and_broadcast_are_rejected() {
        for value in [f64::NAN, f64::INFINITY, -f64::INFINITY, 1429.0] {
            assert!(control_packet(
                0,
                1,
                true,
                ServoTarget {
                    position_rad: value,
                    ..ServoTarget::default()
                }
            )
            .is_err());
        }
        assert!(control_packet(15, 0, true, ServoTarget::default()).is_err());
        assert!(control_packet(0, 2, true, ServoTarget::default()).is_err());
        assert!(control_packet(
            0,
            1,
            true,
            ServoTarget {
                kp: -1.0,
                ..ServoTarget::default()
            }
        )
        .is_err());
        let mut bad = FEEDBACK;
        bad[0] = 0xfe;
        assert!(parse_feedback(&bad).is_err());
    }
}
