//! Verified WIT BLE 5.0 stream profiles and fixed register replies; also used on USB.
use crate::{DeviceCommand, Error, Result};
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub enum Frame {
    Motion {
        acceleration_g: [f64; 3],
        angular_velocity_dps: [f64; 3],
        euler_xyz_deg: [f64; 3],
    },
    Stream {
        profile: u8,
        acceleration_g: Option<[f64; 3]>,
        sample_time_ms: Option<u64>,
        angular_velocity_dps: Option<[f64; 3]>,
        euler_xyz_deg: Option<[f64; 3]>,
        quaternion_wxyz: Option<[f64; 4]>,
    },
    Registers {
        address: u16,
        values: [i16; 8],
    },
}

/// Decode the eight-byte RTC into milliseconds since 2000-01-01 in the device
/// clock. No timezone is known, so this value must never be labelled UTC.
pub fn device_calendar_ms(b: [u8; 8]) -> Option<u64> {
    let year = 2000 + u32::from(b[0]);
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = [
        31,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let month = usize::from(b[1]);
    let millis = u16::from_le_bytes([b[6], b[7]]);
    if !(1..=12).contains(&month)
        || b[2] == 0
        || u32::from(b[2]) > days_in_month[month - 1]
        || b[3] > 23
        || b[4] > 59
        || b[5] > 59
        || millis > 999
    {
        return None;
    }
    let leap_before = |y: u32| (y - 1) / 4 - (y - 1) / 100 + (y - 1) / 400;
    let days = (year - 2000) * 365 + leap_before(year) - leap_before(2000)
        + days_in_month[..month - 1].iter().sum::<u32>()
        + u32::from(b[2])
        - 1;
    Some(
        (((u64::from(days) * 24 + u64::from(b[3])) * 60 + u64::from(b[4])) * 60 + u64::from(b[5]))
            * 1000
            + u64::from(millis),
    )
}

fn frame_length(flag: u8) -> Option<usize> {
    match flag {
        0x61 | 0x71 => Some(20),
        0x01 => Some(8),
        0x04 => Some(10),
        0x81 => Some(16),
        0x84 => Some(18),
        0xa4 => Some(24),
        0xe4 => Some(30),
        _ => None,
    }
}

fn decode(b: &[u8]) -> Option<Frame> {
    let word = |i| i16::from_le_bytes([b[i], b[i + 1]]) as f64 / 32768.0;
    let triple = |offset, scale| std::array::from_fn(|i| word(offset + 2 * i) * scale);
    if b[1] == 0x71 {
        return Some(Frame::Registers {
            address: u16::from_le_bytes([b[2], b[3]]),
            values: std::array::from_fn(|i| i16::from_le_bytes([b[4 + 2 * i], b[5 + 2 * i]])),
        });
    }
    if b[1] == 0x61 {
        return Some(Frame::Motion {
            acceleration_g: triple(2, 16.0),
            angular_velocity_dps: triple(8, 2000.0),
            euler_xyz_deg: triple(14, 180.0),
        });
    }
    let mut offset = 2;
    let sample_time_ms = if b[1] & 0x80 != 0 {
        offset += 8;
        Some(device_calendar_ms(b[2..10].try_into().ok()?)?)
    } else {
        None
    };
    let acceleration_g = if b[1] & 0x40 != 0 {
        let values = triple(offset, 16.0);
        offset += 6;
        Some(values)
    } else {
        None
    };
    let angular_velocity_dps = if b[1] & 0x20 != 0 {
        let values = triple(offset, 2000.0);
        offset += 6;
        Some(values)
    } else {
        None
    };
    let euler_xyz_deg = (b[1] & 0x01 != 0).then(|| triple(offset, 180.0));
    let quaternion_wxyz = if b[1] & 0x04 != 0 {
        let q: [f64; 4] = std::array::from_fn(|i| word(offset + 2 * i));
        // Native int16 quaternions must be close to unit length. This also helps
        // recover framing; these formats have no checksum, so detection is limited.
        if !(0.81..=1.21).contains(&q.iter().map(|v| v * v).sum::<f64>()) {
            return None;
        }
        Some(q)
    } else {
        None
    };
    Some(Frame::Stream {
        profile: b[1],
        acceleration_g,
        sample_time_ms,
        angular_velocity_dps,
        euler_xyz_deg,
        quaternion_wxyz,
    })
}

#[derive(Default)]
pub struct Parser {
    buffer: VecDeque<u8>,
    pub discarded_bytes: u64,
    pub invalid_frames: u64,
}

impl Parser {
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut frames = Vec::new();
        // Byte-wise ingestion keeps the incomplete-frame buffer bounded to 30 bytes.
        for byte in bytes {
            self.buffer.push_back(*byte);
            while self.buffer.len() >= 2 {
                let length = if self.buffer[0] == 0x55 {
                    frame_length(self.buffer[1])
                } else {
                    None
                };
                let Some(length) = length else {
                    self.buffer.pop_front();
                    self.discarded_bytes += 1;
                    continue;
                };
                if self.buffer.len() < length {
                    break;
                }
                if let Some(frame) = decode(&self.buffer.make_contiguous()[..length]) {
                    self.buffer.drain(..length);
                    frames.push(frame);
                } else {
                    self.invalid_frames += 1;
                    self.buffer.pop_front();
                    self.discarded_bytes += 1;
                }
            }
        }
        frames
    }
}

pub const UNLOCK: [u8; 5] = [0xff, 0xaa, 0x69, 0x88, 0xb5];
pub fn read_register(address: u16) -> [u8; 5] {
    let [lo, hi] = address.to_le_bytes();
    [0xff, 0xaa, 0x27, lo, hi]
}

/// Register values from WIT's REG.h and BLE SDK; writes are never sent on ordinary connection.
pub fn command_register(command: &DeviceCommand) -> Result<(u8, u16)> {
    Ok(match command {
        DeviceCommand::Rate { hz } => (
            0x03,
            match hz {
                1 => 0x03,
                2 => 0x04,
                5 => 0x05,
                10 => 0x06,
                20 => 0x07,
                50 => 0x08,
                100 => 0x09,
                200 => 0x0b,
                _ => {
                    return Err(Error::Invalid(
                        "supported device rates: 1, 2, 5, 10, 20, 50, 100, 200 Hz".into(),
                    ));
                }
            },
        ),
        DeviceCommand::Output { format } => (0x0e, *format as u16),
        DeviceCommand::AccelCalibrate => (0x01, 0x01),
        DeviceCommand::MagStart => (0x01, 0x07),
        DeviceCommand::MagStop => (0x01, 0x00),
        DeviceCommand::Save => (0x00, 0x00),
        DeviceCommand::Algorithm { mode } => (0x24, *mode as u16),
        DeviceCommand::ZeroYaw => (0x01, 0x04),
        DeviceCommand::AngleReference => (0x01, 0x08),
        DeviceCommand::ResetDefaults => (0x00, 0x01),
    })
}

pub fn write_register(address: u8, value: u16) -> [u8; 5] {
    let [lo, hi] = value.to_le_bytes();
    [0xff, 0xaa, address, lo, hi]
}
