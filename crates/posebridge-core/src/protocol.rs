//! WIT BLE 5.0 20-byte wire format, also observed on BWT901BLECL5.0 USB.
use crate::{DeviceCommand, Error, Result};
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub enum Frame {
    Motion {
        acceleration_g: [f64; 3],
        angular_velocity_dps: [f64; 3],
        euler_xyz_deg: [f64; 3],
    },
    Registers {
        address: u16,
        values: [i16; 8],
    },
}

#[derive(Default)]
pub struct Parser {
    buffer: VecDeque<u8>,
    pub discarded_bytes: u64,
}

impl Parser {
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut frames = Vec::new();
        for byte in bytes {
            self.buffer.push_back(*byte);
            while self.buffer.len() >= 2
                && (self.buffer[0] != 0x55 || !matches!(self.buffer[1], 0x61 | 0x71))
            {
                self.buffer.pop_front();
                self.discarded_bytes += 1;
            }
            if self.buffer.len() < 20 {
                continue;
            }
            let b: Vec<_> = self.buffer.drain(..20).collect();
            let word = |i| i16::from_le_bytes([b[i], b[i + 1]]);
            if b[1] == 0x61 {
                frames.push(Frame::Motion {
                    acceleration_g: [2, 4, 6].map(|i| word(i) as f64 / 32768.0 * 16.0),
                    angular_velocity_dps: [8, 10, 12].map(|i| word(i) as f64 / 32768.0 * 2000.0),
                    euler_xyz_deg: [14, 16, 18].map(|i| word(i) as f64 / 32768.0 * 180.0),
                });
            } else {
                frames.push(Frame::Registers {
                    address: u16::from_le_bytes([b[2], b[3]]),
                    values: std::array::from_fn(|i| word(4 + i * 2)),
                });
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
        DeviceCommand::AccelCalibrate => (0x01, 0x01),
        DeviceCommand::MagStart => (0x01, 0x07),
        DeviceCommand::MagStop => (0x01, 0x00),
        DeviceCommand::Save => (0x00, 0x00),
    })
}

pub fn write_register(address: u8, value: u16) -> [u8; 5] {
    let [lo, hi] = value.to_le_bytes();
    [0xff, 0xaa, address, lo, hi]
}
