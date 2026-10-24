//! Client preferences; device I/O stays on a dedicated worker.
use serde::{Deserialize, Serialize};

mod configuration;
mod magnetic;
pub mod service;
pub mod ui;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Transport {
    #[default]
    Ble,
    Usb,
}
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Input {
    #[default]
    Automatic,
    RegisterQuaternion,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Device {
    pub transport: Transport,
    pub id: String,
    pub name: String,
    pub baud: u32,
    pub mounting: [i8; 3],
    pub input: Input,
    pub throughput: bool,
}
impl Default for Device {
    fn default() -> Self {
        Self {
            transport: Transport::Ble,
            id: String::new(),
            name: String::new(),
            baud: 115_200,
            mounting: [0; 3],
            input: Input::Automatic,
            throughput: false,
        }
    }
}
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Preferences {
    pub device: Device,
    pub remembered: Vec<Device>,
    pub output: crate::output::Preferences,
}
impl Preferences {
    pub fn validate(&mut self) {
        self.output.max_rate_hz = self.output.max_rate_hz.clamp(1, 200);
        for device in std::iter::once(&mut self.device).chain(self.remembered.iter_mut()) {
            if device.baud == 0 {
                device.baud = 115_200;
            }
            if !cfg!(target_os = "windows") {
                device.throughput = false;
            }
        }
    }
    pub fn remember(&mut self) {
        if self.device.id.is_empty() {
            return;
        }
        self.remembered
            .retain(|d| d.transport != self.device.transport || d.id != self.device.id);
        self.remembered.push(self.device.clone());
    }
    pub fn select(&mut self, transport: Transport, id: String, name: String) {
        self.remember();
        self.device = self
            .remembered
            .iter()
            .find(|d| d.transport == transport && d.id == id)
            .cloned()
            .unwrap_or(Device {
                transport,
                id,
                name,
                ..Device::default()
            });
    }
}

mod coverage;
pub mod mounting;
pub use crate::tracking::Sample;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn defaults_are_disconnected_unmapped_and_output_is_opt_in() {
        let mut p: Preferences = serde_json::from_str("{}").unwrap();
        assert!(!p.output.enabled);
        assert_eq!(p.device.mounting, [0; 3]);
        p.device.baud = 0;
        p.output.max_rate_hz = 0;
        p.validate();
        assert_eq!(p.device.baud, 115_200);
        assert_eq!(p.output.max_rate_hz, 1);
    }
}
