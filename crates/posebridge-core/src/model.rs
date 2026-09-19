use crate::pose::Mounting;
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid argument: {0}")]
    Invalid(String),
    #[error("operation is busy; stop the current operation first")]
    Busy,
    #[error("device or adapter unavailable: {0}")]
    Unavailable(String),
    #[error("permission denied: {0}")]
    Permission(String),
    #[error("I/O error: {0}")]
    Io(String),
    #[error("protocol error: {0}")]
    Protocol(String),
    #[error("operation timed out: {0}")]
    Timeout(String),
    #[error("operation cancelled")]
    Cancelled,
    #[error("internal error: {0}")]
    Internal(String),
}
pub type Result<T> = std::result::Result<T, Error>;

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        if e.kind() == std::io::ErrorKind::PermissionDenied {
            Self::Permission(e.to_string())
        } else {
            Self::Io(e.to_string())
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TransportKind {
    Ble,
    Usb,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum PoseInput {
    #[default]
    Euler,
    Quaternion,
    /// Native notification quaternion, without register polling.
    StreamQuaternion,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Pattern {
    #[default]
    Fixed,
    Yaw,
    Combined,
    Wrap,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum BleConnectionMode {
    #[default]
    Default,
    Throughput,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Source {
    Ble {
        device_id: String,
        #[serde(default)]
        connection_mode: BleConnectionMode,
    },
    Usb {
        port: String,
        baud: u32,
    },
    Simulate {
        #[serde(default)]
        pattern: Pattern,
        #[serde(default)]
        euler_deg: [f64; 3],
        #[serde(default = "default_rate")]
        rate_hz: u32,
        /// Synthetic elapsed clock, never a device timestamp.
        #[serde(default)]
        sample_clock: bool,
    },
}

pub fn default_rate() -> u32 {
    100
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum OscFormat {
    #[default]
    Quaternion,
    Euler,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum OscVersion {
    #[default]
    V1,
    V2,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OscConfig {
    pub target: SocketAddr,
    #[serde(default = "default_rate")]
    pub max_rate_hz: u32,
    #[serde(default)]
    pub format: OscFormat,
    #[serde(default)]
    pub version: OscVersion,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub source: Source,
    #[serde(default)]
    pub pose_input: PoseInput,
    /// Explicit sensor axes pointing right, forward and up; required for hardware.
    #[serde(default)]
    pub mounting: Option<Mounting>,
    #[serde(default)]
    pub osc: Option<OscConfig>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            source: Source::Simulate {
                pattern: Pattern::Fixed,
                euler_deg: [0.0; 3],
                rate_hz: 100,
                sample_clock: false,
            },
            pose_input: PoseInput::Euler,
            mounting: None,
            osc: None,
        }
    }
}

impl Config {
    pub fn validate(&self) -> Result<()> {
        match &self.source {
            Source::Ble { device_id, .. } if device_id.trim().is_empty() => {
                return Err(Error::Invalid("BLE device_id is empty".into()));
            }
            Source::Ble {
                connection_mode: BleConnectionMode::Throughput,
                ..
            } if !cfg!(target_os = "windows") => {
                return Err(Error::Invalid(
                    "BLE throughput preference requires Windows 11 or later".into(),
                ));
            }
            Source::Usb { port, baud } if port.trim().is_empty() || *baud == 0 => {
                return Err(Error::Invalid("USB port or baud is invalid".into()));
            }
            Source::Simulate {
                euler_deg, rate_hz, ..
            } => {
                validate_rate(*rate_hz)?;
                crate::pose::from_euler(*euler_deg)?;
            }
            _ => {}
        }
        if !matches!(self.source, Source::Simulate { .. }) {
            self.mounting
                .ok_or_else(|| {
                    Error::Invalid("hardware input requires an explicit mounting".into())
                })?
                .validate()?;
        }
        if let Some(osc) = &self.osc {
            validate_rate(osc.max_rate_hz)?;
            if !osc.target.ip().is_loopback() || osc.target.port() == 0 {
                return Err(Error::Invalid(
                    "OSC target must be a nonzero loopback port".into(),
                ));
            }
        }
        Ok(())
    }
}

pub fn validate_rate(rate: u32) -> Result<()> {
    if !(1..=200).contains(&rate) {
        return Err(Error::Invalid("rate must be between 1 and 200 Hz".into()));
    }
    Ok(())
}

#[derive(Clone, Debug, Serialize)]
pub struct DeviceInfo {
    pub transport: TransportKind,
    pub id: String,
    pub name: String,
    pub rssi: Option<i16>,
    pub usb_vid: Option<u16>,
    pub usb_pid: Option<u16>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, PartialEq, Eq)]
#[repr(u32)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionState {
    #[default]
    Idle = 0,
    Scanning = 1,
    Connecting = 2,
    Active = 3,
    Stale = 4,
    Reconnecting = 5,
    Stopped = 6,
    Failed = 7,
    Configuring = 8,
    Complete = 9,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct RawData {
    pub acceleration_g: Option<[f64; 3]>,
    pub angular_velocity_dps: Option<[f64; 3]>,
    pub euler_xyz_deg: Option<[f64; 3]>,
    pub quaternion_wxyz: Option<[f64; 4]>,
    pub motion_received_ns: Option<u64>,
    pub quaternion_received_ns: Option<u64>,
}

/// Calendar time is in the unsynchronized device clock, not UTC.
#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[repr(u32)]
#[serde(rename_all = "snake_case")]
pub enum SampleTimeKind {
    DeviceCalendar = 1,
    SimulatedElapsed = 2,
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
pub struct SampleTime {
    pub kind: SampleTimeKind,
    pub time_ms: u64,
    /// Starts at 1 per connection; increments on clock discontinuities.
    pub clock_epoch: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct PoseSnapshot {
    pub session_id: u64,
    pub sequence: u64,
    pub received_ns: u64,
    /// Belongs only to this pose frame; absent on untimestamped/register data.
    pub sample_time: Option<SampleTime>,
    pub quaternion_xyzw: [f64; 4],
    pub euler_deg: [f64; 3],
    pub raw: RawData,
    pub fresh: bool,
}

/// Host read/notification boundaries, before parsing or OSC coalescing.
#[derive(Clone, Debug, Default, Serialize)]
pub struct DeliveryStats {
    pub reads: u64,
    pub max_bytes_per_read: u64,
    pub max_frames_per_read: u64,
    /// Gap counts in [0,1), [1,10), [10,30), [30,100), [100,infinity) milliseconds.
    pub gap_histogram: [u64; 5],
    pub max_gap_ms: f64,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct BleLinkStatus {
    pub requested_mode: BleConnectionMode,
    pub request_status: Option<String>,
    pub connection_interval_ms: Option<f64>,
    pub peripheral_latency: Option<u16>,
    pub note: Option<String>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct StatusSnapshot {
    pub state: ConnectionState,
    pub session_id: u64,
    pub bytes_received: u64,
    pub frames_received: u64,
    pub discarded_bytes: u64,
    pub invalid_poses: u64,
    pub invalid_frames: u64,
    pub duplicate_sample_times: u64,
    pub clock_discontinuities: u64,
    pub pose_count: u64,
    pub osc_sent: u64,
    pub reconnect_count: u64,
    pub actual_rate_hz: f64,
    pub interval_min_ms: f64,
    pub interval_max_ms: f64,
    pub last_error: Option<String>,
    pub configuration_report: Option<String>,
    pub delivery: DeliveryStats,
    pub ble_link: Option<BleLinkStatus>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case", deny_unknown_fields)]
pub enum DeviceCommand {
    Rate { hz: u32 },
    Output { format: OutputProfile },
    AccelCalibrate,
    MagStart,
    MagStop,
    Save,
}

/// Verified, bounded new-firmware stream profiles. No implicit save to flash.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[repr(u16)]
pub enum OutputProfile {
    Motion = 0x61,
    TimestampEuler = 0x81,
    TimestampQuaternion = 0x84,
    TimestampGyroQuaternion = 0xa4,
}
