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
    /// Native notification quaternion, without quaternion register polling.
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

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OscConfig {
    pub target: SocketAddr,
    #[serde(default = "default_rate")]
    pub max_rate_hz: u32,
    #[serde(default)]
    pub format: OscFormat,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub source: Source,
    #[serde(default)]
    pub source_id: Option<String>,
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
            source_id: None,
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
        validate_source_id(&self.logical_source_id())?;
        if let Some(mounting) = self.mounting {
            mounting.validate()?;
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
    Inspecting = 10,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct RawData {
    pub acceleration_g: Option<[f64; 3]>,
    pub angular_velocity_dps: Option<[f64; 3]>,
    pub euler_xyz_deg: Option<[f64; 3]>,
    pub quaternion_wxyz: Option<[f64; 4]>,
    #[serde(serialize_with = "optional_decimal::serialize")]
    pub motion_received_ns: Option<u64>,
    #[serde(serialize_with = "optional_decimal::serialize")]
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
    #[serde(serialize_with = "decimal::serialize")]
    pub time_ms: u64,
    /// Starts at 1 per connection; increments on clock discontinuities.
    #[serde(serialize_with = "decimal::serialize")]
    pub clock_epoch: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct PoseSnapshot {
    #[serde(serialize_with = "decimal::serialize")]
    pub instance_id: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub reference_epoch: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub metadata_revision: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub session_id: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub sequence: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub received_ns: u64,
    /// Host monotonic time since reception at this query; excludes device/link delay.
    #[serde(serialize_with = "decimal::serialize")]
    pub age_ns: u64,
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
    #[serde(serialize_with = "decimal::serialize")]
    pub reads: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub max_bytes_per_read: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub max_frames_per_read: u64,
    /// Gap counts in [0,1), [1,10), [10,30), [30,100), [100,infinity) milliseconds.
    #[serde(serialize_with = "decimal_array::serialize")]
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
pub struct BatteryStatus {
    /// Last valid register 0x64 value, in centivolts; null until observed.
    pub raw_register: Option<u16>,
    pub voltage_v: Option<f64>,
    /// Coarse WIT BLE 5.0 voltage estimate; null above 4.30 V (possible supply
    /// voltage), or before a valid reading. Does not indicate charging state.
    pub estimated_percent: Option<u8>,
    /// Host elapsed time since the last valid response, independent of pose age.
    #[serde(serialize_with = "optional_decimal::serialize")]
    pub age_ns: Option<u64>,
    pub fresh: bool,
    /// Battery query failures do not fail orientation acquisition.
    pub last_error: Option<String>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct StatusSnapshot {
    pub state: ConnectionState,
    #[serde(serialize_with = "decimal::serialize")]
    pub session_id: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub bytes_received: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub frames_received: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub discarded_bytes: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub invalid_poses: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub invalid_frames: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub duplicate_sample_times: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub clock_discontinuities: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub pose_count: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub session_samples: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub osc_sent: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub coalesced_samples: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub send_errors: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub telemetry_sent: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub reconnect_count: u64,
    pub actual_rate_hz: f64,
    pub interval_min_ms: f64,
    pub interval_max_ms: f64,
    pub last_error: Option<String>,
    pub configuration_report: Option<String>,
    pub delivery: DeliveryStats,
    pub ble_link: Option<BleLinkStatus>,
    pub battery: BatteryStatus,
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
    Algorithm { mode: AlgorithmMode },
    ZeroYaw,
    AngleReference,
    ResetDefaults,
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
    /// Candidate 30-byte profile. Only 20 Hz; not a validated default preset.
    #[serde(rename = "experimental_full_inertial_20hz")]
    ExperimentalFullInertial20Hz = 0xe4,
}

pub const PROTOCOL_VERSION: u32 = 3;
/// Local Rust/CLI/C ABI snapshot schema, independent of the OSC wire protocol.
pub const SNAPSHOT_SCHEMA_VERSION: u32 = 4;
pub const MAX_SOURCE_ID_BYTES: usize = 256;

pub fn validate_source_id(value: &str) -> Result<()> {
    if value.trim().is_empty()
        || value.len() > MAX_SOURCE_ID_BYTES
        || value.chars().any(char::is_control)
    {
        return Err(Error::Invalid(
            "source_id must be 1..256 UTF-8 bytes without control characters".into(),
        ));
    }
    Ok(())
}

impl Config {
    pub fn logical_source_id(&self) -> String {
        self.source_id
            .clone()
            .unwrap_or_else(|| match &self.source {
                Source::Ble { device_id, .. } => format!("ble:{device_id}"),
                Source::Usb { port, .. } => format!("usb:{port}"),
                Source::Simulate { .. } => "simulate".into(),
            })
    }
    pub fn validate_acquisition(&self) -> Result<()> {
        self.validate()?;
        if !matches!(self.source, Source::Simulate { .. }) && self.mounting.is_none() {
            return Err(Error::Invalid(
                "hardware acquisition requires an explicit mounting".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
#[repr(u16)]
pub enum AlgorithmMode {
    NineAxis = 0,
    SixAxis = 1,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct DeviceObservation {
    pub valid: bool,
    #[serde(serialize_with = "optional_decimal::serialize")]
    pub observed_unix_ms: Option<u64>,
    pub calsw: Option<u16>,
    pub calibration_state: Option<String>,
    pub rate_hz: Option<f64>,
    pub output_fields: Option<Vec<String>>,
    pub algorithm: Option<AlgorithmMode>,
    pub firmware_version: Option<String>,
    pub rate_register: Option<u16>,
    pub output_register: Option<u16>,
    pub bandwidth_register: Option<u16>,
    pub orientation_register: Option<u16>,
    pub algorithm_register: Option<u16>,
    pub firmware_registers: Option<[u16; 2]>,
}

#[derive(Clone, Debug, Serialize)]
pub struct SourceDescriptor {
    pub source_id: String,
    #[serde(serialize_with = "decimal::serialize")]
    pub instance_id: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub session_id: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub metadata_revision: u64,
    #[serde(serialize_with = "decimal::serialize")]
    pub reference_epoch: u64,
    pub reference_reason: String,
    pub transport: String,
    pub platform_id: Option<String>,
    pub device_name: Option<String>,
    /// Discovery names and the selected driver do not prove a device model.
    pub device_model: Option<String>,
    pub coordinate_profile: String,
    pub application_config: Config,
    pub device: DeviceObservation,
    pub software_capabilities: Vec<String>,
    pub calibration_quality: Option<String>,
}

impl SourceDescriptor {
    pub fn new(config: &Config) -> Self {
        let (transport, platform_id) = match &config.source {
            Source::Ble { device_id, .. } => ("ble", Some(device_id.clone())),
            Source::Usb { port, .. } => ("usb", Some(port.clone())),
            Source::Simulate { .. } => ("simulate", None),
        };
        Self {
            source_id: config.logical_source_id(),
            instance_id: 0,
            session_id: 0,
            metadata_revision: 1,
            reference_epoch: 1,
            reference_reason: "new_context".into(),
            transport: transport.into(),
            platform_id,
            device_name: None,
            device_model: None,
            coordinate_profile: "posebridge.yxz.v1".into(),
            application_config: config.clone(),
            device: DeviceObservation::default(),
            calibration_quality: None,
            software_capabilities: [
                "inspect",
                "battery",
                "rate",
                "output",
                "algorithm",
                "zero_yaw",
                "angle_reference",
                "reset_defaults",
                "accel_calibrate",
                "mag_start",
                "mag_stop",
                "save",
            ]
            .into_iter()
            .map(str::to_string)
            .collect(),
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum OperationOutcome {
    Running,
    Succeeded,
    Unverified,
    Failed,
    Cancelled,
}

#[derive(Clone, Debug, Serialize)]
pub struct OperationStatus {
    pub source_id: Option<String>,
    #[serde(serialize_with = "decimal::serialize")]
    pub id: u64,
    pub action: String,
    pub outcome: OperationOutcome,
    pub write_attempted: bool,
    pub command_sent: bool,
    pub register_verified: bool,
    pub completion_observed: bool,
    pub persistence: String,
    pub reference_may_have_changed: bool,
    pub message: Option<String>,
}
impl OperationStatus {
    pub fn new(action: String) -> Self {
        Self {
            source_id: None,
            id: new_id(),
            action,
            outcome: OperationOutcome::Running,
            write_attempted: false,
            command_sent: false,
            register_verified: false,
            completion_observed: false,
            persistence: "not_requested".into(),
            reference_may_have_changed: false,
            message: None,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct Snapshot {
    pub schema: u32,
    pub descriptor: SourceDescriptor,
    pub status: StatusSnapshot,
    pub pose: Option<PoseSnapshot>,
    pub operation: Option<OperationStatus>,
}

pub fn new_id() -> u64 {
    (uuid::Uuid::new_v4().as_u128() as u64 & i64::MAX as u64).max(1)
}

/// Canonical JSON uses decimal strings for every 64-bit quantity. This includes
/// old-looking fields such as pose.sequence; there is no legacy JSON dialect.
pub fn snapshot_json(snapshot: &Snapshot) -> Result<String> {
    serde_json::to_string(snapshot).map_err(|e| Error::Internal(e.to_string()))
}

mod decimal {
    use serde::Serializer;
    pub fn serialize<S: Serializer>(value: &u64, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&value.to_string())
    }
}
mod optional_decimal {
    use serde::Serializer;
    pub fn serialize<S: Serializer>(value: &Option<u64>, serializer: S) -> Result<S::Ok, S::Error> {
        match value {
            Some(v) => serializer.serialize_some(&v.to_string()),
            None => serializer.serialize_none(),
        }
    }
}
mod decimal_array {
    use serde::{Serialize, Serializer};
    pub fn serialize<S: Serializer>(value: &[u64; 5], serializer: S) -> Result<S::Ok, S::Error> {
        value.map(|v| v.to_string()).serialize(serializer)
    }
}

impl Default for SourceDescriptor {
    fn default() -> Self {
        Self::new(&Config::default())
    }
}
