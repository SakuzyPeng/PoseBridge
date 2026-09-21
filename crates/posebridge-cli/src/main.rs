use clap::{Args, Parser, Subcommand, ValueEnum};
use posebridge_core::{
    AlgorithmMode, BleConnectionMode, Config, ConnectionState, Controller, DeviceCommand, Error,
    OscConfig, OscFormat, OutputProfile, Pattern, PoseInput, Result, Source, TransportKind,
    pose::Mounting,
};
use std::net::SocketAddr;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::{Duration, Instant};

#[derive(Parser)]
#[command(
    name = "posebridge",
    version,
    about = "Read WIT BLE/USB orientation and bridge it to local OSC"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Clone, Copy, ValueEnum)]
enum Transport {
    Ble,
    Usb,
}
impl From<Transport> for TransportKind {
    fn from(t: Transport) -> Self {
        match t {
            Transport::Ble => Self::Ble,
            Transport::Usb => Self::Usb,
        }
    }
}
#[derive(Clone, Copy, ValueEnum)]
enum Input {
    Euler,
    Quaternion,
    StreamQuaternion,
}
#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
enum BleMode {
    Default,
    Throughput,
}
#[derive(Clone, Copy, ValueEnum)]
enum Format {
    Quaternion,
    Euler,
}
#[derive(Clone, Copy, ValueEnum)]
enum Algorithm {
    SixAxis,
    NineAxis,
}
#[derive(Clone, Copy, ValueEnum)]
enum Profile {
    Motion,
    TimestampEuler,
    TimestampQuaternion,
    TimestampGyroQuaternion,
    #[value(name = "experimental-full-inertial-20hz")]
    ExperimentalFullInertial20Hz,
}
#[derive(Clone, Copy, ValueEnum)]
enum Trajectory {
    Fixed,
    Yaw,
    Combined,
    Wrap,
}

#[derive(Args)]
struct InputArgs {
    /// Logical identity shared with consumers; default transport:platform-identifier.
    #[arg(long)]
    source_id: Option<String>,
    #[arg(long, value_enum)]
    transport: Transport,
    /// Platform BLE identifier returned by scan; not a device name.
    #[arg(long)]
    device: Option<String>,
    #[arg(long)]
    port: Option<String>,
    #[arg(long, default_value_t = 115200)]
    baud: u32,
    /// Sensor axes pointing towards head right, forward, up (e.g. -y,+x,+z).
    #[arg(long, allow_hyphen_values = true)]
    mount: Option<String>,
    #[arg(long, value_enum, default_value = "euler")]
    pose_input: Input,
    /// Windows 11+ connection preference, held only while connected; does not configure the sensor.
    #[arg(long, value_enum, default_value = "default")]
    ble_mode: BleMode,
}

impl InputArgs {
    fn config(self, require_mount: bool) -> Result<Config> {
        let mounting = match self.mount {
            Some(value) => Mounting::parse(&value)?,
            None if require_mount => return Err(Error::Invalid(
                "bridge requires --mount (right,forward,up sensor axes); verify the mounting first"
                    .into(),
            )),
            None => Mounting::parse("+x,+y,+z")?,
        };
        let source = match self.transport {
            Transport::Ble => {
                if self.port.is_some() {
                    return Err(Error::Invalid("--port is only valid for USB".into()));
                }
                Source::Ble {
                    device_id: self
                        .device
                        .ok_or_else(|| Error::Invalid("BLE requires --device from scan".into()))?,
                    connection_mode: match self.ble_mode {
                        BleMode::Default => BleConnectionMode::Default,
                        BleMode::Throughput => BleConnectionMode::Throughput,
                    },
                }
            }
            Transport::Usb => {
                if self.device.is_some() {
                    return Err(Error::Invalid("--device is only valid for BLE".into()));
                }
                if self.ble_mode != BleMode::Default {
                    return Err(Error::Invalid("--ble-mode is only valid for BLE".into()));
                }
                Source::Usb {
                    port: self
                        .port
                        .ok_or_else(|| Error::Invalid("USB requires --port from scan".into()))?,
                    baud: self.baud,
                }
            }
        };
        Ok(Config {
            source,
            source_id: self.source_id,
            mounting: Some(mounting),
            pose_input: match self.pose_input {
                Input::Euler => PoseInput::Euler,
                Input::Quaternion => PoseInput::Quaternion,
                Input::StreamQuaternion => PoseInput::StreamQuaternion,
            },
            osc: None,
        })
    }
}

#[derive(Args)]
struct OutputArgs {
    #[arg(long, default_value = "127.0.0.1:9000")]
    osc_target: SocketAddr,
    /// Upper target OSC cadence; does not change the sensor's configured rate.
    #[arg(long, default_value_t = 100)]
    osc_rate_hz: u32,
    #[arg(long, value_enum, default_value = "quaternion")]
    format: Format,
}
impl From<OutputArgs> for OscConfig {
    fn from(v: OutputArgs) -> Self {
        Self {
            target: v.osc_target,
            max_rate_hz: v.osc_rate_hz,
            format: match v.format {
                Format::Quaternion => OscFormat::Quaternion,
                Format::Euler => OscFormat::Euler,
            },
        }
    }
}

#[derive(Subcommand)]
enum ConfigureAction {
    Rate {
        #[arg(long)]
        hz: u32,
    },
    /// Select a verified new-firmware stream profile (register 0x0E); read back, no save.
    Output {
        #[arg(long, value_enum)]
        format: Profile,
    },
    AccelCalibrate,
    MagStart,
    MagStop,
    Save,
    /// Explicit algorithm selection; does not save to flash.
    Algorithm {
        #[arg(long, value_enum)]
        mode: Algorithm,
    },
    /// Six-axis only; does not implicitly change algorithm or save.
    ZeroYaw,
    /// Set device angle reference AND send SAVE (persistent device operation).
    AngleReference,
    /// Restore documented device defaults AND save them to flash.
    ResetDefaults,
}

#[derive(Subcommand)]
enum Command {
    /// Enumerate devices without connecting or writing configuration.
    Scan {
        #[arg(long, value_enum, default_value = "ble")]
        transport: Transport,
        #[arg(long, default_value_t = 5)]
        timeout_seconds: u32,
        #[arg(long)]
        json: bool,
    },
    /// Read the documented configuration registers without changing any settings.
    Inspect {
        #[command(flatten)]
        input: InputArgs,
        #[arg(long)]
        json: bool,
    },
    /// Show raw sensor data, pose, real receive rate and state; never configure the device.
    Diagnose {
        #[command(flatten)]
        input: InputArgs,
        #[arg(long, default_value_t = 10.0)]
        duration: f64,
        #[arg(long)]
        json: bool,
    },
    /// Stream fresh poses to OSC. Ctrl+C releases the device.
    Bridge {
        #[command(flatten)]
        input: InputArgs,
        #[command(flatten)]
        output: OutputArgs,
        #[arg(long, default_value_t = 0.0)]
        duration: f64,
        #[arg(long)]
        json: bool,
    },
    /// Send deterministic, hardware-free pose trajectories.
    Simulate {
        #[arg(long)]
        source_id: Option<String>,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        yaw: f64,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        pitch: f64,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        roll: f64,
        #[arg(long, value_enum, default_value = "fixed")]
        pattern: Trajectory,
        #[arg(long, default_value_t = 100)]
        sample_rate_hz: u32,
        /// Include explicitly synthetic elapsed sample time.
        #[arg(long)]
        sample_clock: bool,
        #[command(flatten)]
        output: OutputArgs,
        #[arg(long, default_value_t = 0.0)]
        duration: f64,
        #[arg(long)]
        json: bool,
    },
    /// Explicit device writes. Rate/calibration do not automatically save to flash.
    Configure {
        #[command(flatten)]
        input: InputArgs,
        #[arg(long)]
        json: bool,
        #[command(subcommand)]
        action: ConfigureAction,
    },
}

fn wait_operation(controller: &mut Controller, interrupted: &AtomicBool) -> Result<()> {
    loop {
        if interrupted.load(Ordering::Relaxed) {
            // Complete cancellation before callers serialize the terminal snapshot.
            controller.stop()?;
            return Err(Error::Cancelled);
        }
        let status = controller.status();
        match status.state {
            ConnectionState::Complete => return Ok(()),
            ConnectionState::Failed => {
                return Err(Error::Io(
                    status
                        .last_error
                        .unwrap_or_else(|| "operation failed".into()),
                ));
            }
            _ => std::thread::sleep(Duration::from_millis(50)),
        }
    }
}

fn stream(
    controller: &mut Controller,
    config: Config,
    duration: f64,
    json: bool,
    interrupted: &AtomicBool,
) -> Result<()> {
    if !duration.is_finite() || duration < 0.0 {
        return Err(Error::Invalid(
            "duration must be finite and nonnegative; zero runs until Ctrl+C".into(),
        ));
    }
    controller.set_config(config)?;
    controller.start()?;
    let start = Instant::now();
    let result = (|| {
        loop {
            if interrupted.load(Ordering::Relaxed)
                || (duration > 0.0 && start.elapsed().as_secs_f64() >= duration)
            {
                return Ok(());
            }
            let snapshot = controller.snapshot();
            let status = snapshot.status.clone();
            let pose = snapshot.pose.clone();
            if json {
                println!("{}", posebridge_core::snapshot_json(&snapshot)?);
            } else {
                let angles = pose.as_ref().map(|p| p.euler_deg);
                let raw = pose.as_ref().and_then(|p| p.raw.euler_xyz_deg);
                println!(
                    "{:?} session={} samples={} rate={:.2}Hz osc={} yaw/pitch/roll={:?} raw XYZ={:?} sample_time={:?}{}",
                    status.state,
                    status.session_id,
                    status.pose_count,
                    status.actual_rate_hz,
                    status.osc_sent,
                    angles,
                    raw,
                    pose.as_ref().and_then(|p| p.sample_time),
                    status
                        .last_error
                        .as_ref()
                        .map(|e| format!(" error={e}"))
                        .unwrap_or_default()
                );
            }
            if status.state == ConnectionState::Failed {
                return Err(Error::Io(
                    status
                        .last_error
                        .unwrap_or_else(|| "acquisition failed".into()),
                ));
            }
            std::thread::sleep(Duration::from_millis(250));
        }
    })();
    let final_status = controller.status();
    let stopped = controller.stop();
    result?;
    stopped?;
    if json {
        println!(
            "{}",
            posebridge_core::snapshot_json(&controller.snapshot())?
        );
    }
    if final_status.pose_count == 0 {
        return Err(Error::Unavailable(
            "no valid poses received during this run".into(),
        ));
    }
    Ok(())
}

fn run() -> Result<()> {
    let cli = Cli::parse();
    let interrupted = Arc::new(AtomicBool::new(false));
    let flag = interrupted.clone();
    ctrlc::set_handler(move || flag.store(true, Ordering::Relaxed))
        .map_err(|e| Error::Internal(e.to_string()))?;
    let mut controller = Controller::new()?;
    match cli.command {
        Command::Scan {
            transport,
            timeout_seconds,
            json,
        } => {
            controller.scan_start(transport.into(), timeout_seconds)?;
            wait_operation(&mut controller, &interrupted)?;
            let devices = controller.devices();
            if json {
                println!(
                    "{}",
                    serde_json::to_string_pretty(&devices)
                        .map_err(|e| Error::Internal(e.to_string()))?
                );
            } else if devices.is_empty() {
                println!(
                    "No matching devices found. Check power, permissions and other connected apps."
                );
            } else {
                for d in devices {
                    println!(
                        "{:?}\t{}\t{}\tRSSI={:?}\tUSB={:?}:{:?}",
                        d.transport, d.id, d.name, d.rssi, d.usb_vid, d.usb_pid
                    );
                }
            }
            controller.stop()
        }
        Command::Inspect { input, json } => {
            controller.set_config(input.config(false)?)?;
            controller.inspect_start()?;
            let result = wait_operation(&mut controller, &interrupted);
            let snapshot = controller.snapshot();
            if json {
                println!("{}", posebridge_core::snapshot_json(&snapshot)?);
            } else {
                println!(
                    "{}",
                    serde_json::to_string_pretty(&snapshot.descriptor)
                        .map_err(|e| Error::Internal(e.to_string()))?
                );
            }
            result?;
            controller.stop()
        }
        Command::Diagnose {
            input,
            duration,
            json,
        } => {
            if input.mount.is_none() {
                eprintln!(
                    "Diagnostic mounting: +x,+y,+z (sensor identity), not a verified head mounting. Raw XYZ is unchanged."
                );
            }
            stream(
                &mut controller,
                input.config(false)?,
                duration,
                json,
                &interrupted,
            )
        }
        Command::Bridge {
            input,
            output,
            duration,
            json,
        } => {
            let mut config = input.config(true)?;
            config.osc = Some(output.into());
            stream(&mut controller, config, duration, json, &interrupted)
        }
        Command::Simulate {
            source_id,
            yaw,
            pitch,
            roll,
            pattern,
            sample_rate_hz,
            sample_clock,
            output,
            duration,
            json,
        } => {
            let config = Config {
                source_id,
                source: Source::Simulate {
                    pattern: match pattern {
                        Trajectory::Fixed => Pattern::Fixed,
                        Trajectory::Yaw => Pattern::Yaw,
                        Trajectory::Combined => Pattern::Combined,
                        Trajectory::Wrap => Pattern::Wrap,
                    },
                    euler_deg: [yaw, pitch, roll],
                    rate_hz: sample_rate_hz,
                    sample_clock,
                },
                osc: Some(output.into()),
                ..Config::default()
            };
            stream(&mut controller, config, duration, json, &interrupted)
        }
        Command::Configure {
            input,
            action,
            json,
        } => {
            let command = match action {
                ConfigureAction::Rate { hz } => DeviceCommand::Rate { hz },
                ConfigureAction::Output { format } => DeviceCommand::Output {
                    format: match format {
                        Profile::Motion => OutputProfile::Motion,
                        Profile::TimestampEuler => OutputProfile::TimestampEuler,
                        Profile::TimestampQuaternion => OutputProfile::TimestampQuaternion,
                        Profile::TimestampGyroQuaternion => OutputProfile::TimestampGyroQuaternion,
                        Profile::ExperimentalFullInertial20Hz => {
                            OutputProfile::ExperimentalFullInertial20Hz
                        }
                    },
                },
                ConfigureAction::AccelCalibrate => DeviceCommand::AccelCalibrate,
                ConfigureAction::MagStart => DeviceCommand::MagStart,
                ConfigureAction::MagStop => DeviceCommand::MagStop,
                ConfigureAction::Save => DeviceCommand::Save,
                ConfigureAction::Algorithm { mode } => DeviceCommand::Algorithm {
                    mode: match mode {
                        Algorithm::SixAxis => AlgorithmMode::SixAxis,
                        Algorithm::NineAxis => AlgorithmMode::NineAxis,
                    },
                },
                ConfigureAction::ZeroYaw => DeviceCommand::ZeroYaw,
                ConfigureAction::AngleReference => DeviceCommand::AngleReference,
                ConfigureAction::ResetDefaults => DeviceCommand::ResetDefaults,
            };
            controller.set_config(input.config(false)?)?;
            controller.configure_device(command)?;
            let result = wait_operation(&mut controller, &interrupted);
            if json {
                println!(
                    "{}",
                    posebridge_core::snapshot_json(&controller.snapshot())?
                );
            } else {
                println!(
                    "{}",
                    controller.status().configuration_report.unwrap_or_default()
                );
            }
            result?;
            controller.stop()
        }
    }
}

fn main() {
    if let Err(error) = run() {
        eprintln!("posebridge: {error}");
        std::process::exit(if matches!(error, Error::Cancelled) {
            130
        } else {
            1
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interrupted_wait_stops_worker_before_returning_snapshot() {
        let mut controller = Controller::new().unwrap();
        controller.start().unwrap();

        let result = wait_operation(&mut controller, &AtomicBool::new(true));
        assert!(matches!(result, Err(Error::Cancelled)));
        let snapshot = controller.snapshot();
        assert_eq!(snapshot.status.state, ConnectionState::Stopped);
        assert!(snapshot.pose.is_none_or(|pose| !pose.fresh));
        // A returned cancellation must also have released the operation slot.
        controller.start().unwrap();
        controller.stop().unwrap();
    }
}
