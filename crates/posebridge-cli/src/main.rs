use clap::{Args, Parser, Subcommand, ValueEnum};
use posebridge_core::{
    Config, ConnectionState, Controller, DeviceCommand, Error, OscConfig, OscFormat, Pattern,
    PoseInput, Result, Source, TransportKind, pose::Mounting,
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
}
#[derive(Clone, Copy, ValueEnum)]
enum Format {
    Quaternion,
    Euler,
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
                }
            }
            Transport::Usb => {
                if self.device.is_some() {
                    return Err(Error::Invalid("--device is only valid for BLE".into()));
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
            mounting: Some(mounting),
            pose_input: match self.pose_input {
                Input::Euler => PoseInput::Euler,
                Input::Quaternion => PoseInput::Quaternion,
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
    AccelCalibrate,
    MagStart,
    MagStop,
    Save,
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
        #[command(subcommand)]
        action: ConfigureAction,
    },
}

fn wait_operation(controller: &Controller, interrupted: &AtomicBool) -> Result<()> {
    loop {
        if interrupted.load(Ordering::Relaxed) {
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
            let status = controller.status();
            let pose = controller.latest_pose();
            if json {
                println!("{}", serde_json::json!({"status":status,"pose":pose}));
            } else {
                let angles = pose.as_ref().map(|p| p.euler_deg);
                let raw = pose.as_ref().and_then(|p| p.raw.euler_xyz_deg);
                println!(
                    "{:?} session={} samples={} rate={:.2}Hz osc={} yaw/pitch/roll={:?} raw XYZ={:?}{}",
                    status.state,
                    status.session_id,
                    status.pose_count,
                    status.actual_rate_hz,
                    status.osc_sent,
                    angles,
                    raw,
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
            wait_operation(&controller, &interrupted)?;
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
            yaw,
            pitch,
            roll,
            pattern,
            sample_rate_hz,
            output,
            duration,
            json,
        } => {
            let config = Config {
                source: Source::Simulate {
                    pattern: match pattern {
                        Trajectory::Fixed => Pattern::Fixed,
                        Trajectory::Yaw => Pattern::Yaw,
                        Trajectory::Combined => Pattern::Combined,
                        Trajectory::Wrap => Pattern::Wrap,
                    },
                    euler_deg: [yaw, pitch, roll],
                    rate_hz: sample_rate_hz,
                },
                osc: Some(output.into()),
                ..Config::default()
            };
            stream(&mut controller, config, duration, json, &interrupted)
        }
        Command::Configure { input, action } => {
            let command = match action {
                ConfigureAction::Rate { hz } => DeviceCommand::Rate { hz },
                ConfigureAction::AccelCalibrate => DeviceCommand::AccelCalibrate,
                ConfigureAction::MagStart => DeviceCommand::MagStart,
                ConfigureAction::MagStop => DeviceCommand::MagStop,
                ConfigureAction::Save => DeviceCommand::Save,
            };
            controller.set_config(input.config(false)?)?;
            controller.configure_device(command)?;
            wait_operation(&controller, &interrupted)?;
            println!(
                "{}",
                controller.status().configuration_report.unwrap_or_default()
            );
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
