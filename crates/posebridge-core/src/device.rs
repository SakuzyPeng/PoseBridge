//! Serialized, explicit register operations. Acquisition never invokes these writes.
use crate::protocol::{self, Frame, Parser};
use crate::transport::{self, Connection};
use crate::*;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tokio::sync::watch;

#[derive(Clone, Copy)]
pub(crate) enum Progress {
    WriteAttempt { reference: bool, persistent: bool },
    CommandSent,
    RegisterVerified,
    CompletionObserved,
}

pub(crate) async fn read_registers(
    connection: &mut Connection,
    address: u16,
    expected: Option<u16>,
    cancel: &mut watch::Receiver<bool>,
) -> Result<[u16; 8]> {
    let mut last_observed = None;
    transport::cancel_after(cancel, Duration::from_secs(3), async {
        let request = protocol::read_register(address);
        connection.write(&request).await?;
        let mut retry_at = Instant::now() + Duration::from_millis(250);
        let mut parser = Parser::default();
        loop {
            tokio::select! {
                _ = tokio::time::sleep_until(retry_at.into()) => {
                    connection.write(&request).await?;
                    retry_at = Instant::now() + Duration::from_millis(250);
                },
                bytes = connection.read() => {
                    for frame in parser.push(&bytes?) {
                        if let Frame::Registers { address: base, values } = frame && base == address {
                            let values = values.map(|v| v as u16);
                            last_observed = Some(values[0]);
                            if expected.is_none_or(|v| v == values[0]) { return Ok(values); }
                        }
                    }
                },
            }
        }
    }).await.map_err(|e| match e {
        Error::Timeout(_) => match (expected, last_observed) {
            (Some(wanted), Some(observed)) => Error::Protocol(format!("register 0x{address:02x}: requested {wanted}, read back {observed} after verification timeout")),
            _ => Error::Timeout(format!("readback register 0x{address:02x}")),
        }, other => other,
    })
}

pub(crate) async fn inspect(
    connection: &mut Connection,
    cancel: &mut watch::Receiver<bool>,
) -> Result<DeviceObservation> {
    // Four exact reads cover only the named fields. Extra registers in a fixed
    // eight-word reply are ignored; no probing of arbitrary addresses.
    let control = read_registers(connection, 0x01, None, cancel).await?;
    let output = read_registers(connection, 0x0e, None, cancel).await?;
    let options = read_registers(connection, 0x1f, None, cancel).await?;
    let version = read_registers(connection, 0x2e, None, cancel).await?;
    Ok(DeviceObservation {
        valid: true,
        observed_unix_ms: Some(
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis()
                .min(i64::MAX as u128) as u64,
        ),
        calsw: Some(control[0]),
        calibration_state: match control[0] {
            0 => Some("idle".into()),
            1 => Some("accelerometer_active".into()),
            7 => Some("magnetic_active".into()),
            _ => None,
        },
        rate_hz: match control[2] {
            1 => Some(0.1),
            2 => Some(0.5),
            3 => Some(1.),
            4 => Some(2.),
            5 => Some(5.),
            6 => Some(10.),
            7 => Some(20.),
            8 => Some(50.),
            9 => Some(100.),
            11 => Some(200.),
            _ => None,
        },
        output_fields: match output[0] {
            0x61 => Some(vec!["acceleration", "angular_velocity", "euler"]),
            0x01 => Some(vec!["euler"]),
            0x04 => Some(vec!["quaternion"]),
            0x81 => Some(vec!["sample_time", "euler"]),
            0x84 => Some(vec!["sample_time", "quaternion"]),
            0xa4 => Some(vec!["sample_time", "angular_velocity", "quaternion"]),
            0xe4 => Some(vec![
                "sample_time",
                "acceleration",
                "angular_velocity",
                "quaternion",
            ]),
            _ => None,
        }
        .map(|items| items.into_iter().map(str::to_string).collect()),
        algorithm: match options[5] {
            0 => Some(AlgorithmMode::NineAxis),
            1 => Some(AlgorithmMode::SixAxis),
            _ => None,
        },
        firmware_version: (version[1] == 0).then(|| version[0].to_string()),
        rate_register: Some(control[2]),
        output_register: Some(output[0]),
        bandwidth_register: Some(options[0]),
        orientation_register: Some(options[4]),
        algorithm_register: Some(options[5]),
        firmware_registers: Some([version[0], version[1]]),
    })
}

async fn delay(cancel: &mut watch::Receiver<bool>, millis: u64) -> Result<()> {
    transport::cancel_after(cancel, Duration::from_secs(1), async {
        tokio::time::sleep(Duration::from_millis(millis)).await;
        Ok(())
    })
    .await
}

pub(crate) async fn execute(
    connection: &mut Connection,
    command: &DeviceCommand,
    cancel: &mut watch::Receiver<bool>,
    mut progress: impl FnMut(Progress),
) -> Result<(OperationOutcome, String, Option<DeviceObservation>)> {
    let (address, value) = protocol::command_register(command)?;
    if matches!(
        command,
        DeviceCommand::Output {
            format: OutputProfile::ExperimentalFullInertial20Hz
        }
    ) && read_registers(connection, 0x03, None, cancel).await?[0] != 7
    {
        return Err(Error::Invalid(
            "experimental full inertial output requires a verified 20 Hz rate".into(),
        ));
    }
    if matches!(command, DeviceCommand::Rate { hz } if *hz != 20)
        && read_registers(connection, 0x0e, None, cancel).await?[0] == 0xe4
    {
        return Err(Error::Invalid(
            "switch to a short output profile before leaving 20 Hz".into(),
        ));
    }

    if matches!(
        command,
        DeviceCommand::Output { .. } | DeviceCommand::ResetDefaults
    ) {
        let current = read_registers(connection, 0x0e, None, cancel).await?[0];
        if !matches!(current, 0x61 | 0x81 | 0x84 | 0xa4 | 0xe4) {
            return Err(Error::Protocol(format!(
                "operation requires verified new-format firmware (0x0E=0x{current:04x})"
            )));
        }
    }
    if matches!(command, DeviceCommand::ZeroYaw)
        && read_registers(connection, 0x24, None, cancel).await?[0] != 1
    {
        return Err(Error::Invalid(
            "zero-yaw requires six-axis mode; switch algorithm explicitly first".into(),
        ));
    }
    transport::cancel_after(
        cancel,
        Duration::from_secs(2),
        connection.write(&protocol::UNLOCK),
    )
    .await?;
    // Vendor magnetic-calibration instructions specify 200 ms after unlock.
    delay(cancel, 200).await?;
    let reference = !matches!(
        command,
        DeviceCommand::Rate { .. } | DeviceCommand::Output { .. } | DeviceCommand::Save
    );
    progress(Progress::WriteAttempt {
        reference,
        persistent: matches!(command, DeviceCommand::Save | DeviceCommand::ResetDefaults),
    });
    transport::cancel_after(
        cancel,
        Duration::from_secs(2),
        connection.write(&protocol::write_register(address, value)),
    )
    .await?;
    progress(Progress::CommandSent);
    if matches!(command, DeviceCommand::AngleReference) {
        delay(cancel, 100).await?;
        progress(Progress::WriteAttempt {
            reference: true,
            persistent: true,
        });
        transport::cancel_after(
            cancel,
            Duration::from_secs(2),
            connection.write(&protocol::write_register(0, 0)),
        )
        .await?;
        return Ok((OperationOutcome::Unverified, "angle-reference and SAVE sent; reference effect and power-cycle persistence unverified".into(), None));
    }
    if matches!(command, DeviceCommand::Save | DeviceCommand::ZeroYaw) {
        return Ok((
            OperationOutcome::Unverified,
            "command sent; effect/persistence requires independent verification".into(),
            None,
        ));
    }
    if matches!(command, DeviceCommand::ResetDefaults) {
        delay(cancel, 100).await?;
        read_registers(connection, 0x03, Some(6), cancel).await?;
        read_registers(connection, 0x0e, Some(0x61), cancel).await?;
        read_registers(connection, 0x24, Some(0), cancel).await?;
        let observation = inspect(connection, cancel).await?;
        if observation.rate_register != Some(6)
            || observation.output_register != Some(0x61)
            || observation.algorithm_register != Some(0)
        {
            return Err(Error::Protocol(
                "default configuration changed during verification".into(),
            ));
        }
        progress(Progress::RegisterVerified);
        return Ok((OperationOutcome::Succeeded, "known defaults read back; calibration coefficients and power-cycle persistence unverified".into(), Some(observation)));
    }
    if matches!(command, DeviceCommand::AccelCalibrate) {
        let observed = read_registers(connection, 1, None, cancel).await?[0];
        if observed != 1 {
            return Ok((OperationOutcome::Unverified, "calibration command sent; start was not observed, completion and accuracy unverified".into(), None));
        }
        progress(Progress::RegisterVerified);
        let completion = async {
            loop {
                delay(cancel, 100).await?;
                if read_registers(connection, 1, None, cancel).await?[0] == 0 {
                    return Ok::<(), Error>(());
                }
            }
        };
        tokio::time::timeout(Duration::from_secs(8), completion)
            .await
            .map_err(|_| Error::Timeout("calibration completion".into()))??;
        progress(Progress::CompletionObserved);
        return Ok((
            OperationOutcome::Succeeded,
            "CALSW start and completion observed; accuracy unverified, no SAVE sent".into(),
            None,
        ));
    }
    delay(cancel, 100).await?;
    read_registers(connection, address as u16, Some(value), cancel).await?;
    progress(Progress::RegisterVerified);
    if matches!(command, DeviceCommand::MagStop) {
        progress(Progress::CompletionObserved);
    }
    Ok((
        OperationOutcome::Succeeded,
        format!(
            "register 0x{address:02x} readback verified: {value}; accuracy/persistence not inferred; no SAVE sent"
        ),
        None,
    ))
}
