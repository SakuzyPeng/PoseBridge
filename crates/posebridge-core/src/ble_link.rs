//! Optional OS connection preference. Never writes WIT registers or saves device settings.

use crate::{BleConnectionMode, BleLinkStatus, Result};
use btleplug::platform::Peripheral;

#[cfg(target_os = "windows")]
mod platform {
    use super::*;
    use crate::Error;
    use btleplug::api::Peripheral as _;
    use windows::Devices::Bluetooth::{
        BluetoothLEDevice, BluetoothLEPreferredConnectionParameters,
        BluetoothLEPreferredConnectionParametersRequest,
    };

    pub(crate) struct Link {
        device: Option<BluetoothLEDevice>,
        request: Option<BluetoothLEPreferredConnectionParametersRequest>,
        mode: BleConnectionMode,
        note: Option<String>,
    }

    impl Link {
        pub(crate) async fn open(peripheral: &Peripheral, mode: BleConnectionMode) -> Result<Self> {
            let mut link = Self {
                device: None,
                request: None,
                mode,
                note: None,
            };
            let setup: windows::core::Result<()> = async {
                let device =
                    BluetoothLEDevice::FromBluetoothAddressAsync(peripheral.address().into())?
                        .await?;
                link.device = Some(device);
                if mode == BleConnectionMode::Throughput {
                    let preference =
                        BluetoothLEPreferredConnectionParameters::ThroughputOptimized()?;
                    link.request = Some(
                        link.device
                            .as_ref()
                            .unwrap()
                            .RequestPreferredConnectionParameters(&preference)?,
                    );
                }
                Ok(())
            }
            .await;
            if let Err(error) = setup {
                if mode == BleConnectionMode::Throughput {
                    return Err(Error::Unavailable(format!(
                        "Windows BLE throughput request failed: {error}; use default mode to connect without this preference"
                    )));
                }
                // Older Windows versions may not expose connection diagnostics. Default
                // acquisition must continue to work without this optional interface.
                link.note = Some(format!(
                    "Windows BLE connection diagnostics unavailable: {error}"
                ));
            }
            Ok(link)
        }

        pub(crate) fn status(&self) -> BleLinkStatus {
            let mut status = BleLinkStatus {
                requested_mode: self.mode,
                note: self.note.clone(),
                ..BleLinkStatus::default()
            };
            if let Some(request) = &self.request {
                match request.Status() {
                    Ok(value) => {
                        status.request_status = Some(
                            match value.0 {
                                0 => "unspecified",
                                1 => "success",
                                2 => "device_not_available",
                                3 => "access_denied",
                                _ => "unknown",
                            }
                            .into(),
                        )
                    }
                    Err(error) => status.note = Some(error.to_string()),
                }
            }
            if let Some(device) = &self.device {
                let parameters = (|| -> windows::core::Result<_> {
                    let parameters = device.GetConnectionParameters()?;
                    Ok((
                        parameters.ConnectionInterval()?,
                        parameters.ConnectionLatency()?,
                    ))
                })();
                match parameters {
                    Ok((interval, latency)) if interval != 0 => {
                        // Windows exposes the Bluetooth units: 1.25 ms per interval unit.
                        status.connection_interval_ms = Some(interval as f64 * 1.25);
                        status.peripheral_latency = Some(latency);
                    }
                    Ok(_) => {
                        status.note = Some("connection parameters are not available yet".into())
                    }
                    Err(error) => status.note = Some(error.to_string()),
                }
            }
            status
        }
    }

    impl Drop for Link {
        fn drop(&mut self) {
            if let Some(request) = self.request.take() {
                let _ = request.Close();
            }
            if let Some(device) = self.device.take() {
                let _ = device.Close();
            }
        }
    }
}

#[cfg(not(target_os = "windows"))]
mod platform {
    use super::*;

    pub(crate) struct Link;

    impl Link {
        pub(crate) async fn open(_: &Peripheral, mode: BleConnectionMode) -> Result<Self> {
            if mode != BleConnectionMode::Default {
                return Err(crate::Error::Invalid(
                    "BLE throughput preference requires Windows 11 or later".into(),
                ));
            }
            Ok(Self)
        }

        pub(crate) fn status(&self) -> BleLinkStatus {
            BleLinkStatus {
                note: Some("connection parameters are managed by the OS; this backend does not expose them".into()),
                ..BleLinkStatus::default()
            }
        }
    }
}

pub(crate) use platform::Link;
