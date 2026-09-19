use crate::{DeviceInfo, Error, Result, Source, TransportKind};
use btleplug::api::{
    Central, CentralState, CharPropFlags, Characteristic, Manager as _, Peripheral as _,
    ScanFilter, ValueNotification, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures_util::{Stream, StreamExt};
use std::future::Future;
use std::pin::Pin;
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::watch;
use tokio_serial::{
    DataBits, FlowControl, Parity, SerialPortBuilderExt, SerialPortType, SerialStream, StopBits,
};
use uuid::Uuid;

const SERVICE: Uuid = Uuid::from_u128(0x0000ffe500001000800000805f9a34fb);
const NOTIFY: Uuid = Uuid::from_u128(0x0000ffe400001000800000805f9a34fb);
const WRITE: Uuid = Uuid::from_u128(0x0000ffe900001000800000805f9a34fb);

fn ble_error(e: btleplug::Error) -> Error {
    let text = e.to_string();
    if matches!(e, btleplug::Error::PermissionDenied) {
        Error::Permission(text)
    } else if matches!(e, btleplug::Error::NotSupported(_)) {
        Error::Protocol(text)
    } else {
        Error::Unavailable(text)
    }
}

pub(crate) async fn cancel_after<T>(
    cancel: &mut watch::Receiver<bool>,
    timeout: Duration,
    work: impl Future<Output = Result<T>>,
) -> Result<T> {
    if *cancel.borrow() {
        return Err(Error::Cancelled);
    }
    tokio::select! {
        _ = cancel.changed() => Err(Error::Cancelled),
        result = tokio::time::timeout(timeout, work) => result.map_err(|_| Error::Timeout("device operation".into()))?,
    }
}

async fn adapter() -> Result<Adapter> {
    let manager = Manager::new().await.map_err(ble_error)?;
    let adapters = manager.adapters().await.map_err(ble_error)?;
    for adapter in adapters {
        if adapter.adapter_state().await.map_err(ble_error)? == CentralState::PoweredOn {
            return Ok(adapter);
        }
    }
    Err(Error::Unavailable(
        "no powered-on BLE adapter; check Bluetooth permission and power".into(),
    ))
}

pub(crate) async fn scan(
    kind: TransportKind,
    duration: Duration,
    cancel: &mut watch::Receiver<bool>,
) -> Result<Vec<DeviceInfo>> {
    if kind == TransportKind::Usb {
        return tokio_serial::available_ports()
            .map_err(|e| Error::Unavailable(e.to_string()))?
            .into_iter()
            .filter(|p| matches!(p.port_type, SerialPortType::UsbPort(_)))
            .filter(|p| !cfg!(target_os = "macos") || !p.port_name.starts_with("/dev/tty."))
            .map(|p| {
                let (name, vid, pid) = match p.port_type {
                    SerialPortType::UsbPort(info) => (
                        info.product.unwrap_or_else(|| "USB serial".into()),
                        Some(info.vid),
                        Some(info.pid),
                    ),
                    _ => ("Serial port".into(), None, None),
                };
                Ok(DeviceInfo {
                    transport: kind,
                    id: p.port_name,
                    name,
                    rssi: None,
                    usb_vid: vid,
                    usb_pid: pid,
                })
            })
            .collect();
    }
    let adapter = cancel_after(cancel, Duration::from_secs(10), adapter()).await?;
    adapter
        .start_scan(ScanFilter::default())
        .await
        .map_err(ble_error)?;
    let result = cancel_after(cancel, duration + Duration::from_secs(1), async {
        tokio::time::sleep(duration).await;
        let mut found = Vec::new();
        for peripheral in adapter.peripherals().await.map_err(ble_error)? {
            if let Some(props) = peripheral.properties().await.map_err(ble_error)? {
                let name = props.local_name.unwrap_or_default();
                // Some WIT firmware omits the service UUID in advertisements. Verify GATT on connection.
                if !name.to_ascii_uppercase().starts_with("WT")
                    && !props.services.contains(&SERVICE)
                {
                    continue;
                }
                found.push(DeviceInfo {
                    transport: kind,
                    id: peripheral.id().to_string(),
                    name,
                    rssi: props.rssi,
                    usb_vid: None,
                    usb_pid: None,
                });
            }
        }
        found.sort_by(|a, b| a.id.cmp(&b.id));
        Ok(found)
    })
    .await;
    let _ = tokio::time::timeout(Duration::from_secs(2), adapter.stop_scan()).await;
    result
}

pub(crate) enum Connection {
    Usb(SerialStream),
    Ble {
        peripheral: Peripheral,
        notify: Characteristic,
        writer: Option<Characteristic>,
        stream: Pin<Box<dyn Stream<Item = ValueNotification> + Send>>,
        link: Option<crate::ble_link::Link>,
    },
}

impl Connection {
    pub(crate) async fn open(source: &Source, cancel: &mut watch::Receiver<bool>) -> Result<Self> {
        match source {
            Source::Usb { port, baud } => {
                let stream = tokio_serial::new(port, *baud)
                    .data_bits(DataBits::Eight)
                    .parity(Parity::None)
                    .stop_bits(StopBits::One)
                    .flow_control(FlowControl::None)
                    .open_native_async()
                    .map_err(|e| match e.kind {
                        tokio_serial::ErrorKind::Io(std::io::ErrorKind::PermissionDenied) => {
                            Error::Permission(format!("{port}: {e}"))
                        }
                        tokio_serial::ErrorKind::InvalidInput => {
                            Error::Invalid(format!("{port}: {e}"))
                        }
                        _ => Error::Unavailable(format!("{port}: {e}")),
                    })?;
                // Drop restores OS ownership; never send probe/configuration bytes on open.
                Ok(Self::Usb(stream))
            }
            Source::Ble {
                device_id,
                connection_mode,
            } => {
                let adapter = cancel_after(cancel, Duration::from_secs(10), adapter()).await?;
                adapter
                    .start_scan(ScanFilter::default())
                    .await
                    .map_err(ble_error)?;
                let result = cancel_after(cancel, Duration::from_secs(15), async {
                    loop {
                        for peripheral in adapter.peripherals().await.map_err(ble_error)? {
                            if peripheral.id().to_string() == *device_id {
                                return Ok(peripheral);
                            }
                        }
                        tokio::time::sleep(Duration::from_millis(150)).await;
                    }
                })
                .await;
                let _ = tokio::time::timeout(Duration::from_secs(2), adapter.stop_scan()).await;
                let peripheral = result?;
                let setup = cancel_after(cancel, Duration::from_secs(15), async {
                    peripheral.connect().await.map_err(ble_error)?;
                    peripheral.discover_services().await.map_err(ble_error)?;
                    let chars = peripheral.characteristics();
                    let notify = chars
                        .iter()
                        .find(|c| {
                            c.service_uuid == SERVICE
                                && c.uuid == NOTIFY
                                && c.properties
                                    .intersects(CharPropFlags::NOTIFY | CharPropFlags::INDICATE)
                        })
                        .cloned()
                        .ok_or_else(|| {
                            Error::Protocol(
                                "WIT FFE5/FFE4 notification characteristic missing".into(),
                            )
                        })?;
                    let writer = chars
                        .iter()
                        .find(|c| {
                            c.service_uuid == SERVICE
                                && c.uuid == WRITE
                                && c.properties.intersects(
                                    CharPropFlags::WRITE | CharPropFlags::WRITE_WITHOUT_RESPONSE,
                                )
                        })
                        .cloned();
                    let stream = peripheral.notifications().await.map_err(ble_error)?;
                    peripheral.subscribe(&notify).await.map_err(ble_error)?;
                    let link = crate::ble_link::Link::open(&peripheral, *connection_mode).await?;
                    Ok((notify, writer, stream, link))
                })
                .await;
                match setup {
                    Ok((notify, writer, stream, link)) => Ok(Self::Ble {
                        peripheral,
                        notify,
                        writer,
                        stream,
                        link: Some(link),
                    }),
                    Err(e) => {
                        let _ =
                            tokio::time::timeout(Duration::from_secs(2), peripheral.disconnect())
                                .await;
                        Err(e)
                    }
                }
            }
            Source::Simulate { .. } => Err(Error::Invalid(
                "simulator is not a physical connection".into(),
            )),
        }
    }

    pub(crate) async fn read(&mut self) -> Result<Vec<u8>> {
        match self {
            Self::Usb(stream) => {
                let mut buffer = [0u8; 1024];
                let n = stream.read(&mut buffer).await?;
                if n == 0 {
                    return Err(Error::Unavailable("USB stream closed".into()));
                }
                Ok(buffer[..n].to_vec())
            }
            Self::Ble { stream, notify, .. } => loop {
                let value = stream
                    .next()
                    .await
                    .ok_or_else(|| Error::Unavailable("BLE notification stream closed".into()))?;
                if value.uuid == notify.uuid {
                    return Ok(value.value);
                }
            },
        }
    }

    pub(crate) async fn write(&mut self, bytes: &[u8]) -> Result<()> {
        match self {
            Self::Usb(stream) => {
                stream.write_all(bytes).await?;
                stream.flush().await?;
                Ok(())
            }
            Self::Ble {
                peripheral, writer, ..
            } => {
                let writer = writer.as_ref().ok_or_else(|| {
                    Error::Protocol("WIT FFE9 writable characteristic missing".into())
                })?;
                let mode = if writer.properties.contains(CharPropFlags::WRITE) {
                    WriteType::WithResponse
                } else {
                    WriteType::WithoutResponse
                };
                peripheral
                    .write(writer, bytes, mode)
                    .await
                    .map_err(ble_error)
            }
        }
    }

    pub(crate) fn link_status(&self) -> Option<crate::BleLinkStatus> {
        match self {
            Self::Ble { link, .. } => link.as_ref().map(crate::ble_link::Link::status),
            Self::Usb(_) => None,
        }
    }

    pub(crate) async fn close(&mut self) {
        if let Self::Ble {
            peripheral,
            notify,
            link,
            ..
        } = self
        {
            // Release any native connection preference before disconnecting.
            *link = None;
            let _ =
                tokio::time::timeout(Duration::from_secs(1), peripheral.unsubscribe(notify)).await;
            let _ = tokio::time::timeout(Duration::from_secs(1), peripheral.disconnect()).await;
        }
    }
}
