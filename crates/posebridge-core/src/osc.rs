use crate::{Error, OscConfig, OscFormat, PoseSnapshot, Result};
use rosc::{OscMessage, OscPacket, OscType};
use std::net::UdpSocket;
use std::time::{Duration, Instant};

pub const QUATERNION_ADDRESS: &str = "/posebridge/v1/quaternion";
pub const EULER_ADDRESS: &str = "/posebridge/v1/euler";

pub fn encode(pose: &PoseSnapshot, format: OscFormat) -> Result<Vec<u8>> {
    let (address, values) = match format {
        OscFormat::Quaternion => (QUATERNION_ADDRESS, pose.quaternion_xyzw.to_vec()),
        OscFormat::Euler => (EULER_ADDRESS, pose.euler_deg.to_vec()),
    };
    let args = values
        .into_iter()
        .map(|v| {
            let v = v as f32;
            if v.is_finite() {
                Ok(OscType::Float(v))
            } else {
                Err(Error::Invalid("OSC value is not a finite float32".into()))
            }
        })
        .collect::<Result<_>>()?;
    rosc::encoder::encode(&OscPacket::Message(OscMessage {
        addr: address.into(),
        args,
    }))
    .map_err(|e| Error::Protocol(e.to_string()))
}

pub(crate) struct Sender {
    socket: UdpSocket,
    config: OscConfig,
    last_key: Option<(u64, u64)>,
    last_sent: Option<Instant>,
}

impl Sender {
    pub(crate) fn new(config: OscConfig) -> Result<Self> {
        crate::model::validate_rate(config.max_rate_hz)?;
        if !config.target.ip().is_loopback() || config.target.port() == 0 {
            return Err(Error::Invalid(
                "OSC target must be loopback with a nonzero port".into(),
            ));
        }
        let socket = UdpSocket::bind(if config.target.is_ipv4() {
            "127.0.0.1:0"
        } else {
            "[::1]:0"
        })?;
        socket.connect(config.target)?;
        socket.set_nonblocking(true)?;
        Ok(Self {
            socket,
            config,
            last_key: None,
            last_sent: None,
        })
    }

    pub(crate) fn send_if_due(&mut self, pose: &PoseSnapshot, now: Instant) -> Result<bool> {
        let key = (pose.session_id, pose.sequence);
        let interval = Duration::from_secs_f64(1.0 / self.config.max_rate_hz as f64);
        if !pose.fresh
            || self.last_key == Some(key)
            || self
                .last_sent
                .is_some_and(|last| now.duration_since(last) < interval)
        {
            return Ok(false);
        }
        let packet = encode(pose, self.config.format)?;
        match self.socket.send(&packet) {
            Ok(n) if n == packet.len() => {
                self.last_key = Some(key);
                self.last_sent = Some(now);
                Ok(true)
            }
            Ok(_) => Err(Error::Io("partial UDP datagram send".into())),
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::ConnectionRefused
                ) =>
            {
                Ok(false)
            }
            Err(e) => Err(e.into()),
        }
    }
}
