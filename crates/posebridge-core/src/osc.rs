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
    interval: Duration,
    next_slot: Option<Instant>,
    pending_deadline: Option<Instant>,
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
        let interval = Duration::from_secs_f64(1.0 / config.max_rate_hz as f64);
        Ok(Self {
            socket,
            config,
            last_key: None,
            interval,
            next_slot: None,
            pending_deadline: None,
        })
    }

    pub(crate) fn deadline(&self) -> Option<Instant> {
        self.pending_deadline
    }

    pub(crate) fn send_if_due(&mut self, pose: &PoseSnapshot, now: Instant) -> Result<bool> {
        let key = (pose.session_id, pose.sequence);
        if !pose.fresh || self.last_key == Some(key) {
            self.pending_deadline = None;
            return Ok(false);
        }
        if let Some(deadline) = self.next_slot.filter(|deadline| *deadline > now) {
            self.pending_deadline = Some(deadline);
            return Ok(false);
        }
        let packet = encode(pose, self.config.format)?;
        match self.socket.send(&packet) {
            Ok(n) if n == packet.len() => {
                self.last_key = Some(key);
                // Keep the cadence anchored across small timer delays instead of adding
                // the delay to every period. After an idle/blocked period, start afresh:
                // missed slots never accumulate credit for a burst of historical poses.
                self.next_slot = Some(
                    self.next_slot
                        .map(|slot| slot + self.interval)
                        .filter(|slot| *slot > now)
                        .unwrap_or(now + self.interval),
                );
                self.pending_deadline = None;
                Ok(true)
            }
            Ok(_) => Err(Error::Io("partial UDP datagram send".into())),
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::ConnectionRefused
                ) =>
            {
                // A nonblocking send failure must not spin on an expired deadline.
                self.next_slot = Some(now + self.interval);
                self.pending_deadline = self.next_slot;
                Ok(false)
            }
            Err(e) => Err(e.into()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RawData;

    fn fixture(rate: u32) -> (Sender, UdpSocket, PoseSnapshot) {
        let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
        receiver
            .set_read_timeout(Some(Duration::from_millis(250)))
            .unwrap();
        let sender = Sender::new(OscConfig {
            target: receiver.local_addr().unwrap(),
            max_rate_hz: rate,
            format: OscFormat::Euler,
        })
        .unwrap();
        let pose = PoseSnapshot {
            session_id: 1,
            sequence: 1,
            received_ns: 0,
            quaternion_xyzw: [0., 0., 0., 1.],
            euler_deg: [0.; 3],
            raw: RawData::default(),
            fresh: true,
        };
        (sender, receiver, pose)
    }

    fn receive(receiver: &UdpSocket) -> Vec<OscType> {
        let mut data = [0u8; 512];
        let n = receiver.recv(&mut data).unwrap();
        let (_, OscPacket::Message(message)) = rosc::decoder::decode_udp(&data[..n]).unwrap()
        else {
            panic!("expected one complete OSC message");
        };
        message.args
    }

    #[test]
    fn timer_jitter_does_not_halve_200_hz_output() {
        let (mut sender, receiver, mut pose) = fixture(200);
        let start = Instant::now();
        // Slightly late wakeups followed by less-late ones used to skip entire ticks.
        for index in 0..1000 {
            let jitter_us = [0, 800, 500, 400][index % 4];
            let now = start + Duration::from_micros(index as u64 * 5000 + jitter_us);
            pose.sequence = index as u64 + 1;
            assert!(sender.send_if_due(&pose, now).unwrap(), "sample {index}");
            receive(&receiver);
            assert!(sender.deadline().is_none());
        }
    }

    #[test]
    fn pending_slot_sends_latest_once_and_idle_earns_no_burst_credit() {
        let (mut sender, receiver, mut pose) = fixture(100);
        let start = Instant::now();
        assert!(sender.send_if_due(&pose, start).unwrap());
        receive(&receiver);
        for sequence in 2..=10 {
            pose.sequence = sequence;
            pose.euler_deg[0] = sequence as f64;
            assert!(
                !sender
                    .send_if_due(&pose, start + Duration::from_micros(sequence * 100))
                    .unwrap()
            );
        }
        assert_eq!(sender.deadline(), Some(start + Duration::from_millis(10)));
        assert!(
            sender
                .send_if_due(&pose, start + Duration::from_millis(11))
                .unwrap()
        );
        assert_eq!(receive(&receiver)[0], OscType::Float(10.));
        assert!(
            !sender
                .send_if_due(&pose, start + Duration::from_secs(1))
                .unwrap()
        );
        assert!(sender.deadline().is_none());

        pose.sequence += 1;
        assert!(
            sender
                .send_if_due(&pose, start + Duration::from_secs(1))
                .unwrap()
        );
        receive(&receiver);
        pose.sequence += 1;
        assert!(
            !sender
                .send_if_due(&pose, start + Duration::from_millis(1001))
                .unwrap()
        );
        assert_eq!(sender.deadline(), Some(start + Duration::from_millis(1010)));
        let mut data = [0; 512];
        receiver.set_nonblocking(true).unwrap();
        assert!(receiver.recv(&mut data).is_err());
    }

    #[test]
    fn stale_pending_pose_is_discarded_and_new_session_can_resume() {
        let (mut sender, receiver, mut pose) = fixture(1);
        let start = Instant::now();
        assert!(sender.send_if_due(&pose, start).unwrap());
        receive(&receiver);
        pose.sequence += 1;
        assert!(
            !sender
                .send_if_due(&pose, start + Duration::from_millis(5))
                .unwrap()
        );
        pose.fresh = false;
        assert!(
            !sender
                .send_if_due(&pose, start + Duration::from_secs(1))
                .unwrap()
        );
        assert!(sender.deadline().is_none());
        pose.session_id += 1;
        pose.sequence = 1;
        pose.fresh = true;
        assert!(
            sender
                .send_if_due(&pose, start + Duration::from_secs(2))
                .unwrap()
        );
        receive(&receiver);
    }
}
