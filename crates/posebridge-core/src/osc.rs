//! Single current PoseBridge protocol. OSC versions 1/2 are intentionally removed.
use crate::{Error, OscConfig, OscFormat, PROTOCOL_VERSION, PoseSnapshot, Result, Snapshot};
use rosc::{OscMessage, OscPacket, OscType};
use serde_json::json;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

pub const QUATERNION_ADDRESS: &str = "/posebridge/quaternion";
pub const EULER_ADDRESS: &str = "/posebridge/euler";
pub const INFO_ADDRESS: &str = "/posebridge/info";
pub const STATUS_ADDRESS: &str = "/posebridge/status";
pub const MAX_PACKET_BYTES: usize = 8192;

fn message(address: &str, args: Vec<OscType>) -> Result<Vec<u8>> {
    let packet = rosc::encoder::encode(&OscPacket::Message(OscMessage {
        addr: address.into(),
        args,
    }))
    .map_err(|e| Error::Protocol(e.to_string()))?;
    if packet.len() > MAX_PACKET_BYTES {
        return Err(Error::Invalid("OSC packet exceeds 8 KiB".into()));
    }
    Ok(packet)
}

pub fn encode(
    pose: &PoseSnapshot,
    format: OscFormat,
    source_id: &str,
    tx_seq: u64,
    age_at_send_ns: u64,
) -> Result<Vec<u8>> {
    crate::validate_source_id(source_id)?;
    let long = |n| {
        i64::try_from(n)
            .map(OscType::Long)
            .map_err(|_| Error::Invalid("OSC metadata exceeds signed int64".into()))
    };
    if [
        pose.instance_id,
        pose.session_id,
        pose.sequence,
        tx_seq,
        pose.reference_epoch,
        pose.metadata_revision,
    ]
    .contains(&0)
        || pose.sample_time.is_some_and(|t| t.clock_epoch == 0)
    {
        return Err(Error::Invalid(
            "OSC identity, sequence and epoch fields must be positive".into(),
        ));
    }
    let mut args = vec![
        OscType::Int(PROTOCOL_VERSION as i32),
        OscType::String(source_id.into()),
        long(pose.instance_id)?,
        long(pose.session_id)?,
        long(pose.sequence)?,
        long(tx_seq)?,
        long(pose.reference_epoch)?,
        long(pose.metadata_revision)?,
        long(pose.received_ns)?,
        long(age_at_send_ns)?,
        OscType::Int(pose.sample_time.map_or(0, |t| t.kind as i32)),
        long(pose.sample_time.map_or(0, |t| t.time_ms))?,
        long(pose.sample_time.map_or(0, |t| t.clock_epoch))?,
    ];
    let (address, values) = match format {
        OscFormat::Quaternion => (QUATERNION_ADDRESS, pose.quaternion_xyzw.to_vec()),
        OscFormat::Euler => (EULER_ADDRESS, pose.euler_deg.to_vec()),
    };
    for value in values {
        let value = value as f32;
        if !value.is_finite() {
            return Err(Error::Invalid("OSC pose must be finite float32".into()));
        }
        args.push(OscType::Float(value));
    }
    message(address, args)
}

fn socket(config: &OscConfig) -> Result<UdpSocket> {
    crate::validate_rate(config.max_rate_hz)?;
    if !config.target.ip().is_loopback() || config.target.port() == 0 {
        return Err(Error::Invalid(
            "OSC target must be loopback with nonzero port".into(),
        ));
    }
    let socket = UdpSocket::bind(if config.target.is_ipv4() {
        "127.0.0.1:0"
    } else {
        "[::1]:0"
    })?;
    socket.connect(config.target)?;
    socket.set_nonblocking(true)?;
    Ok(socket)
}
fn send(socket: &UdpSocket, packet: &[u8]) -> Result<bool> {
    match socket.send(packet) {
        Ok(n) if n == packet.len() => Ok(true),
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

pub(crate) struct Sender {
    socket: UdpSocket,
    config: OscConfig,
    last_key: Option<(u64, u64)>,
    interval: Duration,
    next_slot: Option<Instant>,
    pending_deadline: Option<Instant>,
    tx_seq: u64,
    coalesced: u64,
    errors: u64,
}
impl Sender {
    pub(crate) fn new(config: OscConfig) -> Result<Self> {
        Ok(Self {
            socket: socket(&config)?,
            interval: Duration::from_secs_f64(1.0 / config.max_rate_hz as f64),
            config,
            last_key: None,
            next_slot: None,
            pending_deadline: None,
            tx_seq: 0,
            coalesced: 0,
            errors: 0,
        })
    }
    pub(crate) fn deadline(&self) -> Option<Instant> {
        self.pending_deadline
    }
    pub(crate) fn clear_pending(&mut self) {
        self.pending_deadline = None;
    }
    pub(crate) fn coalesced_samples(&self) -> u64 {
        self.coalesced
    }
    pub(crate) fn send_errors(&self) -> u64 {
        self.errors
    }
    pub(crate) fn send_if_due(
        &mut self,
        pose: &PoseSnapshot,
        source_id: &str,
        age_ns: u64,
        now: Instant,
    ) -> Result<bool> {
        let key = (pose.session_id, pose.sequence);
        if !pose.fresh || self.last_key == Some(key) {
            self.clear_pending();
            return Ok(false);
        }
        if let Some(deadline) = self.next_slot.filter(|v| *v > now) {
            self.pending_deadline = Some(deadline);
            return Ok(false);
        }
        let packet = encode(pose, self.config.format, source_id, self.tx_seq + 1, age_ns)?;
        match send(&self.socket, &packet) {
            Ok(true) => {
                self.tx_seq += 1;
                let previous = self.last_key.filter(|v| v.0 == key.0).map_or(0, |v| v.1);
                self.coalesced += key.1.saturating_sub(previous).saturating_sub(1);
                self.last_key = Some(key);
                self.next_slot = Some(
                    self.next_slot
                        .map(|v| v + self.interval)
                        .filter(|v| *v > now)
                        .unwrap_or(now + self.interval),
                );
                self.clear_pending();
                Ok(true)
            }
            result => {
                self.errors += 1;
                self.next_slot = Some(now + self.interval);
                self.pending_deadline = self.next_slot;
                result
            }
        }
    }
}

pub(crate) struct Telemetry {
    socket: UdpSocket,
    info_seq: u64,
    status_seq: u64,
    next_info: Option<Instant>,
    next_status: Option<Instant>,
    revision: u64,
    last_state: Option<crate::ConnectionState>,
    errors: u64,
}
impl Telemetry {
    pub(crate) fn new(config: &OscConfig) -> Result<Self> {
        Ok(Self {
            socket: socket(config)?,
            info_seq: 0,
            status_seq: 0,
            next_info: None,
            next_status: None,
            revision: 0,
            last_state: None,
            errors: 0,
        })
    }
    pub(crate) fn send_errors(&self) -> u64 {
        self.errors
    }
    pub(crate) fn refresh(
        &mut self,
        snapshot: &Snapshot,
        force: bool,
        now: Instant,
    ) -> Result<u64> {
        let d = &snapshot.descriptor;
        let mut sent = 0;
        for info in [true, false] {
            let due = if info {
                self.next_info.is_none_or(|v| now >= v) || self.revision != d.metadata_revision
            } else {
                self.next_status.is_none_or(|v| now >= v)
                    || self.last_state != Some(snapshot.status.state)
                    || self.revision != d.metadata_revision
            };
            if !due && !force {
                continue;
            }
            let sequence = if info {
                self.info_seq + 1
            } else {
                self.status_seq + 1
            };
            let mut value = json!({"schema":PROTOCOL_VERSION,"kind":if info {"info"} else {"status"},
                "source_id":d.source_id,"instance_id":d.instance_id.to_string(),"session_id":d.session_id.to_string(),
                "metadata_revision":d.metadata_revision.to_string(),"reference_epoch":d.reference_epoch.to_string(),"message_seq":sequence.to_string()});
            if info {
                value["descriptor"] =
                    serde_json::to_value(d).map_err(|e| Error::Internal(e.to_string()))?;
                self.next_info = Some(now + Duration::from_secs(5));
            } else {
                value["status"] = serde_json::to_value(&snapshot.status)
                    .map_err(|e| Error::Internal(e.to_string()))?;
                self.next_status = Some(now + Duration::from_secs(1));
            }
            let packet = message(
                if info { INFO_ADDRESS } else { STATUS_ADDRESS },
                vec![OscType::String(value.to_string())],
            )?;
            let delivered = match send(&self.socket, &packet) {
                Ok(value) => value,
                Err(error) => {
                    self.errors += 1;
                    return Err(error);
                }
            };
            if delivered {
                sent += 1;
                if info {
                    self.info_seq = sequence;
                } else {
                    self.status_seq = sequence;
                }
            } else {
                self.errors += 1;
            }
        }
        self.revision = d.metadata_revision;
        self.last_state = Some(snapshot.status.state);
        Ok(sent)
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
            instance_id: 1,
            reference_epoch: 1,
            metadata_revision: 1,
            session_id: 1,
            sequence: 1,
            received_ns: 0,
            sample_time: None,
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
        message.args.into_iter().skip(13).collect()
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
            assert!(
                sender.send_if_due(&pose, "test", 0, now).unwrap(),
                "sample {index}"
            );
            receive(&receiver);
            assert!(sender.deadline().is_none());
        }
    }

    #[test]
    fn pending_slot_sends_latest_once_and_idle_earns_no_burst_credit() {
        let (mut sender, receiver, mut pose) = fixture(100);
        let start = Instant::now();
        assert!(sender.send_if_due(&pose, "test", 0, start).unwrap());
        receive(&receiver);
        for sequence in 2..=10 {
            pose.sequence = sequence;
            pose.euler_deg[0] = sequence as f64;
            assert!(
                !sender
                    .send_if_due(
                        &pose,
                        "test",
                        0,
                        start + Duration::from_micros(sequence * 100)
                    )
                    .unwrap()
            );
        }
        assert_eq!(sender.deadline(), Some(start + Duration::from_millis(10)));
        assert!(
            sender
                .send_if_due(&pose, "test", 0, start + Duration::from_millis(11))
                .unwrap()
        );
        assert_eq!(receive(&receiver)[0], OscType::Float(10.));
        assert!(
            !sender
                .send_if_due(&pose, "test", 0, start + Duration::from_secs(1))
                .unwrap()
        );
        assert!(sender.deadline().is_none());

        pose.sequence += 1;
        assert!(
            sender
                .send_if_due(&pose, "test", 0, start + Duration::from_secs(1))
                .unwrap()
        );
        receive(&receiver);
        pose.sequence += 1;
        assert!(
            !sender
                .send_if_due(&pose, "test", 0, start + Duration::from_millis(1001))
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
        assert!(sender.send_if_due(&pose, "test", 0, start).unwrap());
        receive(&receiver);
        pose.sequence += 1;
        assert!(
            !sender
                .send_if_due(&pose, "test", 0, start + Duration::from_millis(5))
                .unwrap()
        );
        pose.fresh = false;
        assert!(
            !sender
                .send_if_due(&pose, "test", 0, start + Duration::from_secs(1))
                .unwrap()
        );
        assert!(sender.deadline().is_none());
        pose.session_id += 1;
        pose.sequence = 1;
        pose.fresh = true;
        assert!(
            sender
                .send_if_due(&pose, "test", 0, start + Duration::from_secs(2))
                .unwrap()
        );
        receive(&receiver);
    }
}

#[cfg(test)]
mod telemetry_tests {
    use super::*;
    #[test]
    fn state_changes_periodic_info_and_no_pose_heartbeat() {
        let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
        receiver
            .set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        let config = OscConfig {
            target: receiver.local_addr().unwrap(),
            max_rate_hz: 100,
            format: OscFormat::Quaternion,
        };
        let mut telemetry = Telemetry::new(&config).unwrap();
        let mut s = crate::Controller::new().unwrap().snapshot();
        s.descriptor.instance_id = 1;
        let start = Instant::now();
        assert_eq!(telemetry.refresh(&s, false, start).unwrap(), 2);
        let receive = || {
            let mut b = [0u8; 8192];
            let n = receiver.recv(&mut b).unwrap();
            let (_, OscPacket::Message(m)) = rosc::decoder::decode_udp(&b[..n]).unwrap() else {
                panic!("message")
            };
            m
        };
        for address in [INFO_ADDRESS, STATUS_ADDRESS] {
            let m = receive();
            assert_eq!(m.addr, address);
            let OscType::String(body) = &m.args[0] else {
                panic!("JSON")
            };
            let json: serde_json::Value = serde_json::from_str(body).unwrap();
            assert_eq!(json["schema"], 3);
            assert_eq!(json["message_seq"], "1");
        }
        assert_eq!(
            telemetry
                .refresh(&s, false, start + Duration::from_millis(500))
                .unwrap(),
            0
        );
        s.status.state = crate::ConnectionState::Stale;
        assert_eq!(
            telemetry
                .refresh(&s, false, start + Duration::from_millis(600))
                .unwrap(),
            1
        );
        assert_eq!(receive().addr, STATUS_ADDRESS);
        assert_eq!(
            telemetry
                .refresh(&s, false, start + Duration::from_secs(5))
                .unwrap(),
            2
        );
        receive();
        receive();
        assert!(s.pose.is_none());
        assert_eq!(s.status.pose_count, 0);
    }
}
