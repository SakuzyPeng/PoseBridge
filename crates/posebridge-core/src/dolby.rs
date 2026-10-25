//! Dolby OSC head tracker output, isolated from PoseBridge protocol 3.
use crate::{DolbyOscConfig, DolbyOscStatus, Error, PoseSnapshot, Result, osc, pose};
use rosc::OscType;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

pub(crate) const ADDRESS: &str = "/ypr";
type Identity = (u64, u64, u64);

/// Convert a physical head-to-world rotation (right/forward/up) to the
/// driver's world-to-head matrix in its back/right/up basis, then extract
/// the driver's Rz(-yaw) Ry(-pitch) Rx(roll) angles. This preserves mixed
/// rotations and poles; component-wise Euler sign changes do not.
pub(crate) fn ypr_from_physical(physical: pose::Quat) -> Result<[f64; 3]> {
    let [x, y, z, w] = pose::normalize(physical)?;
    let sine = (2.0 * (w * x - y * z)).clamp(-1.0, 1.0);
    let (yaw, roll) = if sine.abs() >= 1.0 - 1e-12 {
        (
            (2.0 * (w * z - x * y)).atan2(1.0 - 2.0 * (y * y + z * z)),
            0.0,
        )
    } else {
        (
            (2.0 * (x * y + w * z)).atan2(1.0 - 2.0 * (x * x + z * z)),
            (2.0 * (x * z + w * y)).atan2(1.0 - 2.0 * (x * x + y * y)),
        )
    };
    Ok([
        yaw.to_degrees(),
        sine.asin().to_degrees(),
        roll.to_degrees(),
    ])
}

pub(crate) fn encode(ypr: [f64; 3]) -> Result<Vec<u8>> {
    let values = ypr.map(|v| v as f32);
    if values.iter().any(|v| !v.is_finite()) {
        return Err(Error::Invalid(
            "Dolby orientation must be finite float32".into(),
        ));
    }
    osc::message(ADDRESS, values.into_iter().map(OscType::Float).collect())
}

pub(crate) struct Sender {
    socket: UdpSocket,
    interval: Duration,
    next_slot: Option<Instant>,
    pending: Option<Instant>,
    last_key: Option<(u64, u64)>,
    identity: Option<Identity>,
    reference: Option<pose::Quat>,
    reference_after: Option<u64>,
    status: DolbyOscStatus,
}
impl Sender {
    pub(crate) fn new(config: &DolbyOscConfig) -> Result<Self> {
        Ok(Self {
            socket: osc::socket_for(config.target, config.max_rate_hz)?,
            interval: Duration::from_secs_f64(1.0 / f64::from(config.max_rate_hz)),
            next_slot: None,
            pending: None,
            last_key: None,
            identity: None,
            reference: None,
            reference_after: None,
            status: DolbyOscStatus {
                active: true,
                ..Default::default()
            },
        })
    }
    pub(crate) fn deadline(&self) -> Option<Instant> {
        self.pending
    }
    pub(crate) fn clear_pending(&mut self) {
        self.pending = None;
    }
    pub(crate) fn status(&self) -> DolbyOscStatus {
        self.status.clone()
    }
    pub(crate) fn clear_reference(&mut self) {
        self.reference = None;
        self.reference_after = None;
        self.status.reference_set = false;
        self.status.last_ypr_deg = None;
        self.pending = None;
    }
    pub(crate) fn recenter(&mut self, pose: &PoseSnapshot, physical: pose::Quat) -> Result<()> {
        let reference = pose::normalize(physical)?;
        self.identity = Some((pose.instance_id, pose.session_id, pose.reference_epoch));
        self.reference = Some(reference);
        self.reference_after = Some(pose.sequence);
        self.status.reference_set = true;
        self.status.last_ypr_deg = None;
        self.pending = None;
        Ok(())
    }
    pub(crate) fn send_if_due(&mut self, pose: &PoseSnapshot, physical: pose::Quat, now: Instant) {
        let identity = (pose.instance_id, pose.session_id, pose.reference_epoch);
        if self.identity != Some(identity) {
            self.clear_reference();
            self.identity = Some(identity);
        }
        let key = (pose.session_id, pose.sequence);
        if !pose.fresh
            || self.last_key == Some(key)
            || self
                .reference_after
                .is_some_and(|sequence| pose.sequence <= sequence)
        {
            self.pending = None;
            return;
        }
        if let Some(deadline) = self.next_slot.filter(|v| *v > now) {
            self.pending = Some(deadline);
            return;
        }
        let result = (|| -> Result<(bool, [f64; 3])> {
            let relative = if let Some([x, y, z, w]) = self.reference {
                pose::multiply([-x, -y, -z, w], physical)
            } else {
                physical
            };
            let ypr = ypr_from_physical(relative)?;
            Ok((osc::send(&self.socket, &encode(ypr)?)?, ypr))
        })();
        match result {
            Ok((true, ypr)) => {
                self.status.sent += 1;
                let previous = self.last_key.filter(|v| v.0 == key.0).map_or(0, |v| v.1);
                self.status.coalesced_samples += key.1.saturating_sub(previous).saturating_sub(1);
                self.last_key = Some(key);
                self.status.last_ypr_deg = Some(ypr);
                self.status.last_error = None;
                self.next_slot = Some(
                    self.next_slot
                        .map(|v| v + self.interval)
                        .filter(|v| *v > now)
                        .unwrap_or(now + self.interval),
                );
                self.pending = None;
            }
            other => {
                self.status.send_errors += 1;
                self.status.last_error = Some(match other {
                    Ok((false, _)) => {
                        "UDP did not accept the packet; receiver may be unavailable".into()
                    }
                    Err(error) => error.to_string(),
                    _ => unreachable!(),
                });
                self.next_slot = Some(now + self.interval);
                self.pending = self.next_slot;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rosc::{OscPacket, OscType};

    // Matrix formula independently checked against the installed Dolby driver.
    fn driver_matrix([y, p, r]: [f64; 3]) -> [[f64; 3]; 3] {
        let (sy, cy) = y.to_radians().sin_cos();
        let (sp, cp) = p.to_radians().sin_cos();
        let (sr, cr) = r.to_radians().sin_cos();
        [
            [cy * cp, sy * cr - cy * sp * sr, -sy * sr - cy * sp * cr],
            [-sy * cp, cy * cr + sy * sp * sr, sy * sp * cr - cy * sr],
            [sp, cp * sr, cp * cr],
        ]
    }
    fn target_matrix(q: pose::Quat) -> [[f64; 3]; 3] {
        let [x, y, z, w] = pose::normalize(q).unwrap();
        let physical = [
            [
                1. - 2. * (y * y + z * z),
                2. * (x * y - w * z),
                2. * (x * z + w * y),
            ],
            [
                2. * (x * y + w * z),
                1. - 2. * (x * x + z * z),
                2. * (y * z - w * x),
            ],
            [
                2. * (x * z - w * y),
                2. * (y * z + w * x),
                1. - 2. * (x * x + y * y),
            ],
        ];
        // B = [[0,-1,0],[1,0,0],[0,0,1]]. Target is B R^T B^T.
        let basis = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]];
        std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                (0..3)
                    .flat_map(|a| (0..3).map(move |b| basis[i][a] * physical[b][a] * basis[j][b]))
                    .sum()
            })
        })
    }
    #[test]
    fn driver_matrices_match_physical_rotations_including_mixed_poses_and_poles() {
        for y in [-180., -179., -60., 0., 30., 90., 179., 180.] {
            for p in [-90., -89.999, -30., 0., 20., 89.999, 90.] {
                for r in [-90., -20., 0., 10., 90., 180.] {
                    let q = pose::physical_from_euler([y, p, r]).unwrap();
                    for q in [q, q.map(|v| -v)] {
                        let actual = driver_matrix(ypr_from_physical(q).unwrap());
                        let expected = target_matrix(q);
                        for i in 0..3 {
                            for j in 0..3 {
                                assert!(
                                    (actual[i][j] - expected[i][j]).abs() < 2e-6,
                                    "{y},{p},{r}: {actual:?} != {expected:?}"
                                );
                            }
                        }
                    }
                }
            }
        }
        for angles in [[30., 0., 0.], [0., 20., 0.], [0., 0., 15.]] {
            let ypr = ypr_from_physical(pose::physical_from_euler(angles).unwrap()).unwrap();
            for (a, b) in ypr.into_iter().zip(angles) {
                assert!((a - b).abs() < 1e-8);
            }
        }
    }
    #[test]
    fn packet_is_exactly_one_three_float_message() {
        let packet = encode([30., -20., 10.]).unwrap();
        assert_eq!(packet.len(), 28);
        let (remainder, OscPacket::Message(message)) = rosc::decoder::decode_udp(&packet).unwrap()
        else {
            panic!("not a single message")
        };
        assert!(remainder.is_empty());
        assert_eq!(message.addr, "/ypr");
        assert_eq!(
            message.args,
            vec![
                OscType::Float(30.),
                OscType::Float(-20.),
                OscType::Float(10.)
            ]
        );
        assert!(encode([f64::NAN, 0., 0.]).is_err());
    }
    fn fixture() -> (Sender, UdpSocket, PoseSnapshot) {
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        socket
            .set_read_timeout(Some(Duration::from_millis(60)))
            .unwrap();
        let output = Sender::new(&DolbyOscConfig {
            target: socket.local_addr().unwrap(),
            max_rate_hz: 100,
        })
        .unwrap();
        let pose = PoseSnapshot {
            instance_id: 1,
            session_id: 1,
            sequence: 1,
            reference_epoch: 1,
            metadata_revision: 1,
            received_ns: 0,
            age_ns: 0,
            sample_time: None,
            quaternion_xyzw: [0., 0., 0., 1.],
            euler_deg: [0.; 3],
            raw: Default::default(),
            fresh: true,
        };
        (output, socket, pose)
    }
    fn receive(socket: &UdpSocket) -> [f64; 3] {
        let mut data = [0; 64];
        let n = socket.recv(&mut data).unwrap();
        let (_, OscPacket::Message(message)) = rosc::decoder::decode_udp(&data[..n]).unwrap()
        else {
            panic!()
        };
        std::array::from_fn(|i| match message.args[i] {
            OscType::Float(v) => f64::from(v),
            _ => panic!(),
        })
    }
    #[test]
    fn recenter_waits_for_a_new_sample_and_epoch_changes_clear_it() {
        let (mut output, socket, mut p) = fixture();
        let now = Instant::now();
        let q = pose::physical_from_euler([35., 25., -10.]).unwrap();
        output.send_if_due(&p, q, now);
        receive(&socket);
        output.recenter(&p, q).unwrap();
        output.send_if_due(&p, q, now + Duration::from_millis(15));
        assert_eq!(output.status.sent, 1);
        assert!(output.status.reference_set);
        p.sequence += 1;
        output.send_if_due(&p, q, now + Duration::from_millis(20));
        assert!(receive(&socket).into_iter().all(|v| v.abs() < 1e-5));
        p.reference_epoch += 1;
        p.sequence += 1;
        output.send_if_due(&p, q, now + Duration::from_millis(40));
        assert!(!output.status.reference_set);
        assert!(receive(&socket).iter().any(|v| v.abs() > 5.));
        p.fresh = false;
        p.sequence += 1;
        output.send_if_due(&p, q, now + Duration::from_millis(60));
        assert_eq!(output.status.sent, 3);
        assert!(output.deadline().is_none());
    }
    #[test]
    fn recentered_mixed_motion_is_quaternion_relative_and_a_new_session_resets_it() {
        let (mut output, socket, mut p) = fixture();
        let now = Instant::now();
        let reference = pose::physical_from_euler([60., 40., 30.]).unwrap();
        let delta = pose::physical_from_euler([15., -20., 25.]).unwrap();
        let moved = pose::multiply(reference, delta);
        output.recenter(&p, reference).unwrap();
        p.sequence += 1;
        output.send_if_due(&p, moved, now);
        let actual = driver_matrix(receive(&socket));
        let expected = target_matrix(delta);
        for i in 0..3 {
            for j in 0..3 {
                assert!((actual[i][j] - expected[i][j]).abs() < 1e-6);
            }
        }
        p.session_id += 1;
        p.sequence = 1;
        output.send_if_due(&p, moved, now + Duration::from_millis(20));
        assert!(!output.status.reference_set);
        let actual = driver_matrix(receive(&socket));
        let expected = target_matrix(moved);
        for i in 0..3 {
            for j in 0..3 {
                assert!((actual[i][j] - expected[i][j]).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn rate_limit_coalesces_once_and_uses_the_latest_sample() {
        let (mut output, socket, mut p) = fixture();
        let now = Instant::now();
        output.send_if_due(&p, pose::physical_from_euler([1., 0., 0.]).unwrap(), now);
        receive(&socket);
        p.sequence = 2;
        output.send_if_due(
            &p,
            pose::physical_from_euler([2., 0., 0.]).unwrap(),
            now + Duration::from_millis(2),
        );
        p.sequence = 3;
        output.send_if_due(
            &p,
            pose::physical_from_euler([3., 0., 0.]).unwrap(),
            now + Duration::from_millis(10),
        );
        assert!((receive(&socket)[0] - 3.).abs() < 1e-6);
        assert_eq!(output.status.coalesced_samples, 1);
        output.send_if_due(
            &p,
            pose::physical_from_euler([3., 0., 0.]).unwrap(),
            now + Duration::from_millis(20),
        );
        assert_eq!(output.status.sent, 2);
    }
}
