use posebridge_core::{
    pose::{self, Mounting},
    protocol::{self, Frame, Parser},
    *,
};
use std::net::UdpSocket;
use std::time::{Duration, Instant};

fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-6, "{a} != {b}");
}

#[test]
#[allow(clippy::approx_constant)] // Published, rounded protocol test vectors are intentional.
fn documented_quaternions_and_wrap() {
    let cases = [
        ([0., 0., 0.], [0., 0., 0., 1.]),
        ([90., 0., 0.], [0., 0.707106781, 0., 0.707106781]),
        ([0., 30., 0.], [0.258819045, 0., 0., 0.965925826]),
        ([0., 0., 30.], [0., 0., 0.258819045, 0.965925826]),
        (
            [30., 20., 10.],
            [0.189307857, 0.239298338, 0.038134576, 0.951548525],
        ),
        ([179., 0., 0.], [0., 0.999961923, 0., 0.008726535]),
        ([-179., 0., 0.], [0., -0.999961923, 0., 0.008726535]),
    ];
    for (angles, expected) in cases {
        let q = pose::from_euler(angles).unwrap();
        for (a, b) in q.into_iter().zip(expected) {
            close(a, b);
        }
        for (a, b) in pose::to_euler(q).unwrap().into_iter().zip(angles) {
            close(a, b);
        }
        close(pose::angular_distance_deg(q, q.map(|v| -v)).unwrap(), 0.);
    }
    close(
        pose::angular_distance_deg(
            pose::from_euler([179., 0., 0.]).unwrap(),
            pose::from_euler([-179., 0., 0.]).unwrap(),
        )
        .unwrap(),
        2.,
    );
    assert!(pose::normalize([0.; 4]).is_err());
    assert!(pose::from_euler([f64::NAN, 0., 0.]).is_err());
    assert!(pose::normalize([f64::INFINITY, 0., 0., 1.]).is_err());
}

#[test]
fn mounting_is_a_rotation_not_a_quaternion_component_swap() {
    let m = Mounting::parse("-y,+x,+z").unwrap();
    let angles = pose::to_euler(m.from_sensor_euler([10., -20., 30.]).unwrap()).unwrap();
    for (a, b) in angles.into_iter().zip([30., 20., 10.]) {
        close(a, b);
    }
    assert!(Mounting::parse("+x,+x,+z").is_err());
    assert!(Mounting::parse("+x,+z,+y").is_err());
    assert!(Mounting::parse("--x,-y,+z").is_err());
    assert!(
        Mounting {
            right: i8::MIN,
            forward: 1,
            up: 2
        }
        .validate()
        .is_err()
    );
}

const USB_SAMPLE: [u8; 20] = [
    0x55, 0x61, 0x11, 0x00, 0x1b, 0x00, 0x23, 0x08, 0, 0, 0, 0, 0, 0, 0x8b, 0, 0x9f, 0xff, 0x89,
    0xeb,
];

#[test]
fn captured_usb_frame_fragmentation_noise_and_coalescing() {
    let mut parser = Parser::default();
    assert!(parser.push(&[1, 2, 0x55, 0x40, 3]).is_empty());
    assert!(parser.push(&USB_SAMPLE[..7]).is_empty());
    let mut tail = USB_SAMPLE[7..].to_vec();
    tail.extend(USB_SAMPLE);
    let frames = parser.push(&tail);
    assert_eq!(frames.len(), 2);
    assert_eq!(parser.discarded_bytes, 5);
    match &frames[0] {
        Frame::Motion {
            acceleration_g,
            angular_velocity_dps,
            euler_xyz_deg,
        } => {
            close(acceleration_g[0], 17. / 32768. * 16.);
            close(acceleration_g[2], 2083. / 32768. * 16.);
            assert_eq!(*angular_velocity_dps, [0.; 3]);
            close(euler_xyz_deg[0], 139. / 32768. * 180.);
            close(euler_xyz_deg[1], -97. / 32768. * 180.);
            close(euler_xyz_deg[2], -5239. / 32768. * 180.);
        }
        _ => panic!("wrong frame"),
    }
}

#[test]
fn register_values_and_explicit_configuration_commands() {
    let mut bytes = [0u8; 20];
    bytes[..4].copy_from_slice(&[0x55, 0x71, 0x51, 0]);
    bytes[4..6].copy_from_slice(&16384i16.to_le_bytes());
    bytes[6..8].copy_from_slice(&(-16384i16).to_le_bytes());
    let result = Parser::default().push(&bytes);
    match result[0] {
        Frame::Registers { address, values } => {
            assert_eq!(address, 0x51);
            assert_eq!(values[0], 16384);
            assert_eq!(values[1], -16384);
        }
        _ => panic!("wrong frame"),
    }
    assert_eq!(protocol::read_register(0x51), [0xff, 0xaa, 0x27, 0x51, 0]);
    for (hz, code) in [(10, 6), (50, 8), (100, 9), (200, 11)] {
        assert_eq!(
            protocol::command_register(&DeviceCommand::Rate { hz }).unwrap(),
            (3, code)
        );
    }
    assert!(protocol::command_register(&DeviceCommand::Rate { hz: 125 }).is_err());
}

fn wait_pose(c: &Controller) -> PoseSnapshot {
    let start = Instant::now();
    loop {
        if let Some(p) = c.latest_pose() {
            return p;
        }
        assert!(start.elapsed() < Duration::from_secs(3), "{:?}", c.status());
        std::thread::sleep(Duration::from_millis(2));
    }
}

#[test]
fn controller_snapshot_stale_recovery_and_lifecycle() {
    let mut c = Controller::new().unwrap();
    assert!(c.latest_pose().is_none());
    c.set_config(Config {
        source: Source::Simulate {
            pattern: Pattern::Fixed,
            euler_deg: [30., 20., 10.],
            rate_hz: 1,
        },
        ..Config::default()
    })
    .unwrap();
    c.start().unwrap();
    let first = wait_pose(&c);
    assert!(first.fresh);
    assert_eq!(c.latest_pose().unwrap().sequence, first.sequence);
    assert!(matches!(c.start(), Err(Error::Busy)));
    std::thread::sleep(Duration::from_millis(600));
    assert_eq!(c.status().state, ConnectionState::Stale);
    assert!(!c.latest_pose().unwrap().fresh);
    let deadline = Instant::now() + Duration::from_secs(2);
    while c.latest_pose().unwrap().sequence == first.sequence {
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(2));
    }
    assert!(c.latest_pose().unwrap().fresh);
    c.stop().unwrap();
    c.stop().unwrap();
    assert!(!c.latest_pose().unwrap().fresh);
    c.start().unwrap();
    let second = wait_pose(&c);
    assert_ne!(first.session_id, second.session_id);
    c.stop().unwrap();
}

#[test]
fn osc_loopback_types_values_and_rate_limit() {
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    socket
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let mut c = Controller::new().unwrap();
    c.set_config(Config {
        source: Source::Simulate {
            pattern: Pattern::Fixed,
            euler_deg: [30., 20., 10.],
            rate_hz: 200,
        },
        osc: Some(OscConfig {
            target: socket.local_addr().unwrap(),
            max_rate_hz: 50,
            format: OscFormat::Euler,
        }),
        ..Config::default()
    })
    .unwrap();
    c.start().unwrap();
    let start = Instant::now();
    let mut count = 0;
    let mut data = [0u8; 512];
    while start.elapsed() < Duration::from_millis(800) {
        let n = socket.recv(&mut data).unwrap();
        let (rest, packet) = rosc::decoder::decode_udp(&data[..n]).unwrap();
        assert!(rest.is_empty());
        match packet {
            rosc::OscPacket::Message(m) => {
                assert_eq!(m.addr, "/posebridge/v1/euler");
                assert_eq!(
                    m.args,
                    vec![
                        rosc::OscType::Float(30.),
                        rosc::OscType::Float(20.),
                        rosc::OscType::Float(10.)
                    ]
                );
            }
            _ => panic!("not an OSC message"),
        }
        count += 1;
    }
    c.stop().unwrap();
    assert!((25..=43).contains(&count), "packet count {count}");
    assert!(c.status().pose_count > count);
}

#[test]
fn no_duplicate_or_stale_osc_pose() {
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    socket
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let mut c = Controller::new().unwrap();
    c.set_config(Config {
        source: Source::Simulate {
            pattern: Pattern::Fixed,
            euler_deg: [0.; 3],
            rate_hz: 1,
        },
        osc: Some(OscConfig {
            target: socket.local_addr().unwrap(),
            max_rate_hz: 100,
            format: OscFormat::Quaternion,
        }),
        ..Config::default()
    })
    .unwrap();
    c.start().unwrap();
    let mut data = [0u8; 512];
    let _ = socket.recv(&mut data).unwrap();
    std::thread::sleep(Duration::from_millis(550));
    assert!(socket.recv(&mut data).is_err());
    c.stop().unwrap();
}

#[test]
fn configuration_validation_preserves_previous_config() {
    let mut c = Controller::new().unwrap();
    let config = Config {
        osc: Some(OscConfig {
            target: "192.0.2.1:9000".parse().unwrap(),
            max_rate_hz: 100,
            format: OscFormat::Quaternion,
        }),
        ..Config::default()
    };
    assert!(c.set_config(config).is_err());
    assert!(c.config().osc.is_none());
    assert!(
        c.set_config(Config {
            source: Source::Usb {
                port: "fake".into(),
                baud: 115200
            },
            ..Config::default()
        })
        .is_err()
    );
}
