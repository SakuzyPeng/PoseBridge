use posebridge_core::{
    protocol::{Frame, Parser, device_calendar_ms},
    *,
};

fn stamp(year: u8, month: u8, day: u8, hour: u8, minute: u8, second: u8, ms: u16) -> [u8; 8] {
    let [lo, hi] = ms.to_le_bytes();
    [year, month, day, hour, minute, second, lo, hi]
}

#[test]
fn calendar_boundaries_and_invalid_dates() {
    assert_eq!(device_calendar_ms(stamp(0, 1, 1, 0, 0, 0, 0)), Some(0));
    for (before, after) in [
        (
            stamp(0, 2, 28, 23, 59, 59, 999),
            stamp(0, 2, 29, 0, 0, 0, 0),
        ),
        (stamp(0, 2, 29, 23, 59, 59, 999), stamp(0, 3, 1, 0, 0, 0, 0)),
        (
            stamp(99, 12, 31, 23, 59, 59, 999),
            stamp(100, 1, 1, 0, 0, 0, 0),
        ),
        (
            stamp(100, 2, 28, 23, 59, 59, 999),
            stamp(100, 3, 1, 0, 0, 0, 0),
        ),
        (
            stamp(24, 4, 30, 23, 59, 59, 999),
            stamp(24, 5, 1, 0, 0, 0, 0),
        ),
    ] {
        assert_eq!(
            device_calendar_ms(after).unwrap() - device_calendar_ms(before).unwrap(),
            1
        );
    }
    for invalid in [
        stamp(100, 2, 29, 0, 0, 0, 0),
        stamp(1, 2, 29, 0, 0, 0, 0),
        stamp(24, 4, 31, 0, 0, 0, 0),
        stamp(24, 0, 1, 0, 0, 0, 0),
        stamp(24, 13, 1, 0, 0, 0, 0),
        stamp(24, 1, 0, 0, 0, 0, 0),
        stamp(24, 1, 1, 24, 0, 0, 0),
        stamp(24, 1, 1, 0, 60, 0, 0),
        stamp(24, 1, 1, 0, 0, 60, 0),
        stamp(24, 1, 1, 0, 0, 0, 1000),
    ] {
        assert_eq!(device_calendar_ms(invalid), None);
    }
}

fn frame(flag: u8, ms: u16) -> Vec<u8> {
    let mut bytes = vec![0x55, flag];
    if flag & 0x80 != 0 {
        bytes.extend(stamp(15, 1, 1, 3, 38, 46, ms));
    }
    if flag & 0x20 != 0 {
        bytes.extend(
            [
                (-16384i16).to_le_bytes(),
                0i16.to_le_bytes(),
                8192i16.to_le_bytes(),
            ]
            .concat(),
        );
    }
    if flag & 0x01 != 0 {
        bytes.extend(
            [
                (-16384i16).to_le_bytes(),
                8192i16.to_le_bytes(),
                0i16.to_le_bytes(),
            ]
            .concat(),
        );
    } else {
        bytes.extend([0xff, 0x7f, 0, 0, 0, 0, 0, 0]);
    }
    bytes
}

#[test]
fn variable_profiles_split_coalesced_invalid_and_resynchronized() {
    for flag in [0x01, 0x04, 0x81, 0x84, 0xa4] {
        let bytes = frame(flag, 930);
        for split in 0..bytes.len() {
            let mut parser = Parser::default();
            assert!(parser.push(&bytes[..split]).is_empty());
            let frames = parser.push(&[bytes[split..].to_vec(), bytes.clone()].concat());
            assert_eq!(frames.len(), 2);
            let Frame::Stream {
                sample_time_ms,
                angular_velocity_dps,
                euler_xyz_deg,
                quaternion_wxyz,
            } = frames[0]
            else {
                panic!("stream")
            };
            assert_eq!(sample_time_ms.is_some(), flag & 0x80 != 0);
            assert_eq!(euler_xyz_deg, (flag & 1 != 0).then_some([-90., 45., 0.]));
            assert_eq!(
                angular_velocity_dps,
                (flag & 0x20 != 0).then_some([-1000., 0., 500.])
            );
            assert_eq!(quaternion_wxyz.is_some(), flag & 4 != 0);
            assert_eq!(parser.discarded_bytes, 0);
        }
    }
    let mut bad_date = frame(0x84, 1000);
    let mut bad_q = frame(0x84, 930);
    bad_q[10..].fill(0);
    bad_date.extend(bad_q);
    bad_date.extend([0; 256]);
    bad_date.extend(frame(0xa4, 935));
    let mut parser = Parser::default();
    assert_eq!(parser.push(&bad_date).len(), 1);
    assert_eq!(parser.invalid_frames, 2);
    assert_eq!(parser.discarded_bytes, 292);
}

#[test]
fn real_ble_a4_capture_has_correct_time_and_component_order() {
    // 2026-09-19 timestamp_gyro_quaternion200.ndjson, first 24 bytes.
    let bytes = [
        0x55, 0xa4, 0x0f, 1, 1, 3, 0x26, 0x2e, 0xa2, 3, 0, 0, 0, 0, 0, 0, 0x86, 7, 5, 0xb2, 0xb5,
        0x64, 0xda, 0xf5,
    ];
    let frames = Parser::default().push(&bytes);
    let Frame::Stream {
        sample_time_ms,
        angular_velocity_dps,
        quaternion_wxyz,
        ..
    } = frames[0]
    else {
        panic!("stream")
    };
    assert_eq!(
        sample_time_ms,
        device_calendar_ms(stamp(15, 1, 1, 3, 38, 46, 930))
    );
    assert_eq!(angular_velocity_dps, Some([0.; 3]));
    assert_eq!(
        quaternion_wxyz.unwrap(),
        [1926., -19963., 25781., -2598.].map(|v| v / 32768.)
    );
}

#[test]
fn osc_keeps_int64_precision_and_distinguishes_absent_time() {
    let mut pose = PoseSnapshot {
        instance_id: 1,
        reference_epoch: 1,
        metadata_revision: 1,
        session_id: 0x123456789abcdef,
        sequence: (1 << 53) + 1,
        received_ns: (1 << 54) + 3,
        sample_time: Some(SampleTime {
            kind: SampleTimeKind::DeviceCalendar,
            time_ms: 473398726930,
            clock_epoch: 2,
        }),
        quaternion_xyzw: [0., 0., 0., 1.],
        euler_deg: [0.; 3],
        raw: RawData::default(),
        fresh: true,
    };
    for format in [OscFormat::Euler, OscFormat::Quaternion] {
        let bytes = osc::encode(&pose, format, "test", 1, 0).unwrap();
        let (_, rosc::OscPacket::Message(msg)) = rosc::decoder::decode_udp(&bytes).unwrap() else {
            panic!("message")
        };
        assert_eq!(msg.args[0], rosc::OscType::Int(3));
        assert_eq!(msg.args[1], rosc::OscType::String("test".into()));
        assert_eq!(msg.args[3], rosc::OscType::Long(pose.session_id as i64));
        assert_eq!(msg.args[4], rosc::OscType::Long(pose.sequence as i64));
        assert_eq!(msg.args[8], rosc::OscType::Long(pose.received_ns as i64));
        assert_eq!(msg.args[11], rosc::OscType::Long(473398726930));
    }

    pose.sample_time = None;
    let bytes = osc::encode(&pose, OscFormat::Euler, "test", 1, 0).unwrap();
    let (_, rosc::OscPacket::Message(msg)) = rosc::decoder::decode_udp(&bytes).unwrap() else {
        panic!("message")
    };
    assert_eq!(
        msg.args[10..13],
        [
            rosc::OscType::Int(0),
            rosc::OscType::Long(0),
            rosc::OscType::Long(0)
        ]
    );
    pose.session_id = u64::MAX;
    assert!(osc::encode(&pose, OscFormat::Euler, "test", 1, 0).is_err());
}
