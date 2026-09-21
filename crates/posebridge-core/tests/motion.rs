use posebridge_core::{
    pose,
    protocol::{Frame, Parser},
    *,
};
use std::time::{Duration, Instant};

#[test]
fn complete_inertial_frame_is_split_safe_and_keeps_each_field() {
    let mut data = vec![0x55, 0xe4, 26, 9, 20, 12, 0, 0, 0, 0];
    for word in [0i16, 0, 2048, 16384, 0, -8192, 32767, 0, 0, 0] {
        data.extend(word.to_le_bytes());
    }
    assert_eq!(data.len(), 30);
    for split in 0..data.len() {
        let mut p = Parser::default();
        assert!(p.push(&data[..split]).is_empty());
        let frames = p.push(&data[split..]);
        assert_eq!(frames.len(), 1);
        let Frame::Stream {
            profile,
            acceleration_g,
            angular_velocity_dps,
            quaternion_wxyz,
            sample_time_ms,
            ..
        } = frames[0]
        else {
            panic!()
        };
        assert_eq!(profile, 0xe4);
        assert_eq!(acceleration_g, Some([0., 0., 1.]));
        assert_eq!(angular_velocity_dps, Some([1000., 0., -500.]));
        assert!(quaternion_wxyz.is_some() && sample_time_ms.is_some());
        assert_eq!(p.discarded_bytes, 0);
    }
    let mut p = Parser::default();
    data.extend_from_within(..);
    assert_eq!(p.push(&data).len(), 2);
}
#[test]
fn physical_quaternion_and_vectors_use_the_same_proper_basis() {
    let m = pose::Mounting::parse("-y,+x,+z").unwrap();
    assert_eq!(m.vector([1., 2., 3.]).unwrap(), [-2., 1., 3.]);
    // Sensor Y becomes negative head X. Check against the independent axis-angle value.
    let a = 20_f64.to_radians();
    let q = m
        .physical_from_sensor_quaternion([0., a.sin(), 0., a.cos()])
        .unwrap();
    assert!(pose::angular_distance_deg(q, [-a.sin(), 0., 0., a.cos()]).unwrap() < 1e-5);
    // q/-q remain equivalent, including compound rotations and either pitch pole.
    for e in [
        [30., 20., 10.],
        [0., 90., 0.],
        [0., -90., 0.],
        [179., 20., 10.],
    ] {
        let q = pose::physical_from_euler(e).unwrap();
        let a = m.physical_from_sensor_quaternion(q).unwrap();
        let b = m.physical_from_sensor_quaternion(q.map(|v| -v)).unwrap();
        assert!(pose::angular_distance_deg(a, b).unwrap() < 1e-5);
    }
}
#[test]
fn cursor_polling_preserves_samples_and_restarts_reset_identity() {
    let mut c = Controller::new().unwrap();
    c.set_config(Config {
        source: Source::Simulate {
            pattern: Pattern::Combined,
            euler_deg: [0.; 3],
            rate_hz: 100,
            sample_clock: true,
        },
        ..Config::default()
    })
    .unwrap();
    c.start().unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while c.motion_since(None).samples.len() < 12 {
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(5));
    }
    let b = c.motion_since(None);
    assert!(
        b.samples
            .iter()
            .all(|p| p.angular_velocity_rad_s.is_some() && p.sample_time.is_some())
    );
    let cursor = b.cursor;
    c.stop().unwrap();
    assert!(c.motion_since(None).samples.iter().all(|p| !p.fresh));
    c.start().unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while c.motion_since(None).samples.is_empty() {
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(c.motion_since(cursor).reset);
    c.stop().unwrap();
}
