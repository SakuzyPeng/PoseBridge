use posebridge_core::*;
use std::time::{Duration, Instant};

#[test]
fn source_identity_snapshot_and_restart() {
    let mut c = Controller::new().unwrap();
    c.set_config(Config {
        source_id: Some("耳机".into()),
        ..Config::default()
    })
    .unwrap();
    c.start().unwrap();
    let end = Instant::now() + Duration::from_secs(2);
    while c.latest_pose().is_none() {
        assert!(Instant::now() < end);
        std::thread::sleep(Duration::from_millis(2));
    }
    let s = c.snapshot();
    let p = s.pose.as_ref().unwrap();
    assert_eq!(p.instance_id, s.descriptor.instance_id);
    assert_eq!(p.session_id, s.descriptor.session_id);
    assert_eq!(p.reference_epoch, s.descriptor.reference_epoch);
    assert_eq!(p.metadata_revision, s.descriptor.metadata_revision);
    assert_eq!(s.descriptor.device_model, None);
    assert!(!s.descriptor.device.valid);
    assert!(matches!(c.inspect_start(), Err(Error::Busy)));
    assert!(matches!(
        c.configure_device(DeviceCommand::ResetDefaults),
        Err(Error::Busy)
    ));
    let json: serde_json::Value = serde_json::from_str(&snapshot_json(&s).unwrap()).unwrap();
    assert!(json["pose"]["sequence"].is_string());
    assert!(json["status"]["delivery"]["gap_histogram"][0].is_string());
    let instance = p.instance_id;
    c.stop().unwrap();
    c.start().unwrap();
    assert_ne!(c.snapshot().descriptor.instance_id, instance);
    c.stop().unwrap();
}
#[test]
fn configuration_identity_and_legacy_version_rejection() {
    for id in ["", "\n", &"a".repeat(257)] {
        assert!(
            Config {
                source_id: Some(id.into()),
                ..Config::default()
            }
            .validate()
            .is_err()
        );
    }
    assert!(
        serde_json::from_str::<Config>(
            r#"{"source":{"kind":"simulate"},"osc":{"target":"127.0.0.1:9000","version":"v2"}}"#
        )
        .is_err()
    );
    let config = Config {
        source: Source::Usb {
            port: "COM4".into(),
            baud: 115200,
        },
        ..Config::default()
    };
    assert_eq!(config.logical_source_id(), "usb:COM4");
    assert!(config.validate().is_ok());
    assert!(config.validate_acquisition().is_err());
}
