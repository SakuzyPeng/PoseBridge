//! Bounded, same-frame motion records for in-process consumers. OSC/C ABI stay unchanged.
use crate::{SampleTime, pose::Quat};
use std::collections::VecDeque;
use std::time::Instant;

pub const MOTION_HISTORY_CAPACITY: usize = 256;
/// Remains false until the 20 Hz USB/BLE dynamic hardware acceptance is recorded.
pub const FULL_INERTIAL_20HZ_VALIDATED: bool = false;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotionCursor {
    pub instance_id: u64,
    pub session_id: u64,
    pub sequence: u64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrientationSource {
    NativeQuaternion,
    ConvertedEuler,
    RegisterQuaternion,
    Simulator,
}
#[derive(Clone, Debug)]
pub struct MotionSample {
    pub source_id: String,
    pub cursor: MotionCursor,
    pub reference_epoch: u64,
    pub metadata_revision: u64,
    /// One complete transport read/notification, published atomically.
    pub delivery_id: u64,
    pub received_ns: u64,
    /// Reception-to-query age; all records use MotionBatch::queried_at.
    pub age_ns: u64,
    pub fresh: bool,
    pub sample_time: Option<SampleTime>,
    /// Physical Hamilton XYZW, X right / Y forward / Z up. Not the GUI YXZ profile.
    pub orientation_xyzw: Quat,
    /// Existing yaw/pitch/roll semantics for unchanged ordinary consumers.
    pub euler_deg: [f64; 3],
    /// Body-frame head axes, radians/second. Only the same pose frame's fields.
    pub angular_velocity_rad_s: Option<[f64; 3]>,
    pub acceleration_g: Option<[f64; 3]>,
    pub orientation_source: OrientationSource,
    /// Wire flag; zero for synthetic records.
    pub profile: u8,
}
#[derive(Clone, Debug)]
pub struct MotionBatch {
    pub source_id: String,
    pub queried_at: Instant,
    /// Latest acquisition is fresh at query time, including empty cursor polls.
    pub active: bool,
    pub cursor: Option<MotionCursor>,
    pub reset: bool,
    /// Records evicted before this cursor could consume them, not device packet loss.
    pub history_overrun: u64,
    pub samples: Vec<MotionSample>,
}
#[derive(Clone, Debug)]
pub(crate) struct Signals {
    pub orientation: Quat,
    pub gyro: Option<[f64; 3]>,
    pub acceleration: Option<[f64; 3]>,
    pub source: OrientationSource,
    pub profile: u8,
}
#[derive(Default)]
pub(crate) struct History {
    records: VecDeque<(Instant, MotionSample)>,
    pub delivery: u64,
}
impl History {
    pub fn push(&mut self, received: Instant, sample: MotionSample) {
        if self.records.len() == MOTION_HISTORY_CAPACITY {
            self.records.pop_front();
        }
        self.records.push_back((received, sample));
    }
    pub fn since(&self, cursor: Option<MotionCursor>, now: Instant, active: bool) -> MotionBatch {
        let latest = self.records.back().map(|(_, p)| p.cursor);
        let reset = cursor.zip(latest).is_some_and(|(a, b)| {
            a.instance_id != b.instance_id
                || a.session_id != b.session_id
                || a.sequence > b.sequence
        });
        let after = if reset {
            0
        } else {
            cursor.map_or(0, |c| c.sequence)
        };
        let history_overrun = if cursor.is_some() && !reset {
            self.records.front().map_or(0, |(_, p)| {
                p.cursor.sequence.saturating_sub(after.saturating_add(1))
            })
        } else {
            0
        };
        let samples = self
            .records
            .iter()
            .filter(|(_, p)| p.cursor.sequence > after)
            .map(|(received, p)| {
                let mut p = p.clone();
                p.age_ns = now
                    .saturating_duration_since(*received)
                    .as_nanos()
                    .min(i64::MAX as u128) as u64;
                p.fresh = active && p.age_ns < 500_000_000;
                p
            })
            .collect();
        MotionBatch {
            source_id: String::new(),
            queried_at: now,
            active,
            cursor: latest,
            reset,
            history_overrun,
            samples,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    fn sample(sequence: u64) -> MotionSample {
        MotionSample {
            source_id: "test".into(),
            cursor: MotionCursor {
                instance_id: 1,
                session_id: 1,
                sequence,
            },
            reference_epoch: 1,
            metadata_revision: 1,
            delivery_id: 1,
            received_ns: 0,
            age_ns: 0,
            fresh: true,
            sample_time: None,
            orientation_xyzw: [0., 0., 0., 1.],
            euler_deg: [0.; 3],
            angular_velocity_rad_s: None,
            acceleration_g: None,
            orientation_source: OrientationSource::ConvertedEuler,
            profile: 0x61,
        }
    }
    #[test]
    fn bounded_history_reports_loss_and_does_not_reanimate_stopped_data() {
        let mut h = History::default();
        let at = Instant::now();
        for i in 1..=300 {
            h.push(at, sample(i));
        }
        let batch = h.since(
            Some(sample(1).cursor),
            at + Duration::from_millis(100),
            true,
        );
        assert_eq!(batch.samples.len(), 256);
        assert_eq!(batch.history_overrun, 43);
        assert!(
            batch
                .samples
                .iter()
                .all(|s| s.age_ns == 100_000_000 && s.fresh)
        );
        assert!(h.since(batch.cursor, at, true).samples.is_empty());
        assert!(
            h.since(None, at + Duration::from_millis(500), true)
                .samples
                .iter()
                .all(|p| !p.fresh)
        );
        assert!(h.since(None, at, false).samples.iter().all(|p| !p.fresh));
        let mut c = sample(1).cursor;
        c.session_id = 2;
        assert!(h.since(Some(c), at, true).reset);
    }
}
