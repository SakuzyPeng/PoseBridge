use crate::{SampleTime, SampleTimeKind};

/// A connection-local clock tracker. Feed only validated pose samples.
#[derive(Default)]
pub(crate) struct SampleClock {
    previous: Option<(SampleTime, u64)>,
    pub duplicates: u64,
    pub discontinuities: u64,
}

impl SampleClock {
    pub fn observe(
        &mut self,
        kind: SampleTimeKind,
        time_ms: u64,
        received_ns: u64,
    ) -> Option<SampleTime> {
        let mut epoch = 1;
        if let Some((last, host_ns)) = self.previous {
            if kind == last.kind && time_ms == last.time_ms {
                self.duplicates += 1;
                return None;
            }
            let host_gap_ms = received_ns.saturating_sub(host_ns) / 1_000_000;
            let discontinuity = kind != last.kind
                || time_ms < last.time_ms
                || time_ms.saturating_sub(last.time_ms) > host_gap_ms.saturating_add(2000);
            epoch = last.clock_epoch + u64::from(discontinuity);
            self.discontinuities += u64::from(discontinuity);
        }
        let time = SampleTime {
            kind,
            time_ms,
            clock_epoch: epoch,
        };
        self.previous = Some((time, received_ns));
        Some(time)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn batches_duplicates_resets_and_forward_jumps() {
        let mut clock = SampleClock::default();
        let kind = SampleTimeKind::DeviceCalendar;
        assert_eq!(clock.observe(kind, 10000, 0).unwrap().clock_epoch, 1);
        assert_eq!(clock.observe(kind, 10005, 0).unwrap().clock_epoch, 1);
        assert_eq!(clock.observe(kind, 10005, 1_000_000_000), None);
        assert_eq!(clock.duplicates, 1);
        // Duplicate reception did not advance the host clock anchor.
        assert_eq!(
            clock
                .observe(kind, 11005, 1_000_000_000)
                .unwrap()
                .clock_epoch,
            1
        );
        assert_eq!(
            clock.observe(kind, 5, 1_001_000_000).unwrap().clock_epoch,
            2
        );
        assert_eq!(
            clock
                .observe(kind, 90000, 1_002_000_000)
                .unwrap()
                .clock_epoch,
            3
        );
        assert_eq!(
            clock
                .observe(SampleTimeKind::SimulatedElapsed, 90000, 1_003_000_000)
                .unwrap()
                .clock_epoch,
            4
        );
        assert_eq!(clock.discontinuities, 3);
    }
}
