use crate::BatteryStatus;
use std::time::{Duration, Instant};

pub(crate) const REGISTER: u16 = 0x64;
pub(crate) const POLL_INTERVAL: Duration = Duration::from_secs(30);
pub(crate) const RESPONSE_TIMEOUT: Duration = Duration::from_secs(3);
pub(crate) const STALE_AFTER: Duration = Duration::from_secs(90);

/// The BLE 5.0 SDK's voltage lookup, not a coulomb counter or charging detector.
/// https://github.com/WITMOTION/WitBluetooth_BWT901BLE5_0/blob/9efaab0fdd6a06dc807bf80402e58aa91b431c6f/Windows_C%23/Wit.Example_BWT901BLE/ble5/Components/Bwt901bleProcessor.cs
fn estimate(raw: u16) -> Option<u8> {
    // Above a generous single-cell upper bound this can be a module supply
    // voltage (e.g. USB), not evidence of a fully charged battery.
    if !(200..=430).contains(&raw) {
        return None;
    }
    Some(match raw {
        396.. => 100,
        393.. => 90,
        387.. => 75,
        382.. => 60,
        379.. => 50,
        377.. => 40,
        373.. => 30,
        370.. => 20,
        368.. => 15,
        350.. => 10,
        340.. => 5,
        _ => 0,
    })
}

#[derive(Default)]
pub(crate) struct Battery {
    status: BatteryStatus,
    received: Option<Instant>,
}

impl Battery {
    pub(crate) fn observe(&mut self, raw: u16, now: Instant) {
        // Accept plausible module supply voltage too, but not zero/unsupported
        // registers or corrupt replies. A voltage alone does not prove charging.
        if !(200..=550).contains(&raw) {
            self.fail(format!(
                "invalid battery register 0x64: {raw} (expected 200..550 centivolts)"
            ));
            return;
        }
        self.status.raw_register = Some(raw);
        self.status.voltage_v = Some(f64::from(raw) / 100.0);
        self.status.estimated_percent = estimate(raw);
        self.status.last_error = None;
        self.received = Some(now);
    }

    pub(crate) fn fail(&mut self, error: String) {
        self.status.last_error = Some(error);
    }

    pub(crate) fn snapshot(&self, now: Instant, available: bool) -> BatteryStatus {
        let mut status = self.status.clone();
        status.age_ns = self.received.map(|t| {
            now.saturating_duration_since(t)
                .as_nanos()
                .min(i64::MAX as u128) as u64
        });
        status.fresh = available
            && status.last_error.is_none()
            && self
                .received
                .is_some_and(|t| now.saturating_duration_since(t) < STALE_AFTER);
        status
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vendor_thresholds_and_invalid_readings() {
        let levels = [
            (340, 5),
            (350, 10),
            (368, 15),
            (370, 20),
            (373, 30),
            (377, 40),
            (379, 50),
            (382, 60),
            (387, 75),
            (393, 90),
            (396, 100),
        ];
        let mut previous = 0;
        for (threshold, percent) in levels {
            assert_eq!(estimate(threshold - 1), Some(previous));
            assert_eq!(estimate(threshold), Some(percent));
            previous = percent;
        }
        assert_eq!(estimate(420), Some(100));
        for raw in [0, 199, 431, 483, 551, 32767, 65535] {
            assert_eq!(estimate(raw), None);
        }
    }

    #[test]
    fn errors_expiration_and_disconnect_do_not_disguise_old_readings() {
        let now = Instant::now();
        let mut battery = Battery::default();
        let empty = battery.snapshot(now, true);
        assert!(empty.voltage_v.is_none() && empty.estimated_percent.is_none());
        assert!(empty.age_ns.is_none() && !empty.fresh);
        battery.observe(382, now);
        assert!(
            battery
                .snapshot(now + STALE_AFTER - Duration::from_nanos(1), true)
                .fresh
        );
        assert!(!battery.snapshot(now + STALE_AFTER, true).fresh);
        assert!(!battery.snapshot(now, false).fresh);
        battery.observe(0, now + Duration::from_secs(1));
        let failed = battery.snapshot(now + Duration::from_secs(2), true);
        assert_eq!(failed.voltage_v, Some(3.82));
        assert_eq!(failed.estimated_percent, Some(60));
        assert_eq!(failed.age_ns, Some(2_000_000_000));
        assert!(failed.last_error.is_some() && !failed.fresh);
        battery.observe(379, now + Duration::from_secs(3));
        let recovered = battery.snapshot(now + Duration::from_secs(3), true);
        assert!(recovered.fresh && recovered.last_error.is_none());
        assert_eq!(recovered.estimated_percent, Some(50));
        assert_eq!(recovered.age_ns, Some(0));
        battery.observe(483, now + Duration::from_secs(4));
        let supply = battery.snapshot(now + Duration::from_secs(4), true);
        assert_eq!(supply.voltage_v, Some(4.83));
        assert!(supply.fresh && supply.last_error.is_none());
        assert!(supply.estimated_percent.is_none());
    }
}
