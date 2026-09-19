//! Windows' default ~15.6 ms timer granularity cannot service a 5 ms pose cadence.
//! Keep the 1 ms request scoped to acquisition; no busy-wait thread or permanent OS change.

use crate::{Config, Error, PoseInput, Result, Source};
use windows::Win32::Media::{timeBeginPeriod, timeEndPeriod};

pub(crate) struct TimerResolution(bool);

impl TimerResolution {
    pub(crate) fn for_config(config: &Config) -> Result<Self> {
        let needed = config.osc.as_ref().is_some_and(|osc| osc.max_rate_hz > 50)
            || matches!(config.source, Source::Simulate { rate_hz, .. } if rate_hz > 50)
            || config.pose_input == PoseInput::Quaternion;
        if needed {
            // No pointer arguments; each successful request is balanced by Drop, also
            // when the acquisition future is cancelled or unwinds. Windows reference
            // counts requests so multiple C ABI contexts can have overlapping lifetimes.
            let result = unsafe { timeBeginPeriod(1) };
            if result != 0 {
                return Err(Error::Unavailable(format!(
                    "Windows 1 ms timer request failed with code {result}"
                )));
            }
        }
        Ok(Self(needed))
    }
}

impl Drop for TimerResolution {
    fn drop(&mut self) {
        if self.0 {
            unsafe { timeEndPeriod(1) };
        }
    }
}
