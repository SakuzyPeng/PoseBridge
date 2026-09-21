//! Sensor acquisition, orientation conversion and local OSC forwarding.

mod battery;
mod ble_link;
mod controller;
mod device;
mod model;
mod motion;
pub mod osc;
pub mod pose;
pub mod protocol;
mod sample_clock;
#[cfg(target_os = "windows")]
mod timing;
mod transport;

pub use controller::Controller;
pub use model::*;
pub use motion::{
    FULL_INERTIAL_20HZ_VALIDATED, MOTION_HISTORY_CAPACITY, MotionBatch, MotionCursor, MotionSample,
    OrientationSource,
};
