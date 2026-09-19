//! Sensor acquisition, orientation conversion and local OSC forwarding.

mod controller;
mod model;
pub mod osc;
pub mod pose;
pub mod protocol;
mod transport;

pub use controller::Controller;
pub use model::*;
