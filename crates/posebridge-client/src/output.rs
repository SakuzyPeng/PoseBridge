//! Output is part of an acquisition configuration, never a second pose processor.
use crate::posebridge::service::{Phase, View};
use eframe::egui;
use posebridge_core as pb;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Format {
    #[default]
    Quaternion,
    Euler,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Preferences {
    pub enabled: bool,
    pub target: String,
    pub format: Format,
    pub max_rate_hz: u32,
}
impl Default for Preferences {
    fn default() -> Self {
        Self {
            enabled: false,
            target: "127.0.0.1:9000".into(),
            format: Format::Quaternion,
            max_rate_hz: 100,
        }
    }
}
impl Preferences {
    pub fn config(&self) -> Result<Option<pb::OscConfig>, String> {
        if !self.enabled {
            return Ok(None);
        }
        let osc = pb::OscConfig {
            target: self.target.trim().parse().map_err(|_| {
                "Enter a loopback IP address and port, such as 127.0.0.1:9000 or [::1]:9000"
                    .to_owned()
            })?,
            max_rate_hz: self.max_rate_hz,
            format: match self.format {
                Format::Quaternion => pb::OscFormat::Quaternion,
                Format::Euler => pb::OscFormat::Euler,
            },
        };
        pb::Config {
            osc: Some(osc.clone()),
            ..Default::default()
        }
        .validate()
        .map_err(|e| e.to_string())?;
        Ok(Some(osc))
    }
}

pub fn summary(ui: &mut egui::Ui, prefs: &Preferences, view: &View) {
    let active = view
        .snapshot
        .as_ref()
        .and_then(|s| s.descriptor.application_config.osc.as_ref());
    if view.phase == Phase::Tracking {
        if let Some(osc) = active {
            ui.small(format!(
                "OSC → {} · up to {} Hz",
                osc.target, osc.max_rate_hz
            ));
        } else if prefs.enabled {
            ui.small(format!("OSC pending connection → {}", prefs.target));
        } else {
            ui.small("OSC off");
        }
    } else if prefs.enabled {
        ui.small(format!(
            "OSC configured for next connection → {}",
            prefs.target
        ));
    } else {
        ui.small("OSC off · configure on Output");
    }
    if let Err(error) = prefs.config() {
        ui.colored_label(crate::theme::WARNING, error);
    }
}

pub fn draw(ui: &mut egui::Ui, prefs: &mut Preferences, view: &View) {
    ui.heading("OSC output");
    ui.label(
        "Send the sensor orientation, with its installation mapping, to software on this computer.",
    );
    ui.small("Recentring, smoothing and prediction belong to the receiving application.");
    ui.add_space(12.);
    let editable = !view.phase.busy() && view.magnetic.is_none();
    if !editable {
        ui.label("Disconnect or finish the device operation to change output settings.");
    }
    ui.add_enabled_ui(editable, |ui| {
        ui.checkbox(&mut prefs.enabled, "Enable OSC when connecting");
        ui.label("Destination · loopback IP:port");
        ui.add(egui::TextEdit::singleline(&mut prefs.target).desired_width(300.));
        ui.horizontal_wrapped(|ui| {
            ui.selectable_value(&mut prefs.format, Format::Quaternion, "Quaternion · XYZW");
            ui.selectable_value(
                &mut prefs.format,
                Format::Euler,
                "Euler · yaw / pitch / roll",
            );
        });
        ui.add(egui::Slider::new(&mut prefs.max_rate_hz, 1..=200).text("Maximum output · Hz"));
    });
    summary(ui, prefs, view);
    ui.separator();
    ui.label("Protocol 3 · /posebridge/quaternion or /posebridge/euler");
    ui.small("Only fresh, distinct sensor samples are sent. The rate limit does not change the sensor rate. Info and status heartbeats are independent of pose freshness.");
    ui.small("Scanning, settings operations and magnetic sessions do not send OSC. Acquisition and output continue while this window is minimized.");
    if let Some(s) = &view.snapshot {
        ui.label(format!(
            "Sent: {} · coalesced samples: {} · send errors: {} · heartbeats: {}",
            s.status.osc_sent,
            s.status.coalesced_samples,
            s.status.send_errors,
            s.status.telemetry_sent
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn output_is_opt_in_and_reuses_the_core_validation() {
        let mut p = Preferences::default();
        assert!(p.config().unwrap().is_none());
        p.enabled = true;
        assert_eq!(p.config().unwrap().unwrap().max_rate_hz, 100);
        for target in ["192.168.1.2:9000", "127.0.0.1:0", "invalid"] {
            p.target = target.into();
            assert!(p.config().is_err());
        }
        p.target = "[::1]:9000".into();
        assert!(p.config().is_ok());
        p.max_rate_hz = 201;
        assert!(p.config().is_err());
    }
}
