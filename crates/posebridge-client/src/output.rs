//! Two independent OSC receivers share one physical acquisition.
use crate::posebridge::service::{Command, Phase, View};
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
pub struct DolbyPreferences {
    pub enabled: bool,
    pub target: String,
    pub max_rate_hz: u32,
}
impl Default for DolbyPreferences {
    fn default() -> Self {
        Self {
            enabled: false,
            target: "127.0.0.1:8000".into(),
            max_rate_hz: 100,
        }
    }
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Preferences {
    // Preserve the version-1 PoseBridge fields during migration.
    pub enabled: bool,
    pub target: String,
    pub format: Format,
    pub max_rate_hz: u32,
    pub dolby: DolbyPreferences,
}
impl Default for Preferences {
    fn default() -> Self {
        Self {
            enabled: false,
            target: "127.0.0.1:9000".into(),
            format: Format::Quaternion,
            max_rate_hz: 100,
            dolby: DolbyPreferences::default(),
        }
    }
}
#[derive(Clone, Default)]
pub struct Configuration {
    pub posebridge: Option<pb::OscConfig>,
    pub dolby: Option<pb::DolbyOscConfig>,
}
impl Configuration {
    pub fn apply(&self, config: &mut pb::Config) {
        config.osc = self.posebridge.clone();
        config.dolby_osc = self.dolby.clone();
    }
}
impl Preferences {
    pub fn config(&self) -> Result<Configuration, String> {
        let configuration = Configuration {
            posebridge: if self.enabled {
                Some(pb::OscConfig {
                    target: self.target.trim().parse().map_err(|_| "Enter a PoseBridge loopback IP address and port, such as 127.0.0.1:9000".to_owned())?,
                    max_rate_hz: self.max_rate_hz,
                    format: match self.format { Format::Quaternion => pb::OscFormat::Quaternion, Format::Euler => pb::OscFormat::Euler },
                })
            } else {
                None
            },
            dolby: if self.dolby.enabled {
                Some(pb::DolbyOscConfig {
                    target: self.dolby.target.trim().parse().map_err(|_| {
                        "Enter a Dolby loopback IP address and port, usually 127.0.0.1:8000"
                            .to_owned()
                    })?,
                    max_rate_hz: self.dolby.max_rate_hz,
                })
            } else {
                None
            },
        };
        let mut config = pb::Config::default();
        configuration.apply(&mut config);
        config.validate().map_err(|e| e.to_string())?;
        Ok(configuration)
    }
}

pub fn summary(ui: &mut egui::Ui, prefs: &Preferences, view: &View) {
    let live = view.snapshot.as_ref().filter(|s| {
        view.phase == Phase::Tracking
            && matches!(
                s.status.state,
                pb::ConnectionState::Active
                    | pb::ConnectionState::Stale
                    | pb::ConnectionState::Connecting
                    | pb::ConnectionState::Reconnecting
            )
    });
    ui.horizontal_wrapped(|ui| {
        if let Some(snapshot) = live {
            let config = &snapshot.descriptor.application_config;
            if let Some(osc) = &config.osc {
                ui.small(format!(
                    "PoseBridge → {} · up to {} Hz",
                    osc.target, osc.max_rate_hz
                ));
            }
            if let Some(osc) = &config.dolby_osc {
                ui.small(format!(
                    "Dolby → {} · up to {} Hz",
                    osc.target, osc.max_rate_hz
                ));
            }
            if config.osc.is_none() && config.dolby_osc.is_none() {
                ui.small("OSC off");
            }
        } else if prefs.enabled || prefs.dolby.enabled {
            if prefs.enabled {
                ui.small(format!("PoseBridge next connection → {}", prefs.target));
            }
            if prefs.dolby.enabled {
                ui.small(format!("Dolby next connection → {}", prefs.dolby.target));
            }
        } else {
            ui.small("OSC off · configure on Output");
        }
    });
    if let Err(error) = prefs.config() {
        ui.colored_label(crate::theme::WARNING, error);
    }
}

pub fn draw(ui: &mut egui::Ui, prefs: &mut Preferences, view: &View) -> bool {
    ui.heading("OSC outputs");
    ui.label("One sensor connection can send to both applications.");
    let editable = !view.phase.busy() && view.magnetic.is_none();
    if !editable {
        ui.small("Disconnect or finish the device operation to change output settings.");
    }
    ui.add_space(8.);
    let recenter = if ui.available_width() >= 800. {
        ui.columns(2, |columns| {
            posebridge_card(&mut columns[0], prefs, view, editable);
            dolby_card(&mut columns[1], prefs, view, editable)
        })
    } else {
        posebridge_card(ui, prefs, view, editable);
        ui.add_space(12.);
        dolby_card(ui, prefs, view, editable)
    };
    ui.add_space(8.);
    ui.small("Only fresh, distinct samples are sent. Rate limits do not change the sensor rate. Acquisition and outputs continue while minimized.");
    ui.small("Scanning, device settings and magnetic sessions do not send orientation. UDP send counts do not acknowledge reception.");
    recenter
}
fn card(ui: &mut egui::Ui, title: &str, contents: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::group(ui.style())
        .fill(crate::theme::SURFACE)
        .inner_margin(12)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.strong(title);
            ui.add_space(8.);
            contents(ui);
        });
}
fn posebridge_card(ui: &mut egui::Ui, prefs: &mut Preferences, view: &View, editable: bool) {
    ui.push_id("posebridge-output", |ui| {
        card(ui, "PoseBridge", |ui| {
            ui.add_enabled_ui(editable, |ui| {
                ui.checkbox(&mut prefs.enabled, "Enable PoseBridge output");
                ui.label("Destination · loopback IP:port");
                ui.add(
                    egui::TextEdit::singleline(&mut prefs.target)
                        .desired_width(ui.available_width()),
                );
                ui.horizontal_wrapped(|ui| {
                    ui.selectable_value(&mut prefs.format, Format::Quaternion, "Quaternion · XYZW");
                    ui.selectable_value(
                        &mut prefs.format,
                        Format::Euler,
                        "Euler · yaw / pitch / roll",
                    );
                });
                ui.add(egui::Slider::new(&mut prefs.max_rate_hz, 1..=200).text("Maximum · Hz"));
            });
            ui.small("Protocol 3 · original sensor reference, with installation mapping.");
            ui.small("Recentring, smoothing and prediction are handled by this receiver.");
            if let Some(s) = &view.snapshot {
                ui.label(format!(
                    "Sent: {} · coalesced: {}",
                    s.status.osc_sent, s.status.coalesced_samples
                ));
                ui.label(format!(
                    "Send errors: {} · heartbeats: {}",
                    s.status.send_errors, s.status.telemetry_sent
                ));
                if let Some(error) = s
                    .status
                    .last_error
                    .as_deref()
                    .filter(|error| error.starts_with("PoseBridge OSC:"))
                {
                    ui.colored_label(crate::theme::WARNING, error);
                }
            }
        })
    });
}
fn dolby_card(ui: &mut egui::Ui, prefs: &mut Preferences, view: &View, editable: bool) -> bool {
    let mut recenter = false;
    ui.push_id("dolby-output",|ui|card(ui,"Dolby Atmos Renderer",|ui| {
        ui.add_enabled_ui(editable,|ui| {
            ui.checkbox(&mut prefs.dolby.enabled,"Enable Dolby output");
            ui.label("Destination · loopback IP:port");
            ui.add(egui::TextEdit::singleline(&mut prefs.dolby.target).desired_width(ui.available_width()));
            ui.add(egui::Slider::new(&mut prefs.dolby.max_rate_hz,1..=200).text("Maximum · Hz"));
        });
        ui.small("OSC head tracker · /ypr · degrees");
        ui.small("In Renderer: enable Headphone, Binaural and OSC head tracker. Use Person view to check direction.");
        let command=Command::RecenterDolby;
        let refusal=view.refusal(&command);
        recenter=ui.add_enabled(refusal.is_none(),egui::Button::new("Set Dolby forward (Recenter)")).clicked();
        ui.small("Wear the sensor, face forward and keep your head upright before pressing. Applies to Dolby on the next new sample.");
        if let Some(status)=view.snapshot.as_ref().and_then(|s|s.status.dolby_osc.as_ref()) {
            ui.label(format!("Sent: {} · coalesced: {} · errors: {}",status.sent,status.coalesced_samples,status.send_errors));
            ui.label(if status.reference_set {"Forward reference set for this connection"}else{"Sensor reference · set forward for this connection"});
            if let Some([y,p,r])=status.last_ypr_deg {ui.label(format!("Last sent: yaw {y:.1}° · pitch {p:.1}° · roll {r:.1}°"));}
            if let Some(error)=&status.last_error {ui.colored_label(crate::theme::WARNING,error);}
        }
        if let Some(reason)=refusal {ui.colored_label(crate::theme::WARNING,reason);}
    }));
    recenter
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn output_is_opt_in_and_reuses_the_core_validation() {
        let mut p = Preferences::default();
        let initial = p.config().unwrap();
        assert!(initial.posebridge.is_none() && initial.dolby.is_none());
        p.enabled = true;
        assert_eq!(p.config().unwrap().posebridge.unwrap().max_rate_hz, 100);
        for target in ["192.168.1.2:9000", "127.0.0.1:0", "invalid"] {
            p.target = target.into();
            assert!(p.config().is_err());
        }
        p.target = "[::1]:9000".into();
        assert!(p.config().is_ok());
        p.max_rate_hz = 201;
        assert!(p.config().is_err());
    }
    #[test]
    fn both_output_cards_draw_and_opening_them_never_requests_forward() {
        for size in [
            eframe::egui::vec2(1000., 700.),
            eframe::egui::vec2(580., 440.),
        ] {
            let context = egui::Context::default();
            crate::theme::install(&context);
            let mut prefs = Preferences {
                enabled: true,
                dolby: DolbyPreferences {
                    enabled: true,
                    ..Default::default()
                },
                ..Default::default()
            };
            let mut snapshot = pb::Controller::new().unwrap().snapshot();
            snapshot.status.last_error = Some("PoseBridge OSC: test send failure".into());
            snapshot.status.dolby_osc = Some(pb::DolbyOscStatus {
                last_error: Some("Dolby test send failure".into()),
                ..Default::default()
            });
            let view = View {
                snapshot: Some(snapshot),
                ..Default::default()
            };
            let mut labels = std::collections::BTreeSet::new();
            let mut red_errors = std::collections::BTreeSet::new();
            for frame in 0..10 {
                let mut requested = false;
                let mut output = context.run_ui(
                    egui::RawInput {
                        screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, size)),
                        time: Some(f64::from(frame) * 0.1),
                        ..Default::default()
                    },
                    |root| {
                        egui::CentralPanel::default().show(root, |ui| {
                            egui::ScrollArea::vertical()
                                .vertical_scroll_offset(frame as f32 * size.y * 0.4)
                                .show(ui, |ui| requested = draw(ui, &mut prefs, &view));
                        });
                    },
                );
                assert!(!requested);
                output.textures_delta.clear();
                for shape in &output.shapes {
                    if let egui::Shape::Text(text) = &shape.shape {
                        labels.insert(text.galley.job.text.clone());
                        if text
                            .galley
                            .job
                            .sections
                            .iter()
                            .any(|section| section.format.color == crate::theme::WARNING)
                        {
                            red_errors.insert(text.galley.job.text.clone());
                        }
                    }
                }
            }
            for label in [
                "PoseBridge",
                "Dolby Atmos Renderer",
                "Set Dolby forward (Recenter)",
            ] {
                assert!(labels.contains(label), "missing {label}");
            }
            for error in [
                "PoseBridge OSC: test send failure",
                "Dolby test send failure",
            ] {
                assert!(red_errors.contains(error), "missing red error: {error}");
            }
        }
    }

    #[test]
    fn dolby_is_independent_and_old_preferences_keep_their_target_and_format() {
        let mut p: Preferences = serde_json::from_str(
            r#"{"enabled":true,"target":"127.0.0.1:9900","format":"Euler","max_rate_hz":50}"#,
        )
        .unwrap();
        assert!(!p.dolby.enabled);
        p.dolby.enabled = true;
        let both = p.config().unwrap();
        assert_eq!(both.posebridge.unwrap().target.port(), 9900);
        assert_eq!(both.dolby.unwrap().target.port(), 8000);
        p.enabled = false;
        assert!(p.config().unwrap().posebridge.is_none());
        assert!(p.config().unwrap().dolby.is_some());
        p.dolby.target = "127.0.0.1:0".into();
        assert!(p.config().is_err());
    }
}
