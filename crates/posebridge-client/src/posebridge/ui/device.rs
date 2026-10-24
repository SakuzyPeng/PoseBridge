use super::{
    Command, Panel, Phase, Preferences, Service, Settings, Tab, TrackingSnapshot, Transport, View,
    axis_name, command_effect, command_label, configuration, egui, format_label, motion_profile,
    operation_result, pb, phase_tone, section, wide,
};
use crate::posebridge::Input;

impl Panel {
    pub(super) fn sync_settings(&mut self, prefs: &mut Preferences, view: &View) {
        let device = (prefs.device.transport, prefs.device.id.clone());
        if self.settings_device.as_ref() != Some(&device) {
            self.settings_device = Some(device);
            self.baseline = None;
            self.observation = None;
            self.draft = Settings::default();
            self.prepare_stream = false;
            self.prepare_request = None;
            self.confirmation = None;
        }
        let Some(current) = Self::current_settings(prefs, view) else {
            return;
        };
        let stamp = (
            prefs.device.id.clone(),
            view.snapshot
                .as_ref()
                .and_then(|s| s.descriptor.device.observed_unix_ms),
        );
        let applied = view
            .apply
            .as_ref()
            .is_some_and(|a| a.complete && a.target == current);
        // Completion may arrive after this readback timestamp was already drawn.
        // A previous successful operation cannot authorize a newly staged preset.
        if applied && self.prepare_stream && self.prepare_request == Some(view.request) {
            prefs.device.input = Input::Automatic;
            self.prepare_stream = false;
            self.prepare_request = None;
        }
        if self.observation.as_ref() == Some(&stamp) {
            return;
        }
        if self.baseline.is_none_or(|before| before == self.draft) || applied {
            self.draft = current;
        }
        self.baseline = Some(current);
        self.observation = Some(stamp);
    }

    fn current_settings(prefs: &Preferences, view: &View) -> Option<Settings> {
        view.device
            .as_ref()
            .filter(|d| d.id == prefs.device.id && d.transport == prefs.device.transport)?;
        Settings::observed(&view.snapshot.as_ref()?.descriptor.device)
    }

    pub(super) fn connection_bar(
        &mut self,
        ui: &mut egui::Ui,
        prefs: &mut Preferences,
        _head: &TrackingSnapshot,
        service: &Service,
        view: &View,
        enabled: bool,
    ) {
        ui.columns(2, |columns| {
            let name = if prefs.device.id.is_empty() {
                "No device selected"
            } else if prefs.device.name.is_empty() {
                &prefs.device.id
            } else {
                &prefs.device.name
            };
            columns[0].add(egui::Label::new(egui::RichText::new(name).strong()).truncate());
            columns[0].horizontal_wrapped(|ui| {
                ui.colored_label(phase_tone(view.phase), view.label());
                if view.phase == Phase::Tracking
                    && let Some(s) = &view.snapshot
                {
                    ui.small(format!("{:.0} Hz", s.status.actual_rate_hz));
                }
            });
            columns[1].horizontal(|ui| {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if view.phase == Phase::Tracking {
                        if ui.button("Disconnect").clicked() {
                            self.send(service, Command::Stop);
                        }
                    } else if view.phase.busy() {
                        // A session is closed rather than cancelled. Closing
                        // also ends a calibration it started, but only an End
                        // calibration clears the warning.
                        let label = if view.phase == Phase::Magnetic {
                            "Close session"
                        } else {
                            "Cancel operation"
                        };
                        if ui
                            .add_enabled(view.phase != Phase::Stopping, egui::Button::new(label))
                            .clicked()
                        {
                            self.send(service, Command::Stop);
                        }
                    } else if ui
                        .add_enabled(
                            enabled
                                && view.magnetic.is_none()
                                && configuration(&prefs.device, true).is_ok()
                                && prefs.output.config().is_ok(),
                            egui::Button::new("Connect"),
                        )
                        .clicked()
                    {
                        prefs.remember();
                        match prefs.output.config() {
                            Ok(osc) => {
                                self.send(service, Command::Connect(prefs.device.clone(), osc))
                            }
                            Err(error) => self.error = Some(error),
                        }
                    }
                });
            });
        });
        crate::output::summary(ui, &prefs.output, view);
        if let Some(device) = &view.magnetic {
            ui.horizontal_wrapped(|ui| {
                ui.colored_label(crate::theme::WARNING, "Magnetic calibration may be active");
                // Also inside an open magnetic session, where the service sends
                // the stop into the session instead of refusing it as busy.
                let end = Command::Write(device.clone(), pb::DeviceCommand::MagStop);
                if ui
                    .add_enabled(view.admits(&end), egui::Button::new("End calibration"))
                    .clicked()
                {
                    self.send(service, end);
                }
            });
        } else if !enabled {
            ui.small("Tracking is unavailable while a device operation is running.");
        } else if let Err(error) = configuration(&prefs.device, true) {
            ui.colored_label(
                crate::theme::WARNING,
                if prefs.device.id.is_empty() {
                    "Choose a sensor and mounting on the Device page."
                } else {
                    super::mounting::selection_problem(prefs.device.mounting).unwrap_or_else(|| {
                        error.strip_prefix("invalid argument: ").unwrap_or(&error)
                    })
                },
            );
        }
        if let Some(error) = self.error.as_ref().or(view.error.as_ref()) {
            ui.add(
                egui::Label::new(egui::RichText::new(error).color(crate::theme::WARNING))
                    .truncate(),
            )
            .on_hover_text(error);
        }
    }

    pub(super) fn device(
        &mut self,
        ui: &mut egui::Ui,
        prefs: &mut Preferences,
        service: &Service,
        view: &View,
    ) {
        if wide(ui) {
            ui.columns(2, |columns| {
                self.device_selection(&mut columns[0], prefs, service, view);
                self.device_settings(&mut columns[1], prefs, view);
            });
        } else {
            self.device_selection(ui, prefs, service, view);
            ui.add_space(12.);
            self.device_settings(ui, prefs, view);
        }
    }

    fn device_selection(
        &mut self,
        ui: &mut egui::Ui,
        prefs: &mut Preferences,
        service: &Service,
        view: &View,
    ) {
        self.source_selection(ui, prefs, service, view);
        self.sync_settings(prefs, view);
        Self::mounting_input(ui, prefs, !view.phase.busy() && view.magnetic.is_none());
    }

    fn source_selection(
        &mut self,
        ui: &mut egui::Ui,
        prefs: &mut Preferences,
        service: &Service,
        view: &View,
    ) {
        let editable = !view.phase.busy() && view.magnetic.is_none();
        section(ui, "Sensor", |ui| {
            ui.add_enabled_ui(editable, |ui| {
                let mut transport = prefs.device.transport;
                ui.horizontal_wrapped(|ui| {
                    ui.selectable_value(&mut transport, Transport::Ble, "Bluetooth LE");
                    ui.selectable_value(&mut transport, Transport::Usb, "USB serial");
                    if ui.button("Scan").clicked() {
                        self.send(service, Command::Scan(transport));
                    }
                });
                if transport != prefs.device.transport {
                    prefs.select(transport, String::new(), String::new());
                }
                egui::ComboBox::from_id_salt("posebridge-device")
                    .width(ui.available_width())
                    .selected_text(if prefs.device.id.is_empty() {
                        "Choose device"
                    } else if prefs.device.name.is_empty() {
                        &prefs.device.id
                    } else {
                        &prefs.device.name
                    })
                    .show_ui(ui, |ui| {
                        for d in &view.devices {
                            let kind = if d.transport == pb::TransportKind::Ble {
                                Transport::Ble
                            } else {
                                Transport::Usb
                            };
                            if kind == transport
                                && ui
                                    .selectable_label(
                                        prefs.device.id == d.id,
                                        format!("{} · {}", d.name, d.id),
                                    )
                                    .clicked()
                            {
                                prefs.select(kind, d.id.clone(), d.name.clone());
                            }
                        }
                        for d in prefs
                            .remembered
                            .clone()
                            .into_iter()
                            .filter(|d| d.transport == transport)
                        {
                            if ui
                                .selectable_label(
                                    prefs.device.id == d.id,
                                    format!("Remembered: {}", d.id),
                                )
                                .clicked()
                            {
                                prefs.select(d.transport, d.id, d.name);
                            }
                        }
                    });
                if transport == Transport::Usb {
                    ui.horizontal(|ui| {
                        ui.label("Baud");
                        ui.add(egui::DragValue::new(&mut prefs.device.baud).range(1..=921_600));
                    });
                }
                #[cfg(target_os = "windows")]
                if transport == Transport::Ble {
                    ui.checkbox(
                        &mut prefs.device.throughput,
                        "Request higher BLE throughput (Windows 11+)",
                    );
                }
            })
        });
    }

    fn mounting_input(ui: &mut egui::Ui, prefs: &mut Preferences, editable: bool) {
        ui.add_space(12.);
        section(ui, "Mounting", |ui| {
            ui.add_enabled_ui(editable, |ui| {
                ui.small("Sensor axes pointing toward head right, forward and up");
                ui.columns(3, |columns| {
                    for (index, label) in ["Right", "Forward", "Up"].into_iter().enumerate() {
                        let ui = &mut columns[index];
                        ui.label(label);
                        egui::ComboBox::from_id_salt(("mount", index))
                            .height(8.0 * (ui.spacing().interact_size.y + ui.spacing().item_spacing.y))
                            .width(ui.available_width())
                            .truncate()
                            .selected_text(axis_name(prefs.device.mounting[index]))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut prefs.device.mounting[index], 0, "Clear selection");
                                ui.separator();
                                for axis in super::mounting::SENSOR_AXES {
                                    let allowed = super::mounting::allows_axis(prefs.device.mounting, index, axis);
                                    ui.add_enabled_ui(allowed, |ui| {
                                        ui.selectable_value(
                                            &mut prefs.device.mounting[index],
                                            axis,
                                            axis_name(axis),
                                        )
                                    }).inner.on_disabled_hover_ui(|ui| {
                                        ui.colored_label(crate::theme::WARNING, "Conflicts with the other selected mounting directions.");
                                    });
                                }
                            });
                    }
                });
                ui.horizontal_wrapped(|ui| {
                    if ui.add_enabled(prefs.device.mounting != [0; 3], egui::Button::new("Clear choices")).clicked() {
                        prefs.device.mounting = [0; 3];
                    }
                    ui.small("To change a full mapping, clear a direction first.");
                });
                ui.small("Conflicting options are disabled. Check the three motions on Tracking.");
            })
            ;
            if let Some(problem) = super::mounting::selection_problem(prefs.device.mounting) {
                ui.colored_label(crate::theme::WARNING, problem);
            }
        });
        ui.add_space(12.);
        section(ui, "Acquisition", |ui| {
            ui.add_enabled_ui(editable, |ui| {
                egui::ComboBox::from_id_salt("posebridge-input")
                    .width(ui.available_width())
                    .selected_text(if prefs.device.input == Input::Automatic {
                        "Read current stream format"
                    } else {
                        "Poll quaternion registers"
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut prefs.device.input,
                            Input::Automatic,
                            "Read current stream format",
                        );
                        ui.selectable_value(
                            &mut prefs.device.input,
                            Input::RegisterQuaternion,
                            "Poll quaternion registers",
                        );
                    });
                ui.small("Device selection and mounting take effect on the next connection.");
            })
        });
    }

    fn device_settings(&mut self, ui: &mut egui::Ui, prefs: &Preferences, view: &View) {
        let current = Self::current_settings(prefs, view);
        let editable = self.baseline.is_some()
            && matches!(view.phase, Phase::Idle | Phase::Failed | Phase::Tracking)
            && view.magnetic.is_none();
        section(ui, "Stream", |ui| {
            ui.add_enabled_ui(editable, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Sample rate");
                    egui::ComboBox::from_id_salt("sensor-rate")
                        .width(140.)
                        .selected_text(choice(
                            format!("{} Hz", self.draft.rate),
                            self.draft.rate,
                            self.draft.rate,
                            current.map(|s| s.rate),
                        ))
                        .show_ui(ui, |ui| {
                            for rate in [1, 2, 5, 10, 20, 50, 100, 200] {
                                let label = choice(
                                    format!("{rate} Hz"),
                                    rate,
                                    self.draft.rate,
                                    current.map(|s| s.rate),
                                );
                                ui.selectable_value(&mut self.draft.rate, rate, label);
                            }
                        });
                });
                ui.label("Output format");
                egui::ComboBox::from_id_salt("sensor-format")
                    .width(ui.available_width())
                    .selected_text(choice(
                        format_label(self.draft.format),
                        self.draft.format,
                        self.draft.format,
                        current.map(|s| s.format),
                    ))
                    .show_ui(ui, |ui| {
                        for format in [0x61, 0x81, 0x84, 0xa4]
                            .into_iter()
                            .chain(pb::FULL_INERTIAL_20HZ_VALIDATED.then_some(0xe4))
                        {
                            let label = choice(
                                format_label(format),
                                format,
                                self.draft.format,
                                current.map(|s| s.format),
                            );
                            ui.selectable_value(&mut self.draft.format, format, label);
                        }
                    });
                ui.add_space(6.);
                let profile = motion_profile((self.draft.rate == 20).then_some(7)) as u16;
                ui.small(format!(
                    "{} · keeps the selected rate",
                    format_label(profile)
                ));
                if ui
                    .add_enabled(
                        editable
                            && (self.draft.format != profile
                                || prefs.device.input != Input::Automatic),
                        egui::Button::new("Prepare timestamp + gyro"),
                    )
                    .clicked()
                {
                    self.draft.format = profile;
                    self.prepare_stream = true;
                    self.prepare_request = None;
                }
                ui.small("Stages the format and stream acquisition. Apply below to use them.");
            })
        });
        ui.add_space(12.);
        section(ui, "Fusion", |ui| {
            ui.add_enabled_ui(editable, |ui| {
            ui.horizontal_wrapped(|ui| {
                for (mode, text) in [(false, "Nine-axis"), (true, "Six-axis")] {
                    let label = choice(text, mode, self.draft.six_axis, current.map(|s| s.six_axis));
                    ui.selectable_value(&mut self.draft.six_axis, mode, label);
                }
            });
            ui.small("Nine-axis also uses the magnetometer. Applying a mode can change the sensor's reference direction.");
        })
        });
    }

    pub(super) fn settings_actions(
        &mut self,
        ui: &mut egui::Ui,
        prefs: &Preferences,
        service: &Service,
        view: &View,
        feedback_height: f32,
    ) {
        ui.separator();
        let idle = !view.phase.busy() && view.magnetic.is_none();
        let valid = configuration(&prefs.device, false).is_ok();
        let count = self
            .baseline
            .map_or(0, |before| self.draft.differences(before))
            + usize::from(self.prepare_stream && prefs.device.input != Input::Automatic);
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(idle && valid, egui::Button::new("Read settings"))
                .clicked()
            {
                self.send(service, Command::Inspect(prefs.device.clone()));
            }
            if ui
                .add_enabled(count > 0 && idle, egui::Button::new("Discard edits"))
                .clicked()
            {
                if let Some(before) = self.baseline {
                    self.draft = before;
                }
                self.prepare_stream = false;
                self.prepare_request = None;
            }
            if ui
                .add_enabled(
                    idle && valid && count > 0 && self.draft.validate().is_ok(),
                    egui::Button::new("Apply changes"),
                )
                .clicked()
            {
                self.send(
                    service,
                    Command::Apply(
                        prefs.device.clone(),
                        self.baseline.expect("enabled with observed settings"),
                        self.draft,
                    ),
                );
                if self.error.is_none() && self.prepare_stream {
                    self.prepare_request = Some(service.view().request);
                }
            }
            ui.small(format!("{count} pending"));
        });
        // Let feedback establish its natural height, independent of the bottom
        // panel's cached height. Otherwise a new result can get stuck in a
        // zero-height scroll area and never enlarge that panel.
        let feedback_rect = egui::Rect::from_min_size(
            ui.cursor().min,
            egui::vec2(ui.available_width(), feedback_height),
        );
        ui.scope_builder(egui::UiBuilder::new().max_rect(feedback_rect), |ui| {
            egui::ScrollArea::vertical()
                .id_salt("pose-settings-feedback")
                .max_height(feedback_height)
                .auto_shrink([false, true])
                .show(ui, |ui| self.settings_feedback(ui, view));
        });
    }

    fn settings_feedback(&self, ui: &mut egui::Ui, view: &View) {
        ui.horizontal_wrapped(|ui| {
            ui.colored_label(crate::theme::SUCCESS, "✓ Device value");
            ui.colored_label(crate::theme::ACCENT, "• Pending change");
        });
        ui.small(if self.baseline.is_none() {
            "Read settings to edit."
        } else if view.phase == Phase::Tracking {
            "Disconnect above to apply."
        } else {
            "Apply writes and verifies; Flash save stays separate."
        });
        if let Some(progress) = &view.apply {
            if let Some(error) = &progress.error {
                ui.colored_label(crate::theme::WARNING, error);
            } else {
                ui.colored_label(
                    if progress.complete {
                        crate::theme::SUCCESS
                    } else {
                        crate::theme::MUTED
                    },
                    progress.step,
                );
            }
            if !progress.verified.is_empty() && !progress.complete {
                ui.small(format!("Readback passed: {}", progress.verified.join(", ")));
            }
        } else if let Err(error) = self.draft.validate() {
            ui.colored_label(crate::theme::WARNING, error);
        }
    }

    pub(super) fn maintenance(&mut self, ui: &mut egui::Ui, prefs: &Preferences, view: &View) {
        let editable = !view.phase.busy()
            && view.magnetic.is_none()
            && configuration(&prefs.device, false).is_ok();
        if wide(ui) {
            ui.columns(2, |columns| {
                self.maintenance_group(&mut columns[0], prefs, editable, false);
                self.maintenance_group(&mut columns[1], prefs, editable, true);
            });
        } else {
            self.maintenance_group(ui, prefs, editable, false);
            ui.add_space(12.);
            self.maintenance_group(ui, prefs, editable, true);
        }
        operation_result(ui, view);
    }
    fn maintenance_group(
        &mut self,
        ui: &mut egui::Ui,
        prefs: &Preferences,
        editable: bool,
        persistent: bool,
    ) {
        section(
            ui,
            if persistent {
                "Saved device settings"
            } else {
                "Calibration and reference"
            },
            |ui| {
                ui.small(if persistent {
                    "These operations write the sensor's saved settings."
                } else {
                    "These operations do not send Flash save. Listening Recenter is in the top bar."
                });
                let commands: &[pb::DeviceCommand] = if persistent {
                    &[
                        pb::DeviceCommand::Save,
                        pb::DeviceCommand::AngleReference,
                        pb::DeviceCommand::ResetDefaults,
                    ]
                } else {
                    &[
                        pb::DeviceCommand::ZeroYaw,
                        pb::DeviceCommand::AccelCalibrate,
                    ]
                };
                ui.add_enabled_ui(editable, |ui| {
                    for command in commands {
                        let label =
                            egui::RichText::new(command_label(command)).color(if persistent {
                                crate::theme::WARNING
                            } else {
                                crate::theme::TEXT
                            });
                        if ui.button(label).clicked() {
                            self.confirmation = Some((prefs.device.clone(), command.clone()));
                        }
                    }
                });
                if !persistent {
                    // A start from here would calibrate blind: only the
                    // session shows the field while it runs.
                    ui.add_space(4.);
                    if ui.button("Magnetic calibration…").clicked() {
                        self.tab = Tab::Magnetic;
                    }
                    ui.small("Has its own page, which reads the field while it runs.");
                }
            },
        );
    }
    pub(super) fn confirm_operation(
        &mut self,
        context: &egui::Context,
        service: &Service,
        view: &View,
    ) {
        let Some((device, command)) = self.confirmation.clone() else {
            return;
        };
        let write = Command::Write(device.clone(), command.clone());
        // The service decides, not the phase: a start or save for the open
        // magnetic session goes into it while the phase reads busy.
        let refusal = view.refusal(&write);
        let response =
            egui::Modal::new(egui::Id::new("pose-device-confirmation")).show(context, |ui| {
                ui.set_max_width(420.);
                ui.strong(command_label(&command));
                ui.label(format!("Device: {}", device.id));
                ui.label(command_effect(&command));
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(refusal.is_none(), egui::Button::new("Confirm operation"))
                        .clicked()
                    {
                        self.send(service, write);
                        self.confirmation = None;
                    }
                    if ui.button("Cancel").clicked() {
                        self.confirmation = None;
                    }
                });
                if let Some(reason) = &refusal {
                    ui.small(reason);
                }
            });
        if response.should_close() {
            self.confirmation = None;
        }
    }
}

fn choice<T: PartialEq + Copy>(
    text: impl Into<String>,
    value: T,
    selected: T,
    current: Option<T>,
) -> egui::RichText {
    let text = text.into();
    if current.as_ref() == Some(&value) {
        egui::RichText::new(format!("{text} ✓")).color(crate::theme::SUCCESS)
    } else if value == selected && current.is_some() {
        egui::RichText::new(format!("{text} •")).color(crate::theme::ACCENT)
    } else {
        egui::RichText::new(text)
    }
}

#[cfg(test)]
mod mounting_choice_tests {
    use super::*;

    fn frame(
        context: &egui::Context,
        prefs: &mut Preferences,
        events: Vec<egui::Event>,
    ) -> egui::FullOutput {
        let mut output = context.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(800., 600.),
                )),
                events,
                ..Default::default()
            },
            |root| {
                egui::CentralPanel::default()
                    .show(root, |ui| Panel::mounting_input(ui, prefs, true));
            },
        );
        output.textures_delta.clear();
        output
    }

    fn position(output: &egui::FullOutput, label: &str) -> egui::Pos2 {
        output
            .shapes
            .iter()
            .rev()
            .find_map(|shape| match &shape.shape {
                egui::Shape::Text(text) if text.galley.job.text == label => {
                    Some(text.galley.rect.translate(text.pos.to_vec2()).center())
                }
                _ => None,
            })
            .unwrap_or_else(|| {
                panic!(
                    "Missing label {label}; drawn: {:?}",
                    output
                        .shapes
                        .iter()
                        .filter_map(|shape| match &shape.shape {
                            egui::Shape::Text(text) => Some(text.galley.job.text.as_str()),
                            _ => None,
                        })
                        .collect::<Vec<_>>()
                )
            })
    }

    fn click(
        context: &egui::Context,
        prefs: &mut Preferences,
        pos: egui::Pos2,
    ) -> egui::FullOutput {
        for pressed in [true, false] {
            frame(
                context,
                prefs,
                vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed,
                        modifiers: Default::default(),
                    },
                ],
            );
        }
        frame(context, prefs, Vec::new())
    }

    #[test]
    fn clicking_a_conflicting_option_does_not_store_it_and_clearing_stays_available() {
        let context = egui::Context::default();
        crate::theme::install(&context);
        let mut prefs = Preferences::default();
        prefs.device.mounting = [1, 2, 0];
        frame(&context, &mut prefs, Vec::new());
        let output = frame(&context, &mut prefs, Vec::new());
        let output = click(&context, &mut prefs, position(&output, "Choose axis"));
        let output = click(&context, &mut prefs, position(&output, "−Z"));
        assert_eq!(prefs.device.mounting, [1, 2, 0]);
        // A disabled click may close the popup; reopen it explicitly.
        let has_popup = output.shapes.iter().any(|shape| matches!(&shape.shape, egui::Shape::Text(text) if text.galley.job.text == "Clear selection"));
        let output = if has_popup {
            output
        } else {
            click(&context, &mut prefs, position(&output, "Choose axis"))
        };
        let output = click(&context, &mut prefs, position(&output, "+Z"));
        assert_eq!(prefs.device.mounting, [1, 2, 3]);
        let output = click(&context, &mut prefs, position(&output, "+Z"));
        click(&context, &mut prefs, position(&output, "Clear selection"));
        assert_eq!(prefs.device.mounting, [1, 2, 0]);
    }

    #[test]
    fn a_saved_conflict_is_explained_in_red_without_changing_the_mapping() {
        let context = egui::Context::default();
        crate::theme::install(&context);
        let mut prefs = Preferences::default();
        prefs.device.mounting = [-1, 2, 3];
        let before = prefs.device.mounting;
        let output = frame(&context, &mut prefs, Vec::new());
        assert_eq!(prefs.device.mounting, before);
        let message = super::super::mounting::selection_problem(before).unwrap();
        assert!(output.shapes.iter().any(|shape| {
            match &shape.shape {
                egui::Shape::Text(text) if text.galley.job.text == message => text
                    .galley
                    .job
                    .sections
                    .iter()
                    .any(|section| section.format.color == crate::theme::WARNING),
                _ => false,
            }
        }));
        let output = click(&context, &mut prefs, position(&output, "Clear choices"));
        assert_eq!(prefs.device.mounting, [0; 3]);
        assert!(output.shapes.iter().any(|shape| matches!(&shape.shape, egui::Shape::Text(text) if text.galley.job.text == "Choose axis")));
    }
}
