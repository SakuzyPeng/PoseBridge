#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
#![forbid(unsafe_code)]
mod output;
mod posebridge;
mod preferences;
mod theme;
mod tracking;
use eframe::egui;
use posebridge::service::Service;
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

#[derive(Default)]
struct Options {
    directory: Option<PathBuf>,
    check: bool,
    smoke: bool,
}
impl Options {
    fn parse() -> Result<Self, String> {
        let mut options = Self::default();
        let mut args = std::env::args_os().skip(1);
        while let Some(arg) = args.next() {
            match arg.to_str() {
                Some("--data-dir") => {
                    options.directory =
                        Some(args.next().ok_or("--data-dir requires a path")?.into())
                }
                Some("--check-install") => options.check = true,
                Some("--smoke-test") => options.smoke = true,
                _ => return Err(format!("Unknown argument: {}", arg.to_string_lossy())),
            }
        }
        if options.check && options.smoke {
            return Err("Run installation and window checks separately".into());
        }
        if (options.check || options.smoke) && options.directory.is_none() {
            return Err("Checks require an isolated --data-dir".into());
        }
        Ok(options)
    }
}
fn main() {
    if let Err(error) = run() {
        eprintln!("PoseBridge: {error}");
        if !std::env::args_os().any(|a| a == "--check-install" || a == "--smoke-test") {
            rfd::MessageDialog::new()
                .set_title("PoseBridge")
                .set_description(&error)
                .set_level(rfd::MessageLevel::Error)
                .show();
        }
        std::process::exit(1);
    }
}
fn run() -> Result<(), String> {
    let options = Options::parse()?;
    let (store, prefs, warning) = preferences::Store::open(options.directory)?;
    if options.check {
        let context = egui::Context::default();
        theme::install(&context);
        store.save(&prefs)?;
        let report = serde_json::json!({"application":"PoseBridge","version":env!("CARGO_PKG_VERSION"),"protocol":posebridge_core::PROTOCOL_VERSION,"hardware_accessed":false,"settings_warning":warning});
        std::fs::write(store.path.join("install-check.json"), report.to_string())
            .map_err(|e| e.to_string())?;
        println!("{report}");
        return Ok(());
    }
    #[allow(unused_mut)]
    let mut native = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_app_id(preferences::APP_ID)
            .with_title("PoseBridge")
            .with_inner_size([1000., 700.])
            .with_min_inner_size([580., 440.]),
        persistence_path: Some(store.path.join("window.ron")),
        ..Default::default()
    };
    #[cfg(windows)]
    {
        use eframe::egui_wgpu::{WgpuSetup, WgpuSetupCreateNew, wgpu};
        let mut setup = WgpuSetupCreateNew::without_display_handle();
        setup
            .instance_descriptor
            .backend_options
            .dx12
            .shader_compiler = wgpu::Dx12Compiler::Fxc;
        native.wgpu_options.wgpu_setup = WgpuSetup::CreateNew(setup);
    }
    eframe::run_native(
        "PoseBridge",
        native,
        Box::new(move |cc| {
            theme::install(&cc.egui_ctx);
            Ok(Box::new(Client {
                panel: Default::default(),
                service: Service::new(),
                saved: prefs.clone(),
                prefs,
                store,
                warning,
                changed: None,
                smoke: options.smoke.then(Instant::now),
                frames: 0,
            }))
        }),
    )
    .map_err(|e| e.to_string())
}
struct Client {
    panel: posebridge::ui::Panel,
    service: Service,
    prefs: posebridge::Preferences,
    saved: posebridge::Preferences,
    store: preferences::Store,
    warning: Option<String>,
    changed: Option<Instant>,
    smoke: Option<Instant>,
    frames: usize,
}
impl Client {
    fn persist(&mut self) {
        match self.store.save(&self.prefs) {
            Ok(()) => {
                self.saved = self.prefs.clone();
                self.changed = None;
            }
            Err(e) => {
                self.warning = Some(format!("Settings could not be saved: {e}"));
                self.changed = Some(Instant::now());
            }
        }
    }
}
impl eframe::App for Client {
    fn logic(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        let view = self.service.view();
        self.panel.guard_close(ctx, &view);
        if view.phase.busy() || view.magnetic.is_some() {
            ctx.request_repaint_after(Duration::from_millis(100));
        }
        if self.prefs != self.saved {
            let changed = self.changed.get_or_insert_with(Instant::now);
            if changed.elapsed() >= Duration::from_millis(500) {
                self.persist();
            } else {
                ctx.request_repaint_after(Duration::from_millis(500));
            }
        }
        if let Some(start) = self.smoke {
            if start.elapsed() > Duration::from_secs(2) && self.frames >= 6 {
                let report = serde_json::json!({"frames":self.frames,"hardware_accessed":view.request!=0,"tabs":6,"version":env!("CARGO_PKG_VERSION")});
                if let Err(e) = std::fs::write(
                    self.store.path.join("smoke-report.json"),
                    report.to_string(),
                ) {
                    self.warning = Some(e.to_string());
                }
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            ctx.request_repaint_after(Duration::from_millis(100));
        }
    }
    fn ui(&mut self, root: &mut egui::Ui, _: &mut eframe::Frame) {
        let ctx = root.ctx().clone();
        if self.smoke.is_some() {
            self.panel.smoke_tab(self.frames % 6);
        }
        let head = tracking::TrackingSnapshot::from_sample(
            self.service
                .sample
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .as_ref(),
            Instant::now(),
        );
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(theme::BACKGROUND).inner_margin(16))
            .show(root, |ui| {
                if let Some(w) = &self.warning {
                    ui.colored_label(theme::WARNING, w);
                    ui.separator();
                }
                self.panel
                    .contents(ui, &mut self.prefs, &head, &self.service, true);
            });
        self.panel.draw_close_prompt(&ctx, &self.service);
        self.frames += 1;
    }
    fn save(&mut self, _: &mut dyn eframe::Storage) {
        self.persist();
    }
}
