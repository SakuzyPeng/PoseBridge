//! Independent settings and an exclusive data-directory lock; hardware state is never persisted.
use crate::posebridge::Preferences;
use serde::{Deserialize, Serialize};
use std::{
    fs::{self, File, OpenOptions},
    io::Write,
    path::{Path, PathBuf},
};
pub const APP_ID: &str = "org.posebridge.client";
#[derive(Serialize, Deserialize)]
struct Document {
    version: u32,
    preferences: Preferences,
}
pub struct Store {
    pub path: PathBuf,
    _lock: File,
    writable: bool,
}
impl Store {
    pub fn open(explicit: Option<PathBuf>) -> Result<(Self, Preferences, Option<String>), String> {
        let path = explicit
            .or_else(|| eframe::storage_dir(APP_ID))
            .ok_or("Cannot locate the application data directory")?;
        fs::create_dir_all(&path).map_err(|e| e.to_string())?;
        let lock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(path.join("client.lock"))
            .map_err(|e| e.to_string())?;
        lock.try_lock().map_err(|e| {
            format!(
                "PoseBridge is already using this data directory, or it cannot be locked: {e}\n{}",
                path.display()
            )
        })?;
        let mut store = Self {
            path,
            _lock: lock,
            writable: true,
        };
        let file = store.path.join("settings.json");
        let (mut prefs, warning) = if file.exists() {
            match fs::read(&file)
                .map_err(|e| e.to_string())
                .and_then(|bytes| {
                    serde_json::from_slice::<Document>(&bytes).map_err(|e| e.to_string())
                })
                .and_then(|doc| {
                    if matches!(doc.version, 1 | 2) {
                        Ok(doc.preferences)
                    } else {
                        Err(format!("Unsupported preferences version {}", doc.version))
                    }
                }) {
                Ok(p) => (p, None),
                Err(e) => {
                    store.writable = false;
                    (
                        Preferences::default(),
                        Some(format!(
                            "Settings could not be loaded; defaults are temporary and the original file will be preserved. {e}"
                        )),
                    )
                }
            }
        } else {
            (Preferences::default(), None)
        };
        prefs.validate();
        Ok((store, prefs, warning))
    }
    pub fn save(&self, prefs: &Preferences) -> Result<(), String> {
        if !self.writable {
            return Ok(());
        }
        let bytes = serde_json::to_vec_pretty(&Document {
            version: 2,
            preferences: prefs.clone(),
        })
        .map_err(|e| e.to_string())?;
        atomic_write(&self.path.join("settings.json"), &bytes)
    }
}
fn atomic_write(path: &Path, bytes: &[u8]) -> Result<(), String> {
    let mut file =
        tempfile::NamedTempFile::new_in(path.parent().ok_or("Missing parent directory")?)
            .map_err(|e| e.to_string())?;
    file.write_all(bytes)
        .and_then(|_| file.as_file().sync_all())
        .map_err(|e| e.to_string())?;
    file.persist(path).map_err(|e| e.to_string())?;
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roundtrip_is_independent_and_locked() {
        let dir = tempfile::tempdir().unwrap();
        let (s, mut p, w) = Store::open(Some(dir.path().into())).unwrap();
        assert!(w.is_none());
        assert!(Store::open(Some(dir.path().into())).is_err());
        p.device.id = "remembered sensor".into();
        p.output.enabled = true;
        s.save(&p).unwrap();
        drop(s);
        let (_s, q, _) = Store::open(Some(dir.path().into())).unwrap();
        assert_eq!(p, q);
    }
    #[test]
    fn version_one_migrates_to_two_with_dolby_disabled() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("settings.json");
        fs::write(&path, r#"{"version":1,"preferences":{"output":{"enabled":true,"target":"127.0.0.1:9900","format":"Euler","max_rate_hz":50}}}"#).unwrap();
        let (s, p, w) = Store::open(Some(dir.path().into())).unwrap();
        assert!(w.is_none());
        assert!(p.output.enabled);
        assert_eq!(p.output.target, "127.0.0.1:9900");
        assert!(!p.output.dolby.enabled);
        s.save(&p).unwrap();
        let value: serde_json::Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(value["version"], 2);
        assert_eq!(value["preferences"]["output"]["dolby"]["enabled"], false);
    }

    #[test]
    fn invalid_or_future_settings_are_not_overwritten() {
        for content in ["broken", r#"{"version":99,"preferences":{}}"#] {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("settings.json");
            fs::write(&path, content).unwrap();
            let (s, p, w) = Store::open(Some(dir.path().into())).unwrap();
            assert!(w.is_some());
            s.save(&p).unwrap();
            assert_eq!(fs::read_to_string(path).unwrap(), content);
        }
    }
}
