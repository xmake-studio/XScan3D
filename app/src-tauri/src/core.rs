//! The application core: every piece of state, shared by the device thread,
//! the scan session, the background jobs and the UI commands.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use parking_lot::{Mutex, RwLock};
use serde::Serialize;
use tauri::{AppHandle, Emitter, Manager};

use crate::clouds::{self, CloudCache, CloudEntry};
use crate::devcalib::{self, DeviceCalib};
use crate::device::DeviceState;
use crate::jobs::{MergeJob, MeshCacheEntry};
use crate::library::{Library, LibraryIndex, ScanKind, ScanMeta};
use crate::session::ScanState;
use crate::settings::Settings;

pub struct Core {
    pub app: AppHandle,
    pub settings: RwLock<Settings>,
    pub settings_path: PathBuf,
    pub documents: Option<PathBuf>,
    pub library: Mutex<Library>,
    pub device: Mutex<DeviceState>,
    pub scan: Mutex<ScanState>,
    pub clouds: Mutex<CloudCache>,
    /// Bumped whenever the mount geometry changes; clouds carry it.
    pub geometry_version: AtomicU64,
    pub merge: Mutex<Option<MergeJob>>,
    pub merge_cancel: Arc<AtomicBool>,
    pub mesh: Mutex<Option<MeshCacheEntry>>,
    pub mesh_busy: AtomicBool,
    pub mesh_cancel: Arc<AtomicBool>,
    pub calib_busy: AtomicBool,
    pub calib_cancel: Arc<AtomicBool>,
    pub shutting_down: AtomicBool,
}

pub type CoreRef = Arc<Core>;

#[derive(Serialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct LibraryDto {
    pub root: String,
    pub index: LibraryIndex,
}

impl Core {
    pub fn new(app: AppHandle) -> CoreRef {
        let config_dir = app.path().app_config_dir().unwrap_or_else(|_| PathBuf::from("."));
        let settings_path = config_dir.join("settings.json");
        let settings = Settings::load(&settings_path);
        let documents = app.path().document_dir().ok();
        let root = settings.library_path(documents.clone());
        let library = Library::open(&root).unwrap_or_else(|_| {
            // Fall back to the app's own data folder if Documents is unusable.
            let alt = app.path().app_data_dir().unwrap_or_else(|_| PathBuf::from(".")).join("Library");
            Library::open(&alt).expect("cannot create a scan library anywhere")
        });
        let _ = settings.save(&settings_path);
        Arc::new(Core {
            app,
            settings: RwLock::new(settings),
            settings_path,
            documents,
            library: Mutex::new(library),
            device: Mutex::new(DeviceState::default()),
            scan: Mutex::new(ScanState::default()),
            clouds: Mutex::new(CloudCache::new(30_000_000)),
            geometry_version: AtomicU64::new(1),
            merge: Mutex::new(None),
            merge_cancel: Arc::new(AtomicBool::new(false)),
            mesh: Mutex::new(None),
            mesh_busy: AtomicBool::new(false),
            mesh_cancel: Arc::new(AtomicBool::new(false)),
            calib_busy: AtomicBool::new(false),
            calib_cancel: Arc::new(AtomicBool::new(false)),
            shutting_down: AtomicBool::new(false),
        })
    }

    pub fn emit<S: Serialize + Clone>(&self, event: &str, payload: S) {
        let _ = self.app.emit(event, payload);
    }

    pub fn library_dto(&self) -> LibraryDto {
        let lib = self.library.lock();
        LibraryDto { root: lib.root.to_string_lossy().into_owned(), index: lib.index.clone() }
    }

    pub fn emit_library(&self) {
        let dto = self.library_dto();
        self.emit("library", dto);
    }

    pub fn save_library(&self) {
        let lib = self.library.lock();
        if let Err(e) = lib.save() {
            log::error(&format!("library save failed: {e}"));
        }
    }

    /// Applies a settings patch; returns the new settings. Geometry changes
    /// invalidate the built clouds, and a changed calibration is queued for
    /// the scanner's flash.
    pub fn update_settings(&self, patch: &serde_json::Value) -> Settings {
        self.apply_settings(patch, false)
    }

    /// The same for a change that came from the scanner itself: nothing to
    /// write back, and the interface did not ask for it, so it is told.
    pub fn update_settings_from_device(&self, patch: &serde_json::Value) -> Settings {
        let s = self.apply_settings(patch, true);
        self.emit("settings", s.clone());
        s
    }

    fn apply_settings(&self, patch: &serde_json::Value, from_device: bool) -> Settings {
        let mut local_calib = false;
        let (old, new) = {
            let mut s = self.settings.write();
            let old = s.clone();
            let mut new = s.merged(patch);
            if !from_device && DeviceCalib::from_mount(&old.mount) != DeviceCalib::from_mount(&new.mount) {
                new.device.calib_pending = true;
                local_calib = true;
            }
            *s = new.clone();
            (old, new)
        };
        let _ = new.save(&self.settings_path);
        if local_calib {
            devcalib::on_local_change(self);
        }
        if old.mount != new.mount {
            self.clouds.lock().clear_raw();
            *self.mesh.lock() = None;
            let v = self.geometry_version.fetch_add(1, Ordering::SeqCst) + 1;
            self.emit("geometry", v);
        }
        if old.library_dir != new.library_dir {
            let root = new.library_path(self.documents.clone());
            match Library::open(&root) {
                Ok(lib) => {
                    *self.library.lock() = lib;
                    self.clouds.lock().clear_raw();
                    self.recover_orphans();
                    self.emit_library();
                }
                Err(e) => log::error(&format!("cannot open library {root:?}: {e}")),
            }
        }
        new
    }

    /// The built cloud of a scan: from the cache, else built now. A fresh
    /// microstep fit is written back to the library.
    pub fn cloud(&self, id: &str) -> Result<Arc<CloudEntry>, String> {
        let mount = self.settings.read().mount.clone();
        let fp = mount.fingerprint();
        if let Some(e) = self.clouds.lock().get(id, fp) {
            return Ok(e);
        }
        let (meta, path) = {
            let lib = self.library.lock();
            let meta = lib.get(id).cloned().ok_or_else(|| format!("unknown scan {id}"))?;
            let path = lib.path_of(&meta);
            (meta, path)
        };
        let built = clouds::build_scan(&meta, &path, &mount)?;
        let entry = Arc::new(built.entry);
        let mut changed = false;
        {
            let mut lib = self.library.lock();
            if let Some(m) = lib.get_mut(id) {
                if let Some(c) = built.new_microstep {
                    m.microstep = Some(c);
                    changed = true;
                }
                if m.points != entry.len() {
                    m.points = entry.len();
                    changed = true;
                }
            }
        }
        if changed {
            self.save_library();
        }
        self.clouds.lock().put(entry.clone());
        Ok(entry)
    }

    /// Adopts scan files the index does not know about: recordings a crash
    /// interrupted, and files copied into the library folder by hand.
    pub fn recover_orphans(&self) {
        let orphans = self.library.lock().orphans();
        if orphans.is_empty() {
            return;
        }
        let mount = self.settings.read().mount.clone();
        for path in orphans {
            let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("").to_string();
            let lower = name.to_lowercase();
            let (kind, partial) = if lower.ends_with(".ply") {
                (ScanKind::Ply, false)
            } else if lower.ends_with(".bin.part") {
                (ScanKind::Device, true)
            } else {
                (ScanKind::Bin, false)
            };
            // A recovered recording gets its final name.
            let path = if partial {
                let fin = path.with_extension(""); // strips ".part"
                if std::fs::rename(&path, &fin).is_err() {
                    continue;
                }
                fin
            } else {
                path
            };
            let id = path
                .file_stem()
                .and_then(|s| s.to_str())
                .map(|s| s.trim_end_matches(".bin").to_string())
                .unwrap_or_else(crate::library::new_id);
            let created = std::fs::metadata(&path)
                .and_then(|m| m.modified())
                .map(|t| chrono::DateTime::<chrono::Local>::from(t).to_rfc3339_opts(chrono::SecondsFormat::Secs, false))
                .unwrap_or_else(|_| crate::library::now_iso());
            let rel = format!("scans/{}", path.file_name().unwrap().to_string_lossy());
            let mut meta = ScanMeta {
                id: id.clone(),
                name: None,
                created,
                kind,
                file: rel,
                points: 0,
                duration: 0.0,
                sweep_deg: None,
                complete: !partial,
                group: None,
                pose: crate::util::identity4(),
                microstep: None,
                source_name: None,
                bytes: std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0),
            };
            match clouds::build_scan(&meta, &path, &mount) {
                Ok(b) if b.entry.len() > 0 => {
                    meta.points = b.entry.len();
                    meta.microstep = b.new_microstep;
                    if kind != ScanKind::Ply {
                        if let Ok(data) = std::fs::read(&path) {
                            let cap = crate::capture::Capture::from_bytes(&data);
                            meta.duration = cap.sweep_seconds();
                            meta.sweep_deg = cap.config.map(|c| c.degrees as f64);
                        }
                    }
                    self.library.lock().add(meta);
                }
                _ => {
                    log::error(&format!("skipping unreadable scan file {path:?}"));
                }
            }
        }
        self.save_library();
    }
}

/// Minimal logging: stderr in debug builds, a rolling file in the app's data
/// folder always, so a user can send it when something goes wrong.
pub mod log {
    use std::io::Write;
    use std::sync::OnceLock;

    static PATH: OnceLock<std::path::PathBuf> = OnceLock::new();

    pub fn init(dir: std::path::PathBuf) {
        let _ = std::fs::create_dir_all(&dir);
        let p = dir.join("xscan3d.log");
        // Keep the file small: start over past 2 MB.
        if std::fs::metadata(&p).map(|m| m.len() > 2_000_000).unwrap_or(false) {
            let _ = std::fs::remove_file(&p);
        }
        let _ = PATH.set(p);
    }

    fn write(level: &str, msg: &str) {
        let line = format!("{} {level} {msg}\n", chrono::Local::now().format("%Y-%m-%d %H:%M:%S%.3f"));
        #[cfg(debug_assertions)]
        eprint!("{line}");
        if let Some(p) = PATH.get() {
            if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(p) {
                let _ = f.write_all(line.as_bytes());
            }
        }
    }

    pub fn info(msg: &str) {
        write("INFO", msg);
    }

    pub fn error(msg: &str) {
        write("ERROR", msg);
    }
}
