//! The scan library: every scan kept on disk automatically, like a photo
//! library. A folder holds the raw streams (the source of truth, so the
//! calibration can always be re-applied) and one index file with the names,
//! merge groups and poses.
//!
//!   <root>/library.json        index (atomic writes)
//!   <root>/scans/<id>.bin      raw device stream, or an imported .bin
//!   <root>/scans/<id>.ply      an imported point cloud
//!   <root>/scans/<id>.bin.part a scan still recording (recovered on start)
//!   <root>/thumbs/<id>.png     thumbnails rendered by the viewer

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::util::{identity4, Mat4};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ScanKind {
    Device,
    Bin,
    Ply,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ScanMeta {
    pub id: String,
    /// None: the UI shows a date-based title in the user's language.
    #[serde(default)]
    pub name: Option<String>,
    pub created: String,
    pub kind: ScanKind,
    pub file: String,
    #[serde(default)]
    pub points: usize,
    #[serde(default)]
    pub duration: f64,
    #[serde(default)]
    pub sweep_deg: Option<f64>,
    #[serde(default = "yes")]
    pub complete: bool,
    #[serde(default)]
    pub group: Option<String>,
    #[serde(default = "identity4")]
    pub pose: Mat4,
    /// Cached microstep fit for this scan.
    #[serde(default)]
    pub microstep: Option<Vec<f64>>,
    #[serde(default)]
    pub source_name: Option<String>,
    #[serde(default)]
    pub bytes: u64,
}

fn yes() -> bool {
    true
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GroupMeta {
    pub id: String,
    #[serde(default)]
    pub name: Option<String>,
    pub created: String,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct LibraryIndex {
    #[serde(default)]
    pub version: u32,
    #[serde(default)]
    pub scans: Vec<ScanMeta>,
    #[serde(default)]
    pub groups: Vec<GroupMeta>,
}

pub struct Library {
    pub root: PathBuf,
    pub index: LibraryIndex,
}

pub fn new_id() -> String {
    // Time-ordered and unique enough for one user's library.
    let now = chrono::Local::now();
    let rnd = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(0);
    format!("{}-{:05x}", now.format("%Y%m%d-%H%M%S"), rnd & 0xFFFFF)
}

pub fn now_iso() -> String {
    chrono::Local::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, false)
}

impl Library {
    pub fn open(root: &Path) -> std::io::Result<Library> {
        std::fs::create_dir_all(root.join("scans"))?;
        std::fs::create_dir_all(root.join("thumbs"))?;
        let index = match std::fs::read(root.join("library.json")) {
            Ok(b) => serde_json::from_slice(&b).unwrap_or_default(),
            Err(_) => LibraryIndex::default(),
        };
        let mut lib = Library { root: root.to_path_buf(), index };
        lib.index.version = 1;
        Ok(lib)
    }

    pub fn save(&self) -> std::io::Result<()> {
        let path = self.root.join("library.json");
        let tmp = self.root.join("library.json.tmp");
        std::fs::write(&tmp, serde_json::to_vec_pretty(&self.index).unwrap_or_default())?;
        std::fs::rename(&tmp, &path)
    }

    pub fn path_of(&self, meta: &ScanMeta) -> PathBuf {
        self.root.join(&meta.file)
    }

    pub fn thumb_path(&self, id: &str) -> PathBuf {
        self.root.join("thumbs").join(format!("{id}.png"))
    }

    pub fn get(&self, id: &str) -> Option<&ScanMeta> {
        self.index.scans.iter().find(|s| s.id == id)
    }

    pub fn get_mut(&mut self, id: &str) -> Option<&mut ScanMeta> {
        self.index.scans.iter_mut().find(|s| s.id == id)
    }

    pub fn add(&mut self, meta: ScanMeta) {
        self.index.scans.retain(|s| s.id != meta.id);
        self.index.scans.push(meta);
    }

    pub fn group_members(&self, gid: &str) -> Vec<&ScanMeta> {
        self.index.scans.iter().filter(|s| s.group.as_deref() == Some(gid)).collect()
    }

    pub fn new_group(&mut self) -> String {
        let id = format!("g{}", new_id());
        self.index.groups.push(GroupMeta { id: id.clone(), name: None, created: now_iso() });
        id
    }

    /// Drops groups with fewer than two members, freeing a lone survivor.
    pub fn prune_groups(&mut self) {
        let mut keep = Vec::new();
        for g in &self.index.groups {
            let n = self.index.scans.iter().filter(|s| s.group.as_deref() == Some(&g.id)).count();
            if n >= 2 {
                keep.push(g.clone());
            } else {
                for s in self.index.scans.iter_mut() {
                    if s.group.as_deref() == Some(&g.id) {
                        s.group = None;
                    }
                }
            }
        }
        self.index.groups = keep;
    }

    /// Takes scans out of their group; poses are kept (usually still right).
    pub fn ungroup(&mut self, ids: &[String], reset_pose: bool) {
        for s in self.index.scans.iter_mut() {
            if ids.contains(&s.id) {
                s.group = None;
                if reset_pose {
                    s.pose = identity4();
                }
            }
        }
        self.prune_groups();
    }

    /// Removes scans from the index and moves their files to the recycle bin.
    pub fn delete(&mut self, ids: &[String]) {
        let mut files = Vec::new();
        for s in &self.index.scans {
            if ids.contains(&s.id) {
                files.push(self.path_of(s));
                files.push(self.thumb_path(&s.id));
            }
        }
        self.index.scans.retain(|s| !ids.contains(&s.id));
        self.prune_groups();
        for f in files {
            if f.exists() {
                if !recycle(&f) {
                    let _ = std::fs::remove_file(&f);
                }
            }
        }
    }

    /// Scans on disk that the index does not know: recordings interrupted by
    /// a crash (.bin.part) and files copied in by hand. Returns their paths.
    pub fn orphans(&self) -> Vec<PathBuf> {
        let known: std::collections::HashSet<PathBuf> = self.index.scans.iter().map(|s| self.path_of(s)).collect();
        let mut out = Vec::new();
        if let Ok(rd) = std::fs::read_dir(self.root.join("scans")) {
            for e in rd.flatten() {
                let p = e.path();
                let name = p.file_name().and_then(|n| n.to_str()).unwrap_or("").to_lowercase();
                let wanted = name.ends_with(".bin") || name.ends_with(".bin.part") || name.ends_with(".ply");
                if wanted && !known.contains(&p) {
                    out.push(p);
                }
            }
        }
        out
    }
}

/// Moves a file to the Windows recycle bin, so a deletion can be undone.
#[cfg(windows)]
pub fn recycle(path: &Path) -> bool {
    use std::os::windows::ffi::OsStrExt;
    use windows_sys::Win32::UI::Shell::{
        SHFileOperationW, FOF_ALLOWUNDO, FOF_NOCONFIRMATION, FOF_NOERRORUI, FOF_SILENT, FO_DELETE, SHFILEOPSTRUCTW,
    };
    let mut from: Vec<u16> = path.as_os_str().encode_wide().collect();
    from.push(0);
    from.push(0); // double-null terminated list
    let mut op: SHFILEOPSTRUCTW = unsafe { std::mem::zeroed() };
    op.wFunc = FO_DELETE;
    op.pFrom = from.as_ptr();
    op.fFlags = (FOF_ALLOWUNDO | FOF_NOCONFIRMATION | FOF_NOERRORUI | FOF_SILENT) as u16;
    unsafe { SHFileOperationW(&mut op) == 0 && op.fAnyOperationsAborted == 0 }
}

#[cfg(not(windows))]
pub fn recycle(_path: &Path) -> bool {
    false
}
