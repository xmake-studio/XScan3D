//! Persistent settings, one JSON file in the app's config directory. Every
//! section defaults sensibly, so a missing or older file just fills in.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::geometry::Mount;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", default)]
pub struct ScanSettings {
    /// Sweep duration, seconds (the exponential slider).
    pub duration: f64,
    /// Countdown before the sweep starts, seconds.
    pub start_delay: u32,
    /// Stop at every angle instead of sweeping continuously.
    pub stepped: bool,
    pub dwell_ms: u32,
    /// Half-sweep in degrees; 0 = automatic (90 plus what the calibrated
    /// scan-plane tilt needs, or 180 when one half of each revolution is kept).
    pub sweep_deg: f64,
}

impl Default for ScanSettings {
    fn default() -> Self {
        ScanSettings { duration: 180.0, start_delay: 0, stepped: false, dwell_ms: 400, sweep_deg: 0.0 }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", default)]
pub struct DeviceSettings {
    pub auto_connect: bool,
    /// The RP2040's USB serial only talks once the host raises DTR.
    pub dtr: bool,
    /// Chip serials of boards that have proven to be the scanner.
    pub known_serials: Vec<String>,
    /// The mount calibration changed here and the scanner has not confirmed
    /// storing it yet (see devcalib.rs). Persisted, so a change made with the
    /// scanner unplugged still reaches it on the next connect.
    pub calib_pending: bool,
}

impl Default for DeviceSettings {
    fn default() -> Self {
        DeviceSettings { auto_connect: true, dtr: true, known_serials: Vec::new(), calib_pending: false }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", default)]
pub struct ViewSettings {
    /// "height" | "distance" | "mono" | "scan"
    pub color_mode: String,
    pub point_size: f64,
    pub grid: bool,
    pub edl: bool,
    /// "orbit" | "fly"
    pub camera: String,
    pub fly_speed: f64,
}

impl Default for ViewSettings {
    fn default() -> Self {
        ViewSettings {
            color_mode: "height".into(),
            point_size: 1.0,
            grid: true,
            edl: true,
            camera: "orbit".into(),
            fly_speed: 1500.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", default)]
pub struct MeshSettings {
    /// Off by default: the surface tools stay out of the way until wanted.
    pub enabled: bool,
    /// "medium" | "high" | "max"
    pub quality: String,
    /// Trim distance in point spacings.
    pub trim: f64,
    pub budget: usize,
    pub smooth: u32,
}

impl Default for MeshSettings {
    fn default() -> Self {
        MeshSettings { enabled: false, quality: "high".into(), trim: 3.0, budget: 2_500_000, smooth: 0 }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", default)]
pub struct MergeSettings {
    /// "auto" (search everything) | "small" (barely moved) | "hint"
    pub mode: String,
    pub yaw_hint: f64,
    pub voxel: f64,
    pub ignore_foliage: bool,
}

impl Default for MergeSettings {
    fn default() -> Self {
        MergeSettings { mode: "auto".into(), yaw_hint: 0.0, voxel: 40.0, ignore_foliage: true }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", default)]
pub struct Settings {
    /// "system" | "ru" | "en"
    pub language: String,
    /// "system" | "light" | "dark"
    pub theme: String,
    /// Where scans live; None = Documents\XScan3D.
    pub library_dir: Option<String>,
    pub scan: ScanSettings,
    pub device: DeviceSettings,
    pub mount: Mount,
    pub view: ViewSettings,
    pub mesh: MeshSettings,
    pub merge: MergeSettings,
    /// The one-time hint about fly-camera keys has been shown.
    pub fly_hint_seen: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            language: "system".into(),
            theme: "system".into(),
            library_dir: None,
            scan: ScanSettings::default(),
            device: DeviceSettings::default(),
            mount: Mount::default(),
            view: ViewSettings::default(),
            mesh: MeshSettings::default(),
            merge: MergeSettings::default(),
            fly_hint_seen: false,
        }
    }
}

impl Settings {
    pub fn load(path: &Path) -> Settings {
        match std::fs::read_to_string(path) {
            Ok(s) => serde_json::from_str(&s).unwrap_or_default(),
            Err(_) => Settings::default(),
        }
    }

    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let tmp = path.with_extension("json.tmp");
        std::fs::write(&tmp, serde_json::to_vec_pretty(self).unwrap_or_default())?;
        std::fs::rename(&tmp, path)
    }

    /// Applies a partial JSON object on top of these settings.
    pub fn merged(&self, patch: &serde_json::Value) -> Settings {
        let mut base = serde_json::to_value(self).unwrap_or_default();
        merge_json(&mut base, patch);
        serde_json::from_value(base).unwrap_or_else(|_| self.clone())
    }

    /// The half-sweep actually sent to the device.
    pub fn sweep_degrees(&self) -> f64 {
        if self.scan.sweep_deg > 0.0 {
            return self.scan.sweep_deg.clamp(1.0, 180.0);
        }
        match self.mount.scan_half {
            crate::geometry::ScanHalf::Both => 90.0 + self.mount.tilt_overlap(),
            _ => 180.0,
        }
    }

    pub fn library_path(&self, documents: Option<PathBuf>) -> PathBuf {
        match &self.library_dir {
            Some(d) if !d.trim().is_empty() => PathBuf::from(d),
            _ => documents.unwrap_or_else(|| PathBuf::from(".")).join("XScan3D"),
        }
    }
}

fn merge_json(base: &mut serde_json::Value, patch: &serde_json::Value) {
    match (base, patch) {
        (serde_json::Value::Object(b), serde_json::Value::Object(p)) => {
            for (k, v) in p {
                match b.get_mut(k) {
                    Some(slot) if slot.is_object() && v.is_object() => merge_json(slot, v),
                    _ => {
                        b.insert(k.clone(), v.clone());
                    }
                }
            }
        }
        (b, p) => *b = p.clone(),
    }
}
