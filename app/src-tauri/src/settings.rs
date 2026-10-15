//! Persistent settings, one JSON file in the app's config directory. Every
//! section defaults sensibly, so a missing or older file just fills in.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::devcalib::DeviceCalib;
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
    /// Every scanner this app has worked with, by chip serial, with its
    /// calibration as last known: each scan is rebuilt with the calibration
    /// of the scanner that took it, whichever one is plugged in.
    pub scanners: BTreeMap<String, ScannerRecord>,
    /// The scanner `mount` and `calib_pending` describe: the one connected,
    /// else the last one that was.
    pub active: Option<String>,
}

impl Default for DeviceSettings {
    fn default() -> Self {
        DeviceSettings {
            auto_connect: true,
            dtr: true,
            known_serials: Vec::new(),
            calib_pending: false,
            scanners: BTreeMap::new(),
            active: None,
        }
    }
}

/// One scanner's calibration, kept for its scans while it is unplugged.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
#[serde(rename_all = "camelCase", default)]
pub struct ScannerRecord {
    pub calib: DeviceCalib,
    /// Changed here and not yet stored in that scanner.
    pub pending: bool,
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
    /// On by default: the Surface button and mesh export are shown.
    pub enabled: bool,
    /// "medium" | "high" | "max"
    pub quality: String,
    /// Trim distance in scan gaps (see meshing.rs); 0 keeps the surface
    /// closed. A new name for a new unit: the old `trim` counted median point
    /// spacings, a few millimetres, and a saved value of it would mean far
    /// less now.
    pub trim_gaps: f64,
    pub budget: usize,
    pub smooth: u32,
}

impl Default for MeshSettings {
    fn default() -> Self {
        MeshSettings { enabled: true, quality: "high".into(), trim_gaps: 8.0, budget: 2_500_000, smooth: 0 }
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

    /// The mount a scan taken by `scanner` is built with: the processing
    /// choices from here, the calibration of that scanner. A scan of no known
    /// scanner (recorded before scanners were told apart, or imported) gets
    /// the active one's.
    pub fn mount_for(&self, scanner: Option<&str>) -> Mount {
        let mut m = self.mount.clone();
        if let Some(s) = scanner {
            if self.device.active.as_deref() != Some(s) {
                if let Some(r) = self.device.scanners.get(s) {
                    r.calib.apply_to(&mut m);
                }
            }
        }
        m
    }

    /// Copies the mount's calibration into the active scanner's record, so
    /// the two never disagree.
    pub fn sync_active_scanner(&mut self) {
        if let Some(a) = self.device.active.clone() {
            let r = self.device.scanners.entry(a).or_default();
            r.calib = DeviceCalib::from_mount(&self.mount);
            r.pending = self.device.calib_pending;
        }
    }

    /// Whether any scanner's calibration differs between the two.
    pub fn calibrations_differ(&self, other: &Settings) -> bool {
        let a = self.device.scanners.iter().map(|(k, v)| (k, &v.calib));
        let b = other.device.scanners.iter().map(|(k, v)| (k, &v.calib));
        !a.eq(b)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn two_scanners() -> Settings {
        let mut s = Settings::default();
        s.device.active = Some("A".into());
        s.mount.lidar_rotation = 161.95;
        s.mount.scan_tilt = -1.63;
        s.sync_active_scanner();
        let mut b = DeviceCalib::from_mount(&s.mount);
        b.lidar_rotation = 162.59;
        b.scan_tilt = -0.2;
        s.device.scanners.insert("B".into(), ScannerRecord { calib: b, pending: false });
        s
    }

    #[test]
    fn each_scan_is_built_with_its_scanners_calibration() {
        let mut s = two_scanners();
        assert_eq!(s.mount_for(Some("A")).lidar_rotation, 161.95);
        let b = s.mount_for(Some("B"));
        assert_eq!((b.lidar_rotation, b.scan_tilt), (162.59, -0.2));
        // Unknown or unrecorded scanners get the active one's.
        assert_eq!(s.mount_for(None).lidar_rotation, 161.95);
        assert_eq!(s.mount_for(Some("C")).lidar_rotation, 161.95);
        // Processing choices are the app's, whichever scanner.
        s.mount.max_range = 5000.0;
        assert_eq!(s.mount_for(Some("B")).max_range, 5000.0);
    }

    #[test]
    fn a_patch_reaches_one_scanner_and_survives_a_round_trip() {
        let s = two_scanners();
        let patched = s.merged(&serde_json::json!({ "device": { "scanners": { "B": { "calib": { "scanTilt": -0.3 }, "pending": true } } } }));
        assert_eq!(patched.device.scanners["B"].calib.scan_tilt, -0.3);
        assert_eq!(patched.device.scanners["B"].calib.lidar_rotation, 162.59);
        assert!(patched.device.scanners["B"].pending);
        assert_eq!(patched.device.scanners["A"], s.device.scanners["A"]);
        assert!(patched.calibrations_differ(&s));
        let back: Settings = serde_json::from_str(&serde_json::to_string(&patched).unwrap()).unwrap();
        assert_eq!(back, patched);
        // Settings written before scanners were told apart still load.
        let old: Settings = serde_json::from_str(r#"{"device":{"calibPending":true},"mount":{"scanTilt":-1.6}}"#).unwrap();
        assert!(old.device.active.is_none() && old.device.scanners.is_empty() && old.device.calib_pending);
        // A trim saved in the old unit is not read as the new one.
        let old: Settings = serde_json::from_str(r#"{"mesh":{"trim":3.0,"quality":"max"}}"#).unwrap();
        assert_eq!((old.mesh.trim_gaps, old.mesh.quality.as_str()), (8.0, "max"));
    }
}
