//! The rig's calibration lives in the scanner's own flash (see protocol.h), so
//! it belongs to the hardware rather than to whichever PC fitted it. The app
//! keeps a copy in its settings -- clouds still need it with the scanner
//! unplugged -- and keeps that copy in step with the device:
//!
//!   * on connect it asks for the stored calibration and adopts it, so the
//!     device is the authority;
//!   * a calibration changed here (a fit applied, a field edited, even with
//!     the scanner unplugged) is marked pending in the settings and written to
//!     the device once it is connected and idle; the device answers every
//!     write with what it now holds, which is what clears the mark;
//!   * a device with nothing stored -- new firmware on an old rig -- is given
//!     the app's calibration.
//!
//! Only what describes the hardware goes to the device. Processing choices
//! (which half of the scan to keep, the range limits, whether the corrections
//! are switched on) stay with the app.

use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::core::{log, Core, CoreRef};
use crate::device;
use crate::geometry::Mount;
use crate::protocol::{self, CalibAssembler, CalibChunk};

/// The calibration as stored on the device: compact JSON, so the firmware
/// never has to know its layout and new fields cost no firmware change.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct DeviceCalib {
    /// Format version of this blob.
    pub v: u32,
    pub lidar_rotation: f64,
    pub lidar_reverse: bool,
    pub emitter_spacing: f64,
    pub scan_tilt: f64,
    pub flip_upright: bool,
    pub beam_offset: f64,
    pub range_error: Option<Vec<f64>>,
}

impl Default for DeviceCalib {
    fn default() -> Self {
        DeviceCalib::from_mount(&Mount::default())
    }
}

impl DeviceCalib {
    pub fn from_mount(m: &Mount) -> Self {
        DeviceCalib {
            v: 1,
            lidar_rotation: m.lidar_rotation,
            lidar_reverse: m.lidar_reverse,
            emitter_spacing: m.emitter_spacing,
            scan_tilt: m.scan_tilt,
            flip_upright: m.flip_upright,
            beam_offset: m.beam_offset,
            range_error: m.range_error.clone(),
        }
    }

    /// The settings patch that puts this calibration into the mount.
    pub fn mount_patch(&self) -> serde_json::Value {
        serde_json::json!({
            "mount": {
                "lidarRotation": self.lidar_rotation,
                "lidarReverse": self.lidar_reverse,
                "emitterSpacing": self.emitter_spacing,
                "scanTilt": self.scan_tilt,
                "flipUpright": self.flip_upright,
                "beamOffset": self.beam_offset,
                "rangeError": self.range_error,
            }
        })
    }

    pub fn to_blob(&self) -> Vec<u8> {
        serde_json::to_vec(self).unwrap_or_default()
    }

    pub fn from_blob(b: &[u8]) -> Option<Self> {
        serde_json::from_slice(b).ok()
    }
}

/// Wait this long after the last local change before writing, so stepping a
/// field through several values costs one flash erase instead of several.
const SETTLE: Duration = Duration::from_millis(1500);
/// A write the device did not confirm is retried after this long...
const RETRY: Duration = Duration::from_secs(3);
/// ...this many times, then left until the next change or reconnect.
const MAX_ATTEMPTS: u32 = 5;

/// Per-link bookkeeping, reset on every connect.
#[derive(Default)]
pub struct CalibSync {
    asm: CalibAssembler,
    /// What the device holds: None until it has answered, Some(None) when it
    /// holds nothing usable.
    device: Option<Option<DeviceCalib>>,
    /// The firmware predates calibration storage.
    unsupported: bool,
    changed_at: Option<Instant>,
    last_write: Option<Instant>,
    attempts: u32,
}

/// For the UI: "" (no link) | "reading" | "synced" | "writing" | "failed" |
/// "unsupported".
pub fn status(core: &CoreRef) -> String {
    let pending = core.settings.read().device.calib_pending;
    let dev = core.device.lock();
    if dev.link.is_none() {
        return String::new();
    }
    let c = &dev.calib;
    if c.unsupported {
        "unsupported"
    } else if c.device.is_none() {
        "reading"
    } else if !pending {
        "synced"
    } else if c.attempts >= MAX_ATTEMPTS {
        "failed"
    } else {
        "writing"
    }
    .into()
}

/// A calibration chunk from the link thread.
pub fn on_chunk(core: &CoreRef, chunk: &CalibChunk) {
    let blob = core.device.lock().calib.asm.push(chunk);
    if let Some(blob) = blob {
        on_device_calib(core, &blob);
    }
}

/// An event from the device: old firmware says so when asked for calibration.
pub fn on_event(core: &CoreRef, e: &str) {
    if e.starts_with(&format!("unknown command '{}'", protocol::CMD_CALIB_READ)) {
        core.device.lock().calib.unsupported = true;
        log::info("scanner firmware cannot store calibration; keeping it in the app only");
        device::emit_status(core);
    }
}

fn on_device_calib(core: &CoreRef, blob: &[u8]) {
    let held = if blob.is_empty() { None } else { DeviceCalib::from_blob(blob) };
    let (local, pending) = {
        let s = core.settings.read();
        (DeviceCalib::from_mount(&s.mount), s.device.calib_pending)
    };
    core.device.lock().calib.device = Some(held.clone());
    match held {
        Some(d) if d == local => {
            if pending {
                log::info("calibration stored in the scanner");
                set_pending(core, false);
            }
        }
        // The device is the authority, unless there is a local change it has
        // not been given yet; then that change is written over it.
        Some(d) if !pending => {
            log::info("calibration loaded from the scanner");
            core.update_settings_from_device(&d.mount_patch());
        }
        Some(_) => {}
        None => {
            log::info("scanner holds no calibration; giving it the app's");
            if !pending {
                set_pending(core, true);
            }
            core.device.lock().calib.changed_at = None; // write at once
        }
    }
    device::emit_status(core);
}

fn set_pending(core: &CoreRef, on: bool) {
    core.update_settings_from_device(&serde_json::json!({ "device": { "calibPending": on } }));
}

/// The calibration in the settings changed here.
pub fn on_local_change(core: &Core) {
    let mut dev = core.device.lock();
    dev.calib.changed_at = Some(Instant::now());
    dev.calib.attempts = 0;
}

/// Once a second from the device watcher: writes a pending calibration when
/// the device is known to support it and has nothing better to do.
pub fn tick(core: &CoreRef) {
    if !core.settings.read().device.calib_pending || !device::is_connected(core) {
        return;
    }
    if core.scan.lock().active.is_some() {
        return;
    }
    {
        let mut dev = core.device.lock();
        let idle = dev.status.state == protocol::STATE_IDLE;
        let c = &mut dev.calib;
        // Writing to firmware that never answered 'c' would feed the blob to
        // its command parser, letters and all.
        if c.device.is_none() || c.unsupported || !idle || c.attempts >= MAX_ATTEMPTS {
            return;
        }
        if c.changed_at.map_or(false, |t| t.elapsed() < SETTLE) {
            return;
        }
        if c.last_write.map_or(false, |t| t.elapsed() < RETRY) {
            return;
        }
        c.last_write = Some(Instant::now());
        c.attempts += 1;
        if c.attempts == MAX_ATTEMPTS {
            log::error("the scanner did not confirm the calibration write; giving up until the next change");
        }
    }
    let blob = DeviceCalib::from_mount(&core.settings.read().mount).to_blob();
    if blob.len() > protocol::CALIB_MAX_LEN {
        log::error(&format!("calibration is {} bytes, over the device's {}", blob.len(), protocol::CALIB_MAX_LEN));
        core.device.lock().calib.attempts = MAX_ATTEMPTS;
        return;
    }
    log::info(&format!("writing calibration to the scanner ({} bytes)", blob.len()));
    device::send(core, protocol::calib_write(&blob));
    device::emit_status(core);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blob_round_trip_is_exact() {
        let mut m = Mount::default();
        m.lidar_rotation = 161.9;
        m.scan_tilt = -1.2345678901234;
        let d = DeviceCalib::from_mount(&m);
        let back = DeviceCalib::from_blob(&d.to_blob()).unwrap();
        assert_eq!(d, back);
        assert!(d.to_blob().len() < protocol::CALIB_MAX_LEN);
        // A patch applied to a different mount reproduces the calibration.
        let s = crate::settings::Settings::default();
        let mut other = s.clone();
        other.mount.lidar_rotation = 0.0;
        other.mount.range_error = None;
        let merged = other.merged(&d.mount_patch());
        assert_eq!(DeviceCalib::from_mount(&merged.mount), d);
    }

    #[test]
    fn missing_fields_fall_back_to_defaults() {
        let d = DeviceCalib::from_blob(br#"{"v":1,"lidarRotation":10.0}"#).unwrap();
        assert_eq!(d.lidar_rotation, 10.0);
        assert_eq!(d.scan_tilt, Mount::default().scan_tilt);
    }
}
