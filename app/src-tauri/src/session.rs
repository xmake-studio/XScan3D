//! One scan, from the Start button to a finished library entry.
//!
//! The raw stream is written to disk as it arrives (<id>.bin.part, renamed
//! when the sweep ends), so a crash or an unplug mid-scan loses nothing that
//! had already come in. Points are built frame by frame for the live view;
//! the microstep correction, which needs the whole sweep, is fitted once at
//! the end and the final cloud replaces the live one.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use serde::Serialize;

use crate::capture::Capture;
use crate::clouds;
use crate::core::{log, CoreRef};
use crate::device;
use crate::geometry::LiveBuilder;
use crate::library::{new_id, now_iso, ScanKind, ScanMeta};
use crate::protocol::{self, Record};

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum Phase {
    /// Settings sent, waiting for the platform to move.
    Starting,
    /// Travelling to the start of the sweep and settling.
    Preparing,
    Scanning,
    /// Sweep over, building the final cloud.
    Finishing,
}

pub struct ActiveScan {
    pub id: String,
    pub seq: u32,
    created: String,
    started: Instant,
    part_path: PathBuf,
    final_path: PathBuf,
    writer: Option<BufWriter<File>>,
    cap: Capture,
    builder: LiveBuilder,
    /// x, y, z, range, shaft angle.
    live: Vec<[f32; 5]>,
    sweep_deg: f64,
    duration: f64,
    stepped: bool,
    phase: Phase,
    seen_motion: bool,
    seen_capture: bool,
    sweep_started: Option<Instant>,
    platform: f32,
    /// The firmware said the sweep ran to its end. Kept apart from the state
    /// code: on a freshly booted board STATE_DONE can be gone before we look,
    /// and the scan is still a complete one.
    sweep_done: bool,
    stop_requested: bool,
    last_flush: Instant,
    pending_points: Vec<([f32; 3], f32, f32)>,
}

#[derive(Default)]
pub struct ScanState {
    pub active: Option<ActiveScan>,
    pub seq: u32,
}

impl ScanState {
    pub fn active_part_path(&self) -> Option<PathBuf> {
        self.active.as_ref().map(|a| a.part_path.clone())
    }
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ScanProgress {
    pub id: String,
    pub seq: u32,
    pub phase: Phase,
    pub progress: f32,
    pub platform: f32,
    pub sweep_deg: f32,
    pub points: usize,
    pub elapsed: f32,
    pub remaining: f32,
    pub duration: f32,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ScanFinished {
    pub id: String,
    pub complete: bool,
    pub points: usize,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ScanError {
    /// "not-connected" | "busy" | "not-started" | "device-lost" | "empty" | "io"
    pub code: String,
    pub detail: Option<String>,
}

fn scan_error(core: &CoreRef, code: &str, detail: Option<String>) {
    log::error(&format!("scan error {code}: {detail:?}"));
    core.emit("scan-error", ScanError { code: code.into(), detail });
}

/// Pushes the settings to the device and starts a sweep.
pub fn start(core: &CoreRef) -> Result<ScanProgress, String> {
    if !device::is_connected(core) {
        return Err("not-connected".into());
    }
    let settings = core.settings.read().clone();
    let mut st = core.scan.lock();
    if st.active.is_some() {
        return Err("busy".into());
    }
    let sweep = settings.sweep_degrees();
    let duration = settings.scan.duration.clamp(5.0, 6000.0);
    let id = new_id();
    let root = core.library.lock().root.clone();
    let final_path = root.join("scans").join(format!("{id}.bin"));
    let part_path = root.join("scans").join(format!("{id}.bin.part"));
    let file = File::create(&part_path).map_err(|e| format!("io: {e}"))?;
    st.seq += 1;
    let seq = st.seq;
    st.active = Some(ActiveScan {
        id: id.clone(),
        seq,
        created: now_iso(),
        started: Instant::now(),
        part_path,
        final_path,
        writer: Some(BufWriter::with_capacity(1 << 18, file)),
        cap: Capture::new(),
        builder: LiveBuilder::new(settings.mount.clone()),
        live: Vec::with_capacity(1 << 20),
        sweep_deg: sweep,
        duration,
        stepped: settings.scan.stepped,
        phase: Phase::Starting,
        seen_motion: false,
        seen_capture: false,
        sweep_started: None,
        platform: 0.0,
        sweep_done: false,
        stop_requested: false,
        last_flush: Instant::now(),
        pending_points: Vec::with_capacity(64),
    });
    drop(st);
    // The recording is already running, so the config record the firmware
    // emits at start lands in the file.
    device::send_cmd(core, protocol::CMD_ANGLE, Some(format!("{sweep:.2}")));
    device::send_cmd(core, protocol::CMD_TIME, Some(format!("{duration:.2}")));
    device::send_cmd(core, protocol::CMD_MODE, Some(if settings.scan.stepped { "1" } else { "0" }.into()));
    if settings.scan.stepped {
        let dwell = settings.scan.dwell_ms.clamp(50, 5000);
        let per_stop = (250.0 + dwell as f64) / 1000.0;
        let steps = ((duration / per_stop).round() as i64 - 1).clamp(2, 2000);
        device::send_cmd(core, protocol::CMD_STEPS, Some(steps.to_string()));
        device::send_cmd(core, protocol::CMD_DWELL, Some(dwell.to_string()));
    }
    device::send_cmd(core, protocol::CMD_START, None);
    log::info(&format!("scan {id} started: +-{sweep} deg, {duration} s"));
    let p = progress_of(core).ok_or("busy")?;
    core.emit("scan", p.clone());
    Ok(p)
}

/// Asks the device to stop; the partial sweep is kept.
pub fn stop(core: &CoreRef) {
    let has = {
        let mut st = core.scan.lock();
        match st.active.as_mut() {
            Some(a) => {
                a.stop_requested = true;
                true
            }
            None => false,
        }
    };
    if has {
        device::send_cmd(core, protocol::CMD_ABORT, None);
        // A device that never started reports IDLE throughout: finish now
        // rather than waiting for a transition that will not come.
        let quiet = core.scan.lock().active.as_ref().map_or(false, |a| !a.seen_motion);
        if quiet {
            finish(core, false);
        }
    }
}

/// Everything the link thread decoded, plus the raw bytes for the file.
pub fn on_link_data(core: &CoreRef, chunk: &[u8], records: &[Record]) {
    let mut done: Option<bool> = None;
    let mut failed: Option<String> = None;
    {
        let mut st = core.scan.lock();
        let Some(a) = st.active.as_mut() else { return };
        if a.phase == Phase::Finishing {
            return;
        }
        if let Some(w) = a.writer.as_mut() {
            if w.write_all(chunk).is_err() {
                failed = Some("io".into());
            }
            if a.last_flush.elapsed().as_millis() > 1000 {
                let _ = w.flush();
                a.last_flush = Instant::now();
            }
        }
        for r in records {
            match r {
                Record::Sample(s) => {
                    a.cap.samples.push(*s);
                    a.pending_points.clear();
                    a.builder.push(s, &mut a.pending_points);
                    for (p, d, pl) in a.pending_points.drain(..) {
                        a.live.push([p[0], p[1], p[2], d, pl]);
                    }
                }
                Record::Telem(t) => {
                    let mut t = *t;
                    t.sample_index = a.cap.samples.len();
                    a.cap.telem.push(t);
                    a.platform = t.platform;
                    match t.state {
                        protocol::STATE_HOMING | protocol::STATE_PARKING | protocol::STATE_SETTLING => {
                            a.seen_motion = true;
                            if !a.seen_capture {
                                a.phase = Phase::Preparing;
                            }
                        }
                        s if protocol::is_sweep_state(s) => {
                            a.seen_motion = true;
                            a.seen_capture = true;
                            a.phase = Phase::Scanning;
                            if a.sweep_started.is_none() {
                                a.sweep_started = Some(Instant::now());
                            }
                        }
                        protocol::STATE_DONE => {
                            if a.seen_capture {
                                a.sweep_done = true;
                                done = Some(true);
                            }
                        }
                        protocol::STATE_IDLE => {
                            if a.seen_capture {
                                // Idle after a sweep the firmware reported as
                                // finished is a normal end, not an abort.
                                done = Some(a.sweep_done && !a.stop_requested);
                            } else if a.seen_motion || a.stop_requested {
                                done = Some(false);
                            } else if a.started.elapsed().as_secs_f64() > 5.0 {
                                failed = Some("not-started".into());
                            }
                        }
                        _ => {}
                    }
                }
                Record::Config(c) => a.cap.config = Some(*c),
                Record::Event(e) => {
                    if e.starts_with("busy") && !a.seen_motion {
                        failed = Some("busy".into());
                    }
                    if e.starts_with("sweep complete") {
                        a.sweep_done = true;
                    }
                    a.cap.events.push(e.clone());
                }
                Record::Calib(_) => {}
            }
        }
    }
    if let Some(code) = failed {
        discard(core);
        scan_error(core, &code, None);
    } else if let Some(complete) = done {
        finish(core, complete);
    }
}

/// ~10 Hz from the link thread: progress for the UI.
pub fn on_tick(core: &CoreRef) {
    if let Some(p) = progress_of(core) {
        core.emit("scan", p);
    }
}

fn progress_of(core: &CoreRef) -> Option<ScanProgress> {
    let st = core.scan.lock();
    let a = st.active.as_ref()?;
    let sweep = a.sweep_deg.max(1.0) as f32;
    let progress = match a.phase {
        Phase::Scanning | Phase::Finishing => ((a.platform + sweep) / (2.0 * sweep)).clamp(0.0, 1.0),
        _ => 0.0,
    };
    let elapsed = a.sweep_started.map(|t| t.elapsed().as_secs_f32()).unwrap_or(0.0);
    let remaining = match a.phase {
        Phase::Scanning if progress > 0.02 && a.stepped => elapsed * (1.0 - progress) / progress,
        Phase::Scanning => (a.duration as f32 * (1.0 - progress)).max(0.0),
        Phase::Finishing => 0.0,
        _ => a.duration as f32,
    };
    Some(ScanProgress {
        id: a.id.clone(),
        seq: a.seq,
        phase: a.phase,
        progress,
        platform: a.platform,
        sweep_deg: sweep,
        points: a.live.len(),
        elapsed,
        remaining,
        duration: a.duration as f32,
    })
}

pub fn on_link_lost(core: &CoreRef) {
    let state = {
        let st = core.scan.lock();
        st.active.as_ref().map(|a| (a.phase, a.seen_capture))
    };
    match state {
        Some((Phase::Finishing, _)) | None => {}
        Some((_, true)) => {
            scan_error(core, "device-lost", None);
            finish(core, false);
        }
        Some((_, false)) => {
            discard(core);
            scan_error(core, "device-lost", None);
        }
    }
}

/// Drops a scan that produced nothing worth keeping.
fn discard(core: &CoreRef) {
    let a = core.scan.lock().active.take();
    if let Some(mut a) = a {
        a.writer.take();
        let _ = std::fs::remove_file(&a.part_path);
        core.emit("scan-cancelled", a.id.clone());
    }
}

/// Closes the recording and builds the final cloud off the link thread.
fn finish(core: &CoreRef, complete: bool) {
    let taken = {
        let mut st = core.scan.lock();
        match st.active.as_mut() {
            Some(a) if a.phase != Phase::Finishing => {
                a.phase = Phase::Finishing;
                Some((a.id.clone(), a.writer.take(), a.cap.clone(), a.part_path.clone(), a.final_path.clone(), a.created.clone(), a.sweep_deg))
            }
            _ => None,
        }
    };
    let Some((id, writer, cap, part, fin, created, sweep)) = taken else { return };
    if let Some(p) = progress_of(core) {
        core.emit("scan", p);
    }
    let core = core.clone();
    std::thread::Builder::new()
        .name("scan-finish".into())
        .spawn(move || {
            if let Some(mut w) = writer {
                let _ = w.flush();
            }
            let path = if std::fs::rename(&part, &fin).is_ok() { fin } else { part };
            let file = format!("scans/{}", path.file_name().unwrap().to_string_lossy());
            let mount = core.settings.read().mount.clone();
            let mut meta = ScanMeta {
                id: id.clone(),
                name: None,
                created,
                kind: ScanKind::Device,
                file,
                points: 0,
                duration: cap.sweep_seconds(),
                sweep_deg: Some(cap.config.map(|c| c.degrees as f64).unwrap_or(sweep)),
                complete,
                group: None,
                pose: crate::util::identity4(),
                microstep: None,
                source_name: None,
                bytes: std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0),
            };
            match clouds::build_capture(&meta, &cap, &mount) {
                Ok(b) if b.entry.len() > 0 => {
                    meta.points = b.entry.len();
                    meta.microstep = b.new_microstep;
                    let points = meta.points;
                    core.library.lock().add(meta);
                    core.save_library();
                    core.clouds.lock().put(Arc::new(b.entry));
                    core.scan.lock().active = None;
                    log::info(&format!("scan {id} saved: {points} points, complete={complete}"));
                    core.emit_library();
                    core.emit("scan-finished", ScanFinished { id, complete, points });
                }
                _ => {
                    let _ = std::fs::remove_file(&path);
                    core.scan.lock().active = None;
                    scan_error(&core, "empty", None);
                }
            }
        })
        .expect("spawn finish");
}

pub const LIVE_MAGIC: u32 = 0x314C_5358; // "XSL1"

/// Live points from index `since` of scan `seq`: a 20-byte header (magic,
/// seq, total, since, n) then positions, ranges and sweep order like the
/// cloud format. An empty body when that scan is no longer live.
pub fn live_points(core: &CoreRef, seq: u32, since: u32) -> Vec<u8> {
    let st = core.scan.lock();
    let mut out = Vec::new();
    let put = |o: &mut Vec<u8>, v: u32| o.extend_from_slice(&v.to_le_bytes());
    let Some(a) = st.active.as_ref().filter(|a| a.seq == seq) else {
        put(&mut out, LIVE_MAGIC);
        put(&mut out, seq);
        put(&mut out, 0);
        put(&mut out, since);
        put(&mut out, 0);
        return out;
    };
    let total = a.live.len() as u32;
    let since = since.min(total);
    let n = (total - since) as usize;
    out.reserve(20 + n * 16);
    put(&mut out, LIVE_MAGIC);
    put(&mut out, seq);
    put(&mut out, total);
    put(&mut out, since);
    put(&mut out, n as u32);
    let pts = &a.live[since as usize..];
    for p in pts {
        for v in &p[..3] {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    for p in pts {
        out.extend_from_slice(&(p[3].round().clamp(0.0, 65535.0) as u16).to_le_bytes());
    }
    let sweep = a.sweep_deg.max(1.0) as f32;
    for p in pts {
        let t = ((p[4] + sweep) / (2.0 * sweep)).clamp(0.0, 1.0);
        out.extend_from_slice(&((t * 65535.0) as u16).to_le_bytes());
    }
    out
}
