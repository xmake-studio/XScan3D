//! The long-running work, each on its own thread with progress events:
//! merging scans (SLAM), building a surface, calibrating the mount, and file
//! import/export.

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;

use rayon::prelude::*;
use serde::Serialize;

use crate::calibration::{self, CalibResult, Stage};
use crate::capture::Capture;
use crate::clouds;
use crate::core::{log, CoreRef};
use crate::export;
use crate::geometry;
use crate::library::{new_id, now_iso, ScanKind, ScanMeta};
use crate::meshing::{self, Mesh, MeshInfo, MeshParams, MeshStage};
use crate::registration::{self, Reason, RegParams, RegStage, Verdict};
use crate::util::{self, Mat4, V3};

/// Colours for "colour by scan", shared with the viewer (palette.ts).
pub const SCAN_COLORS: [[f32; 3]; 8] = [
    [0.039, 0.518, 1.0],
    [1.0, 0.584, 0.0],
    [0.204, 0.780, 0.349],
    [1.0, 0.176, 0.333],
    [0.686, 0.322, 0.871],
    [0.188, 0.690, 0.780],
    [1.0, 0.800, 0.0],
    [0.639, 0.518, 0.369],
];

/// Scan ids behind a selection of scan and group ids, in selection order.
pub fn resolve(core: &CoreRef, ids: &[String]) -> Vec<String> {
    let lib = core.library.lock();
    let mut out: Vec<String> = Vec::new();
    for id in ids {
        if lib.get(id).is_some() {
            if !out.contains(id) {
                out.push(id.clone());
            }
        } else {
            for m in lib.group_members(id) {
                if !out.contains(&m.id) {
                    out.push(m.id.clone());
                }
            }
        }
    }
    out
}

fn pose_of(core: &CoreRef, id: &str) -> Mat4 {
    core.library.lock().get(id).map(|m| m.pose).unwrap_or_else(util::identity4)
}

/// A scan's points in the scene frame.
fn world_points(core: &CoreRef, id: &str) -> Result<Vec<V3>, String> {
    let c = core.cloud(id)?;
    let t = pose_of(core, id);
    Ok(c.xyz.par_iter().map(|p| util::xform(&t, [p[0] as f64, p[1] as f64, p[2] as f64])).collect())
}

// --- merge (SLAM) ------------------------------------------------------------------------

pub struct MergeJob {
    group: String,
    queue: VecDeque<Vec<String>>,
    target: Vec<V3>,
    pending: Option<Pending>,
    merged: usize,
    total: usize,
}

struct Pending {
    unit: Vec<String>,
    t: Mat4,
}

#[derive(Clone, Serialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum MergeEvent {
    #[serde(rename_all = "camelCase")]
    Progress { unit: Vec<String>, index: usize, total: usize, stage: RegStage, step: usize, steps: usize },
    #[serde(rename_all = "camelCase")]
    Pending {
        unit: Vec<String>,
        poses: Vec<(String, Mat4)>,
        verdict: Verdict,
        reasons: Vec<Reason>,
        overlap: f64,
        rmse: f64,
        stability: f64,
    },
    #[serde(rename_all = "camelCase")]
    Accepted { unit: Vec<String>, auto: bool },
    #[serde(rename_all = "camelCase")]
    Rejected { unit: Vec<String> },
    #[serde(rename_all = "camelCase")]
    Done { group: Option<String>, merged: usize },
    #[serde(rename_all = "camelCase")]
    Failed { code: String, detail: Option<String> },
}

fn thin(p: Vec<V3>) -> Vec<V3> {
    util::voxel_centroids(&p, 5.0)
}

/// Starts aligning a selection into one merged group: an existing group is
/// held fixed and everything else is aligned onto it; with no group, the
/// first scan defines the frame. (The rule comes from the old PyQt tool's
/// `merge_selected`, removed in favour of this app.)
pub fn merge_start(core: &CoreRef, ids: &[String]) -> Result<(), String> {
    if core.merge.lock().is_some() {
        return Err("busy".into());
    }
    let sel = resolve(core, ids);
    if sel.len() < 2 {
        return Err("need-two".into());
    }
    let (groups, loose): (Vec<String>, Vec<String>) = {
        let lib = core.library.lock();
        let mut groups: Vec<String> = Vec::new();
        let mut loose = Vec::new();
        for id in &sel {
            match lib.get(id).and_then(|m| m.group.clone()) {
                Some(g) => {
                    if !groups.contains(&g) {
                        groups.push(g)
                    }
                }
                None => loose.push(id.clone()),
            }
        }
        (groups, loose)
    };
    let members = |g: &str| -> Vec<String> { core.library.lock().group_members(g).iter().map(|m| m.id.clone()).collect() };
    let (gid, target_ids, units): (String, Vec<String>, Vec<Vec<String>>) = if groups.len() > 1 {
        let count = |g: &String| -> usize { core.library.lock().group_members(g).iter().map(|m| m.points).sum() };
        let base = groups.iter().max_by_key(|g| count(g)).unwrap().clone();
        let mut units: Vec<Vec<String>> = groups.iter().filter(|g| **g != base).map(|g| members(g)).collect();
        units.extend(loose.iter().map(|s| vec![s.clone()]));
        (base.clone(), members(&base), units)
    } else if groups.len() == 1 {
        if loose.is_empty() {
            return Err("nothing-new".into());
        }
        (groups[0].clone(), members(&groups[0]), loose.iter().map(|s| vec![s.clone()]).collect())
    } else {
        let gid = core.library.lock().new_group();
        let base = sel[0].clone();
        if let Some(m) = core.library.lock().get_mut(&base) {
            m.group = Some(gid.clone());
        }
        (gid, vec![base], sel[1..].iter().map(|s| vec![s.clone()]).collect())
    };
    core.merge_cancel.store(false, Ordering::SeqCst);
    let total = units.len();
    let core2 = core.clone();
    std::thread::Builder::new()
        .name("merge".into())
        .spawn(move || {
            let mut target = Vec::new();
            for id in &target_ids {
                match world_points(&core2, id) {
                    Ok(p) => target.extend(p),
                    Err(e) => {
                        merge_fail(&core2, "load", Some(e));
                        return;
                    }
                }
            }
            *core2.merge.lock() = Some(MergeJob { group: gid, queue: units.into(), target: thin(target), pending: None, merged: 0, total });
            merge_run(&core2);
        })
        .map_err(|e| e.to_string())?;
    Ok(())
}

fn merge_fail(core: &CoreRef, code: &str, detail: Option<String>) {
    *core.merge.lock() = None;
    core.library.lock().prune_groups();
    core.save_library();
    core.emit_library();
    core.emit("merge", MergeEvent::Failed { code: code.into(), detail });
}

fn reg_params(core: &CoreRef) -> RegParams {
    let s = core.settings.read().merge.clone();
    let mut p = RegParams { voxel: s.voxel.clamp(5.0, 500.0), structure: s.ignore_foliage, ..Default::default() };
    match s.mode.as_str() {
        "small" => p.init = Some(util::identity4()),
        "hint" => p.yaw_hint = Some(s.yaw_hint),
        _ => {}
    }
    p
}

/// Works through the queue until it is empty or an alignment needs a human.
fn merge_run(core: &CoreRef) {
    loop {
        let next = {
            let mut guard = core.merge.lock();
            let Some(job) = guard.as_mut() else { return };
            if job.pending.is_some() {
                return;
            }
            loop {
                match job.queue.front().cloned() {
                    None => break None,
                    Some(u) => {
                        let lib = core.library.lock();
                        let live: Vec<String> = u.into_iter().filter(|i| lib.get(i).is_some()).collect();
                        drop(lib);
                        if live.is_empty() {
                            job.queue.pop_front();
                            continue;
                        }
                        break Some((live, job.total - job.queue.len(), job.total));
                    }
                }
            }
        };
        let Some((unit, index, total)) = next else {
            let job = core.merge.lock().take();
            let (group, merged) = job.map(|j| (j.group, j.merged)).unwrap_or_default();
            core.library.lock().prune_groups();
            core.save_library();
            core.emit_library();
            let exists = core.library.lock().index.groups.iter().any(|g| g.id == group);
            core.emit("merge", MergeEvent::Done { group: exists.then_some(group), merged });
            return;
        };
        let mut source = Vec::new();
        for id in &unit {
            match world_points(core, id) {
                Ok(p) => source.extend(p),
                Err(e) => return merge_fail(core, "load", Some(e)),
            }
        }
        let target = match core.merge.lock().as_ref() {
            Some(j) => j.target.clone(),
            None => return,
        };
        let params = reg_params(core);
        let cancel = core.merge_cancel.clone();
        let progress = |stage: RegStage, step: usize, steps: usize| {
            core.emit("merge", MergeEvent::Progress { unit: unit.clone(), index, total, stage, step, steps });
        };
        let result = registration::register(&source, &target, &params, &progress, &|| cancel.load(Ordering::Relaxed));
        let Some(result) = result else {
            *core.merge.lock() = None;
            core.library.lock().prune_groups();
            core.save_library();
            core.emit_library();
            core.emit("merge", MergeEvent::Done { group: None, merged: 0 });
            return;
        };
        let (verdict, reasons) = registration::verdict(&result);
        log::info(&format!(
            "merge {unit:?}: {verdict:?} overlap {:.2} rmse {:.1} stability {:.3}",
            result.overlap, result.rmse, result.stability
        ));
        if verdict == Verdict::Good {
            apply_unit(core, &unit, &result.t, true);
            continue;
        }
        let poses: Vec<(String, Mat4)> = unit.iter().map(|id| (id.clone(), util::mat4_mul(&result.t, &pose_of(core, id)))).collect();
        if let Some(job) = core.merge.lock().as_mut() {
            job.pending = Some(Pending { unit: unit.clone(), t: result.t });
        }
        core.emit(
            "merge",
            MergeEvent::Pending { unit, poses, verdict, reasons, overlap: result.overlap, rmse: result.rmse, stability: result.stability },
        );
        return;
    }
}

/// Moves a unit to its solved pose and into the group.
fn apply_unit(core: &CoreRef, unit: &[String], t: &Mat4, auto: bool) {
    let gid = match core.merge.lock().as_ref() {
        Some(j) => j.group.clone(),
        None => return,
    };
    {
        let mut lib = core.library.lock();
        for id in unit {
            if let Some(m) = lib.get_mut(id) {
                m.pose = util::mat4_mul(t, &m.pose);
                m.group = Some(gid.clone());
            }
        }
        lib.prune_groups();
        // prune_groups may have dropped the target group if it was the base's
        // only membership so far; the unit now makes it two, so re-add.
        if !lib.index.groups.iter().any(|g| g.id == gid) {
            lib.index.groups.push(crate::library::GroupMeta { id: gid.clone(), name: None, created: now_iso() });
        }
    }
    core.save_library();
    let mut added = Vec::new();
    for id in unit {
        if let Ok(p) = world_points(core, id) {
            added.extend(p);
        }
    }
    if let Some(job) = core.merge.lock().as_mut() {
        job.target.extend(thin(added));
        job.queue.pop_front();
        job.merged += 1;
        job.pending = None;
    }
    core.emit_library();
    core.emit("merge", MergeEvent::Accepted { unit: unit.to_vec(), auto });
}

pub fn merge_decide(core: &CoreRef, accept: bool) {
    let pending = core.merge.lock().as_mut().and_then(|j| j.pending.take());
    let Some(p) = pending else { return };
    if accept {
        apply_unit(core, &p.unit, &p.t, false);
    } else {
        if let Some(job) = core.merge.lock().as_mut() {
            job.queue.pop_front();
        }
        core.emit("merge", MergeEvent::Rejected { unit: p.unit });
    }
    let core = core.clone();
    std::thread::spawn(move || merge_run(&core));
}

pub fn merge_cancel(core: &CoreRef) {
    core.merge_cancel.store(true, Ordering::SeqCst);
    let pending = core.merge.lock().as_ref().map_or(false, |j| j.pending.is_some());
    if pending {
        *core.merge.lock() = None;
        core.library.lock().prune_groups();
        core.save_library();
        core.emit_library();
        core.emit("merge", MergeEvent::Done { group: None, merged: 0 });
    }
}

// --- surface -------------------------------------------------------------------------------

pub struct MeshCacheEntry {
    pub key: String,
    pub mesh: Mesh,
    pub info: MeshInfo,
}

#[derive(Clone, Serialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum MeshEvent {
    #[serde(rename_all = "camelCase")]
    Progress { stage: MeshStage },
    #[serde(rename_all = "camelCase")]
    Done { key: String, info: MeshInfo },
    #[serde(rename_all = "camelCase")]
    Failed { detail: String },
}

pub fn mesh_key(core: &CoreRef, scans: &[String]) -> String {
    let s = core.settings.read();
    let mut ids = scans.to_vec();
    ids.sort();
    let mut key = format!("{:x}|{}|{}|{}|{}", s.mount.fingerprint(), s.mesh.quality, s.mesh.trim, s.mesh.budget, s.mesh.smooth);
    drop(s);
    for id in &ids {
        let p = pose_of(core, id);
        key.push_str(&format!("|{id}:{:.3},{:.3},{:.3},{:.4}", p[3], p[7], p[11], p[0] + p[5]));
    }
    key
}

pub fn mesh_start(core: &CoreRef, ids: &[String]) -> Result<String, String> {
    if core.mesh_busy.swap(true, Ordering::SeqCst) {
        return Err("busy".into());
    }
    core.mesh_cancel.store(false, Ordering::SeqCst);
    let scans = resolve(core, ids);
    let key = mesh_key(core, &scans);
    let core2 = core.clone();
    let key2 = key.clone();
    std::thread::Builder::new()
        .name("mesh".into())
        .spawn(move || {
            let core = core2;
            let run = || -> Result<(Mesh, MeshInfo), String> {
                let mut pts = Vec::new();
                let mut scan_idx = Vec::new();
                let mut origins = Vec::new();
                for (k, id) in scans.iter().enumerate() {
                    let p = world_points(&core, id)?;
                    scan_idx.extend(std::iter::repeat(k as u16).take(p.len()));
                    pts.extend(p);
                    let t = pose_of(&core, id);
                    origins.push([t[3], t[7], t[11]]);
                }
                let s = core.settings.read().mesh.clone();
                let prm = MeshParams::preset(&s.quality, s.trim, s.budget, s.smooth);
                let cancel = core.mesh_cancel.clone();
                meshing::reconstruct(&pts, &scan_idx, &origins, &prm, &|stage| core.emit("mesh", MeshEvent::Progress { stage }), &|| {
                    cancel.load(Ordering::Relaxed)
                })
            };
            match run() {
                Ok((mesh, info)) => {
                    log::info(&format!("mesh: {} tris, {} verts in {:.1}s", info.tris, info.verts, info.seconds));
                    *core.mesh.lock() = Some(MeshCacheEntry { key: key2.clone(), mesh, info: info.clone() });
                    core.emit("mesh", MeshEvent::Done { key: key2, info });
                }
                Err(e) => {
                    log::error(&format!("mesh failed: {e}"));
                    core.emit("mesh", MeshEvent::Failed { detail: e });
                }
            }
            core.mesh_busy.store(false, Ordering::SeqCst);
        })
        .map_err(|e| e.to_string())?;
    Ok(key)
}

pub const MESH_MAGIC: u32 = 0x314D_5358; // "XSM1"

/// The viewer's mesh format: a 16-byte header (magic, verts, tris, 0), then
/// positions (f32 x3), normals (i8 x4, normalised) and u32 indices.
pub fn mesh_bytes(m: &Mesh) -> Vec<u8> {
    let nv = m.verts.len();
    let nt = m.tris.len();
    let mut out = Vec::with_capacity(16 + nv * 16 + nt * 12);
    for v in [MESH_MAGIC, nv as u32, nt as u32, 0] {
        out.extend_from_slice(&v.to_le_bytes());
    }
    for v in &m.verts {
        for x in v {
            out.extend_from_slice(&x.to_le_bytes());
        }
    }
    for n in &m.normals {
        for x in n {
            out.push((x * 127.0).round().clamp(-127.0, 127.0) as i8 as u8);
        }
        out.push(0);
    }
    for t in &m.tris {
        for i in t {
            out.extend_from_slice(&i.to_le_bytes());
        }
    }
    out
}

// --- calibration ----------------------------------------------------------------------------

#[derive(Clone, Serialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum CalibEvent {
    #[serde(rename_all = "camelCase")]
    Progress { stage: Stage, fraction: f64 },
    #[serde(rename_all = "camelCase")]
    Done { result: CalibResult, scan: String },
    #[serde(rename_all = "camelCase")]
    Failed { code: String, detail: Option<String> },
}

pub fn calibrate_start(core: &CoreRef, id: &str) -> Result<(), String> {
    let (meta, path) = {
        let lib = core.library.lock();
        let m = lib.get(id).cloned().ok_or("unknown scan")?;
        let p = lib.path_of(&m);
        (m, p)
    };
    if meta.kind == ScanKind::Ply {
        return Err("no-raw".into());
    }
    if core.calib_busy.swap(true, Ordering::SeqCst) {
        return Err("busy".into());
    }
    core.calib_cancel.store(false, Ordering::SeqCst);
    let core = core.clone();
    let id = id.to_string();
    std::thread::Builder::new()
        .name("calibrate".into())
        .spawn(move || {
            let res = std::fs::read(&path).map_err(|e| e.to_string()).and_then(|data| {
                let cap = Capture::from_bytes(&data);
                let base = core.settings.read().mount.clone();
                let cancel = core.calib_cancel.clone();
                calibration::calibrate(
                    &cap,
                    &base,
                    &|stage, fraction| core.emit("calib", CalibEvent::Progress { stage, fraction }),
                    &|| cancel.load(Ordering::Relaxed),
                )
                .map_err(|e| match e {
                    calibration::CalibError::Cancelled => "cancelled".to_string(),
                    calibration::CalibError::NoSweep => "no-sweep".to_string(),
                    calibration::CalibError::ShortSweep(_) => "short-sweep".to_string(),
                    calibration::CalibError::NoPlanes => "no-planes".to_string(),
                })
            });
            match res {
                Ok(result) => core.emit("calib", CalibEvent::Done { result, scan: id }),
                Err(code) => core.emit("calib", CalibEvent::Failed { code, detail: None }),
            }
            core.calib_busy.store(false, Ordering::SeqCst);
        })
        .map_err(|e| e.to_string())?;
    Ok(())
}

// --- files ----------------------------------------------------------------------------------

/// Imports .bin captures and .ply clouds into the library. Returns new ids.
pub fn import_files(core: &CoreRef, paths: &[String]) -> (Vec<String>, Vec<String>) {
    let mut added = Vec::new();
    let mut failed = Vec::new();
    let mount = core.settings.read().mount.clone();
    for p in paths {
        let src = PathBuf::from(p);
        let ext = src.extension().and_then(|e| e.to_str()).unwrap_or("").to_lowercase();
        let kind = match ext.as_str() {
            "bin" => ScanKind::Bin,
            "ply" => ScanKind::Ply,
            _ => {
                failed.push(p.clone());
                continue;
            }
        };
        let id = new_id();
        let root = core.library.lock().root.clone();
        let rel = format!("scans/{id}.{ext}");
        let dst = root.join(&rel);
        if std::fs::copy(&src, &dst).is_err() {
            failed.push(p.clone());
            continue;
        }
        let created = std::fs::metadata(&src)
            .and_then(|m| m.modified())
            .map(|t| chrono::DateTime::<chrono::Local>::from(t).to_rfc3339_opts(chrono::SecondsFormat::Secs, false))
            .unwrap_or_else(|_| now_iso());
        let mut meta = ScanMeta {
            id: id.clone(),
            name: src.file_stem().and_then(|s| s.to_str()).map(|s| s.to_string()),
            created,
            kind,
            file: rel,
            points: 0,
            duration: 0.0,
            sweep_deg: None,
            complete: true,
            group: None,
            pose: util::identity4(),
            microstep: None,
            source_name: src.file_name().and_then(|s| s.to_str()).map(|s| s.to_string()),
            bytes: std::fs::metadata(&dst).map(|m| m.len()).unwrap_or(0),
        };
        match clouds::build_scan(&meta, &dst, &mount) {
            Ok(b) if b.entry.len() > 0 => {
                meta.points = b.entry.len();
                meta.microstep = b.new_microstep;
                if kind == ScanKind::Bin {
                    if let Ok(data) = std::fs::read(&dst) {
                        let cap = Capture::from_bytes(&data);
                        meta.duration = cap.sweep_seconds();
                        meta.sweep_deg = cap.config.map(|c| c.degrees as f64);
                    }
                }
                core.library.lock().add(meta);
                core.clouds.lock().put(std::sync::Arc::new(b.entry));
                added.push(id);
            }
            _ => {
                let _ = std::fs::remove_file(&dst);
                failed.push(p.clone());
            }
        }
    }
    if !added.is_empty() {
        core.save_library();
        core.emit_library();
    }
    (added, failed)
}

/// Writes the selection as one PLY point cloud, coloured like the viewer.
pub fn export_cloud(core: &CoreRef, ids: &[String], path: &Path, color_mode: &str) -> Result<usize, String> {
    let scans = resolve(core, ids);
    let mut xyz: Vec<[f32; 3]> = Vec::new();
    let mut range: Vec<f32> = Vec::new();
    let mut scan_of: Vec<u16> = Vec::new();
    let mut rgb_src: Vec<Option<[u8; 3]>> = Vec::new();
    for (k, id) in scans.iter().enumerate() {
        let c = core.cloud(id)?;
        let t = pose_of(core, id);
        xyz.extend(c.xyz.iter().map(|p| {
            let w = util::xform(&t, [p[0] as f64, p[1] as f64, p[2] as f64]);
            [w[0] as f32, w[1] as f32, w[2] as f32]
        }));
        range.extend_from_slice(&c.dist);
        scan_of.extend(std::iter::repeat(k as u16).take(c.len()));
        match &c.rgb {
            Some(rgb) => rgb_src.extend(rgb.iter().map(|x| Some(*x))),
            None => rgb_src.extend(std::iter::repeat(None).take(c.len())),
        }
    }
    if xyz.is_empty() {
        return Err("empty".into());
    }
    let colors: Vec<[u8; 3]> = match color_mode {
        "distance" => {
            let (lo, hi) = geometry::robust_span(&range);
            range.par_iter().map(|&d| export::ramp((d - lo) / (hi - lo))).collect()
        }
        "mono" => vec![[210, 212, 216]; xyz.len()],
        "scan" => scan_of
            .par_iter()
            .map(|&k| {
                let c = SCAN_COLORS[k as usize % SCAN_COLORS.len()];
                [(c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8]
            })
            .collect(),
        _ => {
            let z: Vec<f32> = xyz.iter().map(|p| p[2]).collect();
            let (lo, hi) = geometry::robust_span(&z);
            xyz.par_iter().zip(rgb_src.par_iter()).map(|(p, src)| src.unwrap_or_else(|| export::ramp((p[2] - lo) / (hi - lo)))).collect()
        }
    };
    export::write_ply_points(path, &xyz, &colors, Some(&range)).map_err(|e| e.to_string())?;
    Ok(xyz.len())
}

pub fn export_raw(core: &CoreRef, id: &str, path: &Path) -> Result<(), String> {
    let src = {
        let lib = core.library.lock();
        let m = lib.get(id).ok_or("unknown scan")?;
        if m.kind == ScanKind::Ply {
            return Err("no-raw".into());
        }
        lib.path_of(m)
    };
    std::fs::copy(&src, path).map(|_| ()).map_err(|e| e.to_string())
}

pub fn export_mesh(core: &CoreRef, path: &Path) -> Result<(), String> {
    let guard = core.mesh.lock();
    let entry = guard.as_ref().ok_or("no-mesh")?;
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_lowercase();
    match ext.as_str() {
        "obj" => export::write_obj(path, &entry.mesh),
        "stl" => export::write_stl(path, &entry.mesh),
        _ => export::write_ply_mesh(path, &entry.mesh),
    }
    .map_err(|e| e.to_string())
}
