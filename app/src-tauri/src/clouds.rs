//! Built point clouds, cached, and their wire format to the viewer.
//!
//! A scan's cloud is always rebuilt from its raw stream with the current
//! mount geometry, so calibration changes apply to every scan. Building is
//! tens of milliseconds, the microstep fit a little more (and its result is
//! kept in the library), so the cache is only there to make re-selecting,
//! merging and exporting instant.

use std::sync::Arc;

use rayon::prelude::*;

use crate::capture::Capture;
use crate::export;
use crate::geometry::{self, Mount};
use crate::library::{ScanKind, ScanMeta};
use crate::util::Rng;

pub struct CloudEntry {
    pub id: String,
    pub fingerprint: u64,
    pub xyz: Vec<[f32; 3]>,
    pub dist: Vec<f32>,
    /// Corrected shaft angle of each point (0 for imported clouds).
    pub platform: Vec<f32>,
    pub rgb: Option<Vec<[u8; 3]>>,
    pub sweep_deg: f32,
    pub bbox: ([f32; 3], [f32; 3]),
    pub z_span: (f32, f32),
    pub d_span: (f32, f32),
    /// Height of a detected ceiling (scan frame), for the dollhouse view.
    pub ceiling: Option<f32>,
}

/// Finds a ceiling: a dense, wide horizontal layer at the top of the cloud.
/// Indoors that is most of what hides the room from a viewer outside it, so
/// the viewer cuts just below it by default. None outdoors.
fn detect_ceiling(xyz: &[[f32; 3]]) -> Option<f32> {
    if xyz.len() < 5000 {
        return None;
    }
    let step = (xyz.len() / 300_000).max(1);
    let pts: Vec<[f32; 3]> = xyz.iter().step_by(step).copied().collect();
    let mut z: Vec<f64> = pts.iter().map(|p| p[2] as f64).collect();
    let top = crate::util::quantile_mut(&mut z, 0.997) as f32;
    // 25 mm bins over the top 1.5 m.
    const BIN: f32 = 25.0;
    let lo = top - 1500.0;
    let n = ((top + 100.0 - lo) / BIN) as usize + 1;
    let mut hist = vec![0usize; n];
    for p in &pts {
        if p[2] >= lo && p[2] <= top + 100.0 {
            hist[((p[2] - lo) / BIN) as usize] += 1;
        }
    }
    // The densest 100 mm band within 400 mm of the top.
    let first = ((top - 400.0 - lo) / BIN).max(0.0) as usize;
    let mut best = (0usize, 0usize);
    for i in first..n.saturating_sub(3) {
        let c = hist[i] + hist[i + 1] + hist[i + 2] + hist[i + 3];
        if c > best.0 {
            best = (c, i);
        }
    }
    if (best.0 as f64) < 0.04 * pts.len() as f64 {
        return None;
    }
    let band_lo = lo + best.1 as f32 * BIN;
    let band_hi = band_lo + 4.0 * BIN;
    // It must be wide, not a lamp or a shelf top.
    let mut mn = [f32::INFINITY; 2];
    let mut mx = [f32::NEG_INFINITY; 2];
    let mut sum = 0.0f64;
    let mut cnt = 0usize;
    for p in &pts {
        if p[2] >= band_lo && p[2] < band_hi {
            for d in 0..2 {
                mn[d] = mn[d].min(p[d]);
                mx[d] = mx[d].max(p[d]);
            }
            sum += p[2] as f64;
            cnt += 1;
        }
    }
    if cnt == 0 || mx[0] - mn[0] < 1500.0 || mx[1] - mn[1] < 1500.0 {
        return None;
    }
    Some((sum / cnt as f64) as f32)
}

impl CloudEntry {
    pub fn len(&self) -> usize {
        self.xyz.len()
    }

    pub fn new(id: String, fingerprint: u64, xyz: Vec<[f32; 3]>, dist: Vec<f32>, platform: Vec<f32>, rgb: Option<Vec<[u8; 3]>>, sweep_deg: f32) -> CloudEntry {
        let mut lo = [f32::INFINITY; 3];
        let mut hi = [f32::NEG_INFINITY; 3];
        for p in &xyz {
            for d in 0..3 {
                lo[d] = lo[d].min(p[d]);
                hi[d] = hi[d].max(p[d]);
            }
        }
        if xyz.is_empty() {
            lo = [0.0; 3];
            hi = [0.0; 3];
        }
        let z: Vec<f32> = xyz.iter().map(|p| p[2]).collect();
        let z_span = geometry::robust_span(&z);
        let d_span = geometry::robust_span(&dist);
        let ceiling = detect_ceiling(&xyz);
        CloudEntry { id, fingerprint, xyz, dist, platform, rgb, sweep_deg, bbox: (lo, hi), z_span, d_span, ceiling }
    }
}

/// What building a scan produced, including a microstep fit worth keeping.
pub struct Built {
    pub entry: CloudEntry,
    pub new_microstep: Option<Vec<f64>>,
    pub capture_points: usize,
}

/// Builds a scan's cloud from its file.
pub fn build_scan(meta: &ScanMeta, path: &std::path::Path, mount: &Mount) -> Result<Built, String> {
    match meta.kind {
        ScanKind::Ply => {
            let (xyz, rgb) = export::read_ply(path).map_err(|e| e.to_string())?;
            let dist: Vec<f32> = xyz.par_iter().map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt()).collect();
            let n = xyz.len();
            let platform: Vec<f32> = xyz.par_iter().map(|p| p[1].atan2(p[0]).to_degrees()).collect();
            let entry = CloudEntry::new(meta.id.clone(), 0, xyz, dist, platform, rgb, 180.0);
            Ok(Built { entry, new_microstep: None, capture_points: n })
        }
        ScanKind::Device | ScanKind::Bin => {
            let data = std::fs::read(path).map_err(|e| e.to_string())?;
            let cap = Capture::from_bytes(&data);
            build_capture(meta, &cap, mount)
        }
    }
}

pub fn build_capture(meta: &ScanMeta, cap: &Capture, mount: &Mount) -> Result<Built, String> {
    let mut new_microstep = None;
    let coef: Option<Vec<f64>> = if mount.microstep_auto {
        match &meta.microstep {
            Some(c) if !c.is_empty() => Some(c.clone()),
            _ => {
                let c = geometry::fit_microstep(cap, mount);
                new_microstep = c.clone();
                c
            }
        }
    } else {
        None
    };
    let cloud = geometry::build_cloud(cap, mount, coef.as_deref()).map_err(|e| e.to_string())?;
    let sweep = cap
        .config
        .map(|c| c.degrees as f32)
        .or(meta.sweep_deg.map(|d| d as f32))
        .unwrap_or_else(|| cloud.platform.iter().fold(1.0f32, |a, &b| a.max(b.abs())));
    let n = cloud.len();
    let entry = CloudEntry::new(meta.id.clone(), mount.fingerprint(), cloud.xyz, cloud.dist, cloud.platform, None, sweep);
    Ok(Built { entry, new_microstep, capture_points: n })
}

/// Small LRU of built clouds, bounded by total points.
pub struct CloudCache {
    items: Vec<Arc<CloudEntry>>,
    max_points: usize,
}

impl CloudCache {
    pub fn new(max_points: usize) -> Self {
        CloudCache { items: Vec::new(), max_points }
    }

    pub fn get(&mut self, id: &str, fingerprint: u64) -> Option<Arc<CloudEntry>> {
        let pos = self.items.iter().position(|e| e.id == id && (e.fingerprint == fingerprint || e.fingerprint == 0))?;
        let e = self.items.remove(pos);
        self.items.push(e.clone());
        Some(e)
    }

    pub fn put(&mut self, e: Arc<CloudEntry>) {
        self.items.retain(|x| x.id != e.id);
        self.items.push(e);
        let mut total: usize = self.items.iter().map(|x| x.len()).sum();
        while total > self.max_points && self.items.len() > 1 {
            let old = self.items.remove(0);
            total -= old.len();
        }
    }

    pub fn remove(&mut self, id: &str) {
        self.items.retain(|x| x.id != id);
    }

    pub fn clear_raw(&mut self) {
        self.items.retain(|x| x.fingerprint == 0);
    }
}

pub const CLOUD_MAGIC: u32 = 0x3143_5358; // "XSC1"

/// The viewer's format: a 64-byte header (magic, count, flags, sweep, bbox,
/// z and range spans, ceiling height or NaN), then positions, ranges (mm) and the
/// sweep order (0..65535) as parallel arrays, then optional rgb. Points are
/// shuffled so any prefix is a uniform subsample: the viewer can upload and
/// draw progressively and thin while the camera moves.
pub fn serialize(e: &CloudEntry) -> Vec<u8> {
    let n = e.len();
    let has_rgb = e.rgb.is_some();
    let mut perm: Vec<u32> = (0..n as u32).collect();
    let mut rng = Rng::new(n as u64 ^ 0x5eed);
    for i in (1..n).rev() {
        let j = rng.below(i + 1);
        perm.swap(i, j);
    }
    let size = 64 + n * 16 + if has_rgb { n * 3 } else { 0 };
    let mut out = vec![0u8; size];
    let put32 = |b: &mut [u8], o: usize, v: u32| b[o..o + 4].copy_from_slice(&v.to_le_bytes());
    let putf = |b: &mut [u8], o: usize, v: f32| b[o..o + 4].copy_from_slice(&v.to_le_bytes());
    put32(&mut out, 0, CLOUD_MAGIC);
    put32(&mut out, 4, n as u32);
    put32(&mut out, 8, if has_rgb { 1 } else { 0 });
    putf(&mut out, 12, e.sweep_deg);
    for d in 0..3 {
        putf(&mut out, 16 + 4 * d, e.bbox.0[d]);
        putf(&mut out, 28 + 4 * d, e.bbox.1[d]);
    }
    putf(&mut out, 40, e.z_span.0);
    putf(&mut out, 44, e.z_span.1);
    putf(&mut out, 48, e.d_span.0);
    putf(&mut out, 52, e.d_span.1);
    putf(&mut out, 56, e.ceiling.unwrap_or(f32::NAN));
    let (_, body) = out.split_at_mut(64);
    let (pos, rest) = body.split_at_mut(n * 12);
    let (rng_b, rest) = rest.split_at_mut(n * 2);
    let (ord_b, rgb_b) = rest.split_at_mut(n * 2);
    let sweep = e.sweep_deg.max(1.0);
    pos.par_chunks_mut(12).zip(perm.par_iter()).for_each(|(c, &i)| {
        let p = e.xyz[i as usize];
        c[0..4].copy_from_slice(&p[0].to_le_bytes());
        c[4..8].copy_from_slice(&p[1].to_le_bytes());
        c[8..12].copy_from_slice(&p[2].to_le_bytes());
    });
    rng_b.par_chunks_mut(2).zip(perm.par_iter()).for_each(|(c, &i)| {
        let d = e.dist[i as usize].round().clamp(0.0, 65535.0) as u16;
        c.copy_from_slice(&d.to_le_bytes());
    });
    ord_b.par_chunks_mut(2).zip(perm.par_iter()).for_each(|(c, &i)| {
        let t = ((e.platform[i as usize] + sweep) / (2.0 * sweep)).clamp(0.0, 1.0);
        c.copy_from_slice(&((t * 65535.0) as u16).to_le_bytes());
    });
    if let Some(rgb) = &e.rgb {
        rgb_b.par_chunks_mut(3).zip(perm.par_iter()).for_each(|(c, &i)| c.copy_from_slice(&rgb[i as usize]));
    }
    out
}
