//! Turning a capture into a point cloud: mount geometry, lidar calibration and
//! the per-scan microstep correction. A port of tools/scan_proto.py's
//! build_cloud / fit_microstep, kept numerically identical.
//!
//! The lidar is bolted to the stepper shaft with its scan plane vertical and
//! the shaft turns about world Z, so each shaft angle contributes one vertical
//! slice and half a turn carries it through every azimuth. At shaft angle 0 the
//! slice is the world XZ plane with the lidar's spin axis along world Y.
//!
//! The corrections, each explained at length in scan_proto.py:
//!   * lidar roll: rotation of the lidar about its own spin axis (an azimuth
//!     offset, applied before the shaft yaw);
//!   * azimuth direction: the lidar's angle runs clockwise or not (a mirror);
//!   * emitter spacing: the rangefinder's reference point sits half this far to
//!     the side of the spin axis, which splits flat surfaces into two sheets
//!     when wrong;
//!   * scan half: keep one side of each revolution only;
//!   * scan-plane tilt: the plane leans off the rotation axis, twisting points
//!     near the zenith;
//!   * microstep: the rotor sits periodically off the commanded microstep; the
//!     error is measured from the flat surfaces of each scan itself.

use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::capture::Capture;
use crate::protocol::{to_degrees, DIST_INVALID, DIST_MASK, POINTS, RAW_ANGLE_MIN};
use crate::patches::{self, Patches, Projected};
use crate::util::{self, V3};

/// Degrees of shaft per full step: 1.8 for a 200-step motor.
pub const FULL_STEP_DEG: f64 = 1.8;
/// The period of the driver's current table: four full steps.
pub const ELECTRICAL_CYCLE_DEG: f64 = 4.0 * FULL_STEP_DEG;
/// Harmonics of the electrical cycle fitted by the microstep correction.
pub const MICROSTEP_HARMONICS: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ScanHalf {
    Both,
    A,
    B,
}

/// How the sensor is bolted together, plus the range filter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct Mount {
    pub lidar_rotation: f64,
    pub lidar_reverse: bool,
    pub emitter_spacing: f64,
    pub scan_tilt: f64,
    pub scan_half: ScanHalf,
    pub flip_upright: bool,
    pub beam_offset: f64,
    pub min_range: f64,
    /// 0 = no limit.
    pub max_range: f64,
    pub microstep_auto: bool,
    /// Apply the lidar's range non-linearity model below.
    pub range_correction: bool,
    /// (T, p, E, d_lo, d_hi, coefficients...), see `correct_range`. None when
    /// the lidar has not been calibrated.
    pub range_error: Option<Vec<f64>>,
}

/// The range model fitted to scans/room.bin (scan_proto.RANGE_ERROR).
pub fn default_range_error() -> Vec<f64> {
    vec![
        24.7, 3.5, 2.0, 827.0, 3561.0, -0.08861, -0.01755, -0.003573, 0.1086, -0.01447, 0.01174,
        -0.1113, 0.01858, -0.003672, 0.0393, 0.06772, 0.01801,
    ]
}

impl Default for Mount {
    fn default() -> Self {
        // Nominal values of the reference build; each rig replaces them with its
        // own calibration, which the scanner keeps in flash (see devcalib.rs).
        Mount {
            lidar_rotation: 161.65,
            lidar_reverse: true,
            emitter_spacing: -36.5,
            scan_tilt: -1.2,
            scan_half: ScanHalf::Both,
            flip_upright: false,
            beam_offset: 0.0,
            min_range: 60.0,
            max_range: 0.0,
            microstep_auto: true,
            range_correction: true,
            range_error: Some(default_range_error()),
        }
    }
}

/// Highest elevation, above or below the horizon, the automatic sweep closes
/// the tilt seam up to. The seam widens as tan(elevation), so the last few
/// degrees before the pole would cost more sweep than all the rest together.
pub const SWEEP_COVER_ELEVATION_DEG: f64 = 80.0;
/// Added on top, for the frames lost to the sweep's first and last instants.
pub const SWEEP_MARGIN_DEG: f64 = 1.0;

impl Mount {
    /// Degrees to add to each end of a +-90 sweep so both halves of the scan
    /// plane still meet despite the scan-plane tilt.
    ///
    /// Tilted by t, the plane at shaft angle phi holds the direction at
    /// elevation e and azimuth a when sin(a - phi) = -tan e tan t. Each half of
    /// the plane therefore reaches a direction s = asin(tan e tan t) of shaft
    /// early or late, the two halves in opposite senses, and a +-90 sweep
    /// leaves a wedge 2|s| wide at one end (the other end below the horizon).
    /// Moving each end out by |s| closes it up to elevation e. A cone of
    /// half-angle |t| around the pole is never swept at any sweep width.
    pub fn tilt_overlap(&self) -> f64 {
        let k = SWEEP_COVER_ELEVATION_DEG.to_radians().tan() * self.scan_tilt.to_radians().tan().abs();
        k.min(1.0).asin().to_degrees().min(20.0) + SWEEP_MARGIN_DEG
    }

    /// The range model in force, if any.
    pub fn active_range_error(&self) -> Option<&[f64]> {
        match (&self.range_error, self.range_correction) {
            (Some(r), true) if r.len() >= 7 => Some(r.as_slice()),
            _ => None,
        }
    }

    /// A fingerprint of everything that moves points, for cache keys.
    pub fn fingerprint(&self) -> u64 {
        let s = serde_json::to_string(self).unwrap_or_default();
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in s.bytes() {
            h ^= b as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01B3);
        }
        h
    }
}

// --- Range non-linearity ------------------------------------------------------
//
// The lidar is a triangulation rangefinder whose sub-pixel spot interpolation
// has an error that repeats every pixel, so the range error is periodic in
// 1/d and grows with range (see scan_proto.RANGE_ERROR):
//
//     error(d) = (d / 1 m)^p * sum_k a_k(d) cos(2 pi k u / T) + b_k(d) sin(..)
//     u = 1e6 / d,   a_k, b_k drifting linearly in (d / 1 m - 3)
//
// Growth and drift are frozen outside the calibrated span [d_lo, d_hi].

/// Fills `out` with the model's 2*h*drift basis columns at range `d`.
pub fn range_basis(d: f64, period: f64, power: f64, h: usize, drift: usize, span: Option<(f64, f64)>, out: &mut [f64]) {
    let u = 1e6 / d.max(1.0);
    let w = 2.0 * std::f64::consts::PI / period * u;
    let de = match span {
        Some((lo, hi)) => d.clamp(lo, hi),
        None => d,
    };
    let s = (de / 1000.0).powf(power);
    let x = de / 1000.0 - 3.0;
    let (s1, c1) = w.sin_cos();
    let mut xj = 1.0;
    for j in 0..drift {
        let (mut ck, mut sk) = (c1, s1);
        let base = j * 2 * h;
        for k in 0..h {
            out[base + k] = s * ck * xj;
            out[base + h + k] = s * sk * xj;
            let (c2, s2) = (ck * c1 - sk * s1, sk * c1 + ck * s1);
            ck = c2;
            sk = s2;
        }
        xj *= x;
    }
}

/// Measured range in mm -> corrected, under `model` (RANGE_ERROR layout).
#[inline]
pub fn correct_range(d: f64, model: Option<&[f64]>) -> f64 {
    let Some(m) = model else { return d };
    if m.len() < 7 {
        return d;
    }
    let (period, power) = (m[0], m[1]);
    let drift = (m[2] as usize).max(1);
    let span = Some((m[3], m[4]));
    let coef = &m[5..];
    let h = coef.len() / (2 * drift);
    if h == 0 {
        return d;
    }
    let mut basis = [0.0f64; 64];
    let n = 2 * h * drift;
    if n > basis.len() {
        return d;
    }
    range_basis(d, period, power, h, drift, span, &mut basis[..n]);
    let e: f64 = basis[..n].iter().zip(coef).map(|(a, b)| a * b).sum();
    d - e
}

/// Peak size of the modelled error within one ripple around `d_mm`, for
/// display ("+-N mm at 3 m").
pub fn range_error_at(model: Option<&[f64]>, d_mm: f64) -> f64 {
    let Some(m) = model else { return 0.0 };
    if m.len() < 7 {
        return 0.0;
    }
    let u = 1e6 / d_mm;
    let mut best: f64 = 0.0;
    for i in 0..64 {
        let uu = u - 0.5 * m[0] + m[0] * i as f64 / 63.0;
        let d = 1e6 / uu;
        best = best.max((correct_range(d, Some(m)) - d).abs());
    }
    best
}

#[derive(Debug)]
pub enum GeomError {
    TooFewSamples(usize),
    NoSweep,
}

impl std::fmt::Display for GeomError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GeomError::TooFewSamples(n) => write!(f, "only {n} samples; capture a sweep first"),
            GeomError::NoSweep => write!(f, "no usable frames in the sweep"),
        }
    }
}

/// Everything build_cloud does before the shaft rotation.
pub struct MountPoints {
    /// (M*8) points in the mount frame (shaft at angle 0).
    pub p: Vec<V3>,
    /// (M*8) points worth keeping.
    pub good: Vec<bool>,
    /// (M) commanded shaft angle of each kept frame.
    pub platform: Vec<f64>,
    /// (M*8) ranges, mm.
    pub dist: Vec<f64>,
}

/// Frame selection shared by the batch and the live builders: the angular
/// width of every sample, or NaN when the frame is unusable.
fn frame_widths(cap: &Capture) -> (Vec<f64>, Vec<f64>) {
    let n = cap.samples.len();
    let ang: Vec<f64> = cap.samples.iter().map(|s| to_degrees(s.raw_angle)).collect();
    let mut width = vec![f64::NAN; n];
    for k in 0..n {
        let s = &cap.samples[k];
        let w = if s.end_angle >= RAW_ANGLE_MIN {
            (to_degrees(s.end_angle) - ang[k]).rem_euclid(360.0)
        } else if k + 1 < n {
            // Legacy capture: 7/8 of the gap to the successor.
            (ang[k + 1] - ang[k]).rem_euclid(360.0) * (POINTS as f64 - 1.0) / POINTS as f64
        } else {
            f64::NAN
        };
        width[k] = w;
    }
    (ang, width)
}

/// Mount-frame position of one lidar return.
#[inline]
fn mount_point(d: f64, az: f64, m: &Mount, sin_tilt: f64, cos_tilt: f64) -> (V3, f64) {
    let th = ((if m.lidar_reverse { -az } else { az }) + m.lidar_rotation).to_radians();
    let (sin_th, cos_th) = th.sin_cos();
    let b = 0.5 * m.emitter_spacing;
    let x = d * sin_th + b * cos_th;
    let mut y = m.beam_offset;
    let mut z = d * cos_th - b * sin_th;
    if m.scan_tilt != 0.0 {
        let (y0, z0) = (y, z);
        y = cos_tilt * y0 - sin_tilt * z0;
        z = sin_tilt * y0 + cos_tilt * z0;
    }
    ([x, y, z], sin_th)
}

#[inline]
fn half_ok(half: ScanHalf, sin_th: f64) -> bool {
    match half {
        ScanHalf::Both => true,
        ScanHalf::A => sin_th >= 0.0,
        ScanHalf::B => sin_th < 0.0,
    }
}

/// Range of one return after the lidar's own correction, or None when it is
/// invalid or filtered out.
#[inline]
fn range_ok(raw: u16, m: &Mount, model: Option<&[f64]>) -> Option<f64> {
    if raw & DIST_INVALID != 0 {
        return None;
    }
    let d = correct_range((raw & DIST_MASK) as f64, model);
    if d <= m.min_range {
        return None;
    }
    if m.max_range > 0.0 && d > m.max_range {
        return None;
    }
    Some(d)
}

pub fn mount_points(cap: &Capture, m: &Mount, half: ScanHalf) -> Result<MountPoints, GeomError> {
    let n = cap.samples.len();
    if n < 2 {
        return Err(GeomError::TooFewSamples(n));
    }
    let (ang, width) = frame_widths(cap);
    let keep: Vec<usize> = (0..n)
        .filter(|&k| {
            let w = width[k];
            w > 0.0 && w < 90.0 && cap.samples[k].capturing
        })
        .collect();
    if keep.is_empty() {
        return Err(GeomError::NoSweep);
    }
    let (sin_tilt, cos_tilt) = m.scan_tilt.to_radians().sin_cos();
    let model = m.active_range_error();
    let mm = keep.len();
    let mut p = vec![[0.0; 3]; mm * POINTS];
    let mut good = vec![false; mm * POINTS];
    let mut dist = vec![0.0; mm * POINTS];
    let mut platform = vec![0.0; mm];
    p.par_chunks_mut(POINTS)
        .zip(good.par_chunks_mut(POINTS))
        .zip(dist.par_chunks_mut(POINTS))
        .zip(platform.par_iter_mut())
        .zip(keep.par_iter())
        .for_each(|((((pc, gc), dc), pl), &k)| {
            let s = &cap.samples[k];
            *pl = s.platform as f64;
            for i in 0..POINTS {
                let az = ang[k] + width[k] * (i as f64 / (POINTS as f64 - 1.0));
                let raw = s.dist[i];
                // The lidar's own range non-linearity, before anything uses it.
                let d = correct_range((raw & DIST_MASK) as f64, model);
                let (pt, sin_th) = mount_point(d, az, m, sin_tilt, cos_tilt);
                pc[i] = pt;
                dc[i] = d;
                gc[i] = range_ok(raw, m, model).is_some() && half_ok(half, sin_th);
            }
        });
    Ok(MountPoints { p, good, platform, dist })
}

#[inline]
fn microstep_offset(platform_deg: f64, coef: &[f64]) -> f64 {
    let h = coef.len() / 2;
    if h == 0 {
        return 0.0;
    }
    let w = 2.0 * std::f64::consts::PI / ELECTRICAL_CYCLE_DEG * platform_deg;
    let (s1, c1) = w.sin_cos();
    // cos(k w), sin(k w) by the angle-addition recurrence.
    let (mut ck, mut sk) = (c1, s1);
    let mut acc = 0.0;
    for k in 0..h {
        acc += coef[k] * ck + coef[h + k] * sk;
        let (c2, s2) = (ck * c1 - sk * s1, sk * c1 + ck * s1);
        ck = c2;
        sk = s2;
    }
    acc
}

/// Commanded shaft angle -> best estimate of the true one.
#[inline]
pub fn correct_microstep(platform_deg: f64, coef: Option<&[f64]>) -> f64 {
    match coef {
        Some(c) if !c.is_empty() => platform_deg + microstep_offset(platform_deg, c),
        _ => platform_deg,
    }
}

/// Fills `out` with the 2*h basis columns [cos(k w).., sin(k w)..].
#[inline]
fn microstep_basis(platform_deg: f64, h: usize, out: &mut [f64]) {
    let w = 2.0 * std::f64::consts::PI / ELECTRICAL_CYCLE_DEG * platform_deg;
    let (s1, c1) = w.sin_cos();
    let (mut ck, mut sk) = (c1, s1);
    for k in 0..h {
        out[k] = ck;
        out[h + k] = sk;
        let (c2, s2) = (ck * c1 - sk * s1, sk * c1 + ck * s1);
        ck = c2;
        sk = s2;
    }
}

#[inline]
fn yaw(p: V3, deg: f64) -> V3 {
    let (s, c) = deg.to_radians().sin_cos();
    [c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]]
}

/// A finished cloud, in the scan's own frame.
pub struct Cloud {
    pub xyz: Vec<[f32; 3]>,
    pub dist: Vec<f32>,
    /// Corrected shaft angle of each point, degrees.
    pub platform: Vec<f32>,
}

impl Cloud {
    pub fn empty() -> Cloud {
        Cloud { xyz: Vec::new(), dist: Vec::new(), platform: Vec::new() }
    }
    pub fn len(&self) -> usize {
        self.xyz.len()
    }
}

/// Reconstructs the point cloud. `microstep` is the fitted correction, or
/// None for the commanded angle as is.
pub fn build_cloud(cap: &Capture, m: &Mount, microstep: Option<&[f64]>) -> Result<Cloud, GeomError> {
    let mp = mount_points(cap, m, m.scan_half)?;
    Ok(finish_cloud(&mp, m, microstep))
}

pub fn finish_cloud(mp: &MountPoints, m: &Mount, microstep: Option<&[f64]>) -> Cloud {
    let corrected: Vec<f64> = mp.platform.par_iter().map(|&p| correct_microstep(p, microstep)).collect();
    // Keep order: frame by frame, point by point.
    let counts: Vec<usize> = mp.good.par_chunks(POINTS).map(|g| g.iter().filter(|&&x| x).count()).collect();
    let mut offs = Vec::with_capacity(counts.len() + 1);
    let mut acc = 0usize;
    offs.push(0);
    for c in &counts {
        acc += c;
        offs.push(acc);
    }
    let total = acc;
    let mut xyz = vec![[0f32; 3]; total];
    let mut dist = vec![0f32; total];
    let mut plat = vec![0f32; total];
    let xp = util::SyncPtr(xyz.as_mut_ptr());
    let dp = util::SyncPtr(dist.as_mut_ptr());
    let pp = util::SyncPtr(plat.as_mut_ptr());
    (0..counts.len()).into_par_iter().for_each(|f| {
        let (xp, dp, pp) = (xp, dp, pp);
        let mut o = offs[f];
        let deg = corrected[f];
        for i in 0..POINTS {
            let j = f * POINTS + i;
            if !mp.good[j] {
                continue;
            }
            let mut w = yaw(mp.p[j], deg);
            if m.flip_upright {
                w[1] = -w[1];
                w[2] = -w[2];
            }
            unsafe {
                *xp.0.add(o) = [w[0] as f32, w[1] as f32, w[2] as f32];
                *dp.0.add(o) = mp.dist[j] as f32;
                *pp.0.add(o) = deg as f32;
            }
            o += 1;
        }
    });
    Cloud { xyz, dist, platform: plat }
}

/// Builds points one frame at a time as a sweep arrives, without the
/// microstep correction (that is fitted once the sweep is complete).
pub struct LiveBuilder {
    m: Mount,
    sin_tilt: f64,
    cos_tilt: f64,
    /// Legacy frames wait for their successor.
    pending_legacy: Option<crate::protocol::Sample>,
}

impl LiveBuilder {
    pub fn new(m: Mount) -> Self {
        let (sin_tilt, cos_tilt) = m.scan_tilt.to_radians().sin_cos();
        LiveBuilder { m, sin_tilt, cos_tilt, pending_legacy: None }
    }

    /// Appends the points of `s` (world frame, identity pose) to `out`.
    pub fn push(&mut self, s: &crate::protocol::Sample, out: &mut Vec<([f32; 3], f32, f32)>) {
        let (frame, width) = if s.end_angle >= RAW_ANGLE_MIN {
            (*s, (to_degrees(s.end_angle) - to_degrees(s.raw_angle)).rem_euclid(360.0))
        } else {
            let prev = self.pending_legacy.replace(*s);
            let Some(prev) = prev else { return };
            let gap = (to_degrees(s.raw_angle) - to_degrees(prev.raw_angle)).rem_euclid(360.0);
            (prev, gap * (POINTS as f64 - 1.0) / POINTS as f64)
        };
        if !(width > 0.0 && width < 90.0) || !frame.capturing {
            return;
        }
        let a0 = to_degrees(frame.raw_angle);
        let deg = frame.platform as f64;
        for i in 0..POINTS {
            let raw = frame.dist[i];
            let Some(d) = range_ok(raw, &self.m, self.m.active_range_error()) else { continue };
            let az = a0 + width * (i as f64 / (POINTS as f64 - 1.0));
            let (pt, sin_th) = mount_point(d, az, &self.m, self.sin_tilt, self.cos_tilt);
            if !half_ok(self.m.scan_half, sin_th) {
                continue;
            }
            let mut w = yaw(pt, deg);
            if self.m.flip_upright {
                w[1] = -w[1];
                w[2] = -w[2];
            }
            out.push(([w[0] as f32, w[1] as f32, w[2] as f32], d as f32, deg as f32));
        }
    }
}

// --- Microstep self-calibration ---------------------------------------------

/// Measures the microstep error from the capture itself. Returns the
/// coefficients for `correct_microstep`, or None if nothing flat was found.
///
/// The cloud is cut into 120 mm cubes; the flat ones are kept, and in each the
/// offset of every point from the patch's plane is regressed on
/// lever * basis(shaft angle), with lever = n . (zhat x P) the arm that turns
/// an azimuth error into a displacement along the normal, after projecting a
/// free plane per patch out of both sides. One robust reweighting, two rounds.
pub fn fit_microstep(cap: &Capture, m: &Mount) -> Option<Vec<f64>> {
    let h = MICROSTEP_HARMONICS;
    let ncol = 2 * h;
    let patch_mm = 120.0;
    let mp = mount_points(cap, m, ScanHalf::Both).ok()?;
    // Points that survive, with their frame's commanded angle.
    let good_idx: Vec<usize> = (0..mp.good.len()).filter(|&j| mp.good[j]).collect();
    let plat_pt: Vec<f64> = good_idx.iter().map(|&j| mp.platform[j / POINTS]).collect();
    let mut coef = vec![0.0; ncol];
    let mut any = false;
    for _round in 0..2 {
        let pw: Vec<V3> = good_idx
            .par_iter()
            .zip(plat_pt.par_iter())
            .map(|(&j, &pl)| yaw(mp.p[j], correct_microstep(pl, Some(&coef))))
            .collect();
        // Two grids half a cell apart, so a surface cut by one grid's cell
        // boundary is whole in the other.
        let mut all = Patches::empty();
        for shift in [0.0, 0.5 * patch_mm] {
            let pt = patches::flat_patches(&pw, patch_mm, shift, 30.0, 3.0, 10.0);
            // Near zero lever the patch carries no information, only noise.
            let keep: Vec<bool> = pt
                .idx
                .iter()
                .zip(&pt.n)
                .map(|(&i, n)| {
                    let q = pw[i as usize];
                    (n[1] * q[0] - n[0] * q[1]).abs() > 150.0
                })
                .collect();
            if keep.iter().any(|&k| k) {
                all.append(pt.filter(&keep));
            }
        }
        if all.len() == 0 {
            break;
        }
        let rows = |i: usize, z: &mut [f64]| {
            let j = all.idx[i] as usize;
            let n = all.n[i];
            let q = pw[j];
            let lever = n[1] * q[0] - n[0] * q[1];
            z[0] = all.r[i];
            microstep_basis(plat_pt[j], h, &mut z[1..1 + ncol]);
            for v in &mut z[1..1 + ncol] {
                *v *= lever;
            }
        };
        let proj = Projected::new(&all.pid, &all.uv, all.n_patches, 1 + ncol, &rows);
        let sol = proj.solve(&rows, true);
        // A point at commanded angle + delta reads as -g * delta off its plane.
        let mut maxstep: f64 = 0.0;
        for k in 0..ncol {
            let step = -sol[k].to_degrees();
            coef[k] += step;
            maxstep = maxstep.max(step.abs());
        }
        any = true;
        if maxstep < 1e-4 {
            break;
        }
    }
    if any && coef.iter().all(|c| c.is_finite()) {
        Some(coef)
    } else {
        None
    }
}

/// 2nd/98th percentiles of a value, from a subsample.
pub fn robust_span(values: &[f32]) -> (f32, f32) {
    if values.is_empty() {
        return (0.0, 1.0);
    }
    let step = (values.len() / 200_000).max(1);
    let mut v: Vec<f64> = values.iter().step_by(step).filter(|x| x.is_finite()).map(|&x| x as f64).collect();
    if v.is_empty() {
        return (0.0, 1.0);
    }
    let lo = util::quantile_mut(&mut v, 0.02);
    let hi = util::quantile_mut(&mut v, 0.98);
    (lo as f32, hi.max(lo + 1e-3) as f32)
}
