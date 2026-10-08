//! Self-calibration from an ordinary room scan: lidar roll, emitter spacing,
//! scan-plane tilt (tools/calibrate_mount.py) and the lidar's range
//! non-linearity (scan_proto.fit_range_error).
//!
//! Nothing needs a target: where the two ends of the sweep meet, every
//! surface is seen twice, once by each half of the lidar's revolution, and the
//! mount geometry is what makes the two copies agree. Roll, spacing and tilt
//! are fitted to that seam by Gauss-Newton, overhead, at the horizon and below
//! weighed alike, so no band is traded for another. The big planes (walls,
//! ceiling, floor) are found once and scored before and after, for the report.

use rayon::prelude::*;
use serde::Serialize;

use crate::capture::Capture;
use crate::geometry::{self, Mount, ScanHalf};
use crate::kdtree::KdTree;
use crate::patches::{self, Patches, Projected};
use crate::util::{self, cross, dot, sym3_eigen, Rng, V3};

pub const RANGE_HARMONICS: usize = 3;
pub const RANGE_DRIFT_TERMS: usize = 2;
/// Frames a calibration works with before it starts thinning.
pub const CALIBRATION_MAX_FRAMES: usize = 120_000;

#[derive(Debug)]
pub struct Cancelled;

type Progress<'a> = &'a (dyn Fn(Stage, f64) + Sync);
/// The cloud under a trial (roll, spacing, tilt).
type TrialCloud<'a> = &'a dyn Fn(&[f64; 3]) -> Result<Vec<V3>, CalibError>;
type IsCancelled<'a> = &'a (dyn Fn() -> bool + Sync);

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum Stage {
    Microstep,
    Range,
    Planes,
    Seam,
    RangeRefit,
    Done,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct Scores {
    pub planes: f64,
    pub up: f64,
    pub horizon: f64,
    pub down: f64,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct CalibResult {
    pub rotation: f64,
    pub spacing: f64,
    pub tilt: f64,
    pub range_error: Option<Vec<f64>>,
    pub range_error_before: Option<Vec<f64>>,
    pub range_at_3m_before: f64,
    pub range_at_3m_after: f64,
    pub before: Scores,
    pub after: Scores,
    pub stride: usize,
}

#[derive(Debug)]
pub enum CalibError {
    Cancelled,
    NoSweep,
    ShortSweep(f64),
    NoPlanes,
}

impl std::fmt::Display for CalibError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CalibError::Cancelled => write!(f, "cancelled"),
            CalibError::NoSweep => write!(f, "the scan has no usable sweep"),
            CalibError::ShortSweep(s) => write!(f, "the sweep only covers +-{s:.0} deg"),
            CalibError::NoPlanes => write!(f, "found fewer than two walls/ceilings/floors"),
        }
    }
}

fn mount_with(base: &Mount, x: &[f64; 3], range: Option<&[f64]>) -> Mount {
    let mut m = base.clone();
    m.lidar_rotation = x[0];
    m.emitter_spacing = x[1];
    m.scan_tilt = x[2];
    m.scan_half = ScanHalf::Both;
    m.flip_upright = false;
    m.range_correction = range.is_some();
    m.range_error = range.map(|r| r.to_vec());
    m
}

// --- Range self-calibration ----------------------------------------------------------

/// Regression rows of the range model: [r, cos(incidence) * basis(d)].
#[allow(clippy::too_many_arguments)]
fn range_rows<'a>(
    period: f64,
    power: f64,
    harmonics: usize,
    drift: usize,
    span: (f64, f64),
    dd: &'a [f64],
    cos_i: &'a [f64],
    r: &'a [f64],
) -> impl Fn(usize, &mut [f64]) + Sync + 'a {
    let ncol = 2 * harmonics * drift;
    move |i: usize, z: &mut [f64]| {
        z[0] = r[i];
        geometry::range_basis(dd[i], period, power, harmonics, drift, Some(span), &mut z[1..1 + ncol]);
        for v in &mut z[1..1 + ncol] {
            *v *= cos_i[i];
        }
    }
}

/// Measures the lidar's range non-linearity from a scan. Returns a model in
/// the RANGE_ERROR layout, `start` if a round found too little to go on, or
/// None when the scan does not show the error clearly enough to correct.
pub fn fit_range_error(cap: &Capture, base: &Mount, x: &[f64; 3], microstep: Option<&[f64]>, cancelled: IsCancelled) -> Result<Option<Vec<f64>>, Cancelled> {
    let harmonics = RANGE_HARMONICS;
    let drift = RANGE_DRIFT_TERMS;
    let patch_mm = 200.0;
    let mut total: Option<Vec<f64>> = None;
    let mut period = 0.0;
    let mut power = 0.0;
    let mut span = (0.0, 0.0);
    for rnd in 0..2 {
        if cancelled() {
            return Err(Cancelled);
        }
        let mut m = mount_with(base, x, total.as_deref());
        m.max_range = 0.0;
        let Ok(cloud) = geometry::build_cloud(cap, &m, microstep) else { return Ok(total) };
        let p: Vec<V3> = util::to_f64(&cloud.xyz);
        let d: Vec<f64> = cloud.dist.iter().map(|&v| v as f64).collect();
        let mut rows = Patches::empty();
        let mut cos_all: Vec<f64> = Vec::new();
        for shift in [0.0, 0.5 * patch_mm] {
            let pt = patches::flat_patches(&p, patch_mm, shift, 30.0, 6.0, 15.0);
            let cos_i: Vec<f64> = pt
                .idx
                .iter()
                .zip(&pt.n)
                .map(|(&i, n)| {
                    let q = p[i as usize];
                    dot(*n, util::scale(q, 1.0 / util::norm(q).max(1e-9)))
                })
                .collect();
            // Grazing surfaces barely move along their normal; very near ones
            // have no error to speak of.
            let keep: Vec<bool> = pt
                .idx
                .iter()
                .zip(&cos_i)
                .map(|(&i, c)| c.abs() > 0.3 && d[i as usize] > 800.0)
                .collect();
            if keep.iter().filter(|&&k| k).count() < 1000 {
                continue;
            }
            cos_all.extend(cos_i.iter().zip(&keep).filter(|(_, &k)| k).map(|(c, _)| *c));
            rows.append(pt.filter(&keep));
        }
        if rows.len() == 0 {
            return Ok(total);
        }
        let dd: Vec<f64> = rows.idx.iter().map(|&i| d[i as usize]).collect();
        if dd.iter().filter(|&&v| v > 2000.0).count() < 2000 {
            return Ok(total);
        }
        if rnd == 0 {
            let mut s = dd.clone();
            let lo = util::quantile_mut(&mut s, 0.01);
            let hi = util::quantile_mut(&mut s, 0.98);
            span = (lo, hi);
        }
        if rnd == 0 {
            // The period is set by the sensor's pixel pitch, so it is searched
            // rather than assumed. scan_proto searches on a random 150k
            // subsample; this is fast enough to use (up to 600k of) every row,
            // which makes the pick deterministic.
            let n = rows.len();
            let mut keep = vec![false; n];
            const SEARCH_ROWS: usize = 600_000;
            if n <= SEARCH_ROWS {
                keep.iter_mut().for_each(|k| *k = true);
            } else {
                let mut rng = Rng::new(0);
                let mut perm: Vec<usize> = (0..n).collect();
                for i in 0..SEARCH_ROWS {
                    let j = i + rng.below(n - i);
                    perm.swap(i, j);
                }
                for &i in &perm[..SEARCH_ROWS] {
                    keep[i] = true;
                }
            }
            let sub = rows.filter(&keep);
            let sub_dd: Vec<f64> = sub.idx.iter().map(|&i| d[i as usize]).collect();
            let sub_cos: Vec<f64> = cos_all.iter().zip(&keep).filter(|(_, &k)| k).map(|(c, _)| *c).collect();
            let err = |t: f64, pw: f64| {
                let f = range_rows(t, pw, harmonics, 1, span, &sub_dd, &sub_cos, &sub.r);
                patches::projected_error(&sub.pid, &sub.uv, sub.n_patches, 1 + 2 * harmonics, &f)
            };
            // The period does not depend on how the amplitude grows: found at
            // d^2 first, then the growth at that period.
            let periods: Vec<f64> = (0..).map(|k| 15.0 + 0.05 * k as f64).take_while(|&t| t < 40.0).collect();
            let best = periods
                .par_iter()
                .map(|&t| (err(t, 2.0).0, t))
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap();
            period = best.1;
            if cancelled() {
                return Err(Cancelled);
            }
            let powers: Vec<f64> = (0..15).map(|k| 1.5 + 0.25 * k as f64).collect();
            let fits: Vec<((f64, f64), f64)> = powers.par_iter().map(|&pw| (err(period, pw), pw)).collect();
            let ((e, base_err), pw) = *fits.iter().min_by(|a, b| a.0 .0.total_cmp(&b.0 .0)).unwrap();
            power = pw;
            // Not worth it unless it explains a real share of what is left.
            if e > 0.97 * base_err {
                return Ok(None);
            }
        }
        let f = range_rows(period, power, harmonics, drift, span, &dd, &cos_all, &rows.r);
        let cols = 1 + 2 * harmonics * drift;
        let proj = Projected::new(&rows.pid, &rows.uv, rows.n_patches, cols, &f);
        let sol = proj.solve(&f, true);
        total = Some(match total {
            None => {
                let mut v = vec![period, power, drift as f64, span.0, span.1];
                v.extend(sol);
                v
            }
            Some(t) => {
                let mut v = t[..5].to_vec();
                v.extend(t[5..].iter().zip(&sol).map(|(a, b)| a + b));
                v
            }
        });
    }
    Ok(total)
}

// --- Mount calibration ------------------------------------------------------------------

fn fit_plane(p: &[V3], tol: f64, rounds: usize) -> (V3, V3, f64) {
    let mut pts: Vec<V3> = p.to_vec();
    let mut n = [0.0, 0.0, 1.0];
    let mut c = [0.0; 3];
    let mut last_r: Vec<f64> = Vec::new();
    let mut last_keep: Vec<bool> = Vec::new();
    for _ in 0..rounds {
        if pts.len() < 3 {
            break;
        }
        let (cc, cov, _) = util::centered_cov(pts.iter().copied());
        c = cc;
        n = sym3_eigen(cov).1[0];
        let r: Vec<f64> = pts.iter().map(|q| dot(util::sub(*q, c), n)).collect();
        let keep: Vec<bool> = r.iter().map(|v| v.abs() < tol).collect();
        let all = keep.iter().all(|&k| k);
        last_r = r;
        last_keep = keep;
        if all {
            break;
        }
        pts = pts.iter().zip(&last_keep).filter(|(_, &k)| k).map(|(q, _)| *q).collect();
    }
    let kept: Vec<f64> = last_r.iter().zip(&last_keep).filter(|(_, &k)| k).map(|(r, _)| *r).collect();
    if kept.is_empty() {
        return (n, c, f64::INFINITY);
    }
    let mean = kept.iter().sum::<f64>() / kept.len() as f64;
    let var = kept.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>() / kept.len() as f64;
    (n, c, var.sqrt())
}

/// Indices of P on the largest level or plumb planes, one list each.
fn find_planes(p: &[V3], count: usize, voxel: f64, tol: f64, min_voxels: usize) -> Vec<Vec<u32>> {
    let mut rng = Rng::new(0);
    // One representative per voxel: the first point to land in it.
    let mut seen = std::collections::HashSet::new();
    let q: Vec<V3> = p.iter().filter(|&&x| seen.insert(util::cell_key(x, voxel, 0.0))).copied().collect();
    let mut alive = vec![true; q.len()];
    let mut planes = Vec::new();
    for _ in 0..40 {
        if planes.len() >= count {
            break;
        }
        let idx: Vec<usize> = (0..q.len()).filter(|&i| alive[i]).collect();
        if idx.len() < min_voxels {
            break;
        }
        let trials: Vec<[usize; 3]> = (0..400)
            .map(|_| {
                let c = rng.choose_distinct(idx.len(), 3);
                [idx[c[0]], idx[c[1]], idx[c[2]]]
            })
            .collect();
        let best = trials
            .par_iter()
            .filter_map(|t| {
                let (a, b, c) = (q[t[0]], q[t[1]], q[t[2]]);
                let nn = cross(util::sub(b, a), util::sub(c, a));
                let ln = util::norm(nn);
                if ln < 1e-6 {
                    return None;
                }
                let nn = util::scale(nn, 1.0 / ln);
                let k = idx.iter().filter(|&&i| dot(util::sub(q[i], a), nn).abs() < tol).count();
                Some((k, nn, a))
            })
            .reduce_with(|x, y| if y.0 > x.0 { y } else { x });
        let Some((k, n, c)) = best else { break };
        if k < min_voxels {
            break;
        }
        let on: Vec<usize> = idx.iter().copied().filter(|&i| dot(util::sub(q[i], c), n).abs() < tol).collect();
        for &i in &on {
            alive[i] = false;
        }
        // Only walls, ceilings and floors.
        if n[2].abs() > 0.1 && n[2].abs() < 0.97 {
            continue;
        }
        let onp: Vec<V3> = on.iter().map(|&i| q[i]).collect();
        let (n, c, _) = fit_plane(&onp, tol, 4);
        let axis = if n[2].abs() < 0.9 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
        let u = util::normalize(cross(n, axis));
        let v = cross(n, u);
        let cell = 80.0;
        let foot: std::collections::HashSet<(i64, i64)> = onp
            .iter()
            .map(|x| {
                let rel = util::sub(*x, c);
                ((dot(rel, u) / cell).floor() as i64, (dot(rel, v) / cell).floor() as i64)
            })
            .collect();
        let members: Vec<u32> = (0..p.len())
            .into_par_iter()
            .filter(|&i| {
                let rel = util::sub(p[i], c);
                if dot(rel, n).abs() >= 15.0 {
                    return false;
                }
                foot.contains(&((dot(rel, u) / cell).floor() as i64, (dot(rel, v) / cell).floor() as i64))
            })
            .map(|i| i as u32)
            .collect();
        planes.push(members);
    }
    planes
}

fn plane_score(p: &[V3], planes: &[Vec<u32>]) -> f64 {
    let (tot, wsum) = planes
        .par_iter()
        .map(|idx| {
            let pts: Vec<V3> = idx.iter().map(|&i| p[i as usize]).collect();
            let rms = fit_plane(&pts, 6.0, 3).2;
            let w = (idx.len() as f64).sqrt();
            (rms * w, w)
        })
        .reduce(|| (0.0, 0.0), |a, b| (a.0 + b.0, a.1 + b.1));
    tot / wsum.max(1e-12)
}

/// The two ends of the sweep that see the same directions: points past
/// `edge` at the end, and before `-edge` at the start.
///
/// A slice at shaft angle phi covers azimuth phi with one half of the lidar's
/// revolution and phi + 180 with the other, so a sweep of +-span sees every
/// direction with a shaft angle in (180 - span, span) twice: once at the end
/// and once, through the other half, at the start. The automatic sweep is
/// wider than +-90 by the tilt overlap, often by 20 degrees, so this is a
/// band tens of degrees wide rather than a line; a sweep of +-90 or less still
/// gets the last `margin` degrees, where the two ends at least come close.
fn seam_edge(span: f64) -> f64 {
    let margin = 10.0;
    (180.0 - span).min(span - margin)
}

/// Elevation bands the seam is balanced over: below, at and above the horizon.
const SEAM_BANDS: usize = 3;

fn seam_band(el: f64) -> usize {
    if el < -30.0 {
        0
    } else if el <= 30.0 {
        1
    } else {
        2
    }
}

/// A point from the end of the sweep and the patch of the start's surface it
/// should lie on. Cloud indices do not depend on the mount geometry, so a pair
/// found under one trial geometry can be measured under another.
struct SeamPair {
    end: u32,
    start: Vec<u32>,
    normal: V3,
    band: u8,
}

/// Pairs every other end point with the flat start surface within 50 mm.
fn seam_pairs(p: &[V3], platform: &[f32], span: f64) -> Vec<SeamPair> {
    let edge = seam_edge(span);
    let end: Vec<usize> = (0..p.len()).filter(|&i| platform[i] as f64 > edge).step_by(2).collect();
    let start: Vec<u32> = (0..p.len()).filter(|&i| (platform[i] as f64) < -edge).map(|i| i as u32).collect();
    if end.is_empty() || start.len() < 10 {
        return Vec::new();
    }
    let sp: Vec<V3> = start.iter().map(|&i| p[i as usize]).collect();
    let tree = KdTree::new(&sp);
    end.par_iter()
        .map_init(Vec::new, |buf, &i| {
            let a = p[i];
            tree.within(a, 50.0 * 50.0, buf);
            if buf.len() < 10 {
                return None;
            }
            let (c, cov, _) = util::centered_cov(buf.iter().map(|&j| sp[j]));
            let (w, vec) = sym3_eigen(cov);
            if w[1] < 25.0 || w[0] > 0.05 * w[1] {
                return None;
            }
            // Further off than this is another surface, not the same one.
            if dot(util::sub(a, c), vec[0]).abs() > 30.0 {
                return None;
            }
            let el = a[2].atan2(a[0].hypot(a[1])).to_degrees();
            Some(SeamPair { end: i as u32, start: buf.iter().map(|&j| start[j]).collect(), normal: vec[0], band: seam_band(el) as u8 })
        })
        .flatten()
        .collect()
}

/// Signed distance of each pair's end point from its start patch.
fn seam_residuals(p: &[V3], pairs: &[SeamPair]) -> Vec<f64> {
    pairs
        .par_iter()
        .map(|s| {
            let mut c = [0.0; 3];
            for &j in &s.start {
                c = util::add(c, p[j as usize]);
            }
            let c = util::scale(c, 1.0 / s.start.len() as f64);
            dot(util::sub(p[s.end as usize], c), s.normal)
        })
        .collect()
}

/// Mean distance across the seam overhead, at the horizon and below.
fn seam_score(p: &[V3], platform: &[f32], span: f64) -> Scores {
    let pairs = seam_pairs(p, platform, span);
    let r = seam_residuals(p, &pairs);
    let band = |b: usize| {
        let v: Vec<f64> = pairs.iter().zip(&r).filter(|(s, _)| s.band as usize == b).map(|(_, r)| r.abs().min(15.0)).collect();
        if v.is_empty() {
            0.0
        } else {
            v.iter().sum::<f64>() / v.len() as f64
        }
    };
    Scores { planes: 0.0, up: band(2), horizon: band(1), down: band(0) }
}

/// Huber loss, mm.
const HUBER: f64 = 3.0;

fn huber(r: f64) -> f64 {
    if r.abs() < HUBER {
        0.5 * r * r
    } else {
        HUBER * (r.abs() - 0.5 * HUBER)
    }
}

/// Fits roll, spacing and tilt to the seam by Gauss-Newton.
///
/// Each round pairs the two ends of the sweep under the current geometry,
/// measures how every pair's residual moves with each parameter (by finite
/// differences: three more clouds), and takes the robust least-squares step,
/// halved until it actually lowers the misfit of those same pairs. Each
/// elevation band weighs the same in total, so the few pairs overhead -- the
/// ceiling and whatever hangs from it -- count as much as the many below.
fn refine_seam(
    x0: [f64; 3],
    span: f64,
    cloud: TrialCloud,
    platform: &[f32],
    progress: &dyn Fn(f64),
    cancelled: IsCancelled,
) -> Result<[f64; 3], CalibError> {
    const ROUNDS: usize = 8;
    // Finite-difference steps, and the most one round may move: roll and tilt
    // in degrees, spacing in mm.
    let h = [0.05, 1.0, 0.1];
    let max_step = [1.0, 10.0, 1.0];
    let done = [0.002, 0.02, 0.002];
    let mut x = x0;
    for round in 0..ROUNDS {
        if cancelled() {
            return Err(CalibError::Cancelled);
        }
        progress(round as f64 / ROUNDS as f64);
        let p = cloud(&x)?;
        let pairs = seam_pairs(&p, platform, span);
        if pairs.len() < 100 {
            break;
        }
        let mut count = [0usize; SEAM_BANDS];
        for s in &pairs {
            count[s.band as usize] += 1;
        }
        let bw: Vec<f64> = count.iter().map(|&n| if n >= 50 { 1.0 / n as f64 } else { 0.0 }).collect();
        let w: Vec<f64> = pairs.iter().map(|s| bw[s.band as usize]).collect();
        let loss = |r: &[f64]| r.iter().zip(&w).map(|(r, w)| w * huber(*r)).sum::<f64>();
        let r0 = seam_residuals(&p, &pairs);
        let mut jac = Vec::with_capacity(3);
        for k in 0..3 {
            let mut y = x;
            y[k] += h[k];
            let rk = seam_residuals(&cloud(&y)?, &pairs);
            jac.push(rk.iter().zip(&r0).map(|(a, b)| (a - b) / h[k]).collect::<Vec<f64>>());
        }
        // Iteratively reweighted least squares for the Huber loss.
        let mut d = [0.0; 3];
        for _ in 0..6 {
            let mut ata = [0.0; 9];
            let mut atb = [0.0; 3];
            for i in 0..r0.len() {
                let e = r0[i] + (0..3).map(|k| jac[k][i] * d[k]).sum::<f64>();
                let wi = w[i] * if e.abs() < HUBER { 1.0 } else { HUBER / e.abs() };
                for k in 0..3 {
                    atb[k] -= wi * jac[k][i] * r0[i];
                    for l in 0..3 {
                        ata[k * 3 + l] += wi * jac[k][i] * jac[l][i];
                    }
                }
            }
            // A parameter the seam cannot see stays where it is.
            for k in 0..3 {
                ata[k * 3 + k] += 1e-9 * ata.iter().step_by(4).fold(0.0f64, |a, &b| a.max(b)).max(1e-30);
            }
            let Some(s) = util::solve_dense(&ata, &atb, 3) else { break };
            for k in 0..3 {
                d[k] = s[k].clamp(-max_step[k], max_step[k]);
            }
        }
        let before = loss(&r0);
        let mut t = 1.0;
        let mut moved = false;
        for _ in 0..5 {
            let y = [x[0] + t * d[0], x[1] + t * d[1], x[2] + t * d[2]];
            if loss(&seam_residuals(&cloud(&y)?, &pairs)) < before {
                x = y;
                moved = true;
                break;
            }
            t *= 0.5;
        }
        if !moved || (0..3).all(|k| (t * d[k]).abs() < done[k]) {
            break;
        }
    }
    progress(1.0);
    Ok(x)
}

/// Fits roll, spacing and tilt (and the range model around them) to one
/// capture of an ordinary room.
pub fn calibrate(cap: &Capture, base: &Mount, progress: Progress, cancelled: IsCancelled) -> Result<CalibResult, CalibError> {
    let x0 = [base.lidar_rotation, base.emitter_spacing, base.scan_tilt];
    let range_start: Option<Vec<f64>> = base.active_range_error().map(|r| r.to_vec());

    // Long sweeps are thinned: density past ~120k frames buys nothing.
    let all_end = cap.samples.iter().all(|s| s.end_angle >= crate::protocol::RAW_ANGLE_MIN);
    let mut stride = if cap.len() > CALIBRATION_MAX_FRAMES { (cap.len() + CALIBRATION_MAX_FRAMES - 1) / CALIBRATION_MAX_FRAMES } else { 1 };
    if !all_end {
        stride = 1;
    }
    let thin;
    let cap = if stride > 1 {
        thin = cap.thinned(stride);
        &thin
    } else {
        cap
    };

    progress(Stage::Microstep, 0.0);
    let m0 = mount_with(base, &x0, range_start.as_deref());
    let micro = geometry::fit_microstep(cap, &m0);
    let micro = micro.as_deref();
    if cancelled() {
        return Err(CalibError::Cancelled);
    }

    let fit_range = |x: &[f64; 3]| -> Result<Option<Vec<f64>>, CalibError> {
        let fitted = fit_range_error(cap, base, x, micro, cancelled).map_err(|_| CalibError::Cancelled)?;
        Ok(fitted.or_else(|| range_start.clone()))
    };

    // Point clouds under a trial geometry, with the shaft angle of each point.
    let cloud = |x: &[f64; 3], range: Option<&[f64]>| -> Result<(Vec<V3>, Vec<f32>), CalibError> {
        let m = mount_with(base, x, range);
        let c = geometry::build_cloud(cap, &m, micro).map_err(|_| CalibError::NoSweep)?;
        Ok((util::to_f64(&c.xyz), c.platform))
    };

    progress(Stage::Range, 0.01);
    let mut range_now = fit_range(&x0)?;
    let (p0, plat0) = cloud(&x0, range_now.as_deref())?;
    let span = plat0.iter().fold(0.0f64, |a, &b| a.max((b as f64).abs()));
    if span < 45.0 {
        return Err(CalibError::ShortSweep(span));
    }
    progress(Stage::Planes, 0.04);
    let planes = find_planes(&p0, 8, 40.0, 12.0, 400);
    if planes.len() < 2 {
        return Err(CalibError::NoPlanes);
    }

    let detail = |x: &[f64; 3], range: Option<&[f64]>| -> Result<Scores, CalibError> {
        let (p, plat) = cloud(x, range)?;
        let mut s = seam_score(&p, &plat, span);
        s.planes = plane_score(&p, &planes);
        Ok(s)
    };
    let before = detail(&x0, range_start.as_deref())?;

    let seam_cloud = |x: &[f64; 3]| -> Result<Vec<V3>, CalibError> { Ok(cloud(x, range_now.as_deref())?.0) };
    let x = refine_seam(x0, span, &seam_cloud, &plat0, &|f| progress(Stage::Seam, 0.05 + 0.9 * f), cancelled)?;
    progress(Stage::RangeRefit, 0.96);
    range_now = fit_range(&x)?;
    let after = detail(&x, range_now.as_deref())?;
    progress(Stage::Done, 1.0);
    Ok(CalibResult {
        rotation: x[0],
        spacing: x[1],
        tilt: x[2],
        range_at_3m_before: geometry::range_error_at(range_start.as_deref(), 3000.0),
        range_at_3m_after: geometry::range_error_at(range_now.as_deref(), 3000.0),
        range_error: range_now,
        range_error_before: range_start,
        before,
        after,
        stride,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{Sample, POINTS, RAW_ANGLE_MAX, RAW_ANGLE_MIN};

    const RAW_PER_DEG: f64 = (RAW_ANGLE_MAX - RAW_ANGLE_MIN) as f64 / 360.0;

    /// Distance along a ray to the first surface of a room: a box turned 25
    /// degrees off the seam, with a smaller box hanging under its ceiling.
    fn trace(o: V3, u: V3) -> f64 {
        let (s, c) = 25f64.to_radians().sin_cos();
        let rot = |v: V3| [c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]];
        let (o, u) = (rot(o), rot(u));
        let (lo, hi) = ([-2200.0, -1800.0, -900.0], [2000.0, 2300.0, 1500.0]);
        let mut t = f64::INFINITY;
        for k in 0..3 {
            if u[k] > 1e-12 {
                t = t.min((hi[k] - o[k]) / u[k]);
            } else if u[k] < -1e-12 {
                t = t.min((lo[k] - o[k]) / u[k]);
            }
        }
        // The box under the ceiling (slab method).
        let (blo, bhi) = ([-700.0, 1500.0, 1150.0], [300.0, 2300.0, 1500.0]);
        let (mut t0, mut t1) = (0.0f64, f64::INFINITY);
        for k in 0..3 {
            if u[k].abs() < 1e-12 {
                if o[k] < blo[k] || o[k] > bhi[k] {
                    t1 = -1.0;
                }
                continue;
            }
            let (a, b) = ((blo[k] - o[k]) / u[k], (bhi[k] - o[k]) / u[k]);
            t0 = t0.max(a.min(b));
            t1 = t1.min(a.max(b));
        }
        if t1 >= t0 && t0 > 0.0 {
            t = t.min(t0);
        }
        t
    }

    /// A +-`span` sweep of that room by a rig bolted together as `m`.
    fn synthetic(m: &Mount, span: f64) -> Capture {
        let (st, ct) = m.scan_tilt.to_radians().sin_cos();
        let mut cap = Capture::new();
        let step = 0.5;
        let frame = POINTS as f64 * step;
        let revs = (2.0 * span / step) as usize;
        for r in 0..=revs {
            let plat = -span + r as f64 * step;
            let (sy, cy) = plat.to_radians().sin_cos();
            let yaw = |v: V3| [cy * v[0] - sy * v[1], sy * v[0] + cy * v[1], v[2]];
            for f in 0..(360.0 / frame) as usize {
                let a0 = f as f64 * frame;
                let mut dist = [0u16; POINTS];
                for (i, d) in dist.iter_mut().enumerate() {
                    let az = a0 + i as f64 * step;
                    let th = ((if m.lidar_reverse { -az } else { az }) + m.lidar_rotation).to_radians();
                    let (s, c) = th.sin_cos();
                    let b = 0.5 * m.emitter_spacing;
                    let tilt = |v: V3| [v[0], ct * v[1] - st * v[2], st * v[1] + ct * v[2]];
                    let o = yaw(tilt([b * c, 0.0, -b * s]));
                    let u = yaw(tilt([s, 0.0, c]));
                    *d = trace(o, u).round().min(8000.0) as u16;
                }
                let raw = |deg: f64| RAW_ANGLE_MIN + (deg * RAW_PER_DEG).round() as u16;
                cap.samples.push(Sample {
                    t_us: 0,
                    platform: plat as f32,
                    speed: 0,
                    raw_angle: raw(a0),
                    dist,
                    end_angle: raw(a0 + (POINTS - 1) as f64 * step),
                    capturing: true,
                });
            }
        }
        cap
    }

    #[test]
    fn seam_fit_recovers_the_mount() {
        let mut truth = Mount::default();
        truth.lidar_rotation = 161.9;
        truth.emitter_spacing = -38.0;
        truth.scan_tilt = -1.6;
        truth.range_correction = false;
        // As wide as the automatic sweep gets: the seam is a broad band, not
        // the last few degrees at each end.
        let cap = synthetic(&truth, 110.0);
        let cloud = |x: &[f64; 3]| -> Result<(Vec<V3>, Vec<f32>), CalibError> {
            let c = geometry::build_cloud(&cap, &mount_with(&truth, x, None), None).map_err(|_| CalibError::NoSweep)?;
            Ok((util::to_f64(&c.xyz), c.platform))
        };
        let (_, plat) = cloud(&[161.5, -33.0, 0.0]).unwrap();
        let span = plat.iter().fold(0.0f64, |a, &b| a.max((b as f64).abs()));
        let trial = |x: &[f64; 3]| Ok(cloud(x)?.0);
        let x = refine_seam([161.5, -33.0, 0.0], span, &trial, &plat, &|_| {}, &|| false).unwrap();
        assert!((x[0] - 161.9).abs() < 0.05, "roll {x:?}");
        assert!((x[1] + 38.0).abs() < 1.0, "spacing {x:?}");
        assert!((x[2] + 1.6).abs() < 0.05, "tilt {x:?}");
        // The seam score sees the whole overlap: well apart before, together after.
        let (p0, _) = cloud(&[161.5, -33.0, 0.0]).unwrap();
        let (p1, _) = cloud(&x).unwrap();
        let (s0, s1) = (seam_score(&p0, &plat, span), seam_score(&p1, &plat, span));
        assert!(s0.horizon > 2.0 && s1.horizon < 1.0, "{s0:?} -> {s1:?}");
    }
}
