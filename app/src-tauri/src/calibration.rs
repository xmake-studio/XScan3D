//! Self-calibration from an ordinary room scan: lidar roll, emitter spacing,
//! scan-plane tilt (tools/calibrate_mount.py) and the lidar's range
//! non-linearity (scan_proto.fit_range_error).
//!
//! Nothing needs a target: a room is full of surfaces known to be flat. The
//! big planes (walls, ceiling, floor) are found once and refitted under each
//! trial geometry, and the seam where the two ends of the sweep meet is scored
//! overhead, at the horizon and below, so no band is traded for another.

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
type IsCancelled<'a> = &'a (dyn Fn() -> bool + Sync);

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum Stage {
    Microstep,
    Range,
    Planes,
    RollSpacing,
    TiltSearch,
    AllThree,
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

fn seam_score(p: &[V3], platform: &[f32], span: f64) -> Scores {
    let margin = 10.0;
    let radius = 50.0;
    let end: Vec<usize> = (0..p.len()).filter(|&i| platform[i] as f64 > span - margin).step_by(2).collect();
    let start: Vec<V3> = (0..p.len()).filter(|&i| (platform[i] as f64) < -span + margin).map(|i| p[i]).collect();
    if end.is_empty() || start.len() < 10 {
        return Scores::default();
    }
    let tree = KdTree::new(&start);
    let res: Vec<(f64, f64)> = end
        .par_iter()
        .map_init(Vec::new, |buf, &i| {
            let a = p[i];
            tree.within(a, radius * radius, buf);
            if buf.len() < 10 {
                return (f64::NAN, 0.0);
            }
            let (c, cov, _) = util::centered_cov(buf.iter().map(|&j| start[j]));
            let (w, vec) = sym3_eigen(cov);
            if w[1] < 25.0 || w[0] > 0.05 * w[1] {
                return (f64::NAN, 0.0);
            }
            let el = a[2].atan2(a[0].hypot(a[1])).to_degrees();
            (dot(util::sub(a, c), vec[0]).abs(), el)
        })
        .collect();
    let band = |f: &dyn Fn(f64) -> bool| {
        let v: Vec<f64> = res.iter().filter(|(r, el)| r.is_finite() && f(*el)).map(|(r, _)| r.min(15.0)).collect();
        if v.is_empty() {
            0.0
        } else {
            v.iter().sum::<f64>() / v.len() as f64
        }
    };
    Scores {
        planes: 0.0,
        up: band(&|el| el > 45.0),
        horizon: band(&|el| el.abs() < 30.0),
        down: band(&|el| el < -30.0),
    }
}

/// scipy.optimize's Nelder-Mead (non-adaptive), with an explicit initial
/// simplex and its xatol/fatol/maxiter termination.
fn nelder_mead(
    f: &mut dyn FnMut(&[f64]) -> Result<f64, Cancelled>,
    simplex: Vec<Vec<f64>>,
    xatol: f64,
    fatol: f64,
    maxiter: usize,
) -> Result<Vec<f64>, Cancelled> {
    let (rho, chi, psi, sigma) = (1.0, 2.0, 0.5, 0.5);
    let n = simplex.len() - 1;
    let mut sim = simplex;
    let mut fsim: Vec<f64> = Vec::with_capacity(n + 1);
    for x in &sim {
        fsim.push(f(x)?);
    }
    let sort = |sim: &mut Vec<Vec<f64>>, fsim: &mut Vec<f64>| {
        let mut ord: Vec<usize> = (0..fsim.len()).collect();
        ord.sort_by(|&a, &b| fsim[a].total_cmp(&fsim[b]));
        *sim = ord.iter().map(|&i| sim[i].clone()).collect();
        *fsim = ord.iter().map(|&i| fsim[i]).collect();
    };
    sort(&mut sim, &mut fsim);
    let mut iterations = 1;
    while iterations < maxiter {
        let xspread = (1..=n).flat_map(|j| (0..n).map(move |k| (j, k))).map(|(j, k)| (sim[j][k] - sim[0][k]).abs()).fold(0.0, f64::max);
        let fspread = (1..=n).map(|j| (fsim[0] - fsim[j]).abs()).fold(0.0, f64::max);
        if xspread <= xatol && fspread <= fatol {
            break;
        }
        let mut xbar = vec![0.0; n];
        for x in &sim[..n] {
            for k in 0..n {
                xbar[k] += x[k] / n as f64;
            }
        }
        let comb = |a: f64, b: f64, x: &[f64]| -> Vec<f64> { (0..n).map(|k| a * xbar[k] + b * x[k]).collect() };
        let xr = comb(1.0 + rho, -rho, &sim[n]);
        let fxr = f(&xr)?;
        let mut shrink = false;
        if fxr < fsim[0] {
            let xe = comb(1.0 + rho * chi, -rho * chi, &sim[n]);
            let fxe = f(&xe)?;
            if fxe < fxr {
                sim[n] = xe;
                fsim[n] = fxe;
            } else {
                sim[n] = xr;
                fsim[n] = fxr;
            }
        } else if fxr < fsim[n - 1] {
            sim[n] = xr;
            fsim[n] = fxr;
        } else if fxr < fsim[n] {
            let xc = comb(1.0 + psi * rho, -psi * rho, &sim[n]);
            let fxc = f(&xc)?;
            if fxc <= fxr {
                sim[n] = xc;
                fsim[n] = fxc;
            } else {
                shrink = true;
            }
        } else {
            let xcc = comb(1.0 - psi, psi, &sim[n]);
            let fxcc = f(&xcc)?;
            if fxcc < fsim[n] {
                sim[n] = xcc;
                fsim[n] = fxcc;
            } else {
                shrink = true;
            }
        }
        if shrink {
            for j in 1..=n {
                let x: Vec<f64> = (0..n).map(|k| sim[0][k] + sigma * (sim[j][k] - sim[0][k])).collect();
                fsim[j] = f(&x)?;
                sim[j] = x;
            }
        }
        iterations += 1;
        sort(&mut sim, &mut fsim);
    }
    Ok(sim.swap_remove(0))
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

    struct StageInfo {
        stage: Stage,
        lo: f64,
        hi: f64,
        n: usize,
        expect: usize,
    }
    let info = std::cell::RefCell::new(StageInfo { stage: Stage::RollSpacing, lo: 0.05, hi: 0.30, n: 0, expect: 45 });
    let range_ref = std::cell::RefCell::new(range_now.clone());
    let mut score = |x: &[f64; 3]| -> Result<f64, Cancelled> {
        if cancelled() {
            return Err(Cancelled);
        }
        let r = range_ref.borrow().clone();
        let sm = detail(x, r.as_deref()).map_err(|_| Cancelled)?;
        let mut st = info.borrow_mut();
        st.n += 1;
        let f = (st.n as f64 / st.expect as f64).min(1.0);
        progress(st.stage, st.lo + f * (st.hi - st.lo));
        // The horizon seam must not get worse than it started: it is the one
        // usually already tuned by eye.
        let worse = (sm.horizon - before.horizon).max(0.0);
        Ok(sm.planes + sm.horizon + sm.down + 0.5 * sm.up + 4.0 * worse)
    };

    let polish = |x: [f64; 3], free: [bool; 3], step: f64, score: &mut dyn FnMut(&[f64; 3]) -> Result<f64, Cancelled>| -> Result<[f64; 3], CalibError> {
        let idx: Vec<usize> = (0..3).filter(|&i| free[i]).collect();
        let deltas = [0.3, 4.0, -0.5];
        let mut simplex = vec![idx.iter().map(|&i| x[i]).collect::<Vec<f64>>()];
        for &i in &idx {
            let mut y = x;
            y[i] += deltas[i] * step;
            simplex.push(idx.iter().map(|&k| y[k]).collect());
        }
        let mut sub = |v: &[f64]| -> Result<f64, Cancelled> {
            let mut y = x;
            for (k, &i) in idx.iter().enumerate() {
                y[i] = v[k];
            }
            score(&y)
        };
        let v = nelder_mead(&mut sub, simplex, 0.01, 1e-4, 150).map_err(|_| CalibError::Cancelled)?;
        let mut out = x;
        for (k, &i) in idx.iter().enumerate() {
            out[i] = v[k];
        }
        Ok(out)
    };

    let x = polish(x0, [true, true, false], 1.0, &mut score)?;
    {
        let mut st = info.borrow_mut();
        *st = StageInfo { stage: Stage::TiltSearch, lo: 0.30, hi: 0.45, n: 0, expect: 13 };
    }
    let mut best_t = (f64::INFINITY, x[2]);
    for k in 0..13 {
        let t = -3.0 + 0.5 * k as f64;
        let s = score(&[x[0], x[1], t]).map_err(|_| CalibError::Cancelled)?;
        if s < best_t.0 {
            best_t = (s, t);
        }
    }
    {
        let mut st = info.borrow_mut();
        *st = StageInfo { stage: Stage::AllThree, lo: 0.45, hi: 0.95, n: 0, expect: 80 };
    }
    let x = polish([x[0], x[1], best_t.1], [true, true, true], 0.5, &mut score)?;
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
