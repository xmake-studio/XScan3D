//! Rigid registration of one scan onto another (SLAM merge). A port of
//! tools/registration.py: point-to-plane ICP run coarse-to-fine over voxel
//! grids, a floor-plan correlation plus yaw ring in front of it to survive the
//! operator turning the rig, and a sub-voxel polish of the winner.
//!
//! Units are millimetres. Nothing here knows about scans or the UI: it takes
//! two clouds and returns a 4x4.

use rayon::prelude::*;
use rustfft::{num_complex::Complex, FftPlanner};

use crate::kdtree::KdTree;
use crate::util::{self, cross, dot, sym3_eigen, Mat4, V3};

// --- transforms ----------------------------------------------------------------

pub fn identity() -> Mat4 {
    util::identity4()
}

fn rotvec_to_matrix(w: V3) -> [[f64; 3]; 3] {
    let theta = util::norm(w);
    if theta < 1e-12 {
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }
    let k = util::scale(w, 1.0 / theta);
    let kk = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]];
    let (s, c) = theta.sin_cos();
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let mut k2 = 0.0;
            for m in 0..3 {
                k2 += kk[i][m] * kk[m][j];
            }
            r[i][j] = if i == j { 1.0 } else { 0.0 } + s * kk[i][j] + (1.0 - c) * k2;
        }
    }
    r
}

pub fn yaw_matrix(deg: f64) -> Mat4 {
    let (s, c) = deg.to_radians().sin_cos();
    let mut t = identity();
    t[0] = c;
    t[1] = -s;
    t[4] = s;
    t[5] = c;
    t
}

fn apply(t: &Mat4, p: &[V3]) -> Vec<V3> {
    util::transform_points(t, p)
}

fn rotation_of(t: &Mat4) -> [[f64; 3]; 3] {
    [[t[0], t[1], t[2]], [t[4], t[5], t[6]], [t[8], t[9], t[10]]]
}

/// Rotation angle of a transform, degrees.
pub fn rotation_angle_deg(t: &Mat4) -> f64 {
    let tr = t[0] + t[5] + t[10];
    ((tr - 1.0) / 2.0).clamp(-1.0, 1.0).acos().to_degrees()
}

// --- preparation -------------------------------------------------------------------

pub fn voxel_downsample(p: &[V3], size: f64) -> Vec<V3> {
    util::voxel_centroids(p, size)
}

/// Surface normal per point from the local neighbourhood's PCA. Signs are left
/// alone: point-to-plane ICP does not care.
pub fn estimate_normals(p: &[V3], tree: &KdTree, k: usize) -> Vec<V3> {
    let k = k.min(p.len());
    p.par_iter()
        .map_init(Vec::new, |buf, &q| {
            tree.knn(q, k, buf);
            let (_, cov, n) = util::centered_cov(buf.iter().map(|&(i, _)| p[i]));
            if n < 3 {
                return [0.0, 0.0, 1.0];
            }
            // numpy's cov here divides by k-1; scale does not change vectors.
            sym3_eigen(cov).1[0]
        })
        .collect()
}

/// Per-point l0/(l0+l1+l2) of the local covariance. 0 = flat, ~1/3 = blob.
fn surface_variation(p: &[V3], k: usize) -> Vec<f64> {
    let tree = KdTree::new(p);
    let k = k.min(p.len());
    p.par_iter()
        .map_init(Vec::new, |buf, &q| {
            tree.knn(q, k, buf);
            let (_, cov, _) = util::centered_cov(buf.iter().map(|&(i, _)| p[i]));
            let (w, _) = sym3_eigen(cov);
            w[0] / (w[0] + w[1] + w[2] + 1e-12)
        })
        .collect()
}

/// Keeps points on surfaces, drops volumetric clutter (foliage, grass). Fails
/// safe: returns the input when almost nothing reads as structure.
pub fn structure_filter(p: &[V3], voxel: f64, k: usize, max_variation: f64) -> Vec<V3> {
    if p.len() < k || voxel <= 0.0 {
        return p.to_vec();
    }
    let d = voxel_downsample(p, voxel);
    if d.len() < k {
        return p.to_vec();
    }
    let var = surface_variation(&d, k);
    let keep_keys: std::collections::HashSet<u64> = d
        .iter()
        .zip(&var)
        .filter(|(_, &v)| v <= max_variation)
        .map(|(q, _)| util::cell_key(*q, voxel, 0.0))
        .collect();
    if keep_keys.len() < 10 {
        return p.to_vec();
    }
    p.par_iter().filter(|&&q| keep_keys.contains(&util::cell_key(q, voxel, 0.0))).copied().collect()
}

/// Points on near-vertical surfaces, seen from above: the scene's floor plan.
fn wall_footprint(p: &[V3], voxel: f64, max_nz: f64) -> Vec<V3> {
    let d = voxel_downsample(p, voxel);
    if d.len() < 20 {
        return d;
    }
    let tree = KdTree::new(&d);
    let n = estimate_normals(&d, &tree, 18);
    let w: Vec<V3> = d.iter().zip(&n).filter(|(_, nn)| nn[2].abs() < max_nz).map(|(q, _)| *q).collect();
    if w.len() >= 20 {
        w
    } else {
        d
    }
}

fn plan_grid(xy: &[[f64; 2]], lo: [f64; 2], n: usize, cell: f64) -> Vec<f64> {
    let mut g = vec![0.0; n * n];
    for q in xy {
        let i = ((q[0] - lo[0]) / cell).floor();
        let j = ((q[1] - lo[1]) / cell).floor();
        if i >= 0.0 && j >= 0.0 && (i as usize) < n && (j as usize) < n {
            g[i as usize * n + j as usize] = 1.0;
        }
    }
    g
}

fn fft2(data: &mut [Complex<f64>], n: usize, inverse: bool, planner: &mut FftPlanner<f64>) {
    let fft = if inverse { planner.plan_fft_inverse(n) } else { planner.plan_fft_forward(n) };
    for row in data.chunks_mut(n) {
        fft.process(row);
    }
    let mut col = vec![Complex::new(0.0, 0.0); n];
    for c in 0..n {
        for r in 0..n {
            col[r] = data[r * n + c];
        }
        fft.process(&mut col);
        for r in 0..n {
            data[r * n + c] = col[r];
        }
    }
}

/// Whole-pose guesses from correlating the two clouds' floor plans: for each
/// yaw the FFT scores every translation at once. [(T, yaw)] best first.
pub fn footprint_starts(src: &[V3], tgt: &[V3], yaw_steps: usize, cell0: f64, per_yaw: usize, top: usize) -> Vec<(Mat4, f64)> {
    let voxel = 80.0;
    let max_cells = 512usize;
    let ws = wall_footprint(src, voxel, 0.35);
    let wt = wall_footprint(tgt, voxel, 0.35);
    if ws.len() < 20 || wt.len() < 20 {
        return vec![];
    }
    let ptp = |p: &[V3]| {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for q in p {
            for d in 0..2 {
                lo[d] = lo[d].min(q[d]);
                hi[d] = hi[d].max(q[d]);
            }
        }
        (hi[0] - lo[0]).max(hi[1] - lo[1])
    };
    let ext = ptp(&wt).max(ptp(&ws));
    let mut cell = cell0;
    let mut n = (ext / cell).ceil() as usize * 2 + 4;
    if n > max_cells {
        cell *= n as f64 / max_cells as f64;
        n = max_cells;
    }
    let mean2 = |p: &[[f64; 2]]| {
        let mut s = [0.0; 2];
        for q in p {
            s[0] += q[0];
            s[1] += q[1];
        }
        [s[0] / p.len() as f64, s[1] / p.len() as f64]
    };
    let wt2: Vec<[f64; 2]> = wt.iter().map(|q| [q[0], q[1]]).collect();
    let mt = mean2(&wt2);
    let t_lo = [mt[0] - n as f64 * cell / 2.0, mt[1] - n as f64 * cell / 2.0];
    let mut planner = FftPlanner::new();
    let mut f: Vec<Complex<f64>> = plan_grid(&wt2, t_lo, n, cell).into_iter().map(|v| Complex::new(v, 0.0)).collect();
    fft2(&mut f, n, false, &mut planner);

    let median_z = |p: &[V3]| {
        let mut z: Vec<f64> = p.iter().map(|q| q[2]).collect();
        util::median_mut(&mut z)
    };
    let dz = median_z(tgt) - median_z(src);

    let steps = yaw_steps.max(1);
    let found: Vec<Vec<(f64, f64, f64, f64)>> = (0..steps)
        .into_par_iter()
        .map(|k| {
            let deg = 360.0 * k as f64 / steps as f64;
            let (s, c) = deg.to_radians().sin_cos();
            let sp: Vec<[f64; 2]> = ws.iter().map(|q| [c * q[0] - s * q[1], s * q[0] + c * q[1]]).collect();
            let ms = mean2(&sp);
            let s_lo = [ms[0] - n as f64 * cell / 2.0, ms[1] - n as f64 * cell / 2.0];
            let hgrid = plan_grid(&sp, s_lo, n, cell);
            let hsum: f64 = hgrid.iter().sum::<f64>().max(1.0);
            let mut h: Vec<Complex<f64>> = hgrid.into_iter().map(|v| Complex::new(v, 0.0)).collect();
            let mut planner = FftPlanner::new();
            fft2(&mut h, n, false, &mut planner);
            let mut prod: Vec<Complex<f64>> = f.iter().zip(&h).map(|(a, b)| a * b.conj()).collect();
            fft2(&mut prod, n, true, &mut planner);
            let norm = (n * n) as f64;
            let corr: Vec<f64> = prod.iter().map(|v| v.re / norm / hsum).collect();
            let mut order: Vec<usize> = (0..corr.len()).collect();
            let take = per_yaw.max(1).min(order.len());
            order.select_nth_unstable_by(take - 1, |&a, &b| corr[b].total_cmp(&corr[a]));
            order[..take]
                .iter()
                .map(|&fi| {
                    let (i, j) = (fi / n, fi % n);
                    let di = if i > n / 2 { i as f64 - n as f64 } else { i as f64 };
                    let dj = if j > n / 2 { j as f64 - n as f64 } else { j as f64 };
                    let dx = (t_lo[0] - s_lo[0]) + di * cell;
                    let dy = (t_lo[1] - s_lo[1]) + dj * cell;
                    (corr[fi], deg, dx, dy)
                })
                .collect()
        })
        .collect();
    let mut found: Vec<(f64, f64, f64, f64)> = found.into_iter().flatten().collect();
    found.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut out = Vec::new();
    let mut seen: Vec<(i64, i64, i64)> = Vec::new();
    for (_s, deg, dx, dy) in found {
        let key = ((deg * 1000.0) as i64, (dx / (cell * 3.0)).round() as i64, (dy / (cell * 3.0)).round() as i64);
        if seen.contains(&key) {
            continue;
        }
        seen.push(key);
        let mut t = yaw_matrix(deg);
        t[3] = dx;
        t[7] = dy;
        t[11] = dz;
        out.push((t, deg));
        if out.len() >= top {
            break;
        }
    }
    out
}

/// A target cloud prepared once and reused across ICP iterations.
pub struct Target {
    pub xyz: Vec<V3>,
    pub tree: KdTree,
    pub normals: Vec<V3>,
}

impl Target {
    pub fn new(xyz: Vec<V3>) -> Target {
        let tree = KdTree::new(&xyz);
        let normals = estimate_normals(&xyz, &tree, 18);
        Target { xyz, tree, normals }
    }
}

// --- the solver ----------------------------------------------------------------------

/// One Gauss-Newton step of point-to-plane ICP, as an incremental 4x4.
fn point_to_plane_step(p: &[V3], q: &[V3], n: &[V3]) -> Mat4 {
    let (ata, atb) = p
        .par_iter()
        .zip(q.par_iter())
        .zip(n.par_iter())
        .fold(
            || ([0.0f64; 36], [0.0f64; 6]),
            |(mut ata, mut atb), ((&pp, &qq), &nn)| {
                let r = dot(util::sub(pp, qq), nn);
                let c = cross(pp, nn);
                let a = [c[0], c[1], c[2], nn[0], nn[1], nn[2]];
                for i in 0..6 {
                    atb[i] -= a[i] * r;
                    for j in 0..6 {
                        ata[i * 6 + j] += a[i] * a[j];
                    }
                }
                (ata, atb)
            },
        )
        .reduce(
            || ([0.0f64; 36], [0.0f64; 6]),
            |(mut a1, mut b1), (a2, b2)| {
                for i in 0..36 {
                    a1[i] += a2[i];
                }
                for i in 0..6 {
                    b1[i] += b2[i];
                }
                (a1, b1)
            },
        );
    let mut a = ata;
    let tr: f64 = (0..6).map(|i| a[i * 6 + i]).sum::<f64>();
    for i in 0..6 {
        a[i * 6 + i] += 1e-9 * tr.max(1.0) / 6.0;
    }
    let Some(x) = util::solve_dense(&a, &atb, 6) else { return identity() };
    let r = rotvec_to_matrix([x[0], x[1], x[2]]);
    let mut t = identity();
    for i in 0..3 {
        for j in 0..3 {
            t[i * 4 + j] = r[i][j];
        }
    }
    t[3] = x[3];
    t[7] = x[4];
    t[11] = x[5];
    t
}

/// Worst/best eigenvalue ratio of the point-to-plane normal matrix: 0 = free
/// to slide, 1 = rigid.
fn stability(p: &[V3], n: &[V3], scale: f64) -> f64 {
    if p.len() < 10 || scale <= 0.0 {
        return 0.0;
    }
    let mut ata = [0.0f64; 36];
    for (pp, nn) in p.iter().zip(n) {
        let c = util::scale(cross(*pp, *nn), 1.0 / scale);
        let a = [c[0], c[1], c[2], nn[0], nn[1], nn[2]];
        for i in 0..6 {
            for j in 0..6 {
                ata[i * 6 + j] += a[i] * a[j];
            }
        }
    }
    let m = nalgebra::Matrix6::from_row_slice(&ata) / p.len() as f64;
    let e = nalgebra::SymmetricEigen::new(m).eigenvalues;
    let mut w: Vec<f64> = e.iter().copied().collect();
    w.sort_by(|a, b| a.total_cmp(b));
    if w[5] <= 1e-12 {
        return 0.0;
    }
    w[0].max(0.0) / w[5]
}

/// Point-to-plane ICP. Returns (T, fitness, rmse).
pub fn icp(src: &[V3], tgt: &Target, init: &Mat4, max_dist: f64, max_iter: usize, trim: f64, tol: f64) -> (Mat4, f64, f64) {
    let mut t = *init;
    if src.len() < 10 || tgt.xyz.len() < 10 {
        return (t, 0.0, f64::INFINITY);
    }
    let md2 = max_dist * max_dist;
    let mut fitness = 0.0;
    let mut rmse = f64::INFINITY;
    let mut prev: Option<f64> = None;
    for _ in 0..max_iter {
        let pa = apply(&t, src);
        let hits: Vec<Option<(usize, f64)>> = pa.par_iter().map(|&q| tgt.tree.nearest(q, md2)).collect();
        let mut pairs: Vec<(usize, usize, f64)> = hits
            .iter()
            .enumerate()
            .filter_map(|(i, h)| h.map(|(j, d2)| (i, j, d2.sqrt())))
            .collect();
        let n_ok = pairs.len();
        if n_ok < 10 {
            return (t, 0.0, f64::INFINITY);
        }
        fitness = n_ok as f64 / src.len() as f64;
        rmse = (pairs.iter().map(|x| x.2 * x.2).sum::<f64>() / n_ok as f64).sqrt();
        if trim > 0.0 && trim < 1.0 && n_ok > 50 {
            let mut d: Vec<f64> = pairs.iter().map(|x| x.2).collect();
            let thr = util::quantile_mut(&mut d, trim);
            pairs.retain(|x| x.2 <= thr);
        }
        if pairs.len() < 10 {
            return (t, fitness, rmse);
        }
        let p: Vec<V3> = pairs.iter().map(|x| pa[x.0]).collect();
        let q: Vec<V3> = pairs.iter().map(|x| tgt.xyz[x.1]).collect();
        let nn: Vec<V3> = pairs.iter().map(|x| tgt.normals[x.1]).collect();
        let step = point_to_plane_step(&p, &q, &nn);
        t = util::mat4_mul(&step, &t);
        if let Some(pr) = prev {
            if (pr - rmse).abs() < tol * rmse.max(1.0) {
                break;
            }
        }
        prev = Some(rmse);
    }
    (t, fitness, rmse)
}

#[derive(Clone, Copy, Debug)]
pub struct Metrics {
    pub fitness: f64,
    pub rmse: f64,
    pub stability: f64,
    pub overlap: f64,
}

/// Scores an alignment: one-way fitness, rmse, stability and the mutual
/// overlap (the smaller of the two directions).
pub fn evaluate(src: &[V3], tgt: &Target, t: &Mat4, max_dist: f64) -> Metrics {
    let zero = Metrics { fitness: 0.0, rmse: f64::INFINITY, stability: 0.0, overlap: 0.0 };
    if src.len() < 10 || tgt.xyz.len() < 10 {
        return zero;
    }
    let md2 = max_dist * max_dist;
    let p = apply(t, src);
    let hits: Vec<Option<(usize, f64)>> = p.par_iter().map(|&q| tgt.tree.nearest(q, md2)).collect();
    let ok: Vec<(usize, usize, f64)> = hits
        .iter()
        .enumerate()
        .filter_map(|(i, h)| h.map(|(j, d2)| (i, j, d2)))
        .collect();
    if ok.is_empty() {
        return zero;
    }
    let fitness = ok.len() as f64 / src.len() as f64;
    let rmse = (ok.iter().map(|x| x.2).sum::<f64>() / ok.len() as f64).sqrt();
    let back = KdTree::new(&p);
    let n_back = tgt.xyz.par_iter().filter(|&&q| back.nearest(q, md2).is_some()).count();
    let overlap = fitness.min(n_back as f64 / tgt.xyz.len() as f64);
    let pm: Vec<V3> = ok.iter().map(|x| p[x.0]).collect();
    let mut c = [0.0; 3];
    for q in &pm {
        c = util::add(c, *q);
    }
    c = util::scale(c, 1.0 / pm.len() as f64);
    let centered: Vec<V3> = pm.iter().map(|q| util::sub(*q, c)).collect();
    let sc = (centered.iter().map(|q| util::norm(*q)).sum::<f64>() / centered.len() as f64).max(1.0);
    let nn: Vec<V3> = ok.iter().map(|x| tgt.normals[x.1]).collect();
    let stab = stability(&centered, &nn, sc);
    Metrics { fitness, rmse, stability: stab, overlap }
}

fn score(m: &Metrics) -> f64 {
    m.overlap - 0.001 * m.rmse.min(1000.0) + 0.25 * m.stability
}

// --- the thing the app calls -----------------------------------------------------------

#[derive(Clone, Debug)]
pub struct RegParams {
    pub voxel: f64,
    pub levels: usize,
    pub yaw_steps: usize,
    pub init: Option<Mat4>,
    pub yaw_hint: Option<f64>,
    pub max_points: usize,
    pub top_k: usize,
    pub min_voxel: Option<f64>,
    pub fine_points: usize,
    pub structure: bool,
    pub struct_max_variation: f64,
    pub struct_k: usize,
}

impl Default for RegParams {
    fn default() -> Self {
        RegParams {
            voxel: 40.0,
            levels: 3,
            yaw_steps: 12,
            init: None,
            yaw_hint: None,
            max_points: 60000,
            top_k: 4,
            min_voxel: None,
            fine_points: 150000,
            structure: false,
            struct_max_variation: 0.06,
            struct_k: 16,
        }
    }
}

#[derive(Clone, Debug, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct RegResult {
    pub t: Mat4,
    pub fitness: f64,
    pub rmse: f64,
    pub stability: f64,
    pub overlap: f64,
    pub voxel: f64,
    pub yaw: f64,
    pub source_points: usize,
    pub target_points: usize,
}

/// Progress stages reported through the callback, for localisation.
#[derive(Clone, Copy, Debug, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub enum RegStage {
    Structure,
    FloorPlan,
    Orientation,
    Refine,
    Polish,
}

fn prep(p: &[V3], size: f64, cap: usize) -> Vec<V3> {
    let d = voxel_downsample(p, size);
    if d.len() > cap {
        let stride = (d.len() as f64 / cap as f64).ceil() as usize;
        d.into_iter().step_by(stride).collect()
    } else {
        d
    }
}

/// Aligns `source` onto `target`.
pub fn register(
    source: &[V3],
    target: &[V3],
    prm: &RegParams,
    progress: &(dyn Fn(RegStage, usize, usize) + Sync),
    cancelled: &(dyn Fn() -> bool + Sync),
) -> Option<RegResult> {
    let mut result = RegResult {
        t: identity(),
        fitness: 0.0,
        rmse: f64::INFINITY,
        stability: 0.0,
        overlap: 0.0,
        voxel: prm.voxel,
        yaw: 0.0,
        source_points: source.len(),
        target_points: target.len(),
    };
    if source.len() < 100 || target.len() < 100 {
        return Some(result);
    }
    let (src_full, tgt_full): (Vec<V3>, Vec<V3>) = if prm.structure {
        progress(RegStage::Structure, 0, 1);
        let s = structure_filter(source, prm.voxel, prm.struct_k, prm.struct_max_variation);
        let t = structure_filter(target, prm.voxel, prm.struct_k, prm.struct_max_variation);
        if s.len() < 100 || t.len() < 100 {
            return Some(result);
        }
        (s, t)
    } else {
        (source.to_vec(), target.to_vec())
    };
    if cancelled() {
        return None;
    }
    let voxel = prm.voxel;
    let sizes: Vec<f64> = (0..prm.levels).map(|i| voxel * 2f64.powi((prm.levels - 1 - i) as i32)).collect();

    let coarse_src = prep(&src_full, sizes[0], prm.max_points / 4);
    let coarse_tgt_xyz = prep(&tgt_full, sizes[0], prm.max_points / 2);
    let mean = |p: &[V3]| {
        let mut s = [0.0; 3];
        for q in p {
            s = util::add(s, *q);
        }
        util::scale(s, 1.0 / p.len().max(1) as f64)
    };
    let s_mid = mean(&coarse_src);
    let t_mid = mean(&coarse_tgt_xyz);
    let from_yaw = |deg: f64| {
        let mut t = yaw_matrix(deg);
        let r = rotation_of(&t);
        let rs = [dot(r[0], s_mid), dot(r[1], s_mid), dot(r[2], s_mid)];
        t[3] = t_mid[0] - rs[0];
        t[7] = t_mid[1] - rs[1];
        t[11] = t_mid[2] - rs[2];
        t
    };
    let mut levels_data: Vec<(Vec<V3>, Target)> = vec![(coarse_src.clone(), Target::new(coarse_tgt_xyz))];
    for &size in &sizes[1..] {
        levels_data.push((prep(&src_full, size, prm.max_points / 2), Target::new(prep(&tgt_full, size, prm.max_points))));
    }

    let refine = |t0: &Mat4| -> (Mat4, f64, Metrics) {
        let mut t = *t0;
        for ((s, tg), &size) in levels_data.iter().zip(&sizes) {
            t = icp(s, tg, &t, size * 3.0, 40, 0.85, 1e-4).0;
        }
        let (s, tg) = levels_data.last().unwrap();
        let m = evaluate(s, tg, &t, voxel * 3.0);
        (t, score(&m), m)
    };

    let starts: Vec<(Mat4, f64)> = if let Some(init) = prm.init {
        vec![(init, 0.0)]
    } else if let Some(h) = prm.yaw_hint {
        vec![(from_yaw(h), h)]
    } else {
        progress(RegStage::FloorPlan, 0, 1);
        let plan = footprint_starts(&src_full, &tgt_full, 36, (voxel * 2.5).max(50.0), 3, 8);
        if cancelled() {
            return None;
        }
        progress(RegStage::Orientation, 0, 1);
        let mut candidates: Vec<(Mat4, f64, bool)> = vec![(identity(), 0.0, false)];
        let steps = prm.yaw_steps.max(1);
        for k in 0..steps {
            let deg = 360.0 * k as f64 / steps as f64;
            candidates.push((from_yaw(deg), deg, false));
        }
        for (t, d) in &plan {
            candidates.push((*t, *d, true));
        }
        let (cs, ct) = &levels_data[0];
        let mut screened: Vec<(f64, Mat4, f64, bool)> = candidates
            .iter()
            .map(|(t0, deg, is_plan)| {
                let t1 = icp(cs, ct, t0, sizes[0] * 3.0, 12, 0.7, 1e-4).0;
                let m = evaluate(cs, ct, &t1, sizes[0] * 1.5);
                (score(&m), t1, *deg, *is_plan)
            })
            .collect();
        screened.sort_by(|a, b| b.0.total_cmp(&a.0));
        let top = prm.top_k.max(1).min(screened.len());
        let mut st: Vec<(Mat4, f64)> = screened[..top].iter().map(|x| (x.1, x.2)).collect();
        if !plan.is_empty() && !screened[..top].iter().any(|x| x.3) {
            if let Some(x) = screened.iter().find(|x| x.3) {
                st.push((x.1, x.2));
            }
        }
        st
    };
    if cancelled() {
        return None;
    }

    let mut best: Option<(f64, Mat4, f64, Metrics)> = None;
    for (i, (t0, deg)) in starts.iter().enumerate() {
        progress(RegStage::Refine, i, starts.len());
        let (t, sc, m) = refine(t0);
        if best.as_ref().map_or(true, |b| sc > b.0) {
            best = Some((sc, t, *deg, m));
        }
        if cancelled() {
            return None;
        }
    }
    let (_, mut t, yaw, _) = best?;

    let min_voxel = prm.min_voxel.unwrap_or((voxel / 4.0).max(8.0));
    let mut polish = Vec::new();
    let mut size = voxel / 2.0;
    while size >= min_voxel - 1e-9 {
        polish.push(size);
        size /= 2.0;
    }
    for (j, &size) in polish.iter().enumerate() {
        progress(RegStage::Polish, j, polish.len());
        let s = prep(&src_full, size, prm.fine_points);
        let tg = Target::new(prep(&tgt_full, size, prm.fine_points));
        t = icp(&s, &tg, &t, (size * 4.0).max(voxel * 1.5), 40, 0.9, 1e-4).0;
        if cancelled() {
            return None;
        }
    }
    let (s, tg) = levels_data.last().unwrap();
    let m = evaluate(s, tg, &t, voxel * 3.0);
    result.t = t;
    result.fitness = m.fitness;
    result.rmse = m.rmse;
    result.stability = m.stability;
    result.overlap = m.overlap;
    result.yaw = yaw;
    Some(result)
}

// Thresholds calibrated against measured alignments of a cluttered room.
pub const GOOD_OVERLAP: f64 = 0.60;
pub const POOR_OVERLAP: f64 = 0.35;
pub const GOOD_STABILITY: f64 = 0.10;
pub const POOR_STABILITY: f64 = 0.05;
pub const GOOD_RMSE_VOXELS: f64 = 1.15;
pub const POOR_RMSE_VOXELS: f64 = 1.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Verdict {
    Good,
    Check,
    Poor,
}

/// Reasons behind a verdict, as codes the UI localises.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub enum Reason {
    LowOverlap,
    Featureless,
    HighResidual,
    SomeOverlap,
    NearlyFeatureless,
    ElevatedResidual,
}

pub fn verdict(r: &RegResult) -> (Verdict, Vec<Reason>) {
    let mut bad = Vec::new();
    if r.overlap < POOR_OVERLAP {
        bad.push(Reason::LowOverlap);
    }
    if r.stability < POOR_STABILITY {
        bad.push(Reason::Featureless);
    }
    if !r.rmse.is_finite() || r.rmse > POOR_RMSE_VOXELS * r.voxel {
        bad.push(Reason::HighResidual);
    }
    if !bad.is_empty() {
        return (Verdict::Poor, bad);
    }
    let mut weak = Vec::new();
    if r.overlap < GOOD_OVERLAP {
        weak.push(Reason::SomeOverlap);
    }
    if r.stability < GOOD_STABILITY {
        weak.push(Reason::NearlyFeatureless);
    }
    if r.rmse > GOOD_RMSE_VOXELS * r.voxel {
        weak.push(Reason::ElevatedResidual);
    }
    if !weak.is_empty() {
        return (Verdict::Check, weak);
    }
    (Verdict::Good, vec![])
}
