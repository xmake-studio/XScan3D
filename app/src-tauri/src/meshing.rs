//! Point cloud -> triangle mesh, with screened Poisson reconstruction.
//!
//! The pipeline: thin to a point budget on a uniform grid (a lidar sweep is
//! wildly non-uniform), estimate normals by local PCA and orient each one
//! towards the scanner that saw the point -- orientation is the part of
//! Poisson that usually goes wrong, and here it is known exactly -- solve,
//! then trim away the surface no measurement supports (Poisson closes
//! everything), drop the crumbs that trimming leaves, and optionally smooth.

use rayon::prelude::*;
use serde::Serialize;

use crate::kdtree::KdTree;
use crate::poisson::{self, PrParams};
use crate::util::{self, Stopwatch, V3};

#[derive(Clone, Debug)]
pub struct MeshParams {
    /// Finest surface detail wanted, mm. The octree depth follows from it
    /// and the size of the scene, so a desk and a building get the same
    /// physical resolution.
    pub cell_mm: f64,
    pub samples_per_node: f32,
    pub point_weight: f32,
    /// Drop surface further than this many point spacings from any sample.
    pub trim: f64,
    /// Thin the input to about this many points first (0 = no limit).
    pub budget: usize,
    pub smooth_iters: u32,
    pub normal_k: usize,
}

impl MeshParams {
    /// Quality presets: "medium", "high" (the default), "max".
    pub fn preset(quality: &str, trim: f64, budget: usize, smooth: u32) -> MeshParams {
        let (cell_mm, spn) = match quality {
            "medium" => (10.0, 2.0),
            "max" => (2.5, 1.5),
            _ => (5.0, 1.5),
        };
        MeshParams {
            cell_mm,
            samples_per_node: spn,
            point_weight: 2.0,
            trim,
            budget,
            smooth_iters: smooth,
            normal_k: 24,
        }
    }
}

pub struct Mesh {
    pub verts: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub tris: Vec<[u32; 3]>,
}

#[derive(Clone, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct MeshInfo {
    pub input: usize,
    pub used: usize,
    pub voxel: f64,
    pub spacing: f64,
    pub depth: u32,
    pub cell_mm: f64,
    pub verts: usize,
    pub tris: usize,
    pub seconds: f64,
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum MeshStage {
    Thinning,
    Normals,
    Solving,
    Cleaning,
}

/// Centroid per cell of a `size` grid; each output point keeps the scanner
/// index of the first point in its cell.
fn thin(p: &[V3], scan: &[u16], size: f64) -> (Vec<V3>, Vec<u16>) {
    let (cell, n, _) = util::group_cells(p, size, 0.0);
    let mut sum = vec![[0.0f64; 4]; n];
    let mut sc = vec![u16::MAX; n];
    for (i, q) in p.iter().enumerate() {
        let c = cell[i] as usize;
        let s = &mut sum[c];
        s[0] += q[0];
        s[1] += q[1];
        s[2] += q[2];
        s[3] += 1.0;
        if sc[c] == u16::MAX {
            sc[c] = scan[i];
        }
    }
    (sum.into_iter().map(|s| [s[0] / s[3], s[1] / s[3], s[2] / s[3]]).collect(), sc)
}

fn extent(p: &[V3]) -> f64 {
    let mut mn = [f64::INFINITY; 3];
    let mut mx = [f64::NEG_INFINITY; 3];
    for q in p {
        for d in 0..3 {
            mn[d] = mn[d].min(q[d]);
            mx[d] = mx[d].max(q[d]);
        }
    }
    (mx[0] - mn[0]).max(mx[1] - mn[1]).max(mx[2] - mn[2]).max(1.0)
}

/// Thins the cloud for a solve at `depth`.
///
/// A lidar sweep is wildly anisotropic: consecutive slices of a slow sweep sit
/// a millimetre or two apart while points along a slice are centimetres
/// apart, so the raw cloud is mostly redundancy the octree merges anyway. It
/// is thinned to a grid just finer than the finest octree cell -- nothing
/// below that can show in the surface -- which also makes the spacing, and so
/// the trim distance, mean what it says. The point budget caps what is left.
fn downsample_for_depth(p: &[V3], scan: &[u16], depth: u32, budget: usize) -> (Vec<V3>, Vec<u16>, f64) {
    let finest = 1.1 * extent(p) / (1u64 << depth) as f64;
    let mut size = 0.75 * finest;
    let (mut out, mut sc) = thin(p, scan, size);
    if budget > 0 && out.len() > budget {
        // Grow the cell until the count fits: count falls roughly with the
        // square of the cell on surfaces.
        for _ in 0..12 {
            size *= ((out.len() as f64 / budget as f64).sqrt() * 1.02).max(1.02);
            let t = thin(p, scan, size);
            out = t.0;
            sc = t.1;
            if out.len() <= budget {
                break;
            }
        }
    }
    (out, sc, size)
}

/// Median distance to the nearest neighbour, from a subsample.
fn spacing(p: &[V3], tree: &KdTree) -> f64 {
    if p.len() < 2 {
        return 1.0;
    }
    let step = (p.len() / 100_000).max(1);
    let mut d: Vec<f64> = (0..p.len())
        .into_par_iter()
        .step_by(step)
        .map_init(Vec::new, |buf, i| {
            tree.knn(p[i], 2, buf);
            buf.get(1).map(|x| x.1.sqrt()).unwrap_or(0.0)
        })
        .collect();
    util::median_mut(&mut d).max(1e-6)
}

/// PCA normals, flipped to face the scanner that measured each point.
fn oriented_normals(p: &[V3], scan: &[u16], origins: &[V3], tree: &KdTree, k: usize) -> Vec<V3> {
    let k = k.min(p.len());
    p.par_iter()
        .enumerate()
        .map_init(Vec::new, |buf, (i, &q)| {
            tree.knn(q, k, buf);
            let (_, cov, n) = util::centered_cov(buf.iter().map(|&(j, _)| p[j]));
            let mut nn = if n >= 3 { util::sym3_eigen(cov).1[0] } else { [0.0, 0.0, 1.0] };
            let o = origins.get(scan[i] as usize).copied().unwrap_or([0.0; 3]);
            if util::dot(nn, util::sub(o, q)) < 0.0 {
                nn = util::scale(nn, -1.0);
            }
            nn
        })
        .collect()
}

/// Keeps triangles whose three corners are within `max_dist` of a sample, then
/// compacts the vertex array.
fn trim_to_samples(verts: Vec<[f32; 3]>, tris: Vec<[u32; 3]>, tree: &KdTree, max_dist: f64) -> (Vec<[f32; 3]>, Vec<[u32; 3]>) {
    if max_dist <= 0.0 || tris.is_empty() {
        return (verts, tris);
    }
    let md2 = max_dist * max_dist;
    let keep: Vec<bool> = verts
        .par_iter()
        .map(|v| tree.nearest([v[0] as f64, v[1] as f64, v[2] as f64], md2).is_some())
        .collect();
    let tris: Vec<[u32; 3]> = tris.into_par_iter().filter(|t| keep[t[0] as usize] && keep[t[1] as usize] && keep[t[2] as usize]).collect();
    compact(verts, tris)
}

fn compact(verts: Vec<[f32; 3]>, tris: Vec<[u32; 3]>) -> (Vec<[f32; 3]>, Vec<[u32; 3]>) {
    let mut map = vec![u32::MAX; verts.len()];
    let mut out = Vec::new();
    let mut nt = Vec::with_capacity(tris.len());
    for t in tris {
        let mut r = [0u32; 3];
        for k in 0..3 {
            let v = t[k] as usize;
            if map[v] == u32::MAX {
                map[v] = out.len() as u32;
                out.push(verts[v]);
            }
            r[k] = map[v];
        }
        nt.push(r);
    }
    (out, nt)
}

fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        let p = parent[x as usize];
        parent[x as usize] = parent[p as usize];
        x = p;
    }
    x
}

/// Removes connected pieces that are tiny next to the whole: the crumbs
/// trimming leaves along the fringe of the scanned surface.
fn drop_crumbs(verts: Vec<[f32; 3]>, tris: Vec<[u32; 3]>) -> (Vec<[f32; 3]>, Vec<[u32; 3]>) {
    if tris.is_empty() {
        return (verts, tris);
    }
    let mut parent: Vec<u32> = (0..verts.len() as u32).collect();
    for t in &tris {
        let a = find(&mut parent, t[0]);
        for k in 1..3 {
            let b = find(&mut parent, t[k]);
            if a != b {
                parent[b as usize] = a;
            }
        }
    }
    let mut faces = std::collections::HashMap::new();
    let roots: Vec<u32> = tris.iter().map(|t| find(&mut parent, t[0])).collect();
    for &r in &roots {
        *faces.entry(r).or_insert(0usize) += 1;
    }
    let min_faces = (tris.len() / 2000).min(500).max(24);
    let tris: Vec<[u32; 3]> = tris.into_iter().zip(roots).filter(|(_, r)| faces[r] >= min_faces).map(|(t, _)| t).collect();
    compact(verts, tris)
}

fn laplacian_smooth(verts: &mut [[f32; 3]], tris: &[[u32; 3]], iters: u32) {
    if iters == 0 || tris.is_empty() {
        return;
    }
    let n = verts.len();
    let mut deg = vec![0f32; n];
    for t in tris {
        for k in 0..3 {
            deg[t[k] as usize] += 2.0;
        }
    }
    for _ in 0..iters {
        let mut acc = vec![[0f32; 3]; n];
        for t in tris {
            for k in 0..3 {
                let a = t[k] as usize;
                for m in [(k + 1) % 3, (k + 2) % 3] {
                    let b = t[m] as usize;
                    for d in 0..3 {
                        acc[a][d] += verts[b][d];
                    }
                }
            }
        }
        for i in 0..n {
            if deg[i] > 0.0 {
                for d in 0..3 {
                    verts[i][d] += 0.5 * (acc[i][d] / deg[i] - verts[i][d]);
                }
            }
        }
    }
}

/// Area-weighted vertex normals.
pub fn vertex_normals(verts: &[[f32; 3]], tris: &[[u32; 3]]) -> Vec<[f32; 3]> {
    let mut acc = vec![[0f64; 3]; verts.len()];
    for t in tris {
        let a = verts[t[0] as usize];
        let b = verts[t[1] as usize];
        let c = verts[t[2] as usize];
        let u = [(b[0] - a[0]) as f64, (b[1] - a[1]) as f64, (b[2] - a[2]) as f64];
        let v = [(c[0] - a[0]) as f64, (c[1] - a[1]) as f64, (c[2] - a[2]) as f64];
        let n = util::cross(u, v);
        for k in 0..3 {
            let s = &mut acc[t[k] as usize];
            s[0] += n[0];
            s[1] += n[1];
            s[2] += n[2];
        }
    }
    acc.into_par_iter()
        .map(|n| {
            let l = util::norm(n);
            if l > 0.0 {
                [(n[0] / l) as f32, (n[1] / l) as f32, (n[2] / l) as f32]
            } else {
                [0.0, 0.0, 1.0]
            }
        })
        .collect()
}

/// Meshes `points` (world frame). `scan` gives each point's scanner as an
/// index into `origins` (the scanner positions), for normal orientation.
pub fn reconstruct(
    points: &[V3],
    scan: &[u16],
    origins: &[V3],
    prm: &MeshParams,
    progress: &(dyn Fn(MeshStage) + Sync),
    cancelled: &(dyn Fn() -> bool + Sync),
) -> Result<(Mesh, MeshInfo), String> {
    let sw = Stopwatch::start();
    let mut info = MeshInfo { input: points.len(), ..Default::default() };
    if points.len() < 100 {
        return Err("too few points".into());
    }
    progress(MeshStage::Thinning);
    let ext = extent(points);
    let depth = ((1.1 * ext / prm.cell_mm.max(0.5)).log2().round() as i64).clamp(7, 13) as u32;
    info.depth = depth;
    info.cell_mm = 1.1 * ext / (1u64 << depth) as f64;
    let (p, sc, voxel) = downsample_for_depth(points, scan, depth, prm.budget);
    info.used = p.len();
    info.voxel = voxel;
    if cancelled() {
        return Err("cancelled".into());
    }
    progress(MeshStage::Normals);
    let tree = KdTree::new(&p);
    let s = spacing(&p, &tree);
    info.spacing = s;
    let normals = oriented_normals(&p, &sc, origins, &tree, prm.normal_k);
    if cancelled() {
        return Err("cancelled".into());
    }
    progress(MeshStage::Solving);
    let pf: Vec<[f32; 3]> = util::to_f32(&p);
    let nf: Vec<[f32; 3]> = util::to_f32(&normals);
    let threads = std::env::var("XSCAN_PR_THREADS")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .map(|t| t + 1)
        .unwrap_or_else(|| std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4));
    let pr = PrParams {
        depth: depth as i32,
        full_depth: 5,
        samples_per_node: prm.samples_per_node,
        point_weight: prm.point_weight,
        scale: 1.1,
        iters: 8,
        threads: threads.saturating_sub(1).max(1) as i32,
        linear_fit: 0,
    };
    let raw = poisson::reconstruct(&pf, &nf, &pr)?;
    drop(pf);
    drop(nf);
    if cancelled() {
        return Err("cancelled".into());
    }
    progress(MeshStage::Cleaning);
    let (verts, tris) = trim_to_samples(raw.verts, raw.tris, &tree, s * prm.trim);
    let (mut verts, tris) = drop_crumbs(verts, tris);
    laplacian_smooth(&mut verts, &tris, prm.smooth_iters);
    let normals = vertex_normals(&verts, &tris);
    info.verts = verts.len();
    info.tris = tris.len();
    info.seconds = sw.secs();
    if tris.is_empty() {
        return Err("the reconstruction produced no surface".into());
    }
    Ok((Mesh { verts, normals, tris }, info))
}
