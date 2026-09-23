//! Flat-patch regression, shared by the self-calibrations.
//!
//! Every flat patch of a room is a ruler: a systematic error (microstep,
//! range ripple) moves its points off the patch's plane in a way a small model
//! can explain. The recipe, from scan_proto.py:
//!   1. cut the cloud into cubes and keep the flat, two-dimensional ones;
//!   2. per point: its offset r from the patch plane, the patch normal and
//!      in-plane coordinates;
//!   3. regress r on the model's columns after projecting a free plane
//!      a + b u + c v per patch out of both sides (Frisch-Waugh), so the
//!      patches' own planes do not soak up the fit; one robust reweighting.

use rayon::prelude::*;

use crate::util::{self, sym3_eigen, Moments, V3};

/// Points of a cloud that lie in flat patches of one grid.
pub struct Patches {
    /// Index into the cloud.
    pub idx: Vec<u32>,
    /// Offset from the patch plane, mm.
    pub r: Vec<f64>,
    /// Patch normal (sign arbitrary).
    pub n: Vec<V3>,
    /// Dense patch id.
    pub pid: Vec<u32>,
    /// In-plane coordinates scaled to the patch.
    pub uv: Vec<[f64; 2]>,
    pub n_patches: usize,
}

impl Patches {
    pub fn empty() -> Patches {
        Patches { idx: vec![], r: vec![], n: vec![], pid: vec![], uv: vec![], n_patches: 0 }
    }

    pub fn len(&self) -> usize {
        self.idx.len()
    }

    /// Keeps the points where `keep` is true, re-densifying patch ids.
    pub fn filter(&self, keep: &[bool]) -> Patches {
        let mut out = Patches::empty();
        let mut remap = vec![u32::MAX; self.n_patches];
        for i in 0..self.idx.len() {
            if !keep[i] {
                continue;
            }
            let p = self.pid[i] as usize;
            if remap[p] == u32::MAX {
                remap[p] = out.n_patches as u32;
                out.n_patches += 1;
            }
            out.idx.push(self.idx[i]);
            out.r.push(self.r[i]);
            out.n.push(self.n[i]);
            out.pid.push(remap[p]);
            out.uv.push(self.uv[i]);
        }
        out
    }

    /// Appends another grid's patches, keeping ids distinct.
    pub fn append(&mut self, other: Patches) {
        let off = self.n_patches as u32;
        self.idx.extend(other.idx);
        self.r.extend(other.r);
        self.n.extend(other.n);
        self.pid.extend(other.pid.into_iter().map(|p| p + off));
        self.uv.extend(other.uv);
        self.n_patches += other.n_patches;
    }
}

/// Flat patches of a `size` grid shifted by `shift`. Points further than
/// `max_resid` from their plane are left out.
pub fn flat_patches(p: &[V3], size: f64, shift: f64, min_points: f64, flat_mm: f64, max_resid: f64) -> Patches {
    let (cell, m, _) = util::group_cells(p, size, shift);
    let mut mom = vec![Moments::default(); m];
    for (q, &c) in p.iter().zip(&cell) {
        mom[c as usize].add(*q);
    }
    struct Cell {
        c: V3,
        n: V3,
        u: V3,
        v: V3,
        flat: bool,
    }
    let cells: Vec<Cell> = mom
        .par_iter()
        .map(|mo| {
            let c = mo.mean();
            let (w, vec) = sym3_eigen(mo.cov());
            let flat = mo.n >= min_points
                && w[0] < flat_mm * flat_mm
                && w[0] < 0.05 * w[1]
                && w[1] > (size / 6.0) * (size / 6.0);
            Cell { c, n: vec[0], u: vec[1], v: vec[2], flat }
        })
        .collect();
    let mut out = Patches::empty();
    let mut remap = vec![u32::MAX; m];
    for (i, q) in p.iter().enumerate() {
        let ci = cell[i] as usize;
        let ce = &cells[ci];
        if !ce.flat {
            continue;
        }
        let rel = util::sub(*q, ce.c);
        let r = util::dot(rel, ce.n);
        if r.abs() >= max_resid {
            continue;
        }
        if remap[ci] == u32::MAX {
            remap[ci] = out.n_patches as u32;
            out.n_patches += 1;
        }
        out.idx.push(i as u32);
        out.r.push(r);
        out.n.push(ce.n);
        out.pid.push(remap[ci]);
        out.uv.push([util::dot(rel, ce.u) / size, util::dot(rel, ce.v) / size]);
    }
    out
}

/// Points grouped by patch, for parallel per-patch work.
struct ByPatch {
    order: Vec<u32>,
    start: Vec<usize>,
}

fn by_patch(pid: &[u32], n_patches: usize) -> ByPatch {
    let mut count = vec![0usize; n_patches + 1];
    for &p in pid {
        count[p as usize + 1] += 1;
    }
    for i in 0..n_patches {
        count[i + 1] += count[i];
    }
    let start = count.clone();
    let mut fill = count;
    let mut order = vec![0u32; pid.len()];
    for (i, &p) in pid.iter().enumerate() {
        order[fill[p as usize]] = i as u32;
        fill[p as usize] += 1;
    }
    ByPatch { order, start }
}

const MAXC: usize = 32;

/// Regression rows z_i = [y_i, a_i1 .. a_ik] projected per patch.
pub struct Projected<'a> {
    pid: &'a [u32],
    uv: &'a [[f64; 2]],
    groups: ByPatch,
    /// Per patch: 3 x cols plane coefficients, row-major by x-component.
    coef: Vec<[f64; 3 * MAXC]>,
    cols: usize,
}

impl<'a> Projected<'a> {
    /// `rows(i, z)` fills z[0..cols] for point i (y first).
    pub fn new(pid: &'a [u32], uv: &'a [[f64; 2]], n_patches: usize, cols: usize, rows: &(dyn Fn(usize, &mut [f64]) + Sync)) -> Self {
        assert!(cols <= MAXC);
        let groups = by_patch(pid, n_patches);
        let coef: Vec<[f64; 3 * MAXC]> = (0..n_patches)
            .into_par_iter()
            .map(|p| {
                let mut g = [0.0f64; 9];
                let mut b = [0.0f64; 3 * MAXC];
                let mut z = [0.0f64; MAXC];
                for &i in &groups.order[groups.start[p]..groups.start[p + 1]] {
                    let i = i as usize;
                    let x = [1.0, uv[i][0], uv[i][1]];
                    for r in 0..3 {
                        for c in 0..3 {
                            g[r * 3 + c] += x[r] * x[c];
                        }
                    }
                    rows(i, &mut z[..cols]);
                    for r in 0..3 {
                        for j in 0..cols {
                            b[r * MAXC + j] += x[r] * z[j];
                        }
                    }
                }
                for d in 0..3 {
                    g[d * 3 + d] += 1e-9;
                }
                let mut out = [0.0f64; 3 * MAXC];
                for j in 0..cols {
                    let rhs = [b[j], b[MAXC + j], b[2 * MAXC + j]];
                    if let Some(x) = util::solve_dense(&g, &rhs, 3) {
                        out[j] = x[0];
                        out[MAXC + j] = x[1];
                        out[2 * MAXC + j] = x[2];
                    }
                }
                out
            })
            .collect();
        Projected { pid, uv, groups, coef, cols }
    }

    /// Projected row i into z.
    #[inline]
    fn row(&self, i: usize, rows: &(dyn Fn(usize, &mut [f64]) + Sync), z: &mut [f64]) {
        rows(i, z);
        let c = &self.coef[self.pid[i] as usize];
        let x = [1.0, self.uv[i][0], self.uv[i][1]];
        for j in 0..self.cols {
            z[j] -= x[0] * c[j] + x[1] * c[MAXC + j] + x[2] * c[2 * MAXC + j];
        }
    }

    /// Least squares on the projected rows, then one robust reweighting (as
    /// the two-pass lstsq in scan_proto). Returns the k = cols-1 coefficients.
    pub fn solve(&self, rows: &(dyn Fn(usize, &mut [f64]) + Sync), robust: bool) -> Vec<f64> {
        let k = self.cols - 1;
        let pass = |sol: Option<&[f64]>| -> Vec<f64> {
            let (ata, atb) = (0..self.groups.start.len() - 1)
                .into_par_iter()
                .fold(
                    || (vec![0.0f64; k * k], vec![0.0f64; k]),
                    |(mut ata, mut atb), p| {
                        let mut z = [0.0f64; MAXC];
                        for &i in &self.groups.order[self.groups.start[p]..self.groups.start[p + 1]] {
                            self.row(i as usize, rows, &mut z[..self.cols]);
                            let y = z[0];
                            let a = &z[1..self.cols];
                            let w2 = match sol {
                                None => 1.0,
                                Some(s) => {
                                    let pred: f64 = a.iter().zip(s).map(|(x, y)| x * y).sum();
                                    let w = 1.0 / (1.0f64).max((y - pred).abs() / 3.0);
                                    w * w
                                }
                            };
                            for r in 0..k {
                                let ar = a[r] * w2;
                                atb[r] += ar * y;
                                for c in r..k {
                                    ata[r * k + c] += ar * a[c];
                                }
                            }
                        }
                        (ata, atb)
                    },
                )
                .reduce(
                    || (vec![0.0f64; k * k], vec![0.0f64; k]),
                    |(mut a1, mut b1), (a2, b2)| {
                        a1.iter_mut().zip(&a2).for_each(|(x, y)| *x += y);
                        b1.iter_mut().zip(&b2).for_each(|(x, y)| *x += y);
                        (a1, b1)
                    },
                );
            let mut ata = ata;
            for r in 0..k {
                for c in 0..r {
                    ata[r * k + c] = ata[c * k + r];
                }
            }
            util::solve_normal(&ata, &atb, k)
        };
        let sol = pass(None);
        if robust {
            pass(Some(&sol))
        } else {
            sol
        }
    }
}

/// Mean squared residual of the (unweighted) projected least squares, and the
/// mean squared projected y it started from, without materialising the
/// projection: A'MA = A'A - sum_p B_p' G_p^-1 B_p.
pub fn projected_error(pid: &[u32], uv: &[[f64; 2]], n_patches: usize, cols: usize, rows: &(dyn Fn(usize, &mut [f64]) + Sync)) -> (f64, f64) {
    let groups = by_patch(pid, n_patches);
    let s = (0..n_patches)
        .into_par_iter()
        .fold(
            || vec![0.0f64; cols * cols],
            |mut s, p| {
                let mut g = [0.0f64; 9];
                let mut b = [0.0f64; 3 * MAXC];
                let mut z = [0.0f64; MAXC];
                for &i in &groups.order[groups.start[p]..groups.start[p + 1]] {
                    let i = i as usize;
                    let x = [1.0, uv[i][0], uv[i][1]];
                    for r in 0..3 {
                        for c in 0..3 {
                            g[r * 3 + c] += x[r] * x[c];
                        }
                    }
                    rows(i, &mut z[..cols]);
                    for r in 0..cols {
                        for c in r..cols {
                            s[r * cols + c] += z[r] * z[c];
                        }
                        for d in 0..3 {
                            b[d * MAXC + r] += x[d] * z[r];
                        }
                    }
                }
                for d in 0..3 {
                    g[d * 3 + d] += 1e-9;
                }
                // Subtract B' G^-1 B for this patch.
                for c in 0..cols {
                    let rhs = [b[c], b[MAXC + c], b[2 * MAXC + c]];
                    if let Some(gx) = util::solve_dense(&g, &rhs, 3) {
                        for r in 0..=c {
                            let v = b[r] * gx[0] + b[MAXC + r] * gx[1] + b[2 * MAXC + r] * gx[2];
                            s[r * cols + c] -= v;
                        }
                    }
                }
                s
            },
        )
        .reduce(|| vec![0.0f64; cols * cols], |mut a, b| {
            a.iter_mut().zip(&b).for_each(|(x, y)| *x += y);
            a
        });
    let n = pid.len().max(1) as f64;
    let k = cols - 1;
    let mut ata = vec![0.0; k * k];
    let mut aty = vec![0.0; k];
    for r in 0..k {
        aty[r] = s[r + 1]; // s[0][r+1]
        for c in r..k {
            ata[r * k + c] = s[(r + 1) * cols + (c + 1)];
            ata[c * k + r] = ata[r * k + c];
        }
    }
    let yy = s[0];
    let sol = util::solve_normal(&ata, &aty, k);
    let fit: f64 = sol.iter().zip(&aty).map(|(a, b)| a * b).sum();
    ((yy - fit).max(0.0) / n, yy / n)
}
