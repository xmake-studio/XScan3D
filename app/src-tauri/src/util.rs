//! Small numeric helpers shared by the geometry, registration, calibration and
//! meshing modules.

use rayon::prelude::*;

pub type V3 = [f64; 3];

#[inline]
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
#[inline]
pub fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
#[inline]
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline]
pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
#[inline]
pub fn dist2(a: V3, b: V3) -> f64 {
    let d = sub(a, b);
    dot(d, d)
}
#[inline]
pub fn normalize(a: V3) -> V3 {
    let n = norm(a);
    if n > 1e-300 {
        scale(a, 1.0 / n)
    } else {
        [0.0, 0.0, 1.0]
    }
}

pub fn to_f64(p: &[[f32; 3]]) -> Vec<V3> {
    p.par_iter().map(|q| [q[0] as f64, q[1] as f64, q[2] as f64]).collect()
}

pub fn to_f32(p: &[V3]) -> Vec<[f32; 3]> {
    p.par_iter().map(|q| [q[0] as f32, q[1] as f32, q[2] as f32]).collect()
}

/// A 4x4 row-major rigid transform.
pub type Mat4 = [f64; 16];

pub fn identity4() -> Mat4 {
    let mut m = [0.0; 16];
    m[0] = 1.0;
    m[5] = 1.0;
    m[10] = 1.0;
    m[15] = 1.0;
    m
}

#[inline]
pub fn xform(m: &Mat4, p: V3) -> V3 {
    [
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
    ]
}

pub fn mat4_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut r = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[i * 4 + k] * b[k * 4 + j];
            }
            r[i * 4 + j] = s;
        }
    }
    r
}

pub fn transform_points(m: &Mat4, p: &[V3]) -> Vec<V3> {
    p.par_iter().map(|&q| xform(m, q)).collect()
}

/// Eigen-decomposition of a symmetric 3x3 matrix by cyclic Jacobi.
/// Returns eigenvalues ascending and the matching unit eigenvectors.
pub fn sym3_eigen(a: [[f64; 3]; 3]) -> ([f64; 3], [V3; 3]) {
    let mut m = a;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _sweep in 0..24 {
        let off = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
        let diag = m[0][0] * m[0][0] + m[1][1] * m[1][1] + m[2][2] * m[2][2];
        if off <= 1e-30 * diag.max(1e-300) || off == 0.0 {
            break;
        }
        for &(p, q) in &[(0usize, 1usize), (0, 2), (1, 2)] {
            let apq = m[p][q];
            if apq.abs() < 1e-300 {
                continue;
            }
            let theta = (m[q][q] - m[p][p]) / (2.0 * apq);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            // Rotate rows/cols p and q.
            for k in 0..3 {
                let mkp = m[k][p];
                let mkq = m[k][q];
                m[k][p] = c * mkp - s * mkq;
                m[k][q] = s * mkp + c * mkq;
            }
            for k in 0..3 {
                let mpk = m[p][k];
                let mqk = m[q][k];
                m[p][k] = c * mpk - s * mqk;
                m[q][k] = s * mpk + c * mqk;
            }
            for k in 0..3 {
                let vkp = v[k][p];
                let vkq = v[k][q];
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    let ev = [m[0][0], m[1][1], m[2][2]];
    idx.sort_by(|&a, &b| ev[a].partial_cmp(&ev[b]).unwrap_or(std::cmp::Ordering::Equal));
    let vals = [ev[idx[0]], ev[idx[1]], ev[idx[2]]];
    let vecs = [
        [v[0][idx[0]], v[1][idx[0]], v[2][idx[0]]],
        [v[0][idx[1]], v[1][idx[1]], v[2][idx[1]]],
        [v[0][idx[2]], v[1][idx[2]], v[2][idx[2]]],
    ];
    (vals, vecs)
}

/// Covariance accumulator for a set of points.
#[derive(Clone, Copy, Default)]
pub struct Moments {
    pub n: f64,
    pub s: V3,
    pub ss: [f64; 6], // xx xy xz yy yz zz
}

impl Moments {
    #[inline]
    pub fn add(&mut self, p: V3) {
        self.n += 1.0;
        self.s = add(self.s, p);
        self.ss[0] += p[0] * p[0];
        self.ss[1] += p[0] * p[1];
        self.ss[2] += p[0] * p[2];
        self.ss[3] += p[1] * p[1];
        self.ss[4] += p[1] * p[2];
        self.ss[5] += p[2] * p[2];
    }
    pub fn merge(&mut self, o: &Moments) {
        self.n += o.n;
        self.s = add(self.s, o.s);
        for i in 0..6 {
            self.ss[i] += o.ss[i];
        }
    }
    pub fn mean(&self) -> V3 {
        scale(self.s, 1.0 / self.n.max(1.0))
    }
    /// Population covariance.
    pub fn cov(&self) -> [[f64; 3]; 3] {
        let n = self.n.max(1.0);
        let c = self.mean();
        let xx = self.ss[0] / n - c[0] * c[0];
        let xy = self.ss[1] / n - c[0] * c[1];
        let xz = self.ss[2] / n - c[0] * c[2];
        let yy = self.ss[3] / n - c[1] * c[1];
        let yz = self.ss[4] / n - c[1] * c[2];
        let zz = self.ss[5] / n - c[2] * c[2];
        [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
    }
}

/// Covariance of points about their own mean, computed in two passes for
/// accuracy (a single pass loses digits on points far from the origin).
pub fn centered_cov(pts: impl Iterator<Item = V3> + Clone) -> (V3, [[f64; 3]; 3], usize) {
    let mut n = 0usize;
    let mut s = [0.0; 3];
    for p in pts.clone() {
        s = add(s, p);
        n += 1;
    }
    if n == 0 {
        return ([0.0; 3], [[0.0; 3]; 3], 0);
    }
    let c = scale(s, 1.0 / n as f64);
    let mut m = [[0.0; 3]; 3];
    for p in pts {
        let d = sub(p, c);
        for i in 0..3 {
            for j in i..3 {
                m[i][j] += d[i] * d[j];
            }
        }
    }
    for i in 0..3 {
        for j in i..3 {
            m[i][j] /= n as f64;
            m[j][i] = m[i][j];
        }
    }
    (c, m, n)
}

/// Grid cell key for `p` on a `size` grid shifted by `shift`, packed into 63
/// bits (21 per axis, about +-1e6 cells).
#[inline]
pub fn cell_key(p: V3, size: f64, shift: f64) -> u64 {
    let f = |x: f64| -> u64 {
        let c = ((x + shift) / size).floor() as i64 + (1 << 20);
        (c.clamp(0, (1 << 21) - 1)) as u64
    };
    (f(p[0]) << 42) | (f(p[1]) << 21) | f(p[2])
}

#[inline]
pub fn key_cell(k: u64) -> [i64; 3] {
    let m = (1u64 << 21) - 1;
    [
        ((k >> 42) & m) as i64 - (1 << 20),
        ((k >> 21) & m) as i64 - (1 << 20),
        (k & m) as i64 - (1 << 20),
    ]
}

/// Groups points by grid cell. Returns (cell of each point, number of cells,
/// the sorted distinct keys). Cells are numbered in key order.
pub fn group_cells(p: &[V3], size: f64, shift: f64) -> (Vec<u32>, usize, Vec<u64>) {
    let mut keyed: Vec<(u64, u32)> = p
        .par_iter()
        .enumerate()
        .map(|(i, &q)| (cell_key(q, size, shift), i as u32))
        .collect();
    keyed.par_sort_unstable();
    let mut cell = vec![0u32; p.len()];
    let mut keys = Vec::new();
    let mut last = u64::MAX;
    for &(k, i) in &keyed {
        if k != last {
            keys.push(k);
            last = k;
        }
        cell[i as usize] = (keys.len() - 1) as u32;
    }
    (cell, keys.len(), keys)
}

/// One point per `size` cube at the centroid of the points in it.
pub fn voxel_centroids(p: &[V3], size: f64) -> Vec<V3> {
    if size <= 0.0 || p.is_empty() {
        return p.to_vec();
    }
    let (cell, n, _) = group_cells(p, size, 0.0);
    let mut sum = vec![[0.0f64; 4]; n];
    for (q, &c) in p.iter().zip(&cell) {
        let s = &mut sum[c as usize];
        s[0] += q[0];
        s[1] += q[1];
        s[2] += q[2];
        s[3] += 1.0;
    }
    sum.into_iter().map(|s| [s[0] / s[3], s[1] / s[3], s[2] / s[3]]).collect()
}

/// numpy-style linear-interpolated quantile, q in [0, 1]. Reorders `v`.
pub fn quantile_mut(v: &mut [f64], q: f64) -> f64 {
    let n = v.len();
    if n == 0 {
        return f64::NAN;
    }
    let pos = q.clamp(0.0, 1.0) * (n - 1) as f64;
    let lo = pos.floor() as usize;
    let frac = pos - lo as f64;
    let (_, &mut a, rest) = v.select_nth_unstable_by(lo, |x, y| x.total_cmp(y));
    if frac == 0.0 || rest.is_empty() {
        return a;
    }
    let b = rest.iter().copied().fold(f64::INFINITY, f64::min);
    a + frac * (b - a)
}

pub fn median_mut(v: &mut [f64]) -> f64 {
    quantile_mut(v, 0.5)
}

/// Raw pointer that may cross threads, for writes to disjoint slots.
#[derive(Clone, Copy)]
pub struct SyncPtr<T>(pub *mut T);
unsafe impl<T> Send for SyncPtr<T> {}
unsafe impl<T> Sync for SyncPtr<T> {}

/// Deterministic PRNG (splitmix64), so fits are repeatable.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed ^ 0x9E37_79B9_7F4A_7C15)
    }
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n.max(1) as u64) as usize
    }
    /// `k` distinct indices from 0..n.
    pub fn choose_distinct(&mut self, n: usize, k: usize) -> Vec<usize> {
        let mut out: Vec<usize> = Vec::with_capacity(k);
        while out.len() < k.min(n) {
            let c = self.below(n);
            if !out.contains(&c) {
                out.push(c);
            }
        }
        out
    }
}

/// Solves the n x n system `a x = b` (row-major) by Gaussian elimination with
/// partial pivoting. None when singular.
pub fn solve_dense(a: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut m = a.to_vec();
    let mut x = b.to_vec();
    for col in 0..n {
        let mut piv = col;
        let mut best = m[col * n + col].abs();
        for r in col + 1..n {
            let v = m[r * n + col].abs();
            if v > best {
                best = v;
                piv = r;
            }
        }
        if best < 1e-300 || !best.is_finite() {
            return None;
        }
        if piv != col {
            for c in 0..n {
                m.swap(col * n + c, piv * n + c);
            }
            x.swap(col, piv);
        }
        let d = m[col * n + col];
        for r in col + 1..n {
            let f = m[r * n + col] / d;
            if f == 0.0 {
                continue;
            }
            for c in col..n {
                m[r * n + c] -= f * m[col * n + c];
            }
            x[r] -= f * x[col];
        }
    }
    for col in (0..n).rev() {
        let mut s = x[col];
        for c in col + 1..n {
            s -= m[col * n + c] * x[c];
        }
        x[col] = s / m[col * n + col];
    }
    if x.iter().all(|v| v.is_finite()) {
        Some(x)
    } else {
        None
    }
}

/// Least squares from accumulated normal equations (AtA x = Atb), with a
/// whisper of ridge so a rank-deficient system still returns something sane.
pub fn solve_normal(ata: &[f64], atb: &[f64], n: usize) -> Vec<f64> {
    let mut a = ata.to_vec();
    let tr: f64 = (0..n).map(|i| a[i * n + i]).sum::<f64>().max(1e-30);
    for i in 0..n {
        a[i * n + i] += 1e-12 * tr / n as f64;
    }
    solve_dense(&a, atb, n).unwrap_or_else(|| vec![0.0; n])
}

/// Small helper for timing log lines.
pub struct Stopwatch(std::time::Instant);
impl Stopwatch {
    pub fn start() -> Self {
        Stopwatch(std::time::Instant::now())
    }
    pub fn secs(&self) -> f64 {
        self.0.elapsed().as_secs_f64()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eigen_matches_known() {
        let a = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 1.0]];
        let (w, v) = sym3_eigen(a);
        assert!(w[0] <= w[1] && w[1] <= w[2]);
        for k in 0..3 {
            let av = [
                dot(a[0], v[k]),
                dot(a[1], v[k]),
                dot(a[2], v[k]),
            ];
            for i in 0..3 {
                assert!((av[i] - w[k] * v[k][i]).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn quantile_linear() {
        let mut v = vec![1.0, 2.0, 3.0, 4.0];
        assert!((quantile_mut(&mut v, 0.5) - 2.5).abs() < 1e-12);
        let mut v = vec![5.0, 1.0, 3.0];
        assert!((quantile_mut(&mut v, 1.0) - 5.0).abs() < 1e-12);
    }
}
