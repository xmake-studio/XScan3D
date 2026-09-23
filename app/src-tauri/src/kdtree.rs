//! Static 3-d tree with an implicit, pointer-free layout.
//!
//! Node `i` has children `2i+1` and `2i+2`; a node covering points `[lo, hi)`
//! splits at `mid = lo + (hi-lo)/2`, so ranges never need storing. Built in
//! parallel (disjoint halves), queried from many threads at once.

use crate::util::{SyncPtr, V3};

const LEAF: usize = 12;

pub struct KdTree {
    pts: Vec<V3>,
    ids: Vec<u32>,
    dims: Vec<u8>,
    vals: Vec<f64>,
    levels: u32,
}

impl KdTree {
    pub fn new(points: &[V3]) -> KdTree {
        let n = points.len();
        let mut levels = 0u32;
        while (n >> levels) > LEAF {
            levels += 1;
        }
        let internal = (1usize << levels) - 1;
        let mut items: Vec<(V3, u32)> = points.iter().enumerate().map(|(i, &p)| (p, i as u32)).collect();
        let mut dims = vec![0u8; internal];
        let mut vals = vec![0f64; internal];
        if n > 0 && levels > 0 {
            let dp = SyncPtr(dims.as_mut_ptr());
            let vp = SyncPtr(vals.as_mut_ptr());
            build(&mut items, 0, 0, levels, dp, vp);
        }
        let pts = items.iter().map(|x| x.0).collect();
        let ids = items.iter().map(|x| x.1).collect();
        KdTree { pts, ids, dims, vals, levels }
    }

    pub fn len(&self) -> usize {
        self.pts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }

    /// Point by original index is not stored; this is the reordered array.
    pub fn point_at_slot(&self, slot: usize) -> V3 {
        self.pts[slot]
    }

    /// Nearest point with squared distance below `max_d2`: (original index, d2).
    pub fn nearest(&self, q: V3, max_d2: f64) -> Option<(usize, f64)> {
        if self.pts.is_empty() {
            return None;
        }
        let mut best = (max_d2, usize::MAX);
        self.nearest_rec(0, 0, self.pts.len(), 0, q, &mut best);
        if best.1 == usize::MAX {
            None
        } else {
            Some((self.ids[best.1] as usize, best.0))
        }
    }

    fn nearest_rec(&self, node: usize, lo: usize, hi: usize, depth: u32, q: V3, best: &mut (f64, usize)) {
        if depth == self.levels {
            for i in lo..hi {
                let p = &self.pts[i];
                let dx = p[0] - q[0];
                let dy = p[1] - q[1];
                let dz = p[2] - q[2];
                let d2 = dx * dx + dy * dy + dz * dz;
                if d2 < best.0 {
                    *best = (d2, i);
                }
            }
            return;
        }
        let mid = lo + (hi - lo) / 2;
        let diff = q[self.dims[node] as usize] - self.vals[node];
        let (l, r) = (2 * node + 1, 2 * node + 2);
        if diff < 0.0 {
            self.nearest_rec(l, lo, mid, depth + 1, q, best);
            if diff * diff < best.0 {
                self.nearest_rec(r, mid, hi, depth + 1, q, best);
            }
        } else {
            self.nearest_rec(r, mid, hi, depth + 1, q, best);
            if diff * diff < best.0 {
                self.nearest_rec(l, lo, mid, depth + 1, q, best);
            }
        }
    }

    /// The `k` nearest points, closest first, as (original index, d2).
    pub fn knn(&self, q: V3, k: usize, out: &mut Vec<(usize, f64)>) {
        out.clear();
        if self.pts.is_empty() || k == 0 {
            return;
        }
        let mut heap = Knn { d2: Vec::with_capacity(k + 1), slot: Vec::with_capacity(k + 1), k };
        self.knn_rec(0, 0, self.pts.len(), 0, q, &mut heap);
        for (d2, s) in heap.d2.iter().zip(&heap.slot) {
            out.push((self.ids[*s] as usize, *d2));
        }
    }

    fn knn_rec(&self, node: usize, lo: usize, hi: usize, depth: u32, q: V3, h: &mut Knn) {
        if depth == self.levels {
            for i in lo..hi {
                let p = &self.pts[i];
                let dx = p[0] - q[0];
                let dy = p[1] - q[1];
                let dz = p[2] - q[2];
                h.offer(dx * dx + dy * dy + dz * dz, i);
            }
            return;
        }
        let mid = lo + (hi - lo) / 2;
        let diff = q[self.dims[node] as usize] - self.vals[node];
        let (l, r) = (2 * node + 1, 2 * node + 2);
        let (first, flo, fhi, second, slo, shi) =
            if diff < 0.0 { (l, lo, mid, r, mid, hi) } else { (r, mid, hi, l, lo, mid) };
        self.knn_rec(first, flo, fhi, depth + 1, q, h);
        if diff * diff < h.worst() {
            self.knn_rec(second, slo, shi, depth + 1, q, h);
        }
    }

    /// Every point within sqrt(`r2`), as original indices.
    pub fn within(&self, q: V3, r2: f64, out: &mut Vec<usize>) {
        out.clear();
        if self.pts.is_empty() {
            return;
        }
        self.within_rec(0, 0, self.pts.len(), 0, q, r2, out);
    }

    fn within_rec(&self, node: usize, lo: usize, hi: usize, depth: u32, q: V3, r2: f64, out: &mut Vec<usize>) {
        if depth == self.levels {
            for i in lo..hi {
                let p = &self.pts[i];
                let dx = p[0] - q[0];
                let dy = p[1] - q[1];
                let dz = p[2] - q[2];
                if dx * dx + dy * dy + dz * dz <= r2 {
                    out.push(self.ids[i] as usize);
                }
            }
            return;
        }
        let mid = lo + (hi - lo) / 2;
        let diff = q[self.dims[node] as usize] - self.vals[node];
        let (l, r) = (2 * node + 1, 2 * node + 2);
        if diff < 0.0 {
            self.within_rec(l, lo, mid, depth + 1, q, r2, out);
            if diff * diff <= r2 {
                self.within_rec(r, mid, hi, depth + 1, q, r2, out);
            }
        } else {
            self.within_rec(r, mid, hi, depth + 1, q, r2, out);
            if diff * diff <= r2 {
                self.within_rec(l, lo, mid, depth + 1, q, r2, out);
            }
        }
    }
}

struct Knn {
    d2: Vec<f64>,
    slot: Vec<usize>,
    k: usize,
}

impl Knn {
    #[inline]
    fn worst(&self) -> f64 {
        if self.d2.len() < self.k {
            f64::INFINITY
        } else {
            *self.d2.last().unwrap()
        }
    }
    #[inline]
    fn offer(&mut self, d2: f64, slot: usize) {
        if self.d2.len() == self.k && d2 >= *self.d2.last().unwrap() {
            return;
        }
        // Sorted insert; k is small.
        let mut pos = self.d2.len();
        while pos > 0 && self.d2[pos - 1] > d2 {
            pos -= 1;
        }
        self.d2.insert(pos, d2);
        self.slot.insert(pos, slot);
        if self.d2.len() > self.k {
            self.d2.pop();
            self.slot.pop();
        }
    }
}

fn build(items: &mut [(V3, u32)], node: usize, depth: u32, levels: u32, dims: SyncPtr<u8>, vals: SyncPtr<f64>) {
    if depth == levels || items.len() < 2 {
        return;
    }
    // Split on the axis of largest spread.
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for (p, _) in items.iter() {
        for d in 0..3 {
            lo[d] = lo[d].min(p[d]);
            hi[d] = hi[d].max(p[d]);
        }
    }
    let mut dim = 0;
    for d in 1..3 {
        if hi[d] - lo[d] > hi[dim] - lo[dim] {
            dim = d;
        }
    }
    let mid = items.len() / 2;
    items.select_nth_unstable_by(mid, |a, b| a.0[dim].total_cmp(&b.0[dim]));
    unsafe {
        *dims.0.add(node) = dim as u8;
        *vals.0.add(node) = items[mid].0[dim];
    }
    let big = items.len() > 50_000;
    let (l, r) = items.split_at_mut(mid);
    if big {
        rayon::join(
            || build(l, 2 * node + 1, depth + 1, levels, dims, vals),
            || build(r, 2 * node + 2, depth + 1, levels, dims, vals),
        );
    } else {
        build(l, 2 * node + 1, depth + 1, levels, dims, vals);
        build(r, 2 * node + 2, depth + 1, levels, dims, vals);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::util::{dist2, Rng};

    #[test]
    fn matches_brute_force() {
        let mut rng = Rng::new(7);
        let pts: Vec<V3> = (0..5000)
            .map(|_| {
                [
                    (rng.next_u64() % 10000) as f64 * 0.1,
                    (rng.next_u64() % 10000) as f64 * 0.1,
                    (rng.next_u64() % 10000) as f64 * 0.1,
                ]
            })
            .collect();
        let t = KdTree::new(&pts);
        let mut out = Vec::new();
        let mut wout = Vec::new();
        for qi in 0..200 {
            let q = [qi as f64 * 4.3, 500.0 - qi as f64, qi as f64 * 2.1];
            let bf = (0..pts.len())
                .min_by(|&a, &b| dist2(pts[a], q).total_cmp(&dist2(pts[b], q)))
                .unwrap();
            let (i, d2) = t.nearest(q, f64::INFINITY).unwrap();
            assert!((d2 - dist2(pts[bf], q)).abs() < 1e-9 && (i == bf || d2 == dist2(pts[i], q)));
            t.knn(q, 10, &mut out);
            let mut all: Vec<f64> = pts.iter().map(|&p| dist2(p, q)).collect();
            all.sort_by(|a, b| a.total_cmp(b));
            for k in 0..10 {
                assert!((out[k].1 - all[k]).abs() < 1e-9);
            }
            t.within(q, 900.0, &mut wout);
            assert_eq!(wout.len(), all.iter().filter(|&&d| d <= 900.0).count());
        }
    }
}
