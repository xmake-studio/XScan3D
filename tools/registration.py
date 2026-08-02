#!/usr/bin/env python3
"""Rigid registration of one scan onto another.

The scanner sees a room from wherever it happens to be standing. Move it and
scan again and you get a second cloud in a second frame; this module finds the
rigid transform that puts the second one back into the first one's frame, so
overlapping scans can be merged into a single map.

The method is point-to-plane ICP, run coarse-to-fine over voxel grids, with a
yaw search in front of it to survive the one degree of freedom an operator
really does change by a lot: which way the rig was pointing. Everything here is
numpy plus scipy's cKDTree -- no Open3D, no PCL.

Units are millimetres throughout, matching scan_proto's clouds.

Nothing in here talks to the device, the UI, or the filesystem. It takes two
arrays of points and returns a 4x4.
"""

import numpy as np
from scipy.spatial import cKDTree


# --- small rigid-transform helpers ------------------------------------------

def identity():
    return np.eye(4, dtype=np.float64)


def apply(T, xyz):
    """Transform an (N,3) cloud by a 4x4. Returns float32, like the clouds."""
    if xyz.size == 0:
        return xyz.astype(np.float32)
    out = np.asarray(xyz, dtype=np.float64) @ T[:3, :3].T + T[:3, 3]
    return out.astype(np.float32)


def _rotvec_to_matrix(w):
    """Rodrigues, written out rather than pulled from scipy.spatial.transform.

    The increments ICP produces are tiny and this is called once per iteration,
    so the import is not worth it -- and the small-angle branch matters: the
    generic formula divides by the angle and goes to NaN as ICP converges,
    which is exactly when it is called most.
    """
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)
    k = w / theta
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return (np.eye(3) + np.sin(theta) * K
            + (1.0 - np.cos(theta)) * (K @ K))


def yaw_matrix(deg, centre=None):
    """Rotation about world Z, optionally about a point instead of the origin.

    Z is up in the scanner's world frame (see flip_upright in scan_proto), so
    this is the turn the operator makes when they pick the rig up and set it
    down facing somewhere else.
    """
    a = np.radians(deg)
    T = identity()
    T[:3, :3] = np.array([[np.cos(a), -np.sin(a), 0.0],
                          [np.sin(a), np.cos(a), 0.0],
                          [0.0, 0.0, 1.0]])
    if centre is not None:
        T[:3, 3] = np.asarray(centre, float) - T[:3, :3] @ np.asarray(centre, float)
    return T


def pose_summary(T):
    """Human-readable '(dx, dy, dz) mm, N deg about axis' for the log."""
    R, t = T[:3, :3], T[:3, 3]
    # Clip because a matrix that has been through a few compositions can carry
    # a trace a hair outside the valid range, and arccos returns NaN there.
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
    return (f"shift ({t[0]:+.0f}, {t[1]:+.0f}, {t[2]:+.0f}) mm, "
            f"rotation {ang:.1f} deg")


# --- preparation -------------------------------------------------------------

def voxel_downsample(xyz, size):
    """One point per `size`-mm cube, keeping the centroid of each.

    Centroids rather than scan_proto's pick-the-first, because ICP is solving
    for a transform at well under one voxel of precision and the quantisation
    noise of an arbitrary representative is the floor on how well it can do.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    if size <= 0 or xyz.shape[0] == 0:
        return xyz
    keys = np.floor(xyz / size).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    # numpy has shipped this as both (N,) and (N,1) across 2.x when axis is
    # given; ravel so the scatter-add below indexes correctly on either.
    inv = np.asarray(inv).ravel()
    n = inv.max() + 1
    sums = np.zeros((n, 3))
    np.add.at(sums, inv, xyz)
    counts = np.bincount(inv, minlength=n).reshape(-1, 1)
    return sums / counts


def estimate_normals(xyz, k=18):
    """Surface normal per point, from the local neighbourhood's PCA.

    Point-to-plane ICP slides along surfaces instead of getting stuck on the
    nearest-point pairing, which is what lets two scans of the same flat wall
    settle at the right offset. That needs a normal on the target.

    Signs are left alone: the point-to-plane residual is (p-q).n, so flipping a
    normal flips both the residual and its Jacobian row and the solution is
    unchanged. Orienting them consistently would be work with no effect.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    n_pts = xyz.shape[0]
    if n_pts < 3:
        return np.zeros((n_pts, 3))
    k = int(min(k, n_pts))
    tree = cKDTree(xyz)
    _, idx = tree.query(xyz, k=k, workers=-1)
    nbr = xyz[idx]                                  # (N, k, 3)
    nbr = nbr - nbr.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nbr, nbr) / max(k - 1, 1)
    # eigh returns ascending eigenvalues, so column 0 is the direction of least
    # spread -- off the surface.
    _, vecs = np.linalg.eigh(cov)
    return vecs[:, :, 0]


def _rows_in(a, b):
    """Boolean mask of which integer rows of `a` also appear in `b`.

    A void view collapses each 3-vector cell key to one comparable scalar, so
    the membership test is a single vectorised np.isin rather than a Python set
    over hundreds of thousands of rows.
    """
    a = np.ascontiguousarray(a)
    b = np.ascontiguousarray(b)
    va = a.view([("", a.dtype)] * a.shape[1]).ravel()
    vb = b.view([("", b.dtype)] * b.shape[1]).ravel()
    return np.isin(va, vb)


def surface_variation(xyz, k=16):
    """Per-point l0/(l0+l1+l2) of the local covariance. 0 = flat, ~1/3 = blob.

    The smallest eigenvalue over the total spread: tiny when the neighbourhood
    is a surface (one thin direction, two wide), large when it is a little
    volume of points. This is the number that tells vegetation from structure.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    n = xyz.shape[0]
    if n < 3:
        return np.zeros(n)
    k = int(min(k, n))
    tree = cKDTree(xyz)
    _, idx = tree.query(xyz, k=k, workers=-1)
    nbr = xyz[idx] - xyz[idx].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nbr, nbr) / max(k - 1, 1)
    ev = np.linalg.eigvalsh(cov)                       # ascending l0<=l1<=l2
    return ev[:, 0] / (ev.sum(axis=1) + 1e-12)


def structure_filter(xyz, voxel=40.0, k=16, max_variation=0.06):
    """Keep points on surfaces; drop volumetric clutter (foliage, grass).

    Outdoors this is the difference between a merge that works and one that
    only looks like it should. Leaves and grass move between scans -- wind, and
    they are not rigid -- so they never truly correspond, yet they are often
    the majority of the points. Worse, a point buried in a bush has an
    isotropic neighbourhood, so its PCA normal is essentially random, and
    point-to-plane ICP turns each one into a meaningless constraint. The
    combination drags the solve and inflates every residual, so a correct
    building alignment reports a poor fit and gets stamped 'check'.

    A point is 'surface' when its neighbourhood's surface variation is small:
    walls, ground, steps, trunks stay; canopy and grass go. The judgement is
    made on a `voxel`-grid downsample -- raw per-point density swamps the
    neighbourhood shape otherwise, and the grid is the scale alignment works at
    anyway. Full-resolution points are then kept or dropped by which cell they
    fell in, so the density the fine polish needs survives the filter.

    Fails safe: if almost nothing reads as structural (an indoor scan already
    all surfaces, or a threshold set too tight), the original cloud is returned
    rather than stripped to nothing.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    if xyz.shape[0] < k or voxel <= 0:
        return xyz
    d = voxel_downsample(xyz, voxel)
    if d.shape[0] < k:
        return xyz
    keep = surface_variation(d, k=k) <= max_variation
    if int(keep.sum()) < 10:
        return xyz
    # The same integer grid maps full-resolution points back to their cell: a
    # cell's centroid floors to that cell, so the keys line up exactly.
    dkeys = np.floor(d[keep] / voxel).astype(np.int64)
    fkeys = np.floor(xyz / voxel).astype(np.int64)
    return xyz[_rows_in(fkeys, dkeys)]


def wall_footprint(xyz, voxel=80.0, max_nz=0.35):
    """The scene's floor plan: points on near-vertical surfaces, seen from above.

    Seen from directly above, an outdoor scan is a solid disc -- the ground fills
    every cell out to the lidar's range, and one disc looks exactly like another.
    Drop everything but the near-vertical surfaces and what is left is thin
    lines: wall faces, step risers, fence posts, trunks. That is the scene's
    actual plan view, and it is what makes two stations recognisably the same
    place from above.

    Fails safe: a scan with no vertical structure at all (an open field) returns
    the downsampled cloud rather than nothing, so the caller still gets a usable
    -- if uninformative -- footprint instead of an empty array.
    """
    d = voxel_downsample(xyz, voxel)
    if d.shape[0] < 20:
        return d
    n = estimate_normals(d, k=18)
    w = d[np.abs(n[:, 2]) < max_nz]
    return w if w.shape[0] >= 20 else d


def _plan_grid(xy, lo, shape, cell):
    """Binary occupancy of `xy` on a `cell`-mm raster starting at `lo`.

    Binary, not a count: point density falls off with range and with how square
    a surface sat to the beam, so counts would let a near wall outvote the whole
    rest of the plan. Presence is the part that is comparable between stations.
    """
    g = np.zeros(shape, dtype=np.float32)
    k = np.floor((xy - lo) / cell).astype(np.int64)
    ok = ((k >= 0) & (k < np.array(shape))).all(axis=1)
    k = k[ok]
    if k.shape[0]:
        g[k[:, 0], k[:, 1]] = 1.0
    return g


def footprint_starts(source, target, yaw_steps=36, cell=100.0, per_yaw=3,
                     top=8, voxel=80.0, max_cells=512):
    """Whole-pose guesses from correlating the two clouds' floor plans.

    Returns [(T, yaw_deg)] best first -- at most `top`, deduplicated.

    This exists because a yaw ring with the centroids brought together is not a
    search over position at all: it offers exactly one translation per angle,
    and it assumes the two clouds' centroids are the same point of the world.
    Indoors that roughly holds -- both stations see the same four walls, so both
    centroids sit near the middle of the room. Outdoors it does not: each
    station sees a different slice of whatever is around it out to the lidar's
    range, so the centroids are pulled apart by that difference rather than by
    the baseline the operator actually walked. The guess can be metres out while
    ICP's correspondence gate is centimetres, and then every hypothesis in the
    ring starts outside the basin and converges to junk. No amount of extra
    iterations or finer grids recovers from it, because nothing was ever
    pointing at the right answer.

    Correlating plan views searches position properly instead. For each yaw the
    FFT scores *every* translation on the grid at once -- the peak is where the
    two floor plans line up -- so the cost is one transform per angle rather
    than one ICP per (angle, position) pair, and no basin has to be guessed at.

    `per_yaw` peaks are taken per angle because the true pose is not always the
    single tallest: a building front correlates well anywhere it slides along
    its own wall, and the runner-up peak is often the one with the corner in the
    right place. Refinement sorts them out; the point here is only to get the
    right answer into the candidate set.
    """
    src = np.asarray(source, float)
    tgt = np.asarray(target, float)
    ws = wall_footprint(src, voxel=voxel)
    wt = wall_footprint(tgt, voxel=voxel)
    if ws.shape[0] < 20 or wt.shape[0] < 20:
        return []

    # One raster big enough that no rotation of the source can slide off it, and
    # coarse enough that the transform stays cheap on a big outdoor scene.
    ext = max(float(np.ptp(wt[:, :2], axis=0).max()),
              float(np.ptp(ws[:, :2], axis=0).max()))
    n = int(np.ceil(ext / cell)) * 2 + 4
    if n > max_cells:
        cell *= n / max_cells
        n = max_cells
    shape = (n, n)

    t_lo = wt[:, :2].mean(axis=0) - n * cell / 2.0
    F = np.fft.rfft2(_plan_grid(wt[:, :2], t_lo, shape, cell))

    # Height is not part of the search: the rig stands on the ground at both
    # stations, so matching the two clouds' median height is already inside
    # ICP's reach, and searching it would multiply the candidates for nothing.
    dz = float(np.median(tgt[:, 2]) - np.median(src[:, 2]))

    found = []
    for k in range(int(max(yaw_steps, 1))):
        deg = 360.0 * k / max(yaw_steps, 1)
        R2 = yaw_matrix(deg)[:2, :2]
        s = ws[:, :2] @ R2.T
        s_lo = s.mean(axis=0) - n * cell / 2.0
        H = _plan_grid(s, s_lo, shape, cell)
        # Circular cross-correlation. Normalising by the source's own occupied
        # count keeps angles comparable: a rotation that rasterises to more
        # cells would otherwise score higher just for being fatter.
        c = np.fft.irfft2(F * np.conj(np.fft.rfft2(H)), shape)
        c = c / max(float(H.sum()), 1.0)
        for f in np.argsort(c.ravel())[::-1][:int(max(per_yaw, 1))]:
            i, j = np.unravel_index(f, shape)
            # A circular shift past the halfway point is a negative one.
            di = i - shape[0] if i > shape[0] // 2 else i
            dj = j - shape[1] if j > shape[1] // 2 else j
            d = (t_lo - s_lo) + np.array([di, dj], float) * cell
            found.append((float(c[i, j]), deg, d[0], d[1]))

    found.sort(key=lambda r: -r[0])
    out, seen = [], []
    for _sc, deg, dx, dy in found:
        # Neighbouring cells of one peak are the same hypothesis; ICP pulls them
        # to an identical pose, so keeping them only crowds out real
        # alternatives further down the list.
        key = (deg, round(dx / (cell * 3.0)), round(dy / (cell * 3.0)))
        if key in seen:
            continue
        seen.append(key)
        T = yaw_matrix(deg)
        T[:3, 3] = [dx, dy, dz]
        out.append((T, deg))
        if len(out) >= int(top):
            break
    return out


class _Target:
    """A target cloud prepared once and reused across ICP iterations."""

    def __init__(self, xyz, normals_k=18):
        self.xyz = np.asarray(xyz, dtype=np.float64)
        self.tree = cKDTree(self.xyz)
        self.normals = estimate_normals(self.xyz, k=normals_k)


# --- the solver --------------------------------------------------------------

def _point_to_plane_step(p, q, n):
    """One Gauss-Newton step. Returns the incremental 4x4, in the world frame.

    Linearising R about the identity as (I + [w]x) turns the residual
        ((R p + t) - q) . n
    into
        (p - q).n + w.(p x n) + t.n
    which is linear in the 6-vector [w, t] -- hence a 6x6 solve per iteration
    rather than a nonlinear optimisation. Valid because p is already carrying
    the transform accumulated so far, so each step only has to describe what is
    left over, which is small.
    """
    r = np.einsum("ij,ij->i", p - q, n)
    A = np.hstack([np.cross(p, n), n])
    AtA = A.T @ A
    Atb = -A.T @ r
    # A scan that only sees one flat wall constrains three of the six degrees
    # of freedom and leaves AtA singular. Damping keeps the solve from blowing
    # up on the unconstrained directions -- they just do not move.
    AtA[np.diag_indices(6)] += 1e-9 * max(np.trace(AtA), 1.0) / 6.0
    try:
        x = np.linalg.solve(AtA, Atb)
    except np.linalg.LinAlgError:
        return identity()
    if not np.all(np.isfinite(x)):
        return identity()
    T = identity()
    T[:3, :3] = _rotvec_to_matrix(x[:3])
    T[:3, 3] = x[3:]
    return T


def _stability(p, n, scale):
    """How well-constrained the alignment is, as the worst/best eigenvalue
    ratio of the point-to-plane normal matrix. 0 = free to slide, 1 = rigid.

    Fitness cannot answer this. Two scans that share nothing but a floor pair
    up beautifully -- every point finds a plane at the right height -- and
    report a near-perfect fit while being completely free to slide in x, y and
    yaw. That is the failure mode that silently produces a wrong map, so it
    gets its own number rather than being folded into the fit.

    The rotation and translation blocks carry different units (mm and none), so
    the cross-product block is divided by the cloud's radius first; without
    that the ratio would just track how big the room is.
    """
    if p.shape[0] < 10 or scale <= 0:
        return 0.0
    A = np.hstack([np.cross(p, n) / scale, n])
    AtA = A.T @ A / p.shape[0]
    w = np.linalg.eigvalsh(AtA)
    if w[-1] <= 1e-12:
        return 0.0
    return float(max(w[0], 0.0) / w[-1])


def icp(source, target, init=None, max_dist=200.0, max_iter=40,
        trim=0.8, tol=1e-4):
    """Point-to-plane ICP. Returns (T, fitness, rmse).

    `target` may be a prepared _Target (tree and normals already built) or a
    raw (N,3) array.

    fitness is the fraction of source points that found a target point within
    max_dist; rmse is over those pairs. Together they are the only honest
    signal about whether the answer means anything -- ICP always returns a
    transform, including for two clouds that share no surface at all.

    `trim` drops the worst tail of correspondences each iteration. Two scans
    from different positions never overlap completely, and the non-overlapping
    part pairs up with whatever happens to be nearest and drags the fit.
    """
    tgt = target if isinstance(target, _Target) else _Target(target)
    src = np.asarray(source, dtype=np.float64)
    T = identity() if init is None else np.asarray(init, float).copy()
    if src.shape[0] < 10 or tgt.xyz.shape[0] < 10:
        return T, 0.0, float("inf")

    fitness, rmse = 0.0, float("inf")
    prev = None
    for _ in range(int(max_iter)):
        p_all = src @ T[:3, :3].T + T[:3, 3]
        dist, j = tgt.tree.query(p_all, k=1,
                                 distance_upper_bound=max_dist, workers=-1)
        ok = np.isfinite(dist)
        n_ok = int(ok.sum())
        if n_ok < 10:
            return T, 0.0, float("inf")

        fitness = n_ok / src.shape[0]
        rmse = float(np.sqrt(np.mean(dist[ok] ** 2)))

        p, d = p_all[ok], dist[ok]
        idx = j[ok]
        if 0.0 < trim < 1.0 and n_ok > 50:
            thr = np.quantile(d, trim)
            keep = d <= thr
            p, idx = p[keep], idx[keep]
        if p.shape[0] < 10:
            return T, fitness, rmse

        T = _point_to_plane_step(p, tgt.xyz[idx], tgt.normals[idx]) @ T

        # Stop when the objective stops moving. Iterating a converged ICP is
        # pure cost, and there are a lot of these behind a yaw search.
        if prev is not None and abs(prev - rmse) < tol * max(rmse, 1.0):
            break
        prev = rmse

    return T, fitness, rmse


def evaluate(source, target, T, max_dist=120.0):
    """Score an alignment. Returns (fitness, rmse, stability, overlap).

    `fitness` is one-directional -- what fraction of the source landed on the
    target -- and is trivially 1.0 whenever a small cloud is dropped anywhere
    inside a much larger one. `overlap` is the smaller of the two directions,
    which is the number that actually says the scans see the same place.
    """
    tgt = target if isinstance(target, _Target) else _Target(target)
    src = np.asarray(source, dtype=np.float64)
    if src.shape[0] < 10 or tgt.xyz.shape[0] < 10:
        return 0.0, float("inf"), 0.0, 0.0

    p = src @ T[:3, :3].T + T[:3, 3]
    d, j = tgt.tree.query(p, k=1, distance_upper_bound=max_dist, workers=-1)
    ok = np.isfinite(d)
    if not ok.any():
        return 0.0, float("inf"), 0.0, 0.0
    fitness = float(ok.mean())
    rmse = float(np.sqrt(np.mean(d[ok] ** 2)))

    # The reverse direction, on the target's own points.
    back = cKDTree(p)
    d2, _ = back.query(tgt.xyz, k=1, distance_upper_bound=max_dist, workers=-1)
    overlap = float(min(fitness, np.isfinite(d2).mean()))

    idx = j[ok]
    pm = p[ok]
    scale = max(float(np.linalg.norm(pm - pm.mean(axis=0), axis=1).mean()), 1.0)
    stab = _stability(pm - pm.mean(axis=0), tgt.normals[idx], scale)
    return fitness, rmse, stab, overlap


def _score(metrics):
    """Rank alignments by (fitness, rmse, stability, overlap).

    Mutual overlap leads, because a cloud dropped inside a bigger one scores a
    perfect one-way fitness while being nowhere near right. Residual is the
    tiebreak, and stability gets a small weight so that of two equally-covered
    answers the one that is actually pinned down wins.
    """
    _fit, rmse, stab, ov = metrics
    return ov - 0.001 * min(rmse, 1000.0) + 0.25 * stab


# --- the thing the UI calls --------------------------------------------------

def register(source, target, voxel=40.0, levels=3, yaw_steps=12,
             init=None, yaw_hint=None, max_points=60000, progress=None,
             top_k=4, min_voxel=None, fine_points=150000,
             structure=False, struct_max_variation=0.06, struct_k=16):
    """Align `source` onto `target`. Returns a dict describing the result.

    Keys: T (4x4), fitness (0..1), rmse (mm), voxel, yaw (the coarse yaw the
    search settled on, degrees), source_points, target_points.

    Strategy, coarsest first:

    1. If no `init` is given, try a ring of yaw hypotheses about the target's
       centroid, each with the two clouds' centroids brought together, plus the
       identity. Everything else the operator does between scans -- a step
       sideways, a small tilt -- is inside ICP's basin of convergence; turning
       the rig to face a different wall is not, and it is the failure that
       looks like the algorithm simply does not work.

    2. Refine the winner coarse-to-fine, each level with a tighter voxel and a
       tighter correspondence gate, so early iterations pull the cloud a long
       way without caring about detail and later ones settle it precisely.

    3. Polish the single winning pose at sub-voxel grids (`voxel` down to
       `min_voxel`), with far more points than the search levels use. This is
       the step that makes a merge accurate rather than merely correct. The
       coarse-to-`voxel` search only pins the transform to a fraction of a
       40 mm cell; across a room that is many metres wide, a fraction of a
       degree of leftover rotation is centimetres of error at the far wall --
       invisible in a single scan, but exactly the wobble that shows up when
       two clouds are laid on top of each other. Polishing on a fine grid, with
       enough points spread to the room's edges to give rotation a long lever
       arm, drives that leftover angle down to the lidar's own precision. Only
       the winner is polished, because by now the pose is right to a cell and
       what is left is precision, not which hypothesis -- and the fine grids
       are the expensive part.

    `min_voxel` is the finest grid the polish descends to (default voxel/4, but
    never below 8 mm -- past the lidar's own noise there is nothing left to
    align to). `fine_points` caps the polish clouds.

    `structure` turns on vegetation rejection (see structure_filter): both
    clouds are stripped to surface points before the transform is solved *and*
    before it is scored, so foliage stops injecting random point-to-plane
    constraints into the solve and stops inflating the residual the verdict
    reads. The returned T is still expressed in the input frame and applies to
    the full, unfiltered clouds -- filtering only selects which points vote.

    `progress` is an optional callable taking a short status string.
    """
    src_full = np.asarray(source, dtype=np.float64)
    tgt_full = np.asarray(target, dtype=np.float64)
    result = {"T": identity(), "fitness": 0.0, "rmse": float("inf"),
              "stability": 0.0, "overlap": 0.0, "voxel": voxel, "yaw": 0.0,
              "structured": bool(structure),
              "source_points": int(src_full.shape[0]),
              "target_points": int(tgt_full.shape[0])}
    if src_full.shape[0] < 100 or tgt_full.shape[0] < 100:
        return result

    def say(msg):
        if progress is not None:
            progress(msg)

    if structure:
        say("finding structure (ignoring foliage)...")
        src_full = structure_filter(src_full, voxel=voxel, k=struct_k,
                                    max_variation=struct_max_variation)
        tgt_full = structure_filter(tgt_full, voxel=voxel, k=struct_k,
                                    max_variation=struct_max_variation)
        result["source_structure_points"] = int(src_full.shape[0])
        result["target_structure_points"] = int(tgt_full.shape[0])
        if src_full.shape[0] < 100 or tgt_full.shape[0] < 100:
            return result

    # Level 0 is the coarsest. Downsampling is what makes this tractable: a
    # room scan is millions of points and ICP does not get any better answer
    # from them than it does from a 40 mm grid.
    sizes = [voxel * (2 ** (levels - 1 - i)) for i in range(levels)]

    def prep(xyz, size, cap):
        d = voxel_downsample(xyz, size)
        if d.shape[0] > cap:
            # Deterministic stride rather than a random draw, so re-running a
            # registration on the same input gives the same answer.
            d = d[:: int(np.ceil(d.shape[0] / cap))]
        return d

    coarse_src = prep(src_full, sizes[0], max_points // 4)
    coarse_tgt_xyz = prep(tgt_full, sizes[0], max_points // 2)
    coarse_tgt = _Target(coarse_tgt_xyz)

    s_mid = coarse_src.mean(axis=0)
    t_mid = coarse_tgt_xyz.mean(axis=0)

    def from_yaw(deg):
        """Yaw plus the translation that brings the two centroids together.

        The translation is not optional. Two scans of the same room from
        different spots are separated by the baseline the operator walked --
        metres -- and ICP only pairs points inside its correspondence gate,
        which is centimetres. A rotation-only guess leaves the clouds too far
        apart to have a single valid correspondence, and ICP sits there.
        """
        R = yaw_matrix(deg)[:3, :3]
        T = identity()
        T[:3, :3] = R
        T[:3, 3] = t_mid - R @ s_mid
        return T

    # The per-level clouds do not depend on which hypothesis is being refined,
    # so build them once. This is what makes refining several candidates
    # affordable: the KD-trees and normals are the expensive part, not ICP.
    levels_data = [(coarse_src, coarse_tgt)]
    for size in sizes[1:]:
        levels_data.append((prep(src_full, size, max_points // 2),
                            _Target(prep(tgt_full, size, max_points))))

    def refine(T0):
        """Run a hypothesis coarse-to-fine. Returns (T, score, metrics)."""
        T = np.asarray(T0, float).copy()
        for (s, t), size in zip(levels_data, sizes):
            # The gate shrinks with the grid: wide early so points can travel,
            # tight at the end so only genuinely corresponding surfaces pair up.
            T, _, _ = icp(s, t, init=T, max_dist=size * 3.0,
                          max_iter=40, trim=0.85)
        s, t = levels_data[-1]
        m = evaluate(s, t, T, max_dist=voxel * 3.0)          # fit, rmse, stab, ov
        return T, _score(m), m

    if init is not None:
        starts = [(np.asarray(init, float).copy(), 0.0)]
    elif yaw_hint is not None:
        # Trust the operator on which way the rig was pointing, but still let
        # ICP find the rest. A hint that is 20 degrees out is still inside the
        # basin once the centroids are together.
        starts = [(from_yaw(float(yaw_hint)), float(yaw_hint))]
        say(f"using yaw hint {float(yaw_hint):.0f} deg")
    else:
        say("matching floor plans...")
        # Plan-view correlation first: it is the only part of the search that
        # actually looks for *where* the second station stood, rather than
        # assuming the centroids answer that. Cheap enough (one FFT per angle)
        # to run unconditionally.
        plan = footprint_starts(src_full, tgt_full, cell=max(voxel * 2.5, 50.0))

        say("searching orientation...")
        candidates = [(identity(), 0.0)]
        for k in range(int(max(yaw_steps, 1))):
            deg = 360.0 * k / max(yaw_steps, 1)
            candidates.append((from_yaw(deg), deg))
        candidates.extend(plan)

        # Screen cheaply, then refine the best few properly.
        #
        # Screening alone is not enough to pick the winner, and trusting it was
        # a real bug: a room is mostly walls, floor and ceiling, so the pose
        # that is 180 degrees wrong lines those up just as well and only the
        # furniture tells them apart. Twelve iterations do not get either
        # hypothesis close enough for that difference to show, and the alias
        # would out-score the truth. Fully converged, the truth wins -- so the
        # decision has to be made after refinement, not before it.
        n_ring = len(candidates) - len(plan)
        screened = []
        for i, (T0, deg) in enumerate(candidates):
            T1, _, _ = icp(coarse_src, coarse_tgt, init=T0,
                           max_dist=sizes[0] * 3.0, max_iter=12, trim=0.7)
            m = evaluate(coarse_src, coarse_tgt, T1, max_dist=sizes[0] * 1.5)
            screened.append((_score(m), T1, deg, i >= n_ring))
        screened.sort(key=lambda r: -r[0])
        starts = [(T1, deg) for _, T1, deg, _p in screened[:max(int(top_k), 1)]]

        # The screen is twelve iterations on the coarsest grid, and at that
        # resolution a plan-view hypothesis that is right has not yet pulled
        # ahead of the ring hypotheses that are merely parked on the ground
        # plane -- the same reason the winner cannot be picked from screening
        # alone. So the best-screened plan candidate is refined whether or not
        # it made the cut. It is one extra refinement, and on the scans where
        # the ring is hopeless it is the only one that matters.
        if plan and not any(p for _s, _T, _d, p in screened[:max(int(top_k), 1)]):
            for _s, T1, deg, p in screened:
                if p:
                    starts.append((T1, deg))
                    break

    best = None
    for i, (T0, deg) in enumerate(starts):
        if len(starts) > 1:
            say(f"refining candidate {i + 1}/{len(starts)} (yaw {deg:.0f})...")
        else:
            say("refining...")
        T, score, m = refine(T0)
        if best is None or score > best[0]:
            best = (score, T, deg, m)

    _, T, yaw, m = best

    # Polish the winner on sub-voxel grids. The grids below `voxel` halve down
    # to min_voxel; each carries fine_points, far more than the search levels,
    # because it is the points out at the room's edges that give leftover
    # rotation a long enough lever arm to be seen and removed.
    if min_voxel is None:
        min_voxel = max(voxel / 4.0, 8.0)
    polish_sizes = []
    size = voxel / 2.0
    while size >= min_voxel - 1e-9:
        polish_sizes.append(size)
        size /= 2.0

    for j, size in enumerate(polish_sizes):
        say(f"polishing {j + 1}/{len(polish_sizes)} ({size:.0f} mm grid)...")
        s = prep(src_full, size, fine_points)
        t = _Target(prep(tgt_full, size, fine_points))
        # A wider gate than the search's (size*3) at the first, coarsest polish
        # step: correspondences can still sit a working-voxel apart here, and a
        # gate tied to the fine grid would drop the very pairs that carry the
        # correction. It tightens on its own as the grid shrinks.
        T, _, _ = icp(s, t, init=T, max_dist=max(size * 4.0, voxel * 1.5),
                      max_iter=40, trim=0.9)

    # Score the polished pose at the working voxel, so `verdict`'s thresholds
    # (calibrated in units of `voxel`) keep meaning what they meant. The polish
    # improves T; it does not move the goalposts the result is judged against.
    s, t = levels_data[-1]
    fitness, rmse, stab, overlap = evaluate(s, t, T, max_dist=voxel * 3.0)

    result.update(T=T, fitness=float(fitness), rmse=float(rmse),
                  stability=float(stab), overlap=float(overlap),
                  yaw=float(yaw))
    return result


# Thresholds, calibrated against measured alignments of a cluttered room.
# Correct ones land at overlap 0.75-0.99, residual 0.7-0.95 voxels, stability
# 0.23-0.27. Wrong ones -- a pose 40 degrees out, a different room, two scans
# sharing only a floor -- land at overlap 0.28-0.50, residual 1.4 voxels, or
# stability 0.03. The bands below sit in the gaps.
GOOD_OVERLAP, POOR_OVERLAP = 0.60, 0.35
GOOD_STABILITY, POOR_STABILITY = 0.10, 0.05
GOOD_RMSE_VOXELS, POOR_RMSE_VOXELS = 1.15, 1.5

GOOD, CHECK, POOR = "good", "check", "poor"


def verdict(result):
    """(level, explanation) for a register() result.

    Three levels rather than a yes/no, because the two decisions are not the
    same one. Merging a wrong alignment corrupts the map silently and
    irreversibly -- afterwards there is no way to tell which scan did it -- so
    adding without asking has to demand real confidence. Showing a result to
    someone who can look at it on screen does not.

    GOOD  -- safe to merge unattended.
    CHECK -- plausible, but wants a human to look at the overlap first.
    POOR  -- almost certainly wrong; say why.

    No threshold can do better than this. A symmetric room genuinely admits
    more than one alignment that fits the measurements equally well, and
    picking between them needs knowledge of where the operator was standing,
    which is not in the data.
    """
    ov = result.get("overlap", 0.0)
    stab = result.get("stability", 0.0)
    rmse = result.get("rmse", float("inf"))
    voxel = result.get("voxel", 40.0)

    stats = (f"{ov * 100:.0f}% overlap, {rmse:.0f} mm residual, "
             f"stability {stab:.2f}")

    bad = []
    if ov < POOR_OVERLAP:
        bad.append(f"only {ov * 100:.0f}% of the scans overlap")
    if stab < POOR_STABILITY:
        bad.append("the shared region is flat and featureless, so the fit is "
                   "free to slide along it")
    if not np.isfinite(rmse) or rmse > POOR_RMSE_VOXELS * voxel:
        bad.append(f"residual {rmse:.0f} mm is far above the {voxel:.0f} mm "
                   f"working detail")
    if bad:
        return POOR, "; ".join(bad)

    weak = []
    if ov < GOOD_OVERLAP:
        weak.append(f"overlap is only {ov * 100:.0f}%")
    if stab < GOOD_STABILITY:
        weak.append("the shared region is nearly featureless")
    if rmse > GOOD_RMSE_VOXELS * voxel:
        weak.append(f"residual {rmse:.0f} mm is high")
    if weak:
        return CHECK, "; ".join(weak) + f" ({stats})"
    return GOOD, stats
