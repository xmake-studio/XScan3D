#!/usr/bin/env python3
"""Turn a scanned point cloud into a triangle mesh.

Three reconstructions, all of them standard:

  poisson   Screened Poisson surface reconstruction (Kazhdan). Watertight,
            smooth, tolerant of noise -- the usual default for scanner data.
            Needs oriented normals, and needs open3d.
  bpa       Ball pivoting. Keeps the sampled points as mesh vertices, so it
            preserves detail and leaves genuine holes as holes. Needs open3d.
  alpha     3D alpha shape off a Delaunay tetrahedralisation. Cruder than the
            other two, but it is the one that runs on a plain scipy install,
            so there is always something available.

Poisson and ball pivoting each come from whichever of two libraries is
present. They are the same published algorithms either way:

  pymeshlab  MeshLab's build. Has wheels for current Pythons, including 3.13,
             which is the only reason this project can offer Poisson at all --
             open3d stops at 3.12.
  open3d     The other common packaging, for older interpreters.

Both are optional. `available_methods()` reports what this machine can
actually do, and the UI only offers those, rather than failing at the point
where the user presses Build.

Everything here works in millimetres, matching the rest of the pipeline.
"""

import numpy as np

try:
    import pymeshlab as ml
    HAVE_PYMESHLAB = True
except Exception:
    ml = None
    HAVE_PYMESHLAB = False

try:
    import open3d as o3d
    HAVE_OPEN3D = True
except Exception:      # not installed, or installed and broken -- same to us
    o3d = None
    HAVE_OPEN3D = False

# pymeshlab first: it is the one that exists on the Pythons this runs on.
HAVE_RECON = HAVE_PYMESHLAB or HAVE_OPEN3D


# name -> (label, needs a reconstruction backend)
METHODS = {
    "poisson": ("Poisson", True),
    "bpa": ("Ball pivoting", True),
    "alpha": ("Alpha shape", False),
}

# How many points each method is worth feeding, measured on this project's
# clouds. They differ by nearly an order of magnitude because the algorithms
# scale so differently:
#
#   poisson  roughly linear -- 180k in about 13 s. It also fits a smooth
#            surface rather than interpolating points, so extra samples past
#            what the octree resolves buy nothing anyway.
#   bpa      badly superlinear -- 45k in 8 s but 120k in 129 s, so a 270k
#            cloud is twenty minutes. This is the one that needs a hard cap.
#   alpha    Delaunay is about six tets per point, and memory rather than
#            time is the wall.
METHOD_BUDGET = {"poisson": 150000, "bpa": 40000, "alpha": 80000}


def backend():
    """Which library is doing the reconstruction, for the UI to report."""
    if HAVE_PYMESHLAB:
        return "pymeshlab"
    if HAVE_OPEN3D:
        return "open3d"
    return None


def available_methods():
    """Method keys usable on this machine, best first."""
    return [k for k, (_, needs) in METHODS.items() if HAVE_RECON or not needs]


def missing_note():
    """Why the good methods are absent, or None when they are not."""
    if HAVE_RECON:
        return None
    return ("no reconstruction backend -- only the alpha shape is available.\n"
            "  pip install pymeshlab   for Poisson and ball pivoting.\n"
            "  (open3d also works, but has no wheels past Python 3.12.)")


# --- shared helpers ---------------------------------------------------------

def spacing(xyz, k=1):
    """Median distance to the k-th nearest neighbour. The natural length unit
    of a cloud: every radius below is quoted as a multiple of it, so one set of
    defaults works for a desk and for a room."""
    from scipy.spatial import cKDTree
    if len(xyz) < 2:
        return 1.0
    d, _ = cKDTree(xyz).query(xyz, k=min(k + 1, len(xyz)))
    return max(float(np.median(d[:, -1])), 1e-6)


def voxel_downsample(xyz, size):
    """One point per occupied cell of a `size` grid, at the cell's centroid.

    Averaging rather than picking a representative: the mean of the points in
    a cell is a better estimate of the surface than any one of them, so this
    denoises slightly as it thins.
    """
    if size <= 0:
        return xyz
    key = np.floor(xyz / size).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = inv.max() + 1
    out = np.zeros((n, 3))
    np.add.at(out, inv, xyz)
    return out / np.bincount(inv, minlength=n)[:, None]


def downsample_to_budget(xyz, budget):
    """Thin to about `budget` points on a uniform grid.

    Uniform is the operative word. A lidar sweep is wildly non-uniform -- dense
    near the scanner and along the spin axis, sparse far away -- and both the
    alpha shape and ball pivoting take a single radius for the whole cloud, so
    non-uniform density is precisely what makes them fail: any radius large
    enough to close the sparse regions has already swallowed the detail in the
    dense ones. That is the "holes or no detail, pick one" trade, and it is a
    property of the sampling rather than of the algorithms.

    Thinning on a grid removes the variation instead of preserving it the way
    random subsampling does, so one radius then works everywhere.
    """
    if budget <= 0 or len(xyz) <= budget:
        return xyz, 0.0

    # Bracket a voxel size that lands near the budget, then bisect. Point count
    # falls monotonically with cell size, so this converges quickly, and it is
    # far cheaper than it looks: each probe is one pass of integer division.
    lo, hi = 0.0, float(np.ptp(xyz, axis=0).max())
    span = hi
    best = xyz
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        if mid <= 0 or mid < span * 1e-6:
            break
        out = voxel_downsample(xyz, mid)
        if len(out) > budget:
            lo = mid
        else:
            best, hi = out, mid
            # Close enough: chasing the exact figure costs passes and buys
            # nothing, since the budget is a rough guard rail anyway.
            if len(out) > budget * 0.9:
                break
    return best, hi


def _to_o3d(xyz, normal_radius, normal_k):
    """Point cloud with normals oriented towards the scanner.

    Normal *orientation* is the part of Poisson that usually goes wrong, and
    it is free here: the lidar sits at the world origin, so every surface it
    saw is a surface facing it. That beats the generic minimum-spanning-tree
    propagation, which flips whole patches on scenes with thin structure.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=normal_radius, max_nn=normal_k))
    pcd.orient_normals_towards_camera_location(np.zeros(3))
    return pcd


def _from_o3d(mesh):
    return (np.asarray(mesh.vertices, dtype=np.float32),
            np.asarray(mesh.triangles, dtype=np.int32))


# --- the three reconstructions ----------------------------------------------

def _pml_cloud(xyz, normal_k):
    """MeshLab point cloud with normals pointed back at the scanner.

    Same trick as the open3d path: viewpos is the world origin, which is
    where the lidar physically is, so "facing the viewpoint" is exactly right
    rather than a heuristic.
    """
    ms = ml.MeshSet()
    ms.add_mesh(ml.Mesh(vertex_matrix=np.asarray(xyz, dtype=np.float64)))
    ms.compute_normal_for_point_clouds(
        k=int(normal_k), smoothiter=0, flipflag=True,
        viewpos=np.zeros(3))
    return ms


def _from_pml(ms):
    m = ms.current_mesh()
    return (np.asarray(m.vertex_matrix(), dtype=np.float32),
            np.asarray(m.face_matrix(), dtype=np.int32))


def _poisson(xyz, depth, normal_radius, normal_k):
    if HAVE_PYMESHLAB:
        ms = _pml_cloud(xyz, normal_k)
        # preclean drops the duplicate and unreferenced vertices that a lidar
        # cloud is full of, which the solver would otherwise weight twice.
        ms.generate_surface_reconstruction_screened_poisson(
            depth=int(depth), preclean=True)
        return _from_pml(ms)

    pcd = _to_o3d(xyz, normal_radius, normal_k)
    mesh, _density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=int(depth))
    return _from_o3d(mesh)


def _bpa(xyz, radius_mult, normal_radius, normal_k):
    r = spacing(np.asarray(xyz)) * float(radius_mult)
    if HAVE_PYMESHLAB:
        ms = _pml_cloud(xyz, normal_k)
        # AbsoluteValue was renamed PureValue somewhere around the 2023
        # releases, and both spellings are still in the wild.
        absolute = getattr(ml, "PureValue", None) or ml.AbsoluteValue
        ms.generate_surface_reconstruction_ball_pivoting(
            ballradius=absolute(float(r)))
        return _from_pml(ms)

    pcd = _to_o3d(xyz, normal_radius, normal_k)
    # A ladder of radii, not one: a single ball either bridges the sparse
    # regions or shrink-wraps the dense ones, never both. Successive passes
    # with a bigger ball fill what the previous pass left open.
    radii = o3d.utility.DoubleVector([r, r * 2.0, r * 4.0])
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, radii)
    return _from_o3d(mesh)


def trim_to_samples(verts, faces, xyz, max_dist):
    """Drop the parts of a surface that no measurement supports.

    Poisson always closes its surface, so it invents geometry across
    everything the lidar never saw -- scan one wall and it comes back wrapped
    round into a blob. The invented parts are exactly the parts far from any
    real sample, so this is what turns the blob back into a wall.

    Done here by distance rather than by the solver's own density estimate so
    that it means the same thing whichever backend ran, and so the threshold
    is a length the user can reason about.
    """
    if max_dist <= 0 or not len(faces):
        return verts, faces
    from scipy.spatial import cKDTree
    d, _ = cKDTree(np.asarray(xyz)).query(verts)
    keep = d <= max_dist
    if keep.all():
        return verts, faces
    # A triangle survives only with all three corners supported; dropping on
    # "any" would leave spikes reaching out to the trimmed-away region.
    faces = faces[keep[faces].all(axis=1)]
    if not len(faces):
        return verts, faces
    used, faces = np.unique(faces, return_inverse=True)
    return verts[used], faces.reshape(-1, 3).astype(np.int32)


def _alpha_scipy(xyz, alpha):
    """Boundary of the alpha complex, via Delaunay. numpy + scipy only.

    A tetrahedron is in the complex when its circumsphere is smaller than
    alpha; the surface is every triangle that ended up in exactly one such
    tetrahedron. Large alpha tends to the convex hull, small alpha crumbles
    into nothing, and the useful range is a few times the point spacing.
    """
    from scipy.spatial import Delaunay
    xyz = np.asarray(xyz, dtype=np.float64)
    tets = Delaunay(xyz).simplices
    if not len(tets):
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)

    # Circumradius of every tet, batched. The circumcentre solves
    # 2(p_i - p_0).c = |p_i|^2 - |p_0|^2 for i = 1..3.
    p = xyz[tets]                                     # (T, 4, 3)
    a = p[:, 0]
    A = 2.0 * (p[:, 1:] - a[:, None, :])              # (T, 3, 3)
    sq = (p ** 2).sum(axis=-1)                        # (T, 4)
    rhs = sq[:, 1:] - sq[:, :1]                       # (T, 3)

    # Flat (near-degenerate) tets have no usable circumcentre and would make
    # the batched solve raise for the whole array, so they are scored infinite
    # and simply never make the cut.
    scale = np.abs(A).max(axis=(1, 2))
    det = np.abs(np.linalg.det(A))
    ok = det > 1e-10 * np.maximum(scale, 1e-9) ** 3
    R = np.full(len(tets), np.inf)
    if ok.any():
        c = np.linalg.solve(A[ok], rhs[ok][..., None])[..., 0]
        R[ok] = np.linalg.norm(c - a[ok], axis=1)

    keep = tets[R < alpha]
    if not len(keep):
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)

    # Four faces per kept tet, each tagged with the vertex it faces away from.
    faces = np.concatenate([keep[:, [1, 2, 3]], keep[:, [0, 2, 3]],
                            keep[:, [0, 1, 3]], keep[:, [0, 1, 2]]])
    opp = np.concatenate([keep[:, 0], keep[:, 1], keep[:, 2], keep[:, 3]])

    # A face shared by two kept tets is interior; the surface is the rest.
    key = np.sort(faces, axis=1)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True,
                            return_counts=True)
    surf = cnt[inv.ravel()] == 1
    faces, opp = faces[surf], opp[surf]

    # Point every triangle away from the solid, so lit shading reads as a
    # surface instead of a patchwork of dark and bright facets.
    v0, v1, v2 = xyz[faces[:, 0]], xyz[faces[:, 1]], xyz[faces[:, 2]]
    n = np.cross(v1 - v0, v2 - v0)
    inward = np.einsum("ij,ij->i", n, xyz[opp] - v0) > 0
    faces[inward] = faces[inward][:, [0, 2, 1]]

    used, faces = np.unique(faces, return_inverse=True)
    return (xyz[used].astype(np.float32),
            faces.reshape(-1, 3).astype(np.int32))


def _alpha(xyz, alpha):
    # No pymeshlab branch: MeshLab has no alpha-shape filter, and the scipy
    # implementation below is the point of this method anyway.
    if HAVE_OPEN3D:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(xyz, np.float64))
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd, float(alpha))
        return _from_o3d(mesh)
    return _alpha_scipy(xyz, alpha)


# --- smoothing --------------------------------------------------------------

def smooth(verts, faces, iters, lam=0.5):
    """Laplacian smoothing: nudge each vertex towards its neighbours' mean.

    Mainly for the alpha shape, whose surface is made of sampled points and so
    carries the lidar's per-sample noise as visible faceting.
    """
    if iters <= 0 or not len(faces):
        return verts
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    e = np.concatenate([e, e[:, ::-1]])          # both directions
    deg = np.bincount(e[:, 0], minlength=len(verts)).astype(np.float32)
    deg[deg == 0] = 1.0
    v = verts.astype(np.float32).copy()
    for _ in range(int(iters)):
        acc = np.zeros_like(v)
        np.add.at(acc, e[:, 0], v[e[:, 1]])
        v += lam * (acc / deg[:, None] - v)
    return v


# --- entry point ------------------------------------------------------------

def reconstruct(xyz, method="poisson", *, depth=9, trim_mult=3.0,
                radius_mult=2.0, alpha_mult=4.0, normal_k=30,
                normal_mult=4.0, smooth_iters=0, budget=0):
    """Mesh a cloud. Returns (verts (V,3) float32, faces (F,3) int32, info).

    The radius-ish parameters are multiples of the cloud's own point spacing,
    not absolute millimetres, so the defaults survive a change of subject.

    `budget` thins the input to about that many points on a uniform grid
    first; 0 means feed the lot. info reports what actually happened, since
    the thinning is otherwise invisible and it changes the result.

    Raises ValueError with something worth reading when a method is not
    available or the cloud is too small to mesh.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < 4:
        raise ValueError("need at least 4 points to build a surface")
    if method not in METHODS:
        raise ValueError(f"unknown method {method!r}")
    if METHODS[method][1] and not HAVE_RECON:
        raise ValueError(f"{METHODS[method][0]} needs pymeshlab "
                         f"(pip install pymeshlab)")

    info = {"input": len(xyz), "voxel": 0.0}
    if budget:
        xyz, info["voxel"] = downsample_to_budget(xyz, int(budget))
    info["used"] = len(xyz)
    if len(xyz) < 4:
        raise ValueError("too few points left after thinning")

    s = spacing(xyz)
    info["spacing"] = s
    normal_radius = s * float(normal_mult)

    if method == "poisson":
        verts, faces = _poisson(xyz, depth, normal_radius, normal_k)
        verts, faces = trim_to_samples(verts, faces, xyz,
                                       s * float(trim_mult))
    elif method == "bpa":
        verts, faces = _bpa(xyz, radius_mult, normal_radius, normal_k)
    else:
        verts, faces = _alpha(xyz, s * float(alpha_mult))

    if smooth_iters and len(faces):
        verts = smooth(verts, faces, smooth_iters)
    return verts.astype(np.float32), faces.astype(np.int32), info


# --- export -----------------------------------------------------------------

def export_ply(path, verts, faces, rgb=None):
    """Binary PLY with faces. rgb is (V,3) uint8, or None."""
    props = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    if rgb is not None:
        props += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
    vert = np.empty(len(verts), dtype=props)
    vert["x"], vert["y"], vert["z"] = verts[:, 0], verts[:, 1], verts[:, 2]
    if rgb is not None:
        vert["red"], vert["green"], vert["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]

    # PLY face lists are per-element counted, so the count column rides along
    # in the same record rather than being written separately.
    face = np.empty(len(faces), dtype=[("n", "u1"), ("v", "<i4", 3)])
    face["n"] = 3
    face["v"] = faces

    with open(path, "wb") as fh:
        fh.write(b"ply\nformat binary_little_endian 1.0\n")
        fh.write(f"element vertex {len(verts)}\n".encode())
        fh.write(b"property float x\nproperty float y\nproperty float z\n")
        if rgb is not None:
            fh.write(b"property uchar red\nproperty uchar green\n"
                     b"property uchar blue\n")
        fh.write(f"element face {len(faces)}\n".encode())
        fh.write(b"property list uchar int vertex_indices\n")
        fh.write(b"end_header\n")
        fh.write(vert.tobytes())
        fh.write(face.tobytes())


def export_stl(path, verts, faces):
    """Binary STL. No colour and no shared vertices, but every slicer and CAD
    package reads it, which is the point of offering it at all."""
    tri = verts[faces].astype(np.float32)                  # (F, 3, 3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)

    rec = np.zeros(len(faces), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)),
                                      ("attr", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(np.uint32(len(faces)).tobytes())
        fh.write(rec.tobytes())
