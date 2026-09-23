#!/usr/bin/env python3
"""Fit the mount geometry -- lidar roll, emitter spacing, scan-plane tilt --
to a capture of an ordinary room.

    python tools/calibrate_mount.py scans/room.bin
    python tools/calibrate_mount.py scans/room.bin --rotation 161.65 --spacing -36.5

Nothing here needs a target: a room is full of surfaces known to be flat, and
the three constants each bend them in their own way (see the notes in
scan_proto.py). Two things are scored:

  * the big planes -- walls, ceiling, floor -- found by RANSAC in the starting
    cloud. Their points are held fixed and the planes are refitted under each
    trial geometry, so a wrong tilt shows up as a nearby wall twisting into a
    saddle and a wrong roll as the ceiling creasing along the seam;
  * the seam, where the end of the sweep meets the start: each surface there is
    seen by both halves of the lidar's revolution, and they should agree. It is
    scored separately overhead, at the horizon and below, so the fit cannot
    buy a better ceiling with a worse horizon.

Takes a few minutes. The UI runs the same fit from Mount geometry ->
Calibrate from scan; from here it prints the values to enter there.
"""

import argparse
import os
import sys

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scan_proto as sp  # noqa: E402


def load(path):
    p = sp.StreamParser(echo_events=False, sweep_only=True)
    for chunk in sp.file_source(path):
        p.feed(chunk)
    sp.keep_last_sweep(p.cap)
    return p.cap


class Scene:
    """The capture, reduced to what a trial geometry needs."""

    def __init__(self, cap, reverse, microstep, range_error):
        self.cap = cap
        self.reverse = reverse
        self.microstep = microstep
        self.range_error = range_error
        # Shaft angle per kept point, for splitting out the seam.
        _, _, self.platform = sp.build_cloud(cap, lidar_reverse=reverse,
                                             scan_tilt=0.0)

    def cloud(self, rotation, spacing, tilt):
        xyz, _, _ = sp.build_cloud(
            self.cap, lidar_rotation=rotation, lidar_reverse=self.reverse,
            emitter_spacing=spacing, scan_tilt=tilt, microstep=self.microstep,
            range_error=self.range_error)
        return xyz


def _fit_plane(p, tol, rounds=4):
    for _ in range(rounds):
        c = p.mean(0)
        n = np.linalg.svd(p - c, full_matrices=False)[2][2]
        r = (p - c) @ n
        keep = np.abs(r) < tol
        if keep.all():
            break
        p = p[keep]
    return n, c, float(r[keep].std()) if keep.any() else np.inf


def find_planes(P, count=8, voxel=40.0, tol=12.0, min_voxels=400, seed=0):
    """Indices of P on the largest level or plumb planes, one array each."""
    rng = np.random.default_rng(seed)
    _, first = np.unique(np.floor(P / voxel).astype(np.int64), axis=0,
                         return_index=True)
    Q = P[first]
    alive = np.ones(len(Q), bool)
    planes = []
    for _ in range(40):
        if len(planes) >= count:
            break
        idx = np.flatnonzero(alive)
        if len(idx) < min_voxels:
            break
        best = (0, None, None)
        for _ in range(400):
            t = Q[rng.choice(idx, 3, replace=False)]
            n = np.cross(t[1] - t[0], t[2] - t[0])
            if np.linalg.norm(n) < 1e-6:
                continue
            n /= np.linalg.norm(n)
            k = np.sum(np.abs((Q[idx] - t[0]) @ n) < tol)
            if k > best[0]:
                best = (k, n, t[0])
        k, n, c = best
        if k < min_voxels:
            break
        on = idx[np.abs((Q[idx] - c) @ n) < tol]
        alive[on] = False
        # Only walls, ceilings and floors: the clutter RANSAC also finds (a
        # sloping bedspread, a chair back) is not known to be flat.
        if 0.1 < abs(n[2]) < 0.97:
            continue
        n, c, _ = _fit_plane(Q[on], tol)
        # Back to full resolution: near the plane and inside its footprint.
        u = np.cross(n, [0.0, 0.0, 1.0] if abs(n[2]) < 0.9 else [1.0, 0.0, 0.0])
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        cell = 80.0
        foot = {tuple(x) for x in np.floor(
            np.stack([(Q[on] - c) @ u, (Q[on] - c) @ v], 1) / cell).astype(int)}
        near = np.flatnonzero(np.abs((P - c) @ n) < 15.0)
        g = np.floor(np.stack([(P[near] - c) @ u, (P[near] - c) @ v], 1)
                     / cell).astype(int)
        planes.append(near[[tuple(x) in foot for x in g]])
    return planes


def plane_score(P, planes):
    """Point-count weighted rms of the planes, refitted to this cloud."""
    tot = wsum = 0.0
    for idx in planes:
        # Tight enough that a picture frame proud of the wall is not the wall.
        rms = _fit_plane(P[idx], tol=6.0, rounds=3)[2]
        w = np.sqrt(len(idx))
        tot += rms * w
        wsum += w
    return tot / wsum


def seam_score(P, platform, span, margin=10.0, radius=50.0):
    """How well the two ends of the sweep agree, by elevation band.

    Points from the last `margin` degrees are measured against local planes
    fitted to points from the first `margin` degrees.
    """
    end = np.flatnonzero(platform > span - margin)[::2]
    start = platform < -span + margin
    A, B = P[end], P[start]
    tree = cKDTree(B)
    res = np.full(len(A), np.nan)
    for i, ids in enumerate(tree.query_ball_point(A, radius, workers=-1)):
        if len(ids) < 10:
            continue
        q = B[ids]
        c = q.mean(0)
        w, vec = np.linalg.eigh((q - c).T @ (q - c) / len(ids))
        if w[1] < 25.0 or w[0] > 0.05 * w[1]:
            continue
        res[i] = abs((A[i] - c) @ vec[:, 0])
    el = np.degrees(np.arctan2(A[:, 2], np.hypot(A[:, 0], A[:, 1])))
    ok = ~np.isnan(res)
    out = {}
    for name, m in (("up", ok & (el > 45)), ("horizon", ok & (abs(el) < 30)),
                    ("down", ok & (el < -30))):
        out[name] = float(np.mean(np.minimum(res[m], 15.0))) if m.any() \
            else 0.0
    return out


class Cancelled(Exception):
    """Raised out of calibrate() when its `cancelled` callback says so."""


# Frames a calibration works with before it starts thinning: about 5 minutes
# of sweep at the lidar's 6 revolutions (~375 frames) a second.
CALIBRATION_MAX_FRAMES = 120000


def calibrate(cap, rotation, spacing, tilt, reverse=sp.LIDAR_REVERSE,
              range_error=None, progress=None, cancelled=None,
              max_frames=CALIBRATION_MAX_FRAMES):
    """Fit lidar roll, emitter spacing and scan-plane tilt to one capture,
    and the lidar's range non-linearity (scan_proto.RANGE_ERROR) with them.

    Starts from the given values. `progress(text, fraction)` is called as the
    fit goes; `cancelled()` is polled and, once true, aborts with Cancelled.
    `range_error` is the range model in use now (None for none).
    Returns a dict: rotation, spacing, tilt, range_error (the fitted model, or
    the starting one if the scan could not improve on it), range_error_before,
    plus `before` and `after`, each a dict of the scores (planes, up,
    horizon, down; mm) at the start and end, and `stride`: 1 if every frame
    was used, else N for every Nth.
    Raises ValueError if the capture has nothing to fit against.

    `max_frames` caps how many lidar frames are scored; a longer capture is
    evenly thinned down to it. 0 or None uses every frame.
    """
    say = progress or (lambda text, frac: None)
    x0 = np.array([rotation, spacing, tilt], dtype=float)
    # Long sweeps are thinned to about max_frames lidar frames. Density past
    # that buys nothing -- on a 10-minute sweep, every other frame gives the
    # same scores to a tenth of a millimetre -- but each score costs roughly
    # the square of it (the seam search), which is the difference between
    # four minutes and ten. Shorter sweeps are used whole: there every frame
    # is still carrying information. Frames are self-contained -- each has its
    # own start and end angle -- so dropping some loses nothing but density.
    # Not so for legacy captures, whose frame width is the gap to the next.
    stride = int(np.ceil(len(cap) / max_frames)) if max_frames else 1
    if stride > 1 and (cap.arrays()["end_angle"] >= sp.RAW_ANGLE_MIN).all():
        thin = sp.Capture()
        thin.samples = cap.samples[::stride]
        thin.telem = list(cap.telem)
        thin.config = cap.config
        cap = thin
    else:
        stride = 1
    say("fitting the microstep error", 0.0)
    micro = sp.fit_microstep(cap, lidar_rotation=x0[0], emitter_spacing=x0[1],
                             scan_tilt=x0[2], lidar_reverse=reverse,
                             range_error=range_error)

    def fit_range(x):
        if cancelled is not None and cancelled():
            raise Cancelled()
        fitted = sp.fit_range_error(
            cap, microstep=micro, lidar_rotation=x[0], emitter_spacing=x[1],
            scan_tilt=x[2], lidar_reverse=reverse)
        return range_error if fitted is None else fitted

    start = Scene(cap, reverse, micro, range_error)
    # The range error is the lidar's and barely depends on the mount, but the
    # mount fit is cleaner without it, so it goes first -- and once more at
    # the end, on the final geometry.
    say("fitting the range error", 0.01)
    scene = Scene(cap, reverse, micro, fit_range(x0))
    span = float(np.abs(scene.platform).max())
    if span < 45.0:
        raise ValueError(f"the sweep only covers +-{span:.0f} deg; calibration "
                         "needs the two ends of a +-90 deg sweep to meet")

    say("finding walls, ceiling and floor", 0.04)
    planes = find_planes(scene.cloud(*x0))
    if len(planes) < 2:
        raise ValueError("found fewer than two walls/ceilings/floors -- scan "
                         "an ordinary room for this")

    # Roughly how many scores each stage takes, so the progress bar moves
    # evenly; Nelder-Mead's real count varies, so it is capped at the end.
    stage = {"name": "", "lo": 0.0, "hi": 0.0, "n": 0, "expect": 1}

    def begin(name, lo, hi, expect):
        stage.update(name=name, lo=lo, hi=hi, n=0, expect=expect)

    def detail(x, sc=None):
        sc = sc or scene
        P = sc.cloud(*x)
        sm = seam_score(P, sc.platform, span)
        sm["planes"] = plane_score(P, planes)
        return sm

    def score(x):
        if cancelled is not None and cancelled():
            raise Cancelled()
        sm = detail(x)
        stage["n"] += 1
        f = min(stage["n"] / stage["expect"], 1.0)
        say(f"{stage['name']}: roll {x[0]:.2f}, spacing {x[1]:.1f}, "
            f"tilt {x[2]:+.2f}", stage["lo"] + f * (stage["hi"] - stage["lo"]))
        # The horizon and below count in full: that is where the seam is most
        # visible. Overhead is noisier (grazing ceiling) and gets half weight.
        # On top of that, the horizon seam must not get worse than it started:
        # it is the one a user has usually already tuned by eye, and trading a
        # visible step there for a better ceiling is the wrong way round.
        worse = max(0.0, sm["horizon"] - before["horizon"])
        return (sm["planes"] + sm["horizon"] + sm["down"] + 0.5 * sm["up"]
                + 4.0 * worse)

    before = detail(x0, start)

    # The score is bumpy -- the seam terms count a changing set of points -- and
    # roll and tilt interact overhead, so one simplex from the start settles in
    # a dip well short of the real tilt. Staged instead: roll and spacing at
    # the starting tilt, then the tilt's basin by brute force, then all three.
    def polish(x, free, step):
        simplex = [x]
        for i, d in ((0, 0.3), (1, 4.0), (2, -0.5)):
            if free[i]:
                simplex.append(x + np.eye(3)[i] * d * step)
        fixed = np.array(x, dtype=float)
        idx = np.flatnonzero(free)

        def sub(v):
            y = fixed.copy()
            y[idx] = v
            return score(y)
        v = minimize(sub, fixed[idx], method="Nelder-Mead",
                     options=dict(initial_simplex=np.array(simplex)[:, idx],
                                  xatol=0.01, fatol=1e-4, maxiter=150)).x
        fixed[idx] = v
        return fixed

    begin("roll and spacing", 0.05, 0.30, 45)
    x = polish(x0, [True, True, False], 1.0)
    tilts = np.arange(-3.0, 3.01, 0.5)
    begin("tilt search", 0.30, 0.45, len(tilts))
    best = min(tilts, key=lambda t: score(np.array([x[0], x[1], t])))
    begin("all three", 0.45, 0.95, 80)
    x = polish(np.array([x[0], x[1], best]), [True, True, True], 0.5)
    say("refitting the range error", 0.96)
    scene.range_error = fit_range(x)
    after = detail(x)
    say("done", 1.0)
    return dict(rotation=float(x[0]), spacing=float(x[1]), tilt=float(x[2]),
                range_error=scene.range_error,
                range_error_before=range_error,
                before=before, after=after, stride=stride)


def _fmt(x, sc):
    return (f"roll {x[0]:8.3f} deg  spacing {x[1]:7.2f} mm  tilt {x[2]:+6.3f}"
            f" deg | planes {sc['planes']:.2f} mm, seam up {sc['up']:.2f} / "
            f"horizon {sc['horizon']:.2f} / down {sc['down']:.2f} mm")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("capture", help=".bin recorded by scanner_ui")
    ap.add_argument("--rotation", type=float, default=sp.LIDAR_ROTATION_DEG,
                    help="starting lidar roll, deg")
    ap.add_argument("--spacing", type=float, default=sp.EMITTER_SPACING_MM,
                    help="starting emitter spacing, mm")
    ap.add_argument("--tilt", type=float, default=0.0,
                    help="starting scan-plane tilt, deg")
    ap.add_argument("--no-reverse", action="store_true",
                    help="azimuth not reversed (see LIDAR_REVERSE)")
    ap.add_argument("--max-frames", type=int, default=CALIBRATION_MAX_FRAMES,
                    help="thin longer captures to about this many lidar "
                         "frames (default %(default)s); 0 uses every frame")
    a = ap.parse_args()

    cap = load(a.capture)
    print(f"{len(cap)} samples")
    last = [None]

    def progress(text, frac):
        stage = text.split(":")[0]
        if stage != last[0]:
            print(f"[{frac * 100:3.0f}%] {stage}")
            last[0] = stage

    r = calibrate(cap, a.rotation, a.spacing, a.tilt,
                  reverse=not a.no_reverse, range_error=sp.RANGE_ERROR,
                  progress=progress,
                  max_frames=a.max_frames)
    if r["stride"] > 1:
        print(f"(long sweep: used 1 of every {r['stride']} lidar frames; "
              "--max-frames 0 to use all)")
    print("start:  " + _fmt((a.rotation, a.spacing, a.tilt), r["before"]))
    print("fitted: " + _fmt((r["rotation"], r["spacing"], r["tilt"]),
                            r["after"]))
    print(f"\nMount geometry: Lidar roll {r['rotation']:.2f} deg, "
          f"Emitter spacing {r['spacing']:.1f} mm, "
          f"Scan-plane tilt {r['tilt']:.2f} deg")
    if r["range_error"] is not None:
        print("Range error model (scan_proto.RANGE_ERROR): "
              + ", ".join(f"{v:.4g}" for v in r["range_error"])
              + f"  -- about {sp.range_error_at(r['range_error'], 3000):.1f} "
              "mm at 3 m")


if __name__ == "__main__":
    main()
