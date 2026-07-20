#!/usr/bin/env python3
"""Build and render a 3D point cloud from the 2D lidar's scans.

The sensor sweeps a single plane, so there is no elevation channel in the data.
This stacks consecutive revolutions along Z as *time*, which gives a genuine 3D
cloud: a static scene reads as straight vertical walls, while anything that
moved (or a sensor that drifted) bends visibly along Z.

    python lidar_3d.py --replay scan.bin                 # interactive view
    python lidar_3d.py --replay scan.bin --export out.ply
    python lidar_3d.py --replay scan.bin --flat          # collapse Z, one scan

If you later mount the unit on a tilt stage, replace revolution_z() with the
real elevation and the rest of the pipeline is unchanged.

Needs: numpy, matplotlib (and pyserial only for --port).
"""

import argparse
import sys

import numpy as np

import lidar_viz as lv

# Z spacing between successive revolutions, in millimetres. Purely cosmetic:
# it sets how stretched the time axis looks next to the X/Y metric scale.
Z_PER_REVOLUTION = 40.0


def revolution_index(angle):
    """Number each point's revolution by detecting the 360 -> 0 wrap."""
    wrapped = np.diff(angle) < -180.0
    return np.concatenate([[0], np.cumsum(wrapped)])


def build_cloud(frames, flat=False, max_range=None):
    a, d, _ = lv.spread_points(frames)
    if a.size == 0:
        sys.exit("no usable frames - check the capture")

    rev = revolution_index(a)

    good = ~np.isnan(d)
    if max_range is not None:
        good &= d <= max_range
    a, d, rev = a[good], d[good], rev[good]

    th = np.radians(a)
    x = d * np.sin(th)   # 0 deg = +Y, clockwise, matching the polar view
    y = d * np.cos(th)
    z = np.zeros_like(d) if flat else rev * Z_PER_REVOLUTION
    return x, y, z, d, rev


def export_ply(path, x, y, z, d):
    """ASCII PLY with a distance-derived colour, readable by MeshLab etc."""
    t = (d - d.min()) / max(float(np.ptp(d)), 1e-9)  # np.ptp: ndarray.ptp is gone in numpy 2
    r = (255 * t).astype(np.uint8)
    g = (255 * (1.0 - np.abs(t - 0.5) * 2.0)).astype(np.uint8)
    b = (255 * (1.0 - t)).astype(np.uint8)
    with open(path, "w") as fh:
        fh.write("ply\nformat ascii 1.0\n")
        fh.write(f"element vertex {x.size}\n")
        fh.write("property float x\nproperty float y\nproperty float z\n")
        fh.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        fh.write("end_header\n")
        for i in range(x.size):
            fh.write(f"{x[i]:.1f} {y[i]:.1f} {z[i]:.1f} {r[i]} {g[i]} {b[i]}\n")
    print(f"wrote {x.size} points -> {path}", file=sys.stderr)


def render(x, y, z, d, rev, flat):
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(x, y, z, c=d, cmap="viridis", s=1.5, depthshade=False)
    ax.scatter([0], [0], [z.min()], c="red", s=60, marker="^")  # the lidar

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("scan (flattened)" if flat else "time (revolutions)")
    ax.set_title(f"{x.size} points, {rev.max() + 1} revolutions")

    # Equal X/Y scale so the room is not visually sheared.
    r = max(np.abs(x).max(), np.abs(y).max())
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)
    plt.show()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--replay", help="recorded .bin from lidar_viz.py --record")
    p.add_argument("--port", help="capture live instead of replaying")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--seconds", type=float, default=20.0,
                   help="capture duration when using --port")
    p.add_argument("--flat", action="store_true",
                   help="collapse every revolution onto one plane")
    p.add_argument("--max-range", type=float,
                   help="discard points beyond this many mm")
    p.add_argument("--export", help="write the cloud to an ASCII .ply")
    args = p.parse_args()

    if not args.replay and not args.port:
        p.error("need --replay or --port")

    if args.replay:
        src = lv.file_source(args.replay)
    else:
        src = lv.serial_source(args.port, args.baud, args.seconds, None)

    frames = list(lv.iter_frames(src))
    print(f"{len(frames)} frames", file=sys.stderr)

    x, y, z, d, rev = build_cloud(frames, args.flat, args.max_range)
    print(f"{x.size} points over {rev.max() + 1} revolutions", file=sys.stderr)

    if args.export:
        export_ply(args.export, x, y, z, d)
    else:
        render(x, y, z, d, rev, args.flat)


if __name__ == "__main__":
    main()
