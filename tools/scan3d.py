#!/usr/bin/env python3
"""Capture and view the 3D cloud built from the tilting 2D lidar.

    python scan3d.py --port COM7 --start --record scan3d.bin   # run a sweep
    python scan3d.py --replay scan3d.bin                       # view it
    python scan3d.py --replay scan3d.bin --export cloud.ply
    python scan3d.py --port COM7 --start --live                # watch it build

Rendering goes through OpenGL (pyqtgraph), not matplotlib. matplotlib's 3D
scatter re-projects and depth-sorts every point in Python on each redraw, which
is what made the old viewer crawl; pyqtgraph uploads the points to the GPU once
and only sends a new camera matrix per frame, so a full scan orbits smoothly.

Needs: numpy, pyqtgraph, PyQt5 (or PySide6), pyserial for --port.
"""

import argparse
import sys
import threading

import numpy as np

import scan_proto as sp

# Colour ramp for the cloud. Blue (near) -> green -> red (far), computed with
# numpy so the viewer does not need matplotlib at all.
def colorize(v, alpha=1.0):
    t = np.asarray(v, dtype=np.float64).ravel()
    if t.size == 0:
        return np.empty((0, 4), dtype=np.float32)
    # An aggressive voxel size or max range can leave nothing to scale against,
    # and a dropped sample can leave a NaN; percentile raises on the first and
    # poisons the whole ramp with the second, so pick the span off the finite
    # values only and fall back to a flat mid-ramp when there are none.
    finite = t[np.isfinite(t)]
    if finite.size:
        lo, hi = np.percentile(finite, 2), np.percentile(finite, 98)
    else:
        lo = hi = 0.0
    t = np.nan_to_num((t - lo) / max(hi - lo, 1e-9), nan=0.5,
                      posinf=1.0, neginf=0.0)
    t = np.clip(t, 0.0, 1.0)
    rgba = np.empty((t.size, 4), dtype=np.float32)
    rgba[:, 0] = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0, 1)
    rgba[:, 1] = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0, 1)
    rgba[:, 2] = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0, 1)
    rgba[:, 3] = alpha
    return rgba


def export_ply(path, xyz, dist):
    """Binary PLY. The old ASCII writer formatted every vertex in a Python
    loop, which took longer than the scan itself on a big cloud."""
    rgb = (colorize(dist)[:, :3] * 255).astype(np.uint8)
    vert = np.empty(xyz.shape[0], dtype=[("x", "<f4"), ("y", "<f4"),
                                         ("z", "<f4"), ("red", "u1"),
                                         ("green", "u1"), ("blue", "u1")])
    vert["x"], vert["y"], vert["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    vert["red"], vert["green"], vert["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as fh:
        fh.write(b"ply\nformat binary_little_endian 1.0\n")
        fh.write(f"element vertex {xyz.shape[0]}\n".encode())
        fh.write(b"property float x\nproperty float y\nproperty float z\n")
        fh.write(b"property uchar red\nproperty uchar green\nproperty uchar blue\n")
        fh.write(b"end_header\n")
        fh.write(vert.tobytes())
    print(f"wrote {xyz.shape[0]} points -> {path}", file=sys.stderr)


_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def load_ply(path):
    """Read a PLY back in. Returns (xyz, rgb or None).

    Handles ASCII and little-endian binary with the properties in any order,
    so clouds that have been through MeshLab or CloudCompare still load.
    """
    with open(path, "rb") as fh:
        if fh.readline().strip() != b"ply":
            raise ValueError(f"{path}: not a PLY file")
        fmt = None
        count = 0
        props = []          # (name, numpy type) for the vertex element
        in_vertex = False
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f"{path}: header never ended")
            tok = line.split()
            if not tok:
                continue
            key = tok[0]
            if key == b"format":
                fmt = tok[1].decode()  # version token is tok[2], and ignorable
            elif key == b"element":
                in_vertex = tok[1] == b"vertex"
                if in_vertex:
                    count = int(tok[2])
            elif key == b"property" and in_vertex:
                if tok[1] == b"list":
                    raise ValueError(f"{path}: list properties on vertices")
                t = _PLY_TYPES.get(tok[1].decode())
                if t is None:
                    raise ValueError(f"{path}: unknown property type {tok[1]}")
                props.append((tok[2].decode(), t))
            elif key == b"end_header":
                break

        if not count:
            return np.zeros((0, 3), np.float32), None
        names = [n for n, _ in props]
        for axis in ("x", "y", "z"):
            if axis not in names:
                raise ValueError(f"{path}: vertex element has no '{axis}'")

        if fmt == "ascii":
            rows = np.loadtxt(fh, max_rows=count, ndmin=2)
            cols = {n: rows[:, i] for i, n in enumerate(names)}
        elif fmt in ("binary_little_endian", "binary_big_endian"):
            order = "<" if fmt.endswith("little_endian") else ">"
            dt = np.dtype([(n, order + t) for n, t in props])
            raw = fh.read(dt.itemsize * count)
            if len(raw) < dt.itemsize * count:
                raise ValueError(f"{path}: truncated, got {len(raw)} of "
                                 f"{dt.itemsize * count} vertex bytes")
            rows = np.frombuffer(raw, dtype=dt, count=count)
            cols = {n: rows[n] for n in names}
        else:
            raise ValueError(f"{path}: unsupported PLY format '{fmt}'")

    xyz = np.stack([cols["x"], cols["y"], cols["z"]], axis=-1).astype(np.float32)
    rgb = None
    if all(c in cols for c in ("red", "green", "blue")):
        rgb = np.stack([cols["red"], cols["green"], cols["blue"]],
                       axis=-1).astype(np.float32) / 255.0
    return xyz, rgb


def _import_gl():
    try:
        import pyqtgraph as pg
        import pyqtgraph.opengl as gl
        return pg, gl
    except ImportError:
        sys.exit("need pyqtgraph + a Qt binding for the viewer:\n"
                 "    pip install pyqtgraph PyQt5 PyOpenGL\n"
                 "(or use --export and open the .ply in MeshLab/CloudCompare)")


def make_view(pg, gl, title):
    app = pg.mkQApp(title)
    w = gl.GLViewWidget()
    w.setWindowTitle(title)
    w.resize(1200, 900)
    w.setCameraPosition(distance=4000, elevation=20, azimuth=45)

    grid = gl.GLGridItem()
    grid.setSize(8000, 8000)
    grid.setSpacing(500, 500)
    grid.translate(0, 0, -1000)
    w.addItem(grid)

    # The sensor sits at the origin; without a marker there it is easy to lose
    # track of scale in an empty room.
    origin = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=12,
                                  color=(1, 1, 1, 1))
    w.addItem(origin)

    scatter = gl.GLScatterPlotItem(pos=np.zeros((0, 3)), size=2.0,
                                   pxMode=True)
    w.addItem(scatter)
    w.show()
    return app, w, scatter


def view_static(xyz, dist, point_size, color_by):
    pg, gl = _import_gl()
    app, w, scatter = make_view(pg, gl, f"lidar 3D - {xyz.shape[0]} points")
    v = xyz[:, 2] if color_by == "height" else dist
    scatter.setData(pos=xyz.astype(np.float32), color=colorize(v),
                    size=point_size)
    # Frame the cloud rather than trusting the default camera distance.
    if xyz.size:
        w.setCameraPosition(pos=pg.Vector(*xyz.mean(axis=0)),
                            distance=float(np.percentile(
                                np.linalg.norm(xyz, axis=1), 95)) * 2.5)
    pg.exec() if hasattr(pg, "exec") else app.exec_()


def view_live(source, args):
    """Render while the sweep is still running.

    The serial reader runs on its own thread so a slow redraw can never stall
    the USB pipe -- if the host stops draining it the firmware starts dropping
    samples, and the sweep is unrepeatable.
    """
    pg, gl = _import_gl()
    app, w, scatter = make_view(pg, gl, "lidar 3D - live")

    cap = sp.Capture()
    done = threading.Event()

    def reader():
        try:
            for _ in sp.parse(source, capture=cap):
                pass
        finally:
            done.set()

    threading.Thread(target=reader, daemon=True).start()

    state = {"n": 0}

    def refresh():
        n = len(cap)
        if n == state["n"] or n < 4:
            return
        state["n"] = n
        try:
            xyz, dist, _ = sp.build_cloud(cap, sweep_only=not args.all,
                                          max_range=args.max_range,
                                          transpose=args.transpose,
                                          from_stepper=args.from_stepper,
                                          tilt_axis=args.tilt_axis,
                                          beam_offset=args.beam_offset,
                                          lidar_rotation=args.lidar_rotation,
                                          lidar_reverse=args.lidar_reverse,
                                          flip_upright=args.flip_upright,
                                          tilt_offset=args.tilt_offset)
        except SystemExit:
            return  # nothing usable yet; the sweep may not have started
        if args.voxel:
            xyz, (dist,) = sp.voxel_downsample(xyz, [dist], args.voxel)
        v = xyz[:, 2] if args.color_by == "height" else dist
        scatter.setData(pos=xyz.astype(np.float32), color=colorize(v),
                        size=args.point_size)
        w.setWindowTitle(f"lidar 3D - live - {xyz.shape[0]} points")

    timer = pg.QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(200)  # 5 Hz is plenty; the sweep takes 30 s

    pg.exec() if hasattr(pg, "exec") else app.exec_()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="serial port, e.g. COM7 or /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--replay", help="a .bin recorded earlier")
    p.add_argument("--record", help="write the raw stream to this file")
    p.add_argument("--seconds", type=float,
                   help="stop capturing after N seconds (default: sweep + 10)")
    p.add_argument("--start", action="store_true",
                   help="send 's' to begin a sweep as soon as the port opens")
    p.add_argument("--live", action="store_true",
                   help="show the cloud building up during the sweep")
    p.add_argument("--export", help="write a binary .ply instead of viewing")
    p.add_argument("--max-range", type=float, help="discard points beyond N mm")
    p.add_argument("--voxel", type=float, default=0.0,
                   help="keep one point per N-mm cube (e.g. 20)")
    p.add_argument("--point-size", type=float, default=2.0)
    p.add_argument("--color-by", choices=("height", "distance"),
                   default="height")
    p.add_argument("--all", action="store_true",
                   help="keep parking/idle frames too, not just the sweep")
    p.add_argument("--transpose", action="store_true", default=None,
                   help="force the world->sensor quaternion convention")
    p.add_argument("--from-stepper", action="store_true",
                   help="build the tilt from the step count instead of the "
                        "AHRS: no yaw drift, but no slop compensation either")
    p.add_argument("--tilt-axis", default="y", choices=("x", "y"),
                   help="which sensor axis the platform tilts about, for "
                        "--from-stepper (default y)")
    p.add_argument("--beam-offset", type=float, default=sp.BEAM_OFFSET_MM,
                   help="mm from the rotation axis up to the scan plane "
                        f"(default {sp.BEAM_OFFSET_MM:.0f})")
    p.add_argument("--lidar-rotation", type=float,
                   default=sp.LIDAR_ROTATION_DEG,
                   help="degrees clockwise the lidar is mounted relative to "
                        "the IMU's frame; wrong values hinge the sweep about "
                        f"the wrong axis (default {sp.LIDAR_ROTATION_DEG:.0f})")
    p.add_argument("--tilt-offset", default="auto",
                   help="degrees added to the commanded platform angle for "
                        "--from-stepper, correcting a home position that was "
                        "not level; 'auto' (the default) measures it against "
                        "the IMU, '0' disables it")
    p.add_argument("--no-lidar-reverse", dest="lidar_reverse",
                   action="store_false", default=sp.LIDAR_REVERSE,
                   help="the lidar's azimuth is reversed by default; pass this "
                        "if the cloud comes out mirrored (this convention "
                        "cannot be inferred from a capture -- see "
                        "LIDAR_REVERSE in scan_proto.py)")
    p.add_argument("--no-flip-upright", dest="flip_upright",
                   action="store_false", default=sp.FLIP_UPRIGHT,
                   help="scans come out upside down and are turned 180 deg "
                        "about world X by default; pass this to get the raw "
                        "orientation back")
    args = p.parse_args()
    args.tilt_axis = (1.0, 0.0, 0.0) if args.tilt_axis == "x" else (0.0, 1.0, 0.0)
    # Keep "auto" as the string build_cloud recognises; anything else must be
    # a number, and a typo should say so here rather than silently meaning 0.
    if args.tilt_offset != "auto":
        try:
            args.tilt_offset = float(args.tilt_offset)
        except ValueError:
            p.error(f"--tilt-offset: expected a number or 'auto', "
                    f"got {args.tilt_offset!r}")

    if not args.port and not args.replay:
        p.error("need --port or --replay")

    if args.replay:
        source = sp.file_source(args.replay)
    else:
        seconds = args.seconds
        if seconds is None and args.start:
            seconds = 30.0 + 20.0  # SCAN_TIME plus parking and settling
        source = sp.serial_source(args.port, args.baud, seconds, args.record,
                                  start=args.start)

    if args.live and not args.replay:
        view_live(source, args)
        return

    cap = sp.Capture()
    for _ in sp.parse(source, capture=cap):
        pass
    print(f"{len(cap)} samples, {len(cap.telem)} telemetry records",
          file=sys.stderr)

    xyz, dist, platform = sp.build_cloud(cap, sweep_only=not args.all,
                                         max_range=args.max_range,
                                         transpose=args.transpose,
                                         from_stepper=args.from_stepper,
                                         tilt_axis=args.tilt_axis,
                                         beam_offset=args.beam_offset,
                                         lidar_rotation=args.lidar_rotation,
                                         lidar_reverse=args.lidar_reverse,
                                         flip_upright=args.flip_upright,
                                         tilt_offset=args.tilt_offset)
    print(f"{xyz.shape[0]} points over "
          f"{platform.min():+.1f}..{platform.max():+.1f} deg of platform tilt",
          file=sys.stderr)

    if args.voxel:
        before = xyz.shape[0]
        xyz, (dist,) = sp.voxel_downsample(xyz, [dist], args.voxel)
        print(f"voxel {args.voxel} mm: {before} -> {xyz.shape[0]} points",
              file=sys.stderr)

    if args.export:
        export_ply(args.export, xyz, dist)
    else:
        view_static(xyz, dist, args.point_size, args.color_by)


if __name__ == "__main__":
    main()
