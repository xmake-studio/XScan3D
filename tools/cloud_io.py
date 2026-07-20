#!/usr/bin/env python3
"""Colouring and PLY read/write for point clouds and meshes.

Split out of the old command-line viewer so the UI keeps the exact ramp and the
exact file format the older captures were written with. Nothing here talks to
the device or knows anything about the scanner's geometry.
"""

import sys

import numpy as np


# Colour ramp for the cloud. Blue (near) -> green -> red (far), computed with
# numpy so nothing here needs matplotlib.
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
