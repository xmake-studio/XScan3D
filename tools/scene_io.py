#!/usr/bin/env python3
"""Read and write a whole scene as one portable `.scene` file.

A `.scene` is a zip archive, so a project is a single file to copy, share or
back up rather than a scatter of point clouds plus a note about how they line
up. Inside:

    manifest.json     scene name/units, the group table, and per-scan metadata
                      (id, name, source, group, visibility, 4x4 pose, counts)
    scans/<id>.npz    that scan's points -- xyz, the range channel, and vertex
                      colours if it had any

The points are stored as .npz rather than .ply on purpose: it is lossless (the
range channel and float positions survive exactly, where a .ply would quantise
the colours and force the range to be re-derived) and it is the format numpy
loads fastest. The pose lives only in the manifest, so re-aligning a scan and
saving again rewrites a few numbers, not the cloud.

The write is atomic -- a temp file swapped into place -- so a crash mid-save
cannot leave a half-written project where a working one used to be.
"""

import io
import json
import os
import zipfile

import numpy as np

from scene_model import Scan, Scene, identity

MANIFEST = "manifest.json"
FORMAT_VERSION = 1


def _scan_entry(scan_id):
    return f"scans/{scan_id}.npz"


def save_scene(path, scene):
    """Write `scene` to `path` (a `.scene` zip). Marks the scene clean."""
    manifest = {
        "format": FORMAT_VERSION,
        "name": scene.name,
        "units": scene.units,
        "groups": [{"id": gid, "name": g["name"]}
                   for gid, g in scene.groups.items()],
        "scans": [],
    }
    for s in scene.scans:
        # The live sweep is transient state, not part of the saved project;
        # skip it so a save taken mid-scan does not bake in a half cloud.
        if s.generating:
            continue
        manifest["scans"].append({
            "id": s.id,
            "name": s.name,
            "source": s.source,
            "group": s.group,
            "visible": s.visible,
            "color": None if s.color is None else list(s.color),
            "points": int(s.n_points),
            "has_rgb": s.rgb is not None,
            "T": np.asarray(s.T, float).tolist(),
            "data": _scan_entry(s.id),
        })

    tmp = path + ".tmp"
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr(MANIFEST, json.dumps(manifest, indent=2))
        for s in scene.scans:
            if s.generating:
                continue
            buf = io.BytesIO()
            arrays = {"xyz": s.xyz.astype(np.float32),
                      "dist": s.dist.astype(np.float32)}
            if s.rgb is not None:
                arrays["rgb"] = s.rgb.astype(np.float32)
            np.savez_compressed(buf, **arrays)
            z.writestr(_scan_entry(s.id), buf.getvalue())
    os.replace(tmp, path)

    scene.path = path
    scene.dirty = False
    return path


def load_scene(path):
    """Read a `.scene` zip back into a fresh Scene."""
    with zipfile.ZipFile(path, "r") as z:
        manifest = json.loads(z.read(MANIFEST).decode("utf-8"))

        scene = Scene(name=manifest.get("name", "Untitled scene"),
                      units=manifest.get("units", "mm"))
        for g in manifest.get("groups", []):
            scene.groups[g["id"]] = {"name": g["name"]}

        for meta in manifest.get("scans", []):
            entry = meta.get("data") or _scan_entry(meta["id"])
            with z.open(entry) as fh:
                data = np.load(io.BytesIO(fh.read()))
                xyz = data["xyz"]
                dist = data["dist"] if "dist" in data else None
                rgb = data["rgb"] if "rgb" in data else None
            color = meta.get("color")
            scan = Scan(
                xyz, dist,
                name=meta.get("name", "scan"),
                rgb=rgb,
                T=np.asarray(meta.get("T", identity()), float),
                source=meta.get("source", "ply"),
                scan_id=meta["id"],
                visible=meta.get("visible", True),
                group=meta.get("group"),
                color=tuple(color) if color else None,
            )
            scene.scans.append(scan)

    scene.path = path
    scene.dirty = False
    return scene
