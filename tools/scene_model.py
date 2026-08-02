#!/usr/bin/env python3
"""The scene graph the UI is built around.

A scan is the unit the whole application now revolves around: one point cloud
that came off the device, was loaded from a file, or was produced by aligning
others. Every scan is a first-class object -- it can be selected, hidden,
renamed, deleted, transformed and merged -- and the scene is just the ordered
collection of them plus the bookkeeping for which are grouped together.

This module is deliberately Qt-free and numpy-only: it is the data, and the
panels, the renderer and the .scene file all read and write it without any of
them owning it. Nothing here talks to the device or reconstructs geometry.

The one invariant worth stating up front: a merge never destroys its inputs.
Aligning scans writes each one a pose (`Scan.T`) and a shared `group` id; the
points stay in their own local frame. Un-merging is therefore just clearing the
group (and optionally the pose), which is what makes a bad alignment safe to
undo -- the reason the old bake-it-in map could not be trusted.
"""

import uuid

import numpy as np


def identity():
    """A fresh 4x4 identity pose."""
    return np.eye(4, dtype=float)


def apply_transform(T, xyz):
    """Place local points into the scene frame: (N,3) -> (N,3) float32.

    Kept inline rather than importing registration.apply so the data model
    stays numpy-only and free of the SLAM module; it is the same rigid
    transform either way.
    """
    xyz = np.asarray(xyz)
    if xyz.shape[0] == 0:
        return xyz.reshape(0, 3).astype(np.float32)
    T = np.asarray(T, dtype=float)
    return (xyz @ T[:3, :3].T + T[:3, 3]).astype(np.float32)


def _short_id():
    """A stable, collision-proof id that is still short enough to read in a
    manifest or a log line."""
    return uuid.uuid4().hex[:12]


class Scan:
    """One point cloud object in the scene.

    `xyz` is always in the scan's own local frame -- exactly as it was captured
    or loaded -- and `T` is the pose that carries it into the shared scene
    frame. Keeping those two separate is the whole point: an alignment only ever
    rewrites `T`, so the measured points are never resampled or lost.
    """

    __slots__ = ("id", "name", "xyz", "dist", "rgb", "T", "visible",
                 "source", "group", "generating", "raw", "color")

    def __init__(self, xyz, dist=None, *, name="scan", rgb=None, T=None,
                 source="device", scan_id=None, visible=True, group=None,
                 generating=False, raw=None, color=None):
        self.id = scan_id or _short_id()
        self.name = name
        self.xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        # The range channel drives the distance colour ramp. Loaded clouds have
        # no such channel, so derive one from the radius, which is what the old
        # _load_ply path did too.
        if dist is None:
            self.dist = np.linalg.norm(self.xyz, axis=1).astype(np.float32)
        else:
            self.dist = np.asarray(dist, dtype=np.float32).ravel()
        self.rgb = None if rgb is None else np.asarray(rgb, dtype=np.float32)
        self.T = identity() if T is None else np.asarray(T, dtype=float).copy()
        self.visible = bool(visible)
        self.source = source
        self.group = group
        # True only for the live sweep still being built; the renderer paints it
        # distinctly and the outliner badges it.
        self.generating = bool(generating)
        # Optional raw sample block for a device scan, kept so mount-geometry
        # changes can rebuild xyz. Not persisted to .scene.
        self.raw = raw
        # Optional per-object colour override (r,g,b in 0..1); None = follow the
        # view's colour mode.
        self.color = color

    @property
    def n_points(self):
        return self.xyz.shape[0]

    def world_points(self):
        """This scan's points in the scene frame."""
        return apply_transform(self.T, self.xyz)

    def bounds(self):
        """(min_xyz, max_xyz) in the scene frame, or None if empty."""
        if not self.n_points:
            return None
        w = self.world_points()
        return w.min(axis=0), w.max(axis=0)


class Scene:
    """The ordered collection of scans plus grouping and selection state.

    Selection is an ordered list of ids; the last one is the *active* scan, the
    one the Properties panel edits. Group membership lives on each scan
    (`Scan.group`) and the `groups` table only carries the group's display name,
    so there is a single source of truth for who is in a group.
    """

    def __init__(self, name="Untitled scene", units="mm"):
        self.name = name
        self.units = units
        self.scans = []
        self.groups = {}          # group_id -> {"name": str}
        self.selection = []       # ordered scan ids; last = active
        self.dirty = False        # unsaved changes since last save/load
        self.path = None          # last .scene path, if any

    # --- membership ---------------------------------------------------------

    def add(self, scan, select=True):
        self.scans.append(scan)
        self.dirty = True
        if select:
            self.set_selection([scan.id])
        return scan

    def remove(self, scan_id):
        scan = self.get(scan_id)
        if scan is None:
            return None
        self.scans.remove(scan)
        if scan_id in self.selection:
            self.selection.remove(scan_id)
        # A group that has just lost its last member is meaningless; drop it so
        # the outliner does not keep an empty parent node around.
        if scan.group and not self.group_members(scan.group):
            self.groups.pop(scan.group, None)
        self.dirty = True
        return scan

    def clear(self):
        self.scans.clear()
        self.groups.clear()
        self.selection.clear()
        self.dirty = True

    def get(self, scan_id):
        for s in self.scans:
            if s.id == scan_id:
                return s
        return None

    def unique_name(self, base):
        """`base`, or `base 2`, `base 3`... so no two scans read the same."""
        existing = {s.name for s in self.scans}
        if base not in existing:
            return base
        i = 2
        while f"{base} {i}" in existing:
            i += 1
        return f"{base} {i}"

    # --- selection ----------------------------------------------------------

    def set_selection(self, ids):
        ids = [i for i in ids if self.get(i) is not None]
        # De-duplicate while preserving order, so "active = last" is well
        # defined even if a caller passes a repeat.
        seen, out = set(), []
        for i in ids:
            if i not in seen:
                seen.add(i)
                out.append(i)
        self.selection = out

    def selected(self):
        return [self.get(i) for i in self.selection if self.get(i) is not None]

    def active(self):
        for i in reversed(self.selection):
            s = self.get(i)
            if s is not None:
                return s
        return None

    def is_selected(self, scan_id):
        return scan_id in self.selection

    # --- visibility / geometry ---------------------------------------------

    def visible_scans(self):
        return [s for s in self.scans if s.visible]

    def visible_world(self):
        """(xyz, dist) union of every visible scan, in the scene frame.

        This is what whole-scene operations -- surface reconstruction, framing,
        Save-visible -- act on. Returns (empty, empty) when nothing is shown.
        """
        parts, dists = [], []
        for s in self.visible_scans():
            if s.n_points:
                parts.append(s.world_points())
                dists.append(s.dist)
        if not parts:
            return (np.zeros((0, 3), np.float32), np.zeros((0,), np.float32))
        return np.vstack(parts), np.concatenate(dists)

    def bounds(self, only_visible=True):
        """(min, max) over the chosen scans in the scene frame, or None."""
        src = self.visible_scans() if only_visible else self.scans
        mins, maxs = [], []
        for s in src:
            b = s.bounds()
            if b is not None:
                mins.append(b[0])
                maxs.append(b[1])
        if not mins:
            return None
        return np.min(mins, axis=0), np.max(maxs, axis=0)

    # --- groups -------------------------------------------------------------

    def new_group(self, name=None):
        gid = _short_id()
        n = len([g for g in self.groups]) + 1
        self.groups[gid] = {"name": name or f"Merged {n}"}
        self.dirty = True
        return gid

    def group_members(self, gid):
        return [s for s in self.scans if s.group == gid]

    def group_visible(self, gid):
        """Whether a group is shown: True (all members shown), False (all
        hidden), or None (mixed -- some shown, some hidden)."""
        members = self.group_members(gid)
        if not members:
            return True
        shown = [s.visible for s in members]
        if all(shown):
            return True
        if not any(shown):
            return False
        return None

    def set_group_visible(self, gid, visible):
        """Show or hide every member of a group in one go."""
        for s in self.group_members(gid):
            s.visible = bool(visible)
        self.dirty = True

    def group_name(self, gid):
        g = self.groups.get(gid)
        return g["name"] if g else gid

    def assign_group(self, scan_ids, gid):
        for i in scan_ids:
            s = self.get(i)
            if s is not None:
                s.group = gid
        self.dirty = True

    def ungroup(self, scan_ids, reset_pose=False):
        """Take scans out of their group. Optionally reset their pose too.

        The pose is left in place by default: after a merge the aligned pose is
        usually what you want to keep, and only the grouping is being undone.
        Resetting is offered for the case where the merge itself was wrong and
        the scan should go back to sitting in its own frame.
        """
        touched = set()
        for i in scan_ids:
            s = self.get(i)
            if s is None:
                continue
            touched.add(s.group)
            s.group = None
            if reset_pose:
                s.T = identity()
        for gid in touched:
            if not gid:
                continue
            members = self.group_members(gid)
            # A group of one is meaningless: dissolve it so the lone survivor
            # goes back to being a loose scan rather than a one-member merge.
            if len(members) < 2:
                for s in members:
                    s.group = None
                self.groups.pop(gid, None)
        self.dirty = True

    def group_union(self, gid):
        """(xyz, dist) union of one group's scans, in the scene frame."""
        parts, dists = [], []
        for s in self.group_members(gid):
            if s.n_points:
                parts.append(s.world_points())
                dists.append(s.dist)
        if not parts:
            return (np.zeros((0, 3), np.float32), np.zeros((0,), np.float32))
        return np.vstack(parts), np.concatenate(dists)

    # --- ordering helpers for the outliner ----------------------------------

    def loose_scans(self):
        return [s for s in self.scans if not s.group]

    def group_ids(self):
        """Group ids in first-appearance order of their members."""
        seen, out = set(), []
        for s in self.scans:
            if s.group and s.group not in seen:
                seen.add(s.group)
                out.append(s.group)
        return out
