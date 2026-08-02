#!/usr/bin/env python3
"""The 3D viewport: an orbit/fly camera, and a renderer that keeps it in step
with the scene graph.

`SceneView` is the camera -- moved here unchanged from the original UI, so orbit
and fly behave exactly as they did, layout-safe keys and touchpad gestures
included. `SceneRenderer` is new: it owns one scatter item per scan so a scan
can be shown, hidden, recoloured or highlighted on its own, and it is where the
colour philosophy lives -- the rules that make selected, generating, merged and
loose scans read differently at a glance.
"""

import sys
import time

import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import cloud_io
from widgets import (_enum, _key, STRONG_FOCUS, LEFT_BUTTON, MIDDLE_BUTTON,
                     SHIFT_MOD, CTRL_MOD, ALT_MOD, TRANSPARENT_FOR_MOUSE,
                     EV_KEY_PRESS, EV_KEY_RELEASE, EV_NATIVE_GESTURE,
                     EV_WINDOW_DEACTIVATE, ZOOM_GESTURE)
from workers import studio_shader

CAM_ORBIT, CAM_FPS = "orbit", "fps"

# Colour modes for the point cloud.
COLOR_OBJECT, COLOR_GROUP, COLOR_HEIGHT, COLOR_DISTANCE = (
    "object", "group", "height", "distance")

# A categorical palette for "colour by object/group": distinct, reasonably
# equal in brightness, and picked to stay apart on a dark background. Cycled if
# there are more scans than colours.
PALETTE = [
    (0.30, 0.69, 0.94), (0.98, 0.61, 0.20), (0.47, 0.82, 0.43),
    (0.93, 0.39, 0.55), (0.68, 0.55, 0.93), (0.40, 0.85, 0.80),
    (0.95, 0.85, 0.32), (0.60, 0.72, 0.30), (0.90, 0.55, 0.80),
    (0.55, 0.65, 0.75),
]

# Loose (un-merged) scans share this muted colour in group mode, so "not yet
# merged" reads as a state rather than as just another object colour.
LOOSE_COLOR = (0.62, 0.64, 0.68)
# The live sweep in progress. Same warm orange the old pending overlay used.
GENERATING_COLOR = (1.00, 0.52, 0.10)
# Selected scans are lifted toward white and drawn a touch larger, so selection
# is legible on top of any colour mode without hiding which object it is.
SELECT_TINT = (1.00, 1.00, 1.00)
SELECT_MIX = 0.45
SELECT_SIZE_BONUS = 1.5


def _vec3(v):
    """pyqtgraph Vector / QVector3D -> plain (3,) array."""
    return np.array([v.x(), v.y(), v.z()], dtype=float)


class SceneView(gl.GLViewWidget):
    """The 3D view, with a second camera style bolted on.

    Orbit mode is stock pyqtgraph. FPS mode reuses the same machinery: the
    camera position is a function of centre/azimuth/elevation, so looking around
    turns and then slides the centre so the eye stays put, and walking is a
    translation of the centre. One pose representation means switching modes
    never jumps the view.
    """

    FPS_DISTANCE = 100.0

    MOVES = {
        "fwd": (1, 0, 0), "back": (-1, 0, 0),
        "right": (0, 1, 0), "left": (0, -1, 0),
        "rise": (0, 0, 1), "sink": (0, 0, -1),
    }

    LAYOUT_SAFE_KEYS = {
        "Up": "fwd", "Down": "back", "Right": "right", "Left": "left",
        "Space": "rise", "Control": "sink", "Shift": "fast",
    }

    SCAN_ACTIONS = {
        "win32": {0x11: "fwd", 0x1F: "back", 0x20: "right", 0x1E: "left",
                  0x12: "rise", 0x10: "sink"},
        "darwin": {13: "fwd", 1: "back", 2: "right", 0: "left",
                   14: "rise", 12: "sink"},
        "linux": {25: "fwd", 39: "back", 40: "right", 38: "left",
                  26: "rise", 24: "sink"},
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._scan_actions = self.SCAN_ACTIONS.get(
            "linux" if sys.platform.startswith("linux") else sys.platform, {})
        self._key_actions = {
            _key(n): a for n, a in dict(
                self.LAYOUT_SAFE_KEYS,
                W="fwd", S="back", D="right", A="left", E="rise", Q="sink",
            ).items()
        }
        self.cam_mode = CAM_ORBIT
        self._orbit_distance = None
        self._held = set()
        self._toast = None
        self._toast_timer = None
        self.speed = 1500.0
        self.setFocusPolicy(STRONG_FOCUS)
        self._tick = QtCore.QTimer(self)
        self._tick.setInterval(16)
        self._tick.timeout.connect(self._fly)
        self._last_t = None

    # --- mode ---------------------------------------------------------------

    def set_cam_mode(self, mode):
        if mode == self.cam_mode:
            return
        if mode == CAM_FPS:
            eye = _vec3(self.cameraPosition())
            self._orbit_distance = self.opts["distance"]
            self.opts["distance"] = self.FPS_DISTANCE
            self._place(eye)
            self._last_t = time.monotonic()
            self._tick.start()
            QtWidgets.QApplication.instance().installEventFilter(self)
        else:
            self._tick.stop()
            QtWidgets.QApplication.instance().removeEventFilter(self)
            self._held.clear()
            if self._toast is not None:
                self._toast.hide()
            eye = _vec3(self.cameraPosition())
            self.opts["distance"] = self._orbit_distance or 4000.0
            self._place(eye)
        self.cam_mode = mode
        self.update()

    def setCameraPosition(self, pos=None, distance=None, elevation=None,
                          azimuth=None, rotation=None):
        """Framing, without letting it undo FPS mode."""
        if self.cam_mode != CAM_FPS:
            return super().setCameraPosition(pos, distance, elevation,
                                             azimuth, rotation)
        if rotation is not None:
            raise ValueError("cannot set rotation while flying")
        if elevation is not None:
            self.opts["elevation"] = elevation
        if azimuth is not None:
            self.opts["azimuth"] = azimuth
        if distance is not None:
            self._orbit_distance = distance
        target = _vec3(pos if pos is not None else self.opts["center"])
        self.opts["distance"] = self.FPS_DISTANCE
        self._place(target - self._forward() * (self._orbit_distance or 4000.0))
        self.update()

    def _forward(self):
        el = np.radians(self.opts["elevation"])
        az = np.radians(self.opts["azimuth"])
        return -np.array([np.cos(el) * np.cos(az),
                          np.cos(el) * np.sin(az),
                          np.sin(el)])

    def _place(self, eye):
        c = np.asarray(eye) + self._forward() * self.opts["distance"]
        self.opts["center"] = pg.Vector(*c)

    # --- looking ------------------------------------------------------------

    def _orbit_move(self, ev):
        pos = ev.position() if hasattr(ev, "position") else ev.localPos()
        if getattr(self, "mousePos", None) is None:
            self.mousePos = pos
        diff = pos - self.mousePos
        mods = ev.modifiers()
        panning = (ev.buttons() & MIDDLE_BUTTON
                   or (ev.buttons() & LEFT_BUTTON
                       and mods & (SHIFT_MOD | ALT_MOD)))
        if not panning:
            return super().mouseMoveEvent(ev)
        self.mousePos = pos
        self.pan(diff.x(), diff.y(), 0, relative="view")

    def mouseMoveEvent(self, ev):
        if self.cam_mode != CAM_FPS:
            return self._orbit_move(ev)
        pos = ev.position() if hasattr(ev, "position") else ev.localPos()
        if getattr(self, "mousePos", None) is None:
            self.mousePos = pos
        diff = pos - self.mousePos
        self.mousePos = pos
        eye = _vec3(self.cameraPosition())
        self.opts["azimuth"] -= diff.x() * 0.5
        self.opts["elevation"] = float(
            np.clip(self.opts["elevation"] + diff.y() * 0.5, -89.9, 89.9))
        self._place(eye)
        self.update()

    @staticmethod
    def _scroll_steps(ev):
        if not hasattr(ev, "angleDelta"):
            return 0.0, ev.delta() / 120.0, False
        pixels = ev.pixelDelta() if hasattr(ev, "pixelDelta") else None
        if pixels is not None and (pixels.x() or pixels.y()):
            return pixels.x() / 120.0, pixels.y() / 120.0, True
        angle = ev.angleDelta()
        return angle.x() / 120.0, angle.y() / 120.0, False

    def _set_speed(self, value):
        self.speed = float(np.clip(value, 50.0, 50000.0))
        self._flash(f"Fly speed  {self.speed / 1000.0:.2f} m/s")

    def _flash(self, text, ms=900):
        if self._toast is None:
            self._toast = QtWidgets.QLabel(self)
            self._toast.setStyleSheet(
                "background: rgba(0, 0, 0, 160); color: white;"
                "padding: 6px 10px; border-radius: 4px;")
            self._toast.setAttribute(TRANSPARENT_FOR_MOUSE, True)
            self._toast_timer = QtCore.QTimer(self)
            self._toast_timer.setSingleShot(True)
            self._toast_timer.timeout.connect(self._toast.hide)
        self._toast.setText(text)
        self._toast.adjustSize()
        self._toast.move((self.width() - self._toast.width()) // 2,
                         self.height() - self._toast.height() - 24)
        self._toast.show()
        self._toast_timer.start(ms)

    def _zoom(self, steps):
        self.opts["distance"] *= 0.999 ** (steps * 120.0)
        self.update()

    def wheelEvent(self, ev):
        dx, dy, precise = self._scroll_steps(ev)
        if self.cam_mode == CAM_FPS:
            self._set_speed(self.speed * 1.15 ** dy)
            ev.accept()
            return
        if ev.modifiers() & SHIFT_MOD:
            self.pan(dx * 40.0, dy * 40.0, 0, relative="view")
        elif precise and not ev.modifiers() & CTRL_MOD:
            self.orbit(-dx * 120.0, dy * 120.0)
        else:
            if dy:
                self._zoom(dy)
            if dx and not precise:
                self.pan(dx * 40.0, 0, 0, relative="view")
        ev.accept()

    def event(self, ev):
        if ev.type() == EV_NATIVE_GESTURE and ZOOM_GESTURE is not None:
            if ev.gestureType() == ZOOM_GESTURE:
                if self.cam_mode == CAM_FPS:
                    self._set_speed(self.speed * (1.0 + ev.value()))
                else:
                    self.opts["distance"] *= max(0.1, 1.0 - ev.value())
                    self.update()
                return True
        return super().event(ev)

    # --- walking ------------------------------------------------------------

    def eventFilter(self, obj, ev):
        t = ev.type()
        if t == EV_WINDOW_DEACTIVATE:
            self._held.clear()
            return False
        if t not in (EV_KEY_PRESS, EV_KEY_RELEASE):
            return False
        if not (self._pointer_over_view() or self.hasFocus()):
            return False
        action = self._action(ev)
        if action is None:
            return False
        if not ev.isAutoRepeat():
            if t == EV_KEY_PRESS:
                self._held.add(action)
            else:
                self._held.discard(action)
        return True

    def _action(self, ev):
        action = self._scan_actions.get(ev.nativeScanCode())
        if action is not None:
            return action
        return self._key_actions.get(ev.key())

    def _pointer_over_view(self):
        if not self.isVisible():
            return False
        return self.rect().contains(self.mapFromGlobal(QtGui.QCursor.pos()))

    def mousePressEvent(self, ev):
        self.setFocus()
        super().mousePressEvent(ev)

    def _fly(self):
        now = time.monotonic()
        dt, self._last_t = now - (self._last_t or now), now
        if not self._held:
            return
        fwd = self._forward()
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(fwd, up)
        n = np.linalg.norm(right)
        right = right / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])

        move = np.zeros(3)
        for action, (f, r, u) in self.MOVES.items():
            if action in self._held:
                move += f * fwd + r * right + u * up
        if not move.any():
            return
        fast = 4.0 if "fast" in self._held else 1.0
        step = move / np.linalg.norm(move) * self.speed * fast * dt
        self.opts["center"] = pg.Vector(*(_vec3(self.opts["center"]) + step))
        self.update()


class SceneRenderer:
    """Keeps the viewport's GL items in step with a Scene.

    One GLScatterPlotItem per scan: with only a handful of scans this is cheap,
    and it makes visibility a per-item flag and colour a per-item array, so
    hiding, selecting or recolouring one scan never touches the others. The
    single mesh item is shared, because surface mode is whole-scene.
    """

    def __init__(self, view):
        self.view = view
        self.color_mode = COLOR_HEIGHT
        self.point_size = 2.0
        self.show_surface = False
        # The scans whose alignment is awaiting a Keep/Discard verdict; painted
        # orange over the model so the candidate is unmistakable. A whole merged
        # group can move as one unit, so this is a set rather than a single id.
        self.pending_ids = set()
        self._items = {}          # scan_id -> GLScatterPlotItem
        self._scene = None

        grid = gl.GLGridItem()
        grid.setSize(10000, 10000)
        grid.setSpacing(500, 500)
        grid.translate(0, 0, -1500)
        self.grid = grid
        view.addItem(grid)

        self.origin = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=12,
                                           color=(1, 1, 1, 1))
        view.addItem(self.origin)

        self._shader = studio_shader()
        self.mesh_item = gl.GLMeshItem(smooth=True, shader=self._shader,
                                       drawEdges=False)
        self.mesh_item.setVisible(False)
        view.addItem(self.mesh_item)

    def set_scene(self, scene):
        self._scene = scene

    def set_grid_visible(self, on):
        self.grid.setVisible(on)

    # --- colouring ----------------------------------------------------------

    def _object_color(self, scan, index):
        if scan.color is not None:
            return tuple(scan.color)
        return PALETTE[index % len(PALETTE)]

    def _group_color(self, scan, group_index):
        if scan.group is None:
            return LOOSE_COLOR
        if scan.color is not None:
            return tuple(scan.color)
        return PALETTE[group_index % len(PALETTE)]

    def _base_rgba(self, scene):
        """rgba per scan id for the current colour mode, before highlights.

        Height and distance are ramped over the union of the visible scans so
        the ramp means the same thing across the whole scene rather than being
        renormalised per object; object and group are flat per-scan colours.
        """
        out = {}
        vis = scene.visible_scans()
        if self.color_mode in (COLOR_HEIGHT, COLOR_DISTANCE):
            values, spans = [], []
            for s in vis:
                if self.color_mode == COLOR_HEIGHT:
                    v = s.world_points()[:, 2] if s.n_points else \
                        np.zeros((0,), np.float32)
                else:
                    v = s.dist
                values.append(v)
                spans.append((s.id, v.shape[0]))
            allv = np.concatenate(values) if values else np.zeros((0,))
            rgba = cloud_io.colorize(allv)
            off = 0
            for sid, n in spans:
                out[sid] = rgba[off:off + n]
                off += n
            return out

        group_ids = scene.group_ids()
        for i, s in enumerate(scene.scans):
            if self.color_mode == COLOR_GROUP:
                gi = group_ids.index(s.group) if s.group in group_ids else 0
                rgb = self._group_color(s, gi)
            else:
                rgb = self._object_color(s, i)
            arr = np.empty((s.n_points, 4), np.float32)
            arr[:, :3] = rgb
            arr[:, 3] = 1.0
            out[s.id] = arr
        return out

    def _apply_highlights(self, scan, rgba, selected):
        if scan.generating or scan.id in self.pending_ids:
            rgba = np.empty((scan.n_points, 4), np.float32)
            rgba[:, :3] = GENERATING_COLOR
            rgba[:, 3] = 1.0
            return rgba
        if selected and rgba.shape[0]:
            rgba = rgba.copy()
            tint = np.array(SELECT_TINT, np.float32)
            rgba[:, :3] = rgba[:, :3] * (1 - SELECT_MIX) + tint * SELECT_MIX
        return rgba

    # --- syncing ------------------------------------------------------------

    def sync(self):
        """Reconcile the GL scatter items with the scene's scans."""
        scene = self._scene
        if scene is None:
            return

        alive = {s.id for s in scene.scans}
        for sid in list(self._items):
            if sid not in alive:
                self.view.removeItem(self._items.pop(sid))

        base = self._base_rgba(scene)
        for s in scene.scans:
            item = self._items.get(s.id)
            if item is None:
                item = gl.GLScatterPlotItem(pos=np.zeros((0, 3)), pxMode=True)
                self.view.addItem(item)
                self._items[s.id] = item

            show = s.visible and not self.show_surface
            item.setVisible(show)
            if not show:
                continue

            selected = scene.is_selected(s.id)
            rgba = self._apply_highlights(
                s, base.get(s.id, np.zeros((s.n_points, 4), np.float32)),
                selected)
            size = self.point_size + (SELECT_SIZE_BONUS if selected else 0.0)
            item.setData(pos=s.world_points(), color=rgba, size=size)

    def set_mesh(self, verts, faces, style):
        """Show a reconstructed surface. verts/faces None hides it."""
        if verts is None or faces is None or not len(faces):
            self.mesh_item.setVisible(False)
            return
        kwargs = {}
        if style == "height":
            kwargs["vertexColors"] = cloud_io.colorize(verts[:, 2])
        md = gl.MeshData(vertexes=verts, faces=faces, **kwargs)
        self.mesh_item.setMeshData(meshdata=md)
        self.mesh_item.setShader(self._shader)
        self.mesh_item.opts["drawFaces"] = style != "wireframe"
        self.mesh_item.opts["drawEdges"] = style == "wireframe"
        self.mesh_item.setColor((0.78, 0.80, 0.84, 1.0) if style == "shaded"
                                else (1.0, 1.0, 1.0, 1.0))
        self.mesh_item.setVisible(self.show_surface)
        self.mesh_item.update()

    def set_surface_mode(self, on):
        self.show_surface = on
        self.mesh_item.setVisible(on and self.mesh_item.opts.get(
            "meshdata") is not None)
        self.sync()
