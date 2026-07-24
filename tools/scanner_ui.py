#!/usr/bin/env python3
"""Single-window front end for the 3D lidar scanner.

    python scanner_ui.py

Connect to the device, set the sweep, run it, watch the cloud build, then save
or reload it as .ply. This is the whole host side; the only other tool is
serial_probe.py, for the case where this one cannot get the port to talk at all.

The View panel's Mode dropdown switches between the raw point cloud and a
reconstructed triangle surface; the settings below it follow the mode.

Needs: pyqtgraph, PyQt5, pyserial, numpy.
Optional: pymeshlab, for Poisson and ball-pivoting surface reconstruction
(open3d is used instead if that is what the machine has).
"""

import json
import os
import queue
import sys
import time

import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import scan_proto as sp
import cloud_io
import meshing
import registration as reg

BAUD = 115200

# Panel state lives next to the source and travels with the repo, so the mount
# geometry dialled in on one machine is the geometry every checkout starts with.
SETTINGS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "ui_settings.json")

# Scans belong to the project, not to whatever directory the UI happened to be
# launched from, so both the save dialogs and the autosaves anchor here.
SCANS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scans")


def scan_path(name, sub=None):
    """Absolute path for `name` under scans/ (or scans/<sub>/), dir created.

    The directory is made on demand rather than at import: a checkout without
    scans/ should still get one the first time something is written.
    """
    d = os.path.join(SCANS_DIR, sub) if sub else SCANS_DIR
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        # Unwritable location: fall back to the working directory so a save can
        # still happen, rather than failing before the dialog even opens.
        return os.path.abspath(name)
    return os.path.join(d, name)


class FileSettings:
    """QSettings-shaped store backed by one JSON file.

    Same value()/setValue() surface QSettings had, so the panel code did not
    have to change, but the file is versioned instead of living in the
    registry (Windows) or ~/.config (Linux) where it could not be shared.
    Values keep their JSON types, so the string round-tripping QSettings forced
    on bools no longer happens -- the readers still tolerate it for old files.
    """

    def __init__(self, path):
        self.path = path
        self._data = {}
        self._load()

    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError):
            # Missing or hand-broken: start empty rather than refuse to launch.
            # Defaults stand, and the next save rewrites the file.
            return
        if isinstance(data, dict):
            self._data = {k: v for k, v in data.items() if isinstance(k, str)}

    def value(self, key, default=None):
        return self._data.get(key, default)

    def setValue(self, key, val):
        self._data[key] = val

    def allKeys(self):
        return list(self._data)

    def sync(self):
        """Write the file. Atomic, so a crash mid-save cannot truncate it."""
        tmp = self.path + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(self._data, fh, indent=2, sort_keys=True)
                fh.write("\n")
            os.replace(tmp, self.path)
        except OSError:
            pass  # a read-only checkout should not take the session down

    def migrate_from_qsettings(self, qs):
        """One-time import of the pre-file location. No-op once the file exists.

        Without this the switch to a versioned file would silently reset every
        panel value, geometry included, which is exactly the failure the
        settings exist to prevent.
        """
        if self._data:
            return False
        for key in qs.allKeys():
            val = qs.value(key)
            if isinstance(val, str):
                # QSettings handed bools and numbers back as strings on some
                # backends; recover the real types on the way in.
                low = val.strip().lower()
                if low in ("true", "false"):
                    val = low == "true"
                else:
                    try:
                        val = int(val)
                    except ValueError:
                        try:
                            val = float(val)
                        except ValueError:
                            pass
            self._data[key] = val
        return bool(self._data)


def _enum(owner, *paths):
    """Look up a Qt enum member across bindings.

    PyQt6 scopes enum members under their type (Qt.ItemDataRole.ToolTipRole)
    where PyQt5 exposed them flat (Qt.ToolTipRole). Both bindings are commonly
    installed side by side and pyqtgraph picks whichever it finds first, so the
    UI should not care which one it got.
    """
    for path in paths:
        obj = owner
        for part in path.split("."):
            obj = getattr(obj, part, None)
            if obj is None:
                break
        if obj is not None:
            return obj
    raise AttributeError(f"none of {paths} on {owner}")


TOOLTIP_ROLE = _enum(QtCore.Qt, "ItemDataRole.ToolTipRole", "ToolTipRole")
ARROW_DOWN = _enum(QtCore.Qt, "ArrowType.DownArrow", "DownArrow")
ARROW_RIGHT = _enum(QtCore.Qt, "ArrowType.RightArrow", "RightArrow")
BESIDE_ICON = _enum(QtCore.Qt, "ToolButtonStyle.ToolButtonTextBesideIcon",
                    "ToolButtonTextBesideIcon")
NO_FRAME = _enum(QtWidgets.QFrame, "Shape.NoFrame", "NoFrame")
SCROLLBAR_OFF = _enum(QtCore.Qt, "ScrollBarPolicy.ScrollBarAlwaysOff",
                      "ScrollBarAlwaysOff")
SCROLLBAR_AUTO = _enum(QtCore.Qt, "ScrollBarPolicy.ScrollBarAsNeeded",
                       "ScrollBarAsNeeded")
STRONG_FOCUS = _enum(QtCore.Qt, "FocusPolicy.StrongFocus", "StrongFocus")
LEFT_BUTTON = _enum(QtCore.Qt, "MouseButton.LeftButton", "LeftButton")
MIDDLE_BUTTON = _enum(QtCore.Qt, "MouseButton.MiddleButton", "MiddleButton")
SHIFT_MOD = _enum(QtCore.Qt, "KeyboardModifier.ShiftModifier", "ShiftModifier")
CTRL_MOD = _enum(QtCore.Qt, "KeyboardModifier.ControlModifier",
                 "ControlModifier")
ALT_MOD = _enum(QtCore.Qt, "KeyboardModifier.AltModifier", "AltModifier")
TRANSPARENT_FOR_MOUSE = _enum(
    QtCore.Qt, "WidgetAttribute.WA_TransparentForMouseEvents",
    "WA_TransparentForMouseEvents")
EV_KEY_PRESS = _enum(QtCore.QEvent, "Type.KeyPress", "KeyPress")
EV_KEY_RELEASE = _enum(QtCore.QEvent, "Type.KeyRelease", "KeyRelease")
EV_WHEEL = _enum(QtCore.QEvent, "Type.Wheel", "Wheel")
EV_NATIVE_GESTURE = _enum(QtCore.QEvent, "Type.NativeGesture", "NativeGesture")
EV_WINDOW_DEACTIVATE = _enum(QtCore.QEvent, "Type.WindowDeactivate",
                             "WindowDeactivate")
# Pinch, as Windows precision touchpads and macOS trackpads report it. Older
# bindings predate the member, hence the tolerant lookup.
try:
    ZOOM_GESTURE = _enum(QtCore.Qt, "NativeGestureType.ZoomNativeGesture",
                         "ZoomNativeGesture")
except AttributeError:  # pragma: no cover - binding without native gestures
    ZOOM_GESTURE = None


def _key(name):
    return _enum(QtCore.Qt, f"Key.Key_{name}", f"Key_{name}")


# --- Scroll-safe value widgets ----------------------------------------------

class _NoWheelEdits(QtCore.QObject):
    """Stops the wheel from editing the widget it happens to be hovering.

    Qt's default is that a spin box, slider or combo under the pointer eats the
    wheel and changes its value. In a panel this tall that is a trap: scrolling
    down to the meshing controls drags the pointer across half a dozen editable
    widgets, and any of them silently absorbing a notch retunes a scan
    parameter -- a change that shows up later as a worse cloud with no obvious
    cause. Keyboard, arrows and typing still edit; the wheel simply stops being
    an editing gesture.

    The event is not merely swallowed, because that would make the panel refuse
    to scroll wherever a control lies under the pointer -- which is most of it.
    It is forwarded to the enclosing scroll area instead, so the wheel always
    means the one thing it should mean here: scroll the panel.
    """

    def eventFilter(self, obj, ev):
        if ev.type() != EV_WHEEL:
            return False
        area = obj.parent()
        while area is not None and not isinstance(area, QtWidgets.QAbstractScrollArea):
            area = area.parent()
        if area is not None:
            QtWidgets.QApplication.sendEvent(area.viewport(), ev)
        return True


#: One filter instance serves every widget; QObject parenting keeps it alive.
_NO_WHEEL = None

#: Wheel-editable widget types, all of which sit in the scrolling side panel.
_VALUE_WIDGETS = (QtWidgets.QAbstractSpinBox, QtWidgets.QComboBox,
                  QtWidgets.QSlider)


def _disable_wheel_edits(root):
    """Apply _NoWheelEdits to every value widget under `root`.

    Done in one sweep after the panel is built rather than at each widget's
    construction, so a control added later cannot forget to opt in.
    """
    global _NO_WHEEL
    if _NO_WHEEL is None:
        _NO_WHEEL = _NoWheelEdits()
    for cls in _VALUE_WIDGETS:
        for w in root.findChildren(cls):
            w.installEventFilter(_NO_WHEEL)
            # Otherwise the widget takes focus from a passing wheel or click
            # and then steals the arrow keys as well.
            w.setFocusPolicy(STRONG_FOCUS)


# --- Collapsible section ----------------------------------------------------

class Collapsible(QtWidgets.QWidget):
    """A group box with its title turned into a fold/unfold header.

    Collapsing hides the group as a whole rather than its individual widgets,
    so the per-widget visible and enabled states that the mode handlers set
    (_on_view_mode_changed, _on_algo_changed, _on_mode_changed) survive a fold
    and come back exactly as they were.
    """

    def __init__(self, group, parent=None):
        super().__init__(parent)
        self.group = group
        title = group.title()
        # The header carries the name now; a second copy inside the frame just
        # wastes a line of the panel we are trying to shorten.
        group.setTitle("")

        self.button = QtWidgets.QToolButton()
        self.button.setText(title)
        self.button.setCheckable(True)
        self.button.setChecked(True)
        self.button.setArrowType(ARROW_DOWN)
        self.button.setToolButtonStyle(BESIDE_ICON)
        self.button.setStyleSheet(
            "QToolButton { border: none; font-weight: bold; padding: 2px; }")
        self.button.toggled.connect(self.set_expanded)

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        v.addWidget(self.button)
        v.addWidget(group)

    def set_expanded(self, on):
        self.button.setArrowType(ARROW_DOWN if on else ARROW_RIGHT)
        self.group.setVisible(on)
        if self.button.isChecked() != on:
            self.button.setChecked(on)

    def is_expanded(self):
        return self.button.isChecked()


# --- Serial link ------------------------------------------------------------

class SerialLink(QtCore.QThread):
    """Owns the port. Reads on this thread and writes commands from a queue.

    Both directions are funnelled through one thread on purpose: the reader
    must never block, because if the host stops draining the pipe the firmware
    starts dropping samples and the sweep is unrepeatable.
    """

    event = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(object)     # telem tuple, or None
    config = QtCore.pyqtSignal(object)     # (degrees, time, gear)
    failed = QtCore.pyqtSignal(str)
    opened = QtCore.pyqtSignal()

    def __init__(self, port, capture, record_path=None, dtr=True, raw=None,
                 parent=None):
        super().__init__(parent)
        self.port = port
        self.cap = capture
        self.record_path = record_path
        self.dtr = dtr
        # Every byte the device sent, kept so "Save .bin" can write the exact
        # stream on demand. This is what makes an always-on auto-recording
        # unnecessary: nothing is lost by not writing to disk up front.
        self.raw = raw
        self._cmds = queue.Queue()
        self._stop = False
        # Read by the GUI thread's watchdog to tell "connected but silent"
        # apart from "working". Plain int assignment is atomic enough here.
        self.bytes_in = 0
        self.cmds_sent = 0

    def send(self, text):
        self._cmds.put(text.encode() if isinstance(text, str) else text)

    def stop(self):
        self._stop = True

    def run(self):
        fh = None
        try:
            ser = sp.open_port(self.port, BAUD, timeout=0.05, dtr=self.dtr)
        except Exception as exc:
            self.failed.emit(f"{self.port}: {exc}")
            return

        if self.record_path:
            fh = open(self.record_path, "wb")

        self.opened.emit()
        # One parser for the life of the connection: serial reads split records
        # at arbitrary points, and a per-chunk parser would drop every record
        # that lands on a seam.
        # sweep_only: the device streams lidar frames in every state, so an
        # idle-but-connected link would otherwise pile up samples forever and
        # keep the viewer rebuilding a finished cloud. The raw .bin recording
        # above is written before the parser sees the bytes, so it still gets
        # every frame regardless.
        parser = sp.StreamParser(self.cap, echo_events=False, sweep_only=True)
        n_ev = n_tel = 0
        n_cfg = None
        try:
            # Ask the device what it is set to, so the UI adopts the device's
            # real values rather than assuming its own defaults took.
            ser.write(b"?\n")
            while not self._stop:
                while not self._cmds.empty():
                    ser.write(self._cmds.get_nowait())
                    self.cmds_sent += 1

                chunk = ser.read(8192)
                if chunk:
                    if fh:
                        fh.write(chunk)
                    if self.raw is not None:
                        # Read back by the GUI thread in _save_bin. A bytearray
                        # += under the GIL is atomic, and that side only ever
                        # slices to a length it already observed, so it cannot
                        # catch a half-appended chunk.
                        self.raw += chunk
                    self.bytes_in += len(chunk)
                    parser.feed(chunk)

                # Counts, not high-water marks: "Clear scene" empties these
                # lists under us, and a `>` test against a stale count would
                # then stay false until the new scan had produced as many
                # records as the old one -- which froze the live attitude
                # readout for the rest of the session.
                if len(self.cap.events) < n_ev:
                    n_ev = 0
                while len(self.cap.events) > n_ev:
                    self.event.emit(self.cap.events[n_ev])
                    n_ev += 1
                n = len(self.cap.telem)
                if n != n_tel:
                    n_tel = n
                    if n:
                        self.status.emit(self.cap.telem[-1])
                if self.cap.config is not None and self.cap.config != n_cfg:
                    n_cfg = self.cap.config
                    self.config.emit(n_cfg)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if fh:
                fh.close()
            try:
                ser.close()
            except Exception:
                pass


# --- Surface shading --------------------------------------------------------

# pyqtgraph's stock "shaded" shader is one hard-coded lamp from (1,-1,-1) with
# no ambient term, which is why a surface under it reads as a single flat
# colour: every face steeper than the lamp crushes to the same near-black, and
# curvature disappears with it. It also lights only the front face, and a
# scanned surface is an open shell whose winding is arbitrary, so roughly half
# the triangles came out black and looked like holes in the mesh.
#
# This replaces it with the usual three-point rig plus a hemispheric ambient.
# The lights are defined in eye space so they travel with the camera and the
# shape stays legible from every orbit position, rather than going dark on one
# side of the scene.
# Written against pyqtgraph's GL ES 2 conventions -- u_mvp / u_normal
# uniforms, a_position / a_normal / a_color attributes -- and not the legacy
# fixed-function names, which this pyqtgraph does not accept.
MESH_VERT = """
    uniform mat4 u_mvp;
    uniform mat3 u_normal;
    attribute vec4 a_position;
    attribute vec3 a_normal;
    attribute vec4 a_color;
    varying vec4 v_color;
    varying vec3 v_normal;
    varying vec3 v_world;
    void main() {
        v_normal = normalize(u_normal * a_normal);
        // Kept unrotated as well: the ambient term below is anchored to world
        // up, not to the camera, so it needs the normal before the view
        // transform gets at it.
        v_world = normalize(a_normal);
        v_color = a_color;
        gl_Position = u_mvp * a_position;
    }
"""

MESH_FRAG = """
    #ifdef GL_ES
    precision mediump float;
    #endif
    varying vec4 v_color;
    varying vec3 v_normal;
    varying vec3 v_world;
    void main() {
        // Light whichever side we can actually see: a scanned surface is an
        // open shell, and without this roughly half of it renders unlit and
        // reads as holes.
        //
        // This used to test the eye-space normal's z, which silently assumes
        // the view direction is +Z everywhere -- true only at the centre of
        // the screen under perspective, and degenerate wherever N.z is near
        // zero. A near-vertical camera puts the whole floor into exactly that
        // regime, so the sign flickered across it, and since the ambient term
        // is flipped by the same sign every flicker swung a face between
        // ground and sky and washed it out.
        //
        // gl_FrontFacing is the hardware's own front/back answer, exact at
        // every angle and under any projection. It is the right test here
        // rather than merely a safer one: MeshData derives vertex normals
        // from face winding, so facing and normal cannot disagree.
        float side = gl_FrontFacing ? 1.0 : -1.0;
        vec3 N = normalize(v_normal) * side;
        vec3 W = normalize(v_world) * side;

        // The lamps sit well off the view axis. They used to point most of
        // the way down +Z, straight along the line of sight, and since the
        // flip above forces every visible normal to have N.z >= 0 that meant
        // every visible surface caught both lamps at once -- no surface could
        // ever be on the shadow side. Looking down at a floor, whose normal
        // then points right at the camera, both hit maximum and the whole
        // thing blew out to white. Keeping them lateral is what buys back the
        // light-to-dark falloff that makes curvature readable.
        vec3 Lkey  = normalize(vec3(-0.45,  0.60,  0.25));
        vec3 Lfill = normalize(vec3( 0.80, -0.20,  0.15));
        vec3 Lrim  = normalize(vec3( 0.05, -0.40, -0.90));

        vec3 Ckey  = vec3(1.00, 0.95, 0.86);   // warm key, upper left
        vec3 Cfill = vec3(0.42, 0.53, 0.72);   // cool fill, opposite side
        vec3 Crim  = vec3(0.55, 0.68, 0.95);   // rim, to separate silhouettes

        // Hemispheric ambient about world up, which is +Z in this scene.
        // Anchoring it to the world rather than the screen means it reads as
        // sky and ground rather than swinging round with the camera, so an
        // upward face stays brighter than a downward one from every angle.
        //
        // Deliberately a narrow spread. Looking straight down fills the view
        // with floor, every bit of it facing the same way and so taking the
        // same ambient; if that term carried most of the light the whole
        // screen would flatten to one bright tone no matter how good the
        // lamps were. The modelling has to come from the lamps, which vary
        // across a surface, not from ambient, which does not.
        float up = W.z * 0.5 + 0.5;
        vec3 amb = mix(vec3(0.13, 0.12, 0.12), vec3(0.21, 0.23, 0.28), up);

        float dk = max(dot(N, Lkey),  0.0);
        float df = max(dot(N, Lfill), 0.0);
        float dr = max(dot(N, Lrim),  0.0);

        // Blinn-Phong against the key only; the view vector is +Z as above.
        vec3 H = normalize(Lkey + vec3(0.0, 0.0, 1.0));
        float spec = pow(max(dot(N, H), 0.0), 32.0) * 0.12;

        vec3 rgb = v_color.rgb * (amb + Ckey * dk * 0.68 + Cfill * df * 0.30)
                 + Ckey * spec + Crim * dr * 0.14;
        gl_FragColor = vec4(rgb, v_color.a);
    }
"""


def _studio_shader():
    from pyqtgraph.opengl import shaders
    return shaders.ShaderProgram("lidarStudio", [
        shaders.VertexShader(MESH_VERT),
        shaders.FragmentShader(MESH_FRAG),
    ])


# --- Meshing worker ---------------------------------------------------------

CAM_ORBIT, CAM_FPS = "orbit", "fps"


def _vec3(v):
    """pyqtgraph Vector / QVector3D -> plain (3,) array."""
    return np.array([v.x(), v.y(), v.z()], dtype=float)


class SceneView(gl.GLViewWidget):
    """The 3D view, with a second camera style bolted on.

    Orbit mode is stock pyqtgraph: the camera swings around a fixed centre,
    which is what Blender-ish tools do and what suits inspecting one cloud.

    FPS mode reuses the same machinery instead of tracking its own pose. The
    camera's position is a function of centre/azimuth/elevation, so looking
    around means turning as usual and then sliding the centre so the *eye*
    lands back where it was -- rotation about the head rather than about the
    scene. Walking is then just a translation of the centre. Keeping one pose
    representation means switching modes never jumps the view, and everything
    that reads the camera (framing, the shader's eye-space lights) is unaware
    there are two modes at all.
    """

    # Eye-to-centre distance held while flying. Small enough that the centre
    # is effectively the head, large enough to stay clear of the near plane.
    FPS_DISTANCE = 100.0

    # Direction per action as (forward, right, up) coefficients in the camera's
    # frame. Keys map to actions, actions map to motion: the two tables meet at
    # a name, so the keys the event filter claims cannot drift out of step with
    # the keys walking actually honours.
    MOVES = {
        "fwd": (1, 0, 0),
        "back": (-1, 0, 0),
        "right": (0, 1, 0),
        "left": (0, -1, 0),
        "rise": (0, 0, 1),
        "sink": (0, 0, -1),
    }

    # Keys whose meaning the keyboard layout cannot change: arrows and
    # modifiers sit outside the alphabetic block, so Qt reports the same
    # Key_ value for them whatever language is selected.
    LAYOUT_SAFE_KEYS = {
        "Up": "fwd", "Down": "back", "Right": "right", "Left": "left",
        "Space": "rise", "Control": "sink", "Shift": "fast",
    }

    # WASDQE by *position on the keyboard*, which is the only thing about them
    # that is stable. On a Cyrillic (or Greek, or Hebrew) layout the physical W
    # key delivers Key_Tse, not Key_W, so a table keyed on ev.key() stops
    # matching the moment the layout is switched -- and the camera turns but
    # refuses to walk, since the arrows kept working. That is the whole of the
    # intermittent "flying is broken" bug: it depended on the input language,
    # not on focus, so it could start broken or break mid-session at Alt+Shift.
    # Scan codes are the hardware's own numbering and are untouched by layout.
    SCAN_ACTIONS = {
        "win32": {0x11: "fwd", 0x1F: "back", 0x20: "right", 0x1E: "left",
                  0x12: "rise", 0x10: "sink"},
        "darwin": {13: "fwd", 1: "back", 2: "right", 0: "left",
                   14: "rise", 12: "sink"},
        # X11/Wayland report evdev codes offset by 8.
        "linux": {25: "fwd", 39: "back", 40: "right", 38: "left",
                  26: "rise", 24: "sink"},
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._scan_actions = self.SCAN_ACTIONS.get(
            "linux" if sys.platform.startswith("linux") else sys.platform, {})
        # The Latin names stay in play as a fallback, for a platform whose scan
        # codes we do not know and for anyone on a plain US layout.
        self._key_actions = {
            _key(n): a for n, a in dict(
                self.LAYOUT_SAFE_KEYS,
                W="fwd", S="back", D="right", A="left", E="rise", Q="sink",
            ).items()
        }
        self.cam_mode = CAM_ORBIT
        self._orbit_distance = None   # remembered across a trip into FPS
        self._held = set()
        self._toast = None            # built on first use, see _flash
        self._toast_timer = None
        self.speed = 1500.0           # mm/s at a walk
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
            # Stand where the camera already is, looking the same way.
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
            # Pull the centre back out to a sane orbit radius along the
            # current view direction, so the scene stays in front of you.
            eye = _vec3(self.cameraPosition())
            self.opts["distance"] = self._orbit_distance or 4000.0
            self._place(eye)
        self.cam_mode = mode
        self.update()

    def setCameraPosition(self, pos=None, distance=None, elevation=None,
                          azimuth=None, rotation=None):
        """Framing, without letting it undo FPS mode.

        In FPS mode the eye-to-centre distance is the whole trick: it is held
        tiny so that turning pivots about the head. Stock framing writes a
        scene-sized radius straight into opts['distance'], which silently
        turned flying back into orbiting about a point tens of metres away --
        the camera looked stuck, and toggling the mode was the only way back,
        because that is where FPS_DISTANCE gets restored. So here framing means
        standing back from the target and looking at it, and the requested
        radius is remembered for the eventual return to orbit instead.
        """
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
        """Unit vector from the eye towards the centre."""
        el = np.radians(self.opts["elevation"])
        az = np.radians(self.opts["azimuth"])
        return -np.array([np.cos(el) * np.cos(az),
                          np.cos(el) * np.sin(az),
                          np.sin(el)])

    def _place(self, eye):
        """Move the centre so the eye sits at `eye` for the current angles."""
        c = np.asarray(eye) + self._forward() * self.opts["distance"]
        self.opts["center"] = pg.Vector(*c)

    # --- looking ------------------------------------------------------------

    def _orbit_move(self, ev):
        """Orbit-mode dragging, with a pan that can lift the pivot.

        Stock pyqtgraph pans with relative='view-upright', whose two axes both
        lie in the world's horizontal plane: the pivot slides around the floor
        and never leaves the height it was framed at. On a room scan that
        height lands somewhere around the middle of the walls, and anything
        below it -- the floor, the underside of furniture -- could only be
        looked at from above, because there was no way to bring the point being
        orbited down to it.

        relative='view' instead pans in the plane of the screen, so a vertical
        drag carries the pivot up and down through the scene exactly as
        shift-drag does in Blender. Shift+left and Alt+left are bound to the
        same thing for laptops, where there is no middle button to hold: a
        touchpad can only report left and right, so the Blender pivot-slide is
        otherwise simply unreachable. Alt+left is the pairing Blender itself
        offers under "emulate 3 button mouse", so the muscle memory carries.
        """
        pos = ev.position() if hasattr(ev, "position") else ev.localPos()
        if getattr(self, "mousePos", None) is None:
            self.mousePos = pos
        diff = pos - self.mousePos
        mods = ev.modifiers()
        # `&` rather than `==` on the buttons: a touchpad tap-drag can report a
        # stray second button, and a strict compare would drop the whole drag.
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
        # pyqtgraph only creates mousePos on the first press, so it can be
        # missing entirely rather than None.
        if getattr(self, "mousePos", None) is None:
            self.mousePos = pos
        diff = pos - self.mousePos
        self.mousePos = pos
        eye = _vec3(self.cameraPosition())
        # Opposite sign to orbiting, on both axes. Orbiting drags the *scene*:
        # pull right and the object turns right, so the camera goes left. Here
        # the drag moves the head, so right means look right -- which is the
        # same swap Unity's scene view makes between its orbit and fly modes.
        self.opts["azimuth"] -= diff.x() * 0.5
        self.opts["elevation"] = float(
            np.clip(self.opts["elevation"] + diff.y() * 0.5, -89.9, 89.9))
        self._place(eye)
        self.update()

    @staticmethod
    def _scroll_steps(ev):
        """Scroll amount as (horizontal, vertical, precise) in wheel notches.

        `precise` says the deltas came from pixelDelta, i.e. from fingers on a
        precision touchpad rather than from a wheel's detents. The two devices
        want different jobs from the same event, so the caller has to be able to
        tell them apart.

        A mouse wheel arrives in whole 120-unit detents, so one notch is one
        unit here and the feel is unchanged. A precision touchpad instead
        streams many small deltas and fills in pixelDelta, which is the finer
        and more honest measure of how far the fingers actually moved -- using
        it is what makes two-finger scrolling glide rather than jump in wheel
        detents. 120px of finger travel is called one notch so that both
        devices cover comparable ground.

        Stock pyqtgraph reads angleDelta().x() first and only falls back to y,
        which on a touchpad means an unavoidably imperfect horizontal component
        of a vertical swipe hijacks the zoom. The two axes are kept separate
        here and given separate jobs.
        """
        if not hasattr(ev, "angleDelta"):       # very old bindings
            return 0.0, ev.delta() / 120.0, False
        pixels = ev.pixelDelta() if hasattr(ev, "pixelDelta") else None
        if pixels is not None and (pixels.x() or pixels.y()):
            return pixels.x() / 120.0, pixels.y() / 120.0, True
        angle = ev.angleDelta()
        return angle.x() / 120.0, angle.y() / 120.0, False

    def _set_speed(self, value):
        """Retune the walk, and say so briefly on the view.

        Speed has no control of its own and no readout, so before this a notch
        of wheel changed how fast you fly with nothing to show for it -- you
        found out by walking. Unity answers the same gesture with a number that
        fades, which is enough to aim at a speed instead of feeling for it.
        """
        self.speed = float(np.clip(value, 50.0, 50000.0))
        self._flash(f"Fly speed  {self.speed / 1000.0:.2f} m/s")

    def _flash(self, text, ms=900):
        """Show `text` over the view, then let it disappear."""
        if self._toast is None:
            self._toast = QtWidgets.QLabel(self)
            self._toast.setStyleSheet(
                "background: rgba(0, 0, 0, 160); color: white;"
                "padding: 6px 10px; border-radius: 4px;")
            # Clicks belong to the camera; the label is a readout, not a
            # target, and must not swallow a drag that starts under it.
            self._toast.setAttribute(TRANSPARENT_FOR_MOUSE, True)
            self._toast_timer = QtCore.QTimer(self)
            self._toast_timer.setSingleShot(True)
            self._toast_timer.timeout.connect(self._toast.hide)
        self._toast.setText(text)
        self._toast.adjustSize()
        self._toast.move((self.width() - self._toast.width()) // 2,
                         self.height() - self._toast.height() - 24)
        self._toast.show()
        self._toast_timer.start(ms)   # restart: each notch buys another moment

    def _zoom(self, steps):
        """Pull the camera in or out by `steps` wheel notches."""
        # Matches pyqtgraph's 0.999**delta on a 120-unit detent exactly, so a
        # mouse wheel feels the way it always did.
        self.opts["distance"] *= 0.999 ** (steps * 120.0)
        self.update()

    def wheelEvent(self, ev):
        dx, dy, precise = self._scroll_steps(ev)
        if self.cam_mode == CAM_FPS:
            # Zooming has no meaning without an orbit radius; spend the wheel
            # on how fast you walk instead, which is what you actually retune.
            self._set_speed(self.speed * 1.15 ** dy)
            ev.accept()
            return
        if ev.modifiers() & SHIFT_MOD:
            # Shift turns the whole gesture into a two-axis pan: the way to
            # slide the pivot with no buttons held at all, which is the state a
            # touchpad is in whenever it is not being pressed.
            self.pan(dx * 40.0, dy * 40.0, 0, relative="view")
        elif precise and not ev.modifiers() & CTRL_MOD:
            # A bare two-finger swipe orbits, as it does in Blender. It used to
            # zoom, which made it a second, clumsier pinch and left the gesture
            # a touchpad reaches for most doing the job the pinch already does
            # better -- while orbiting, the thing you actually want, needed a
            # button held down. Pixels are read as if the fingers were dragging,
            # so a swipe turns the view by as much as a drag of the same length.
            self.orbit(-dx * 120.0, dy * 120.0)
        else:
            # A wheel zooms, as it always has, and so does ctrl+scroll -- both
            # what a non-precision touchpad sends for a pinch and Blender's
            # touchpad zoom. Horizontal wheel scroll has no tradition to honour
            # and is free to do what a mouse otherwise cannot: slide the pivot.
            if dy:
                self._zoom(dy)
            if dx and not precise:
                self.pan(dx * 40.0, 0, 0, relative="view")
        ev.accept()

    def event(self, ev):
        """Pinch-to-zoom, which arrives outside the wheel path.

        Windows precision touchpads and macOS trackpads report a pinch as a
        native gesture rather than as ctrl+wheel, and Qt has no virtual handler
        for those -- unhandled, the most natural zoom gesture a laptop has does
        nothing at all.
        """
        if ev.type() == EV_NATIVE_GESTURE and ZOOM_GESTURE is not None:
            if ev.gestureType() == ZOOM_GESTURE:
                # value() is the fractional scale change for this step.
                if self.cam_mode == CAM_FPS:
                    self._set_speed(self.speed * (1.0 + ev.value()))
                else:
                    self.opts["distance"] *= max(0.1, 1.0 - ev.value())
                    self.update()
                return True
        return super().event(ev)

    # --- walking ------------------------------------------------------------

    def eventFilter(self, obj, ev):
        """Fly keys, taken from the application rather than from focus.

        Focus was the whole problem: WASD only reached the view while the view
        itself was the focus widget, so any click on the side panel -- or just
        the combo that turns FPS mode on keeping focus for itself -- left the
        camera dead until the mode was toggled. setFocus() calls scattered over
        press/enter patched individual routes into that state and still missed
        most of them, which is why flying worked only some of the time.

        So while flying, the keys are read here, before Qt routes them by
        focus. A pointer over the view means the flying keys are ours no matter
        what is focused, which is the Unity rule and lets you retune a spinbox
        and fly again without a click in between. Off the view, only genuine
        focus counts, so typing into that spinbox still types.
        """
        t = ev.type()
        if t == EV_WINDOW_DEACTIVATE:
            # Releases go to whoever has the keyboard next; without this the
            # keys held at alt-tab stay held and the camera drifts on return.
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
        """What this keystroke means, by position first and letter second.

        Position wins because it is what the finger actually did: the scan code
        of the key under the left middle finger is the same number whether the
        layout calls it W or Ц. ev.key() is consulted only as a fallback, which
        keeps the arrows and modifiers working (they have no scan code entry)
        and keeps some unknown platform's letters working too.
        """
        action = self._scan_actions.get(ev.nativeScanCode())
        if action is not None:
            return action
        return self._key_actions.get(ev.key())

    def _pointer_over_view(self):
        if not self.isVisible():
            return False
        return self.rect().contains(self.mapFromGlobal(QtGui.QCursor.pos()))

    def mousePressEvent(self, ev):
        # pyqtgraph's handler does not chain up, so nothing here can be assumed
        # to have run; take focus explicitly rather than relying on Qt's
        # click-to-focus surviving that.
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
        # Looking straight up or down leaves no unique "right"; hold the last
        # usable one by falling back to the world axis rather than dividing by
        # zero and flinging the camera off.
        right = right / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])

        move = np.zeros(3)
        for action, (f, r, u) in self.MOVES.items():
            if action in self._held:
                move += f * fwd + r * right + u * up
        if not move.any():
            return
        # Shift comes from the held set like everything else, not from
        # QApplication.keyboardModifiers(): that reports the modifiers of the
        # last event Qt delivered, and this timer fires with no events in
        # between, so it lagged a keystroke behind -- the boost arrived on
        # release rather than on press.
        fast = 4.0 if "fast" in self._held else 1.0
        step = move / np.linalg.norm(move) * self.speed * fast * dt
        self.opts["center"] = pg.Vector(*(_vec3(self.opts["center"]) + step))
        self.update()


class MeshWorker(QtCore.QThread):
    """Runs one reconstruction off the GUI thread.

    Meshing is seconds of work where everything else here is milliseconds:
    Poisson at depth 10 on a room-sized cloud is long enough that doing it
    inline would stall the serial drain and cost samples out of a scan that
    cannot be repeated.
    """

    done = QtCore.pyqtSignal(object, object, object)  # verts, faces, info
    failed = QtCore.pyqtSignal(str)

    def __init__(self, xyz, method, params, parent=None):
        super().__init__(parent)
        # Copy: the caller's cloud is replaced wholesale by the next rebuild,
        # and a worker reading it as it goes would mesh half of one scan and
        # half of another.
        self.xyz = np.array(xyz, copy=True)
        self.method = method
        self.params = dict(params)

    def run(self):
        t0 = time.time()
        try:
            verts, faces, info = meshing.reconstruct(self.xyz, self.method,
                                                     **self.params)
        except Exception as exc:
            self.failed.emit(str(exc))
            return
        info["seconds"] = time.time() - t0
        self.done.emit(verts, faces, info)


class RegisterWorker(QtCore.QThread):
    """Aligns a new scan onto the map off the GUI thread.

    Registration is seconds of numpy on a big cloud. Run inline it freezes the
    window, which on a tool that is also holding a serial link looks exactly
    like the board has stopped talking.
    """

    done = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    note = QtCore.pyqtSignal(str)

    def __init__(self, source, target, params, parent=None):
        super().__init__(parent)
        self.source = source
        self.target = target
        self.params = params

    def run(self):
        try:
            result = reg.register(self.source, self.target,
                                  progress=self.note.emit, **self.params)
        except Exception as exc:                       # noqa: BLE001
            self.failed.emit(str(exc))
            return
        self.done.emit(result)


# --- Main window ------------------------------------------------------------

class ScannerUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("3D Lidar Scanner")
        self.resize(1500, 950)

        self.link = None
        self.cap = sp.Capture()
        # The undecoded stream behind self.cap, for "Save .bin". Held in memory
        # rather than written as it arrives, so no scan leaves a file behind
        # unless it was asked for. It is strictly smaller than the decoded
        # tuples in self.cap, so keeping it costs nothing that was not already
        # being spent.
        self.raw = bytearray()
        self.cloud = None          # (xyz, dist) currently displayed
        self.loaded_rgb = None     # colours from a loaded .ply, if any
        self._built_n = 0          # sample count the displayed cloud was built from

        # --- multi-scan map ---
        # The scanner only ever sees the room from where it is standing. In
        # multi-scan mode each sweep is kept as its own cloud plus the 4x4 that
        # puts it in the first scan's frame, and the map is their union. Poses
        # are kept rather than baked in so a bad alignment can be undone.
        self.scans = []            # [{"xyz", "dist", "T", "label"}]
        self.live = None           # (xyz, dist) of the sweep not yet added
        self._map = None           # cached (xyz, dist) union of self.scans
        self._reg_worker = None
        self._pending = None       # {"result", "xyz", "dist"} awaiting a verdict
        self._last_state = sp.STATE_IDLE
        self.mesh = None           # (verts, faces) currently displayed
        self._mesh_worker = None
        self._mesh_points = 0      # cloud size the mesh was built from
        self.state = sp.STATE_IDLE
        self.platform = 0.0        # last shaft angle the device reported
        self.sweep_started = None
        self.connected_at = 0.0
        self.warned_silent = False

        # Meshing parameters are usually found by dragging a spinbox, and each
        # stop on the way is a full reconstruction. Coalesce a flurry of edits
        # into the one build the user actually meant.
        self._mesh_debounce = QtCore.QTimer(self)
        self._mesh_debounce.setSingleShot(True)
        self._mesh_debounce.setInterval(600)
        self._mesh_debounce.timeout.connect(self._build_mesh)

        self._build_ui()
        # Ports first: the saved port is restored by name, so the combo has to
        # be populated before there is anything for it to match against.
        self._refresh_ports()
        self._init_settings()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(250)

    # --- layout -------------------------------------------------------------

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        side = QtWidgets.QVBoxLayout()
        side.setSpacing(4)
        panel = QtWidgets.QWidget()
        panel.setLayout(side)

        # The panel is taller than most screens with every section open, so it
        # rides inside a scroll area rather than squashing its own contents
        # (which is what a plain layout does once it runs out of room: spin
        # boxes and the log shrink towards unusable).
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        scroll.setFrameShape(NO_FRAME)
        scroll.setHorizontalScrollBarPolicy(SCROLLBAR_OFF)
        scroll.setVerticalScrollBarPolicy(SCROLLBAR_AUTO)
        # Room for the scrollbar on top of the 320 the controls were laid out
        # for, so nothing reflows when it appears.
        scroll.setFixedWidth(340)
        root.addWidget(scroll)

        # key -> section, for remembering which were folded between runs.
        self.sections = {}
        for key, group in (("device", self._device_group()),
                           ("scan", self._scan_group()),
                           ("map", self._map_group()),
                           ("view", self._view_group()),
                           ("geometry", self._geometry_group()),
                           ("file", self._file_group())):
            sec = Collapsible(group)
            self.sections[key] = sec
            side.addWidget(sec)
        # _on_view_mode_changed shows and hides this wholesale, header and all.
        self.geom_group = self.sections["geometry"]

        # After every group exists, so no control can be missed.
        _disable_wheel_edits(panel)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setPlaceholderText("device messages")
        self.log.setMinimumHeight(120)
        side.addWidget(self.log, 1)

        # --- 3D view ---
        self.view = SceneView()
        self.view.setCameraPosition(distance=4000, elevation=20, azimuth=45)
        grid = gl.GLGridItem()
        grid.setSize(10000, 10000)
        grid.setSpacing(500, 500)
        grid.translate(0, 0, -1500)
        self.view.addItem(grid)
        self.origin = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=12,
                                           color=(1, 1, 1, 1))
        self.view.addItem(self.origin)
        self.scatter = gl.GLScatterPlotItem(pos=np.zeros((0, 3)), size=2.0,
                                            pxMode=True)
        self.view.addItem(self.scatter)

        # The reconstructed surface. Lives alongside the scatter rather than
        # replacing it, so switching view modes is a visibility flip and never
        # costs a rebuild of the thing you just waited for.
        self._shader = _studio_shader()
        self.mesh_item = gl.GLMeshItem(smooth=True, shader=self._shader,
                                       drawEdges=False)
        self.mesh_item.setVisible(False)
        self.view.addItem(self.mesh_item)

        root.addWidget(self.view, 1)

        self.statusBar().showMessage("disconnected")

    def _device_group(self):
        g = QtWidgets.QGroupBox("Device")
        f = QtWidgets.QGridLayout(g)
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(140)
        f.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        f.addWidget(self.port_box, 0, 1)
        b = QtWidgets.QPushButton("Refresh")
        b.clicked.connect(self._refresh_ports)
        f.addWidget(b, 0, 2)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        f.addWidget(self.connect_btn, 1, 0, 1, 3)

        self.dtr_chk = QtWidgets.QCheckBox("Assert DTR (needed by RP2040 USB)")
        self.dtr_chk.setChecked(True)
        self.dtr_chk.setToolTip(
            "The RP2040's USB serial only sends once the host raises DTR. "
            "If the device connects but stays silent, this is the first thing "
            "to try toggling.")
        f.addWidget(self.dtr_chk, 2, 0, 1, 3)

        self.record_chk = QtWidgets.QCheckBox("Record raw stream alongside scan")
        self.record_chk.setChecked(False)
        self.record_chk.setToolTip(
            "Write a scan_*.bin into scans/autosaves as the scan runs, "
            "without being asked.\n"
            "Off by default: the same bytes are kept in memory either way, so "
            "Save .bin can write them afterwards. Turn this on only if you "
            "want the file to survive the application crashing mid-scan.")
        f.addWidget(self.record_chk, 3, 0, 1, 3)

        self.link_lbl = QtWidgets.QLabel("not connected")
        self.link_lbl.setWordWrap(True)
        f.addWidget(self.link_lbl, 4, 0, 1, 3)
        return g

    def _scan_group(self):
        g = QtWidgets.QGroupBox("Scan")
        f = QtWidgets.QGridLayout(g)

        self.angle_spin = QtWidgets.QDoubleSpinBox()
        self.angle_spin.setRange(1.0, 180.0)
        self.angle_spin.setValue(90.0)
        self.angle_spin.setSuffix(" deg")
        self.angle_spin.setToolTip(
            "Half-sweep: the shaft runs -this to +this about the vertical.\n"
            "90 already covers the whole scene in both-halves mode, because "
            "the lidar's scan plane is vertical and half a turn carries it "
            "through every azimuth.\n"
            "Anything past 90 is still allowed there, and re-scans what it "
            "passes over a second time: more shots per surface, and a full "
            "180 means every direction is seen from both sides of the sweep.\n"
            "One-side scanning (see 'Scan half' in the View panel) *needs* "
            "180: half a scan plane is a pole-to-pole arc, so it takes the "
            "whole turn to cover the same sphere.")
        f.addWidget(QtWidgets.QLabel("Angle  ±"), 0, 0)
        f.addWidget(self.angle_spin, 0, 1)

        self.time_spin = QtWidgets.QDoubleSpinBox()
        self.time_spin.setRange(1.0, 6000.0)
        self.time_spin.setValue(30.0)
        self.time_spin.setSuffix(" s")
        f.addWidget(QtWidgets.QLabel("Duration"), 1, 0)
        f.addWidget(self.time_spin, 1, 1)

        # Stepped mode: the platform stops at each of these before capturing,
        # so the duration above no longer describes the runtime -- the label
        # updates to say what it will actually take.
        self.stepped_chk = QtWidgets.QCheckBox("Stepped (stop at each angle)")
        self.stepped_chk.setToolTip(
            "Moves, stops, waits out the ring-down, then captures with the "
            "shaft genuinely still.\n"
            "Much slower than a continuous sweep, and much less sensitive to "
            "the lidar's vibration.")
        self.stepped_chk.stateChanged.connect(self._on_mode_changed)
        f.addWidget(self.stepped_chk, 2, 0, 1, 2)

        self.steps_spin = QtWidgets.QSpinBox()
        self.steps_spin.setRange(2, 2000)
        self.steps_spin.setValue(60)
        self.steps_spin.setToolTip("Intervals across the sweep; stops = this + 1")
        self.steps_lbl = QtWidgets.QLabel("Steps")
        f.addWidget(self.steps_lbl, 3, 0)
        f.addWidget(self.steps_spin, 3, 1)

        self.dwell_spin = QtWidgets.QSpinBox()
        self.dwell_spin.setRange(50, 5000)
        self.dwell_spin.setValue(400)
        self.dwell_spin.setSuffix(" ms")
        self.dwell_spin.setToolTip(
            "Lidar capture time at each stop. One revolution of the lidar is "
            "about 200 ms, so 400 gives roughly two full sweeps per stop.")
        self.dwell_lbl = QtWidgets.QLabel("Dwell")
        f.addWidget(self.dwell_lbl, 4, 0)
        f.addWidget(self.dwell_spin, 4, 1)

        self.est_lbl = QtWidgets.QLabel("")
        self.est_lbl.setStyleSheet("color: #9a9a9a;")
        f.addWidget(self.est_lbl, 5, 0, 1, 2)
        for w in (self.steps_spin, self.dwell_spin, self.angle_spin):
            w.valueChanged.connect(self._update_estimate)
        self.angle_spin.valueChanged.connect(self._on_half_changed)

        self.start_btn = QtWidgets.QPushButton("Start scan")
        self.start_btn.setToolTip(
            "Pushes the settings above to the device, then sweeps. There is no "
            "separate apply step -- what is on screen is what runs.")
        self.start_btn.clicked.connect(self._start)
        f.addWidget(self.start_btn, 6, 0)
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.clicked.connect(self._stop)
        f.addWidget(self.stop_btn, 6, 1)

        self.delay_spin = QtWidgets.QSpinBox()
        self.delay_spin.setRange(0, 600)
        self.delay_spin.setValue(0)
        self.delay_spin.setSuffix(" s")
        self.delay_spin.setToolTip(
            "Wait this many seconds after pressing Start before the sweep "
            "actually begins. Gives you time to step out of the scan volume. "
            "0 starts immediately.")
        self.delay_lbl = QtWidgets.QLabel("Start delay")
        f.addWidget(self.delay_lbl, 7, 0)
        f.addWidget(self.delay_spin, 7, 1)

        # No unwrap or re-home buttons. The tether does twist as the shaft
        # turns, but the coils are released whenever the rig is idle
        # (SCAN_IDLE_DISABLE_MS in scanner.h), so the shaft is backdrivable and
        # the twist comes out by hand. The firmware's 'h' and 'u' commands
        # still exist if a future rig holds torque.
        self.clear_btn = QtWidgets.QPushButton("Clear scene")
        self.clear_btn.setToolTip(
            "Drop the points collected so far and start the cloud over. The "
            "raw .bin recording is untouched, so nothing is actually lost.")
        self.clear_btn.clicked.connect(self._clear_scene)
        f.addWidget(self.clear_btn, 8, 0, 1, 2)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setTextVisible(True)
        f.addWidget(self.progress, 9, 0, 1, 2)

        self.state_lbl = QtWidgets.QLabel("idle")
        f.addWidget(self.state_lbl, 10, 0, 1, 2)

        self._on_mode_changed()
        return g

    def _map_group(self):
        """Multi-scan: build one map out of several sweeps from different spots.

        The scanner cannot see round a corner or through itself, so one sweep is
        always partly a shadow. Scanning again from somewhere else fills the
        shadow in, but the second cloud arrives in its own frame -- centred on
        wherever the rig now stands, pointing wherever it now points. This panel
        is the machinery that puts the two in the same frame.
        """
        g = QtWidgets.QGroupBox("Multi-scan map")
        f = QtWidgets.QGridLayout(g)
        row = 0

        self.multi_chk = QtWidgets.QCheckBox("Merge scans into one map")
        self.multi_chk.setToolTip(
            "Off: every scan replaces the last, as before.\n"
            "On: each finished scan is aligned onto what has already been "
            "mapped and added to it, so you can scan, move the rig, and scan "
            "again to fill in what the first pass could not see.\n"
            "The scans have to overlap -- roughly a third of the new sweep "
            "needs to land on ground the map already covers.")
        self.multi_chk.stateChanged.connect(self._on_multi_changed)
        f.addWidget(self.multi_chk, row, 0, 1, 2)
        row += 1

        self.auto_add_chk = QtWidgets.QCheckBox("Add each scan automatically")
        self.auto_add_chk.setToolTip(
            "Align and add as soon as a sweep finishes. Turn this off to look "
            "at a scan before it goes into the map.")
        self.auto_add_chk.setChecked(True)
        f.addWidget(self.auto_add_chk, row, 0, 1, 2)
        row += 1

        # How much the operator is willing to tell the algorithm about how they
        # moved the rig. A room is mostly flat walls, so a scan taken from a
        # different spot can line up convincingly in more than one way; saying
        # roughly which way the rig was turned removes the ambiguity outright.
        self.align_box = QtWidgets.QComboBox()
        self.align_box.addItem("Search all", "auto")
        self.align_box.addItem("Barely moved", "small")
        self.align_box.addItem("Known turn", "hint")
        # The item text has to stay short or it sets the width of the whole
        # side panel; the explanation lives here instead.
        self.align_box.setToolTip(
            "How much you can tell it about how the rig moved.\n\n"
            "Search all: no assumptions, tries every heading. Slowest, and in "
            "a bare symmetric room it can pick the alignment that is 180 "
            "degrees out.\n"
            "Barely moved: assumes the rig is near where it was, facing much "
            "the same way. Fastest and safest for a small nudge.\n"
            "Known turn: you give the rough heading change below and it works "
            "out the rest. Being 20 degrees out is fine.")
        self.align_box.setSizeAdjustPolicy(
            QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
            if hasattr(QtWidgets.QComboBox, "SizeAdjustPolicy")
            else QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.align_box.setMinimumContentsLength(10)
        self.align_box.currentIndexChanged.connect(self._on_align_changed)
        f.addWidget(QtWidgets.QLabel("Movement"), row, 0)
        f.addWidget(self.align_box, row, 1)
        row += 1

        self.yaw_spin = QtWidgets.QDoubleSpinBox()
        self.yaw_spin.setRange(-180.0, 180.0)
        self.yaw_spin.setValue(0.0)
        self.yaw_spin.setSuffix(" deg")
        self.yaw_spin.setToolTip(
            "How far the rig was turned about the vertical since the previous "
            "scan -- not since the first one. Positive is anticlockwise seen "
            "from above.\n"
            "Eyeballing it is fine: 20 degrees out still lands correctly.")
        self.yaw_lbl = QtWidgets.QLabel("Turned by")
        f.addWidget(self.yaw_lbl, row, 0)
        f.addWidget(self.yaw_spin, row, 1)
        row += 1

        self.reg_voxel_spin = QtWidgets.QDoubleSpinBox()
        self.reg_voxel_spin.setRange(5.0, 500.0)
        self.reg_voxel_spin.setValue(40.0)
        self.reg_voxel_spin.setSuffix(" mm")
        self.reg_voxel_spin.setToolTip(
            "Detail the alignment works at. Smaller is more precise and "
            "slower, and below the noise on a single shot it stops helping. "
            "40 mm suits a room.")
        f.addWidget(QtWidgets.QLabel("Detail"), row, 0)
        f.addWidget(self.reg_voxel_spin, row, 1)
        row += 1

        self.add_btn = QtWidgets.QPushButton("Add scan")
        self.add_btn.setToolTip(
            "Align the scan on screen onto the map and add it. The first one "
            "goes in as-is and defines the map's frame.")
        self.add_btn.clicked.connect(self._add_scan)
        f.addWidget(self.add_btn, row, 0)
        self.undo_btn = QtWidgets.QPushButton("Remove last")
        self.undo_btn.setToolTip("Take the most recently added scan back out.")
        self.undo_btn.clicked.connect(self._undo_scan)
        f.addWidget(self.undo_btn, row, 1)
        row += 1

        # Shown only while an alignment is waiting on the operator. No
        # threshold can tell a correct alignment from the mirror-image one a
        # symmetric room also admits, so the last word is a human looking at
        # the overlap on screen.
        self.accept_btn = QtWidgets.QPushButton("Keep")
        self.accept_btn.clicked.connect(self._accept_pending)
        f.addWidget(self.accept_btn, row, 0)
        self.reject_btn = QtWidgets.QPushButton("Discard")
        self.reject_btn.clicked.connect(self._reject_pending)
        f.addWidget(self.reject_btn, row, 1)
        row += 1

        self.map_lbl = QtWidgets.QLabel("map empty")
        self.map_lbl.setWordWrap(True)
        self.map_lbl.setStyleSheet("color: #9a9a9a;")
        f.addWidget(self.map_lbl, row, 0, 1, 2)
        row += 1

        self.clear_map_btn = QtWidgets.QPushButton("Clear map")
        self.clear_map_btn.setToolTip(
            "Throw away every added scan and start the map over. The raw .bin "
            "stream is untouched.")
        self.clear_map_btn.clicked.connect(self._clear_map)
        f.addWidget(self.clear_map_btn, row, 0, 1, 2)

        return g

    def _view_group(self):
        g = QtWidgets.QGroupBox("View")
        f = QtWidgets.QGridLayout(g)

        # Which of the two renderings is on screen. The settings below it split
        # the same way: everything from here down to "Voxel" is about points,
        # everything in the Geometry box is about the surface, and the mount
        # geometry further down feeds both because it decides where the points
        # are before either of them gets a say.
        self.mode_box = QtWidgets.QComboBox()
        self.mode_box.addItems(["Points", "Geometry"])
        self.mode_box.setToolTip(
            "Points draws the raw cloud.\n"
            "Geometry reconstructs a triangle surface from it -- slower, and "
            "it does not update live during a scan.")
        self.mode_box.currentIndexChanged.connect(self._on_view_mode_changed)
        f.addWidget(QtWidgets.QLabel("Mode"), 0, 0)
        f.addWidget(self.mode_box, 0, 1)

        self.color_lbl = QtWidgets.QLabel("Colour by")
        self.color_box = QtWidgets.QComboBox()
        self.color_box.addItems(["height", "distance"])
        self.color_box.currentIndexChanged.connect(self._redraw)
        f.addWidget(self.color_lbl, 1, 0)
        f.addWidget(self.color_box, 1, 1)

        self.size_lbl = QtWidgets.QLabel("Point size")
        self.size_spin = QtWidgets.QDoubleSpinBox()
        self.size_spin.setRange(0.5, 12.0)
        self.size_spin.setValue(2.0)
        self.size_spin.setSingleStep(0.5)
        self.size_spin.valueChanged.connect(self._redraw)
        f.addWidget(self.size_lbl, 2, 0)
        f.addWidget(self.size_spin, 2, 1)

        self.voxel_spin = QtWidgets.QDoubleSpinBox()
        self.voxel_spin.setRange(0.0, 500.0)
        self.voxel_spin.setValue(0.0)
        self.voxel_spin.setSuffix(" mm")
        self.voxel_spin.setToolTip("0 = keep every point")
        self.voxel_spin.valueChanged.connect(self._rebuild)
        f.addWidget(QtWidgets.QLabel("Voxel"), 3, 0)
        f.addWidget(self.voxel_spin, 3, 1)

        self.range_spin = QtWidgets.QDoubleSpinBox()
        self.range_spin.setRange(0.0, 50000.0)
        self.range_spin.setValue(0.0)
        self.range_spin.setSuffix(" mm")
        self.range_spin.setToolTip("0 = no limit")
        self.range_spin.valueChanged.connect(self._rebuild)
        f.addWidget(QtWidgets.QLabel("Max range"), 4, 0)
        f.addWidget(self.range_spin, 4, 1)

        # No axis-offset control: the lidar is centred on the shaft, so the
        # standoff is zero and there is nothing to dial in. build_cloud still
        # takes the parameter (it defaults to sp.BEAM_OFFSET_MM = 0) for a rig
        # that one day mounts the sensor off-centre.
        self.lidar_rot_spin = QtWidgets.QDoubleSpinBox()
        self.lidar_rot_spin.setRange(-180.0, 180.0)
        self.lidar_rot_spin.setValue(sp.LIDAR_ROTATION_DEG)
        self.lidar_rot_spin.setSingleStep(90.0)
        self.lidar_rot_spin.setSuffix(" deg CW")
        self.lidar_rot_spin.setToolTip(
            "How far the lidar is rolled about its own spin axis, clockwise.\n"
            "This is the knob that decides which direction within the vertical "
            "scan plane is up. Get it wrong and the slice is tipped: the floor "
            "climbs into the walls and a room comes out as a cone. Try "
            "0 / 90 / 180 / -90 and keep the one where the floor is flat.")
        self.lidar_rot_spin.valueChanged.connect(self._rebuild)
        f.addWidget(QtWidgets.QLabel("Lidar roll"), 5, 0)
        f.addWidget(self.lidar_rot_spin, 5, 1)

        self.reverse_chk = QtWidgets.QCheckBox("Reverse azimuth direction")
        self.reverse_chk.setChecked(sp.LIDAR_REVERSE)
        self.reverse_chk.setToolTip(
            "Which way the lidar's reported angle runs. Getting this wrong "
            "reflects the cloud, and no rotation can undo a reflection.\n"
            "It cannot be worked out from a capture either: a mirrored scan of "
            "a room is exactly as self-consistent as a correct one. Set it by "
            "eye against a scene whose handedness you know -- text on a wall "
            "reading backwards is the giveaway.")
        self.reverse_chk.stateChanged.connect(self._rebuild)
        f.addWidget(self.reverse_chk, 6, 0, 1, 2)

        self.flip_chk = QtWidgets.QCheckBox("Flip upright (180 deg)")
        self.flip_chk.setChecked(sp.FLIP_UPRIGHT)
        self.flip_chk.setToolTip(
            "Turns the finished cloud 180 deg about world X, for a sensor "
            "mounted inverted.\n"
            "A rigid transform, so it changes no measurement -- purely "
            "cosmetic, and safe to toggle on its own.")
        self.flip_chk.stateChanged.connect(self._rebuild)
        f.addWidget(self.flip_chk, 7, 0, 1, 2)

        # --- Rangefinder standoff -------------------------------------------
        # The lidar body is centred on the shaft but the rangefinder inside it
        # is not, and that lateral offset reverses sign between the two halves
        # of each revolution -- which is what splits a flat table into two
        # heights. These two controls are the two ways out; see the module
        # header in scan_proto.py.
        self.spacing_spin = QtWidgets.QDoubleSpinBox()
        self.spacing_spin.setRange(-200.0, 200.0)
        self.spacing_spin.setDecimals(1)
        self.spacing_spin.setSingleStep(1.0)
        self.spacing_spin.setValue(sp.EMITTER_SPACING_MM)
        self.spacing_spin.setSuffix(" mm")
        self.spacing_spin.setToolTip(
            "Distance between the laser diode and the receiver inside the "
            "lidar.\n"
            "The rangefinder's reference point sits between the two optics, "
            "off to one side of the spin axis, and that sideways offset "
            "reverses as the head turns -- so a table scanned by one side of "
            "the lidar comes out higher than the same table scanned by the "
            "other. Half this value is used as the standoff.\n"
            "This is the real fix for that artefact, and the only setting that "
            "removes both the step between the two halves and the gentle bowl "
            "within each one.\n"
            "Measure it with calipers to get close, then tune: the step is "
            "smallest at the true value and grows about equally either side of "
            "it, so if a flat surface still steps, try moving this both up and "
            "down -- the size tells you how far off you are, not which way.")
        self.spacing_spin.valueChanged.connect(self._rebuild)
        f.addWidget(QtWidgets.QLabel("Emitter spacing"), 8, 0)
        f.addWidget(self.spacing_spin, 8, 1)

        self.half_box = QtWidgets.QComboBox()
        for key in (sp.SCAN_HALF_BOTH, sp.SCAN_HALF_A, sp.SCAN_HALF_B):
            self.half_box.addItem(sp.SCAN_HALF_NAMES[key], key)
        self.half_box.setToolTip(
            "Which half of each lidar revolution to keep, split at straight up "
            "and straight down.\n"
            "Keeping one side never mixes the two, so a flat surface cannot "
            "arrive as two sheets. It does NOT replace the spacing above: the "
            "remaining error stops being a step and becomes a gentle bowl, "
            "which is easier to mesh but no more accurate. Set the spacing "
            "first, then use this as insurance against the two optical paths "
            "not being quite symmetric.\n"
            "The cost is half the points, and half a scan plane is a "
            "pole-to-pole arc rather than a full circle -- so it needs a 360 "
            "deg sweep to cover the sphere: set Angle to ±180.\n"
            "A and B are the two sides; pick whichever looks cleaner.")
        self.half_box.currentIndexChanged.connect(self._on_half_changed)
        f.addWidget(QtWidgets.QLabel("Scan half"), 9, 0)
        f.addWidget(self.half_box, 9, 1)

        self.half_lbl = QtWidgets.QLabel("")
        self.half_lbl.setWordWrap(True)
        self.half_lbl.setStyleSheet("color: #e0a030;")
        f.addWidget(self.half_lbl, 10, 0, 1, 2)

        # --- Microstep non-linearity ----------------------------------------
        # The firmware reports the angle it commanded; the rotor sits a little
        # off it, periodically with the full step. See scan_proto.py.
        self.ustep_spin = QtWidgets.QDoubleSpinBox()
        self.ustep_spin.setRange(0.0, 0.2)
        self.ustep_spin.setDecimals(4)
        self.ustep_spin.setSingleStep(0.0025)
        self.ustep_spin.setValue(sp.MICROSTEP_ERROR_DEG)
        self.ustep_spin.setSuffix(" deg")
        self.ustep_spin.setToolTip(
            "How far the rotor sits from the microstep the firmware asked "
            "for.\n"
            "The driver divides each 1.8 deg full step into 16, but the motor "
            "is pulled toward the nearest detent, so those 16 bunch up instead "
            "of dividing it evenly. The sample is then placed at the commanded "
            "azimuth rather than the true one, which slides it sideways along "
            "a wall -- so a flat wall arrives with ripples one full step "
            "apart, faint where the wall faces the sensor and growing toward "
            "its ends. At 3 m along a wall, one microstep of error is a 6 mm "
            "bump.\n"
            "0 disables the correction. Try 0.011 to 0.056 (a tenth to half a "
            "microstep) and wind it up until the ripple flattens. Set the "
            "phase below first -- amplitude alone does nothing at the wrong "
            "phase.\n"
            "Unlike the emitter spacing this knob can invent structure: "
            "bending the azimuth at the full-step period will always find "
            "something to flatten in a noisy cloud. Tune it on one long clean "
            "wall, then confirm the same numbers still help on a different "
            "scan before trusting them.")
        self.ustep_spin.valueChanged.connect(self._rebuild)
        f.addWidget(QtWidgets.QLabel("Microstep error"), 11, 0)
        f.addWidget(self.ustep_spin, 11, 1)

        self.ustep_phase_spin = QtWidgets.QDoubleSpinBox()
        self.ustep_phase_spin.setRange(0.0, 360.0)
        self.ustep_phase_spin.setDecimals(0)
        self.ustep_phase_spin.setSingleStep(15.0)
        self.ustep_phase_spin.setWrapping(True)
        self.ustep_phase_spin.setValue(sp.MICROSTEP_ERROR_PHASE)
        self.ustep_phase_spin.setSuffix(" deg")
        self.ustep_phase_spin.setToolTip(
            "Where within the full step the correction above is applied. "
            "360 here is one 1.8 deg full step, so 90 slides it by four "
            "microsteps.\n"
            "Sweep this at a fixed amplitude until the ripple is at its "
            "weakest, then trim the amplitude. At the wrong phase a larger "
            "amplitude makes the wall worse, not better.")
        self.ustep_phase_spin.valueChanged.connect(self._rebuild)
        f.addWidget(QtWidgets.QLabel("Microstep phase"), 12, 0)
        f.addWidget(self.ustep_phase_spin, 12, 1)

        self.cam_box = QtWidgets.QComboBox()
        self.cam_box.addItem("Orbit (Blender)", CAM_ORBIT)
        self.cam_box.addItem("Fly / FPS (Unity)", CAM_FPS)
        self.cam_box.setToolTip(
            "Orbit: drag swings the camera around a fixed point, and so does "
            "two-finger scroll. The wheel, pinch and ctrl+scroll zoom in and "
            "out of it. "
            "Middle-drag slides that point through the scene, sideways and up "
            "and down, as in Blender -- use it to bring the floor into reach. "
            "On a laptop touchpad, Shift+drag or Alt+drag slides it instead, "
            "and so does a two-finger scroll with Shift held. Best for turning "
            "one object over.\n"
            "Fly: drag looks around from where you stand, WASD walks, "
            "E / Space up and Q / Ctrl down, Shift for four times the speed, "
            "and the wheel sets that speed rather than zooming. The keys work "
            "whenever the pointer is over the 3D view. Best for getting "
            "inside a scanned room.")
        self.cam_box.currentIndexChanged.connect(self._on_cam_mode_changed)
        f.addWidget(QtWidgets.QLabel("Camera"), 13, 0)
        f.addWidget(self.cam_box, 13, 1)

        b = QtWidgets.QPushButton("Reset camera")
        b.clicked.connect(self._frame_cloud)
        f.addWidget(b, 14, 0, 1, 2)
        return g

    def _on_cam_mode_changed(self):
        self.view.set_cam_mode(self.cam_box.currentData())
        if self.cam_box.currentData() == CAM_FPS:
            # Deferred: choosing an item leaves the combo taking focus back
            # *after* this handler returns, so a setFocus() from in here is
            # undone. Flying no longer depends on focus, but starting out with
            # it on the view keeps the arrow keys off the dropdown.
            QtCore.QTimer.singleShot(0, self.view.setFocus)

    def _geometry_group(self):
        """Surface reconstruction settings. Only on screen in Geometry mode."""
        g = QtWidgets.QGroupBox("Geometry")
        f = QtWidgets.QGridLayout(g)

        self.algo_box = QtWidgets.QComboBox()
        # Offer only what this machine can run. Listing Poisson and then
        # failing on Build because open3d is missing is a worse experience
        # than not listing it, and the note below says how to get it back.
        for key in meshing.available_methods():
            self.algo_box.addItem(meshing.METHODS[key][0], key)
        self.algo_box.setToolTip(
            "Poisson: watertight and smooth, best for a solid object.\n"
            "Ball pivoting: keeps the measured points, leaves real gaps open.\n"
            "Alpha shape: crude but dependency-free.")
        self.algo_box.currentIndexChanged.connect(self._on_algo_changed)
        f.addWidget(QtWidgets.QLabel("Method"), 0, 0)
        f.addWidget(self.algo_box, 0, 1)

        self.depth_spin = QtWidgets.QSpinBox()
        self.depth_spin.setRange(5, 12)
        self.depth_spin.setValue(9)
        self.depth_spin.setToolTip(
            "Poisson octree depth: the resolution of the reconstruction.\n"
            "Each step up doubles the detail and roughly quadruples the work. "
            "Past what the cloud actually resolves it just fits the noise.")
        self.depth_lbl = QtWidgets.QLabel("Detail")
        f.addWidget(self.depth_lbl, 1, 0)
        f.addWidget(self.depth_spin, 1, 1)

        self.trim_spin = QtWidgets.QDoubleSpinBox()
        self.trim_spin.setRange(0.0, 40.0)
        self.trim_spin.setValue(3.0)
        self.trim_spin.setSingleStep(0.5)
        self.trim_spin.setSuffix(" x")
        self.trim_spin.setToolTip(
            "Discard surface further than this from any measured point, in "
            "multiples of the point spacing.\n"
            "Poisson always closes its surface, so it bridges the space "
            "behind whatever you scanned -- one wall comes back as a blob "
            "wrapped round it. Trimming cuts that invented part away. Lower "
            "is tighter to the data; 0 leaves the closed blob intact.")
        self.trim_lbl = QtWidgets.QLabel("Trim")
        f.addWidget(self.trim_lbl, 2, 0)
        f.addWidget(self.trim_spin, 2, 1)

        self.radius_spin = QtWidgets.QDoubleSpinBox()
        self.radius_spin.setRange(0.5, 20.0)
        self.radius_spin.setValue(2.0)
        self.radius_spin.setSingleStep(0.5)
        self.radius_spin.setSuffix(" x")
        self.radius_spin.setToolTip(
            "Ball radius, as a multiple of the cloud's own point spacing.\n"
            "Too small and the ball falls through the surface leaving holes; "
            "too large and it bridges across gaps that are really there.")
        self.radius_lbl = QtWidgets.QLabel("Ball radius")
        f.addWidget(self.radius_lbl, 3, 0)
        f.addWidget(self.radius_spin, 3, 1)

        self.alpha_spin = QtWidgets.QDoubleSpinBox()
        self.alpha_spin.setRange(1.0, 40.0)
        self.alpha_spin.setValue(4.0)
        self.alpha_spin.setSingleStep(0.5)
        self.alpha_spin.setSuffix(" x")
        self.alpha_spin.setToolTip(
            "Alpha, as a multiple of the point spacing.\n"
            "Large tends towards the convex hull and swallows concave detail; "
            "small crumbles the surface into scattered fragments.")
        self.alpha_lbl = QtWidgets.QLabel("Alpha")
        f.addWidget(self.alpha_lbl, 4, 0)
        f.addWidget(self.alpha_spin, 4, 1)

        self.budget_spin = QtWidgets.QSpinBox()
        self.budget_spin.setRange(0, 2000000)
        self.budget_spin.setSingleStep(10000)
        self.budget_spin.setValue(meshing.METHOD_BUDGET["poisson"])
        self.budget_spin.setGroupSeparatorShown(True)
        self.budget_spin.setToolTip(
            "Thin the cloud to about this many points on a uniform grid "
            "before reconstructing. 0 feeds the lot.\n"
            "This is the control that decides how long a build takes. Ball "
            "pivoting especially is superlinear -- 45k points is 8 seconds "
            "but 120k is over two minutes -- so the default drops when you "
            "pick it.\n"
            "Thinning on a grid also evens out the density, which is what "
            "lets one alpha or ball radius work across the whole scan "
            "instead of holes far away and mush close up.")
        f.addWidget(QtWidgets.QLabel("Point budget"), 5, 0)
        f.addWidget(self.budget_spin, 5, 1)

        self.smooth_spin = QtWidgets.QSpinBox()
        self.smooth_spin.setRange(0, 20)
        self.smooth_spin.setValue(0)
        self.smooth_spin.setToolTip(
            "Laplacian smoothing passes. The lidar's per-sample noise shows "
            "up as faceting, and a few passes settle it; too many shrink the "
            "model and round off the corners you were measuring.")
        f.addWidget(QtWidgets.QLabel("Smoothing"), 6, 0)
        f.addWidget(self.smooth_spin, 6, 1)

        self.surface_box = QtWidgets.QComboBox()
        self.surface_box.addItems(["shaded", "height", "wireframe"])
        self.surface_box.setToolTip(
            "How the surface is drawn. Shaded reads the shape best; height "
            "keeps the cloud's colour ramp; wireframe shows the triangles.")
        self.surface_box.currentIndexChanged.connect(self._draw_mesh)
        f.addWidget(QtWidgets.QLabel("Surface"), 7, 0)
        f.addWidget(self.surface_box, 7, 1)

        self.mesh_btn = QtWidgets.QPushButton("Build surface")
        self.mesh_btn.setToolTip(
            "Reconstruct from the current cloud. Changing a setting above "
            "rebuilds on its own; this is for after new points have arrived.")
        self.mesh_btn.clicked.connect(self._build_mesh)
        f.addWidget(self.mesh_btn, 8, 0, 1, 2)

        self.mesh_lbl = QtWidgets.QLabel("no surface built")
        self.mesh_lbl.setWordWrap(True)
        self.mesh_lbl.setStyleSheet("color: #9a9a9a;")
        f.addWidget(self.mesh_lbl, 9, 0, 1, 2)

        note = meshing.missing_note()
        if note:
            warn = QtWidgets.QLabel(note.splitlines()[0])
            warn.setWordWrap(True)
            warn.setToolTip(note)
            warn.setStyleSheet("color: #e0a030;")
            f.addWidget(warn, 10, 0, 1, 2)
        else:
            # Which library ran matters when comparing results with someone
            # else's machine, and it is otherwise invisible.
            via = QtWidgets.QLabel(f"via {meshing.backend()}")
            via.setStyleSheet("color: #9a9a9a;")
            f.addWidget(via, 10, 0, 1, 2)

        # Every knob that changes the geometry itself schedules a rebuild.
        # "Surface" is deliberately not among them: it only repaints.
        for w in (self.depth_spin, self.trim_spin, self.radius_spin,
                  self.alpha_spin, self.smooth_spin, self.budget_spin):
            w.valueChanged.connect(self._mesh_param_changed)

        self._on_algo_changed()
        return g

    def _file_group(self):
        g = QtWidgets.QGroupBox("File")
        f = QtWidgets.QVBoxLayout(g)
        row1 = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("Save .ply")
        b1.setToolTip("Saves all original points (full resolution, not voxel-filtered)")
        b1.clicked.connect(self._save_ply)
        row1.addWidget(b1)
        b2 = QtWidgets.QPushButton("Load .ply")
        b2.clicked.connect(self._load_ply)
        row1.addWidget(b2)
        f.addLayout(row1)

        row2 = QtWidgets.QHBoxLayout()
        b3 = QtWidgets.QPushButton("Save .bin")
        b3.setToolTip(
            "Write the raw stream exactly as the device sent it. Every byte "
            "since connecting is kept in memory, so this stays available "
            "without recording to disk as the scan runs -- and it still works "
            "after Clear scene.")
        b3.clicked.connect(self._save_bin)
        row2.addWidget(b3)
        b4 = QtWidgets.QPushButton("Open .bin")
        b4.setToolTip("Re-open a recorded raw stream and rebuild the cloud")
        b4.clicked.connect(self._load_bin)
        row2.addWidget(b4)
        f.addLayout(row2)
        return g

    # --- settings -----------------------------------------------------------
    # Everything on the panel is remembered between runs. The mount geometry is
    # the reason this matters: beam offset, lidar rotation, azimuth direction
    # and the upright flip describe how the hardware is bolted together, so
    # they are dialled in once by eye and then must not change. Re-entering
    # them every launch is how a scan silently gets reconstructed with last
    # week's guess. The capture settings ride along for convenience.

    def _init_settings(self):
        self.settings = FileSettings(SETTINGS_PATH)
        if self.settings.migrate_from_qsettings(
                QtCore.QSettings("LidarScanner", "scanner_ui")):
            self.settings.sync()

        # key -> widget. Keys are stable strings, deliberately not derived from
        # the attribute names, so a later rename cannot quietly orphan a saved
        # value and drop the user back to a default mid-project.
        self._persist = {
            "port": self.port_box,
            "dtr": self.dtr_chk,
            "record": self.record_chk,
            "scan/angle": self.angle_spin,
            "scan/time": self.time_spin,
            "scan/stepped": self.stepped_chk,
            "scan/steps": self.steps_spin,
            "scan/dwell": self.dwell_spin,
            "map/multi": self.multi_chk,
            "map/auto_add": self.auto_add_chk,
            "map/align": self.align_box,
            "map/yaw_hint": self.yaw_spin,
            "map/voxel": self.reg_voxel_spin,
            "view/mode": self.mode_box,
            "view/color_by": self.color_box,
            "view/point_size": self.size_spin,
            "view/camera": self.cam_box,
            "mesh/method": self.algo_box,
            "mesh/depth": self.depth_spin,
            "mesh/trim": self.trim_spin,
            "mesh/radius": self.radius_spin,
            "mesh/alpha": self.alpha_spin,
            "mesh/smooth": self.smooth_spin,
            "mesh/surface": self.surface_box,
            "view/voxel": self.voxel_spin,
            "view/max_range": self.range_spin,
            "geom/lidar_rotation": self.lidar_rot_spin,
            "geom/lidar_reverse": self.reverse_chk,
            "geom/emitter_spacing": self.spacing_spin,
            "geom/scan_half": self.half_box,
            "geom/flip_upright": self.flip_chk,
            "geom/microstep_error": self.ustep_spin,
            "geom/microstep_phase": self.ustep_phase_spin,
        }
        self._restore_settings()

        # Save on every change rather than only on close: the UI talks to
        # hardware, and a session that ends in a crash or an unplugged board
        # should not also cost the geometry.
        for w in self._persist.values():
            sig = getattr(w, "valueChanged", None) or \
                getattr(w, "stateChanged", None) or \
                getattr(w, "currentTextChanged", None)
            sig.connect(self._save_settings)
        # Not in _persist: it is keyed by the meshing method, see _saved_budget.
        self.budget_spin.valueChanged.connect(self._save_settings)

    def _saved_budget(self, method):
        """Stored point budget for one meshing method, or None."""
        # The panel calls _on_algo_changed while it is still being built, which
        # is before _init_settings has run; the defaults stand until then.
        settings = getattr(self, "settings", None)
        if settings is None:
            return None
        val = settings.value(f"mesh/budget/{method}")
        try:
            return None if val is None else int(float(val))
        except (TypeError, ValueError):
            return None

    def _restore_settings(self):
        for key, w in self._persist.items():
            val = self.settings.value(key)
            if val is None:
                continue
            w.blockSignals(True)
            try:
                if isinstance(w, QtWidgets.QCheckBox):
                    # QSettings round-trips bools as the strings "true"/"false"
                    # on some backends and as real bools on others.
                    w.setChecked(val if isinstance(val, bool)
                                 else str(val).lower() in ("true", "1"))
                elif isinstance(w, QtWidgets.QComboBox):
                    i = w.findText(str(val))
                    if i >= 0:
                        w.setCurrentIndex(i)
                elif isinstance(w, QtWidgets.QSpinBox):
                    w.setValue(int(float(val)))
                else:
                    w.setValue(float(val))
            except (TypeError, ValueError):
                pass  # a stale or hand-edited value; the default stands
            finally:
                w.blockSignals(False)
        self._on_mode_changed()
        self._on_half_changed()
        self._on_algo_changed()
        self._on_cam_mode_changed()
        self._on_multi_changed()
        # Last, and unconditionally: it decides which half of the panel is
        # visible, so it has to run even when nothing was restored.
        self._on_view_mode_changed()

    def _save_settings(self, *_):
        for key, w in self._persist.items():
            if isinstance(w, QtWidgets.QCheckBox):
                self.settings.setValue(key, w.isChecked())
            elif isinstance(w, QtWidgets.QComboBox):
                self.settings.setValue(key, w.currentText())
            else:
                self.settings.setValue(key, w.value())
        self.settings.setValue(f"mesh/budget/{self.algo_box.currentData()}",
                               self.budget_spin.value())
        for key, sec in self.sections.items():
            self.settings.setValue(f"fold/{key}", sec.is_expanded())
        self.settings.sync()

    # --- device -------------------------------------------------------------

    def _refresh_ports(self):
        from serial.tools import list_ports
        current = self.port_box.currentText()
        self.port_box.clear()
        for p in list_ports.comports():
            self.port_box.addItem(p.device, p.description)
            idx = self.port_box.count() - 1
            self.port_box.setItemData(idx, f"{p.device} - {p.description}",
                                      TOOLTIP_ROLE)
        if current:
            i = self.port_box.findText(current)
            if i >= 0:
                self.port_box.setCurrentIndex(i)
        if self.port_box.count() == 0:
            self.port_box.addItem("(no ports found)")

    def _toggle_connect(self):
        if self.link:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_box.currentText()
        if not port or port.startswith("("):
            self._log("no serial port selected")
            return

        rec = None
        if self.record_chk.isChecked():
            rec = scan_path(time.strftime("scan_%Y%m%d_%H%M%S.bin"),
                            "autosaves")

        self.cap = sp.Capture()
        # New session, new stream. Reassigned rather than cleared so a reader
        # thread still winding down cannot append into the fresh buffer.
        self.raw = bytearray()
        self.connected_at = time.time()
        self.warned_silent = False
        self.link = SerialLink(port, self.cap, rec,
                               dtr=self.dtr_chk.isChecked(), raw=self.raw,
                               parent=self)
        self.link.event.connect(self._log)
        self.link.status.connect(self._on_status)
        self.link.config.connect(self._on_config)
        self.link.failed.connect(self._on_failed)
        self.link.opened.connect(
            lambda: self.statusBar().showMessage(f"connected to {port}"))
        self.link.start()
        self.connect_btn.setText("Disconnect")
        if rec:
            self._log(f"recording raw stream -> {os.path.basename(rec)}")

    def _disconnect(self):
        if not self.link:
            return
        self.link.stop()
        self.link.wait(2000)
        self.link = None
        self.connect_btn.setText("Connect")
        self.statusBar().showMessage("disconnected")

    def _send(self, text):
        if not self.link:
            self._log("not connected")
            return False
        self.link.send(text + "\n")
        return True

    def _on_mode_changed(self):
        """Grey out the controls the other mode ignores."""
        stepped = self.stepped_chk.isChecked()
        for w in (self.steps_lbl, self.steps_spin,
                  self.dwell_lbl, self.dwell_spin):
            w.setEnabled(stepped)
        # Duration only drives a continuous sweep; in stepped mode the runtime
        # falls out of the dwell windows instead.
        self.time_spin.setEnabled(not stepped)
        self._update_estimate()

    def _on_half_changed(self):
        """One-side scanning needs a full turn; say so rather than silently
        reconstructing half a sphere from half a sweep."""
        one_side = self.half_box.currentData() != sp.SCAN_HALF_BOTH
        if one_side and self.angle_spin.value() < 180.0:
            self.half_lbl.setText(
                "One-side scanning covers only half the scene at this angle - "
                "set Angle to ±180 for a full 360° sweep.")
        else:
            self.half_lbl.setText("")
        self._rebuild()

    def _update_estimate(self):
        if not self.stepped_chk.isChecked():
            self.est_lbl.setText("")
            return
        stops = self.steps_spin.value() + 1
        # Mirrors SCAN_STEP_SETTLE_MS in scanner.h; the travel between stops is
        # on top and depends on the step size.
        secs = stops * (250 + self.dwell_spin.value()) / 1000.0
        arc = 2 * self.angle_spin.value() / self.steps_spin.value()
        self.est_lbl.setText(
            f"{stops} stops of {arc:.2f} deg, at least {secs / 60:.1f} min")

    def _push_settings(self):
        """Send everything on screen to the device. Returns False if offline.

        Called from _start rather than from a button: a scan that ran with
        settings other than the ones displayed was the whole problem with
        having a separate apply step, since forgetting it was silent.
        """
        if not self._send(f"a{self.angle_spin.value():.2f}"):
            return False
        self._send(f"t{self.time_spin.value():.2f}")
        self._send(f"m{1 if self.stepped_chk.isChecked() else 0}")
        if self.stepped_chk.isChecked():
            self._send(f"n{self.steps_spin.value()}")
            self._send(f"d{self.dwell_spin.value()}")
        return True

    def _clear_scene(self):
        """Throw away the points collected so far and start the cloud over.

        Only the decoded samples go: the raw .bin recording keeps running, and
        a scan is a physical event that cannot be repeated, so this must never
        be the thing that loses one.
        """
        self.cap.samples.clear()
        self.cap.telem.clear()
        self.cloud = None
        self.live = None
        self.loaded_rgb = None
        self._built_n = 0
        self.mesh = None
        self._mesh_points = 0
        self.mesh_item.setVisible(False)
        self._update_mesh_label()
        self.scatter.setData(pos=np.zeros((0, 3)))
        self.progress.setValue(0)
        # Scans already added to the map are not part of "the scene": clearing
        # is what happens at the start of every sweep, and it must not be the
        # thing that throws away an hour of mapping. Clear map does that, and
        # only when it is asked for.
        self._compose()
        self.statusBar().showMessage("scene cleared", 3000)
        self._log("scene cleared (the raw stream is kept -- Save .bin still "
                  "has everything)"
                  + (f"; {len(self.scans)} mapped scans kept"
                     if self.scans else ""))

    # --- multi-scan map -----------------------------------------------------

    def _on_multi_changed(self):
        on = self.multi_chk.isChecked()
        for w in (self.auto_add_chk, self.align_box, self.reg_voxel_spin,
                  self.add_btn, self.undo_btn, self.clear_map_btn):
            w.setEnabled(on)
        self._on_align_changed()
        self._update_map_ui()

    def _on_align_changed(self):
        hint = (self.multi_chk.isChecked()
                and self.align_box.currentData() == "hint")
        self.yaw_lbl.setVisible(hint)
        self.yaw_spin.setVisible(hint)

    def _map_cloud(self):
        """The union of every added scan, in the map's frame. None if empty.

        Cached because it is rebuilt only when a scan is added or removed, and
        read on every redraw.
        """
        if not self.scans:
            return None
        if self._map is None:
            xyz = np.vstack([reg.apply(s["T"], s["xyz"]) for s in self.scans])
            dist = np.concatenate([s["dist"] for s in self.scans])
            self._map = (xyz, dist)
        return self._map

    def _compose(self):
        """Assemble what is on screen: the map, plus the sweep not yet added.

        Single-scan mode falls out of this for free -- with no added scans the
        map is empty and this is just the live cloud, which is what it always
        was.
        """
        parts, dists = [], []
        m = self._map_cloud()
        if m is not None:
            parts.append(m[0])
            dists.append(m[1])
        self._map_n = sum(p.shape[0] for p in parts)

        if self._pending is not None:
            # Show the candidate alignment in place, so accepting or rejecting
            # it is a question about something visible rather than about a
            # number.
            parts.append(reg.apply(self._pending["result"]["T"],
                                   self._pending["xyz"]))
            dists.append(self._pending["dist"])
        elif self.live is not None:
            parts.append(self.live[0])
            dists.append(self.live[1])

        if not parts:
            self.cloud = None
            self.scatter.setData(pos=np.zeros((0, 3)))
            self._update_map_ui()
            return
        self.cloud = (np.vstack(parts).astype(np.float32),
                      np.concatenate(dists))
        if len(parts) > 1:
            # Colours loaded from a .ply only describe that one cloud; there is
            # nothing to paint the rest of the map with.
            self.loaded_rgb = None
        self._redraw()
        self._update_map_ui()

    def _update_map_ui(self):
        pend = self._pending is not None
        busy = self._reg_worker is not None and self._reg_worker.isRunning()
        for w in (self.accept_btn, self.reject_btn):
            w.setVisible(pend)
        on = self.multi_chk.isChecked()
        self.add_btn.setEnabled(on and not pend and not busy)
        self.undo_btn.setEnabled(on and bool(self.scans) and not busy)
        self.clear_map_btn.setEnabled(on and bool(self.scans) and not busy)

        if pend:
            return          # the pending message stays until it is resolved
        if not self.scans:
            self.map_lbl.setText("map empty" if on else "")
            return
        n = sum(s["xyz"].shape[0] for s in self.scans)
        self.map_lbl.setText(f"{len(self.scans)} scans in map, {n} points")

    def _add_scan(self):
        """Align the sweep on screen onto the map and add it."""
        if not self.multi_chk.isChecked():
            return
        if self._pending is not None:
            self._log("resolve the pending alignment first")
            return
        if self._reg_worker is not None and self._reg_worker.isRunning():
            self._log("an alignment is already running")
            return
        if self.live is None or not self.live[0].shape[0]:
            self._log("nothing to add - no scan on screen")
            return

        xyz, dist = self.live
        if not self.scans:
            # The first scan defines the frame everything else is measured in,
            # so it goes in exactly as it came out of the scanner.
            self._commit(xyz, dist, reg.identity(), "first scan (reference)")
            self._log(f"map started from {xyz.shape[0]} points - move the rig "
                      f"and scan again")
            return

        target = self._map_cloud()[0]
        params = {"voxel": self.reg_voxel_spin.value()}
        mode = self.align_box.currentData()
        if mode == "small":
            params["init"] = reg.identity()
        elif mode == "hint":
            # The operator knows how far they turned the rig since the *last*
            # scan; the map's frame is the *first* one, and after a few scans
            # those are nowhere near each other. Asking for the turn relative
            # to the map would be asking them to do the bookkeeping, and
            # getting it 40 degrees wrong is enough to misalign the scan -- so
            # add the previous scan's heading here instead.
            prev = self.scans[-1]["T"]
            prev_yaw = np.degrees(np.arctan2(prev[1, 0], prev[0, 0]))
            params["yaw_hint"] = prev_yaw + self.yaw_spin.value()

        self.map_lbl.setText("aligning...")
        self._reg_worker = RegisterWorker(xyz.astype(np.float64),
                                          target.astype(np.float64),
                                          params, self)
        self._reg_worker.note.connect(
            lambda s: self.map_lbl.setText(f"aligning: {s}"))
        self._reg_worker.done.connect(self._on_register_done)
        self._reg_worker.failed.connect(self._on_register_failed)
        self._reg_worker.start()
        self._update_map_ui()

    def _on_register_failed(self, msg):
        self._log(f"alignment failed: {msg}")
        self.map_lbl.setText("alignment failed - see log")
        self._update_map_ui()

    def _on_register_done(self, result):
        if self.live is None:
            return
        level, why = reg.verdict(result)
        xyz, dist = self.live
        self._pending = {"result": result, "xyz": xyz, "dist": dist}
        self._log(f"alignment: {reg.pose_summary(result['T'])}; {why}")

        colour, lead = {
            reg.GOOD: ("#66bb6a", "Looks right"),
            reg.CHECK: ("#ffa726", "Worth a look"),
            reg.POOR: ("#ef5350", "Probably wrong"),
        }[level]
        self.map_lbl.setStyleSheet(f"color: {colour};")
        self.map_lbl.setText(
            f"{lead} -- {why}. The new scan is drawn in orange: check it lines "
            "up with the blue map, then keep or discard it.")
        self._compose()
        self._update_map_ui()

        # Auto-add commits only what is clearly right. Anything less stops for
        # a look: a wrong merge corrupts the map silently and there is no way
        # to tell afterwards which scan did it.
        if level == reg.GOOD and self.auto_add_chk.isChecked():
            self._accept_pending()
        elif self.auto_add_chk.isChecked():
            self._log("not added automatically - look it over and press Keep, "
                      "or try a different Movement setting")

    def _accept_pending(self):
        if self._pending is None:
            return
        p = self._pending
        self._pending = None
        self.map_lbl.setStyleSheet("color: #9a9a9a;")
        self._commit(p["xyz"], p["dist"], p["result"]["T"],
                     f"scan {len(self.scans) + 1}")
        self._log(f"added scan {len(self.scans)} to the map "
                  f"({p['xyz'].shape[0]} points)")

    def _reject_pending(self):
        if self._pending is None:
            return
        self._pending = None
        self.map_lbl.setStyleSheet("color: #9a9a9a;")
        self._log("alignment discarded - the scan is still on screen, so you "
                  "can try again with a different Movement setting")
        self._compose()
        self._update_map_ui()

    def _commit(self, xyz, dist, T, label):
        self.scans.append({"xyz": xyz, "dist": dist, "T": np.asarray(T, float),
                           "label": label})
        self._map = None
        # The sweep has become part of the map; leaving it as the live cloud
        # too would draw it twice and offer it for adding a second time.
        self.live = None
        self._compose()
        self._mesh_param_changed()

    def _undo_scan(self):
        if not self.scans:
            return
        s = self.scans.pop()
        self._map = None
        self._log(f"removed {s['label']} from the map "
                  f"({s['xyz'].shape[0]} points)")
        self._compose()
        self._update_map_ui()

    def _clear_map(self):
        if not self.scans:
            return
        n = len(self.scans)
        self.scans.clear()
        self._map = None
        self._pending = None
        self.map_lbl.setStyleSheet("color: #9a9a9a;")
        self._log(f"map cleared ({n} scans dropped)")
        self._compose()
        self._update_map_ui()

    def _start(self):
        # Push the settings first so the sweep always runs with what is on
        # screen, rather than whatever the device happened to still hold.

        # A new sweep answers whatever question the pending alignment was
        # asking, so it does not survive into it.
        if self._pending is not None:
            self._log("starting a new scan - pending alignment discarded")
            self._pending = None
            self.map_lbl.setStyleSheet("color: #9a9a9a;")

        delay = self.delay_spin.value()
        if delay > 0:
            self._begin_start_countdown(delay)
        else:
            self._begin_scan()

    def _begin_start_countdown(self, secs):
        """Count down `secs` seconds before actually launching the sweep.

        Lets the operator clear the scan volume. Start is disabled while the
        countdown runs; Stop cancels it.
        """
        self._countdown_left = secs
        self.start_btn.setEnabled(False)
        self.state_lbl.setText(f"starting in {secs} s")
        if not hasattr(self, "_countdown_timer"):
            self._countdown_timer = QtCore.QTimer(self)
            self._countdown_timer.setInterval(1000)
            self._countdown_timer.timeout.connect(self._on_countdown_tick)
        self._countdown_timer.start()

    def _on_countdown_tick(self):
        self._countdown_left -= 1
        if self._countdown_left <= 0:
            self._cancel_countdown()
            self._begin_scan()
        else:
            self.state_lbl.setText(f"starting in {self._countdown_left} s")

    def _cancel_countdown(self):
        if getattr(self, "_countdown_timer", None) is not None:
            self._countdown_timer.stop()
        self.start_btn.setEnabled(True)

    def _begin_scan(self):
        self._clear_scene()

        if self._push_settings():
            self._send("s")
            self.sweep_started = None
            self.progress.setValue(0)

    def _stop(self):
        if getattr(self, "_countdown_timer", None) is not None \
                and self._countdown_timer.isActive():
            self._cancel_countdown()
            self.state_lbl.setText("idle")
            return
        self._send("x")

    # --- signals ------------------------------------------------------------

    def _log(self, text):
        self.log.appendPlainText(text)

    def _on_failed(self, msg):
        self._log(f"serial error: {msg}")
        self.statusBar().showMessage(f"error: {msg}")
        self._disconnect()

    def _on_config(self, cfg):
        deg = cfg[sp.CFG_DEGREES]
        secs = cfg[sp.CFG_TIME]
        stepped = int(cfg[sp.CFG_MODE]) == sp.MODE_STEPPED
        # Adopt the device's values only for fields the user has never set.
        # The board reports its own defaults on connect, and adopting those
        # wholesale overwrote the restored angle/duration every session -- the
        # settings looked like they were not being saved at all. What is on
        # screen wins instead; _push_settings sends it before the scan starts.
        for key, w, val in (("scan/angle", self.angle_spin, deg),
                            ("scan/time", self.time_spin, secs),
                            ("scan/steps", self.steps_spin,
                             int(cfg[sp.CFG_STEPS])),
                            ("scan/dwell", self.dwell_spin,
                             int(cfg[sp.CFG_CAPTURE_MS])),
                            ("scan/stepped", self.stepped_chk, stepped)):
            if self.settings.value(key) is not None:
                continue
            w.blockSignals(True)
            if isinstance(w, QtWidgets.QCheckBox):
                w.setChecked(val)
            else:
                w.setValue(val)
            w.blockSignals(False)
        self._on_mode_changed()

        self._log(f"device config: ±{deg:.1f} deg, "
                  + (f"stepped, {int(cfg[sp.CFG_STEPS]) + 1} stops, "
                     f"{int(cfg[sp.CFG_SETTLE_MS])}/"
                     f"{int(cfg[sp.CFG_CAPTURE_MS])} ms settle/capture"
                     if stepped else f"continuous over {secs:.1f} s"))

    def _on_status(self, telem):
        # Indexed via the named constants in scan_proto, because the record has
        # already grown once and hand-counted offsets silently drifted.
        self.state = int(telem[sp.TEL_STATE])
        # Note the sweep finishing, but do not act on it here: the last samples
        # are still arriving behind the telemetry that announced it, and adding
        # the scan now would map a cloud missing its tail. _refresh fires it
        # once the built cloud has caught up with the capture.
        if self.state == sp.STATE_DONE and self._last_state != sp.STATE_DONE:
            self._sweep_done = True
        self._last_state = self.state
        platform = telem[sp.TEL_PLATFORM]
        dropped = int(telem[sp.TEL_DROPPED])
        self.platform = float(platform)
        name = sp.STATE_NAMES.get(self.state, "?")
        self.state_lbl.setText(
            f"{name} - platform {platform:+.1f} deg"
            + (f" - {dropped} dropped" if dropped else ""))

        # Progress is the shaft's position across the sweep in both modes:
        # stepped runs through the same -span..+span, just discontinuously.
        if self.state in sp.SWEEP_STATES:
            span = self.angle_spin.value()
            frac = (platform + span) / (2 * span) if span else 0.0
            self.progress.setValue(int(np.clip(frac, 0, 1) * 100))
        elif self.state == sp.STATE_DONE:
            self.progress.setValue(100)

        if dropped:
            self.statusBar().showMessage(
                f"{dropped} samples dropped - host not keeping up", 3000)

    # --- cloud --------------------------------------------------------------

    def _refresh(self):
        """Watchdog plus live rebuild, on the GUI timer.

        The rebuild triggers on new samples arriving, not on the device's
        reported state. Gating it on the state was wrong in both directions:
        the cloud stopped updating the moment the device went back to IDLE at
        the end of a scan -- which is why the finished scan showed nothing
        until you nudged a spinbox and called _rebuild by hand -- and any
        state this list forgot (every stepped-mode state, once those existed)
        silently froze the live preview too.

        Sample count is the honest trigger: it is exactly the thing that makes
        the previous cloud stale, and it does not care what the state machine
        is called this week.
        """
        self._check_link()
        if self.link and len(self.cap) != self._built_n:
            self._rebuild()
        elif getattr(self, "_sweep_done", False):
            # The cloud now holds every sample of the finished sweep.
            self._sweep_done = False
            if self.multi_chk.isChecked() and self.auto_add_chk.isChecked():
                self._add_scan()

    def _check_link(self):
        """Report a connected-but-silent link instead of looking idle.

        An open port that never delivers a byte is the single most confusing
        failure here, because nothing raises: the UI sat there looking
        connected and simply did nothing. Silence is now a reported state.
        """
        if not self.link:
            self.link_lbl.setText("not connected")
            return

        n = self.link.bytes_in
        age = time.time() - self.connected_at
        self.link_lbl.setText(
            f"{n} bytes in, {self.link.cmds_sent} commands sent"
            + (f", {len(self.cap)} samples" if len(self.cap) else ""))

        if n == 0 and age > 3.0 and not self.warned_silent:
            self.warned_silent = True
            self.link_lbl.setStyleSheet("color: #ef5350; font-weight: bold;")
            self.link_lbl.setText(f"NO DATA after {age:.0f}s - see log")
            self._log(
                "--- port is open but the device has sent nothing ---\n"
                f"  DTR is currently {'ON' if self.dtr_chk.isChecked() else 'OFF'}"
                " - try toggling it, then reconnect.\n"
                "  The RP2040's USB serial stays mute until the host asserts"
                " DTR, so this looks identical to dead firmware.\n"
                "  Also check: no other program holds the port; the board is"
                " running the app and not the UF2 bootloader.\n"
                "  For a direct test outside this UI, run:\n"
                f"    python serial_probe.py --port {self.link.port}")
        elif n > 0:
            self.link_lbl.setStyleSheet("")

    def _rebuild(self):
        if not len(self.cap):
            return
        # Claim the count before building, not after: a build that throws below
        # would otherwise be retried on every one of the 4 Hz ticks for the
        # rest of the session.
        self._built_n = len(self.cap)

        try:
            xyz, dist, _ = sp.build_cloud(
                self.cap,
                max_range=self.range_spin.value() or None,
                lidar_rotation=self.lidar_rot_spin.value(),
                lidar_reverse=self.reverse_chk.isChecked(),
                flip_upright=self.flip_chk.isChecked(),
                emitter_spacing=self.spacing_spin.value(),
                half=self.half_box.currentData(),
                microstep_error=self.ustep_spin.value(),
                microstep_phase=self.ustep_phase_spin.value())
        except SystemExit:
            return  # no sweep data yet
        if self.voxel_spin.value():
            xyz, (dist,) = sp.voxel_downsample(xyz, [dist],
                                               self.voxel_spin.value())
        self.live = (xyz, dist)
        self.loaded_rgb = None
        self._compose()

    # --- geometry mode ------------------------------------------------------

    def _geometry_mode(self):
        return self.mode_box.currentText() == "Geometry"

    def _on_view_mode_changed(self):
        """Swap the panel and the rendering between points and surface."""
        geom = self._geometry_mode()
        for w in (self.color_lbl, self.color_box,
                  self.size_lbl, self.size_spin):
            w.setVisible(not geom)
        self.geom_group.setVisible(geom)

        self.scatter.setVisible(not geom)
        self.mesh_item.setVisible(geom and self.mesh is not None)
        if geom and self.mesh is None and self.cloud is not None:
            # Entering the mode is a request to see a surface, and an empty
            # view with a Build button is a worse answer than just building it.
            self._build_mesh()
        else:
            self._update_mesh_label()

    def _on_algo_changed(self):
        """Show only the parameters the chosen method actually reads."""
        method = self.algo_box.currentData()
        for w in (self.depth_lbl, self.depth_spin,
                  self.trim_lbl, self.trim_spin):
            w.setVisible(method == "poisson")
        for w in (self.radius_lbl, self.radius_spin):
            w.setVisible(method == "bpa")
        for w in (self.alpha_lbl, self.alpha_spin):
            w.setVisible(method == "alpha")

        # Adopt this method's budget. The methods' costs differ by an order of
        # magnitude at the same point count, so carrying one number across a
        # switch means either wasting Poisson's headroom or walking into a
        # ball-pivoting build that takes twenty minutes. The budget is stored
        # per method for the same reason: a single saved number would be reset
        # to the default of whichever method was restored.
        want = self._saved_budget(method) or meshing.METHOD_BUDGET.get(method)
        if want and self.budget_spin.value() != want:
            self.budget_spin.blockSignals(True)
            self.budget_spin.setValue(want)
            self.budget_spin.blockSignals(False)
        self._mesh_param_changed()

    def _mesh_param_changed(self, *_):
        if self._geometry_mode() and self.cloud is not None:
            self._mesh_debounce.start()

    def _build_mesh(self):
        if self.cloud is None or not self.cloud[0].size:
            self.mesh_lbl.setText("no cloud to reconstruct")
            return
        if self._mesh_worker is not None and self._mesh_worker.isRunning():
            # Rather than queue: the run in flight is already using older
            # settings than the ones on screen, so finishing it and then
            # building again would show a stale surface first and cost twice
            # the wait. The debounce timer will bring us back here.
            self._mesh_debounce.start()
            return

        method = self.algo_box.currentData()
        params = dict(depth=self.depth_spin.value(),
                      trim_mult=self.trim_spin.value(),
                      radius_mult=self.radius_spin.value(),
                      alpha_mult=self.alpha_spin.value(),
                      smooth_iters=self.smooth_spin.value(),
                      budget=self.budget_spin.value())
        xyz = self.cloud[0]
        self._mesh_points = xyz.shape[0]
        self.mesh_btn.setEnabled(False)
        self.mesh_lbl.setText(f"reconstructing from {xyz.shape[0]} points...")
        self.statusBar().showMessage("building surface...", 0)

        self._mesh_worker = MeshWorker(xyz, method, params, parent=self)
        self._mesh_worker.done.connect(self._on_mesh_done)
        self._mesh_worker.failed.connect(self._on_mesh_failed)
        self._mesh_worker.start()

    def _on_mesh_done(self, verts, faces, info):
        self.mesh_btn.setEnabled(True)
        self._mesh_worker = None
        self._mesh_info = info
        if not len(faces):
            self.mesh = None
            self.mesh_item.setVisible(False)
            self.mesh_lbl.setText(
                "reconstruction produced no faces -- try a larger Alpha or "
                "Ball radius, or a lower Detail")
            self.statusBar().showMessage("surface is empty", 5000)
            return
        self.mesh = (verts, faces)
        self._draw_mesh()
        thinned = (f", thinned {info['input']} -> {info['used']} points "
                   f"({info['voxel']:.1f} mm grid)"
                   if info.get("voxel") else "")
        self._log(f"surface: {len(verts)} vertices, {len(faces)} triangles "
                  f"in {info['seconds']:.1f}s{thinned}")
        self.statusBar().showMessage(f"{len(faces)} triangles", 0)

    def _on_mesh_failed(self, msg):
        self.mesh_btn.setEnabled(True)
        self._mesh_worker = None
        self.mesh_lbl.setText(msg)
        self._log(f"reconstruction failed: {msg}")
        self.statusBar().showMessage("reconstruction failed", 5000)

    def _draw_mesh(self):
        if self.mesh is None:
            return
        verts, faces = self.mesh
        style = self.surface_box.currentText()

        kwargs = {}
        if style == "height":
            # Same ramp the points use, so switching modes does not also
            # change what the colours mean.
            kwargs["vertexColors"] = cloud_io.colorize(verts[:, 2])
        md = gl.MeshData(vertexes=verts, faces=faces, **kwargs)

        self.mesh_item.setMeshData(meshdata=md)
        self.mesh_item.setShader(self._shader)
        # Wireframe wants the faces off, or the edges are buried in the fill.
        self.mesh_item.opts["drawFaces"] = style != "wireframe"
        self.mesh_item.opts["drawEdges"] = style == "wireframe"
        # A near-neutral base: the shader supplies the colour variation, so a
        # tinted base would fight the key/fill split rather than add to it.
        self.mesh_item.setColor((0.78, 0.80, 0.84, 1.0) if style == "shaded"
                                else (1.0, 1.0, 1.0, 1.0))
        self.mesh_item.setVisible(self._geometry_mode())
        self.mesh_item.update()
        self._update_mesh_label()

    def _update_mesh_label(self):
        if self.mesh is None:
            self.mesh_lbl.setText("no surface built")
            return
        verts, faces = self.mesh
        txt = f"{len(faces)} triangles, {len(verts)} vertices"
        info = getattr(self, "_mesh_info", None) or {}
        if info.get("voxel"):
            # The thinning changes the result, so it has to be visible rather
            # than something the user discovers by wondering where detail went.
            txt += (f"\nfrom {info['used']} of {info['input']} points "
                    f"({info['voxel']:.0f} mm grid)")
        if info.get("seconds"):
            txt += f", {info['seconds']:.1f}s"
        # The surface is a snapshot; the cloud behind it keeps growing during a
        # scan. Say so, rather than letting a stale surface pass for current.
        if self.cloud is not None and self.cloud[0].shape[0] != self._mesh_points:
            txt += (f"\nbuilt from {self._mesh_points} points, cloud now has "
                    f"{self.cloud[0].shape[0]} -- press Build surface")
        self.mesh_lbl.setText(txt)

    # --- drawing ------------------------------------------------------------

    def _redraw(self):
        if self.cloud is None:
            return
        xyz, dist = self.cloud
        if self._pending is not None:
            # Judging an alignment means seeing which points came from where:
            # one ramp over the union hides the seam, which is the only thing
            # worth looking at. Map cool, candidate warm -- where they overlap
            # correctly the two interleave, and where they do not it is obvious.
            n_map = getattr(self, "_map_n", 0)
            rgba = np.empty((xyz.shape[0], 4), np.float32)
            rgba[:n_map] = (0.30, 0.55, 0.75, 1.0)
            rgba[n_map:] = (1.00, 0.52, 0.10, 1.0)
        elif self.loaded_rgb is not None:
            rgba = np.empty((xyz.shape[0], 4), np.float32)
            rgba[:, :3] = self.loaded_rgb
            rgba[:, 3] = 1.0
        else:
            v = xyz[:, 2] if self.color_box.currentText() == "height" else dist
            rgba = cloud_io.colorize(v)
        self.scatter.setData(pos=xyz.astype(np.float32), color=rgba,
                             size=self.size_spin.value())
        if self._geometry_mode():
            # The surface is what is on screen, so the cloud's own status line
            # would only overwrite the triangle count. Keeping the scatter
            # fed anyway makes switching back instant.
            self._update_mesh_label()
            return
        if xyz.shape[0]:
            self.statusBar().showMessage(f"{xyz.shape[0]} points", 0)
        else:
            # Silently drawing nothing looks like a hung viewer; say why.
            self.statusBar().showMessage(
                "no points -- check Voxel and Max range", 0)

    def _frame_cloud(self):
        if self.cloud is None or not self.cloud[0].size:
            return
        xyz = self.cloud[0]
        self.view.setCameraPosition(
            pos=pg.Vector(*xyz.mean(axis=0)),
            distance=float(np.percentile(np.linalg.norm(xyz, axis=1), 95)) * 2.5)

    # --- files --------------------------------------------------------------

    def _save_ply(self):
        if not len(self.cap) and not self.scans:
            self._log("nothing to save - no capture data")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save point cloud",
            scan_path(time.strftime("scan_%Y%m%d_%H%M%S.ply")),
            "PLY point cloud (*.ply)")
        if not path:
            return

        # A map is several sweeps that have each been placed by a transform, so
        # it cannot be rebuilt from the current capture the way a single scan
        # can -- the capture only holds the last one. Write what is on screen.
        if self.scans:
            xyz, dist = self.cloud
            cloud_io.export_ply(path, xyz, dist)
            self._log(f"saved map of {len(self.scans)} scans, "
                      f"{xyz.shape[0]} points -> {os.path.basename(path)}")
            return

        try:
            xyz, dist, _ = sp.build_cloud(
                self.cap,
                max_range=self.range_spin.value() or None,
                lidar_rotation=self.lidar_rot_spin.value(),
                lidar_reverse=self.reverse_chk.isChecked(),
                flip_upright=self.flip_chk.isChecked(),
                emitter_spacing=self.spacing_spin.value(),
                half=self.half_box.currentData(),
                microstep_error=self.ustep_spin.value(),
                microstep_phase=self.ustep_phase_spin.value())
        except SystemExit:
            self._log("failed to build cloud for saving")
            return
        cloud_io.export_ply(path, xyz, dist)
        self._log(f"saved {xyz.shape[0]} points -> {os.path.basename(path)}")

    def _save_bin(self):
        """Write the exact bytes the device sent.

        Deliberately not rebuilt from the decoded records. Re-encoding what the
        parser produced loses whatever it did not keep -- the event lines have
        no tuple at all -- and it has to invent an interleaving for records
        whose real order is only knowable from the stream itself. Copying the
        bytes is both lossless and impossible to get subtly wrong.
        """
        # Snapshot the length once: the reader thread keeps appending, and
        # slicing to a length already observed is what makes this safe to do
        # mid-scan without a lock.
        n = len(self.raw)
        if not n:
            self._log("nothing to save - no raw stream captured")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save raw capture",
            scan_path(time.strftime("scan_%Y%m%d_%H%M%S.bin")),
            "Raw capture (*.bin)")
        if not path:
            return
        try:
            with open(path, "wb") as fh:
                fh.write(memoryview(self.raw)[:n])
        except Exception as exc:
            self._log(f"save failed: {exc}")
            return
        self._log(f"saved {n} bytes ({len(self.cap)} samples) "
                  f"-> {os.path.basename(path)}")

    def _save_mesh(self):
        verts, faces = self.mesh
        path, filt = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save surface",
            scan_path(time.strftime("scan_%Y%m%d_%H%M%S.ply")),
            "PLY mesh (*.ply);;STL mesh (*.stl)")
        if not path:
            return
        # Honour the chosen filter even when the typed name says otherwise:
        # the extension is what every downstream tool actually dispatches on.
        if "stl" in filt.lower() and not path.lower().endswith(".stl"):
            path += ".stl"
        elif "ply" in filt.lower() and not path.lower().endswith(".ply"):
            path += ".ply"

        if path.lower().endswith(".stl"):
            meshing.export_stl(path, verts, faces)
        else:
            rgb = (cloud_io.colorize(verts[:, 2])[:, :3] * 255).astype(np.uint8)
            meshing.export_ply(path, verts, faces, rgb)
        self._log(f"saved {len(faces)} triangles -> {os.path.basename(path)}")

    def _load_ply(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load point cloud", SCANS_DIR, "PLY point cloud (*.ply)")
        if not path:
            return
        try:
            xyz, rgb = cloud_io.load_ply(path)
        except Exception as exc:
            self._log(f"load failed: {exc}")
            return
        # A loaded cloud has no range channel, so derive one for the colour
        # ramp when the file carried no vertex colours.
        #
        # It lands as the live cloud rather than straight onto the display, so
        # that in multi-scan mode a scan saved earlier can be aligned onto the
        # map exactly like one that just came off the device.
        self.live = (xyz, np.linalg.norm(xyz, axis=1))
        self.loaded_rgb = rgb
        self._compose()
        self._frame_cloud()
        self._mesh_param_changed()
        self._log(f"loaded {xyz.shape[0]} points from {os.path.basename(path)}")

    def _load_bin(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open raw capture", SCANS_DIR, "Raw capture (*.bin)")
        if not path:
            return
        try:
            with open(path, "rb") as fh:
                data = fh.read()
        except Exception as exc:
            self._log(f"load failed: {exc}")
            return
        # Cleared in place rather than reassigned, unlike _connect. A live
        # SerialLink was handed these two objects at construction and its
        # reader thread decodes straight into them, so rebinding self.cap here
        # left the link filling a Capture nothing on screen looks at any more:
        # the device stayed connected, bytes kept arriving, and not one new
        # point ever appeared. _connect can reassign because it builds the link
        # afterwards; this cannot.
        self.cap.samples.clear()
        self.cap.telem.clear()
        self.cap.events.clear()
        self.cap.config = None
        # Keep the file's bytes as the current stream, so a capture that was
        # opened rather than recorded can still be saved back out.
        self.raw[:] = data
        # sweep_only mirrors the live link: the device streams frames in every
        # state, so without it the idle frames either side of the sweep get
        # decoded into the cloud too -- which the live path never shows.
        sp.StreamParser(self.cap, echo_events=False,
                        sweep_only=True).feed(data)
        # And a file holds the *session*, which may be several sweeps: live,
        # _start clears the scene each time, so only the last one is ever on
        # screen. Without this the sweeps pile into one cloud, which is why a
        # reloaded .bin looked like two scans mushed together while the .ply --
        # written from the already-cleared scene -- came back fine.
        dropped = sp.keep_last_sweep(self.cap)
        self._log(f"{len(self.cap)} samples from {os.path.basename(path)}"
                  + (f" ({dropped} from earlier sweeps ignored)"
                     if dropped else ""))
        if self.cap.config:
            self._on_config(self.cap.config)
        self._rebuild()
        self._frame_cloud()
        self._mesh_param_changed()

    def closeEvent(self, ev):
        self._save_settings()
        self._disconnect()
        # Let a reconstruction finish before the window it belongs to goes
        # away: tearing down a parented QThread mid-run aborts the process.
        if self._mesh_worker is not None and self._mesh_worker.isRunning():
            self._mesh_worker.wait(5000)
        if self._reg_worker is not None and self._reg_worker.isRunning():
            self._reg_worker.wait(5000)
        super().closeEvent(ev)


def apply_dark_theme(app):
    """Force the dark palette on every machine.

    Qt does not follow the Windows dark-mode setting, and the platform styles
    disagree about what to do with a palette they were not built for, so the
    look drifted between checkouts. Fusion is the one style that honours a
    custom palette everywhere, so pin both.
    """
    app.setStyle("Fusion")

    bg = QtGui.QColor(43, 43, 43)
    base = QtGui.QColor(30, 30, 30)
    text = QtGui.QColor(220, 220, 220)
    disabled = QtGui.QColor(128, 128, 128)
    highlight = QtGui.QColor(42, 130, 218)

    def role(name):
        return _enum(QtGui.QPalette, f"ColorRole.{name}", name)

    pal = QtGui.QPalette()
    for name, c in (("Window", bg), ("WindowText", text),
                    ("Base", base), ("AlternateBase", bg),
                    ("ToolTipBase", bg), ("ToolTipText", text),
                    ("Text", text), ("Button", bg),
                    ("ButtonText", text), ("BrightText", QtGui.QColor("red")),
                    ("Link", highlight), ("Highlight", highlight),
                    ("HighlightedText", QtGui.QColor(0, 0, 0))):
        pal.setColor(role(name), c)

    greyed = _enum(QtGui.QPalette, "ColorGroup.Disabled", "Disabled")
    for name in ("WindowText", "Text", "ButtonText", "HighlightedText"):
        pal.setColor(greyed, role(name), disabled)
    app.setPalette(pal)

    # The 2D plots keep their own colours, independent of the widget palette.
    pg.setConfigOption("background", base)
    pg.setConfigOption("foreground", text)


def main():
    app = pg.mkQApp("3D Lidar Scanner")
    apply_dark_theme(app)
    ui = ScannerUI()
    ui.show()
    sys.exit(app.exec_() if hasattr(app, "exec_") else app.exec())


if __name__ == "__main__":
    main()
