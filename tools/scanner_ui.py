#!/usr/bin/env python3
"""Single-window front end for the 3D lidar scanner.

    python scanner_ui.py

Connect to the device, set the sweep, run it, watch the cloud build, then save
or reload it as .ply. Everything the command-line tools do is here, and they
share the same reconstruction code (scan_proto) so results are identical.

The View panel's Mode dropdown switches between the raw point cloud and a
reconstructed triangle surface; the settings below it follow the mode.

Needs: pyqtgraph, PyQt5, pyserial, numpy.
Optional: pymeshlab, for Poisson and ball-pivoting surface reconstruction
(open3d is used instead if that is what the machine has).
"""

import os
import queue
import sys
import time

import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

import scan_proto as sp
import scan3d
import meshing

BAUD = 115200


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
        # every frame and `scan3d --all` can still reconstruct them.
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
        self.mesh = None           # (verts, faces) currently displayed
        self._mesh_worker = None
        self._mesh_points = 0      # cloud size the mesh was built from
        self.state = sp.STATE_IDLE
        # Cached quaternion convention, resolved once per capture. The overlay
        # has to apply the same one the cloud did or it contradicts the points.
        self._transpose = None
        self.platform = 0.0        # last commanded tilt the device reported
        self._last_quat = None     # so the triad can be redrawn off-tick
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
                           ("view", self._view_group()),
                           ("geometry", self._geometry_group()),
                           ("file", self._file_group())):
            sec = Collapsible(group)
            self.sections[key] = sec
            side.addWidget(sec)
        # _on_view_mode_changed shows and hides this wholesale, header and all.
        self.geom_group = self.sections["geometry"]

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setPlaceholderText("device messages")
        self.log.setMinimumHeight(120)
        side.addWidget(self.log, 1)

        # --- 3D view ---
        self.view = gl.GLViewWidget()
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

        # Live IMU attitude: a triad of the sensor's own axes drawn in world
        # space. X red, Y green, Z blue. Z is the lidar's spin axis, so the
        # blue arm is the one that tips as the platform sweeps.
        self.axis_items = []
        for colour in ((1, 0.2, 0.2, 1), (0.2, 1, 0.2, 1), (0.3, 0.5, 1, 1)):
            it = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=colour,
                                   width=3.0, antialias=True)
            self.view.addItem(it)
            self.axis_items.append(it)
        # The scan plane the lidar actually sweeps, as a ring at the beam's
        # standoff, so the 59 mm offset is visible rather than notional.
        self.plane_item = gl.GLLinePlotItem(pos=np.zeros((0, 3)),
                                            color=(1, 1, 1, 0.35), width=1.0,
                                            antialias=True)
        self.view.addItem(self.plane_item)

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
            "Write a scan_*.bin in the working directory as the scan runs, "
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
        self.angle_spin.setRange(1.0, 90.0)
        self.angle_spin.setValue(30.0)
        self.angle_spin.setSuffix(" deg")
        self.angle_spin.setToolTip("Half-sweep: the platform runs -this to +this")
        f.addWidget(QtWidgets.QLabel("Angle  ±"), 0, 0)
        f.addWidget(self.angle_spin, 0, 1)

        self.time_spin = QtWidgets.QDoubleSpinBox()
        self.time_spin.setRange(2.0, 600.0)
        self.time_spin.setValue(30.0)
        self.time_spin.setSuffix(" s")
        f.addWidget(QtWidgets.QLabel("Duration"), 1, 0)
        f.addWidget(self.time_spin, 1, 1)

        # Stepped mode: the platform stops at each of these before capturing,
        # so the duration above no longer describes the runtime -- the label
        # updates to say what it will actually take.
        self.stepped_chk = QtWidgets.QCheckBox("Stepped (stop at each angle)")
        self.stepped_chk.setToolTip(
            "Moves, stops, waits out the ring-down, averages the IMU with the "
            "platform genuinely still, then captures against that one pose.\n"
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
        self.est_lbl.setStyleSheet("color: #666;")
        f.addWidget(self.est_lbl, 5, 0, 1, 2)
        for w in (self.steps_spin, self.dwell_spin, self.angle_spin):
            w.valueChanged.connect(self._update_estimate)

        self.start_btn = QtWidgets.QPushButton("Start scan")
        self.start_btn.setToolTip(
            "Pushes the settings above to the device, then sweeps. There is no "
            "separate apply step -- what is on screen is what runs.")
        self.start_btn.clicked.connect(self._start)
        f.addWidget(self.start_btn, 6, 0)
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.clicked.connect(self._stop)
        f.addWidget(self.stop_btn, 6, 1)

        self.home_btn = QtWidgets.QPushButton("Set home")
        self.home_btn.setToolTip("Call the current platform angle zero")
        self.home_btn.clicked.connect(lambda: self._send("h"))
        f.addWidget(self.home_btn, 7, 0)
        self.cal_btn = QtWidgets.QPushButton("Calibrate IMU")
        self.cal_btn.setToolTip("Re-bias the gyro. Keep the rig still.")
        self.cal_btn.clicked.connect(self._calibrate)
        f.addWidget(self.cal_btn, 7, 1)

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

        self.orient_chk = QtWidgets.QCheckBox("Show live IMU orientation")
        self.orient_chk.setChecked(True)
        self.orient_chk.stateChanged.connect(self._apply_orient_visibility)
        f.addWidget(self.orient_chk, 11, 0, 1, 2)

        self.orient_lbl = QtWidgets.QLabel("attitude: --")
        self.orient_lbl.setStyleSheet("font-family: monospace;")
        f.addWidget(self.orient_lbl, 12, 0, 1, 2)

        self._on_mode_changed()
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

        self.offset_spin = QtWidgets.QDoubleSpinBox()
        self.offset_spin.setRange(-500.0, 500.0)
        self.offset_spin.setValue(sp.BEAM_OFFSET_MM)
        self.offset_spin.setSuffix(" mm")
        self.offset_spin.setToolTip(
            "Distance from the rotation axis up to the lidar's scan plane. "
            "Wrong values show up as doubled or thickened walls.")
        self.offset_spin.valueChanged.connect(self._rebuild)
        f.addWidget(QtWidgets.QLabel("Beam offset"), 5, 0)
        f.addWidget(self.offset_spin, 5, 1)

        self.lidar_rot_spin = QtWidgets.QDoubleSpinBox()
        self.lidar_rot_spin.setRange(-180.0, 180.0)
        self.lidar_rot_spin.setValue(sp.LIDAR_ROTATION_DEG)
        self.lidar_rot_spin.setSingleStep(90.0)
        self.lidar_rot_spin.setSuffix(" deg CW")
        self.lidar_rot_spin.setToolTip(
            "How far the lidar is mounted round from the IMU's frame, "
            "clockwise.\n"
            "The platform tilts about the sensor's Y axis, so if this is "
            "wrong the cloud hinges about the wrong direction: a flat wall "
            "comes out as a curved fan. Try 0 / 90 / 180 / -90 and keep the "
            "one where straight things look straight.")
        self.lidar_rot_spin.valueChanged.connect(self._rebuild)
        f.addWidget(QtWidgets.QLabel("Lidar rotation"), 6, 0)
        f.addWidget(self.lidar_rot_spin, 6, 1)

        self.reverse_chk = QtWidgets.QCheckBox("Reverse azimuth direction")
        self.reverse_chk.setChecked(sp.LIDAR_REVERSE)
        self.reverse_chk.setToolTip(
            "Which way the lidar's reported angle runs. Getting this wrong "
            "reflects the cloud, and no rotation can undo a reflection.\n"
            "This one cannot be worked out from a capture: the platform tilts "
            "only about Y, and the mirror commutes with that exactly, so a "
            "mirrored scan is just as self-consistent as a correct one. Set it "
            "by eye against a scene you know.")
        self.reverse_chk.stateChanged.connect(self._rebuild)
        f.addWidget(self.reverse_chk, 7, 0, 1, 2)

        self.flip_chk = QtWidgets.QCheckBox("Flip upright (180 deg)")
        self.flip_chk.setChecked(sp.FLIP_UPRIGHT)
        self.flip_chk.setToolTip(
            "Turns the finished cloud 180 deg about world X, because scans "
            "otherwise come out upside down.\n"
            "The stream itself is self-consistent, so this corrects the "
            "mounting rather than a decoding error -- see FLIP_UPRIGHT in "
            "scan_proto.py.")
        self.flip_chk.stateChanged.connect(self._rebuild)
        f.addWidget(self.flip_chk, 8, 0, 1, 2)

        self.stepper_chk = QtWidgets.QCheckBox("Tilt from stepper, not IMU")
        self.stepper_chk.setToolTip(
            "The IMU sees real mechanical slop but its yaw drifts over a long "
            "sweep. The step count has no drift. Try both and keep the sharper.")
        self.stepper_chk.stateChanged.connect(self._on_tilt_source_changed)
        f.addWidget(self.stepper_chk, 9, 0, 1, 2)

        # The stepper's zero is wherever the platform was when it was homed,
        # which is not level. Uncorrected it tips the whole scene.
        self.tilt_off_spin = QtWidgets.QDoubleSpinBox()
        self.tilt_off_spin.setRange(-90.0, 90.0)
        self.tilt_off_spin.setDecimals(2)
        self.tilt_off_spin.setSingleStep(0.25)
        self.tilt_off_spin.setSuffix(" deg")
        self.tilt_off_spin.setToolTip(
            "Added to every commanded platform angle, to correct a home "
            "position that was not level.")
        self.tilt_off_spin.valueChanged.connect(self._rebuild)
        self.tilt_off_lbl = QtWidgets.QLabel("Stepper zero")
        f.addWidget(self.tilt_off_lbl, 10, 0)
        f.addWidget(self.tilt_off_spin, 10, 1)

        self.tilt_auto_chk = QtWidgets.QCheckBox("Auto from IMU")
        self.tilt_auto_chk.setChecked(True)
        self.tilt_auto_chk.setToolTip(
            "Measure the offset against gravity instead of typing it: the "
            "median difference between the IMU's tilt and the commanded "
            "angle, over the whole sweep.\n"
            "This does not reintroduce the drift you switched modes to "
            "avoid -- it is the filter's YAW that drifts, while pitch stays "
            "accelerometer-corrected, and this is one constant for the whole "
            "scan rather than a per-sample pose.")
        self.tilt_auto_chk.stateChanged.connect(self._on_tilt_source_changed)
        f.addWidget(self.tilt_auto_chk, 11, 0, 1, 2)

        b = QtWidgets.QPushButton("Reset camera")
        b.clicked.connect(self._frame_cloud)
        f.addWidget(b, 12, 0, 1, 2)
        self._on_tilt_source_changed()
        return g

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
        self.mesh_lbl.setStyleSheet("color: #666;")
        f.addWidget(self.mesh_lbl, 9, 0, 1, 2)

        note = meshing.missing_note()
        if note:
            warn = QtWidgets.QLabel(note.splitlines()[0])
            warn.setWordWrap(True)
            warn.setToolTip(note)
            warn.setStyleSheet("color: #b26a00;")
            f.addWidget(warn, 10, 0, 1, 2)
        else:
            # Which library ran matters when comparing results with someone
            # else's machine, and it is otherwise invisible.
            via = QtWidgets.QLabel(f"via {meshing.backend()}")
            via.setStyleSheet("color: #666;")
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
        self.settings = QtCore.QSettings("LidarScanner", "scanner_ui")

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
            "scan/show_orientation": self.orient_chk,
            "view/mode": self.mode_box,
            "view/color_by": self.color_box,
            "view/point_size": self.size_spin,
            "mesh/method": self.algo_box,
            "mesh/depth": self.depth_spin,
            "mesh/trim": self.trim_spin,
            "mesh/radius": self.radius_spin,
            "mesh/alpha": self.alpha_spin,
            "mesh/smooth": self.smooth_spin,
            "mesh/budget": self.budget_spin,
            "mesh/surface": self.surface_box,
            "view/voxel": self.voxel_spin,
            "view/max_range": self.range_spin,
            "geom/beam_offset": self.offset_spin,
            "geom/lidar_rotation": self.lidar_rot_spin,
            "geom/lidar_reverse": self.reverse_chk,
            "geom/flip_upright": self.flip_chk,
            "geom/from_stepper": self.stepper_chk,
            "geom/tilt_offset": self.tilt_off_spin,
            "geom/tilt_offset_auto": self.tilt_auto_chk,
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
        self._on_tilt_source_changed()
        self._on_algo_changed()
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
        for key, sec in self.sections.items():
            self.settings.setValue(f"fold/{key}", sec.is_expanded())

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
            rec = time.strftime("scan_%Y%m%d_%H%M%S.bin")
            rec = os.path.join(os.getcwd(), rec)

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

    def _update_estimate(self):
        if not self.stepped_chk.isChecked():
            self.est_lbl.setText("")
            return
        stops = self.steps_spin.value() + 1
        # Mirrors SCAN_STEP_SETTLE_MS + SCAN_STEP_AVERAGE_MS in scanner.h; the
        # travel between stops is on top and depends on the step size.
        secs = stops * (250 + 200 + self.dwell_spin.value()) / 1000.0
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

    def _on_tilt_source_changed(self):
        """The offset only means anything when the tilt comes from the stepper."""
        stepper = self.stepper_chk.isChecked()
        auto = self.tilt_auto_chk.isChecked()
        for w in (self.tilt_off_lbl, self.tilt_auto_chk):
            w.setEnabled(stepper)
        # In auto mode the box becomes a readout of the measured value, so it
        # stays visible -- seeing "+4.66" is how you know the estimate is sane.
        self.tilt_off_spin.setEnabled(stepper and not auto)
        self.tilt_off_spin.setReadOnly(auto)
        # The switch changes which pose the overlay draws, so repaint it now
        # rather than leaving the old source on screen until the next telemetry
        # record lands.
        self._rebuild()
        self._update_orientation(self._last_quat)

    def _clear_scene(self):
        """Throw away the points collected so far and start the cloud over.

        Only the decoded samples go: the raw .bin recording keeps running, and
        a scan is a physical event that cannot be repeated, so this must never
        be the thing that loses one.
        """
        self.cap.samples.clear()
        self.cap.telem.clear()
        self.cloud = None
        self.loaded_rgb = None
        self._built_n = 0
        self._transpose = None
        self.mesh = None
        self._mesh_points = 0
        self.mesh_item.setVisible(False)
        self._update_mesh_label()
        self.scatter.setData(pos=np.zeros((0, 3)))
        self.progress.setValue(0)
        self.statusBar().showMessage("scene cleared", 3000)
        self._log("scene cleared (the raw stream is kept -- Save .bin still "
                  "has everything)")

    def _start(self):
        # Push the settings first so the sweep always runs with what is on
        # screen, rather than whatever the device happened to still hold.

        self._clear_scene()
        
        if self._push_settings():
            self._send("s")
            self.sweep_started = None
            self.progress.setValue(0)

    def _stop(self):
        self._send("x")

    def _calibrate(self):
        if self.state not in (sp.STATE_IDLE, sp.STATE_DONE):
            self._log("cannot calibrate mid-sweep")
            return
        self._send("z")

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
        gear = cfg[sp.CFG_GEAR]
        stepped = int(cfg[sp.CFG_MODE]) == sp.MODE_STEPPED
        # Adopt the device's values without echoing them straight back.
        for w, val in ((self.angle_spin, deg), (self.time_spin, secs),
                       (self.steps_spin, int(cfg[sp.CFG_STEPS])),
                       (self.dwell_spin, int(cfg[sp.CFG_CAPTURE_MS]))):
            w.blockSignals(True)
            w.setValue(val)
            w.blockSignals(False)
        self.stepped_chk.blockSignals(True)
        self.stepped_chk.setChecked(stepped)
        self.stepped_chk.blockSignals(False)
        self._on_mode_changed()

        self._log(f"device config: ±{deg:.1f} deg, gear {gear:.1f}:1, "
                  + (f"stepped, {int(cfg[sp.CFG_STEPS]) + 1} stops, "
                     f"{int(cfg[sp.CFG_SETTLE_MS])}/"
                     f"{int(cfg[sp.CFG_AVERAGE_MS])}/"
                     f"{int(cfg[sp.CFG_CAPTURE_MS])} ms settle/avg/capture"
                     if stepped else f"continuous over {secs:.1f} s"))

    # Length of the drawn sensor axes, mm. Long enough to read against a
    # room-sized cloud without hiding it.
    AXIS_LEN = 700.0

    def _apply_orient_visibility(self):
        on = self.orient_chk.isChecked()
        for it in self.axis_items:
            it.setVisible(on)
        self.plane_item.setVisible(on)

    def _pose_matrix(self, quat):
        """The rotation the *cloud* is currently being built with, or None.

        This deliberately mirrors build_cloud rather than always reading the
        AHRS: in "tilt from stepper" mode the points are placed by the
        commanded stepper angle and the quaternion is not consulted at all, so
        a triad drawn from the quaternion showed an attitude the cloud did not
        share -- the IMU's drifting yaw against points that have none, plus the
        whole tilt-offset correction missing. Same geometry in, same axes out.
        """
        if self.stepper_chk.isChecked():
            deg = self.platform + self.tilt_off_spin.value()
            return sp.axis_angle_matrix(sp.TILT_AXIS, np.array([deg]))[0]

        if quat is None:
            return None
        q = np.asarray(quat, dtype=np.float64)
        if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-6:
            return None
        R = sp.quat_to_matrix(q[None, :])[0]
        # build_cloud applies R^T when the AHRS turned out to report
        # world->sensor; the overlay has to make the same choice.
        return R.T if self._transpose else R

    def _update_orientation(self, quat):
        """Redraw the attitude triad from whatever pose the cloud is using."""
        if not self.orient_chk.isChecked():
            return
        R = self._pose_matrix(quat)
        if R is None:
            return
        # The overlay lives in the same world as the cloud, so it takes the
        # same upright correction. Without this the triad would keep pointing
        # the old way and contradict the points it is drawn among.
        if self.flip_chk.isChecked():
            R = np.diag([1.0, -1.0, -1.0]) @ R

        for it, axis in zip(self.axis_items, np.eye(3)):
            seg = np.zeros((2, 3))
            seg[1] = R @ (axis * self.AXIS_LEN)
            it.setData(pos=seg)

        # The scan plane: a circle in the sensor's XY at the beam standoff,
        # pushed through the same rotation the points get.
        t = np.linspace(0, 2 * np.pi, 65)
        ring = np.stack([np.cos(t) * self.AXIS_LEN * 0.75,
                         np.sin(t) * self.AXIS_LEN * 0.75,
                         np.full_like(t, self.offset_spin.value())], axis=-1)
        self.plane_item.setData(pos=ring @ R.T)

        # Euler angles for the readout only -- the maths uses the quaternion.
        pitch = np.degrees(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
        roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        # Say which source is on screen: the two disagree (that is the point of
        # having the switch), so an unlabelled readout is unreadable.
        src = "stepper" if self.stepper_chk.isChecked() else "IMU"
        self.orient_lbl.setText(
            f"{src}: roll {roll:+7.1f}  pitch {pitch:+7.1f}  yaw {yaw:+7.1f}")

    def _on_status(self, telem):
        # Indexed via the named constants in scan_proto, because the record has
        # already grown once and hand-counted offsets silently drifted.
        self.state = int(telem[sp.TEL_STATE])
        platform = telem[sp.TEL_PLATFORM]
        dropped = int(telem[sp.TEL_DROPPED])
        # Stash it before drawing: the stepper-mode triad is built from this
        # angle, not from the quaternion.
        self.platform = float(platform)
        self._last_quat = telem[sp.TEL_QUAT]
        self._update_orientation(self._last_quat)
        name = sp.STATE_NAMES.get(self.state, "?")
        self.state_lbl.setText(
            f"{name} - platform {platform:+.1f} deg"
            + (f" - {dropped} dropped" if dropped else ""))

        # Progress is the platform's position across the sweep in both modes:
        # stepped runs through the same -span..+span, just discontinuously.
        if self.state in sp.CAPTURE_STATES or self.state in (
                sp.STATE_STEP_MOVE, sp.STATE_STEP_SETTLE,
                sp.STATE_STEP_AVERAGE):
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
            self.link_lbl.setStyleSheet("color: #c62828; font-weight: bold;")
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

        # Resolve the auto offset here rather than inside build_cloud, so the
        # measured value lands back in the spinbox where it can be sanity
        # checked -- and so switching auto off leaves that value behind as the
        # starting point for a manual tweak instead of snapping back to zero.
        tilt_offset = self.tilt_off_spin.value()
        if self.stepper_chk.isChecked() and self.tilt_auto_chk.isChecked():
            est = sp.estimate_tilt_offset(self.cap)
            if est is not None:
                tilt_offset = est
                self.tilt_off_spin.blockSignals(True)
                self.tilt_off_spin.setValue(est)
                self.tilt_off_spin.blockSignals(False)
        # Resolve the sensor->world vs world->sensor convention once and reuse
        # it, rather than letting build_cloud re-decide (and re-log) it on every
        # 4 Hz tick. Caching it also gives the attitude overlay something to
        # match, so the triad and the points agree about which way is which.
        # Wait for enough telemetry to decide with: resolve_frame falls back to
        # False when it has fewer than 5 records, and caching that early guess
        # would pin the whole session to it.
        if not self.stepper_chk.isChecked() and self._transpose is None \
                and len(self.cap.telem) >= 5:
            try:
                self._transpose = bool(sp.resolve_frame(self.cap))
            except SystemExit:
                pass
        try:
            xyz, dist, _ = sp.build_cloud(
                self.cap,
                max_range=self.range_spin.value() or None,
                transpose=self._transpose,
                from_stepper=self.stepper_chk.isChecked(),
                beam_offset=self.offset_spin.value(),
                lidar_rotation=self.lidar_rot_spin.value(),
                lidar_reverse=self.reverse_chk.isChecked(),
                flip_upright=self.flip_chk.isChecked(),
                tilt_offset=tilt_offset)
        except SystemExit:
            return  # no sweep data yet
        if self.voxel_spin.value():
            xyz, (dist,) = sp.voxel_downsample(xyz, [dist],
                                               self.voxel_spin.value())
        self.cloud = (xyz, dist)
        self.loaded_rgb = None
        self._redraw()

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
        # ball-pivoting build that takes twenty minutes.
        want = meshing.METHOD_BUDGET.get(method)
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
            kwargs["vertexColors"] = scan3d.colorize(verts[:, 2])
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
        if self.loaded_rgb is not None:
            rgba = np.empty((xyz.shape[0], 4), np.float32)
            rgba[:, :3] = self.loaded_rgb
            rgba[:, 3] = 1.0
        else:
            v = xyz[:, 2] if self.color_box.currentText() == "height" else dist
            rgba = scan3d.colorize(v)
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
        if not len(self.cap):
            self._log("nothing to save - no capture data")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save point cloud", time.strftime("scan_%Y%m%d_%H%M%S.ply"),
            "PLY point cloud (*.ply)")
        if not path:
            return
        try:
            xyz, dist, _ = sp.build_cloud(
                self.cap,
                max_range=self.range_spin.value() or None,
                transpose=self._transpose,
                from_stepper=self.stepper_chk.isChecked(),
                beam_offset=self.offset_spin.value(),
                lidar_rotation=self.lidar_rot_spin.value(),
                lidar_reverse=self.reverse_chk.isChecked(),
                flip_upright=self.flip_chk.isChecked(),
                tilt_offset=self.tilt_off_spin.value())
        except SystemExit:
            self._log("failed to build cloud for saving")
            return
        scan3d.export_ply(path, xyz, dist)
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
            self, "Save raw capture", time.strftime("scan_%Y%m%d_%H%M%S.bin"),
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
            self, "Save surface", time.strftime("scan_%Y%m%d_%H%M%S.ply"),
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
            rgb = (scan3d.colorize(verts[:, 2])[:, :3] * 255).astype(np.uint8)
            meshing.export_ply(path, verts, faces, rgb)
        self._log(f"saved {len(faces)} triangles -> {os.path.basename(path)}")

    def _load_ply(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load point cloud", "", "PLY point cloud (*.ply)")
        if not path:
            return
        try:
            xyz, rgb = scan3d.load_ply(path)
        except Exception as exc:
            self._log(f"load failed: {exc}")
            return
        # A loaded cloud has no range channel, so derive one for the colour
        # ramp when the file carried no vertex colours.
        self.cloud = (xyz, np.linalg.norm(xyz, axis=1))
        self.loaded_rgb = rgb
        self._redraw()
        self._frame_cloud()
        self._mesh_param_changed()
        self._log(f"loaded {xyz.shape[0]} points from {os.path.basename(path)}")

    def _load_bin(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open raw capture", "", "Raw capture (*.bin)")
        if not path:
            return
        self.cap = sp.Capture()
        # Keep the file's bytes as the current stream, so a capture that was
        # opened rather than recorded can still be saved back out.
        try:
            with open(path, "rb") as fh:
                self.raw = bytearray(fh.read())
        except Exception as exc:
            self._log(f"load failed: {exc}")
            return
        for _ in sp.parse(sp.file_source(path), capture=self.cap):
            pass
        self._log(f"{len(self.cap)} samples from {os.path.basename(path)}")
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
        super().closeEvent(ev)


def main():
    app = pg.mkQApp("3D Lidar Scanner")
    ui = ScannerUI()
    ui.show()
    sys.exit(app.exec_() if hasattr(app, "exec_") else app.exec())


if __name__ == "__main__":
    main()
