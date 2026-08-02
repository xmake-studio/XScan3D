#!/usr/bin/env python3
"""Background threads and the surface shader.

Everything here does work that must not run on the GUI thread: draining the
serial port, reconstructing a surface, aligning two clouds. Pulled out of the
original single-file UI unchanged.
"""

import queue
import time

import numpy as np

from pyqtgraph.Qt import QtCore

import scan_proto as sp
import meshing
import registration as reg

BAUD = 115200


# --- Serial link ------------------------------------------------------------

class SerialLink(QtCore.QThread):
    """Owns the port. Reads on this thread and writes commands from a queue.

    Both directions are funnelled through one thread on purpose: the reader must
    never block, because if the host stops draining the pipe the firmware starts
    dropping samples and the sweep is unrepeatable.
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
        self.raw = raw
        self._cmds = queue.Queue()
        self._stop = False
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
        parser = sp.StreamParser(self.cap, echo_events=False, sweep_only=True)
        n_ev = n_tel = 0
        n_cfg = None
        try:
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
                        self.raw += chunk
                    self.bytes_in += len(chunk)
                    parser.feed(chunk)

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
#
# pyqtgraph's stock "shaded" shader is one hard-coded lamp with no ambient, so a
# scanned open shell reads as a flat colour with half its triangles black. This
# replaces it with a three-point rig plus hemispheric ambient, lit by
# gl_FrontFacing so both sides of the shell are visible. See the long-form notes
# that shipped with the original UI for the full rationale.
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
        float side = gl_FrontFacing ? 1.0 : -1.0;
        vec3 N = normalize(v_normal) * side;
        vec3 W = normalize(v_world) * side;

        vec3 Lkey  = normalize(vec3(-0.45,  0.60,  0.25));
        vec3 Lfill = normalize(vec3( 0.80, -0.20,  0.15));
        vec3 Lrim  = normalize(vec3( 0.05, -0.40, -0.90));

        vec3 Ckey  = vec3(1.00, 0.95, 0.86);
        vec3 Cfill = vec3(0.42, 0.53, 0.72);
        vec3 Crim  = vec3(0.55, 0.68, 0.95);

        float up = W.z * 0.5 + 0.5;
        vec3 amb = mix(vec3(0.13, 0.12, 0.12), vec3(0.21, 0.23, 0.28), up);

        float dk = max(dot(N, Lkey),  0.0);
        float df = max(dot(N, Lfill), 0.0);
        float dr = max(dot(N, Lrim),  0.0);

        vec3 H = normalize(Lkey + vec3(0.0, 0.0, 1.0));
        float spec = pow(max(dot(N, H), 0.0), 32.0) * 0.12;

        vec3 rgb = v_color.rgb * (amb + Ckey * dk * 0.68 + Cfill * df * 0.30)
                 + Ckey * spec + Crim * dr * 0.14;
        gl_FragColor = vec4(rgb, v_color.a);
    }
"""


def studio_shader():
    from pyqtgraph.opengl import shaders
    return shaders.ShaderProgram("lidarStudio", [
        shaders.VertexShader(MESH_VERT),
        shaders.FragmentShader(MESH_FRAG),
    ])


# --- Meshing worker ---------------------------------------------------------

class MeshWorker(QtCore.QThread):
    """Runs one reconstruction off the GUI thread.

    Meshing is seconds of work where everything else is milliseconds; inline it
    would stall the serial drain and cost samples out of an unrepeatable scan.
    """

    done = QtCore.pyqtSignal(object, object, object)  # verts, faces, info
    failed = QtCore.pyqtSignal(str)

    def __init__(self, xyz, method, params, parent=None):
        super().__init__(parent)
        # Copy: the caller's cloud is replaced wholesale by the next rebuild.
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


# --- Registration worker ----------------------------------------------------

class RegisterWorker(QtCore.QThread):
    """Aligns a new scan onto the map off the GUI thread.

    Registration is seconds of numpy on a big cloud. Run inline it freezes the
    window, which on a tool also holding a serial link looks like the board has
    stopped talking.
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
