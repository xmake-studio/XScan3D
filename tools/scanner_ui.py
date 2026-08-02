#!/usr/bin/env python3
"""Scene-based front end for the 3D lidar scanner.

    python scanner_ui.py

The application is built around a *scene*: a collection of scans you can select,
hide, rename, delete and merge, laid out like Unity or Blender with a 3D
viewport in the middle and dockable panels around it. A scan can come off the
device, be loaded from a .ply/.bin, or be produced by aligning others with SLAM,
and every one of them is a first-class object -- so it is always clear which
scans exist, which are merged, which is selected, and which is being generated
right now. Whole scenes save and load as a single .scene file.

This module is the controller: it owns the scene, the viewport renderer, the
device link and the workers, and wires the panels' actions to them. The heavy
lifting lives in the modules it imports -- the data model (scene_model), the
file format (scene_io), the viewport (scene_view), the threads and shader
(workers), the Qt plumbing (widgets), and the panels (panels/). The numeric
core is unchanged: scan_proto, registration, meshing, cloud_io.

Needs: pyqtgraph, PyQt5, pyserial, numpy.
Optional: pymeshlab / open3d for Poisson and ball-pivoting surface builds.
"""

import os
import sys
import time

import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

import scan_proto as sp
import cloud_io
import registration as reg

from widgets import (FileSettings, SETTINGS_PATH, SCANS_DIR, scan_path,
                     apply_dark_theme, disable_wheel_edits, NO_FRAME)
from workers import SerialLink, MeshWorker, RegisterWorker
from scene_model import Scene, Scan, identity
from scene_view import SceneView, SceneRenderer, CAM_FPS
import scene_io
from panels import (ScenePanel, PropertiesPanel, ViewPanel, NewScanPanel,
                    SlamPanel, SceneSettingsPanel)


class ScannerUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.resize(1600, 950)

        # --- model + view ---
        self.scene = Scene()
        self.view = SceneView()
        self.view.setCameraPosition(distance=4000, elevation=20, azimuth=45)
        self.renderer = SceneRenderer(self.view)
        self.renderer.set_scene(self.scene)
        self.setCentralWidget(self.view)

        # --- device / capture state ---
        self.link = None
        self.cap = sp.Capture()
        self.raw = bytearray()
        self.generating_id = None      # scan id of the live sweep, or None
        self._built_n = 0
        self._last_state = sp.STATE_IDLE
        self._sweep_done = False
        self.connected_at = 0.0
        self.warned_silent = False
        self.sweep_started = None

        # --- meshing state ---
        self._mesh = None              # (verts, faces)
        self._mesh_info = {}
        self._mesh_worker = None
        self._mesh_points = 0
        self._mesh_debounce = QtCore.QTimer(self)
        self._mesh_debounce.setSingleShot(True)
        self._mesh_debounce.setInterval(600)
        self._mesh_debounce.timeout.connect(self.build_surface)

        # --- SLAM state ---
        self._reg_worker = None
        self._pending = None           # {unit, result, orig_T, level, label}
        # Remaining merge units. Each unit is a list of scan ids that move
        # together under one solved transform: a loose scan is a 1-id unit; a
        # whole merged group aligned onto another is a multi-id unit.
        self._merge_queue = []
        self._merge_group = None
        self._merge_target = None      # accumulating target points (float64)
        self._merge_desc = ""          # what the queue is aligned onto
        self._merge_current = None     # ids of the unit being aligned right now

        self._build_panels()
        self._build_docks()
        self._build_menu()

        disable_wheel_edits(self)
        self.refresh_ports()
        self._init_settings()

        self.statusBar().showMessage("disconnected")
        self._refresh_all()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(250)

    # --- construction -------------------------------------------------------

    def _build_panels(self):
        self.scene_panel = ScenePanel(self)
        self.properties = PropertiesPanel(self)
        self.view_panel = ViewPanel(self)
        self.newscan = NewScanPanel(self)
        self.slam = SlamPanel(self)
        self.settings_panel = SceneSettingsPanel(self)

    def _dock(self, title, widget, closable=True):
        dock = QtWidgets.QDockWidget(title, self)
        dock.setObjectName(title)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(NO_FRAME)
        scroll.setWidget(widget)
        dock.setWidget(scroll)
        feats = (QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable
                 | QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetFloatable
                 if hasattr(QtWidgets.QDockWidget, "DockWidgetFeature")
                 else QtWidgets.QDockWidget.DockWidgetMovable
                 | QtWidgets.QDockWidget.DockWidgetFloatable)
        if closable:
            closable_f = (
                QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetClosable
                if hasattr(QtWidgets.QDockWidget, "DockWidgetFeature")
                else QtWidgets.QDockWidget.DockWidgetClosable)
            feats = feats | closable_f
        dock.setFeatures(feats)
        return dock

    def _build_docks(self):
        left = _enum_area("LeftDockWidgetArea")
        right = _enum_area("RightDockWidgetArea")

        # Scene is pinned: it is the one panel you always need, so it is not
        # closable and sits on its own on the left.
        self.scene_dock = self._dock("Scene", self.scene_panel, closable=False)
        self.addDockWidget(left, self.scene_dock)

        self.prop_dock = self._dock("Properties", self.properties)
        self.view_dock = self._dock("View", self.view_panel)
        self.newscan_dock = self._dock("New Scan", self.newscan)
        self.slam_dock = self._dock("SLAM", self.slam)
        self.settings_dock = self._dock("Scene Settings", self.settings_panel)

        for d in (self.prop_dock, self.view_dock, self.newscan_dock,
                  self.slam_dock, self.settings_dock):
            self.addDockWidget(right, d)
        # Tab the right-hand docks together so they read as one panel with tabs,
        # the way Unity/Blender stack their inspectors.
        self.tabifyDockWidget(self.prop_dock, self.view_dock)
        self.tabifyDockWidget(self.view_dock, self.newscan_dock)
        self.tabifyDockWidget(self.newscan_dock, self.slam_dock)
        self.tabifyDockWidget(self.slam_dock, self.settings_dock)
        self.newscan_dock.raise_()

        self._docks = {
            "scene": self.scene_dock, "properties": self.prop_dock,
            "view": self.view_dock, "newscan": self.newscan_dock,
            "slam": self.slam_dock, "settings": self.settings_dock,
        }
        self._default_state = None     # captured after show(), for Reset layout

    def _build_menu(self):
        mb = self.menuBar()
        filem = mb.addMenu("&File")
        filem.addAction("New Scene", self.new_scene)
        filem.addAction("Open Scene…", self.open_scene)
        filem.addAction("Save Scene", self.save_scene)
        filem.addAction("Save Scene As…", self.save_scene_as)
        filem.addSeparator()
        filem.addAction("Import .ply…", self.load_ply)
        filem.addAction("Import .bin…", self.open_bin)
        filem.addAction("Export selection .ply…", self.save_ply)
        filem.addAction("Export selection .bin…", self.save_selected_bin)
        filem.addSeparator()
        filem.addAction("Quit", self.close)

        viewm = mb.addMenu("&View")
        for d in (self.scene_dock, self.prop_dock, self.view_dock,
                  self.newscan_dock, self.slam_dock, self.settings_dock):
            viewm.addAction(d.toggleViewAction())
        viewm.addSeparator()
        viewm.addAction("Reset layout", self._reset_layout)
        viewm.addAction("Frame scene", self.frame_scene)

    # --- central refresh ----------------------------------------------------

    def _refresh_all(self):
        """Push scene state to the viewport and every panel. Called after any
        change, so no panel ever has to read another's state."""
        self.renderer.color_mode = self.view_panel.color_mode()
        self.renderer.point_size = self.view_panel.point_size()
        self.renderer.sync()
        self.scene_panel.refresh()
        self.properties.refresh()
        self.slam.refresh()
        self._update_title()

    def _update_title(self):
        star = "*" if self.scene.dirty else ""
        name = self.scene.name or "Untitled scene"
        self.setWindowTitle(f"3D Lidar Scanner — {name}{star}")

    def color_mode(self):
        return self.view_panel.color_mode()

    # --- selection / per-scan edits -----------------------------------------

    def on_selection_changed(self, ids):
        self.scene.set_selection(ids)
        self._refresh_all()

    def on_visibility_toggled(self, scan_id, visible):
        s = self.scene.get(scan_id)
        if s is not None:
            s.visible = visible
            self.scene.dirty = True
            self._refresh_all()

    def on_group_visibility_toggled(self, gid, visible):
        self.scene.set_group_visible(gid, visible)
        n = len(self.scene.group_members(gid))
        self._log(f"{'shown' if visible else 'hidden'} group "
                  f"'{self.scene.group_name(gid)}' ({n} scans)")
        self._refresh_all()

    def on_rename(self, scan_id, name):
        s = self.scene.get(scan_id)
        if s is not None and name and name != s.name:
            s.name = name
            self.scene.dirty = True
            self._refresh_all()

    def on_group_rename(self, gid, name):
        g = self.scene.groups.get(gid)
        if g is not None and name and name != g["name"]:
            g["name"] = name
            self.scene.dirty = True
            self._refresh_all()

    def on_color_override(self, scan_id, rgb):
        s = self.scene.get(scan_id)
        if s is not None:
            s.color = rgb
            self.scene.dirty = True
            self._refresh_all()

    def on_transform_changed(self, scan_id, T):
        s = self.scene.get(scan_id)
        if s is not None:
            s.T = np.asarray(T, float)
            self.scene.dirty = True
            self._refresh_all()

    def delete_selected(self):
        sel = list(self.scene.selection)
        if not sel:
            return
        # Never silently delete the sweep in progress; stop it first.
        for sid in sel:
            if sid == self.generating_id:
                self.generating_id = None
            self.scene.remove(sid)
        self._log(f"removed {len(sel)} scan(s)")
        self._refresh_all()

    def duplicate_selected(self):
        made = []
        for s in self.scene.selected():
            dup = Scan(s.xyz.copy(), s.dist.copy(),
                       name=self.scene.unique_name(s.name + " copy"),
                       rgb=None if s.rgb is None else s.rgb.copy(),
                       T=s.T.copy(), source=s.source, color=s.color)
            self.scene.add(dup, select=False)
            made.append(dup.id)
        if made:
            self.scene.set_selection(made)
            self._refresh_all()

    def isolate_selected(self):
        sel = set(self.scene.selection)
        if not sel:
            return
        for s in self.scene.scans:
            s.visible = s.id in sel
        self.scene.dirty = True
        self._refresh_all()

    def show_all(self):
        for s in self.scene.scans:
            s.visible = True
        self.scene.dirty = True
        self._refresh_all()

    def clear_scene(self):
        if self.scene.scans and not self._confirm(
                "Clear scene",
                "Remove every scan from the scene? Scans not saved to a file "
                "will be lost. This does not touch files already on disk."):
            return
        self.scene.clear()
        self.generating_id = None
        self._pending = None
        self.renderer.pending_ids = set()
        self._mesh = None
        self.renderer.set_mesh(None, None, "shaded")
        self._log("scene cleared")
        self._refresh_all()

    # --- view ---------------------------------------------------------------

    def on_color_mode_changed(self):
        self._refresh_all()

    def on_point_size_changed(self):
        self.renderer.point_size = self.view_panel.point_size()
        self.renderer.sync()

    def on_camera_mode_changed(self):
        self.view.set_cam_mode(self.view_panel.camera_mode())
        if self.view_panel.camera_mode() == CAM_FPS:
            QtCore.QTimer.singleShot(0, self.view.setFocus)

    def on_grid_toggled(self, on):
        self.renderer.set_grid_visible(on)

    def on_view_mode_changed(self):
        surf = self.view_panel.is_surface()
        self.view_panel.show_surface_controls(surf)
        self.renderer.set_surface_mode(surf)
        if surf and self._mesh is None:
            self.build_surface()

    def on_surface_style_changed(self):
        if self._mesh is not None:
            v, faces = self._mesh
            self.renderer.set_mesh(v, faces, self.view_panel.surface_style())

    def frame_scene(self):
        b = self.scene.bounds()
        if b is None:
            return
        lo, hi = b
        center = (lo + hi) / 2.0
        extent = float(np.linalg.norm(hi - lo))
        self.view.setCameraPosition(pos=pg.Vector(*center),
                                    distance=max(extent * 1.2, 500.0))

    # --- build filters / geometry -------------------------------------------

    def on_build_filter_changed(self):
        if hasattr(self, "settings_panel"):
            self._rebuild_capture_scans()

    def on_geometry_changed(self):
        if hasattr(self, "settings_panel"):
            self._rebuild_capture_scans()

    def _build_xyz(self, cap):
        """(xyz, dist) from a capture with the current mount geometry and build
        filters, or (empty, empty) if there is no sweep data yet."""
        if not len(cap):
            return (np.zeros((0, 3), np.float32), np.zeros((0,), np.float32))
        geom = self.settings_panel.geometry()
        filt = self.view_panel.build_filters()
        try:
            xyz, dist, _ = sp.build_cloud(cap, max_range=filt["max_range"],
                                          **geom)
        except SystemExit:
            return (np.zeros((0, 3), np.float32), np.zeros((0,), np.float32))
        if filt["voxel"]:
            xyz, (dist,) = sp.voxel_downsample(xyz, [dist], filt["voxel"])
        return xyz.astype(np.float32), dist.astype(np.float32)

    def _rebuild_capture_scans(self):
        """Rebuild every scan that still has a capture behind it.

        This is what makes mount geometry tunable after the fact: change the
        emitter spacing and the live scan (and any just-loaded .bin) rebuild in
        place, so a flat surface can be flattened by eye.
        """
        touched = False
        for s in self.scene.scans:
            if s.raw is not None and len(s.raw):
                s.xyz, s.dist = self._build_xyz(s.raw)
                touched = True
        if touched:
            self._refresh_all()

    # --- device -------------------------------------------------------------

    def refresh_ports(self):
        from serial.tools import list_ports
        ports = [(p.device, p.description) for p in list_ports.comports()]
        self.newscan.set_ports(ports)

    def toggle_connect(self):
        if self.link:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.newscan.port()
        if not port or port.startswith("("):
            self._log("no serial port selected")
            return
        rec = None
        if self.newscan.record():
            rec = scan_path(time.strftime("scan_%Y%m%d_%H%M%S.bin"),
                            "autosaves")
        self.cap = sp.Capture()
        self.raw = bytearray()
        self.connected_at = time.time()
        self.warned_silent = False
        self.link = SerialLink(port, self.cap, rec, dtr=self.newscan.dtr(),
                               raw=self.raw, parent=self)
        self.link.event.connect(self._log)
        self.link.status.connect(self._on_status)
        self.link.config.connect(self._on_config)
        self.link.failed.connect(self._on_failed)
        self.link.opened.connect(
            lambda: self.statusBar().showMessage(f"connected to {port}"))
        self.link.start()
        self.newscan.set_connected(True)
        if rec:
            self._log(f"recording raw stream -> {os.path.basename(rec)}")

    def _disconnect(self):
        if not self.link:
            return
        self.link.stop()
        self.link.wait(2000)
        self.link = None
        # The sweep in progress is now as complete as it will get.
        if self.generating_id is not None:
            g = self.scene.get(self.generating_id)
            if g is not None:
                g.generating = False
            self.generating_id = None
        self.newscan.set_connected(False)
        self.statusBar().showMessage("disconnected")
        self._refresh_all()

    def _send(self, text):
        if not self.link:
            self._log("not connected")
            return False
        self.link.send(text + "\n")
        return True

    def _push_settings(self):
        s = self.newscan.scan_settings()
        if not self._send(f"a{s['angle']:.2f}"):
            return False
        self._send(f"t{s['time']:.2f}")
        self._send(f"m{1 if s['stepped'] else 0}")
        if s["stepped"]:
            self._send(f"n{s['steps']}")
            self._send(f"d{s['dwell']}")
        return True

    def focus_newscan(self):
        self.newscan_dock.raise_()
        self.newscan.start_btn.setFocus()

    # --- scanning -----------------------------------------------------------

    def start_scan(self):
        if not self.link:
            self._log("connect to a device first")
            return
        if self._pending is not None:
            self._log("resolve the pending alignment first")
            return
        delay = self.newscan.scan_settings()["delay"]
        if delay > 0:
            self._begin_countdown(delay)
        else:
            self._begin_scan()

    def _begin_countdown(self, secs):
        self._countdown_left = secs
        self.newscan.set_start_enabled(False)
        self.newscan.set_state_text(f"starting in {secs} s")
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
            self.newscan.set_state_text(f"starting in {self._countdown_left} s")

    def _cancel_countdown(self):
        if getattr(self, "_countdown_timer", None) is not None:
            self._countdown_timer.stop()
        self.newscan.set_start_enabled(True)

    def _begin_scan(self):
        # Freeze any scan still fed by the live capture into a permanent one, so
        # starting a new sweep never overwrites the last -- each scan is its own
        # object now.
        for s in self.scene.scans:
            if s.raw is self.cap:
                s.raw = None
                s.generating = False
        self.generating_id = None
        # Fresh sweep into the same capture object the link is feeding.
        self.cap.samples.clear()
        self.cap.telem.clear()
        self._built_n = 0
        self._sweep_done = False

        g = Scan(np.zeros((0, 3), np.float32), name=self.scene.unique_name("Scan"),
                 source="device", generating=True, raw=self.cap)
        self.scene.add(g, select=True)
        self.generating_id = g.id

        if self._push_settings():
            self._send("s")
            self.sweep_started = None
            self.newscan.set_progress(0)
        self._refresh_all()

    def stop_scan(self):
        if getattr(self, "_countdown_timer", None) is not None \
                and self._countdown_timer.isActive():
            self._cancel_countdown()
            self.newscan.set_state_text("idle")
            return
        self._send("x")

    # --- device signals -----------------------------------------------------

    def _on_failed(self, msg):
        self._log(f"serial error: {msg}")
        self.statusBar().showMessage(f"error: {msg}")
        self._disconnect()

    def _on_config(self, cfg):
        deg = cfg[sp.CFG_DEGREES]
        secs = cfg[sp.CFG_TIME]
        stepped = int(cfg[sp.CFG_MODE]) == sp.MODE_STEPPED
        # Adopt the device's values only for fields the user has never set.
        for key, w, val in (("scan/angle", self.newscan.angle_spin, deg),
                            ("scan/time", self.newscan.time_spin, secs),
                            ("scan/steps", self.newscan.steps_spin,
                             int(cfg[sp.CFG_STEPS])),
                            ("scan/dwell", self.newscan.dwell_spin,
                             int(cfg[sp.CFG_CAPTURE_MS])),
                            ("scan/stepped", self.newscan.stepped_chk, stepped)):
            if self.settings.value(key) is not None:
                continue
            w.blockSignals(True)
            if isinstance(w, QtWidgets.QCheckBox):
                w.setChecked(val)
            else:
                w.setValue(val)
            w.blockSignals(False)
        self.newscan._on_mode_changed()
        self._log(f"device config: ±{deg:.1f} deg, "
                  + (f"stepped, {int(cfg[sp.CFG_STEPS]) + 1} stops"
                     if stepped else f"continuous over {secs:.1f} s"))

    def _on_status(self, telem):
        self.state = int(telem[sp.TEL_STATE])
        if self.state == sp.STATE_DONE and self._last_state != sp.STATE_DONE:
            self._sweep_done = True
        self._last_state = self.state
        platform = telem[sp.TEL_PLATFORM]
        dropped = int(telem[sp.TEL_DROPPED])
        name = sp.STATE_NAMES.get(self.state, "?")
        self.newscan.set_state_text(
            f"{name} - platform {platform:+.1f} deg"
            + (f" - {dropped} dropped" if dropped else ""))

        if self.state in sp.SWEEP_STATES:
            span = self.newscan.angle_spin.value()
            frac = (platform + span) / (2 * span) if span else 0.0
            self.newscan.set_progress(np.clip(frac, 0, 1) * 100)
        elif self.state == sp.STATE_DONE:
            self.newscan.set_progress(100)
        if dropped:
            self.statusBar().showMessage(
                f"{dropped} samples dropped - host not keeping up", 3000)

    # --- live loop ----------------------------------------------------------

    def _tick(self):
        self._check_link()
        if self.link and len(self.cap) != self._built_n \
                and self.generating_id is not None:
            self._built_n = len(self.cap)
            g = self.scene.get(self.generating_id)
            if g is not None:
                g.xyz, g.dist = self._build_xyz(self.cap)
                self.renderer.sync()
                self.scene_panel.refresh()
        elif self._sweep_done:
            self._sweep_done = False
            # The cloud now holds every sample of the finished sweep; settle the
            # live object into a normal scan.
            if self.generating_id is not None:
                g = self.scene.get(self.generating_id)
                if g is not None:
                    g.generating = False
                self.generating_id = None
                self._refresh_all()

    def _check_link(self):
        if not self.link:
            self.newscan.set_link_text("not connected")
            return
        n = self.link.bytes_in
        age = time.time() - self.connected_at
        self.newscan.set_link_text(
            f"{n} bytes in, {self.link.cmds_sent} commands sent"
            + (f", {len(self.cap)} samples" if len(self.cap) else ""))
        if n == 0 and age > 3.0 and not self.warned_silent:
            self.warned_silent = True
            self.newscan.set_link_text(f"NO DATA after {age:.0f}s - see log",
                                       warn=True)
            self._log(
                "--- port is open but the device has sent nothing ---\n"
                f"  DTR is currently "
                f"{'ON' if self.newscan.dtr() else 'OFF'} - try toggling it, "
                "then reconnect.\n"
                "  The RP2040's USB serial stays mute until the host asserts "
                "DTR.\n"
                "  Also check no other program holds the port, and that the "
                "board is running the app, not the bootloader.")

    # --- meshing ------------------------------------------------------------

    def on_mesh_param_changed(self, *_):
        # Panels emit this during their own construction, before every panel
        # attribute exists; ignore until the window is fully built.
        if not hasattr(self, "view_panel"):
            return
        if self.view_panel.is_surface() and self.scene.visible_scans():
            self._mesh_debounce.start()

    def build_surface(self):
        xyz, _ = self.scene.visible_world()
        if not xyz.shape[0]:
            self.view_panel.set_mesh_label("no visible points to reconstruct")
            return
        if self._mesh_worker is not None and self._mesh_worker.isRunning():
            self._mesh_debounce.start()
            return
        params = self.view_panel.mesh_params()
        method = params.pop("method")
        self._mesh_points = xyz.shape[0]
        self.view_panel.set_mesh_button_enabled(False)
        self.view_panel.set_mesh_label(
            f"reconstructing from {xyz.shape[0]:,} points...")
        self.statusBar().showMessage("building surface...", 0)
        self._mesh_worker = MeshWorker(xyz, method, params, parent=self)
        self._mesh_worker.done.connect(self._on_mesh_done)
        self._mesh_worker.failed.connect(self._on_mesh_failed)
        self._mesh_worker.start()

    def _on_mesh_done(self, verts, faces, info):
        self.view_panel.set_mesh_button_enabled(True)
        self._mesh_worker = None
        self._mesh_info = info
        if not len(faces):
            self._mesh = None
            self.renderer.set_mesh(None, None, "shaded")
            self.view_panel.set_mesh_label(
                "reconstruction produced no faces -- try a larger Alpha/Ball "
                "radius, or lower Detail")
            return
        self._mesh = (verts, faces)
        self.renderer.set_mesh(verts, faces, self.view_panel.surface_style())
        thinned = (f", thinned {info['input']:,} -> {info['used']:,} points"
                   if info.get("voxel") else "")
        self.view_panel.set_mesh_label(
            f"{len(faces):,} triangles, {len(verts):,} vertices in "
            f"{info['seconds']:.1f}s{thinned}")
        self.statusBar().showMessage(f"{len(faces):,} triangles", 0)

    def _on_mesh_failed(self, msg):
        self.view_panel.set_mesh_button_enabled(True)
        self._mesh_worker = None
        self.view_panel.set_mesh_label(msg)
        self._log(f"reconstruction failed: {msg}")

    # --- SLAM ---------------------------------------------------------------

    def slam_busy(self):
        return self._reg_worker is not None and self._reg_worker.isRunning()

    def has_pending(self):
        return self._pending is not None

    def merge_selected(self):
        sel = self.scene.selected()
        if len(sel) < 2:
            self._log("select two or more scans to merge")
            return
        if self._pending is not None or self.slam_busy():
            self._log("finish the current alignment first")
            return

        # If the selection already contains an existing merged group, treat that
        # group as one fixed object and align the loose (ungrouped) scans onto
        # it -- so a new scan can be added to a group you already merged, rather
        # than only ever building a group from scratch.
        groups = {s.group for s in sel if s.group}
        loose = [s for s in sel if not s.group]

        # Two or more merged groups selected: align each group onto another as a
        # single rigid point cloud. The largest group is held fixed as the
        # baseline; every other group (and any loose scans) is fitted onto it in
        # one solve and, once kept, joins the baseline group -- so two merges can
        # themselves be merged and thereafter behave as one point cloud.
        if len(groups) > 1:
            gid = max(groups, key=lambda g: self.scene.group_union(g)[0].shape[0])
            target = self.scene.group_union(gid)[0].astype(np.float64)
            others = [g for g in self.scene.group_ids() if g in groups and g != gid]
            units = [[s.id for s in self.scene.group_members(g)] for g in others]
            units += [[s.id] for s in loose]
            desc = (f"the group '{self.scene.group_name(gid)}' "
                    f"({len(self.scene.group_members(gid))} scans, "
                    f"{target.shape[0]:,} pts)")
            movers = len(others) + len(loose)
            self._log(f"merging {movers} object(s) onto {desc}")
            self.slam_dock.raise_()
            self._start_merge(gid, target, units, desc)
            return

        if groups:
            gid = groups.pop()
            if not loose:
                self._log("nothing new to add -- select the group plus at least "
                          "one un-merged scan")
                return
            # Align against the whole existing group, held fixed at its poses --
            # every member's points are the baseline, not one picked scan.
            members = self.scene.group_members(gid)
            target = self.scene.group_union(gid)[0].astype(np.float64)
            desc = (f"the group '{self.scene.group_name(gid)}' "
                    f"({len(members)} scans, {target.shape[0]:,} pts)")
            self._log(f"merging {len(loose)} scan(s) onto {desc}")
            self.slam_dock.raise_()
            self._start_merge(gid, target, [[s.id] for s in loose], desc)
            return

        # No existing group: build a fresh one, first scan defines the frame.
        gid = self.scene.new_group()
        base = sel[0]
        base.group = gid
        target = base.world_points()
        desc = f"'{base.name}' ({target.shape[0]:,} pts)"
        self._log(f"merging {len(sel) - 1} scan(s) onto {desc}")
        self.slam_dock.raise_()
        self._start_merge(gid, target, [[s.id] for s in sel[1:]], desc)

    def rerun_slam(self):
        sel = self.scene.selected()
        gids = {s.group for s in sel if s.group}
        if len(gids) != 1:
            self._log("select the scans of one merged group to re-run")
            return
        if self._pending is not None or self.slam_busy():
            return
        gid = gids.pop()
        members = self.scene.group_members(gid)
        base = members[0]
        # Re-run from scratch: the base holds the frame, the rest go back to
        # their own frame before being re-aligned.
        for s in members[1:]:
            s.T = identity()
        target = base.world_points()
        desc = f"'{base.name}' ({target.shape[0]:,} pts)"
        self._start_merge(gid, target, [[s.id] for s in members[1:]], desc)

    def _start_merge(self, gid, target_points, queue, target_desc=""):
        self._merge_group = gid
        self._merge_queue = list(queue)
        self._merge_target = np.asarray(target_points, dtype=np.float64)
        # Human-readable description of what the queued scans are aligned onto,
        # shown in the SLAM verdict so it is never a mystery whether the whole
        # group or a single scan is the baseline.
        self._merge_desc = target_desc
        self.scene.dirty = True
        self._refresh_all()
        self._merge_next()

    def _merge_next(self):
        if not self._merge_queue:
            # Done. A group needs at least two members to mean anything.
            if len(self.scene.group_members(self._merge_group)) < 2:
                self.scene.ungroup(
                    [s.id for s in self.scene.group_members(self._merge_group)])
            self.slam.set_verdict("Merge complete.", "#66bb6a")
            self.slam.set_busy(False)
            self._refresh_all()
            return
        unit = [i for i in self._merge_queue[0] if self.scene.get(i) is not None]
        if not unit:
            self._merge_queue.pop(0)
            return self._merge_next()
        self._merge_current = unit
        scans = [self.scene.get(i) for i in unit]
        # A whole group moves as one rigid body: register its combined point
        # cloud, so the single solved transform is applied to every member.
        source = np.vstack(
            [s.world_points() for s in scans]).astype(np.float64)
        label = (scans[0].name if len(scans) == 1
                 else f"the group '{self.scene.group_name(scans[0].group)}'")
        self._merge_label = label
        p = self.slam.params()
        params = {"voxel": p["voxel"], "structure": p.get("structure", False)}
        if p["mode"] == "small":
            params["init"] = reg.identity()
        elif p["mode"] == "hint":
            params["yaw_hint"] = p["yaw"]
        self.slam.set_busy(True)
        onto = f" onto {self._merge_desc}" if self._merge_desc else ""
        self.slam.set_verdict(f"aligning {label}{onto}...")
        self._reg_worker = RegisterWorker(source, self._merge_target, params,
                                          self)
        self._reg_worker.note.connect(
            lambda s: self.slam.set_verdict(f"aligning: {s}"))
        self._reg_worker.done.connect(self._on_register_done)
        self._reg_worker.failed.connect(self._on_register_failed)
        self._reg_worker.start()
        self.slam.refresh()

    def _on_register_failed(self, msg):
        self._log(f"alignment failed: {msg}")
        self.slam.set_verdict("alignment failed - see log", "#ef5350")
        self.slam.set_busy(False)
        self._reg_worker = None
        self._merge_queue = []
        self._refresh_all()

    def _on_register_done(self, result):
        self._reg_worker = None
        unit = [i for i in self._merge_current if self.scene.get(i) is not None]
        if not unit:
            return self._merge_next()
        scans = [self.scene.get(i) for i in unit]
        level, why = reg.verdict(result)
        # Preview: place the whole unit at the solved pose and paint it orange.
        # Every scan in the unit gets the same rigid transform, so a merged
        # group keeps its internal alignment while moving as one body.
        Tr = np.asarray(result["T"], float)
        orig = {s.id: s.T.copy() for s in scans}
        for s in scans:
            s.T = Tr @ s.T
        label = getattr(self, "_merge_label", scans[0].name)
        self._pending = {"unit": unit, "result": result, "orig_T": orig,
                         "level": level, "label": label}
        self.renderer.pending_ids = set(unit)
        colour, lead = {
            reg.GOOD: ("#66bb6a", "Looks right"),
            reg.CHECK: ("#ffa726", "Worth a look"),
            reg.POOR: ("#ef5350", "Probably wrong"),
        }[level]
        self.slam.set_verdict(
            f"{lead} — {why}. {label} is drawn in orange; check it lines up, "
            "then Keep or Discard.", colour)
        self.slam.set_pending(True)
        self.slam.set_busy(False)
        self._refresh_all()
        if level == reg.GOOD:
            self.accept_pending()

    def accept_pending(self):
        if self._pending is None:
            return
        unit = self._pending["unit"]
        label = self._pending["label"]
        scans = [self.scene.get(i) for i in unit if self.scene.get(i) is not None]
        self._pending = None
        self.renderer.pending_ids = set()
        self.slam.set_pending(False)
        if scans:
            # Groups the movers are leaving; prune any left empty so a merged
            # group that has been folded into another doesn't linger as a
            # ghost entry.
            vacated = {s.group for s in scans if s.group and s.group != self._merge_group}
            for s in scans:
                s.group = self._merge_group
            for gid in vacated:
                if not self.scene.group_members(gid):
                    self.scene.groups.pop(gid, None)
            self._merge_target = np.vstack(
                [self._merge_target]
                + [s.world_points().astype(np.float64) for s in scans])
            # Subsequent units in the queue align onto the group as it now
            # stands, so keep the baseline description in step.
            members = self.scene.group_members(self._merge_group)
            self._merge_desc = (f"the group '{self.scene.group_name(self._merge_group)}' "
                                f"({len(members)} scans, "
                                f"{self._merge_target.shape[0]:,} pts)")
            self._log(f"merged {label}")
        if self._merge_queue and self._merge_queue[0] == unit:
            self._merge_queue.pop(0)
        self._refresh_all()
        self._merge_next()

    def reject_pending(self):
        if self._pending is None:
            return
        unit = self._pending["unit"]
        orig = self._pending["orig_T"]
        for i in unit:
            scan = self.scene.get(i)
            if scan is not None and i in orig:
                scan.T = orig[i]
        self._pending = None
        self.renderer.pending_ids = set()
        self.slam.set_pending(False)
        if self._merge_queue and self._merge_queue[0] == unit:
            self._merge_queue.pop(0)
        self.slam.set_verdict("alignment discarded", "#9a9a9a")
        self._refresh_all()
        self._merge_next()

    def unmerge_selected(self):
        sel = self.scene.selected()
        ids = [s.id for s in sel if s.group]
        if not ids:
            return
        self.scene.ungroup(ids)
        self._log(f"un-merged {len(ids)} scan(s)")
        self._refresh_all()

    # --- point-cloud files --------------------------------------------------

    def load_ply(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load point cloud", SCANS_DIR, "PLY point cloud (*.ply)")
        if not path:
            return
        try:
            xyz, rgb = cloud_io.load_ply(path)
        except Exception as exc:
            self._log(f"load failed: {exc}")
            return
        scan = Scan(xyz, name=self.scene.unique_name(
            os.path.splitext(os.path.basename(path))[0]),
            rgb=rgb, source="ply")
        self.scene.add(scan, select=True)
        self._log(f"loaded {xyz.shape[0]:,} points from {os.path.basename(path)}")
        self._refresh_all()
        self.frame_scene()

    def open_bin(self):
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
        cap = sp.Capture()
        sp.StreamParser(cap, echo_events=False, sweep_only=True).feed(data)
        dropped = sp.keep_last_sweep(cap)
        xyz, dist = self._build_xyz(cap)
        scan = Scan(xyz, dist, name=self.scene.unique_name(
            os.path.splitext(os.path.basename(path))[0]),
            source="bin", raw=cap)
        self.scene.add(scan, select=True)
        self._log(f"{len(cap)} samples from {os.path.basename(path)}"
                  + (f" ({dropped} from earlier sweeps ignored)"
                     if dropped else ""))
        self._refresh_all()
        self.frame_scene()

    def save_ply(self):
        sel = self.scene.selected()
        scans = sel if sel else self.scene.visible_scans()
        if not scans:
            self._log("nothing to save")
            return
        parts = [s.world_points() for s in scans if s.n_points]
        dists = [s.dist for s in scans if s.n_points]
        if not parts:
            self._log("nothing to save - selected scans are empty")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export point cloud",
            scan_path(time.strftime("scan_%Y%m%d_%H%M%S.ply")),
            "PLY point cloud (*.ply)")
        if not path:
            return
        xyz = np.vstack(parts)
        cloud_io.export_ply(path, xyz, np.concatenate(dists))
        self._log(f"saved {len(scans)} scan(s), {xyz.shape[0]:,} points -> "
                  f"{os.path.basename(path)}")

    def save_selected_bin(self):
        """Write the selected scan's raw capture back out as a .bin.

        Unlike the old whole-session dump, this saves one scan: it re-serialises
        that scan's own capture, so a device or .bin-loaded scan can be exported
        individually. Scans with no capture behind them (loaded .ply, merge
        results) have no raw stream to save.
        """
        scan = self.scene.active()
        if scan is None:
            self._log("nothing to save - select a scan first")
            return
        if scan.raw is None or not len(scan.raw):
            self._log(f"'{scan.name}' has no raw capture to save "
                      "(.bin export needs a device or .bin-loaded scan)")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export raw capture",
            scan_path(f"{scan.name}.bin"), "Raw capture (*.bin)")
        if not path:
            return
        try:
            data = sp.write_capture(scan.raw)
            with open(path, "wb") as fh:
                fh.write(data)
        except Exception as exc:
            self._log(f"save failed: {exc}")
            return
        self._log(f"saved '{scan.name}' ({len(data):,} bytes) -> "
                  f"{os.path.basename(path)}")

    # --- scene files --------------------------------------------------------

    def on_scene_name_changed(self, name):
        self.scene.name = name.strip() or "Untitled scene"
        self.scene.dirty = True
        self._update_title()

    def new_scene(self):
        if self.scene.dirty and self.scene.scans and not self._confirm(
                "New scene", "Discard the current scene? Unsaved scans will be "
                "lost."):
            return
        self.scene = Scene()
        self.renderer.set_scene(self.scene)
        self.generating_id = None
        self._pending = None
        self._mesh = None
        self.renderer.set_mesh(None, None, "shaded")
        self.settings_panel.set_scene_name("")
        self.settings_panel.set_path_text("not saved yet")
        self._log("new scene")
        self._refresh_all()

    def open_scene(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open scene", SCANS_DIR, "Scene (*.scene)")
        if not path:
            return
        try:
            scene = scene_io.load_scene(path)
        except Exception as exc:
            self._log(f"open failed: {exc}")
            return
        self.scene = scene
        self.renderer.set_scene(self.scene)
        self.generating_id = None
        self._pending = None
        self._mesh = None
        self.renderer.set_mesh(None, None, "shaded")
        self.settings_panel.set_scene_name(scene.name)
        self.settings_panel.set_path_text(path)
        self._log(f"opened {os.path.basename(path)}: {len(scene.scans)} scans")
        self._refresh_all()
        self.frame_scene()

    def save_scene(self):
        if self.scene.path:
            self._write_scene(self.scene.path)
        else:
            self.save_scene_as()

    def save_scene_as(self):
        base = (self.scene.name or "scene").replace(" ", "_")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save scene", scan_path(base + ".scene"), "Scene (*.scene)")
        if not path:
            return
        if not path.lower().endswith(".scene"):
            path += ".scene"
        self._write_scene(path)

    def _write_scene(self, path):
        self.scene.name = self.settings_panel.scene_name() or self.scene.name
        try:
            scene_io.save_scene(path, self.scene)
        except Exception as exc:
            self._log(f"save failed: {exc}")
            return
        self.settings_panel.set_path_text(path)
        self._log(f"saved scene -> {os.path.basename(path)}")
        self._update_title()

    # --- layout / settings --------------------------------------------------

    def _reset_layout(self):
        if self._default_state is not None:
            self.restoreState(self._default_state)
        for d in self._docks.values():
            d.show()

    def _init_settings(self):
        self.settings = FileSettings(SETTINGS_PATH)
        self.settings.migrate_from_qsettings(
            QtCore.QSettings("LidarScanner", "scanner_ui"))
        n = self.newscan
        v = self.view_panel
        sl = self.slam
        g = self.settings_panel
        self._persist = {
            "port": n.port_box, "dtr": n.dtr_chk, "record": n.record_chk,
            "scan/angle": n.angle_spin, "scan/time": n.time_spin,
            "scan/stepped": n.stepped_chk, "scan/steps": n.steps_spin,
            "scan/dwell": n.dwell_spin, "scan/delay": n.delay_spin,
            "view/mode": v.mode_box, "view/color_by": v.color_box,
            "view/point_size": v.size_spin, "view/voxel": v.voxel_spin,
            "view/max_range": v.range_spin, "view/camera": v.cam_box,
            "view/grid": v.grid_chk,
            "mesh/method": v.algo_box, "mesh/depth": v.depth_spin,
            "mesh/trim": v.trim_spin, "mesh/radius": v.radius_spin,
            "mesh/alpha": v.alpha_spin, "mesh/smooth": v.smooth_spin,
            "mesh/surface": v.surface_box,
            "slam/align": sl.align_box, "slam/yaw": sl.yaw_spin,
            "slam/voxel": sl.voxel_spin, "slam/foliage": sl.foliage_chk,
            "geom/lidar_rotation": g.lidar_rot_spin,
            "geom/lidar_reverse": g.reverse_chk,
            "geom/emitter_spacing": g.spacing_spin,
            "geom/scan_half": g.half_box, "geom/flip_upright": g.flip_chk,
            "geom/microstep_error": g.ustep_spin,
            "geom/microstep_phase": g.ustep_phase_spin,
        }
        self._restore_settings()
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
                pass
            finally:
                w.blockSignals(False)
        # Apply the restored view/camera/grid state to the live objects.
        self.newscan._on_mode_changed()
        self.view_panel._on_algo_changed()
        self.settings_panel._on_half_changed()
        self.renderer.set_grid_visible(self.view_panel.grid_chk.isChecked())
        self.view.set_cam_mode(self.view_panel.camera_mode())
        self.on_view_mode_changed()

    def _save_settings(self, *_):
        for key, w in self._persist.items():
            if isinstance(w, QtWidgets.QCheckBox):
                self.settings.setValue(key, w.isChecked())
            elif isinstance(w, QtWidgets.QComboBox):
                self.settings.setValue(key, w.currentText())
            else:
                self.settings.setValue(key, w.value())
        self.settings.sync()

    # --- misc ---------------------------------------------------------------

    def _confirm(self, title, text):
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(title)
        box.setText(text)
        yes = _enum_msg("Yes")
        no = _enum_msg("No")
        box.setStandardButtons(yes | no)
        box.setDefaultButton(no)
        return box.exec() == yes

    def _log(self, text):
        self.statusBar().showMessage(text, 6000)
        print(text, file=sys.stderr)

    def showEvent(self, ev):
        super().showEvent(ev)
        if self._default_state is None:
            # Capture the pristine layout once the docks have their real sizes,
            # so Reset layout has something to return to.
            self._default_state = self.saveState()

    def closeEvent(self, ev):
        self._save_settings()
        self._disconnect()
        for worker in (self._mesh_worker, self._reg_worker):
            if worker is not None and worker.isRunning():
                worker.wait(5000)
        super().closeEvent(ev)


def _enum_area(name):
    from pyqtgraph.Qt import QtCore as _q
    return getattr(_q.Qt.DockWidgetArea, name, None) \
        if hasattr(_q.Qt, "DockWidgetArea") else getattr(_q.Qt, name)


def _enum_msg(name):
    sb = QtWidgets.QMessageBox.StandardButton if hasattr(
        QtWidgets.QMessageBox, "StandardButton") else QtWidgets.QMessageBox
    return getattr(sb, name)


def main():
    app = pg.mkQApp("3D Lidar Scanner")
    apply_dark_theme(app)
    ui = ScannerUI()
    ui.show()
    sys.exit(app.exec_() if hasattr(app, "exec_") else app.exec())


if __name__ == "__main__":
    main()
