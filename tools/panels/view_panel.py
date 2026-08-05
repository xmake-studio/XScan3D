#!/usr/bin/env python3
"""How the scene is drawn: colouring, point size, camera, and the points-vs-
surface switch with its reconstruction controls.

Surface mode is whole-scene: it reconstructs one mesh from every visible scan's
points, so it is a way of viewing the scene rather than a property of a single
scan. The reconstruction controls follow the chosen method, exactly as in the
original UI, and only ever repaint or rebuild -- they never touch the scans.
"""

from pyqtgraph.Qt import QtWidgets

import meshing
from scene_view import (COLOR_OBJECT, COLOR_GROUP, COLOR_HEIGHT,
                        COLOR_DISTANCE, CAM_ORBIT, CAM_FPS)


class ViewPanel(QtWidgets.QWidget):
    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self._build()

    def _build(self):
        f = QtWidgets.QFormLayout(self)
        f.setContentsMargins(6, 6, 6, 6)

        self.mode_box = QtWidgets.QComboBox()
        self.mode_box.addItem("Points", "points")
        self.mode_box.addItem("Surface", "surface")
        self.mode_box.setToolTip(
            "Points draws the clouds.\n"
            "Surface reconstructs one triangle mesh from every visible scan -- "
            "slower, and it does not update live during a scan.")
        self.mode_box.currentIndexChanged.connect(self.ctrl.on_view_mode_changed)
        f.addRow("View", self.mode_box)

        self.color_box = QtWidgets.QComboBox()
        self.color_box.addItem("By object", COLOR_OBJECT)
        self.color_box.addItem("By group (merged/loose)", COLOR_GROUP)
        self.color_box.addItem("By height", COLOR_HEIGHT)
        self.color_box.addItem("By distance", COLOR_DISTANCE)
        self.color_box.setToolTip(
            "By object: each scan its own colour, to tell them apart.\n"
            "By group: merged groups share a colour, loose scans are grey.\n"
            "By height / distance: the measurement colour ramp.")
        self.color_box.currentIndexChanged.connect(self.ctrl.on_color_mode_changed)
        f.addRow("Colour by", self.color_box)

        self.size_spin = QtWidgets.QDoubleSpinBox()
        self.size_spin.setRange(0.5, 12.0)
        self.size_spin.setValue(2.0)
        self.size_spin.setSingleStep(0.5)
        self.size_spin.valueChanged.connect(self.ctrl.on_point_size_changed)
        f.addRow("Point size", self.size_spin)

        self.voxel_spin = QtWidgets.QDoubleSpinBox()
        self.voxel_spin.setRange(0.0, 500.0)
        self.voxel_spin.setValue(0.0)
        self.voxel_spin.setSuffix(" mm")
        self.voxel_spin.setToolTip(
            "Thin new scans to one point per grid cell as they are built. "
            "0 keeps every point. Applies to scans built from the device or a "
            ".bin; already-loaded clouds are not re-thinned.")
        self.voxel_spin.valueChanged.connect(self.ctrl.on_build_filter_changed)
        f.addRow("Voxel", self.voxel_spin)

        self.range_spin = QtWidgets.QDoubleSpinBox()
        self.range_spin.setRange(0.0, 50000.0)
        self.range_spin.setValue(0.0)
        self.range_spin.setSuffix(" mm")
        self.range_spin.setToolTip("Drop points beyond this range when building "
                                   "a scan. 0 = no limit.")
        self.range_spin.valueChanged.connect(self.ctrl.on_build_filter_changed)
        f.addRow("Max range", self.range_spin)

        self.cam_box = QtWidgets.QComboBox()
        self.cam_box.addItem("Orbit (Blender)", CAM_ORBIT)
        self.cam_box.addItem("Fly / FPS (Unity)", CAM_FPS)
        self.cam_box.setToolTip(
            "Orbit: drag swings around a point; wheel zooms; middle/shift/alt-"
            "drag slides the pivot.\n"
            "Fly: drag looks around, WASD walks, E/Space up, Q/Ctrl down, Shift "
            "faster, wheel sets speed. Keys work whenever the pointer is over "
            "the view.")
        self.cam_box.currentIndexChanged.connect(self.ctrl.on_camera_mode_changed)
        f.addRow("Camera", self.cam_box)

        self.grid_chk = QtWidgets.QCheckBox("Show grid")
        self.grid_chk.setChecked(True)
        self.grid_chk.stateChanged.connect(
            lambda: self.ctrl.on_grid_toggled(self.grid_chk.isChecked()))
        f.addRow("", self.grid_chk)

        reset = QtWidgets.QPushButton("Frame scene")
        reset.setToolTip("Move the camera to fit every visible scan.")
        reset.clicked.connect(self.ctrl.frame_scene)
        f.addRow("", reset)

        # --- surface / meshing ---
        self.surf_group = QtWidgets.QGroupBox("Surface reconstruction")
        sf = QtWidgets.QFormLayout(self.surf_group)

        self.algo_box = QtWidgets.QComboBox()
        for key in meshing.available_methods():
            self.algo_box.addItem(meshing.METHODS[key][0], key)
        self.algo_box.setToolTip(
            "Poisson: watertight and smooth.\n"
            "Ball pivoting: keeps measured points, leaves real gaps.\n"
            "Alpha shape: crude but dependency-free.")
        self.algo_box.currentIndexChanged.connect(self._on_algo_changed)
        sf.addRow("Method", self.algo_box)

        self.depth_spin = QtWidgets.QSpinBox()
        self.depth_spin.setRange(5, 12)
        self.depth_spin.setValue(9)
        self.depth_lbl = QtWidgets.QLabel("Detail")
        sf.addRow(self.depth_lbl, self.depth_spin)

        self.trim_spin = QtWidgets.QDoubleSpinBox()
        self.trim_spin.setRange(0.0, 40.0)
        self.trim_spin.setValue(3.0)
        self.trim_spin.setSingleStep(0.5)
        self.trim_spin.setSuffix(" x")
        self.trim_lbl = QtWidgets.QLabel("Trim")
        sf.addRow(self.trim_lbl, self.trim_spin)

        self.radius_spin = QtWidgets.QDoubleSpinBox()
        self.radius_spin.setRange(0.5, 20.0)
        self.radius_spin.setValue(2.0)
        self.radius_spin.setSingleStep(0.5)
        self.radius_spin.setSuffix(" x")
        self.radius_lbl = QtWidgets.QLabel("Ball radius")
        sf.addRow(self.radius_lbl, self.radius_spin)

        self.alpha_spin = QtWidgets.QDoubleSpinBox()
        self.alpha_spin.setRange(1.0, 40.0)
        self.alpha_spin.setValue(4.0)
        self.alpha_spin.setSingleStep(0.5)
        self.alpha_spin.setSuffix(" x")
        self.alpha_lbl = QtWidgets.QLabel("Alpha")
        sf.addRow(self.alpha_lbl, self.alpha_spin)

        self.budget_spin = QtWidgets.QSpinBox()
        self.budget_spin.setRange(0, 2000000)
        self.budget_spin.setSingleStep(10000)
        self.budget_spin.setValue(meshing.METHOD_BUDGET["poisson"])
        self.budget_spin.setGroupSeparatorShown(True)
        self.budget_spin.setToolTip(
            "Thin the cloud to about this many points before reconstructing. "
            "0 feeds the lot. This is the control that decides build time.")
        sf.addRow("Point budget", self.budget_spin)

        self.smooth_spin = QtWidgets.QSpinBox()
        self.smooth_spin.setRange(0, 20)
        self.smooth_spin.setValue(0)
        sf.addRow("Smoothing", self.smooth_spin)

        self.surface_box = QtWidgets.QComboBox()
        self.surface_box.addItems(["shaded", "height", "wireframe"])
        self.surface_box.currentIndexChanged.connect(self.ctrl.on_surface_style_changed)
        sf.addRow("Surface", self.surface_box)

        self.mesh_btn = QtWidgets.QPushButton("Build surface")
        self.mesh_btn.clicked.connect(self.ctrl.build_surface)
        sf.addRow(self.mesh_btn)

        self.mesh_lbl = QtWidgets.QLabel("no surface built")
        self.mesh_lbl.setWordWrap(True)
        self.mesh_lbl.setStyleSheet("color: #9a9a9a;")
        sf.addRow(self.mesh_lbl)

        note = meshing.missing_note()
        if note:
            warn = QtWidgets.QLabel(note.splitlines()[0])
            warn.setWordWrap(True)
            warn.setToolTip(note)
            warn.setStyleSheet("color: #e0a030;")
            sf.addRow(warn)

        # Parameter changes no longer auto-trigger builds; use "Build surface" button only

        f.addRow(self.surf_group)
        self._on_algo_changed()
        self.surf_group.setVisible(False)

    # --- getters used by the controller -------------------------------------

    def color_mode(self):
        return self.color_box.currentData()

    def camera_mode(self):
        return self.cam_box.currentData()

    def point_size(self):
        return self.size_spin.value()

    def is_surface(self):
        return self.mode_box.currentData() == "surface"

    def surface_style(self):
        return self.surface_box.currentText()

    def build_filters(self):
        return {"max_range": self.range_spin.value() or None,
                "voxel": self.voxel_spin.value()}

    def mesh_params(self):
        return dict(method=self.algo_box.currentData(),
                    depth=self.depth_spin.value(),
                    trim_mult=self.trim_spin.value(),
                    radius_mult=self.radius_spin.value(),
                    alpha_mult=self.alpha_spin.value(),
                    smooth_iters=self.smooth_spin.value(),
                    budget=self.budget_spin.value())

    def set_mesh_label(self, text):
        self.mesh_lbl.setText(text)

    def set_mesh_button_enabled(self, on):
        self.mesh_btn.setEnabled(on)

    def show_surface_controls(self, on):
        self.surf_group.setVisible(on)

    # --- internal -----------------------------------------------------------

    def _on_algo_changed(self):
        method = self.algo_box.currentData()
        for w in (self.depth_lbl, self.depth_spin, self.trim_lbl, self.trim_spin):
            w.setVisible(method == "poisson")
        for w in (self.radius_lbl, self.radius_spin):
            w.setVisible(method == "bpa")
        for w in (self.alpha_lbl, self.alpha_spin):
            w.setVisible(method == "alpha")
        want = meshing.METHOD_BUDGET.get(method)
        if want and self.budget_spin.value() != want:
            self.budget_spin.blockSignals(True)
            self.budget_spin.setValue(want)
            self.budget_spin.blockSignals(False)
