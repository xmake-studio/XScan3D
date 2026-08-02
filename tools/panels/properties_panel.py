#!/usr/bin/env python3
"""Properties of the active scan.

This panel always shows the one scan the last click landed on -- the active
scan -- so there is never a doubt about which object an edit will affect. When
nothing is selected it says so rather than showing stale controls. Editing here
writes straight back to the scan: its name, visibility, colour override, and the
pose that places it in the scene (a manual nudge for touching up an alignment).
"""

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from scene_model import identity

_SOURCE_NAME = {"device": "device scan", "ply": "loaded .ply",
                "bin": "loaded .bin", "merge": "merge result"}


class PropertiesPanel(QtWidgets.QWidget):
    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self._scan_id = None
        self._updating = False
        self._build()

    def _build(self):
        self.stack = QtWidgets.QStackedLayout(self)

        # Empty state.
        empty = QtWidgets.QLabel(
            "No scan selected.\n\nSelect a scan in the Scene panel to see and "
            "edit its properties.")
        empty.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter
                           if hasattr(QtCore.Qt, "AlignmentFlag")
                           else QtCore.Qt.AlignCenter)
        empty.setWordWrap(True)
        empty.setStyleSheet("color: #9a9a9a;")
        self.stack.addWidget(empty)

        # Editing state.
        page = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(page)
        f.setContentsMargins(6, 6, 6, 6)

        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.editingFinished.connect(self._name_changed)
        f.addRow("Name", self.name_edit)

        self.info_lbl = QtWidgets.QLabel()
        self.info_lbl.setStyleSheet("color: #9a9a9a;")
        f.addRow("Info", self.info_lbl)

        self.visible_chk = QtWidgets.QCheckBox("Visible")
        self.visible_chk.stateChanged.connect(self._visible_changed)
        f.addRow("", self.visible_chk)

        self.color_btn = QtWidgets.QPushButton("Set colour…")
        self.color_btn.clicked.connect(self._pick_color)
        self.color_clear = QtWidgets.QPushButton("Auto")
        self.color_clear.setToolTip("Clear the override; follow the view's "
                                    "colour mode.")
        self.color_clear.clicked.connect(self._clear_color)
        crow = QtWidgets.QHBoxLayout()
        crow.addWidget(self.color_btn)
        crow.addWidget(self.color_clear)
        cwrap = QtWidgets.QWidget()
        cwrap.setLayout(crow)
        f.addRow("Colour", cwrap)

        # --- transform ---
        f.addRow(QtWidgets.QLabel("<b>Transform</b>"))
        self.pos_lbls = []
        self.pos_spins = []
        for axis in ("X", "Y", "Z"):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-1e6, 1e6)
            spin.setDecimals(1)
            spin.setSingleStep(10.0)
            spin.setSuffix(" mm")
            spin.valueChanged.connect(self._transform_changed)
            self.pos_spins.append(spin)
            f.addRow(f"Pos {axis}", spin)

        self.yaw_spin = QtWidgets.QDoubleSpinBox()
        self.yaw_spin.setRange(-180.0, 180.0)
        self.yaw_spin.setDecimals(1)
        self.yaw_spin.setSuffix(" deg")
        self.yaw_spin.setWrapping(True)
        self.yaw_spin.setToolTip(
            "Yaw about the vertical through the scan's own centre. A manual "
            "touch-up for an alignment; combine with Un-merge / Re-run SLAM in "
            "the SLAM panel for larger corrections.")
        self.yaw_spin.valueChanged.connect(self._transform_changed)
        f.addRow("Yaw", self.yaw_spin)

        self.reset_btn = QtWidgets.QPushButton("Reset transform")
        self.reset_btn.setToolTip("Return this scan to its own frame "
                                  "(identity pose).")
        self.reset_btn.clicked.connect(self._reset_transform)
        f.addRow("", self.reset_btn)

        self.stack.addWidget(page)

    # --- refresh ------------------------------------------------------------

    def refresh(self):
        scan = self.ctrl.scene.active()
        if scan is None:
            self._scan_id = None
            self.stack.setCurrentIndex(0)
            return
        self._scan_id = scan.id
        self.stack.setCurrentIndex(1)
        self._updating = True
        self.name_edit.setText(scan.name)
        grp = ""
        if scan.group:
            grp = f" · group: {self.ctrl.scene.group_name(scan.group)}"
        gen = " · GENERATING" if scan.generating else ""
        self.info_lbl.setText(
            f"{_SOURCE_NAME.get(scan.source, scan.source)} · "
            f"{scan.n_points:,} points{grp}{gen}")
        self.visible_chk.setChecked(scan.visible)

        px, py, pz = scan.T[:3, 3]
        for spin, val in zip(self.pos_spins, (px, py, pz)):
            spin.setValue(float(val))
        import numpy as np
        yaw = np.degrees(np.arctan2(scan.T[1, 0], scan.T[0, 0]))
        self.yaw_spin.setValue(float(yaw))

        c = scan.color
        if c is None:
            self.color_btn.setStyleSheet("")
            self.color_btn.setText("Set colour…")
        else:
            self.color_btn.setStyleSheet(
                "background-color: rgb(%d,%d,%d);" %
                tuple(int(x * 255) for x in c))
            self.color_btn.setText("")
        self._updating = False

    # --- edits --------------------------------------------------------------

    def _active(self):
        return self.ctrl.scene.get(self._scan_id) if self._scan_id else None

    def _name_changed(self):
        s = self._active()
        if s and not self._updating and self.name_edit.text().strip():
            self.ctrl.on_rename(s.id, self.name_edit.text().strip())

    def _visible_changed(self):
        s = self._active()
        if s and not self._updating:
            self.ctrl.on_visibility_toggled(s.id, self.visible_chk.isChecked())

    def _pick_color(self):
        s = self._active()
        if not s:
            return
        col = QtWidgets.QColorDialog.getColor(parent=self)
        if col.isValid():
            self.ctrl.on_color_override(
                s.id, (col.redF(), col.greenF(), col.blueF()))

    def _clear_color(self):
        s = self._active()
        if s:
            self.ctrl.on_color_override(s.id, None)

    def _transform_changed(self):
        s = self._active()
        if not s or self._updating:
            return
        import numpy as np
        yaw = np.radians(self.yaw_spin.value())
        c, s_ = np.cos(yaw), np.sin(yaw)
        T = identity()
        # Yaw about the scan's local centroid so the spin feels like turning the
        # object in place rather than orbiting it about the world origin.
        centroid = s.xyz.mean(axis=0) if s.n_points else np.zeros(3)
        R = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]], float)
        T[:3, :3] = R
        T[:3, 3] = centroid - R @ centroid
        T[0, 3] += self.pos_spins[0].value()
        T[1, 3] += self.pos_spins[1].value()
        T[2, 3] += self.pos_spins[2].value()
        self.ctrl.on_transform_changed(s.id, T)

    def _reset_transform(self):
        s = self._active()
        if s:
            self.ctrl.on_transform_changed(s.id, identity())
            self.refresh()
