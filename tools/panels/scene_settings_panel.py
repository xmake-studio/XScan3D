#!/usr/bin/env python3
"""Scene-level settings and the hardware mount geometry.

Two things live here. The scene bits -- name, units, and the .scene file
actions -- describe the project. The mount geometry -- lidar roll, azimuth
direction, emitter spacing, scan half, upright flip, microstep correction --
describes how the sensor is bolted together, so it is dialled in once by eye and
then applies to every scan built from the device or a .bin. Changing it rebuilds
those scans in place, which is how the emitter spacing and microstep are tuned:
watch a flat surface and wind the knob until it flattens.
"""

from pyqtgraph.Qt import QtWidgets

from widgets import Collapsible
import scan_proto as sp


class SceneSettingsPanel(QtWidgets.QWidget):
    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self._build()

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(4)
        v.addWidget(Collapsible(self._scene_group()))
        v.addWidget(Collapsible(self._geometry_group()))
        v.addStretch(1)
        self._on_half_changed()

    def _scene_group(self):
        g = QtWidgets.QGroupBox("Scene")
        f = QtWidgets.QFormLayout(g)

        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.setPlaceholderText("Untitled scene")
        self.name_edit.editingFinished.connect(
            lambda: self.ctrl.on_scene_name_changed(self.name_edit.text()))
        f.addRow("Name", self.name_edit)

        row = QtWidgets.QHBoxLayout()
        for label, slot in (("New", self.ctrl.new_scene),
                            ("Open…", self.ctrl.open_scene),
                            ("Save", self.ctrl.save_scene),
                            ("Save As…", self.ctrl.save_scene_as)):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(slot)
            row.addWidget(b)
        wrap = QtWidgets.QWidget()
        wrap.setLayout(row)
        f.addRow(wrap)

        self.path_lbl = QtWidgets.QLabel("not saved yet")
        self.path_lbl.setWordWrap(True)
        self.path_lbl.setStyleSheet("color: #9a9a9a;")
        f.addRow(self.path_lbl)
        return g

    def _geometry_group(self):
        g = QtWidgets.QGroupBox("Mount geometry")
        f = QtWidgets.QGridLayout(g)
        r = 0

        note = QtWidgets.QLabel(
            "Describes how the sensor is mounted. Applies to scans built from "
            "the device or a .bin.")
        note.setWordWrap(True)
        note.setStyleSheet("color: #9a9a9a; font-size: 11px;")
        f.addWidget(note, r, 0, 1, 2)
        r += 1

        self.lidar_rot_spin = QtWidgets.QDoubleSpinBox()
        self.lidar_rot_spin.setRange(-180.0, 180.0)
        self.lidar_rot_spin.setValue(sp.LIDAR_ROTATION_DEG)
        self.lidar_rot_spin.setSingleStep(90.0)
        self.lidar_rot_spin.setSuffix(" deg CW")
        self.lidar_rot_spin.setToolTip(
            "How far the lidar is rolled about its own spin axis. Decides which "
            "direction in the scan plane is up. Try 0/90/180/-90 and keep the "
            "one where the floor is flat.")
        self.lidar_rot_spin.valueChanged.connect(self.ctrl.on_geometry_changed)
        f.addWidget(QtWidgets.QLabel("Lidar roll"), r, 0)
        f.addWidget(self.lidar_rot_spin, r, 1)
        r += 1

        self.reverse_chk = QtWidgets.QCheckBox("Reverse azimuth direction")
        self.reverse_chk.setChecked(sp.LIDAR_REVERSE)
        self.reverse_chk.setToolTip(
            "Which way the lidar's reported angle runs. Getting it wrong "
            "reflects the cloud, which no rotation can undo. Set by eye against "
            "a scene whose handedness you know.")
        self.reverse_chk.stateChanged.connect(self.ctrl.on_geometry_changed)
        f.addWidget(self.reverse_chk, r, 0, 1, 2)
        r += 1

        self.spacing_spin = QtWidgets.QDoubleSpinBox()
        self.spacing_spin.setRange(-200.0, 200.0)
        self.spacing_spin.setDecimals(1)
        self.spacing_spin.setSingleStep(1.0)
        self.spacing_spin.setValue(sp.EMITTER_SPACING_MM)
        self.spacing_spin.setSuffix(" mm")
        self.spacing_spin.setToolTip(
            "Distance between the laser diode and the receiver inside the "
            "lidar. Removes the step/bowl on a flat surface. Measure with "
            "calipers, then tune: the step is smallest at the true value.")
        self.spacing_spin.valueChanged.connect(self.ctrl.on_geometry_changed)
        f.addWidget(QtWidgets.QLabel("Emitter spacing"), r, 0)
        f.addWidget(self.spacing_spin, r, 1)
        r += 1

        self.half_box = QtWidgets.QComboBox()
        for key in (sp.SCAN_HALF_BOTH, sp.SCAN_HALF_A, sp.SCAN_HALF_B):
            self.half_box.addItem(sp.SCAN_HALF_NAMES[key], key)
        self.half_box.setToolTip(
            "Which half of each lidar revolution to keep. Keeping one side "
            "never mixes the two, so a flat surface cannot arrive as two "
            "sheets, but it needs a ±180 sweep to cover the sphere.")
        self.half_box.currentIndexChanged.connect(self._on_half_changed)
        f.addWidget(QtWidgets.QLabel("Scan half"), r, 0)
        f.addWidget(self.half_box, r, 1)
        r += 1

        self.half_lbl = QtWidgets.QLabel("")
        self.half_lbl.setWordWrap(True)
        self.half_lbl.setStyleSheet("color: #e0a030;")
        f.addWidget(self.half_lbl, r, 0, 1, 2)
        r += 1

        self.flip_chk = QtWidgets.QCheckBox("Flip upright (180 deg)")
        self.flip_chk.setChecked(sp.FLIP_UPRIGHT)
        self.flip_chk.setToolTip(
            "Turns the finished cloud 180° about world X, for an inverted "
            "sensor. Rigid, so it changes no measurement.")
        self.flip_chk.stateChanged.connect(self.ctrl.on_geometry_changed)
        f.addWidget(self.flip_chk, r, 0, 1, 2)
        r += 1

        self.ustep_spin = QtWidgets.QDoubleSpinBox()
        self.ustep_spin.setRange(0.0, 0.2)
        self.ustep_spin.setDecimals(4)
        self.ustep_spin.setSingleStep(0.0025)
        self.ustep_spin.setValue(sp.MICROSTEP_ERROR_DEG)
        self.ustep_spin.setSuffix(" deg")
        self.ustep_spin.setToolTip(
            "How far the rotor sits from the microstep the firmware asked for. "
            "Flattens the one-full-step ripple on a wall. Set the phase first; "
            "0 disables. Tune on one long clean wall.")
        self.ustep_spin.valueChanged.connect(self.ctrl.on_geometry_changed)
        f.addWidget(QtWidgets.QLabel("Microstep error"), r, 0)
        f.addWidget(self.ustep_spin, r, 1)
        r += 1

        self.ustep_phase_spin = QtWidgets.QDoubleSpinBox()
        self.ustep_phase_spin.setRange(0.0, 360.0)
        self.ustep_phase_spin.setDecimals(0)
        self.ustep_phase_spin.setSingleStep(15.0)
        self.ustep_phase_spin.setWrapping(True)
        self.ustep_phase_spin.setValue(sp.MICROSTEP_ERROR_PHASE)
        self.ustep_phase_spin.setSuffix(" deg")
        self.ustep_phase_spin.setToolTip(
            "Where within the full step the correction is applied. 360 is one "
            "full step. Sweep this until the ripple is weakest, then trim the "
            "amplitude.")
        self.ustep_phase_spin.valueChanged.connect(self.ctrl.on_geometry_changed)
        f.addWidget(QtWidgets.QLabel("Microstep phase"), r, 0)
        f.addWidget(self.ustep_phase_spin, r, 1)
        r += 1
        return g

    def _on_half_changed(self):
        one_side = self.half_box.currentData() != sp.SCAN_HALF_BOTH
        angle = self.ctrl.newscan.angle_spin.value() if getattr(
            self.ctrl, "newscan", None) else 180.0
        if one_side and angle < 180.0:
            self.half_lbl.setText(
                "One-side scanning covers only half the scene at this angle — "
                "set Angle to ±180 for a full 360° sweep.")
        else:
            self.half_lbl.setText("")
        self.ctrl.on_geometry_changed()

    # --- accessors ----------------------------------------------------------

    def geometry(self):
        return dict(lidar_rotation=self.lidar_rot_spin.value(),
                    lidar_reverse=self.reverse_chk.isChecked(),
                    flip_upright=self.flip_chk.isChecked(),
                    emitter_spacing=self.spacing_spin.value(),
                    half=self.half_box.currentData(),
                    microstep_error=self.ustep_spin.value(),
                    microstep_phase=self.ustep_phase_spin.value())

    def scene_name(self):
        return self.name_edit.text().strip()

    def set_scene_name(self, name):
        self.name_edit.setText(name)

    def set_path_text(self, text):
        self.path_lbl.setText(text)
