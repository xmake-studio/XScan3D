#!/usr/bin/env python3
"""Scene-level settings and the hardware mount geometry.

Two things live here. The scene bits -- name, units, and the .scene file
actions -- describe the project. The mount geometry -- lidar roll, azimuth
direction, emitter spacing, scan half, upright flip, scan-plane tilt --
describes how the sensor is bolted together, so it is dialled in once (by eye or
with Calibrate from scan) and then applies to every scan built from the
device or a .bin. Changing it rebuilds those scans in place, which is how the
emitter spacing is tuned: watch a flat surface and wind the knob until it
flattens. The microstep correction is the exception: it is measured per scan.
"""

from pyqtgraph.Qt import QtWidgets

from widgets import Collapsible
import scan_proto as sp


class SceneSettingsPanel(QtWidgets.QWidget):
    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self._calib_running = False
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

        self.tilt_spin = QtWidgets.QDoubleSpinBox()
        self.tilt_spin.setRange(-10.0, 10.0)
        self.tilt_spin.setDecimals(2)
        self.tilt_spin.setSingleStep(0.05)
        self.tilt_spin.setValue(sp.SCAN_TILT_DEG)
        self.tilt_spin.setSuffix(" deg")
        self.tilt_spin.setToolTip(
            "How far the lidar's scan plane leans off the rotation axis. "
            "Invisible at eye level; wrong, it twists a nearby wall into a "
            "saddle and smears anything overhead around the zenith. Set it with "
            "Calibrate from scan.")
        self.tilt_spin.valueChanged.connect(self.ctrl.on_geometry_changed)
        f.addWidget(QtWidgets.QLabel("Scan-plane tilt"), r, 0)
        f.addWidget(self.tilt_spin, r, 1)
        r += 1

        self.ustep_chk = QtWidgets.QCheckBox("Auto microstep correction")
        self.ustep_chk.setChecked(True)
        self.ustep_chk.setToolTip(
            "Measures how far the rotor sits from each commanded microstep "
            "from the flat surfaces in the scan itself, and corrects for it. "
            "Removes the waves a long wall picks up in step with the motor. "
            "Fitted once per finished scan, which takes a second or two.")
        self.ustep_chk.stateChanged.connect(self.ctrl.on_geometry_changed)
        f.addWidget(self.ustep_chk, r, 0, 1, 2)
        r += 1

        self.calib_btn = QtWidgets.QPushButton("Calibrate from scan…")
        self.calib_btn.setToolTip(
            "Fits Lidar roll, Emitter spacing and Scan-plane tilt to the "
            "selected scan (or the latest one): walls and ceiling flat, the "
            "two ends of the sweep meeting cleanly. Needs a ±90° sweep of an "
            "ordinary room. Takes a few minutes; you choose whether to apply "
            "the result.")
        self.calib_btn.clicked.connect(self._on_calib_clicked)
        f.addWidget(self.calib_btn, r, 0, 1, 2)
        r += 1

        self.calib_bar = QtWidgets.QProgressBar()
        self.calib_bar.setRange(0, 1000)
        self.calib_bar.setTextVisible(False)
        self.calib_bar.setMaximumHeight(8)
        self.calib_bar.hide()
        f.addWidget(self.calib_bar, r, 0, 1, 2)
        r += 1

        self.calib_lbl = QtWidgets.QLabel("")
        self.calib_lbl.setWordWrap(True)
        self.calib_lbl.setStyleSheet("color: #9a9a9a; font-size: 11px;")
        self.calib_lbl.hide()
        f.addWidget(self.calib_lbl, r, 0, 1, 2)
        r += 1
        return g

    # --- calibration --------------------------------------------------------

    def _on_calib_clicked(self):
        if self._calib_running:
            self.ctrl.cancel_calibration()
        else:
            self.ctrl.start_calibration()

    def set_calibration_running(self, running):
        self._calib_running = running
        self.calib_btn.setText("Cancel calibration" if running
                               else "Calibrate from scan…")
        self.calib_bar.setVisible(running)
        if running:
            self.calib_bar.setValue(0)
        # The fit starts from these; changing them mid-run would not be seen.
        for w in (self.lidar_rot_spin, self.spacing_spin, self.tilt_spin,
                  self.reverse_chk):
            w.setEnabled(not running)

    def set_calibration_progress(self, text, fraction):
        self.calib_bar.setValue(int(fraction * 1000))
        self.set_calibration_text(text)

    def set_calibration_text(self, text):
        self.calib_lbl.setText(text)
        self.calib_lbl.setVisible(bool(text))

    def set_mount(self, rotation, spacing, tilt):
        """Set the three fitted values with one rebuild instead of three."""
        for w, v in ((self.lidar_rot_spin, rotation),
                     (self.spacing_spin, spacing), (self.tilt_spin, tilt)):
            w.blockSignals(True)
            w.setValue(v)
            w.blockSignals(False)
        self.ctrl.on_geometry_changed()

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
                    scan_tilt=self.tilt_spin.value(),
                    microstep="auto" if self.ustep_chk.isChecked() else None)

    def scene_name(self):
        return self.name_edit.text().strip()

    def set_scene_name(self, name):
        self.name_edit.setText(name)

    def set_path_text(self, text):
        self.path_lbl.setText(text)
