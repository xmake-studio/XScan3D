#!/usr/bin/env python3
"""Adding scans to the scene: connect to the device and sweep, or load a file.

Every scan the scene holds enters through here. A device sweep appears in the
Scene panel as a GENERATING object while it runs and settles into a normal scan
when it finishes; Load .ply / Open .bin add a scan straight away. The device and
sweep controls are the old Device and Scan groups, unchanged in behaviour.
"""

from pyqtgraph.Qt import QtWidgets

from widgets import Collapsible, TOOLTIP_ROLE


class NewScanPanel(QtWidgets.QWidget):
    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self._build()

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(4)
        v.addWidget(Collapsible(self._device_group()))
        v.addWidget(Collapsible(self._scan_group()))
        v.addWidget(Collapsible(self._file_group()))
        v.addStretch(1)
        self._on_mode_changed()

    # --- device -------------------------------------------------------------

    def _device_group(self):
        g = QtWidgets.QGroupBox("Device")
        f = QtWidgets.QGridLayout(g)
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(140)
        f.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        f.addWidget(self.port_box, 0, 1)
        b = QtWidgets.QPushButton("Refresh")
        b.clicked.connect(self.ctrl.refresh_ports)
        f.addWidget(b, 0, 2)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.ctrl.toggle_connect)
        f.addWidget(self.connect_btn, 1, 0, 1, 3)

        self.auto_chk = QtWidgets.QCheckBox("Auto-connect to the scanner")
        self.auto_chk.setChecked(True)
        self.auto_chk.setToolTip(
            "Watch for the scanner and connect as soon as it is plugged in, "
            "including after a replug or a firmware upload. It is recognised "
            "by its USB descriptor only, so other RP2040 boards are never "
            "opened. Disconnect by hand to pause it until the next replug.")
        self.auto_chk.stateChanged.connect(self.ctrl.on_auto_connect_changed)
        f.addWidget(self.auto_chk, 2, 0, 1, 3)

        self.dtr_chk = QtWidgets.QCheckBox("Assert DTR (needed by RP2040 USB)")
        self.dtr_chk.setChecked(True)
        self.dtr_chk.setToolTip(
            "The RP2040's USB serial only sends once the host raises DTR. If "
            "the device connects but stays silent, try toggling this.")
        f.addWidget(self.dtr_chk, 3, 0, 1, 3)

        self.record_chk = QtWidgets.QCheckBox("Record raw stream alongside scan")
        self.record_chk.setChecked(False)
        self.record_chk.setToolTip(
            "Write a scan_*.bin into scans/autosaves as the scan runs. Off by "
            "default: the same bytes are kept in memory either way, so Save .bin "
            "can write them afterwards.")
        f.addWidget(self.record_chk, 4, 0, 1, 3)

        self.link_lbl = QtWidgets.QLabel("not connected")
        self.link_lbl.setWordWrap(True)
        f.addWidget(self.link_lbl, 5, 0, 1, 3)
        return g

    # --- scan ---------------------------------------------------------------

    def _scan_group(self):
        g = QtWidgets.QGroupBox("Scan")
        f = QtWidgets.QGridLayout(g)

        self.angle_spin = QtWidgets.QDoubleSpinBox()
        self.angle_spin.setRange(1.0, 180.0)
        self.angle_spin.setValue(90.0)
        self.angle_spin.setSuffix(" deg")
        self.angle_spin.setToolTip(
            "Half-sweep: the shaft runs -this to +this about the vertical. 90 "
            "already covers the whole scene in both-halves mode. One-side "
            "scanning needs ±180.")
        f.addWidget(QtWidgets.QLabel("Angle  ±"), 0, 0)
        f.addWidget(self.angle_spin, 0, 1)

        self.time_spin = QtWidgets.QDoubleSpinBox()
        self.time_spin.setRange(1.0, 6000.0)
        self.time_spin.setValue(30.0)
        self.time_spin.setSuffix(" s")
        f.addWidget(QtWidgets.QLabel("Duration"), 1, 0)
        f.addWidget(self.time_spin, 1, 1)

        self.stepped_chk = QtWidgets.QCheckBox("Stepped (stop at each angle)")
        self.stepped_chk.setToolTip(
            "Moves, stops, waits out the ring-down, then captures with the "
            "shaft still. Slower, much less sensitive to vibration.")
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
        self.dwell_spin.setToolTip("Lidar capture time at each stop.")
        self.dwell_lbl = QtWidgets.QLabel("Dwell")
        f.addWidget(self.dwell_lbl, 4, 0)
        f.addWidget(self.dwell_spin, 4, 1)

        self.est_lbl = QtWidgets.QLabel("")
        self.est_lbl.setStyleSheet("color: #9a9a9a;")
        f.addWidget(self.est_lbl, 5, 0, 1, 2)
        for w in (self.steps_spin, self.dwell_spin, self.angle_spin):
            w.valueChanged.connect(self._update_estimate)

        self.start_btn = QtWidgets.QPushButton("Start scan")
        self.start_btn.setToolTip(
            "Push the settings above to the device, then sweep. The result is a "
            "new scan object in the scene.")
        self.start_btn.clicked.connect(self.ctrl.start_scan)
        f.addWidget(self.start_btn, 6, 0)
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.clicked.connect(self.ctrl.stop_scan)
        f.addWidget(self.stop_btn, 6, 1)

        self.delay_spin = QtWidgets.QSpinBox()
        self.delay_spin.setRange(0, 600)
        self.delay_spin.setValue(0)
        self.delay_spin.setSuffix(" s")
        self.delay_spin.setToolTip(
            "Wait this many seconds after Start before the sweep begins, to "
            "step out of the scan volume. 0 starts immediately.")
        self.delay_lbl = QtWidgets.QLabel("Start delay")
        f.addWidget(self.delay_lbl, 7, 0)
        f.addWidget(self.delay_spin, 7, 1)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setTextVisible(True)
        f.addWidget(self.progress, 8, 0, 1, 2)

        self.state_lbl = QtWidgets.QLabel("idle")
        f.addWidget(self.state_lbl, 9, 0, 1, 2)
        return g

    # --- files --------------------------------------------------------------

    def _file_group(self):
        g = QtWidgets.QGroupBox("Load / Save")
        f = QtWidgets.QVBoxLayout(g)
        row1 = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("Load .ply")
        b1.setToolTip("Add a saved point cloud to the scene as a new scan.")
        b1.clicked.connect(self.ctrl.load_ply)
        row1.addWidget(b1)
        b2 = QtWidgets.QPushButton("Open .bin")
        b2.setToolTip("Rebuild a scan from a recorded raw stream and add it.")
        b2.clicked.connect(self.ctrl.open_bin)
        row1.addWidget(b2)
        f.addLayout(row1)

        row2 = QtWidgets.QHBoxLayout()
        b3 = QtWidgets.QPushButton("Save selection .ply")
        b3.setToolTip("Write the selected scans (or all visible, if none "
                      "selected) to a .ply point cloud.")
        b3.clicked.connect(self.ctrl.save_ply)
        row2.addWidget(b3)
        b4 = QtWidgets.QPushButton("Save selection .bin")
        b4.setToolTip("Write the selected scan's raw capture to a .bin. "
                      "Available for device or .bin-loaded scans.")
        b4.clicked.connect(self.ctrl.save_selected_bin)
        row2.addWidget(b4)
        f.addLayout(row2)
        return g

    # --- helpers ------------------------------------------------------------

    def _on_mode_changed(self):
        stepped = self.stepped_chk.isChecked()
        for w in (self.steps_lbl, self.steps_spin,
                  self.dwell_lbl, self.dwell_spin):
            w.setEnabled(stepped)
        self.time_spin.setEnabled(not stepped)
        self._update_estimate()

    def _update_estimate(self):
        if not self.stepped_chk.isChecked():
            self.est_lbl.setText("")
            return
        stops = self.steps_spin.value() + 1
        secs = stops * (250 + self.dwell_spin.value()) / 1000.0
        arc = 2 * self.angle_spin.value() / self.steps_spin.value()
        self.est_lbl.setText(
            f"{stops} stops of {arc:.2f} deg, at least {secs / 60:.1f} min")

    # --- accessors for the controller ---------------------------------------

    def scan_settings(self):
        return dict(angle=self.angle_spin.value(), time=self.time_spin.value(),
                    stepped=self.stepped_chk.isChecked(),
                    steps=self.steps_spin.value(), dwell=self.dwell_spin.value(),
                    delay=self.delay_spin.value())

    def port(self):
        return self.port_box.currentText()

    def dtr(self):
        return self.dtr_chk.isChecked()

    def record(self):
        return self.record_chk.isChecked()

    def auto_connect(self):
        return self.auto_chk.isChecked()

    def select_port(self, device):
        i = self.port_box.findText(device)
        if i >= 0:
            self.port_box.setCurrentIndex(i)

    def set_ports(self, ports):
        current = self.port_box.currentText()
        self.port_box.clear()
        for device, desc in ports:
            self.port_box.addItem(device, desc)
            idx = self.port_box.count() - 1
            self.port_box.setItemData(idx, f"{device} - {desc}", TOOLTIP_ROLE)
        if current:
            i = self.port_box.findText(current)
            if i >= 0:
                self.port_box.setCurrentIndex(i)
        if self.port_box.count() == 0:
            self.port_box.addItem("(no ports found)")

    def set_connected(self, on):
        self.connect_btn.setText("Disconnect" if on else "Connect")

    def set_link_text(self, text, warn=False):
        self.link_lbl.setText(text)
        self.link_lbl.setStyleSheet(
            "color: #ef5350; font-weight: bold;" if warn else "")

    def set_state_text(self, text):
        self.state_lbl.setText(text)

    def set_progress(self, pct):
        self.progress.setValue(int(pct))

    def set_start_enabled(self, on):
        self.start_btn.setEnabled(on)
