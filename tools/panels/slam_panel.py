#!/usr/bin/env python3
"""SLAM: aligning scans into one merged model, reversibly.

Merging picks the selected scans, aligns each onto the growing model with the
registration module, and puts them in a shared group -- the source scans are
never destroyed, so a bad alignment is undone by Un-merge (in the Scene panel)
and tried again from here with Re-run. Because a symmetric room can align two
ways, an alignment that is not clearly right stops for the operator to Keep or
Discard, with the candidate drawn in orange over the model.
"""

from pyqtgraph.Qt import QtWidgets


class SlamPanel(QtWidgets.QWidget):
    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self._build()

    def _build(self):
        f = QtWidgets.QFormLayout(self)
        f.setContentsMargins(6, 6, 6, 6)

        intro = QtWidgets.QLabel(
            "Select two or more scans in the Scene panel, then Merge. The first "
            "defines the frame; the rest are aligned onto it.\n\n"
            "To add to an existing merge, select the group plus the new scan and "
            "Merge: the group is held fixed and the new scan is aligned into it.")
        intro.setWordWrap(True)
        intro.setStyleSheet("color: #9a9a9a;")
        f.addRow(intro)

        self.align_box = QtWidgets.QComboBox()
        self.align_box.addItem("Search all", "auto")
        self.align_box.addItem("Barely moved", "small")
        self.align_box.addItem("Known turn", "hint")
        self.align_box.setToolTip(
            "How much you can tell it about how the rig moved between scans.\n"
            "Search all: no assumptions, slowest, can pick a 180°-out fit in a "
            "bare room.\nBarely moved: assumes near the same spot and heading.\n"
            "Known turn: give the rough heading change below.")
        self.align_box.currentIndexChanged.connect(self._on_align_changed)
        f.addRow("Movement", self.align_box)

        self.yaw_spin = QtWidgets.QDoubleSpinBox()
        self.yaw_spin.setRange(-180.0, 180.0)
        self.yaw_spin.setValue(0.0)
        self.yaw_spin.setSuffix(" deg")
        self.yaw_spin.setToolTip(
            "How far the rig was turned about the vertical between the scans "
            "being merged. Eyeballing it is fine; 20° out still lands.")
        self.yaw_lbl = QtWidgets.QLabel("Turned by")
        f.addRow(self.yaw_lbl, self.yaw_spin)

        self.voxel_spin = QtWidgets.QDoubleSpinBox()
        self.voxel_spin.setRange(5.0, 500.0)
        self.voxel_spin.setValue(40.0)
        self.voxel_spin.setSuffix(" mm")
        self.voxel_spin.setToolTip(
            "Detail the alignment works at. Smaller is more precise and slower; "
            "40 mm suits a room.")
        f.addRow("Detail", self.voxel_spin)

        self.foliage_chk = QtWidgets.QCheckBox("Ignore foliage")
        self.foliage_chk.setChecked(True)
        self.foliage_chk.setToolTip(
            "Align on solid surfaces only -- walls, ground, steps, trunks -- "
            "and ignore trees, bushes and grass.\n"
            "Outdoors this is almost always what you want: leaves and grass "
            "move between scans and never line up, so they only add noise to "
            "the alignment and make a good merge look worse than it is. Turn "
            "off for a bare indoor scan with nothing leafy in it.")
        f.addRow("Vegetation", self.foliage_chk)

        self.merge_btn = QtWidgets.QPushButton("Merge selected")
        self.merge_btn.clicked.connect(self.ctrl.merge_selected)
        f.addRow(self.merge_btn)

        self.rerun_btn = QtWidgets.QPushButton("Re-run SLAM on group")
        self.rerun_btn.setToolTip(
            "Re-align the selected group's scans from scratch with the current "
            "settings -- for when a merge came out wrong.")
        self.rerun_btn.clicked.connect(self.ctrl.rerun_slam)
        f.addRow(self.rerun_btn)

        self.unmerge_btn = QtWidgets.QPushButton("Un-merge selected")
        self.unmerge_btn.clicked.connect(self.ctrl.unmerge_selected)
        f.addRow(self.unmerge_btn)

        # Pending verdict.
        self.verdict_lbl = QtWidgets.QLabel("")
        self.verdict_lbl.setWordWrap(True)
        f.addRow(self.verdict_lbl)

        pend = QtWidgets.QHBoxLayout()
        self.keep_btn = QtWidgets.QPushButton("Keep")
        self.keep_btn.clicked.connect(self.ctrl.accept_pending)
        pend.addWidget(self.keep_btn)
        self.discard_btn = QtWidgets.QPushButton("Discard")
        self.discard_btn.clicked.connect(self.ctrl.reject_pending)
        pend.addWidget(self.discard_btn)
        wrap = QtWidgets.QWidget()
        wrap.setLayout(pend)
        f.addRow(wrap)
        self._pend_wrap = wrap

        self._on_align_changed()
        self.set_pending(False)

    # --- accessors ----------------------------------------------------------

    def params(self):
        return dict(mode=self.align_box.currentData(),
                    yaw=self.yaw_spin.value(),
                    voxel=self.voxel_spin.value(),
                    structure=self.foliage_chk.isChecked())

    def set_verdict(self, text, color=None):
        self.verdict_lbl.setText(text)
        self.verdict_lbl.setStyleSheet(
            f"color: {color};" if color else "color: #9a9a9a;")

    def set_pending(self, on):
        self._pend_wrap.setVisible(on)

    def set_busy(self, busy):
        for b in (self.merge_btn, self.rerun_btn, self.unmerge_btn):
            b.setEnabled(not busy)

    def refresh(self):
        sel = self.ctrl.scene.selected()
        busy = self.ctrl.slam_busy()
        pend = self.ctrl.has_pending()
        self.merge_btn.setEnabled(len(sel) >= 2 and not busy and not pend)
        self.unmerge_btn.setEnabled(any(s.group for s in sel) and not busy)
        self.rerun_btn.setEnabled(
            any(s.group for s in sel) and not busy and not pend)

    # --- internal -----------------------------------------------------------

    def _on_align_changed(self):
        hint = self.align_box.currentData() == "hint"
        self.yaw_lbl.setVisible(hint)
        self.yaw_spin.setVisible(hint)
