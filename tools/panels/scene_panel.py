#!/usr/bin/env python3
"""The Scene outliner -- the one panel that is always on screen.

It is the answer to "what is in my scene and what state is each thing in": a
tree of scans, grouped under their merge groups, each row showing its
visibility, its source, its size, and a badge when it is the one being generated
right now. Selecting here drives the highlight in the 3D view and the contents
of the Properties panel; the toolbar and context menu are where scans are added,
removed, merged and un-merged.
"""

import os

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from widgets import (USER_ROLE, CHECKED, UNCHECKED, PARTIALLY_CHECKED,
                     ITEM_IS_USER_CHECKABLE, ITEM_IS_EDITABLE, EXTENDED_SELECT,
                     CUSTOM_CONTEXT_MENU)
from scene_view import (PALETTE, LOOSE_COLOR, GENERATING_COLOR, COLOR_OBJECT,
                        COLOR_GROUP, COLOR_HEIGHT, COLOR_DISTANCE)

_SOURCE_ICON = {"device": "📡", "ply": "📄", "bin": "💾", "merge": "🧩"}


class ScenePanel(QtWidgets.QWidget):
    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self._updating = False
        self._build()

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(4)

        # --- toolbar ---
        bar = QtWidgets.QHBoxLayout()
        add = QtWidgets.QToolButton()
        add.setText("Add ▾")
        add.setToolTip("Add a scan to the scene")
        menu = QtWidgets.QMenu(add)
        menu.addAction("New scan (device)…", lambda: self.ctrl.focus_newscan())
        menu.addAction("Load .ply…", self.ctrl.load_ply)
        menu.addAction("Open .bin…", self.ctrl.open_bin)
        add.setMenu(menu)
        add.setPopupMode(
            QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup
            if hasattr(QtWidgets.QToolButton, "ToolButtonPopupMode")
            else QtWidgets.QToolButton.InstantPopup)
        bar.addWidget(add)

        self.merge_btn = QtWidgets.QToolButton()
        self.merge_btn.setText("Merge")
        self.merge_btn.setToolTip(
            "Align the selected scans together with SLAM into one merged group. "
            "Select two or more scans first. Two merged groups can also be "
            "selected to merge them into one, each moving as a single cloud.")
        self.merge_btn.clicked.connect(self.ctrl.merge_selected)
        bar.addWidget(self.merge_btn)

        self.unmerge_btn = QtWidgets.QToolButton()
        self.unmerge_btn.setText("Un-merge")
        self.unmerge_btn.setToolTip(
            "Take the selected scans back out of their merged group. Their "
            "aligned pose is kept; nothing is lost.")
        self.unmerge_btn.clicked.connect(self.ctrl.unmerge_selected)
        bar.addWidget(self.unmerge_btn)

        self.del_btn = QtWidgets.QToolButton()
        self.del_btn.setText("Delete")
        self.del_btn.setToolTip("Remove the selected scans from the scene.")
        self.del_btn.clicked.connect(self.ctrl.delete_selected)
        bar.addWidget(self.del_btn)
        bar.addStretch(1)
        v.addLayout(bar)

        # --- tree ---
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setColumnCount(2)
        self.tree.setHeaderLabels(["Scan", "Info"])
        self.tree.setSelectionMode(EXTENDED_SELECT)
        self.tree.setRootIsDecorated(True)
        self.tree.setUniformRowHeights(True)
        self.tree.header().setStretchLastSection(False)
        self.tree.header().setSectionResizeMode(
            0, QtWidgets.QHeaderView.ResizeMode.Stretch
            if hasattr(QtWidgets.QHeaderView, "ResizeMode")
            else QtWidgets.QHeaderView.Stretch)
        self.tree.setContextMenuPolicy(CUSTOM_CONTEXT_MENU)
        self.tree.customContextMenuRequested.connect(self._context_menu)
        self.tree.itemSelectionChanged.connect(self._on_selection)
        self.tree.itemChanged.connect(self._on_item_changed)
        v.addWidget(self.tree, 1)

        # --- clear + legend ---
        self.clear_btn = QtWidgets.QPushButton("Clear scene")
        self.clear_btn.setToolTip(
            "Remove every scan from the scene and start over. Files already "
            "saved to disk are untouched; unsaved scans are lost.")
        self.clear_btn.clicked.connect(self.ctrl.clear_scene)
        v.addWidget(self.clear_btn)

        self.legend = QtWidgets.QLabel()
        self.legend.setWordWrap(True)
        self.legend.setStyleSheet("color: #9a9a9a; font-size: 11px;")
        v.addWidget(self.legend)

        self.count_lbl = QtWidgets.QLabel("empty scene")
        self.count_lbl.setStyleSheet("color: #9a9a9a;")
        v.addWidget(self.count_lbl)

    # --- refresh ------------------------------------------------------------

    def refresh(self):
        scene = self.ctrl.scene
        self._updating = True
        self.tree.blockSignals(True)
        # Remember expansion + rebuild. Scans are few, so a full rebuild is
        # simpler than diffing and never leaves a stale row behind.
        expanded = self._expanded_groups()
        self.tree.clear()

        items = {}
        for gid in scene.group_ids():
            parent = QtWidgets.QTreeWidgetItem(self.tree)
            parent.setData(0, USER_ROLE, ("group", gid))
            members = scene.group_members(gid)
            n = sum(s.n_points for s in members)
            # A checkbox on the group row shows/hides the whole merged group at
            # once; it goes half-checked when only some members are shown.
            parent.setFlags(parent.flags() | ITEM_IS_USER_CHECKABLE
                            | ITEM_IS_EDITABLE)
            vis = scene.group_visible(gid)
            parent.setCheckState(0, CHECKED if vis else
                                 UNCHECKED if vis is False else PARTIALLY_CHECKED)
            parent.setText(0, f"🧩 {scene.group_name(gid)}")
            parent.setText(1, f"{len(members)} scans · {n:,} pts")
            f = parent.font(0)
            f.setBold(True)
            parent.setFont(0, f)
            for s in members:
                self._add_scan_item(parent, s)
            parent.setExpanded(gid not in expanded or expanded.get(gid, True))
            items[gid] = parent

        for s in scene.loose_scans():
            self._add_scan_item(self.tree, s)

        self._restore_selection(scene)
        self.tree.blockSignals(False)
        self._updating = False

        # Buttons reflect the selection.
        sel = scene.selected()
        self.merge_btn.setEnabled(len(sel) >= 2)
        self.unmerge_btn.setEnabled(any(s.group for s in sel))
        self.del_btn.setEnabled(bool(sel))
        self.clear_btn.setEnabled(bool(scene.scans))

        total = sum(s.n_points for s in scene.scans)
        self.count_lbl.setText(
            f"{len(scene.scans)} scans · {total:,} points"
            if scene.scans else "empty scene")
        self._update_legend()

    def _add_scan_item(self, parent, s):
        item = QtWidgets.QTreeWidgetItem(parent)
        item.setData(0, USER_ROLE, ("scan", s.id))
        flags = item.flags() | ITEM_IS_USER_CHECKABLE | ITEM_IS_EDITABLE
        item.setFlags(flags)
        item.setCheckState(0, CHECKED if s.visible else UNCHECKED)
        icon = _SOURCE_ICON.get(s.source, "•")
        item.setText(0, f"{icon} {s.name}")
        badge = "⚡ GENERATING" if s.generating else f"{s.n_points:,} pts"
        item.setText(1, badge)
        if s.generating:
            item.setForeground(1, QtGui.QBrush(QtGui.QColor(*[
                int(c * 255) for c in GENERATING_COLOR])))
        return item

    def _expanded_groups(self):
        out = {}
        for i in range(self.tree.topLevelItemCount()):
            it = self.tree.topLevelItem(i)
            data = it.data(0, USER_ROLE)
            if data and data[0] == "group":
                out[data[1]] = it.isExpanded()
        return out

    def _restore_selection(self, scene):
        def walk(item):
            data = item.data(0, USER_ROLE)
            if data and data[0] == "scan" and scene.is_selected(data[1]):
                item.setSelected(True)
            elif data and data[0] == "group":
                # Mark the group row itself selected when its whole membership
                # is in the selection, so a merged group reads as one selected
                # object rather than only its children lighting up.
                members = scene.group_members(data[1])
                if members and all(scene.is_selected(s.id) for s in members):
                    item.setSelected(True)
            for i in range(item.childCount()):
                walk(item.child(i))
        for i in range(self.tree.topLevelItemCount()):
            walk(self.tree.topLevelItem(i))

    # --- events -------------------------------------------------------------

    def _selected_scan_ids(self):
        ids = []
        for item in self.tree.selectedItems():
            data = item.data(0, USER_ROLE)
            if not data:
                continue
            if data[0] == "scan":
                ids.append(data[1])
            elif data[0] == "group":
                # Selecting a group selects its members.
                for s in self.ctrl.scene.group_members(data[1]):
                    ids.append(s.id)
        # De-dup preserving order.
        seen, out = set(), []
        for i in ids:
            if i not in seen:
                seen.add(i)
                out.append(i)
        return out

    def _on_selection(self):
        if self._updating:
            return
        self.ctrl.on_selection_changed(self._selected_scan_ids())

    def _on_item_changed(self, item, column):
        if self._updating:
            return
        # The signal can arrive after the underlying C++ item was destroyed
        # (e.g. an in-progress edit commits just as a refresh clears the tree).
        # Touching such a wrapper raises RuntimeError, so bail out quietly.
        try:
            data = item.data(0, USER_ROLE)
        except RuntimeError:
            return
        if not data:
            return
        if data[0] == "group":
            if column == 0:
                gid = data[1]
                want = item.checkState(0) == CHECKED
                # Only act on a definite user check/uncheck. A group whose
                # members are mixed lands on PartiallyChecked; leave that alone.
                if want != (self.ctrl.scene.group_visible(gid) is True):
                    self.ctrl.on_group_visibility_toggled(gid, want)
                # Name text (strip the 🧩 prefix we added).
                text = item.text(0)
                name = text.split(" ", 1)[1] if " " in text else text
                if name and name != self.ctrl.scene.group_name(gid):
                    self.ctrl.on_group_rename(gid, name)
            return
        if data[0] != "scan":
            return
        scan_id = data[1]
        if column == 0:
            scan = self.ctrl.scene.get(scan_id)
            if scan is None:
                return
            # Visibility check state.
            want = item.checkState(0) == CHECKED
            if want != scan.visible:
                self.ctrl.on_visibility_toggled(scan_id, want)
            # Name text (strip the source icon prefix we added).
            text = item.text(0)
            name = text.split(" ", 1)[1] if " " in text else text
            if name and name != scan.name:
                self.ctrl.on_rename(scan_id, name)

    def _context_menu(self, pos):
        item = self.tree.itemAt(pos)
        menu = QtWidgets.QMenu(self)
        sel = self.ctrl.scene.selected()
        data = item.data(0, USER_ROLE) if item is not None else None
        if data and data[0] == "group":
            gid = data[1]
            vis = self.ctrl.scene.group_visible(gid)
            menu.addAction(
                "Hide group" if vis else "Show group",
                lambda: self.ctrl.on_group_visibility_toggled(gid, not vis))
            menu.addAction(
                "Rename group", lambda: self.tree.editItem(item, 0))
            menu.addSeparator()
        elif item is not None:
            menu.addAction("Rename", lambda: self.tree.editItem(item, 0))
            menu.addAction("Duplicate", self.ctrl.duplicate_selected)
            menu.addSeparator()
        if len(sel) >= 2:
            menu.addAction("Merge (SLAM)…", self.ctrl.merge_selected)
        if any(s.group for s in sel):
            menu.addAction("Un-merge", self.ctrl.unmerge_selected)
        if sel:
            menu.addSeparator()
            menu.addAction("Save selection as .ply…", self.ctrl.save_ply)
            # .bin export re-serialises the active scan's raw capture, so it is
            # only offered when that scan actually has one behind it.
            active = self.ctrl.scene.active()
            if active is not None and active.raw is not None \
                    and len(active.raw):
                menu.addAction(f"Save '{active.name}' as .bin…",
                               self.ctrl.save_selected_bin)
        menu.addSeparator()
        menu.addAction("Show only these", self.ctrl.isolate_selected)
        menu.addAction("Show all", self.ctrl.show_all)
        menu.addSeparator()
        if sel:
            menu.addAction("Delete", self.ctrl.delete_selected)
        menu.addAction("Clear scene", self.ctrl.clear_scene)
        menu.exec(self.tree.viewport().mapToGlobal(pos))

    def _update_legend(self):
        mode = self.ctrl.color_mode()
        if mode == COLOR_OBJECT:
            txt = ("Colour: <b>by object</b> — each scan a distinct colour. "
                   "Selected scans are brightened; the live scan is orange.")
        elif mode == COLOR_GROUP:
            loose = "rgb(%d,%d,%d)" % tuple(int(c * 255) for c in LOOSE_COLOR)
            txt = (f"Colour: <b>by group</b> — merged groups share a colour; "
                   f"<span style='color:{loose}'>loose scans are grey</span>. "
                   "Live scan is orange.")
        elif mode == COLOR_HEIGHT:
            txt = ("Colour: <b>by height</b> (blue low → red high). Selected "
                   "scans brightened; live scan orange.")
        else:
            txt = ("Colour: <b>by distance</b> (blue near → red far). Selected "
                   "scans brightened; live scan orange.")
        self.legend.setText(txt)
