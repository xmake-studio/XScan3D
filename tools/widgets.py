#!/usr/bin/env python3
"""Qt plumbing shared by every panel: enum lookups, small custom widgets, the
settings store, the dark theme and the paths scans live under.

None of this is specific to the scene model; it is the binding-portability and
look-and-feel layer the rest of the UI is built on top of. It was pulled out of
the original single-file UI unchanged, so behaviour here is exactly what it was.
"""

import json
import os

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets


# Scans and settings belong to the project, not to wherever the UI was launched
# from, so both anchor next to the source / one level up.
SETTINGS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "ui_settings.json")
SCANS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scans")


def scan_path(name, sub=None):
    """Absolute path for `name` under scans/ (or scans/<sub>/), dir created.

    The directory is made on demand rather than at import: a checkout without
    scans/ should still get one the first time something is written.
    """
    d = os.path.join(SCANS_DIR, sub) if sub else SCANS_DIR
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        return os.path.abspath(name)
    return os.path.join(d, name)


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
STRONG_FOCUS = _enum(QtCore.Qt, "FocusPolicy.StrongFocus", "StrongFocus")
LEFT_BUTTON = _enum(QtCore.Qt, "MouseButton.LeftButton", "LeftButton")
MIDDLE_BUTTON = _enum(QtCore.Qt, "MouseButton.MiddleButton", "MiddleButton")
SHIFT_MOD = _enum(QtCore.Qt, "KeyboardModifier.ShiftModifier", "ShiftModifier")
CTRL_MOD = _enum(QtCore.Qt, "KeyboardModifier.ControlModifier",
                 "ControlModifier")
ALT_MOD = _enum(QtCore.Qt, "KeyboardModifier.AltModifier", "AltModifier")
TRANSPARENT_FOR_MOUSE = _enum(
    QtCore.Qt, "WidgetAttribute.WA_TransparentForMouseEvents",
    "WA_TransparentForMouseEvents")
EV_KEY_PRESS = _enum(QtCore.QEvent, "Type.KeyPress", "KeyPress")
EV_KEY_RELEASE = _enum(QtCore.QEvent, "Type.KeyRelease", "KeyRelease")
EV_WHEEL = _enum(QtCore.QEvent, "Type.Wheel", "Wheel")
EV_NATIVE_GESTURE = _enum(QtCore.QEvent, "Type.NativeGesture", "NativeGesture")
EV_WINDOW_DEACTIVATE = _enum(QtCore.QEvent, "Type.WindowDeactivate",
                             "WindowDeactivate")
USER_ROLE = _enum(QtCore.Qt, "ItemDataRole.UserRole", "UserRole")
CHECKED = _enum(QtCore.Qt, "CheckState.Checked", "Checked")
UNCHECKED = _enum(QtCore.Qt, "CheckState.Unchecked", "Unchecked")
PARTIALLY_CHECKED = _enum(QtCore.Qt, "CheckState.PartiallyChecked",
                          "PartiallyChecked")
ITEM_IS_USER_CHECKABLE = _enum(QtCore.Qt, "ItemFlag.ItemIsUserCheckable",
                               "ItemIsUserCheckable")
ITEM_IS_EDITABLE = _enum(QtCore.Qt, "ItemFlag.ItemIsEditable", "ItemIsEditable")
EXTENDED_SELECT = _enum(QtWidgets.QAbstractItemView,
                        "SelectionMode.ExtendedSelection", "ExtendedSelection")
CUSTOM_CONTEXT_MENU = _enum(QtCore.Qt, "ContextMenuPolicy.CustomContextMenu",
                            "CustomContextMenu")
try:
    ZOOM_GESTURE = _enum(QtCore.Qt, "NativeGestureType.ZoomNativeGesture",
                         "ZoomNativeGesture")
except AttributeError:  # pragma: no cover - binding without native gestures
    ZOOM_GESTURE = None


def _key(name):
    return _enum(QtCore.Qt, f"Key.Key_{name}", f"Key_{name}")


# --- Settings store ---------------------------------------------------------

class FileSettings:
    """QSettings-shaped store backed by one JSON file.

    Same value()/setValue() surface QSettings had, so the panel code did not
    have to change, but the file is versioned instead of living in the registry
    or ~/.config where it could not be shared. Values keep their JSON types.
    """

    def __init__(self, path):
        self.path = path
        self._data = {}
        self._load()

    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError):
            return
        if isinstance(data, dict):
            self._data = {k: v for k, v in data.items() if isinstance(k, str)}

    def value(self, key, default=None):
        return self._data.get(key, default)

    def setValue(self, key, val):
        self._data[key] = val

    def allKeys(self):
        return list(self._data)

    def sync(self):
        """Write the file. Atomic, so a crash mid-save cannot truncate it."""
        tmp = self.path + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(self._data, fh, indent=2, sort_keys=True)
                fh.write("\n")
            os.replace(tmp, self.path)
        except OSError:
            pass

    def migrate_from_qsettings(self, qs):
        """One-time import of the pre-file location. No-op once the file exists."""
        if self._data:
            return False
        for key in qs.allKeys():
            val = qs.value(key)
            if isinstance(val, str):
                low = val.strip().lower()
                if low in ("true", "false"):
                    val = low == "true"
                else:
                    try:
                        val = int(val)
                    except ValueError:
                        try:
                            val = float(val)
                        except ValueError:
                            pass
            self._data[key] = val
        return bool(self._data)


# --- Scroll-safe value widgets ----------------------------------------------

class _NoWheelEdits(QtCore.QObject):
    """Stops the wheel from editing the widget it happens to be hovering.

    Qt's default is that a spin box, slider or combo under the pointer eats the
    wheel and changes its value. In a tall panel that is a trap: scrolling drags
    the pointer across editable widgets and any of them silently absorbing a
    notch retunes a parameter. Keyboard, arrows and typing still edit; the wheel
    is forwarded to the enclosing scroll area instead of being swallowed.
    """

    def eventFilter(self, obj, ev):
        if ev.type() != EV_WHEEL:
            return False
        area = obj.parent()
        while area is not None and not isinstance(
                area, QtWidgets.QAbstractScrollArea):
            area = area.parent()
        if area is not None:
            QtWidgets.QApplication.sendEvent(area.viewport(), ev)
        return True


_NO_WHEEL = None
_VALUE_WIDGETS = (QtWidgets.QAbstractSpinBox, QtWidgets.QComboBox,
                  QtWidgets.QSlider)


def disable_wheel_edits(root):
    """Apply _NoWheelEdits to every value widget under `root`.

    Done in one sweep rather than at each widget's construction, so a control
    added later cannot forget to opt in.
    """
    global _NO_WHEEL
    if _NO_WHEEL is None:
        _NO_WHEEL = _NoWheelEdits()
    for cls in _VALUE_WIDGETS:
        for w in root.findChildren(cls):
            w.installEventFilter(_NO_WHEEL)
            w.setFocusPolicy(STRONG_FOCUS)


# --- Collapsible section ----------------------------------------------------

class Collapsible(QtWidgets.QWidget):
    """A group box with its title turned into a fold/unfold header.

    Collapsing hides the group as a whole rather than its individual widgets, so
    the per-widget visible/enabled states mode handlers set survive a fold.
    """

    def __init__(self, group, parent=None):
        super().__init__(parent)
        self.group = group
        title = group.title()
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


# --- Dark theme -------------------------------------------------------------

def apply_dark_theme(app):
    """Force the dark palette on every machine.

    Qt does not follow the OS dark-mode setting and platform styles disagree
    about a palette they were not built for, so the look drifted between
    checkouts. Fusion honours a custom palette everywhere, so pin both.
    """
    app.setStyle("Fusion")

    bg = QtGui.QColor(43, 43, 43)
    base = QtGui.QColor(30, 30, 30)
    text = QtGui.QColor(220, 220, 220)
    disabled = QtGui.QColor(128, 128, 128)
    highlight = QtGui.QColor(42, 130, 218)

    def role(name):
        return _enum(QtGui.QPalette, f"ColorRole.{name}", name)

    pal = QtGui.QPalette()
    for name, c in (("Window", bg), ("WindowText", text),
                    ("Base", base), ("AlternateBase", bg),
                    ("ToolTipBase", bg), ("ToolTipText", text),
                    ("Text", text), ("Button", bg),
                    ("ButtonText", text), ("BrightText", QtGui.QColor("red")),
                    ("Link", highlight), ("Highlight", highlight),
                    ("HighlightedText", QtGui.QColor(0, 0, 0))):
        pal.setColor(role(name), c)

    greyed = _enum(QtGui.QPalette, "ColorGroup.Disabled", "Disabled")
    for name in ("WindowText", "Text", "ButtonText", "HighlightedText"):
        pal.setColor(greyed, role(name), disabled)
    app.setPalette(pal)

    pg.setConfigOption("background", base)
    pg.setConfigOption("foreground", text)
