"""Dockable panels for the scanner UI.

Each panel is a plain QWidget that holds a reference to the controller (the main
window) and calls its methods for anything that changes state; the controller
calls each panel's refresh() after a change so the panels never read each other.
This keeps the scene model the single source of truth.
"""

from .scene_panel import ScenePanel
from .properties_panel import PropertiesPanel
from .view_panel import ViewPanel
from .newscan_panel import NewScanPanel
from .slam_panel import SlamPanel
from .scene_settings_panel import SceneSettingsPanel

__all__ = ["ScenePanel", "PropertiesPanel", "ViewPanel", "NewScanPanel",
           "SlamPanel", "SceneSettingsPanel"]
