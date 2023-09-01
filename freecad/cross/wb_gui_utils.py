"""GUI elements for this workbench."""

from __future__ import annotations

import os
from pathlib import Path

import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

from .freecad_utils import warn
from .wb_utils import UI_PATH


def get_ros_workspace(old_ros_workspace: [Path | str] = '') -> Path:
    return WbSettingsGetter().get_ros_workspace(old_ros_workspace)


def _warn_if_not_workspace(path: [Path | str], gui: bool = True) -> None:
    p = Path(path)
    if not (p / 'install/setup.bash').exists():
        warn(f'{path} does not appear to be a valid ROS workspace', gui)


def _warn_if_not_vhacd_ok(path: [Path | str], gui: bool = True) -> None:
    p = Path(path)
    if not p.exists():
        warn(f'{path} does not exist', gui)
    elif not p.is_file():
        warn(f'{path} is not a file', gui)
    elif not os.access(p, os.X_OK):
        warn(f'{path} is not executable', gui)


def guess_vhacd_path() -> Path:
    """Guess and return the path to the V-HACD executable.

    Return an empty path if not found.

    """
    candidate_dirs: list[str] = os.get_exec_path()
    for dir in candidate_dirs:
        path = Path(dir) / 'v-hacd'
        if path.exists():
            return path
    return Path()


class WbSettingsGetter:
    """A class to get the settings for this workbench.

    The settings are stored in the class's attributes
    `ros_workspace` and `vhacd_path`.

    """

    def __init__(self,
                 old_ros_workspace: [Path | str] = '',
                 old_vhacd_path: [Path | str] = '',
                 ):
        self._old_ros_workspace = Path(old_ros_workspace)
        self._old_vhacd_path = Path(old_vhacd_path)
        self.ros_workspace = self._old_ros_workspace
        self.vhacd_path = self._old_vhacd_path

    def get_settings(self) -> bool:
        """Get the settings for this workbench.

        Return True if the settings' dialog was confirmed.

        """
        self.form = fcgui.PySideUic.loadUi(str(UI_PATH / 'wb_settings.ui'))

        self.form.lineedit_workspace.setText(str(self.ros_workspace))
        self.form.button_browse_workspace.clicked.connect(
                self.on_button_browse_workspace)

        self.form.lineedit_vhacd_path.setText(str(self.vhacd_path))
        self.form.button_browse_vhacd_path.clicked.connect(
                self.on_button_browse_vhacd_path)

        self.form.button_box.accepted.connect(self.on_ok)
        self.form.button_box.rejected.connect(self.on_cancel)

        if self.form.exec_():
            return True
        # Implementation note: need to close to avoid a segfault when exiting
        # FreeCAD.
        self.form.close()
        return False

    def get_ros_workspace(self,
                          old_ros_workspace: [Path | str] = Path(),
                          ) -> Path:
        self._old_ros_workspace = Path(old_ros_workspace)
        if self.get_settings():
            return self.ros_workspace
        return self._old_ros_workspace

    def get_vhacd_path(self,
                       old_vhacd_path: [Path | str] = Path(),
                       ) -> Path:
        self._old_vhacd_path = Path(old_vhacd_path)
        if self.form.exec_():
            return self.vhacd_path
        return self._old_vhacd_path

    def on_button_browse_workspace(self):
        path = QtGui.QFileDialog.getExistingDirectory(
                fcgui.getMainWindow(),
                'Select the root of your workspace',
                str(self.ros_workspace))
        if path:
            _warn_if_not_workspace(path, True)
            self.form.lineedit_workspace.setText(path)

    def on_button_browse_vhacd_path(self):
        print(f'on_button_browse_vhacd_path({self.vhacd_path})') # DEBUG
        path = QtGui.QFileDialog.getOpenFileName(
                fcgui.getMainWindow(),
                'Select the V-HACD executable',
                str(self.vhacd_path))[0]
        if path:
            _warn_if_not_vhacd_ok(path, True)
            self.form.lineedit_vhacd_path.setText(path)

    def on_ok(self):
        workspace_path = Path(self.form.lineedit_workspace.text())
        _warn_if_not_workspace(workspace_path, True)
        self.ros_workspace = workspace_path

        vhacd_path = Path(self.form.lineedit_vhacd_path.text())
        if not vhacd_path.exists():
            _warn_if_not_vhacd_ok(vhacd_path, True)
        self.vhacd_path = vhacd_path

    def on_cancel(self):
        if hasattr(self, '_old_ros_workspace'):
            self.ros_workspace = self._old_ros_workspace
        else:
            self.ros_workspace = Path()
        if hasattr(self, '_old_vhacd_path'):
            self.vhacd_path = self._old_vhacd_path
        else:
            self.vhacd_path = Path()
