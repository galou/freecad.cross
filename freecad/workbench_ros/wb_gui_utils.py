"""GUI elements for this workbench."""

from __future__ import annotations

from pathlib import Path

import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

from .freecad_utils import warn
from .wb_utils import UI_PATH


def get_ros_workspace(old_ros_workspace: [Path | str] = '') -> Path:
    return WorkspaceGetter().get(old_ros_workspace)


class WorkspaceGetter:
    def get(self,
            old_ros_workspace: [Path | str] = Path(),
            ) -> Path:
        self.old_ros_workspace = Path(old_ros_workspace)
        self.ros_workspace = Path(old_ros_workspace)
        self.form = fcgui.PySideUic.loadUi(str(UI_PATH / 'wb_settings.ui'))
        self.form.lineedit_workspace.setText(str(old_ros_workspace))
        self.form.button_browse.clicked.connect(self.on_button_browse)
        self.form.button_box.accepted.connect(self.on_ok)
        self.form.button_box.rejected.connect(self.on_cancel)
        if self.form.exec_():
            return self.ros_workspace
        # Implementation note: need to close to avoid a segfault when exiting
        # FreeCAD.
        self.form.close()
        return self.old_ros_workspace

    def on_button_browse(self):
        path = QtGui.QFileDialog.getExistingDirectory(
                fcgui.getMainWindow(),
                'Select the root of your workspace',
                str(self.ros_workspace))
        if path:
            _warn_if_not_workspace(path, True)
            self.form.lineedit_workspace.setText(path)

    def on_ok(self):
        path = self.form.lineedit_workspace.text()
        _warn_if_not_workspace(path, True)
        self.ros_workspace = Path(path)

    def on_cancel(self):
        pass


def _warn_if_not_workspace(path: str, gui: bool = True):
    p = Path(path)
    if not (p / 'install/setup.bash').exists():
        warn(f'{path} does not appear to be a valid ROS workspace', gui)
