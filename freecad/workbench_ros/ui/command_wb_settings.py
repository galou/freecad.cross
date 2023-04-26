from __future__ import annotations

from pathlib import Path

import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

from ..freecad_utils import warn
from ..gui_utils import tr
from ..wb_utils import UI_PATH
import freecad.workbench_ros


def _warn_if_not_workspace(path: str, gui: bool = True):
    p = Path(path)
    if not (p / 'install/setup.bash').exists():
        warn(f'{path} does not appear to be a valid ROS workspace', gui)


class _WbSettingsCommand:
    """The command definition to set up the workbench."""

    def GetResources(self):
        return {'Pixmap': 'wb_settings.svg',
                'Accel': 'W, S',
                'ToolTip': tr('Workbench settings')}

    def IsActive(self):
        return True

    def Activated(self):
        self.form = fcgui.PySideUic.loadUi(str(UI_PATH / 'wb_settings.ui'))
        self.form.lineedit_workspace.setText(freecad.workbench_ros.g_ros_workspace)
        self.form.button_browse.clicked.connect(self.on_button_browse)
        self.form.button_box.accepted.connect(self.on_ok)
        self.form.button_box.rejected.connect(self.on_cancel)
        self.form.show()

    def on_button_browse(self):
        path = QtGui.QFileDialog.getExistingDirectory(fcgui.getMainWindow(),
                                                      'Select the root of your workspace',
                                                      freecad.workbench_ros.g_ros_workspace)
        if path:
            _warn_if_not_workspace(path, True)
            self.form.lineedit_workspace.setText(path)
            freecad.workbench_ros.g_ros_workspace = path

    def on_ok(self):
        path = self.form.lineedit_workspace.text()
        _warn_if_not_workspace(path, True)
        freecad.workbench_ros.g_ros_workspace = path

    def on_cancel(self):
        pass


fcgui.addCommand('WbSettings', _WbSettingsCommand())
