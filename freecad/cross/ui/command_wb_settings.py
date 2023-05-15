from __future__ import annotations

from pathlib import Path

import FreeCADGui as fcgui

from ..freecad_utils import warn
from ..gui_utils import tr
from ..wb_gui_utils import get_ros_workspace
from .. import wb_globals


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
        path = get_ros_workspace(wb_globals.g_ros_workspace)
        wb_globals.g_ros_workspace = path


fcgui.addCommand('WbSettings', _WbSettingsCommand())
