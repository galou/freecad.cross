from __future__ import annotations

import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import is_robot_selected
from .move_cartesian_dialog import MoveCartesianDialog


class _MoveCartesianCommand:
    """The command definition to move a robot end-effector cartesianly."""

    def GetResources(self):
        return {
            'Pixmap': 'pose.svg',
            'MenuText': tr('Move Cartesian'),
            'Accel': 'M, C',
            'ToolTip': tr('Move a robot end-effector by Cartesian steps.'),
        }

    def IsActive(self):
        return is_robot_selected()

    def Activated(self):
        doc = fc.activeDocument()
        objs = fcgui.Selection.getSelection()
        if not doc or not objs:
            return
        robot = objs[0]
        diag = MoveCartesianDialog(robot, fcgui.getMainWindow())
        diag.exec_()
        diag.close()
        doc.recompute()


fcgui.addCommand('MoveCartesian', _MoveCartesianCommand())
