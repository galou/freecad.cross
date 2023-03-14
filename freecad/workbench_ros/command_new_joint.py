import FreeCAD as fc

import FreeCADGui as fcgui

from PySide import QtCore  # FreeCAD's PySide!

from .utils import ICON_PATH
from .utils import is_robot_selected


class _NewJointCommand:
    """The command definition to create a new Joint object."""
    def GetResources(self):
        return {'Pixmap': str(ICON_PATH.joinpath('joint.svg')),
                'MenuText': QtCore.QT_TRANSLATE_NOOP("workbench_ros", 'New Joint'),
                'Accel': 'N, L',
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Create a Joint.')}

    def IsActive(self):
        return is_robot_selected()

    def Activated(self):
        fc.activeDocument().openTransaction('Create Joint')
        fcgui.doCommand('')
        fcgui.addModule('freecad.workbench_ros.joint')
        fcgui.doCommand("_joint = freecad.workbench_ros.joint.make_joint('Joint')")
        fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_joint.Name)')


fcgui.addCommand('NewJoint', _NewJointCommand())
