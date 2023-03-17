import FreeCAD as fc

import FreeCADGui as fcgui

from PySide import QtCore  # FreeCAD's PySide!

from .utils import ICON_PATH


class _NewRobotCommand:
    """The command definition to create a new Robot object."""
    def GetResources(self):
        return {'Pixmap': 'robot',
                'MenuText': QtCore.QT_TRANSLATE_NOOP("workbench_ros", 'New Robot'),
                'Accel': 'N, R',
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Create a Robot container.')}

    def IsActive(self):
        return (fc.activeDocument() is not None)


    def Activated(self):
        fc.activeDocument().openTransaction('Create Robot')
        fcgui.doCommand('')
        fcgui.addModule('freecad.workbench_ros.robot')
        fcgui.doCommand("_robot = freecad.workbench_ros.robot.make_robot('Robot')")
        fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_robot.Name)')


fcgui.addCommand('NewRobot', _NewRobotCommand())
