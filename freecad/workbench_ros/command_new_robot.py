import os

import FreeCAD as fc

import FreeCADGui as fcgui

from PySide import QtCore  # FreeCAD's PySide!

from .utils import ICONPATH


class _NewRobotCommand:
    """The command definition to create a new Robot object."""
    def GetResources(self):
        return {'Pixmap': str(ICONPATH.joinpath('ros_9dotslogo_color.svg')),
                'MenuText': QtCore.QT_TRANSLATE_NOOP("workbench_ros", 'New Robot'),
                'Accel': 'N, R',
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Create a Robot container.')}

    def IsActive(self):
        return (fc.activeDocument() is not None)


    def Activated(self):
        fc.activeDocument().openTransaction('Create Robot')
        fcgui.doCommand('')
        fcgui.addModule("freecad.workbench_ros")
        fcgui.doCommand("robot = freecad.workbench_ros.makeRobot('Robot')")


fcgui.addCommand('NewRobot', _NewRobotCommand())
