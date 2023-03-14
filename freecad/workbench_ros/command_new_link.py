import FreeCAD as fc

import FreeCADGui as fcgui

from PySide import QtCore  # FreeCAD's PySide!

from .utils import ICON_PATH
from .utils import is_robot_selected


class _NewLinkCommand:
    """The command definition to create a new Link object."""
    def GetResources(self):
        return {'Pixmap': str(ICON_PATH.joinpath('link.svg')),
                'MenuText': QtCore.QT_TRANSLATE_NOOP("workbench_ros", 'New Link'),
                'Accel': 'N, L',
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Create a Link container.')}

    def IsActive(self):
        return is_robot_selected()

    def Activated(self):
        fc.activeDocument().openTransaction('Create Link')
        fcgui.doCommand('')
        fcgui.addModule('freecad.workbench_ros.link')
        fcgui.doCommand("_link = freecad.workbench_ros.link.make_link('Link')")
        fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_link.Name)')


fcgui.addCommand('NewLink', _NewLinkCommand())
