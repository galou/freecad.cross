import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import is_robot_selected
from ..wb_utils import is_joint_selected

class _NewLinkCommand:
    """The command definition to create a new Link object."""

    def GetResources(self):
        return {'Pixmap': 'link.svg',
                'MenuText': tr('New Link'),
                'Accel': 'N, L',
                'ToolTip': tr('Create a Link container.')}

    def IsActive(self):
        return is_robot_selected() or is_joint_selected()

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Create Link'))
        fcgui.doCommand('')
        fcgui.addModule('freecad.workbench_ros.link')
        fcgui.doCommand("_link = freecad.workbench_ros.link.make_link('Link')")
        fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_link.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewLink', _NewLinkCommand())
