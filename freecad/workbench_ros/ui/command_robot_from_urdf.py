
import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

from ..freecad_utils import warn
from ..gui_utils import tr
from ..robot_from_urdf import robot_from_urdf
from ..ros_utils import is_ros_found
try:
    from ..urdf_loader import UrdfLoader
except ModuleNotFoundError as e:
    # TODO: Warn the user more nicely.
    warn(str(e), gui=False)
    pass


class _UrdfImportCommand:
    def GetResources(self):
        return {'Pixmap': 'robot_from_urdf.svg',
                'MenuText': tr('Import a URDF or xacro file'),
                'ToolTip': tr('Import a URDF or xacro file'),
                }

    def Activated(self):
        doc = fc.activeDocument()
        dialog = QtGui.QFileDialog(fcgui.getMainWindow(),
                                   'Select URDF/xacro file to import part from')
        # set option "DontUseNativeDialog"=True, as native Filedialog shows
        # misbehavior on Unbuntu 18.04. It works case sensitively, what is not wanted...
        dialog.setNameFilter('Supported Formats *.urdf *.xacro;;All files (*.*)')
        if dialog.exec_():
            if not doc:
                doc = fc.newDocument()
            filename = str(dialog.selectedFiles()[0])
            urdf_robot = UrdfLoader.load_from_file(filename)
            robot_from_urdf(doc, urdf_robot)
            doc.recompute()
            fcgui.SendMsgToActiveView('ViewFit')

    def IsActive(self):
        return is_ros_found()


fcgui.addCommand('UrdfImport', _UrdfImportCommand())
