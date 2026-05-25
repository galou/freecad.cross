
import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

from ..freecad_utils import warn
from ..gui_utils import tr
from ..robot_from_urdf import robot_from_urdf
from ..ros.utils import is_ros_found
from .urdf_parser_dialog import choose_parsing_library
try:
    from ..urdf_loader import UrdfLoader
    loader_import_ok = True
except ImportError as e:
    # TODO: Warn the user more nicely.
    warn(str(e), gui=False)
    loader_import_ok = False


class _UrdfImportCommand:
    def GetResources(self):
        return {
            'Pixmap': 'robot_from_urdf.svg',
            'MenuText': tr('Import a URDF or xacro file'),
            'ToolTip': tr('Import a URDF or xacro file'),
        }

    def Activated(self):
        doc = fc.activeDocument()
        dialog = QtGui.QFileDialog(
            fcgui.getMainWindow(),
            'Select URDF/xacro file to import part from',
        )
        # set option "DontUseNativeDialog"=True, as native Filedialog shows
        # misbehavior on Ubuntu 18.04. It works case sensitively, what is not wanted...
        dialog.setNameFilter('Supported Formats *.urdf *.xacro;;All files (*.*)')
        if dialog.exec_():
            if not doc:
                doc = fc.newDocument()
            filename = str(dialog.selectedFiles()[0])
            parser = choose_parsing_library()
            if parser is None:
                return
            urdf_robot = UrdfLoader.load_from_file(filename, parser=parser)
            doc.openTransaction(tr('Robot from URDF'))
            robot_from_urdf(doc, urdf_robot)
            doc.commitTransaction()
            doc.recompute()
            fcgui.SendMsgToActiveView('ViewFit')

    def IsActive(self):
        return is_ros_found() and loader_import_ok


fcgui.addCommand('UrdfImport', _UrdfImportCommand())
