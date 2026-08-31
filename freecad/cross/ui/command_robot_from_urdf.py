import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

from .. import wb_globals
from ..freecad_utils import warn
from ..gui_utils import tr
from ..robot_from_urdf import robot_from_urdf
from ..ros.utils import is_ros_found
try:
    from ..urdf_loader import UrdfLoader
    imports_ok = True
except ImportError as e:
    # TODO: Warn the user more nicely.
    warn(str(e), gui=False)
    imports_ok = False


class _UrdfImportCommand:
    def GetResources(self):
        return {
            'Pixmap': 'robot_from_urdf.svg',
            'MenuText': tr('Import a robot description'),
            'ToolTip': tr(
                'Import a robot from a URDF/xacro file or from the'
                ' /robot_description topic',
            ),
        }

    def Activated(self):
        # Import late to avoid slowing down workbench start-up.
        from .import_method_dialog import ImportMethodDialog

        method = ImportMethodDialog(
            topic_available=(wb_globals.g_ros_node is not None),
        ).exec()
        if method == ImportMethodDialog.TOPIC:
            self._import_from_topic()
        elif method == ImportMethodDialog.FILE:
            self._import_from_file()

    def IsActive(self):
        return is_ros_found() and imports_ok

    def _import_from_file(self) -> None:
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
            urdf_robot = UrdfLoader.load_from_file(filename)
            doc.openTransaction(tr('Robot from URDF'))
            robot_from_urdf(doc, urdf_robot, filename)
            doc.commitTransaction()
            doc.recompute()
            fcgui.SendMsgToActiveView('ViewFit')

    def _import_from_topic(self) -> None:
        # Import late to avoid slowing down workbench start-up.
        from .robot_description_topic_dialog import RobotDescriptionTopicDialog

        topic = RobotDescriptionTopicDialog().exec()
        if not topic:
            return
        fcgui.addModule('freecad.cross.robot_from_topic')
        fcgui.doCommand(
            'freecad.cross.robot_from_topic.robot_from_description_topic('
            f'{topic!r})',
        )


fcgui.addCommand('UrdfImport', _UrdfImportCommand())
