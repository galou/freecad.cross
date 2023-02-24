import copy
from typing import Optional
import xml.etree.ElementTree as et
from xml.dom import minidom

import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!
from PySide import QtCore  # FreeCAD's PySide!

from .assembly_from_urdf import assembly_from_urdf
from .ros_utils import is_ros_found
from .urdf_loader import UrdfLoader


class _UrdfImportCommand:
    def GetResources(self):
        return {'Pixmap': 'urdf_import',
                'MenuText': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Import a URDF or xacro file'),  # TODO: translatable
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Import a URDF or xacro file'),
                }

    def Activated(self):
        doc = fc.activeDocument()
        dialog = QtGui.QFileDialog(fcgui.getMainWindow(),
                                   "Select URDF/xacro file to import part from" )
        # set option "DontUseNativeDialog"=True, as native Filedialog shows
        # misbehavior on Unbuntu 18.04 LTS. It works case sensitively, what is not wanted...
        dialog.setNameFilter("Supported Formats *.urdf *.xacro;;All files (*.*)")
        if dialog.exec_():
            if not doc:
                doc = fc.newDocument()
            filename = str(dialog.selectedFiles()[0])
            urdf_robot = UrdfLoader.load_from_file(filename)
            assembly_from_urdf(urdf_robot, doc)
            doc.recompute()
            fcgui.SendMsgToActiveView('ViewFit')

    def IsActive(self):
        return is_ros_found()


fcgui.addCommand('UrdfImport', _UrdfImportCommand())
