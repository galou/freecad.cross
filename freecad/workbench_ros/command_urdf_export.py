import copy
from typing import Optional
import xml.etree.ElementTree as et
from xml.dom import minidom

import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!
from PySide import QtCore  # FreeCAD's PySide!

from .export_urdf import urdf_collision_from_box
from .export_urdf import urdf_collision_from_cylinder
from .export_urdf import urdf_collision_from_object
from .export_urdf import urdf_collision_from_sphere
from .utils import is_box
from .utils import is_cylinder
from .utils import is_robot
from .utils import is_sphere
from .utils import valid_filename


def _supported_object_selected():
    for obj in fcgui.Selection.getSelection():
        if hasattr(obj, 'Placement'):
            return True
        if is_robot(obj):
            return True
    return False


class _UrdfExportCommand:
    def GetResources(self):
        return {'Pixmap': 'urdf_export',
                'MenuText': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Export to URDF'),  # TODO: translatable
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Exported selected elements to URDF'),
                }

    def Activated(self):
        def set_package_name() -> None:
            nonlocal txt
            txt = original_txt.replace('$package_name$', package_name_lineedit.text())
            txt_view.setPlainText(txt)

        txt = ''
        has_mesh = False
        for obj in fcgui.Selection.getSelection():
            if not hasattr(obj, 'TypeId'):
                # For now, only objects with 'TypeId' are supported.
                continue
            xml: Optional[et.ElementTree] = None
            if is_box(obj):
                xml = urdf_collision_from_box(obj)
            elif is_sphere(obj):
                xml = urdf_collision_from_sphere(obj)
            elif is_cylinder(obj):
                xml = urdf_collision_from_cylinder(obj)
            elif is_robot(obj):
                if hasattr(obj, 'Proxy'):
                    xml = obj.Proxy.export_urdf()
            elif hasattr(obj, 'Placement'):
                has_mesh = True
                mesh_name = valid_filename(obj.Label) if hasattr(obj, 'Label') else 'mesh.dae'
                package_name = '$package_name$'  # Will be replaced later by the GUI.
                xml = urdf_collision_from_object(
                    obj,
                    mesh_name=valid_filename(obj.Label) + '.dae',
                    package_name=package_name,
                )
            if xml:
                txt += et.tostring(xml).decode('utf-8')
        if txt:
            txt = minidom.parseString(txt).toprettyxml(indent='  ', encoding='utf-8').decode('utf-8')
            original_txt = copy.copy(txt)
            main_win = fcgui.getMainWindow()
            dialog = QtGui.QDialog(main_win, QtCore.Qt.Tool)
            layout = QtGui.QVBoxLayout(dialog)
            if has_mesh:
                package_name_lineedit = QtGui.QLineEdit('$package_name$')
                package_name_lineedit.editingFinished.connect(lambda: set_package_name())
                layout.addWidget(package_name_lineedit)
            txt_view = QtGui.QPlainTextEdit(txt)
            txt_view.setReadOnly(True)
            txt_view.setMinimumWidth(main_win.width() // 2)
            txt_view.setMinimumHeight(main_win.height() // 2)
            txt_view.setLineWrapMode(QtGui.QPlainTextEdit.NoWrap)
            layout.addWidget(txt_view)
            dialog.exec_()

    def IsActive(self):
        return _supported_object_selected()


fcgui.addCommand('UrdfExport', _UrdfExportCommand())
