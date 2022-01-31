import xml.etree.ElementTree as et

import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!
from PySide import QtCore  # FreeCAD's PySide!

from .export_urdf import urdf_collision_xml_from_box
from .export_urdf import urdf_collision_xml_from_sphere
from .export_urdf import urdf_collision_xml_from_cylinder


def _supported_object_selected():
    for obj in fcgui.Selection.getSelection():
        if (hasattr(obj, 'TypeId')
                and (obj.TypeId in ['Part::Box', 'Part::Sphere', 'Part::Cylinder'])):
            return True
    return False


def xml_comment(comment: str) -> str:
    return f'<!-- {comment.replace("--", "⸗⸗")} -->'


class UrdfExportCommand:
    def GetResources(self):
        return {'Pixmap': 'urdf_export',
                'MenuText': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Export to URDF'),  # TODO: translatable
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Exported selected elements to URDF'),
                }

    def Activated(self):
        txt = ''
        for obj in fcgui.Selection.getSelection():
            if not hasattr(obj, 'TypeId'):
                # For now, only objects with 'TypeId' are supported.
                continue
            xml: Optional[et.ElementTree] = None
            if obj.TypeId == 'Part::Box':
                xml = urdf_collision_xml_from_box(obj)
            elif obj.TypeId == 'Part::Sphere':
                xml = urdf_collision_xml_from_sphere(obj)
            elif obj.TypeId == 'Part::Cylinder':
                xml = urdf_collision_xml_from_cylinder(obj)
            if xml:
                txt += xml_comment(obj.Label)
                txt += et.tostring(xml).decode('utf-8').replace('<', '\n<') + '\n\n'
        if txt:
            main_win = fcgui.getMainWindow()
            dialog = QtGui.QDialog(main_win, QtCore.Qt.Tool)
            layout = QtGui.QVBoxLayout(dialog)
            txt_view = QtGui.QPlainTextEdit(txt)
            txt_view.setReadOnly(True)
            txt_view.setMinimumWidth(main_win.width() // 2)
            txt_view.setMinimumHeight(main_win.height() // 2)
            txt_view.setLineWrapMode(QtGui.QPlainTextEdit.NoWrap)
            layout.addWidget(txt_view)
            dialog.exec_()

    def IsActive(self):
        return _supported_object_selected()


fcgui.addCommand('UrdfExport', UrdfExportCommand())
