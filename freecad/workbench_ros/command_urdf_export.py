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
from .placement_utils import get_global_placement
from .utils import get_subobjects_by_full_name
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
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('workbench_ros', 'Export selected elements to URDF'),
                }

    def Activated(self):
        def set_package_name() -> None:
            nonlocal txt
            txt = original_txt.replace('$package_name$',
                                       package_name_lineedit.text())
            txt_view.setPlainText(txt)

        selection = fcgui.Selection.getSelectionEx('', 0)
        if not selection:
            fc.Console.PrintWarning('Nothing selected, nothing to do\n')
            return
        txt = ''
        has_mesh = False
        exported_objects: list[fc.DocumentObject] = []
        for selection_object in selection:
            root_obj = selection_object.Object
            sub_fullpaths = selection_object.SubElementNames
            if not sub_fullpaths:
                # An object is selected, not a face, edge, vertex.
                obj = root_obj
                placement = get_global_placement(root_obj, '')
            for sub_fullpath in sub_fullpaths:
                obj = get_subobjects_by_full_name(root_obj,
                                                  sub_fullpath)[-1]
                # One or more subelements are selected.
                placement = get_global_placement(root_obj, sub_fullpath)
                if not hasattr(obj, 'TypeId'):
                    # For now, only objects with 'TypeId' are supported.
                    continue
                if obj in exported_objects:
                    # Object already exported.
                    continue
                exported_objects.append(obj)
                xml: Optional[et.ElementTree] = None
                if is_box(obj):
                    xml = urdf_collision_from_box(obj, placement,
                                                  ignore_obj_placement=True)
                elif is_sphere(obj):
                    xml = urdf_collision_from_sphere(obj, placement,
                                                     ignore_obj_placement=True)
                elif is_cylinder(obj):
                    xml = urdf_collision_from_cylinder(
                            obj, placement, ignore_obj_placement=True)
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
                        placement=placement,
                    )
                if xml:
                    txt += f'  <!-- {obj.Label} -->\n'
                    txt += et.tostring(xml).decode('utf-8')
        if txt:
            if 'dummy>' in txt:
                fc.Console.PrintError('Object labels cannot contain '
                                      '"<dummy>" or "</dummy>"')
            txt = f'<dummy>{txt}</dummy>'  # xml cannot have multiple root tags
            txt = (minidom.parseString(txt)
                   .toprettyxml(indent='  ', encoding='utf-8')
                   .decode('utf-8'))
            # Maybe see
            # http://www.ronrothman.com/public/leftbraned/xml-dom-minidom-toprettyxml-and-silly-whitespace/#best-solution
            # if whitespace problems.
            txt = txt.replace('<dummy>', '').replace('</dummy>', '')
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
