import os
from pathlib import Path
import string
from typing import Any, List, Tuple, Union
import xml.etree.ElementTree as et
from xml.dom import minidom

import FreeCAD as fc

import Mesh  # FreeCAD

import importDAE  # FreeCAD.

if fc.GuiUp:
    import FreeCADGui as fcgui

    from PySide import QtCore  # FreeCAD's PySide!
    from PySide.QtCore import QT_TRANSLATE_NOOP
    from PySide import QtGui  # FreeCAD's PySide!

    def tr(text: str) -> str:
        return QtGui.QApplication.translate('workbench_ros', text)
else:
    def tr(text: str) -> str:
        return text


# MOD_PATH = Path(os.path.join(fc.getResourceDir(), 'Mod', 'workbench_ros'))
MOD_PATH = Path(os.path.dirname(__file__)).joinpath('../..').resolve()  # For development
RESOURCES_PATH = MOD_PATH.joinpath('resources')
UI_PATH = RESOURCES_PATH.joinpath('ui')
ICON_PATH = RESOURCES_PATH.joinpath('icons')


def valid_filename(text: str) -> str:
    """Return a string that is a valid file name."""
    valids = string.ascii_letters + string.digits + '_-.'
    return ''.join(c if c in valids else '_' for c in text)


def xml_comment(comment: str) -> str:
    """Returns the string without '--'."""
    return f'{comment.replace("--", "⸗⸗")}'


def valid_urdf_name(name: str) -> str:
    if not name:
        return 'no_label'
    return name.replace(' ', '_')


def label_or(
        obj: fc.DocumentObject,
        alternative: str = 'no label') -> str:
    """Return the `Label` or the alternative."""
    return obj.Label if hasattr(obj, 'Label') else alternative


def warn(text: str, gui: bool = False) -> None:
    """Warn the user."""
    fc.Console.PrintWarning(text + '\n')
    if gui and fc.GuiUp:
        diag = QtGui.QMessageBox(QtGui.QMessageBox.Warning,
                                 u'ROS Workbench', text)
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def error(text: str, gui: bool = False) -> None:
    """Log an error to the user."""
    fc.Console.PrintError(text + '\n')
    if gui and fc.GuiUp:
        diag = QtGui.QMessageBox(QtGui.QMessageBox.Error,
                                 u'FreeCAD - ROS Workbench', text)
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def add_property(
        obj: fc.DocumentObject,
        type_: str,
        name: str,
        category: str,
        help_: str,
        ) -> fc.DocumentObject:
    """Add a dynamic property to the object and return the object."""
    if name not in obj.PropertiesList:
        return obj.addProperty(type_, name, category, tr(help_))

    # Return the object, similaryly to obj.addProperty.
    return obj


def _has_ros_type(obj: fc.DocumentObject, type_: str) -> bool:
    """Return True if the object is an object from this workbench."""
    if not isinstance(obj, fc.DocumentObject):
        return False
    return hasattr(obj, 'Type') and (obj.Type == type_)


def is_robot(obj: fc.DocumentObject) -> bool:
    """Return True if the object is a Ros::Robot."""
    return _has_ros_type(obj, 'Ros::Robot')


def is_link(obj: fc.DocumentObject) -> bool:
    """Return True if the object is a Ros::Link."""
    return _has_ros_type(obj, 'Ros::Link')


def _has_typeid(obj: fc.DocumentObject, typeid: str) -> bool:
    """Return True if the object is a object of the given type."""
    if not isinstance(obj, fc.DocumentObject):
        return False
    return hasattr(obj, 'TypeId') and (obj.TypeId == typeid)


def is_box(obj: fc.DocumentObject) -> bool:
    """Return True if the object is a 'Part::Box'."""
    return _has_typeid(obj, 'Part::Box')


def is_sphere(obj: fc.DocumentObject) -> bool:
    """Return True if the object is a 'Part::Sphere'."""
    return _has_typeid(obj, 'Part::Sphere')


def is_cylinder(obj: fc.DocumentObject) -> bool:
    """Return True if the object is a 'Part::Cylinder'."""
    return _has_typeid(obj, 'Part::Cylinder')


def is_primitive(obj: fc.DocumentObject) -> bool:
    """Return True if the object is a 'Part::{Box,Cylinder,Sphere}'."""
    return is_box(obj) or is_sphere(obj) or is_cylinder(obj)


def is_robot_selected() -> bool:
    """Return True if the first selected object is a Ros::Object."""
    if not fc.GuiUp:
        return False
    if fc.activeDocument() is None:
        return False
    sel = fcgui.Selection.getSelection()
    if not sel:
        return False
    return is_robot(sel[0])


def has_placement(obj: fc.DocumentObject) -> bool:
    """Return True if obj has a Placement."""
    return hasattr(obj, 'Placement') and isinstance(obj.Placement, fc.Placement)


def get_links(objs: List[fc.DocumentObject]) -> List[fc.DocumentObject]:
    """Return only the objects that are Ros::Link instances."""
    return [o for o in objs if is_link(o)]


def hasallattr(obj: Any, attrs: List[str]):
    """Return True if object has all attributes."""
    for attr in attrs:
        if not hasattr(obj, attr):
            return False
    return True


def get_placement(obj: fc.DocumentObject) -> fc.Placement:
    """Return the object's placement."""
    if not isinstance(obj, fc.DocumentObject):
        raise RuntimeError('Not a DocumentObject')
    if obj.TypeId == 'App::Link':
        if not obj.Parents:
            # A link to nothing.
            return obj.LinkPlacement
        parent, subname = obj.Parents[0]
        return parent.getSubObject(subname, retType=3)
    else:
        if hasattr(obj, 'getGlobalPlacement'):
            return obj.getGlobalPlacement()
        elif hasattr(obj, 'Placement'):
            return obj.Placement


def split_package_path(package_path: Union[Path, str]) -> Tuple[Path, Path]:
    """Return the package parent and the package name."""
    package_path = Path(package_path)
    if not package_path.is_dir():
        error('"OutputPath" must be a directory', True)
    path = package_path.resolve()
    parent = path.parent
    package_name = path.stem
    return parent, package_name


def save_mesh_dae(obj: fc.DocumentObject, filename: Union[Path, str]) -> None:
    """Save the mesh of a FreeCAD object into a Collada file."""
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    importDAE.export([obj], str(filename))


def save_mesh(obj: fc.DocumentObject, filename: Union[Path, str]) -> None:
    """Save the mesh of a FreeCAD object into a file.

    The type of the exported file is determined by the Mesh module.
    See the Mesh module for a list of supported formats.

    """
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    # TODO: scale to meters.
    Mesh.export([obj], str(filename))


def save_xml(xml: et.ElementTree, filename: Union[Path, str]) -> None:
    """Save the xml element into a file."""
    file_path = Path(filename)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    txt = minidom.parseString(et.tostring(xml)).toprettyxml(indent='  ')
    file_path.write_text(txt)
