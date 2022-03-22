import os
from pathlib import Path
import string
from typing import List

import FreeCAD as fc

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
    return ''.join(c  if c in valids else '_' for c in text)


def xml_comment(comment: str) -> str:
    """Returns the string without '--'."""
    return f'{comment.replace("--", "⸗⸗")}'


def label_or(obj: fc.DocumentObject, alternative: str = 'unknown label') -> str:
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


def is_robot_selected() -> bool:
    """Return True if the first selected object is a Ros::Object."""
    if not fc.GuiUp:
        return False
    if fc.activeDocument() is None:
        return False
    sel = fcgui.Selection.getSelection()
    if not sel:
        return False
    candidate = sel[0]
    return hasattr(candidate, 'Type') and (candidate.Type == 'Ros::Robot')


def get_links(l: List[fc.DocumentObject]) -> List[fc.DocumentObject]:
    """Return only the objects that are Ros::Link instances."""
    return [o for o in l if hasattr(o, 'Type') and o.Type == 'Ros::Link']

def get_path(obj: fc.DocumentObject) -> str:
    """Return the path to an object in the form parent0.parent1.....object.

    The object must belong to an "assembly", i.e. all parents of objects must
    be 'App::Part' or 'App::Link' to a 'App::Part'.

    """

