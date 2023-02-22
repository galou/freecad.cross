import os
import sys

import FreeCAD as fc

from PySide import QtCore  # FreeCAD's PySide!
from PySide import QtGui  # FreeCAD's PySide!


def warn(text: str, gui: bool = False) -> None:
    """Warn the user."""
    # This is a copy of .utils.warn but the utils module should not be imported
    # without GUI.
    fc.Console.PrintWarning(text + '\n')
    if gui and hasattr(fc, 'GuiUp') and fc.GuiUp:
        diag = QtGui.QMessageBox(QtGui.QMessageBox.Warning,
                                 u'ROS Workbench', text)
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def is_ros_found():
    return 'ROS_DISTRO' in os.environ


def add_ros_python_library() -> bool:
    """Add /opt/ros/$ROS_DISTRO/lib/python?.?/site-packages to sys.path."""
    if not is_ros_found():
        warn('The environment variable `ROS_DISTRO`'
             ' is not set, some functionalities will be missing')
        return False
    major = sys.version_info.major
    minor = sys.version_info.minor
    sys.path.append(f'/opt/ros/{os.environ["ROS_DISTRO"]}/lib/python{major}.{minor}/site-packages')
    return True
