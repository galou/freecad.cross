import os
from pathlib import Path
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


def is_ros_found() -> bool:
    return (('ROS_DISTRO' in os.environ)
            and (Path(f'/opt/ros/{os.environ["ROS_DISTRO"]}').exists()))


def add_ros_python_library() -> bool:
    """Add /opt/ros/$ROS_DISTRO/lib/python?.?/site-packages to sys.path."""
    if not is_ros_found():
        warn('The environment variable `ROS_DISTRO`'
             ' is not set, some functionalities will be missing')
        return False
    if 'PYTHONPATH' in os.environ:
        for path in os.environ['PYTHONPATH'].split(':'):
            if path not in sys.path:
                sys.path.append(path)
    # On some systems (e.g. FreeCAD 0.21 on Ubuntu 20), $PYTHONPATH is not
    # taken into account in FreeCAD.
    major = sys.version_info.major
    minor = sys.version_info.minor
    base = f'/opt/ros/{os.environ["ROS_DISTRO"]}'
    for path in [
        Path(f'{base}/lib/python{major}.{minor}/site-packages'),
        # Humble and later.
        Path(f'{base}/local/lib/python{major}.{minor}/dist-packages'),
        ]:
        if path.exists() and (str(path) not in sys.path):
            sys.path.append(str(path))
    return True
