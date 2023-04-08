from __future__ import annotations

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


def get_package_and_file(file_path: [Path | str]) -> tuple[str, str]:
    """Return the package name and relative file path.

    If the file path is relative, return an empty package and `file_path`.

    """
    file_path = Path(file_path).expanduser()
    if not file_path.is_absolute():
        return '', str(file_path)
    relative_file_path = ''
    while True:
        candidate_package_xml = file_path / 'package.xml'
        if candidate_package_xml.exists() and candidate_package_xml.is_file():
            # TODO: the package name is actually given in 'package.xml'
            # and may differ from the directory name containing this file.
            return file_path.name, relative_file_path
        relative_file_path = (f'{file_path.name}/{relative_file_path}'
                              if relative_file_path else file_path.name)
        file_path = file_path.parent
        if file_path == file_path.root:
            # We are at the root.
            return '', relative_file_path


def split_package_path(package_path: [Path | str]) -> tuple[Path, Path]:
    """Return the package parent and the package name."""
    package_path = Path(package_path)
    if not package_path.is_dir():
        warn('"package_path" must be a directory', True)
    path = package_path.resolve()
    parent = path.parent
    package_name = path.stem
    return parent, package_name
