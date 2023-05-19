from __future__ import annotations

from copy import copy
from pathlib import Path
from typing import Optional
import os
import sys

import FreeCAD as fc

from PySide import QtCore  # FreeCAD's PySide!
from PySide import QtGui  # FreeCAD's PySide!

from .utils import get_parent_by_pattern


def warn(text: str, gui: bool = False) -> None:
    """Warn the user."""
    # This is a copy of .utils.warn but the utils module should not be imported
    # without GUI.
    fc.Console.PrintWarning(text + '\n')
    if gui and hasattr(fc, 'GuiUp') and fc.GuiUp:
        diag = QtGui.QMessageBox(QtGui.QMessageBox.Warning,
                                 'CROSS - FreeCAD ROS Workbench', text)
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def has_ros_distro() -> bool:
    """Return True if environment variable ROS_DISTRO is set."""
    return (('ROS_DISTRO' in os.environ)
            and (Path(f'/opt/ros/{os.environ["ROS_DISTRO"]}').exists()))


def is_ros_found() -> bool:
    return get_ros_distro_from_env() != ''


def add_ros_python_library(ros_distro: str = '') -> bool:
    """Add /opt/ros/$ROS_DISTRO/lib/python?.?/site-packages to sys.path."""
    if not ros_distro:
        ros_distro = get_ros_distro_from_env()
    if not ros_distro:
        warn('The environment variable `ROS_DISTRO` is not set and no ROS'
             ' installation was found in /opt/ros'
             ', some functionalities will be missing')
        return False
    else:
        if not has_ros_distro():
            warn('The environment variable `ROS_DISTRO` is not set but a ROS'
                 f' installation was found in /opt/ros/{ros_distro}'
                 ', using it')

    # Add the paths in PYTHONPATH to sys.path.
    # Unfortunately, on some systems and with some versions of FreeCAD, the
    # environment variable PYTHONPATH is not taken into account and is reset.
    if 'PYTHONPATH' in os.environ:
        for path in os.environ['PYTHONPATH'].split(':'):
            if path not in sys.path:
                sys.path.append(path)
    # Python version.
    major = sys.version_info.major
    minor = sys.version_info.minor
    python_ver = f'python{major}.{minor}'
    # Add directories from ROS_WORKSPACE before system ones.
    ros_workspace = get_ros_workspace_from_env()
    # Works only for workspace with colcon's merge install strategy.
    _add_python_path(f'{ros_workspace}/install/lib/{python_ver}/site-packages')
    # On some systems (e.g. FreeCAD 0.21 on Ubuntu 20), $PYTHONPATH is not
    # taken into account in FreeCAD, add them manually.
    base = f'/opt/ros/{ros_distro}'
    for path in [
        Path(f'{base}/lib/{python_ver}/site-packages'),
        # Humble and later.
        Path(f'{base}/local/lib/{python_ver}/dist-packages'),
        ]:
        _add_python_path(path)
    return True


def get_ros_distro_from_env() -> str:
    """Return or guess the ROS distribution.

    Return the environment variable `ROS_DISTRO` if defined or guess from
    /opt/ros.

    When guessing, the most recent (and known) distro is returned, "rolling"
    having a higher priority.

    """
    if os.environ.get('ROS_DISTRO'):
        return os.environ.get('ROS_DISTRO')
    candidates = ['rolling', 'humble', 'galactic', 'foxy']
    for c in candidates:
        if Path(f'/opt/ros/{c}').exists():
            return c


def get_ros_workspace_from_env() -> Path:
    """Return the content of environment variable ROS_WORKSPACE.

    If not defined, try to guess from environment variable COLCON_PREFIX_PATH.

    """
    ws = os.environ.get('ROS_WORKSPACE', '')
    if ws:
        return Path(ws)

    # Guess from COLCON_PREFIX_PATH that looks like /home/user/ros_ws/install.
    colcon_prefix_path = os.environ.get('COLCON_PREFIX_PATH', '')
    if not colcon_prefix_path.endswith('/install'):
        return Path()
    return Path(colcon_prefix_path[:-len('/install')])


def get_ros_workspace_from_file(file_path: [Path | str]) -> Path:
    """Return the workspace containing the given file or directory.

    Return Path() if no workspace was found.

    """
    path, _ = get_parent_by_pattern(file_path, 'install/setup.bash')
    return path


def is_in_ros_workspace(path: [Path | str]) -> bool:
    """Return true if the given path starts with $ROS_WORKSPACE/src."""
    # Import here to avoid circular import.
    from .wb_globals import g_ros_workspace

    src = str(g_ros_workspace / 'src')
    return str(path).startswith(src)


def without_ros_workspace(path: [Path | str]) -> str:
    """Return the path relative to $ROS_WORKSPACE/src.

    Return the path as-is if it doesn't start with $ROS_WORKSPACE/src.

    """
    # Import here to avoid circular import.
    from .wb_globals import g_ros_workspace

    src = str(g_ros_workspace / 'src')
    if str(path).startswith(src):
        len_src_with_sep = len(src) + len(os.path.sep)
        return path[len_src_with_sep:]
    return copy(path)


def get_package_and_file(file_path: [Path | str]) -> tuple[str, str]:
    """Return the package name and relative file path.

    If the file path is relative, return an empty package and `file_path`.

    """
    pkg_path, relative_file_path = get_parent_by_pattern(file_path, 'package.xml')
    if not pkg_path.name:
        # No package found.
        return '', str(file_path)
    return pkg_path.name, relative_file_path


def pkg_and_file_from_ros_path(
        path: str,
        ) -> tuple[Optional[str], Optional[str]]:
    """Return the tuple (package_name, relative_file_path).

    Return (None, None) if the guessed package does not exist.

    The input path must have the following format
    `package://<package_name>/<relative_file_path>`.

    """
    # Import here to be able to import the module without ROS.
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory

    if not path or not isinstance(path, str):
        return None, None
    if not path.startswith('package://'):
        return None, None
    try:
        pkg, _, rel_path = path[len('package://'):].partition('/')
    except ValueError:
        return None, None
    try:
        get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None, None
    return pkg, rel_path


def path_from_ros_path(
        path: str,
        relative_to: Optional[Path | str] = None,
        ) -> Optional[Path]:
    """Return the absolute path to a file given in ROS format.

    Return the absolute path to a file given in ROS format.
    Supported formats are:
    - `package://<package_name>/<relative_file_path>` and
    - `file://<absolute_file_path>`.
    - `file://<relative_file_path>` if `relative_to` is given.

    With the `file://` format, if the input path is relative and `relative_to`
    is not given, the output path will also be relative. You've been warned.

    """
    # Import here to be able to import the module without ROS.
    from ament_index_python.packages import get_package_share_directory

    if not path:
        return
    if (not (path.startswith('package://')
             or path.startswith('file://'))):
        return
    if path.startswith('package://'):
        pkg, rel_path = pkg_and_file_from_ros_path(path)
        if not pkg:
            return
        pkg_path = get_package_share_directory(pkg)
        return Path(pkg_path) / rel_path
    elif path.startswith('file://'):
        if relative_to is not None:
            return Path(relative_to) / path[len('file://'):]
        return Path(path[len('file://'):])


def split_package_path(package_path: [Path | str]) -> tuple[Path, str]:
    """Return the package parent and the package name.

    For example, if `package_path` is `/home/user/ros_ws/src/my_pkg`,
    return (Path('/home/user/ros_ws/src'), 'my_pkg').

    Parameters
    ----------
    - package_path: path to the directory containing `package.xml`.

    """
    package_path = Path(package_path)
    if not package_path.is_dir():
        warn('"package_path" must be a directory', True)
    package_path = package_path.resolve()
    parent = package_path.parent
    package_name = package_path.name
    return parent, package_name


def _add_python_path(path: [Path | str]) -> None:
    """Add the path to sys.path if existing."""
    path = Path(path)
    if path.exists() and (str(path) not in sys.path):
        sys.path.append(str(path))
