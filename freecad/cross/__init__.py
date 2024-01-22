"""Entry point of the CROSS workbench."""

import FreeCAD as fc

from .ros.utils import add_ros_python_library
from .version import __version__
from .wb_globals import g_ros_distro

add_ros_python_library(g_ros_distro)

# Must be imported after the call to `add_ros_python_library`.
from .ros.utils import is_ros_found  # noqa: E402.


if is_ros_found():
    fc.addImportType('URDF files (*.urdf *.xacro)', 'import_urdf')
