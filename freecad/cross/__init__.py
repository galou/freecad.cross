"""Entry point of the ROS Workbench."""

# import FreeCAD as fc

from .ros_utils import add_ros_python_library
from .ros_utils import get_ros_distro_from_env
from .ros_utils import get_ros_workspace_from_env
from .version import __version__
from .wb_globals import g_ros_distro
from .wb_globals import g_ros_workspace

add_ros_python_library(g_ros_distro)

# fc.addImportType('URDF files (*.urdf)', 'import_urdf')
