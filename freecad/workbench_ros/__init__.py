"""Entry point of the ROS Workbench."""
import FreeCAD as fc

from .version import __version__
from .ros_utils import add_ros_python_library
from .ros_utils import get_ros_workspace_from_env
from .ros_utils import get_ros_distro_from_env

# Global workbench configuration.
# Can be changed in the GUI.
g_ros_distro = get_ros_distro_from_env()
g_ros_workspace = get_ros_workspace_from_env()

add_ros_python_library(g_ros_distro)

# fc.addImportType('URDF files (*.urdf)', 'import_urdf')
