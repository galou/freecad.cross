"""Entry point of the ROS Workbench."""
import FreeCAD as fc

from .version import __version__
from .ros_utils import add_ros_python_library

add_ros_python_library()

# fc.addImportType('URDF files (*.urdf)', 'import_urdf')
