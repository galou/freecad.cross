"""Entry point of the ROS Workbench."""
import FreeCAD as fc

from .version import __version__

from .robot import makeRobot

__all__ = [
        'makeRobot',
        ]

# fc.addImportType('URDF files (*.urdf)', 'import_urdf')
