"""Entry point of the ROS Workbench."""
import os
from pathlib import Path

import FreeCAD as fc

from .version import __version__

ICONPATH = Path(os.path.dirname(__file__)).joinpath('resources/icons')

# fc.addImportType('URDF files (*.urdf)', 'import_urdf')
