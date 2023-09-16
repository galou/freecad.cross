"""Global workbench configuration."""

from pathlib import Path

from .ros_utils import get_ros_workspace_from_env
from .ros_utils import get_ros_distro_from_env

# Constants.
PREFS_CATEGORY = 'CROSS'  # Category in the preferences dialog.
PREF_VHACD_PATH = 'vhacd_path'  # Path to the V-HACD executable.

# Session-wide globals.
g_ros_distro = get_ros_distro_from_env()

# Can be changed in the GUI.
g_ros_workspace: Path = get_ros_workspace_from_env()
