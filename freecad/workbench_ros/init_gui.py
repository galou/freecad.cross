
import FreeCADGui as fcgui

from .ui import command_assembly_from_urdf
from .ui import command_box_from_bounding_box
from .ui import command_new_joint
from .ui import command_new_link
from .ui import command_new_robot
from .ui import command_robot_from_urdf
from .ui import command_sphere_from_bounding_box
from .ui import command_urdf_export
from .utils import ICON_PATH


class RosWorkbench(fcgui.Workbench):
    """Class which gets initiated at startup of the GUI."""

    MenuText = 'ROS workbench'
    ToolTip = 'ROS-related workbench'
    Icon = str(ICON_PATH / 'ros_9dotslogo_color.svg')

    def GetClassName(self):
        return 'Gui::PythonWorkbench'

    def Initialize(self):
        """This function is called at the first activation of the workbench.

        This is the place to import all the commands.

        """
        # The order here defines the order of the icons in the GUI.
        commands = [
            'NewRobot',  # Defined in ./command_new_robot.py
            'NewLink',  # Defined in ./command_new_link.py
            'NewJoint',  # Defined in ./command_new_joint.py
            'BoxFromBoundingBox',  # Defined in ./box_from_bounding_box.py.
            'SphereFromBoundingBox',  # Defined in ./sphere_from_bounding_box.py.
            'UrdfImport',  # Defined in ./command_robot_from_urdf.py.
            'AssemblyFromUrdf',  # Defined in ./command_assembly_from_urdf.py.
            'UrdfExport',  # Defined in ./command_urdf_export.py.
            ]
        self.appendToolbar('ROS', commands)
        self.appendMenu('ROS', commands)

        fcgui.addIconPath(str(ICON_PATH))
        # fcgui.addLanguagePath(joinDir("Resources/translations"))

    def Activated(self):
        """Code run when a user switches to this workbench."""
        pass

    def Deactivated(self):
        """Code run when this workbench is deactivated."""
        pass


fcgui.addWorkbench(RosWorkbench())
