import os

import FreeCADGui as fcgui

import FreeCAD as fc

from . import ICONPATH
from . import urdf_export_command
from . import box_from_bounding_box
from . import sphere_from_bounding_box


class RosWorkbench(fcgui.Workbench):
    """Class which gets initiated at startup of the gui."""

    MenuText = 'ROS workbench'
    ToolTip = 'ROS-related workbench'
    Icon = str(ICONPATH.joinpath('ros_9dotslogo_color.svg'))

    def GetClassName(self):
        return 'Gui::PythonWorkbench'

    def Initialize(self):
        """This function is called at the first activation of the workbench.

        This is the place to import all the commands.

        """
        commands = [
            'UrdfExport',  # Defined in ./urdf_export_command.py.
            'BoxFromBoundingBox',  # Defined in ./box_from_bounding_box.py.
            'SphereFromBoundingBox',  # Defined in ./sphere_from_bounding_box.py.
            ]
        self.appendToolbar('ROS', commands)
        self.appendMenu('ROS', commands)

        fcgui.addIconPath(str(ICONPATH))
        # fcgui.addLanguagePath(joinDir("Resources/translations"))

    def Activated(self):
        """Code run when a user switches to this workbench."""
        pass

    def Deactivated(self):
        """Code run when this workbench is deactivated."""
        pass


fcgui.addWorkbench(RosWorkbench())
