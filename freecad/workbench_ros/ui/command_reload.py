# Developer tool to reload the workbench.

from importlib import import_module, reload

import FreeCADGui as fcgui


class _ReloadCommand:
    """The command definition to create a new Robot object."""

    def GetResources(self):
        return {'Pixmap': 'bulb',
                'MenuText': 'Reload the ROS workbench',
                'Accel': 'W, R',
                'ToolTip': 'Reload the ROS workbench'}

    def IsActive(self):
        return True

    def Activated(self):
        def _reload_module(module_name: str) -> None:
            try:
                module = import_module(module_name)
            except ImportError:
                return
            reload(module)
        _reload_module('freecad.workbench_ros.assembly4_utils')
        _reload_module('freecad.workbench_ros.assembly_from_urdf')
        _reload_module('freecad.workbench_ros.coin_utils')
        _reload_module('freecad.workbench_ros.deep_copy')
        _reload_module('freecad.workbench_ros.freecad_utils')
        _reload_module('freecad.workbench_ros.freecadgui_utils')
        _reload_module('freecad.workbench_ros.gui_utils')
        _reload_module('freecad.workbench_ros.import_dae')
        _reload_module('freecad.workbench_ros.joint')
        _reload_module('freecad.workbench_ros.link')
        _reload_module('freecad.workbench_ros.mesh_utils')
        _reload_module('freecad.workbench_ros.placement_utils')
        _reload_module('freecad.workbench_ros.robot')
        _reload_module('freecad.workbench_ros.robot_from_urdf')
        _reload_module('freecad.workbench_ros.ros_utils')
        _reload_module('freecad.workbench_ros.ui.command_assembly_from_urdf')
        _reload_module('freecad.workbench_ros.ui.command_box_from_bounding_box')
        _reload_module('freecad.workbench_ros.ui.command_new_joint')
        _reload_module('freecad.workbench_ros.ui.command_new_link')
        _reload_module('freecad.workbench_ros.ui.command_new_robot')
        _reload_module('freecad.workbench_ros.ui.command_new_workcell')
        _reload_module('freecad.workbench_ros.ui.command_new_xacro_object')
        _reload_module('freecad.workbench_ros.ui.command_reload')
        _reload_module('freecad.workbench_ros.ui.command_robot_from_urdf')
        _reload_module('freecad.workbench_ros.ui.command_set_link_mounted_placement')
        _reload_module('freecad.workbench_ros.ui.command_sphere_from_bounding_box')
        _reload_module('freecad.workbench_ros.ui.command_urdf_export')
        _reload_module('freecad.workbench_ros.ui.command_wb_settings')
        _reload_module('freecad.workbench_ros.urdf_loader')
        _reload_module('freecad.workbench_ros.urdf_parser_utils')
        _reload_module('freecad.workbench_ros.urdf_utils')
        _reload_module('freecad.workbench_ros.utils')
        _reload_module('freecad.workbench_ros.version')
        _reload_module('freecad.workbench_ros.wb_utils')
        _reload_module('freecad.workbench_ros.workcell')
        _reload_module('freecad.workbench_ros.xacro_loader')
        _reload_module('freecad.workbench_ros.xacro_object')


fcgui.addCommand('Reload', _ReloadCommand())
