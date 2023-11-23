from __future__ import annotations

from math import degrees

import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import is_robot_selected
from .set_joints_dialog import SetJointsDialog

# Stubs and type hints.
from ..joint import Joint
from ..robot import Robot
CrossJoint = Joint
CrossRobot = Robot


class _SetJointsCommand:
    """The command definition to create a set the joint values of a robot."""

    def GetResources(self):
        return {'Pixmap': 'set_joints.svg',
                'MenuText': tr('Set Joints'),
                'Accel': 'S, J',
                'ToolTip': tr('Set the joint values of a robot.')}

    def IsActive(self):
        return is_robot_selected()

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Set Joints'))
        objs = fcgui.Selection.getSelection()
        if not objs:
            doc = fc.activeDocument()
        else:
            robot = objs[0]
        diag = SetJointsDialog(robot)
        joint_values = diag.exec_()
        diag.close()
        if joint_values:
            print(f'Activated(), {joint_values=}')
            self._set_robot_joint_values(robot, joint_values)
        doc.recompute()
        doc.commitTransaction()

    def _set_robot_joint_values(self,
                                robot: CrossRobot,
                                joint_values: dict[CrossJoint, float]) -> None:
        """Set the joint values of the robot from values in meters and radians."""
        joint_variables: dict[CrossJoint, str] = robot.Proxy.joint_variables
        for joint, value in joint_values.items():
            var = joint_variables[joint]
            if joint.Type == 'prismatic':
                setattr(robot, var, value * 1000)
            else:
                # joint.Type in ['revolute', 'continuous']
                setattr(robot, var, degrees(value))


fcgui.addCommand('SetJoints', _SetJointsCommand())
