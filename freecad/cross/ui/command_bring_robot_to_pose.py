# Bring a Cross::Robot to a Cross::Pose.

import FreeCADGui as fcgui

# Typing hints.
from freecad.cross.robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from freecad.cross.pose import Pose as CrossPose  # A Cross::Pose, i.e. a DocumentObject with Proxy "Pose". # noqa: E501


class _BringRobotToPoseCommand:
    """The command definition to bring a Robot to a Pose."""

    def GetResources(self):
        return {
            'Pixmap': 'bulb',
            'MenuText': 'Bring Robot to Pose',
            'Accel': 'B, R',
            'ToolTip': 'Bring a CROSS::Robot to a CROSS::Pose. Select a CROSS::Robot and a CROSS::Pose.',
        }

    def IsActive(self):
        # Import late to avoid slowing down workbench start-up.
        from ..freecad_utils import validate_types

        objs = fcgui.Selection.getSelection()
        if not objs:
            return False
        if len(objs) != 2:
            return False
        return bool(validate_types(
                objs,
                ['Cross::Robot', 'Cross::Pose'],
                respect_order=False,
        ))

    def Activated(self):
        # Import late to avoid slowing down workbench start-up.
        from ..freecad_utils import validate_types
        from ..freecad_utils import warn

        sel = fcgui.Selection.getSelection()
        robot_and_pose = validate_types(sel, ['Cross::Robot', 'Cross::Pose'])
        if not robot_and_pose:
            warn('Please select a CROSS::Robot and a CROSS::Pose', True)
            return

        robot, pose = robot_and_pose
        bring_to_pose(robot, pose)


def bring_to_pose(robot: CrossRobot, pose: CrossPose):
    # Import late to avoid slowing down workbench start-up.
    from ..freecad_utils import warn
    from ..ik import ik
    from ..wb_utils import get_chain_from_to
    from ..wb_utils import ros_name
    from ..wb_utils import joint_values_si_units_from_freecad as wb_si_from_fc

    if not pose.EndEffector:
        warn(f'Pose "{pose.Label}" has no end-effector link', True)
        return
    root_link = robot.Proxy.get_root_link()
    if not root_link:
        warn(f'Robot "{robot.Label}" has no root link', True)
        return
    root_link_name = ros_name(root_link)
    ee_link = robot.Proxy.get_link(pose.EndEffector)
    if not ee_link:
        warn(f'The end-effector link "{pose.EndEffector}" of pose "{pose.Label}" is not part of robot "{robot.Label}"', True)
        return
    chain = get_chain_from_to(robot, root_link_name, pose.EndEffector)
    if not chain:
        warn(f'Could not find a kinematic chain from the root link "{root_link_name}" to the end-effector link "{pose.EndEffector}" in robot "{robot.Label}"', True)
        return

    sols = ik(
            robot=robot,
            from_link=ros_name(root_link),
            to_link=pose.EndEffector,
            target=pose.Placement,
            # algorithm: IKAlgorithm = IKAlgorithm.PINOCCHIO_SINGLE,
    )
    if not sols:
        warn(f'IK failed to find a solution for robot "{robot.Label}" to reach pose "{pose.Label}"', True)
        return

    actuated_joints = robot.Proxy.get_actuated_joints()
    chain_joints = [o for o in chain if o in actuated_joints]
    first_sol_si_dict = wb_si_from_fc({j: sol for j, sol in zip(chain_joints, sols[0])})
    robot.Proxy.set_joint_values(first_sol_si_dict)
    robot.Document.recompute()


fcgui.addCommand('BringRobotToPose', _BringRobotToPoseCommand())
