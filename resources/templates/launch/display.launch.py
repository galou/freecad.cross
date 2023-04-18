from pathlib import Path

import launch
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

import launch_ros
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(launch_ros.substitutions.FindPackageShare(package='{package_name}').find('{package_name}'))
    default_model_path = pkg_share / 'urdf/{urdf_file}'
    default_rviz_config_path = pkg_share / 'rviz/robot_description.rviz'

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {{
                # ParameterValue is required to avoid being interpreted as YAML.
                'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str),
            }},
            ]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui',
                                             default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model',
                                             default_value=str(default_model_path),
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig',
                                             default_value=str(default_rviz_config_path),
                                             description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
