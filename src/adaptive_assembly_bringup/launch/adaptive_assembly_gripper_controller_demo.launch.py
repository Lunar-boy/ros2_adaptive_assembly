"""Compose the Gazebo Panda controllers and simulator-only gripper bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch Gazebo controller actuation without physical grasp validation."""
    bridge_defaults = {
        'controller_action_name': (
            '/panda_gripper_controller/follow_joint_trajectory'
        ),
        'send_goals': 'true',
        'simulated_only': 'true',
        'open_position': '0.04',
        'close_position': '0.0',
        'goal_time_sec': '1.0',
        'wait_for_controller_sec': '5.0',
        'result_timeout_sec': '5.0',
    }
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'launch',
        'adaptive_assembly_panda_gazebo.launch.py',
    ])
    bridge_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_manipulation'),
        'launch',
        'gripper_action_bridge.launch.py',
    ])
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in bridge_defaults.items()
        ],
        LogInfo(msg=(
            'Launching simulator-only Panda arm and gripper actuation. '
            'No contact sensing or physical grasp verification is included.'
        )),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={
                name: LaunchConfiguration(name) for name in bridge_defaults
            }.items(),
        ),
    ])
