"""Compose the PR35 success fixture with logical grasp state."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch execution success and the independent lifecycle observer."""
    success_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_ros2_control_success_demo.launch.py',
    ])
    lifecycle = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_manipulation'),
        'launch',
        'logical_grasp_lifecycle.launch.py',
    ])
    return LaunchDescription([
        LogInfo(msg=(
            'Launching simulator-only execution with logical grasp state. '
            'No physical gripper or Gazebo attachment is used.'
        )),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lifecycle)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(success_demo)),
    ])
