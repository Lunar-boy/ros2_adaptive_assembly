"""Add the visual-only Gazebo workcell to the action-level success demo."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose visual Gazebo scenery with non-physical action execution."""
    bringup_share = FindPackageShare('adaptive_assembly_bringup')
    gazebo_launch = PathJoinSubstitution([
        bringup_share, 'launch',
        'adaptive_assembly_gazebo_workcell_demo.launch.py',
    ])
    success_launch = PathJoinSubstitution([
        bringup_share, 'launch',
        'adaptive_assembly_ros2_control_success_demo.launch.py',
    ])
    return LaunchDescription([
        LogInfo(msg=(
            'Gazebo is visual only: the simulated action server validates '
            'the execution path and does not move a Panda model in Gazebo.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'launch_pipeline': 'false'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(success_launch),
        ),
    ])
