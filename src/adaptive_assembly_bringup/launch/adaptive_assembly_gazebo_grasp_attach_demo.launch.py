"""Compose Gazebo Panda execution with logical and kinematic grasp state."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch full simulation, logical lifecycle, and Gazebo attachment."""
    full_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'), 'launch',
        'adaptive_assembly_full_gazebo_execution_demo.launch.py',
    ])
    lifecycle = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_manipulation'), 'launch',
        'logical_grasp_lifecycle.launch.py',
    ])
    attachment = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_manipulation'), 'launch',
        'gazebo_attach_detach.launch.py',
    ])
    enable_calls = LaunchConfiguration('enable_service_calls')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_service_calls', default_value='true',
            description=(
                'Call Gazebo set_pose; false enables fixture validation.'
            ),
        ),
        LogInfo(msg=(
            'Launching simulator-only Gazebo Panda execution with logical '
            'grasp lifecycle and kinematic object attachment.'
        )),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(full_demo)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lifecycle)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(attachment),
            launch_arguments={'enable_service_calls': enable_calls}.items(),
        ),
    ])
