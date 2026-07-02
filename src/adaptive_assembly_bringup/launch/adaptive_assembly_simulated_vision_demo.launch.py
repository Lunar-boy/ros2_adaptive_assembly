"""Compose simulated vision with task generation and Gazebo target sync."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the headless simulator-only vision demonstration."""
    vision = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_perception'), 'launch',
        'simulated_vision_perception.launch.py',
    ])
    task = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_task'), 'launch',
        'assembly_task.launch.py',
    ])
    workcell = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'), 'launch',
        'adaptive_assembly_gazebo_workcell_demo.launch.py',
    ])
    sync = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'), 'launch',
        'gazebo_target_pose_sync.launch.py',
    ])
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    default_world = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'), 'worlds',
        'adaptive_assembly_workcell.sdf',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gz_args', default_value=['-r -s ', world]),
        LogInfo(msg=(
            'Launching simulator-only marker perception, task pose generation, '
            'and headless Gazebo target synchronization. No real camera or '
            'hardware interface is enabled.'
        )),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vision)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(task)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(workcell),
            launch_arguments={'world': world, 'gz_args': gz_args}.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sync)),
    ])
