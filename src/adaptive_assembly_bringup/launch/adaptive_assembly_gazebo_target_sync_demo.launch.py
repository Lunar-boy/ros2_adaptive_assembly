"""Compose fake perception, the Gazebo workcell, and target synchronization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch fake perception, Gazebo workcell, and pose synchronization."""
    workcell = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'), 'launch',
        'adaptive_assembly_gazebo_workcell_demo.launch.py',
    ])
    sync = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'), 'launch',
        'gazebo_target_pose_sync.launch.py',
    ])
    default_world = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'), 'worlds',
        'adaptive_assembly_workcell.sdf',
    ])
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    enable_service_calls = LaunchConfiguration('enable_service_calls')
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Absolute path to the Gazebo workcell world.',
        ),
        DeclareLaunchArgument(
            'gz_args', default_value=['-r -s ', world],
            description='Gazebo arguments; default is headless server mode.',
        ),
        DeclareLaunchArgument(
            'enable_service_calls', default_value='true',
            description='Call Gazebo, or exercise deterministic skipped mode.',
        ),
        LogInfo(msg=(
            'Launching simulator-only target synchronization. No gripper, '
            'attachment, real hardware, or contact execution is enabled.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(workcell),
            launch_arguments={'world': world, 'gz_args': gz_args}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sync),
            launch_arguments={
                'enable_service_calls': enable_service_calls,
            }.items(),
        ),
    ])
