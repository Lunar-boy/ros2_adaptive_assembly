"""Launch the simulator-only full physical pick-place demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start Gazebo with the physical workcell and physical execution path."""
    default_world = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'worlds',
        'adaptive_assembly_physical_workcell.sdf',
    ])
    sim_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'launch',
        'adaptive_assembly_panda_gazebo.launch.py',
    ])
    execution_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
    ])

    world = LaunchConfiguration('world')
    enable_arm_collisions = LaunchConfiguration('enable_arm_collisions')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Physical Gazebo SDF workcell for contact verification.',
        ),
        DeclareLaunchArgument(
            'enable_arm_collisions',
            default_value='true',
            description=(
                'Enable Panda collision geometry required by the physical '
                'finger contact path.'
            ),
        ),
        LogInfo(msg=(
            'Launching simulator-only full physical pick-place demo with '
            'adaptive_assembly_physical_workcell.sdf, '
            'world_name=adaptive_assembly_physical_workcell, and '
            'enable_arm_collisions=true by default. This path is separate '
            'from the kinematic attach visual demo.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'world': world,
                'world_name': 'adaptive_assembly_physical_workcell',
                'enable_arm_collisions': enable_arm_collisions,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(execution_launch),
            launch_arguments={
                'use_standard_panda_demo': 'false',
            }.items(),
        ),
    ])
