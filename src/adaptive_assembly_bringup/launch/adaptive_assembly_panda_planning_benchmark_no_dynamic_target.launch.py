"""Launch the seeded Panda planning benchmark without dynamic target object."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start seeded planning benchmark with dynamic target scene disabled."""
    panda_planning_demo_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_planning_demo.launch.py',
    ])
    benchmark_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_benchmark_params.yaml',
    ])

    return LaunchDescription([
        LogInfo(
            msg='Launching seeded Panda planning benchmark without the '
            'dynamic target collision object. Trajectories are not executed.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_planning_demo_launch),
            launch_arguments={
                'params_file': benchmark_params_file,
                'use_dynamic_target_scene': 'false',
            }.items(),
        ),
    ])
