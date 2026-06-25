"""Launch the deterministic fixed-yaw Panda planning benchmark profile."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the Panda planning demo with fixed-yaw benchmark parameters."""
    panda_planning_demo_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_planning_demo.launch.py',
    ])
    benchmark_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_benchmark_fixed_yaw_params.yaml',
    ])

    return LaunchDescription([
        LogInfo(
            msg='Launching fixed-yaw deterministic Panda planning benchmark: '
            'random_seed=303 and yaw_min=yaw_max=0. Trajectories are not '
            'executed.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_planning_demo_launch),
            launch_arguments={'params_file': benchmark_params_file}.items(),
        ),
    ])
