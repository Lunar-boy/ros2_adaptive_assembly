"""Launch seeded Panda planning benchmark with slow scaling factors."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the benchmark profile with 0.25 velocity/acceleration scaling."""
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
            msg='Launching seeded Panda planning benchmark with '
            'num_planning_attempts=1, max_velocity_scaling_factor=0.25, '
            'and max_acceleration_scaling_factor=0.25. Trajectories are not '
            'executed.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_planning_demo_launch),
            launch_arguments={
                'params_file': benchmark_params_file,
                'num_planning_attempts': '1',
                'max_velocity_scaling_factor': '0.25',
                'max_acceleration_scaling_factor': '0.25',
            }.items(),
        ),
    ])
