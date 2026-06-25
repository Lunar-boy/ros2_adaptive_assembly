"""Launch seeded Panda planning benchmark with request guard enabled."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the seeded benchmark profile with guarded planning requests."""
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
            msg='Launching guarded seeded Panda planning benchmark. '
            'enable_request_guard=true, required_frame_id=panda_link0, '
            'workspace x=[0.20,0.80], y=[-0.50,0.50], z=[0.10,0.60]. '
            'Trajectories are not executed.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_planning_demo_launch),
            launch_arguments={
                'params_file': benchmark_params_file,
                'enable_request_guard': 'true',
                'required_frame_id': 'panda_link0',
                'workspace_min_x': '0.20',
                'workspace_max_x': '0.80',
                'workspace_min_y': '-0.50',
                'workspace_max_y': '0.50',
                'workspace_min_z': '0.10',
                'workspace_max_z': '0.60',
            }.items(),
        ),
    ])
