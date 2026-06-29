"""Launch a deterministic known-reachable plan-only assembly sequence."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start fixed-target, fixed-start two-stage planning without execution."""
    sequence_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_demo.launch.py',
    ])
    reachable_params = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_sequence_reachable_params.yaml',
    ])

    return LaunchDescription([
        LogInfo(
            msg='Launching the deterministic known-reachable Panda assembly '
            'sequence profile. Both stages are planned only; execution is disabled.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sequence_demo),
            launch_arguments={
                'params_file': reachable_params,
                'use_dynamic_target_scene': 'false',
                'start_state_mode': 'fixed',
                'planning_time_sec': '5.0',
                'num_planning_attempts': '1',
                'position_tolerance': '0.01',
                'orientation_tolerance': '0.10',
            }.items(),
        ),
    ])
