"""Launch plan-only Panda assembly sequencing from a fixed robot state."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the sequence demo with deterministic fixed-state planning."""
    sequence_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_demo.launch.py',
    ])

    return LaunchDescription([
        LogInfo(
            msg='Launching plan-only Panda assembly sequence planning with a '
            'deterministic fixed start state. No trajectory is executed.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sequence_demo),
            launch_arguments={'start_state_mode': 'fixed'}.items(),
        ),
    ])
