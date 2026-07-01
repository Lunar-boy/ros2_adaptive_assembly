"""Launch the simulator-only recovery orchestrator."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the recovery orchestrator launch description."""
    return LaunchDescription([
        Node(
            package='adaptive_assembly_recovery',
            executable='recovery_orchestrator_node',
            name='recovery_orchestrator_node',
            output='screen',
        ),
    ])
