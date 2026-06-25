"""Launch lightweight RViz markers for adaptive assembly poses."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the visualization-only adaptive assembly marker node."""
    return LaunchDescription([
        Node(
            package='adaptive_assembly_planning',
            executable='adaptive_assembly_marker_node',
            name='adaptive_assembly_marker_node',
            output='screen',
            parameters=[{
                'marker_topic': '/adaptive_assembly_markers',
                'status_topic': '/adaptive_assembly_marker_status',
                'marker_scale': 0.05,
                'arrow_length': 0.12,
                'marker_lifetime_sec': 0.0,
            }],
        ),
    ])
