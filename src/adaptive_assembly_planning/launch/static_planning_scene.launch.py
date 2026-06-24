"""Launch static PlanningScene collision objects for the Panda demo."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the static PlanningScene node without launching MoveIt2."""
    return LaunchDescription([
        Node(
            package='adaptive_assembly_planning',
            executable='static_planning_scene_node',
            name='static_planning_scene_node',
            output='screen',
            parameters=[{
                'planning_frame': 'panda_link0',
                'apply_delay_sec': 1.0,
                'add_work_table': True,
                'table_x': 0.45,
                'table_y': 0.0,
                'table_z': -0.04,
                'table_size_x': 0.80,
                'table_size_y': 0.80,
                'table_size_z': 0.04,
                'add_target_support': True,
                'target_support_x': 0.45,
                'target_support_y': 0.0,
                'target_support_z': 0.01,
                'target_support_size_x': 0.12,
                'target_support_size_y': 0.12,
                'target_support_size_z': 0.02,
            }],
        ),
    ])
