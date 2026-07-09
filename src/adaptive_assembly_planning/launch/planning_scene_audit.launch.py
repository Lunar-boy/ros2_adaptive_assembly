"""Launch the read-only PlanningScene audit node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the PlanningScene audit node without modifying the scene."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'expected_object_ids',
            default_value='work_table,target_support,target_object_dynamic',
            description='Comma-separated PlanningScene object IDs expected.',
        ),
        Node(
            package='adaptive_assembly_planning',
            executable='planning_scene_audit_node',
            name='planning_scene_audit_node',
            output='screen',
            parameters=[{
                'expected_object_ids': LaunchConfiguration(
                    'expected_object_ids'
                ),
                'audit_period_sec': 2.0,
                'status_topic': '/planning_scene_audit_status',
                'ready_topic': '/planning_scene_audit_ready',
            }],
        ),
    ])
