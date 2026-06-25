"""Launch the dynamic target PlanningScene collision object node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start only the dynamic target scene updater."""
    return LaunchDescription([
        Node(
            package='adaptive_assembly_planning',
            executable='dynamic_target_scene_node',
            name='dynamic_target_scene_node',
            output='screen',
            parameters=[{
                'input_pose_topic': '/panda_pre_grasp_pose',
                'object_id': 'target_object_dynamic',
                'size_x': 0.05,
                'size_y': 0.05,
                'size_z': 0.04,
                'x_offset': 0.0,
                'y_offset': 0.0,
                'z_offset': -0.20,
                'min_update_distance': 0.02,
                'publish_updates': True,
            }],
        ),
    ])
