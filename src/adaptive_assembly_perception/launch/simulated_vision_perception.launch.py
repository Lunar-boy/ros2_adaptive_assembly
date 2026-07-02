"""Launch deterministic simulator-only marker perception."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Expose all simulated vision interfaces and emulator parameters."""
    defaults = {
        'target_pose_topic': '/target_pose',
        'perceived_pose_topic': '/perceived_target_pose',
        'status_topic': '/simulated_vision_perception_status',
        'world_frame': 'world',
        'camera_frame': 'simulated_camera',
        'target_frame_id': 'target_object',
        'marker_id': '0',
        'target_entity_name': 'target_object',
        'publish_period_sec': '1.0',
        'camera_x': '0.0', 'camera_y': '0.0', 'camera_z': '1.0',
        'camera_yaw': '0.0',
        'x': '0.45', 'y': '0.0', 'z': '0.15', 'yaw': '0.0',
        'position_noise_std': '0.0', 'yaw_noise_std': '0.0',
        'publish_immediately': 'true',
        'enable_camera_topics': 'false',
        'simulated_only': 'true',
    }
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in defaults.items()
        ],
        Node(
            package='adaptive_assembly_perception',
            executable='simulated_marker_pose_node',
            name='simulated_marker_pose_node',
            output='screen',
            parameters=[{
                name: LaunchConfiguration(name) for name in defaults
            }],
        ),
    ])
