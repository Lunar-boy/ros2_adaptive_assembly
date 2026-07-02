"""Launch simulator-only Gazebo grasp attachment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Declare the public interface and start the attachment node."""
    defaults = {
        'object_grasp_state_topic': '/object_grasp_state',
        'object_grasp_attached_topic': '/object_grasp_attached',
        'status_topic': '/gazebo_attach_detach_status',
        'gazebo_object_attached_topic': '/gazebo_object_attached',
        'pose_error_mm_topic': '/gazebo_attach_pose_error_mm',
        'target_entity_name': 'target_object',
        'world_frame': 'world',
        'gripper_frame': 'panda_hand',
        'attach_update_period_sec': '0.05',
        'service_timeout_sec': '2.0',
        'enable_service_calls': 'true',
        'simulated_only': 'true',
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    parameters = {name: LaunchConfiguration(name) for name in defaults}
    return LaunchDescription(arguments + [
        Node(
            package='adaptive_assembly_manipulation',
            executable='gazebo_attach_detach_node',
            name='gazebo_attach_detach_node', output='screen',
            parameters=[parameters],
        )
    ])
