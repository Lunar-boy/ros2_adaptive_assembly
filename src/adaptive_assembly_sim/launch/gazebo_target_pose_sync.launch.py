"""Launch the simulator-only Gazebo target pose synchronizer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the target pose synchronizer with configurable interfaces."""
    arguments = [
        ('target_pose_topic', '/target_pose'),
        ('target_entity_name', 'target_object'),
        ('world_frame', 'world'),
        ('status_topic', '/gazebo_target_sync_status'),
        ('pose_error_mm_topic', '/gazebo_target_pose_error_mm'),
        ('pose_error_deg_topic', '/gazebo_target_pose_error_deg'),
        ('service_timeout_sec', '2.0'),
        ('simulated_only', 'true'),
        ('enable_service_calls', 'true'),
    ]
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, default in arguments
        ],
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_target_pose_service_bridge',
            output='screen',
            arguments=[
                '/world/adaptive_assembly_workcell/set_pose'
                '@ros_gz_interfaces/srv/SetEntityPose'
                '@gz.msgs.Pose@gz.msgs.Boolean',
            ],
        ),
        Node(
            package='adaptive_assembly_sim',
            executable='gazebo_target_pose_sync_node',
            name='gazebo_target_pose_sync_node',
            output='screen',
            parameters=[{
                name: LaunchConfiguration(name) for name, _ in arguments
            }],
        ),
    ])
