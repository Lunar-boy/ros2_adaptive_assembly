"""Bridge Gazebo world poses and publish one selected entity pose."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        ('pose_info_topic', '/world/adaptive_assembly_workcell/pose/info'),
        ('input_message_type', 'pose_vector'),
        ('target_entity_name', 'target_object'),
        ('world_frame', 'world'),
        ('output_pose_topic', '/gazebo_target_object_pose'),
        ('status_topic', '/gazebo_target_object_pose_status'),
        ('available_topic', '/gazebo_target_object_pose_available'),
        ('pose_age_ms_topic', '/gazebo_target_object_pose_age_ms'),
        ('stale_timeout_sec', '2.0'),
        ('publish_period_sec', '0.1'),
        ('require_model_name_match', 'true'),
        ('simulated_only', 'true'),
    ]
    topic = LaunchConfiguration('pose_info_topic')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default)
          for name, default in arguments],
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_entity_pose_info_bridge',
            output='screen',
            # Jazzy ros_gz_bridge maps gz.msgs.Pose_V to TFMessage. The '['
            # requests the Gazebo-to-ROS direction only.
            arguments=[[topic, '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']],
        ),
        Node(
            package='adaptive_assembly_sim',
            executable='gazebo_entity_pose_observer_node',
            name='gazebo_entity_pose_observer_node',
            output='screen',
            parameters=[{name: LaunchConfiguration(name)
                         for name, _ in arguments}],
        ),
    ])
