"""Adapt the observed Gazebo target model pose for task consumption."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start the event-driven Gazebo-to-task target pose adapter."""
    arguments = (
        ('input_pose_topic', '/gazebo_target_object_pose'),
        ('output_pose_topic', '/target_pose'),
        ('target_reference_z_offset', '0.05'),
        ('output_frame_id', 'world'),
        ('use_sim_time', 'false'),
    )
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, default in arguments
        ],
        Node(
            package='adaptive_assembly_sim',
            executable='gazebo_target_pose_adapter_node',
            name='gazebo_target_pose_adapter_node',
            output='screen',
            parameters=[{
                'input_pose_topic': LaunchConfiguration('input_pose_topic'),
                'output_pose_topic': LaunchConfiguration('output_pose_topic'),
                'target_reference_z_offset': ParameterValue(
                    LaunchConfiguration('target_reference_z_offset'),
                    value_type=float,
                ),
                'output_frame_id': LaunchConfiguration('output_frame_id'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
            }],
        ),
    ])
