"""Adapt task-level pre-grasp poses into the Panda planning frame."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the adapter without launching MoveIt2."""
    return LaunchDescription([
        Node(
            package='adaptive_assembly_planning',
            executable='panda_pre_grasp_pose_adapter_node',
            name='panda_pre_grasp_pose_adapter_node',
            output='screen',
            parameters=[{
                'input_topic': '/pre_grasp_pose',
                'output_topic': '/panda_pre_grasp_pose',
                'output_frame_id': 'panda_link0',
                'x_offset': 0.0,
                'y_offset': 0.0,
                'z_offset': 0.0,
                'use_fixed_orientation': True,
                'fixed_qx': 1.0,
                'fixed_qy': 0.0,
                'fixed_qz': 0.0,
                'fixed_qw': 0.0,
                'normalize_quaternion': True,
            }],
        ),
    ])
