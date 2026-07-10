"""Adapt task-level grasp poses into the Panda planning frame."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start the parameterized Panda pose adapter for grasp poses."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time when an upstream simulator provides /clock.',
        ),
        Node(
            package='adaptive_assembly_planning',
            executable='panda_pre_grasp_pose_adapter_node',
            name='panda_grasp_pose_adapter_node',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
                'input_topic': '/grasp_pose',
                'output_topic': '/panda_grasp_pose',
                'output_frame_id': 'panda_link0',
                'use_tf_transform': False,
                'target_frame_id': 'panda_link0',
                'tf_lookup_timeout_sec': 0.2,
                'status_topic': '/panda_grasp_pose_adapter_status',
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
