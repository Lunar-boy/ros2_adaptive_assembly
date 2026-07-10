"""
Launch the current non-MoveIt adaptive assembly pipeline.

The params_file launch argument allows switching between normal demo and
benchmark parameter profiles.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start fake perception and task-level pose generation."""
    parameter_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_params.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=parameter_file,
            description='Parameter YAML for perception and task nodes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when an upstream simulator provides /clock.',
        ),
        LogInfo(
            msg='Launching non-MoveIt adaptive assembly pipeline: '
            'fake perception + task pose generation.'
        ),
        Node(
            package='adaptive_assembly_perception',
            executable='fake_object_pose_node',
            name='fake_object_pose_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package='adaptive_assembly_task',
            executable='assembly_task_node',
            name='assembly_task_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])
