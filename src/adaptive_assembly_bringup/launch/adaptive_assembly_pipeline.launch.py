"""Launch the current non-MoveIt adaptive assembly pipeline."""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start fake perception and task-level pose generation."""
    parameter_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_params.yaml',
    ])

    return LaunchDescription([
        LogInfo(
            msg='Launching non-MoveIt adaptive assembly pipeline: '
            'fake perception + task pose generation.'
        ),
        Node(
            package='adaptive_assembly_perception',
            executable='fake_object_pose_node',
            name='fake_object_pose_node',
            output='screen',
            parameters=[parameter_file],
        ),
        Node(
            package='adaptive_assembly_task',
            executable='assembly_task_node',
            name='assembly_task_node',
            output='screen',
            parameters=[parameter_file],
        ),
    ])
