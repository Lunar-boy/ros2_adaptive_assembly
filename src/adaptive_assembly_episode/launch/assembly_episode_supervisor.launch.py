"""Launch the passive simulator-only assembly episode supervisor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the standalone supervisor launch description."""
    arguments = [
        DeclareLaunchArgument('episode_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument('require_planning_success', default_value='true'),
        DeclareLaunchArgument('require_execution_success', default_value='true'),
        DeclareLaunchArgument(
            'require_logical_grasp_released', default_value='true'
        ),
        DeclareLaunchArgument(
            'require_gazebo_attach_success', default_value='true'
        ),
        DeclareLaunchArgument('require_insertion_success', default_value='true'),
        DeclareLaunchArgument('simulated_only', default_value='true'),
    ]
    parameters = {
        name: LaunchConfiguration(name)
        for name in (
            'episode_timeout_sec',
            'require_planning_success',
            'require_execution_success',
            'require_logical_grasp_released',
            'require_gazebo_attach_success',
            'require_insertion_success',
            'simulated_only',
        )
    }
    return LaunchDescription(arguments + [
        Node(
            package='adaptive_assembly_episode',
            executable='assembly_episode_supervisor_node',
            name='assembly_episode_supervisor_node',
            output='screen',
            parameters=[parameters],
        ),
    ])
