"""Launch the adaptive assembly task node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the assembly task launch description."""
    parameter_defaults = {
        'pre_grasp_height_offset': '0.20',
        'assembly_height_offset': '0.05',
        'replan_distance_threshold': '0.03',
    }

    launch_arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in parameter_defaults.items()
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in parameter_defaults
    }

    return LaunchDescription(launch_arguments + [
        Node(
            package='adaptive_assembly_task',
            executable='assembly_task_node',
            name='assembly_task_node',
            output='screen',
            parameters=[parameters],
        ),
    ])
