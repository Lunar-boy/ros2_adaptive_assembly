"""Launch the fake adaptive assembly perception node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the fake perception launch description."""
    parameter_defaults = {
        'publish_period_sec': '5.0',
        'x_min': '0.35',
        'x_max': '0.55',
        'y_min': '-0.25',
        'y_max': '0.25',
        'z': '0.15',
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
            package='adaptive_assembly_perception',
            executable='fake_object_pose_node',
            name='fake_object_pose_node',
            output='screen',
            parameters=[parameters],
        ),
    ])
