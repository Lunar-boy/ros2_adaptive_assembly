"""Launch the simulator-only gripper trajectory action bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Declare bridge configuration and launch the node."""
    defaults = {
        'controller_action_name': (
            '/panda_gripper_controller/follow_joint_trajectory'
        ),
        'send_goals': 'true',
        'simulated_only': 'true',
        'open_position': '0.04',
        'close_position': '0.0',
        'goal_time_sec': '1.0',
        'wait_for_controller_sec': '5.0',
        'result_timeout_sec': '5.0',
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    parameters = {
        name: ParameterValue(
            LaunchConfiguration(name),
            value_type=(
                bool if name in ('send_goals', 'simulated_only')
                else float if name.endswith('_position') or name.endswith('_sec')
                else str
            ),
        )
        for name in defaults
    }
    return LaunchDescription(arguments + [
        LogInfo(msg=(
            'Launching simulator-only gripper controller action bridge. '
            'This does not verify grasp success or support real hardware.'
        )),
        Node(
            package='adaptive_assembly_manipulation',
            executable='gripper_action_bridge_node',
            name='gripper_action_bridge_node',
            output='screen',
            parameters=[parameters],
        ),
    ])
