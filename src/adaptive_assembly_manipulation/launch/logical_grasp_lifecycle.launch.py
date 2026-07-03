"""Launch the simulator-only logical grasp lifecycle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Declare lifecycle configuration and launch the node."""
    arguments = [
        (
            'stage_status_topic',
            '/assembly_ros2_control_execution_stage_status',
        ),
        ('execution_status_topic', '/assembly_ros2_control_execution_status'),
        ('gripper_command_topic', '/gripper_command'),
        ('gripper_command_status_topic', '/gripper_command_status'),
        ('object_grasp_state_topic', '/object_grasp_state'),
        ('object_grasp_attached_topic', '/object_grasp_attached'),
        ('lifecycle_status_topic', '/logical_grasp_lifecycle_status'),
        ('object_id', 'target_object'),
        ('gripper_id', 'panda_hand'),
        ('attach_parent_frame', 'panda_hand'),
        ('attach_stage', 'pre_grasp'),
        ('release_stage', 'execution_success'),
        ('release_parent_frame', 'world'),
        ('detach_on_failure', 'false'),
        ('simulated_only', 'true'),
    ]
    parameters = {
        name: (
            ParameterValue(LaunchConfiguration(name), value_type=bool)
            if name in ('detach_on_failure', 'simulated_only')
            else LaunchConfiguration(name)
        )
        for name, _ in arguments
    }
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, default in arguments
        ],
        LogInfo(msg=(
            'Launching logical grasp state only. No physical gripper or '
            'Gazebo attachment is used.'
        )),
        Node(
            package='adaptive_assembly_manipulation',
            executable='logical_grasp_lifecycle_node',
            name='logical_grasp_lifecycle_node',
            output='screen',
            parameters=[parameters],
        ),
    ])
