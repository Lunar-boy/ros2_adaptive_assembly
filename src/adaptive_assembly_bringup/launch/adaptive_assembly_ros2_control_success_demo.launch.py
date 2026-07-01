"""Launch the deterministic action-level ros2_control success fixture."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose reachable planning, the action fixture, and the executor."""
    execution_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_ros2_control_execution.launch.py',
    ])
    action_name = LaunchConfiguration('controller_action_name')
    result_delay = LaunchConfiguration('result_delay_sec')
    result_timeout = LaunchConfiguration('result_timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_action_name',
            default_value=(
                '/simulated_panda_arm_controller/follow_joint_trajectory'
            ),
            description=(
                'Isolated fixture action, separate from MoveIt demo '
                'controllers.'
            ),
        ),
        DeclareLaunchArgument('result_delay_sec', default_value='0.1'),
        DeclareLaunchArgument('result_timeout_sec', default_value='10.0'),
        LogInfo(msg=(
            'Launching simulator-only action-level ros2_control success '
            'fixture. No real hardware or physical Gazebo motion is used.'
        )),
        Node(
            package='adaptive_assembly_execution',
            executable='simulated_follow_joint_trajectory_server_node',
            name='simulated_follow_joint_trajectory_server_node',
            output='screen',
            parameters=[{
                'action_name': action_name,
                'result_mode': 'success',
                'result_delay_sec': ParameterValue(
                    result_delay, value_type=float
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(execution_launch),
            launch_arguments={
                'controller_action_name': action_name,
                'result_timeout_sec': result_timeout,
            }.items(),
        ),
    ])
