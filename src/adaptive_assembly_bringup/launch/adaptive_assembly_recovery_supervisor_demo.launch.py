"""Launch the dry-run sequence pipeline with recovery supervision."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose deterministic dry-run execution and the supervisor."""
    dry_run_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_dry_run_execution.launch.py',
    ])
    enable_service_calls = LaunchConfiguration('enable_service_calls')
    max_recovery_attempts = LaunchConfiguration('max_recovery_attempts')
    publish_heartbeat = LaunchConfiguration('publish_heartbeat')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_service_calls',
            default_value='false',
            description='Call existing PlanningScene reset Trigger services.',
        ),
        DeclareLaunchArgument(
            'max_recovery_attempts',
            default_value='1',
            description='Maximum recovery actions published per run.',
        ),
        DeclareLaunchArgument(
            'publish_heartbeat',
            default_value='true',
            description='Publish status heartbeats before a terminal result.',
        ),
        LogInfo(
            msg='Launching the deterministic dry-run sequence with recovery '
            'supervision. Service calls default to disabled and real '
            'trajectory execution is unavailable.'
        ),
        Node(
            package='adaptive_assembly_recovery',
            executable='recovery_supervisor_node',
            name='recovery_supervisor_node',
            output='screen',
            parameters=[{
                'enable_service_calls': ParameterValue(
                    enable_service_calls, value_type=bool
                ),
                'max_recovery_attempts': ParameterValue(
                    max_recovery_attempts, value_type=int
                ),
                'publish_heartbeat': ParameterValue(
                    publish_heartbeat, value_type=bool
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dry_run_launch),
            launch_arguments={
                # The reachable profile intentionally has no dynamic object.
                'use_planning_scene_audit': 'false',
            }.items(),
        ),
    ])
