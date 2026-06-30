"""Launch deterministic sequence planning with dry-run execution."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose the reachable planner with a message-only dry-run executor."""
    reachable_sequence = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
    ])

    pre_grasp_trajectory_topic = LaunchConfiguration(
        'pre_grasp_trajectory_topic'
    )
    assembly_trajectory_topic = LaunchConfiguration(
        'assembly_trajectory_topic'
    )
    status_topic = LaunchConfiguration('status_topic')
    success_topic = LaunchConfiguration('success_topic')
    duration_topic = LaunchConfiguration('duration_topic')
    stage_status_topic = LaunchConfiguration('stage_status_topic')
    require_panda_joints = LaunchConfiguration('require_panda_joints')
    expected_joint_prefix = LaunchConfiguration('expected_joint_prefix')
    simulate_real_time = LaunchConfiguration('simulate_real_time')
    simulated_stage_duration_ms = LaunchConfiguration(
        'simulated_stage_duration_ms'
    )
    publish_stage_status = LaunchConfiguration('publish_stage_status')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pre_grasp_trajectory_topic',
            default_value='/pre_grasp_trajectory',
            description='Exported pre-grasp RobotTrajectory topic.',
        ),
        DeclareLaunchArgument(
            'assembly_trajectory_topic',
            default_value='/assembly_trajectory',
            description='Exported assembly RobotTrajectory topic.',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/assembly_execution_status',
            description='Aggregate dry-run execution status topic.',
        ),
        DeclareLaunchArgument(
            'success_topic',
            default_value='/assembly_execution_success',
            description='Aggregate dry-run execution success topic.',
        ),
        DeclareLaunchArgument(
            'duration_topic',
            default_value='/assembly_execution_duration_ms',
            description='Aggregate dry-run execution duration topic.',
        ),
        DeclareLaunchArgument(
            'stage_status_topic',
            default_value='/assembly_execution_stage_status',
            description='Per-stage dry-run execution status topic.',
        ),
        DeclareLaunchArgument(
            'require_panda_joints',
            default_value='true',
            description='Require an expected Panda joint in each trajectory.',
        ),
        DeclareLaunchArgument(
            'expected_joint_prefix',
            default_value='panda_joint',
            description=(
                'Required joint-name prefix when validation is enabled.'
            ),
        ),
        DeclareLaunchArgument(
            'simulate_real_time',
            default_value='false',
            description=(
                'Sleep for the configured duration during each dry-run stage.'
            ),
        ),
        DeclareLaunchArgument(
            'simulated_stage_duration_ms',
            default_value='10.0',
            description='Optional dry-run delay for each sequence stage.',
        ),
        DeclareLaunchArgument(
            'publish_stage_status',
            default_value='true',
            description='Publish per-stage dry-run status messages.',
        ),
        LogInfo(
            msg='Launching the deterministic known-reachable sequence with a '
            'message-only dry-run executor. Real execution is disabled.'
        ),
        Node(
            package='adaptive_assembly_execution',
            executable='dry_run_sequence_executor_node',
            name='dry_run_sequence_executor_node',
            output='screen',
            parameters=[{
                'pre_grasp_trajectory_topic': pre_grasp_trajectory_topic,
                'assembly_trajectory_topic': assembly_trajectory_topic,
                'status_topic': status_topic,
                'success_topic': success_topic,
                'duration_topic': duration_topic,
                'stage_status_topic': stage_status_topic,
                'require_panda_joints': ParameterValue(
                    require_panda_joints,
                    value_type=bool,
                ),
                'expected_joint_prefix': expected_joint_prefix,
                'simulate_real_time': ParameterValue(
                    simulate_real_time,
                    value_type=bool,
                ),
                'simulated_stage_duration_ms': ParameterValue(
                    simulated_stage_duration_ms,
                    value_type=float,
                ),
                'publish_stage_status': ParameterValue(
                    publish_stage_status,
                    value_type=bool,
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(reachable_sequence),
            launch_arguments={
                'pre_grasp_trajectory_topic': pre_grasp_trajectory_topic,
                'assembly_trajectory_topic': assembly_trajectory_topic,
            }.items(),
        ),
    ])
