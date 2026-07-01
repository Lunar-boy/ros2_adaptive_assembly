"""Launch reachable planning with an optional ros2_control execution bridge."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose deterministic planning and simulator-only controller output."""
    reachable_sequence = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
    ])

    pre_grasp_topic = LaunchConfiguration('pre_grasp_trajectory_topic')
    assembly_topic = LaunchConfiguration('assembly_trajectory_topic')
    controller = LaunchConfiguration('controller_action_name')
    status_topic = LaunchConfiguration('status_topic')
    success_topic = LaunchConfiguration('success_topic')
    duration_topic = LaunchConfiguration('duration_topic')
    stage_status_topic = LaunchConfiguration('stage_status_topic')
    wait_for_controller = LaunchConfiguration('wait_for_controller_sec')
    result_timeout = LaunchConfiguration('result_timeout_sec')
    cancel_on_timeout = LaunchConfiguration('cancel_on_timeout')
    send_goals = LaunchConfiguration('send_goals')
    require_non_empty = LaunchConfiguration('require_non_empty_trajectory')
    require_panda_joints = LaunchConfiguration('require_panda_joints')
    expected_joint_prefix = LaunchConfiguration('expected_joint_prefix')
    simulated_only = LaunchConfiguration('simulated_execution_only')
    use_planning_scene_audit = LaunchConfiguration(
        'use_planning_scene_audit'
    )
    launch_reachable_sequence = LaunchConfiguration(
        'launch_reachable_sequence'
    )

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
            'controller_action_name',
            default_value=(
                '/panda_arm_controller/follow_joint_trajectory'
            ),
            description='Simulated FollowJointTrajectory action server.',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/assembly_ros2_control_execution_status',
            description='Aggregate ros2_control bridge status.',
        ),
        DeclareLaunchArgument(
            'success_topic',
            default_value='/assembly_ros2_control_execution_success',
            description='Aggregate ros2_control bridge success.',
        ),
        DeclareLaunchArgument(
            'duration_topic',
            default_value='/assembly_ros2_control_execution_duration_ms',
            description='Aggregate ros2_control bridge duration.',
        ),
        DeclareLaunchArgument(
            'stage_status_topic',
            default_value='/assembly_ros2_control_execution_stage_status',
            description='Per-stage ros2_control bridge status.',
        ),
        DeclareLaunchArgument(
            'wait_for_controller_sec',
            default_value='5.0',
            description='Maximum controller action discovery wait.',
        ),
        DeclareLaunchArgument(
            'result_timeout_sec',
            default_value='10.0',
            description='Maximum wait for each accepted controller result.',
        ),
        DeclareLaunchArgument(
            'cancel_on_timeout',
            default_value='true',
            description='Request action cancellation after result timeout.',
        ),
        DeclareLaunchArgument(
            'send_goals',
            default_value='true',
            description='Send validated trajectories to the action server.',
        ),
        DeclareLaunchArgument(
            'require_non_empty_trajectory',
            default_value='true',
            description='Reject trajectories without joint names or points.',
        ),
        DeclareLaunchArgument(
            'require_panda_joints',
            default_value='true',
            description='Require an expected Panda joint in each trajectory.',
        ),
        DeclareLaunchArgument(
            'expected_joint_prefix',
            default_value='panda_joint',
            description='Required joint prefix when validation is enabled.',
        ),
        DeclareLaunchArgument(
            'simulated_execution_only',
            default_value='true',
            description='Safety invariant; false is unsupported.',
        ),
        DeclareLaunchArgument(
            'use_planning_scene_audit',
            default_value='true',
            description='Include the read-only PlanningScene audit.',
        ),
        DeclareLaunchArgument(
            'launch_reachable_sequence',
            default_value='true',
            description=(
                'Launch the existing MoveIt demo-backed reachable planner. '
                'Set false when another planning stack is already running.'
            ),
        ),
        LogInfo(
            msg='Launching the simulator-only ros2_control execution bridge. '
            'It connects to an existing controller when available and does '
            'not start Gazebo or support real hardware.'
        ),
        Node(
            package='adaptive_assembly_execution',
            executable='ros2_control_sequence_executor_node',
            name='ros2_control_sequence_executor_node',
            output='screen',
            parameters=[{
                'pre_grasp_trajectory_topic': pre_grasp_topic,
                'assembly_trajectory_topic': assembly_topic,
                'controller_action_name': controller,
                'status_topic': status_topic,
                'success_topic': success_topic,
                'duration_topic': duration_topic,
                'stage_status_topic': stage_status_topic,
                'wait_for_controller_sec': ParameterValue(
                    wait_for_controller, value_type=float
                ),
                'result_timeout_sec': ParameterValue(
                    result_timeout, value_type=float
                ),
                'cancel_on_timeout': ParameterValue(
                    cancel_on_timeout, value_type=bool
                ),
                'send_goals': ParameterValue(send_goals, value_type=bool),
                'require_non_empty_trajectory': ParameterValue(
                    require_non_empty, value_type=bool
                ),
                'require_panda_joints': ParameterValue(
                    require_panda_joints, value_type=bool
                ),
                'expected_joint_prefix': expected_joint_prefix,
                'simulated_execution_only': ParameterValue(
                    simulated_only, value_type=bool
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(reachable_sequence),
            condition=IfCondition(launch_reachable_sequence),
            launch_arguments={
                'pre_grasp_trajectory_topic': pre_grasp_topic,
                'assembly_trajectory_topic': assembly_topic,
                'use_planning_scene_audit': use_planning_scene_audit,
            }.items(),
        ),
    ])
