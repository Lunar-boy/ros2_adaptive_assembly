"""Launch one controller-gated, visually coherent simulator episode."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start Gazebo first and release all consumers on controller readiness."""
    bringup = FindPackageShare('adaptive_assembly_bringup')
    params_file = PathJoinSubstitution([
        bringup, 'config', 'adaptive_assembly_visual_single_trial_params.yaml'
    ])
    simulation = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'), 'launch',
        'adaptive_assembly_panda_gazebo.launch.py'
    ])
    grasp_demo = PathJoinSubstitution([
        bringup, 'launch',
        'adaptive_assembly_gazebo_grasp_attach_demo.launch.py'
    ])
    target_sync = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'), 'launch',
        'gazebo_target_pose_sync.launch.py'
    ])
    pose_observer = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'), 'launch',
        'gazebo_entity_pose_observer.launch.py'
    ])
    supervisor = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_episode'), 'launch',
        'assembly_episode_supervisor.launch.py'
    ])
    enable_calls = LaunchConfiguration('enable_service_calls')
    require_ready = LaunchConfiguration('require_gazebo_controller_ready')
    ready_topic = LaunchConfiguration('gazebo_controller_ready_status_topic')
    ready_timeout = LaunchConfiguration('gazebo_controller_ready_timeout_sec')

    def downstream_actions(condition=None):
        """Build fresh downstream actions for gated and opt-out paths."""
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(grasp_demo),
                launch_arguments={
                    'enable_service_calls': enable_calls,
                    'params_file': params_file,
                    'launch_simulation': 'false',
                    'require_grasp_trajectory': 'true',
                    'require_place_sequence': 'true',
                    'require_target_sync_success': 'true',
                    'target_sync_status_topic': '/gazebo_target_sync_status',
                    'target_sync_timeout_sec': '10.0',
                    'attach_stage': 'grasp',
                    'release_stage': 'place',
                    'attached_object_offset_z': '0.10',
                }.items(),
                condition=condition,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(target_sync),
                launch_arguments={
                    'enable_service_calls': enable_calls,
                    'target_entity_name': 'target_object',
                    'world_frame': 'world',
                    'control_owner_topic': '/target_object_control_owner',
                }.items(),
                condition=condition,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pose_observer),
                launch_arguments={
                    'target_entity_name': 'target_object',
                    'world_frame': 'world',
                    'output_pose_topic': '/gazebo_target_object_pose',
                    'simulated_only': 'true',
                }.items(),
                condition=condition,
            ),
            Node(
                package='adaptive_assembly_benchmark',
                executable='contact_lite_insertion_evaluator_node',
                name='contact_lite_insertion_evaluator_node',
                output='screen',
                parameters=[{
                    'target_pose_topic': '/object_place_pose',
                    'achieved_pose_topic': '/gazebo_target_object_pose',
                    'achieved_pose_source': 'gazebo_entity_pose_observer',
                    'require_execution_success': True,
                    'position_tolerance_mm': 5.0,
                    'orientation_tolerance_deg': 5.0,
                }],
                condition=condition,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(supervisor),
                launch_arguments={
                    'episode_timeout_sec': '120.0',
                    'simulated_only': 'true',
                }.items(),
                condition=condition,
            ),
        ]

    readiness_gate = Node(
        package='adaptive_assembly_execution',
        executable='wait_for_gazebo_controller_ready_node',
        name='wait_for_gazebo_controller_ready_node',
        output='screen',
        parameters=[{
            'status_topic': ready_topic,
            'timeout_sec': ParameterValue(ready_timeout, value_type=float),
            'simulated_only': True,
        }],
        condition=IfCondition(require_ready),
    )
    readiness_waiter = Node(
        package='adaptive_assembly_execution',
        executable='wait_for_gazebo_controller_ready_status_node',
        name='wait_for_gazebo_controller_ready_status_node',
        output='screen',
        parameters=[{
            'status_topic': ready_topic,
            'timeout_sec': ParameterValue(
                PythonExpression([ready_timeout, ' + 5.0']),
                value_type=float,
            ),
        }],
        condition=IfCondition(require_ready),
    )

    def start_after_readiness(event, context):
        del context
        if event.returncode != 0:
            return [LogInfo(msg=(
                'Gazebo controller readiness failed; planning, execution, '
                'attachment, evaluation, and supervision remain stopped.'
            ))]
        return downstream_actions()

    return LaunchDescription([
        DeclareLaunchArgument('enable_service_calls', default_value='true'),
        DeclareLaunchArgument(
            'require_gazebo_controller_ready', default_value='true',
            description='Gate all visual episode consumers on controller readiness.',
        ),
        DeclareLaunchArgument(
            'gazebo_controller_ready_status_topic',
            default_value='/gazebo_controller_ready_status',
        ),
        DeclareLaunchArgument(
            'gazebo_controller_ready_timeout_sec', default_value='60.0',
        ),
        LogInfo(msg=(
            'Launching simulator-only visual episode: Gazebo first, then '
            'controller-gated planning, execution, target sync, and supervision.'
        )),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(simulation)),
        readiness_gate,
        readiness_waiter,
        RegisterEventHandler(OnProcessExit(
            target_action=readiness_waiter,
            on_exit=start_after_readiness,
        )),
        *downstream_actions(UnlessCondition(require_ready)),
    ])
