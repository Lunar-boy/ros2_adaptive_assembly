"""Launch the Panda MoveIt2 demo with adapted plan-only pre-grasp planning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start adaptive assembly, Panda demo, adapter, and planning bridge."""
    panda_demo_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_demo.launch.py',
    ])
    default_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_params.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    use_dynamic_target_scene = LaunchConfiguration('use_dynamic_target_scene')
    use_planning_scene_audit = LaunchConfiguration('use_planning_scene_audit')
    use_pre_grasp_planning = LaunchConfiguration('use_pre_grasp_planning')
    planner_id = LaunchConfiguration('planner_id')
    num_planning_attempts = LaunchConfiguration('num_planning_attempts')
    max_velocity_scaling_factor = LaunchConfiguration('max_velocity_scaling_factor')
    max_acceleration_scaling_factor = LaunchConfiguration(
        'max_acceleration_scaling_factor'
    )
    enable_request_guard = LaunchConfiguration('enable_request_guard')
    required_frame_id = LaunchConfiguration('required_frame_id')
    workspace_min_x = LaunchConfiguration('workspace_min_x')
    workspace_max_x = LaunchConfiguration('workspace_max_x')
    workspace_min_y = LaunchConfiguration('workspace_min_y')
    workspace_max_y = LaunchConfiguration('workspace_max_y')
    workspace_min_z = LaunchConfiguration('workspace_min_z')
    workspace_max_z = LaunchConfiguration('workspace_max_z')
    min_quaternion_norm = LaunchConfiguration('min_quaternion_norm')
    static_planning_scene_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'static_planning_scene.launch.py',
    ])
    panda_pose_adapter_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'panda_pre_grasp_pose_adapter.launch.py',
    ])
    dynamic_target_scene_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'dynamic_target_scene.launch.py',
    ])
    planning_scene_audit_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'planning_scene_audit.launch.py',
    ])
    pre_grasp_planning_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'pre_grasp_planning.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Parameter YAML for adaptive assembly pipeline nodes.',
        ),
        DeclareLaunchArgument(
            'use_dynamic_target_scene',
            default_value='true',
            description=(
                'Whether to include the dynamic target PlanningScene object.'
            ),
        ),
        DeclareLaunchArgument(
            'use_planning_scene_audit',
            default_value='true',
            description='Whether to include the read-only PlanningScene audit node.',
        ),
        DeclareLaunchArgument(
            'use_pre_grasp_planning',
            default_value='true',
            description='Whether to launch the single-pose pre-grasp planner.',
        ),
        DeclareLaunchArgument(
            'planner_id',
            default_value='',
            description='Optional MoveIt2 planner ID. Empty uses the default.',
        ),
        DeclareLaunchArgument(
            'num_planning_attempts',
            default_value='1',
            description='Number of MoveIt2 planning attempts.',
        ),
        DeclareLaunchArgument(
            'max_velocity_scaling_factor',
            default_value='1.0',
            description='MoveIt2 velocity scaling factor in (0.0, 1.0].',
        ),
        DeclareLaunchArgument(
            'max_acceleration_scaling_factor',
            default_value='1.0',
            description='MoveIt2 acceleration scaling factor in (0.0, 1.0].',
        ),
        DeclareLaunchArgument(
            'enable_request_guard',
            default_value='false',
            description='Enable pre-MoveIt2 planning request safety checks.',
        ),
        DeclareLaunchArgument(
            'required_frame_id',
            default_value='',
            description='Required input pose frame when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_min_x',
            default_value='-10.0',
            description='Minimum allowed x when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_max_x',
            default_value='10.0',
            description='Maximum allowed x when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_min_y',
            default_value='-10.0',
            description='Minimum allowed y when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_max_y',
            default_value='10.0',
            description='Maximum allowed y when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_min_z',
            default_value='-10.0',
            description='Minimum allowed z when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_max_z',
            default_value='10.0',
            description='Maximum allowed z when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'min_quaternion_norm',
            default_value='1e-6',
            description='Minimum allowed quaternion norm when guard is enabled.',
        ),
        LogInfo(
            msg='Launching adaptive assembly Panda planning demo: fake '
            'perception, task pose generation, standard Panda MoveIt2 demo, '
            'static PlanningScene collision objects, Panda pre-grasp pose '
            'adapter, dynamic target collision object, read-only PlanningScene '
            'audit, and plan-only planning bridge. use_dynamic_target_scene '
            'controls whether the dynamic target object is launched. '
            'use_planning_scene_audit controls whether the audit node is '
            'launched. use_pre_grasp_planning controls '
            'the existing single-pose planning bridge. Planner settings can be '
            'overridden with planner_id, '
            'num_planning_attempts, '
            'max_velocity_scaling_factor, and '
            'max_acceleration_scaling_factor. enable_request_guard can enable '
            'pre-MoveIt2 request checks. Execution is disabled.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_demo_launch),
            launch_arguments={'params_file': params_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_planning_scene_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_pose_adapter_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dynamic_target_scene_launch),
            condition=IfCondition(use_dynamic_target_scene),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_scene_audit_launch),
            condition=IfCondition(use_planning_scene_audit),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pre_grasp_planning_launch),
            condition=IfCondition(use_pre_grasp_planning),
            launch_arguments={
                'planner_id': planner_id,
                'num_planning_attempts': num_planning_attempts,
                'max_velocity_scaling_factor': max_velocity_scaling_factor,
                'max_acceleration_scaling_factor': max_acceleration_scaling_factor,
                'enable_request_guard': enable_request_guard,
                'required_frame_id': required_frame_id,
                'workspace_min_x': workspace_min_x,
                'workspace_max_x': workspace_max_x,
                'workspace_min_y': workspace_min_y,
                'workspace_max_y': workspace_max_y,
                'workspace_min_z': workspace_min_z,
                'workspace_max_z': workspace_max_z,
                'min_quaternion_norm': min_quaternion_norm,
            }.items(),
        ),
    ])
