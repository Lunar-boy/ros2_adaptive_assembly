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
    planner_id = LaunchConfiguration('planner_id')
    num_planning_attempts = LaunchConfiguration('num_planning_attempts')
    max_velocity_scaling_factor = LaunchConfiguration('max_velocity_scaling_factor')
    max_acceleration_scaling_factor = LaunchConfiguration(
        'max_acceleration_scaling_factor'
    )
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
        LogInfo(
            msg='Launching adaptive assembly Panda planning demo: fake '
            'perception, task pose generation, standard Panda MoveIt2 demo, '
            'static PlanningScene collision objects, Panda pre-grasp pose '
            'adapter, dynamic target collision object, and plan-only planning '
            'bridge. use_dynamic_target_scene controls whether the dynamic '
            'target object is launched. Planner settings can be overridden '
            'with planner_id, num_planning_attempts, '
            'max_velocity_scaling_factor, and '
            'max_acceleration_scaling_factor. Execution is disabled.'
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
            PythonLaunchDescriptionSource(pre_grasp_planning_launch),
            launch_arguments={
                'planner_id': planner_id,
                'num_planning_attempts': num_planning_attempts,
                'max_velocity_scaling_factor': max_velocity_scaling_factor,
                'max_acceleration_scaling_factor': max_acceleration_scaling_factor,
            }.items(),
        ),
    ])
