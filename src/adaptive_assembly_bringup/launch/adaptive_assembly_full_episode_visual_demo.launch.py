"""Launch one visually coherent simulator-only assembly episode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose fixed source/place execution, synchronization, and evaluation."""
    bringup = FindPackageShare('adaptive_assembly_bringup')
    params_file = PathJoinSubstitution([
        bringup, 'config', 'adaptive_assembly_visual_single_trial_params.yaml'
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_service_calls', default_value='true',
            description='Enable simulator set-pose service calls.',
        ),
        LogInfo(msg=(
            'Launching visual single-trial episode with distinct deterministic '
            'source and fixed socket poses; simulator and logical gripper only.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(grasp_demo),
            launch_arguments={
                'enable_service_calls': enable_calls,
                'params_file': params_file,
                'require_grasp_trajectory': 'true',
                'attach_stage': 'grasp',
                # Local panda_hand +Z keeps the cylinder visibly at the tool.
                'attached_object_offset_z': '0.10',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(target_sync),
            launch_arguments={
                'enable_service_calls': enable_calls,
                'target_entity_name': 'target_object',
                'world_frame': 'world',
                'control_owner_topic': '/target_object_control_owner',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pose_observer),
            launch_arguments={
                'target_entity_name': 'target_object',
                'world_frame': 'world',
                'output_pose_topic': '/gazebo_target_object_pose',
                'simulated_only': 'true',
            }.items(),
        ),
        Node(
            package='adaptive_assembly_benchmark',
            executable='contact_lite_insertion_evaluator_node',
            name='contact_lite_insertion_evaluator_node',
            output='screen',
            parameters=[{
                'target_pose_topic': '/panda_assembly_pose',
                'achieved_pose_topic': '/gazebo_target_object_pose',
                'achieved_pose_source': 'gazebo_entity_pose_observer',
                'require_execution_success': True,
                'position_tolerance_mm': 5.0,
                'orientation_tolerance_deg': 5.0,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(supervisor),
            launch_arguments={
                'episode_timeout_sec': '120.0',
                'simulated_only': 'true',
            }.items(),
        ),
    ])
