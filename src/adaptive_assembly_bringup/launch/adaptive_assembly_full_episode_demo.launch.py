"""Compose the complete simulator-only adaptive assembly episode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch execution, grasping, pose observation, evaluation, and supervision."""
    grasp_attach_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_gazebo_grasp_attach_demo.launch.py',
    ])
    pose_observer = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'launch',
        'gazebo_entity_pose_observer.launch.py',
    ])
    episode_supervisor = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_episode'),
        'launch',
        'assembly_episode_supervisor.launch.py',
    ])

    arguments = {
        'enable_service_calls': 'true',
        'target_entity_name': 'target_object',
        'world_frame': 'world',
        'achieved_pose_topic': '/gazebo_target_object_pose',
        'position_tolerance_mm': '5.0',
        'orientation_tolerance_deg': '5.0',
        'require_execution_success': 'true',
        'episode_timeout_sec': '120.0',
        'simulated_only': 'true',
    }
    config = {name: LaunchConfiguration(name) for name in arguments}

    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, default in arguments.items()
        ],
        LogInfo(msg=(
            'Launching complete simulator-only adaptive assembly episode: '
            'Gazebo execution + logical grasp + kinematic attachment + Gazebo '
            'achieved pose + contact-lite insertion + passive episode supervisor.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(grasp_attach_demo),
            launch_arguments={
                'enable_service_calls': config['enable_service_calls'],
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pose_observer),
            launch_arguments={
                'target_entity_name': config['target_entity_name'],
                'world_frame': config['world_frame'],
                'output_pose_topic': config['achieved_pose_topic'],
                'simulated_only': config['simulated_only'],
            }.items(),
        ),
        Node(
            package='adaptive_assembly_benchmark',
            executable='contact_lite_insertion_evaluator_node',
            name='contact_lite_insertion_evaluator_node',
            output='screen',
            parameters=[{
                'target_pose_topic': '/panda_assembly_pose',
                'achieved_pose_topic': config['achieved_pose_topic'],
                'achieved_pose_source': 'gazebo_entity_pose_observer',
                'require_execution_success': config['require_execution_success'],
                'position_tolerance_mm': config['position_tolerance_mm'],
                'orientation_tolerance_deg': config['orientation_tolerance_deg'],
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(episode_supervisor),
            launch_arguments={
                'episode_timeout_sec': config['episode_timeout_sec'],
                'simulated_only': config['simulated_only'],
            }.items(),
        ),
    ])
