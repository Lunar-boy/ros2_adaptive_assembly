"""Compose Gazebo Panda execution with logical and kinematic grasp state."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch full simulation, logical lifecycle, and Gazebo attachment."""
    full_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'), 'launch',
        'adaptive_assembly_full_gazebo_execution_demo.launch.py',
    ])
    lifecycle = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_manipulation'), 'launch',
        'logical_grasp_lifecycle.launch.py',
    ])
    attachment = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_manipulation'), 'launch',
        'gazebo_attach_detach.launch.py',
    ])
    enable_calls = LaunchConfiguration('enable_service_calls')
    params_file = LaunchConfiguration('params_file')
    require_grasp = LaunchConfiguration('require_grasp_trajectory')
    attach_stage = LaunchConfiguration('attach_stage')
    offset_x = LaunchConfiguration('attached_object_offset_x')
    offset_y = LaunchConfiguration('attached_object_offset_y')
    offset_z = LaunchConfiguration('attached_object_offset_z')
    use_hand_orientation = LaunchConfiguration(
        'attached_object_use_hand_orientation'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('adaptive_assembly_bringup'), 'config',
                'adaptive_assembly_sequence_reachable_params.yaml',
            ]),
            description='Parameter YAML for perception and task nodes.',
        ),
        DeclareLaunchArgument(
            'require_grasp_trajectory', default_value='false',
            description='Plan and execute the intermediate grasp stage.',
        ),
        DeclareLaunchArgument(
            'attach_stage', default_value='pre_grasp',
            description='Successful execution stage that triggers attachment.',
        ),
        DeclareLaunchArgument(
            'enable_service_calls', default_value='true',
            description=(
                'Call Gazebo set_pose; false enables fixture validation.'
            ),
        ),
        DeclareLaunchArgument('attached_object_offset_x', default_value='0.0'),
        DeclareLaunchArgument('attached_object_offset_y', default_value='0.0'),
        DeclareLaunchArgument('attached_object_offset_z', default_value='0.0'),
        DeclareLaunchArgument(
            'attached_object_use_hand_orientation', default_value='true'
        ),
        LogInfo(msg=(
            'Launching simulator-only Gazebo Panda execution with logical '
            'grasp lifecycle and kinematic object attachment.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_demo),
            launch_arguments={
                'params_file': params_file,
                'require_grasp_trajectory': require_grasp,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lifecycle),
            launch_arguments={'attach_stage': attach_stage}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(attachment),
            launch_arguments={
                'enable_service_calls': enable_calls,
                'attached_object_offset_x': offset_x,
                'attached_object_offset_y': offset_y,
                'attached_object_offset_z': offset_z,
                'attached_object_use_hand_orientation': use_hand_orientation,
            }.items(),
        ),
    ])
