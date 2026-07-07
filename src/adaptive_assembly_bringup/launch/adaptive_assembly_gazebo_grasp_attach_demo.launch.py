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
    target_sync = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'), 'launch',
        'gazebo_target_pose_sync.launch.py',
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
    require_place = LaunchConfiguration('require_place_sequence')
    require_target_sync = LaunchConfiguration('require_target_sync_success')
    target_sync_topic = LaunchConfiguration('target_sync_status_topic')
    target_sync_timeout = LaunchConfiguration('target_sync_timeout_sec')
    launch_simulation = LaunchConfiguration('launch_simulation')
    attach_stage = LaunchConfiguration('attach_stage')
    release_stage = LaunchConfiguration('release_stage')
    offset_x = LaunchConfiguration('attached_object_offset_x')
    offset_y = LaunchConfiguration('attached_object_offset_y')
    offset_z = LaunchConfiguration('attached_object_offset_z')
    use_hand_orientation = LaunchConfiguration(
        'attached_object_use_hand_orientation'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_simulation', default_value='true',
            description='Start Gazebo; false when an outer launch owns it.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('adaptive_assembly_bringup'), 'config',
                'adaptive_assembly_fixed_socket_params.yaml',
            ]),
            description='Parameter YAML for perception and task nodes.',
        ),
        DeclareLaunchArgument(
            'require_grasp_trajectory', default_value='true',
            description='Plan and execute the intermediate grasp stage.',
        ),
        DeclareLaunchArgument('require_place_sequence', default_value='true'),
        DeclareLaunchArgument(
            'require_target_sync_success', default_value='true',
            description='Optionally gate initial execution on target sync.',
        ),
        DeclareLaunchArgument(
            'target_sync_status_topic',
            default_value='/gazebo_target_sync_status',
        ),
        DeclareLaunchArgument(
            'target_sync_timeout_sec', default_value='10.0',
        ),
        DeclareLaunchArgument(
            'attach_stage', default_value='grasp',
            description='Successful execution stage that triggers attachment.',
        ),
        DeclareLaunchArgument('release_stage', default_value='place'),
        DeclareLaunchArgument(
            'enable_service_calls', default_value='true',
            description=(
                'Call Gazebo set_pose; false enables fixture validation.'
            ),
        ),
        DeclareLaunchArgument('attached_object_offset_x', default_value='0.0'),
        DeclareLaunchArgument('attached_object_offset_y', default_value='0.0'),
        DeclareLaunchArgument('attached_object_offset_z', default_value='0.10'),
        DeclareLaunchArgument(
            'attached_object_use_hand_orientation', default_value='true'
        ),
        LogInfo(msg=(
            'Launching simulator-only Gazebo Panda execution with logical '
            'grasp lifecycle and kinematic object attachment.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(target_sync),
            launch_arguments={
                'enable_service_calls': enable_calls,
                'status_topic': target_sync_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_demo),
            launch_arguments={
                'params_file': params_file,
                'require_grasp_trajectory': require_grasp,
                'require_place_sequence': require_place,
                'require_target_sync_success': require_target_sync,
                'target_sync_status_topic': target_sync_topic,
                'target_sync_timeout_sec': target_sync_timeout,
                'launch_simulation': launch_simulation,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lifecycle),
            launch_arguments={
                'attach_stage': attach_stage,
                'release_stage': release_stage,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(attachment),
            launch_arguments={
                'enable_service_calls': enable_calls,
                'status_topic': '/gazebo_attach_detach_status',
                'attached_object_offset_x': offset_x,
                'attached_object_offset_y': offset_y,
                'attached_object_offset_z': offset_z,
                'attached_object_use_hand_orientation': use_hand_orientation,
            }.items(),
        ),
    ])
