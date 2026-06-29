"""Launch the Panda demo with plan-only pre-grasp and assembly sequencing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose the existing demo with assembly adaptation and sequencing."""
    planner_id = LaunchConfiguration('planner_id')
    num_planning_attempts = LaunchConfiguration('num_planning_attempts')
    planning_time_sec = LaunchConfiguration('planning_time_sec')
    position_tolerance = LaunchConfiguration('position_tolerance')
    orientation_tolerance = LaunchConfiguration('orientation_tolerance')
    publish_diagnostics = LaunchConfiguration('publish_diagnostics')
    start_state_mode = LaunchConfiguration('start_state_mode')

    panda_planning_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_planning_demo.launch.py',
    ])
    assembly_adapter = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'panda_assembly_pose_adapter.launch.py',
    ])
    sequence_planner = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'assembly_sequence_planning.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_id',
            default_value='',
            description='Optional MoveIt2 planner ID. Empty uses the default.',
        ),
        DeclareLaunchArgument(
            'num_planning_attempts',
            default_value='1',
            description='Number of MoveIt2 planning attempts per stage.',
        ),
        DeclareLaunchArgument(
            'planning_time_sec',
            default_value='5.0',
            description='MoveIt2 planning time limit for each sequence stage.',
        ),
        DeclareLaunchArgument(
            'position_tolerance',
            default_value='0.01',
            description='Position goal tolerance for each sequence stage.',
        ),
        DeclareLaunchArgument(
            'orientation_tolerance',
            default_value='0.10',
            description='Orientation goal tolerance for each sequence stage.',
        ),
        DeclareLaunchArgument(
            'publish_diagnostics',
            default_value='true',
            description='Publish sequence status and duration diagnostics.',
        ),
        DeclareLaunchArgument(
            'start_state_mode',
            default_value='current',
            description=(
                "Pre-grasp start state source: 'current' or deterministic 'fixed'."
            ),
        ),
        LogInfo(
            msg='Launching the existing Panda plan-only demo plus the Panda '
            'assembly pose adapter and two-stage pre-grasp/assembly sequence '
            'planner. Both stages are planned only; execution is disabled.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_planning_demo),
            launch_arguments={
                'planner_id': planner_id,
                'num_planning_attempts': num_planning_attempts,
                'use_pre_grasp_planning': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(assembly_adapter),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sequence_planner),
            launch_arguments={
                'planner_id': planner_id,
                'num_planning_attempts': num_planning_attempts,
                'planning_time_sec': planning_time_sec,
                'position_tolerance': position_tolerance,
                'orientation_tolerance': orientation_tolerance,
                'publish_diagnostics': publish_diagnostics,
                'start_state_mode': start_state_mode,
            }.items(),
        ),
    ])
