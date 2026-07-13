"""Launch a deterministic known-reachable plan-only assembly sequence."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start fixed-target, fixed-start two-stage planning without execution."""
    sequence_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_demo.launch.py',
    ])
    reachable_params = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_sequence_reachable_params.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    pre_grasp_trajectory_topic = LaunchConfiguration(
        'pre_grasp_trajectory_topic'
    )
    use_standard_panda_demo = LaunchConfiguration('use_standard_panda_demo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_fake_object_pose_node = LaunchConfiguration(
        'launch_fake_object_pose_node'
    )
    assembly_trajectory_topic = LaunchConfiguration(
        'assembly_trajectory_topic'
    )
    use_planning_scene_audit = LaunchConfiguration(
        'use_planning_scene_audit'
    )
    static_planning_scene_params_file = LaunchConfiguration(
        'static_planning_scene_params_file'
    )
    stage_names = LaunchConfiguration('stage_names')
    end_effector_link = LaunchConfiguration('end_effector_link')
    position_tolerance = LaunchConfiguration('position_tolerance')
    orientation_tolerance = LaunchConfiguration('orientation_tolerance')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=reachable_params,
            description=(
                'Parameter YAML for the deterministic reachable task and '
                'perception profile.'
            ),
        ),
        DeclareLaunchArgument(
            'pre_grasp_trajectory_topic',
            default_value='/pre_grasp_trajectory',
            description='Topic for successful pre-grasp trajectories.',
        ),
        DeclareLaunchArgument(
            'assembly_trajectory_topic',
            default_value='/assembly_trajectory',
            description='Topic for successful assembly trajectories.',
        ),
        DeclareLaunchArgument(
            'use_standard_panda_demo',
            default_value='true',
            description=(
                'Whether the nested Panda planning demo uses the standard '
                'fake-control MoveIt demo.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when an upstream simulator provides /clock.',
        ),
        DeclareLaunchArgument(
            'launch_fake_object_pose_node',
            default_value='true',
            description=(
                'Whether the adaptive pipeline starts fake perception.'
            ),
        ),
        DeclareLaunchArgument(
            'use_planning_scene_audit',
            default_value='true',
            description='Whether to include the read-only scene audit.',
        ),
        DeclareLaunchArgument(
            'stage_names',
            default_value='pre_grasp,assembly',
            description='Comma-separated ordered sequence stages.',
        ),
        DeclareLaunchArgument(
            'end_effector_link',
            default_value='panda_link8',
            description='Robot link whose pose each stage target specifies.',
        ),
        DeclareLaunchArgument(
            'position_tolerance',
            default_value='0.01',
            description='MoveIt position tolerance for each sequence stage.',
        ),
        DeclareLaunchArgument(
            'orientation_tolerance',
            default_value='0.10',
            description='MoveIt orientation tolerance for each sequence stage.',
        ),
        DeclareLaunchArgument(
            'static_planning_scene_params_file',
            default_value='',
            description=(
                'Optional static PlanningScene parameter YAML forwarded to '
                'the sequence planning demo.'
            ),
        ),
        LogInfo(
            msg='Launching the deterministic known-reachable Panda assembly '
            'sequence profile. Both stages are planned only; execution is '
            'disabled.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sequence_demo),
            launch_arguments={
                'params_file': params_file,
                'use_standard_panda_demo': use_standard_panda_demo,
                'use_sim_time': use_sim_time,
                'launch_fake_object_pose_node': launch_fake_object_pose_node,
                'use_dynamic_target_scene': 'false',
                'planning_scene_audit_expected_object_ids': (
                    'work_table,target_support'
                ),
                'start_state_mode': 'fixed',
                'planning_time_sec': '5.0',
                'num_planning_attempts': '1',
                'position_tolerance': position_tolerance,
                'orientation_tolerance': orientation_tolerance,
                'stage_names': stage_names,
                'end_effector_link': end_effector_link,
                'pre_grasp_trajectory_topic': pre_grasp_trajectory_topic,
                'assembly_trajectory_topic': assembly_trajectory_topic,
                'use_planning_scene_audit': use_planning_scene_audit,
                'static_planning_scene_params_file': (
                    static_planning_scene_params_file
                ),
            }.items(),
        ),
    ])
