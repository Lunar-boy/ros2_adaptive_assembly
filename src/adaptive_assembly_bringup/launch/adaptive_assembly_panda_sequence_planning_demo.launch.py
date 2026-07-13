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
    default_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_params.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    use_standard_panda_demo = LaunchConfiguration('use_standard_panda_demo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_fake_object_pose_node = LaunchConfiguration(
        'launch_fake_object_pose_node'
    )
    use_dynamic_target_scene = LaunchConfiguration('use_dynamic_target_scene')
    use_planning_scene_audit = LaunchConfiguration('use_planning_scene_audit')
    planning_scene_audit_expected_object_ids = LaunchConfiguration(
        'planning_scene_audit_expected_object_ids'
    )
    planner_id = LaunchConfiguration('planner_id')
    num_planning_attempts = LaunchConfiguration('num_planning_attempts')
    planning_time_sec = LaunchConfiguration('planning_time_sec')
    position_tolerance = LaunchConfiguration('position_tolerance')
    orientation_tolerance = LaunchConfiguration('orientation_tolerance')
    publish_diagnostics = LaunchConfiguration('publish_diagnostics')
    publish_trajectories = LaunchConfiguration('publish_trajectories')
    pre_grasp_trajectory_topic = LaunchConfiguration(
        'pre_grasp_trajectory_topic'
    )
    assembly_trajectory_topic = LaunchConfiguration('assembly_trajectory_topic')
    trajectory_status_topic = LaunchConfiguration('trajectory_status_topic')
    start_state_mode = LaunchConfiguration('start_state_mode')
    stage_names = LaunchConfiguration('stage_names')
    end_effector_link = LaunchConfiguration('end_effector_link')
    static_planning_scene_params_file = LaunchConfiguration(
        'static_planning_scene_params_file'
    )

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
    grasp_adapter = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'panda_grasp_pose_adapter.launch.py',
    ])

    lift_adapter = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'panda_lift_pose_adapter.launch.py',
    ])

    pre_place_adapter = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'panda_pre_place_pose_adapter.launch.py',
    ])

    place_adapter = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'panda_place_pose_adapter.launch.py',
    ])

    retreat_adapter = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'panda_retreat_pose_adapter.launch.py',
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
            description='Whether to include the dynamic target collision object.',
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
            description='Whether to include the read-only PlanningScene audit.',
        ),
        DeclareLaunchArgument(
            'planning_scene_audit_expected_object_ids',
            default_value='work_table,target_support,target_object_dynamic',
            description='Comma-separated object IDs expected by the audit.',
        ),
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
            'publish_trajectories',
            default_value='true',
            description='Publish successful plans as RobotTrajectory messages.',
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
            'trajectory_status_topic',
            default_value='/assembly_sequence_trajectory_status',
            description='Topic for trajectory publication status.',
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
            'start_state_mode',
            default_value='current',
            description=(
                "Pre-grasp start state source: 'current' or deterministic 'fixed'."
            ),
        ),
        DeclareLaunchArgument(
            'static_planning_scene_params_file',
            default_value='',
            description=(
                'Optional static PlanningScene parameter YAML forwarded to '
                'the nested Panda planning demo.'
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
                'params_file': params_file,
                'use_standard_panda_demo': use_standard_panda_demo,
                'use_sim_time': use_sim_time,
                'launch_fake_object_pose_node': launch_fake_object_pose_node,
                'use_dynamic_target_scene': use_dynamic_target_scene,
                'use_planning_scene_audit': use_planning_scene_audit,
                'planning_scene_audit_expected_object_ids': (
                    planning_scene_audit_expected_object_ids
                ),
                'planner_id': planner_id,
                'num_planning_attempts': num_planning_attempts,
                'use_pre_grasp_planning': 'false',
                'static_planning_scene_params_file': (
                    static_planning_scene_params_file
                ),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(assembly_adapter),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
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
                'publish_trajectories': publish_trajectories,
                'pre_grasp_trajectory_topic': pre_grasp_trajectory_topic,
                'assembly_trajectory_topic': assembly_trajectory_topic,
                'trajectory_status_topic': trajectory_status_topic,
                'stage_names': stage_names,
                'end_effector_link': end_effector_link,
                'start_state_mode': start_state_mode,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(grasp_adapter),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lift_adapter),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pre_place_adapter),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(place_adapter),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(retreat_adapter),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
