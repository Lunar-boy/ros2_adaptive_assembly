"""Launch the plan-only Panda assembly sequence planning node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start the sequence planner without robot bringup or execution."""
    planner_id = LaunchConfiguration('planner_id')
    num_planning_attempts = LaunchConfiguration('num_planning_attempts')
    planning_time_sec = LaunchConfiguration('planning_time_sec')
    position_tolerance = LaunchConfiguration('position_tolerance')
    orientation_tolerance = LaunchConfiguration('orientation_tolerance')
    publish_diagnostics = LaunchConfiguration('publish_diagnostics')
    publish_trajectories = LaunchConfiguration('publish_trajectories')
    stage_names = LaunchConfiguration('stage_names')
    pre_grasp_trajectory_topic = LaunchConfiguration(
        'pre_grasp_trajectory_topic'
    )
    grasp_trajectory_topic = LaunchConfiguration('grasp_trajectory_topic')
    assembly_trajectory_topic = LaunchConfiguration('assembly_trajectory_topic')
    lift_trajectory_topic = LaunchConfiguration('lift_trajectory_topic')
    require_grasp_pose = LaunchConfiguration('require_grasp_pose')
    require_place_sequence = LaunchConfiguration('require_place_sequence')
    trajectory_status_topic = LaunchConfiguration('trajectory_status_topic')
    start_state_mode = LaunchConfiguration('start_state_mode')
    end_effector_link = LaunchConfiguration('end_effector_link')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stage_names',
            default_value='pre_grasp,assembly',
            description=(
                'Comma-separated ordered planning stages. This explicit value '
                'takes precedence over deprecated compatibility switches.'
            ),
        ),
        DeclareLaunchArgument(
            'planner_id',
            default_value='',
            description='Optional MoveIt2 planner ID. Empty uses the default.',
        ),
        DeclareLaunchArgument(
            'lift_trajectory_topic',
            default_value='/lift_trajectory',
            description='Topic for successful lift trajectories.',
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
            description='Position goal tolerance for each stage.',
        ),
        DeclareLaunchArgument(
            'orientation_tolerance',
            default_value='0.10',
            description='Orientation goal tolerance for each stage.',
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
            'grasp_trajectory_topic',
            default_value='/grasp_trajectory',
            description='Topic for successful grasp trajectories.',
        ),
        DeclareLaunchArgument(
            'assembly_trajectory_topic',
            default_value='/assembly_trajectory',
            description='Topic for successful assembly trajectories.',
        ),
        DeclareLaunchArgument('pre_place_trajectory_topic', default_value='/pre_place_trajectory'),
        DeclareLaunchArgument('place_trajectory_topic', default_value='/place_trajectory'),
        DeclareLaunchArgument('retreat_trajectory_topic', default_value='/retreat_trajectory'),
        DeclareLaunchArgument(
            'trajectory_status_topic',
            default_value='/assembly_sequence_trajectory_status',
            description='Topic for trajectory publication status.',
        ),
        DeclareLaunchArgument(
            'require_grasp_pose',
            default_value='false',
            description=(
                'Deprecated compatibility switch for pre_grasp,grasp,assembly '
                'when stage_names is not provided directly to the node.'
            ),
        ),
        DeclareLaunchArgument(
            'require_place_sequence', default_value='false',
            description=(
                'Deprecated compatibility switch for the five-stage place '
                'sequence when stage_names is not provided directly to the node.'
            ),
        ),
        DeclareLaunchArgument(
            'start_state_mode',
            default_value='current',
            description=(
                "Pre-grasp start state source: 'current' or deterministic 'fixed'."
            ),
        ),
        DeclareLaunchArgument(
            'end_effector_link',
            default_value='panda_link8',
            description='Robot link whose pose each stage target specifies.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when an upstream simulator provides /clock.',
        ),
        Node(
            package='adaptive_assembly_planning',
            executable='assembly_sequence_planning_node',
            name='assembly_sequence_planning_node',
            output='screen',
            parameters=[{
                'pre_grasp_topic': '/panda_pre_grasp_pose',
                'grasp_topic': '/panda_grasp_pose',
                'assembly_topic': '/panda_assembly_pose',
                'pre_place_topic': '/panda_pre_place_pose',
                'place_topic': '/panda_place_pose',
                'retreat_topic': '/panda_retreat_pose',
                'lift_topic': '/panda_lift_pose',
                'success_topic': '/assembly_sequence_plan_success',
                'status_topic': '/assembly_sequence_planning_status',
                'duration_topic': '/assembly_sequence_planning_duration_ms',
                'stage_status_topic': '/assembly_sequence_stage_status',
                'stage_success_topic': '/assembly_sequence_stage_success',
                'stage_duration_topic': '/assembly_sequence_stage_duration_ms',
                'pre_grasp_trajectory_topic': pre_grasp_trajectory_topic,
                'grasp_trajectory_topic': grasp_trajectory_topic,
                'assembly_trajectory_topic': assembly_trajectory_topic,
                'lift_trajectory_topic': lift_trajectory_topic,
                **{
                    f'{name}_trajectory_topic': LaunchConfiguration(
                        f'{name}_trajectory_topic'
                    )
                    for name in ('pre_place', 'place', 'retreat')
                },
                'require_grasp_pose': ParameterValue(
                    require_grasp_pose,
                    value_type=bool,
                ),
                'require_place_sequence': ParameterValue(require_place_sequence, value_type=bool),
                'stage_names_csv': stage_names,
                'trajectory_status_topic': trajectory_status_topic,
                'publish_diagnostics': ParameterValue(
                    publish_diagnostics,
                    value_type=bool,
                ),
                'publish_trajectories': ParameterValue(
                    publish_trajectories,
                    value_type=bool,
                ),
                'planning_group': 'panda_arm',
                'end_effector_link': end_effector_link,
                'planner_id': planner_id,
                'num_planning_attempts': ParameterValue(
                    num_planning_attempts,
                    value_type=int,
                ),
                'planning_time_sec': ParameterValue(
                    planning_time_sec,
                    value_type=float,
                ),
                'position_tolerance': ParameterValue(
                    position_tolerance,
                    value_type=float,
                ),
                'orientation_tolerance': ParameterValue(
                    orientation_tolerance,
                    value_type=float,
                ),
                'start_state_mode': start_state_mode,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
