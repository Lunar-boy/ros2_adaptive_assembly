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
        Node(
            package='adaptive_assembly_planning',
            executable='assembly_sequence_planning_node',
            name='assembly_sequence_planning_node',
            output='screen',
            parameters=[{
                'pre_grasp_topic': '/panda_pre_grasp_pose',
                'assembly_topic': '/panda_assembly_pose',
                'success_topic': '/assembly_sequence_plan_success',
                'status_topic': '/assembly_sequence_planning_status',
                'duration_topic': '/assembly_sequence_planning_duration_ms',
                'publish_diagnostics': ParameterValue(
                    publish_diagnostics,
                    value_type=bool,
                ),
                'planning_group': 'panda_arm',
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
            }],
        ),
    ])
