"""Launch the plan-only pre-grasp MoveIt2 planning bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start the pre-grasp planning node without launching robot bringup."""
    planner_id = LaunchConfiguration('planner_id')
    num_planning_attempts = LaunchConfiguration('num_planning_attempts')
    max_velocity_scaling_factor = LaunchConfiguration('max_velocity_scaling_factor')
    max_acceleration_scaling_factor = LaunchConfiguration(
        'max_acceleration_scaling_factor'
    )

    return LaunchDescription([
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
        Node(
            package='adaptive_assembly_planning',
            executable='pre_grasp_planning_node',
            name='pre_grasp_planning_node',
            output='screen',
            parameters=[{
                'input_topic': '/panda_pre_grasp_pose',
                'success_topic': '/pre_grasp_plan_success',
                'status_topic': '/pre_grasp_planning_status',
                'duration_topic': '/pre_grasp_planning_duration_ms',
                'publish_diagnostics': True,
                'planning_group': 'panda_arm',
                'planning_time_sec': 5.0,
                'position_tolerance': 0.01,
                'orientation_tolerance': 0.10,
                'min_replan_distance': 0.03,
                'planner_id': planner_id,
                'num_planning_attempts': ParameterValue(
                    num_planning_attempts,
                    value_type=int,
                ),
                'max_velocity_scaling_factor': ParameterValue(
                    max_velocity_scaling_factor,
                    value_type=float,
                ),
                'max_acceleration_scaling_factor': ParameterValue(
                    max_acceleration_scaling_factor,
                    value_type=float,
                ),
            }],
        ),
    ])
