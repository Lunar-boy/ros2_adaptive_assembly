"""Launch the plan-only pre-grasp MoveIt2 planning bridge."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the pre-grasp planning node without launching robot bringup."""
    return LaunchDescription([
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
                'planner_id': '',
                'num_planning_attempts': 1,
                'max_velocity_scaling_factor': 1.0,
                'max_acceleration_scaling_factor': 1.0,
            }],
        ),
    ])
