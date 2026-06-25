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
    enable_request_guard = LaunchConfiguration('enable_request_guard')
    required_frame_id = LaunchConfiguration('required_frame_id')
    workspace_min_x = LaunchConfiguration('workspace_min_x')
    workspace_max_x = LaunchConfiguration('workspace_max_x')
    workspace_min_y = LaunchConfiguration('workspace_min_y')
    workspace_max_y = LaunchConfiguration('workspace_max_y')
    workspace_min_z = LaunchConfiguration('workspace_min_z')
    workspace_max_z = LaunchConfiguration('workspace_max_z')
    min_quaternion_norm = LaunchConfiguration('min_quaternion_norm')

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
        DeclareLaunchArgument(
            'enable_request_guard',
            default_value='false',
            description='Enable pre-MoveIt2 planning request safety checks.',
        ),
        DeclareLaunchArgument(
            'required_frame_id',
            default_value='',
            description='Required input pose frame when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_min_x',
            default_value='-10.0',
            description='Minimum allowed x when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_max_x',
            default_value='10.0',
            description='Maximum allowed x when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_min_y',
            default_value='-10.0',
            description='Minimum allowed y when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_max_y',
            default_value='10.0',
            description='Maximum allowed y when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_min_z',
            default_value='-10.0',
            description='Minimum allowed z when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'workspace_max_z',
            default_value='10.0',
            description='Maximum allowed z when request guard is enabled.',
        ),
        DeclareLaunchArgument(
            'min_quaternion_norm',
            default_value='1e-6',
            description='Minimum allowed quaternion norm when guard is enabled.',
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
                'enable_request_guard': ParameterValue(
                    enable_request_guard,
                    value_type=bool,
                ),
                'required_frame_id': required_frame_id,
                'workspace_min_x': ParameterValue(workspace_min_x, value_type=float),
                'workspace_max_x': ParameterValue(workspace_max_x, value_type=float),
                'workspace_min_y': ParameterValue(workspace_min_y, value_type=float),
                'workspace_max_y': ParameterValue(workspace_max_y, value_type=float),
                'workspace_min_z': ParameterValue(workspace_min_z, value_type=float),
                'workspace_max_z': ParameterValue(workspace_max_z, value_type=float),
                'min_quaternion_norm': ParameterValue(
                    min_quaternion_norm,
                    value_type=float,
                ),
            }],
        ),
    ])
