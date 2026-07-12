"""Launch static PlanningScene collision objects for the Panda demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _static_planning_scene_node(context, *args, **kwargs):
    """Create the node with an optional physical-workcell parameter file."""
    del args, kwargs
    default_parameters = {
        'planning_frame': 'panda_link0',
        'apply_delay_sec': 1.0,
        'add_work_table': True,
        'table_x': 0.45,
        'table_y': 0.0,
        'table_z': -0.04,
        'table_size_x': 0.80,
        'table_size_y': 0.80,
        'table_size_z': 0.04,
        'add_target_support': True,
        'target_support_x': ParameterValue(
            LaunchConfiguration('target_support_x'), value_type=float
        ),
        'target_support_y': ParameterValue(
            LaunchConfiguration('target_support_y'), value_type=float
        ),
        'target_support_z': ParameterValue(
            LaunchConfiguration('target_support_z'), value_type=float
        ),
        'target_support_size_x': ParameterValue(
            LaunchConfiguration('target_support_size_x'), value_type=float
        ),
        'target_support_size_y': ParameterValue(
            LaunchConfiguration('target_support_size_y'), value_type=float,
        ),
        'target_support_size_z': ParameterValue(
            LaunchConfiguration('target_support_size_z'), value_type=float,
        ),
        'add_socket_fixture': True,
        'socket_x': 0.62,
        'socket_y': -0.18,
        'socket_z': 0.0,
        'socket_base_size_x': 0.20,
        'socket_base_size_y': 0.20,
        'socket_base_size_z': 0.03,
        'socket_wall_height': 0.08,
        'socket_wall_thickness': 0.03,
        'socket_side_wall_length': 0.16,
        'socket_back_front_wall_length': 0.10,
        'socket_side_wall_y_offset': 0.065,
        'socket_back_front_wall_x_offset': 0.065,
        'socket_base_center_z_offset': 0.015,
        'socket_wall_center_z_offset': 0.055,
    }
    parameters = [default_parameters]
    params_file = LaunchConfiguration(
        'static_planning_scene_params_file'
    ).perform(context)
    if params_file:
        parameters.append(params_file)

    return [Node(
        package='adaptive_assembly_planning',
        executable='static_planning_scene_node',
        name='static_planning_scene_node',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description() -> LaunchDescription:
    """Start the static PlanningScene node without launching MoveIt2."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'static_planning_scene_params_file',
            default_value='',
            description=(
                'Optional ROS 2 parameter YAML for static PlanningScene '
                'geometry. Empty keeps the existing default workcell.'
            ),
        ),
        DeclareLaunchArgument('target_support_x', default_value='0.45'),
        DeclareLaunchArgument('target_support_y', default_value='0.0'),
        DeclareLaunchArgument('target_support_z', default_value='0.01'),
        DeclareLaunchArgument('target_support_size_x', default_value='0.12'),
        DeclareLaunchArgument('target_support_size_y', default_value='0.12'),
        DeclareLaunchArgument('target_support_size_z', default_value='0.02'),
        OpaqueFunction(function=_static_planning_scene_node),
    ])
