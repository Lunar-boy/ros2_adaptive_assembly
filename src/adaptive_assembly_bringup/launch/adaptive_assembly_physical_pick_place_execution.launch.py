"""Launch simulator-only multi-stage pick-place execution."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _typed_value(name: str, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    """Start the PR65 planner, PR63 bridge, and PR66 executor as requested."""
    stage_names = LaunchConfiguration('stage_names')
    launch_reachable_sequence = LaunchConfiguration('launch_reachable_sequence')
    launch_gripper_bridge = LaunchConfiguration('launch_gripper_bridge')

    reachable_sequence_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
    ])

    string_arguments = {
        'stage_names': 'pre_grasp,grasp,lift,pre_place,place,retreat',
        'pre_grasp_trajectory_topic': '/pre_grasp_trajectory',
        'grasp_trajectory_topic': '/grasp_trajectory',
        'lift_trajectory_topic': '/lift_trajectory',
        'pre_place_trajectory_topic': '/pre_place_trajectory',
        'place_trajectory_topic': '/place_trajectory',
        'retreat_trajectory_topic': '/retreat_trajectory',
        'arm_controller_action_name': (
            '/panda_arm_controller/follow_joint_trajectory'
        ),
        'gripper_command_topic': '/gripper_command',
        'gripper_status_topic': '/physical_gripper_command_status',
        'gripper_success_topic': '/physical_gripper_command_success',
        'gripper_closed_topic': '/physical_gripper_closed',
        'close_after_stage': 'grasp',
        'open_after_stage': 'place',
    }
    bool_arguments = {
        'send_arm_goals': 'true',
        'send_gripper_commands': 'true',
        'require_gripper_success': 'true',
        'require_non_empty_trajectory': 'true',
        'require_panda_joints': 'true',
        'require_joint_state': 'true',
        'simulated_execution_only': 'true',
        'launch_reachable_sequence': 'true',
        'launch_gripper_bridge': 'true',
    }
    float_arguments = {
        'wait_for_arm_controller_sec': '5.0',
        'arm_result_timeout_sec': '10.0',
        'gripper_command_timeout_sec': '5.0',
        'start_state_tolerance': '0.05',
    }

    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in {
            **string_arguments, **bool_arguments, **float_arguments
        }.items()
    ]

    executor_parameters = {
        name: LaunchConfiguration(name)
        for name in string_arguments
    }
    executor_parameters.update({
        name: _typed_value(name, bool) for name in bool_arguments
        if name not in ('launch_reachable_sequence', 'launch_gripper_bridge')
    })
    executor_parameters.update({
        name: _typed_value(name, float) for name in float_arguments
    })

    return LaunchDescription(declarations + [
        LogInfo(msg=(
            'Launching simulator-only physical pick-place execution. '
            'No contact sensing, grasp verification, or hardware execution '
            'is enabled.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(reachable_sequence_launch),
            condition=IfCondition(launch_reachable_sequence),
            launch_arguments={
                'stage_names': stage_names,
                'pre_grasp_trajectory_topic': LaunchConfiguration(
                    'pre_grasp_trajectory_topic'
                ),
                'grasp_trajectory_topic': LaunchConfiguration(
                    'grasp_trajectory_topic'
                ),
                'lift_trajectory_topic': LaunchConfiguration(
                    'lift_trajectory_topic'
                ),
                'pre_place_trajectory_topic': LaunchConfiguration(
                    'pre_place_trajectory_topic'
                ),
                'place_trajectory_topic': LaunchConfiguration(
                    'place_trajectory_topic'
                ),
                'retreat_trajectory_topic': LaunchConfiguration(
                    'retreat_trajectory_topic'
                ),
            }.items(),
        ),
        Node(
            package='adaptive_assembly_manipulation',
            executable='gripper_action_bridge_node',
            name='gripper_action_bridge_node',
            output='screen',
            condition=IfCondition(launch_gripper_bridge),
            parameters=[{
                'command_topic': LaunchConfiguration('gripper_command_topic'),
                'status_topic': LaunchConfiguration('gripper_status_topic'),
                'success_topic': LaunchConfiguration('gripper_success_topic'),
                'closed_topic': LaunchConfiguration('gripper_closed_topic'),
                'send_goals': _typed_value('send_gripper_commands', bool),
                'simulated_only': _typed_value(
                    'simulated_execution_only', bool
                ),
            }],
        ),
        Node(
            package='adaptive_assembly_execution',
            executable='physical_pick_place_executor_node',
            name='physical_pick_place_executor_node',
            output='screen',
            parameters=[executor_parameters],
        ),
    ])
