"""Launch simulator-only physical contact pick-place execution."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _typed_value(name: str, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def _warn_if_non_physical_pose_topic(context, *args, **kwargs):
    """Warn when the physical launch is pointed at the static visual world."""
    del args, kwargs
    pose_topic = LaunchConfiguration('pose_info_topic').perform(context)
    if 'adaptive_assembly_physical_workcell' in pose_topic:
        return []
    return [LogInfo(msg=(
        'WARNING: adaptive_assembly_physical_pick_place_execution.launch.py '
        'is configured with pose_info_topic=' + pose_topic + '. Physical '
        'grasp verification must use '
        '/world/adaptive_assembly_physical_workcell/pose/info from '
        'adaptive_assembly_physical_workcell.sdf, not the static visual '
        'adaptive_assembly_workcell.sdf.'
    ))]


def generate_launch_description() -> LaunchDescription:
    """Start the PR65 planner, PR63 bridge, and PR66 executor as requested."""
    stage_names = LaunchConfiguration('stage_names')
    launch_reachable_sequence = LaunchConfiguration('launch_reachable_sequence')
    launch_gripper_bridge = LaunchConfiguration('launch_gripper_bridge')
    launch_contact_bridge = LaunchConfiguration('launch_contact_bridge')
    launch_contact_status_node = LaunchConfiguration('launch_contact_status_node')
    launch_grasp_verifier = LaunchConfiguration('launch_grasp_verifier')
    launch_object_pose_observer = LaunchConfiguration('launch_object_pose_observer')
    launch_physical_grasp_preflight = LaunchConfiguration(
        'launch_physical_grasp_preflight'
    )
    use_standard_panda_demo = LaunchConfiguration('use_standard_panda_demo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_fake_object_pose_node = LaunchConfiguration(
        'launch_fake_object_pose_node'
    )

    reachable_sequence_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
    ])
    physical_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_physical_pick_place_params.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    static_planning_scene_params_file = LaunchConfiguration(
        'static_planning_scene_params_file'
    )

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
        'left_contact_topic': '/panda_leftfinger_contact',
        'right_contact_topic': '/panda_rightfinger_contact',
        'target_object_name': 'target_object',
        'left_contact_status_topic': '/left_gripper_contact_status',
        'right_contact_status_topic': '/right_gripper_contact_status',
        'aggregate_contact_status_topic': '/grasp_contact_status',
        'left_contact_detected_topic': '/left_gripper_contact_detected',
        'right_contact_detected_topic': '/right_gripper_contact_detected',
        'both_contacts_detected_topic': '/both_gripper_contacts_detected',
        'contact_status_topic': '/grasp_contact_status',
        'object_pose_topic': '/gazebo_target_object_pose',
        'object_pose_available_topic': '/gazebo_target_object_pose_available',
        'physical_grasp_preflight_status_topic': (
            '/physical_grasp_preflight_status'
        ),
        'grasp_verification_request_topic': '/grasp_verification_request',
        'grasp_verification_status_topic': '/grasp_verification_status',
        'grasp_verified_topic': '/grasp_verified',
        'lift_verified_topic': '/lift_verified',
        'slip_distance_mm_topic': '/grasp_slip_distance_mm',
        'pose_info_topic': '/world/adaptive_assembly_physical_workcell/pose/info',
        'target_object_gazebo_pose_topic': '/model/target_object/pose',
        'target_object_raw_pose_topic': '/gazebo_target_object_pose_raw',
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
        'launch_contact_bridge': 'true',
        'launch_contact_status_node': 'true',
        'launch_grasp_verifier': 'true',
        'launch_object_pose_observer': 'true',
        'launch_physical_grasp_preflight': 'true',
        'use_standard_panda_demo': 'false',
        'use_sim_time': 'false',
        'require_physical_grasp_preflight': 'true',
        'require_grasp_verification': 'true',
        'require_lift_verification': 'true',
        'require_both_contacts': 'true',
        'require_gripper_closed': 'true',
        'require_object_pose': 'true',
        'require_target_object_contact': 'true',
        'require_target_entity_exact_match': 'false',
    }
    float_arguments = {
        'wait_for_arm_controller_sec': '5.0',
        'arm_result_timeout_sec': '10.0',
        'gripper_command_timeout_sec': '5.0',
        'contact_stale_timeout_sec': '0.5',
        'verification_timeout_sec': '5.0',
        'physical_grasp_preflight_timeout_sec': '20.0',
        'min_lift_delta_m': '0.02',
        'max_slip_distance_m': '0.025',
        'pose_stale_timeout_sec': '1.0',
        'start_state_tolerance': '0.05',
    }

    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in {
            **string_arguments, **bool_arguments, **float_arguments
        }.items()
    ]
    declarations.append(DeclareLaunchArgument(
        'launch_fake_object_pose_node',
        default_value='true',
        description=(
            'Whether the nested adaptive pipeline starts fake perception.'
        ),
    ))
    declarations.append(DeclareLaunchArgument(
        'params_file',
        default_value=physical_params_file,
        description=(
            'Task parameter YAML for physical pick-place placement at the '
            'Gazebo assembly socket.'
        ),
    ))
    declarations.append(DeclareLaunchArgument(
        'static_planning_scene_params_file',
        default_value='',
        description=(
            'Optional static PlanningScene parameter YAML forwarded to the '
            'nested planning launch.'
        ),
    ))

    executor_string_names = (
        'stage_names',
        'pre_grasp_trajectory_topic',
        'grasp_trajectory_topic',
        'lift_trajectory_topic',
        'pre_place_trajectory_topic',
        'place_trajectory_topic',
        'retreat_trajectory_topic',
        'arm_controller_action_name',
        'gripper_command_topic',
        'gripper_status_topic',
        'gripper_success_topic',
        'gripper_closed_topic',
        'grasp_verification_request_topic',
        'grasp_verification_status_topic',
        'grasp_verified_topic',
        'lift_verified_topic',
        'physical_grasp_preflight_status_topic',
        'close_after_stage',
        'open_after_stage',
    )
    executor_parameters = {
        name: LaunchConfiguration(name)
        for name in executor_string_names
    }
    executor_parameters.update({
        name: _typed_value(name, bool) for name in bool_arguments
        if name not in (
            'launch_reachable_sequence',
            'launch_gripper_bridge',
            'launch_contact_bridge',
            'launch_contact_status_node',
            'launch_grasp_verifier',
            'launch_object_pose_observer',
            'launch_physical_grasp_preflight',
            'use_standard_panda_demo',
            'require_both_contacts',
            'require_gripper_closed',
            'require_object_pose',
            'require_target_object_contact',
            'require_target_entity_exact_match',
        )
    })
    executor_parameters.update({
        name: _typed_value(name, float) for name in float_arguments
        if name not in (
            'contact_stale_timeout_sec',
            'min_lift_delta_m',
            'max_slip_distance_m',
            'pose_stale_timeout_sec',
        )
    })
    executor_parameters['use_sim_time'] = _typed_value('use_sim_time', bool)

    preflight_parameters = {
        'use_sim_time': _typed_value('use_sim_time', bool),
        'pose_info_topic': LaunchConfiguration('pose_info_topic'),
        'object_pose_available_topic': LaunchConfiguration(
            'object_pose_available_topic'
        ),
        'kinematic_attach_status_topic': '/gazebo_attach_detach_status',
        'left_contact_topic': LaunchConfiguration('left_contact_topic'),
        'right_contact_topic': LaunchConfiguration('right_contact_topic'),
        'contact_status_topic': LaunchConfiguration('contact_status_topic'),
        'status_topic': LaunchConfiguration(
            'physical_grasp_preflight_status_topic'
        ),
        'timeout_sec': _typed_value(
            'physical_grasp_preflight_timeout_sec', float
        ),
        'simulated_only': _typed_value('simulated_execution_only', bool),
    }

    contact_status_parameters = {
        name: LaunchConfiguration(name)
        for name in (
            'left_contact_topic',
            'right_contact_topic',
            'target_object_name',
            'left_contact_status_topic',
            'right_contact_status_topic',
            'aggregate_contact_status_topic',
            'left_contact_detected_topic',
            'right_contact_detected_topic',
            'both_contacts_detected_topic',
        )
    }
    contact_status_parameters.update({
        'use_sim_time': _typed_value('use_sim_time', bool),
        'contact_stale_timeout_sec': _typed_value(
            'contact_stale_timeout_sec', float
        ),
        'publish_period_sec': 0.05,
        'require_target_object_contact': _typed_value(
            'require_target_object_contact', bool
        ),
        'simulated_only': _typed_value('simulated_execution_only', bool),
    })

    verifier_parameters = {
        name: LaunchConfiguration(name)
        for name in (
            'contact_status_topic',
            'both_contacts_detected_topic',
            'gripper_success_topic',
            'gripper_closed_topic',
            'object_pose_topic',
            'object_pose_available_topic',
            'grasp_verification_request_topic',
            'grasp_verification_status_topic',
            'grasp_verified_topic',
            'lift_verified_topic',
            'slip_distance_mm_topic',
        )
    }
    verifier_parameters.update({
        'use_sim_time': _typed_value('use_sim_time', bool),
        'require_both_contacts': _typed_value('require_both_contacts', bool),
        'require_gripper_closed': _typed_value('require_gripper_closed', bool),
        'require_object_pose': _typed_value('require_object_pose', bool),
        'min_lift_delta_m': _typed_value('min_lift_delta_m', float),
        'max_slip_distance_m': _typed_value('max_slip_distance_m', float),
        'pose_stale_timeout_sec': _typed_value('pose_stale_timeout_sec', float),
        'simulated_only': _typed_value('simulated_execution_only', bool),
    })

    return LaunchDescription(declarations + [
        LogInfo(msg=(
            'Launching simulator-only physical pick-place execution. '
            'This launch must be paired with '
            'adaptive_assembly_physical_workcell.sdf and '
            'pose_info_topic=/world/adaptive_assembly_physical_workcell/pose/info. '
            'Target pose transport uses /model/target_object/pose as one '
            'gz.msgs.Pose bridged to /gazebo_target_object_pose_raw. '
            'Do not use adaptive_assembly_workcell.sdf for physical grasp '
            'verification. '
            'Gazebo contact status and grasp/lift verification launch '
            'according to launch_contact_status_node and '
            'launch_grasp_verifier. Real hardware execution is unsupported.'
        )),
        OpaqueFunction(function=_warn_if_non_physical_pose_topic),
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
                'use_standard_panda_demo': use_standard_panda_demo,
                'use_sim_time': use_sim_time,
                'launch_fake_object_pose_node': launch_fake_object_pose_node,
                'params_file': params_file,
                'static_planning_scene_params_file': (
                    static_planning_scene_params_file
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
                'use_sim_time': _typed_value('use_sim_time', bool),
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
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_finger_contact_bridge',
            output='screen',
            condition=IfCondition(launch_contact_bridge),
            arguments=[
                [
                    LaunchConfiguration('left_contact_topic'),
                    '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                ],
                [
                    LaunchConfiguration('right_contact_topic'),
                    '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                ],
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='physical_target_object_pose_bridge',
            output='screen',
            condition=IfCondition(launch_object_pose_observer),
            arguments=[[
                LaunchConfiguration('target_object_gazebo_pose_topic'),
                '@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            ]],
            remappings=[(
                LaunchConfiguration('target_object_gazebo_pose_topic'),
                LaunchConfiguration('target_object_raw_pose_topic'),
            )],
        ),
        Node(
            package='adaptive_assembly_sim',
            executable='gazebo_entity_pose_observer_node',
            name='physical_target_object_pose_observer',
            output='screen',
            condition=IfCondition(launch_object_pose_observer),
            parameters=[{
                'use_sim_time': _typed_value('use_sim_time', bool),
                'pose_info_topic': LaunchConfiguration(
                    'target_object_raw_pose_topic'
                ),
                'input_message_type': 'pose_stamped',
                'target_entity_name': LaunchConfiguration('target_object_name'),
                'world_frame': 'world',
                'output_pose_topic': LaunchConfiguration('object_pose_topic'),
                'status_topic': '/gazebo_target_object_pose_status',
                'available_topic': LaunchConfiguration(
                    'object_pose_available_topic'
                ),
                'pose_age_ms_topic': '/gazebo_target_object_pose_age_ms',
                'stale_timeout_sec': _typed_value(
                    'pose_stale_timeout_sec', float
                ),
                'publish_period_sec': 0.1,
                'require_model_name_match': _typed_value(
                    'require_target_entity_exact_match', bool
                ),
                'simulated_only': _typed_value(
                    'simulated_execution_only', bool
                ),
            }],
        ),
        Node(
            package='adaptive_assembly_execution',
            executable='gazebo_grasp_contact_status_node',
            name='gazebo_grasp_contact_status_node',
            output='screen',
            condition=IfCondition(launch_contact_status_node),
            parameters=[contact_status_parameters],
        ),
        Node(
            package='adaptive_assembly_execution',
            executable='grasp_verifier_node',
            name='grasp_verifier_node',
            output='screen',
            condition=IfCondition(launch_grasp_verifier),
            parameters=[verifier_parameters],
        ),
        Node(
            package='adaptive_assembly_execution',
            executable='physical_grasp_preflight_node',
            name='physical_grasp_preflight_node',
            output='screen',
            condition=IfCondition(launch_physical_grasp_preflight),
            parameters=[preflight_parameters],
        ),
        Node(
            package='adaptive_assembly_execution',
            executable='physical_pick_place_executor_node',
            name='physical_pick_place_executor_node',
            output='screen',
            parameters=[executor_parameters],
        ),
    ])
