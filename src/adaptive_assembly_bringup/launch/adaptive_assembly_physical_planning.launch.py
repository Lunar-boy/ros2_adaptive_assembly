"""Launch the physical pick-place planning stack without legacy demo wrappers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


STAGES = (
    ('pre_grasp', '/pre_grasp_pose', '/panda_pre_grasp_pose'),
    ('grasp', '/grasp_pose', '/panda_grasp_pose'),
    ('lift', '/lift_pose', '/panda_lift_pose'),
    ('pre_place', '/pre_place_pose', '/panda_pre_place_pose'),
    ('place', '/place_pose', '/panda_place_pose'),
    ('retreat', '/retreat_pose', '/panda_retreat_pose'),
)


def _typed(name: str, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def _pose_adapter(stage: str, input_topic: str, output_topic: str) -> Node:
    """Create one deterministic task-pose to Panda-base adapter."""
    status_topic = (
        '/panda_pose_adapter_status'
        if stage == 'pre_grasp'
        else f'/panda_{stage}_pose_adapter_status'
    )
    return Node(
        package='adaptive_assembly_planning',
        executable='panda_pre_grasp_pose_adapter_node',
        name=f'panda_{stage}_pose_adapter_node',
        output='screen',
        parameters=[{
            'use_sim_time': _typed('use_sim_time', bool),
            'input_topic': input_topic,
            'output_topic': output_topic,
            'output_frame_id': 'panda_link0',
            'use_tf_transform': False,
            'target_frame_id': 'panda_link0',
            'tf_lookup_timeout_sec': 0.2,
            'status_topic': status_topic,
            'x_offset': 0.0,
            'y_offset': 0.0,
            'z_offset': 0.0,
            'use_fixed_orientation': True,
            'fixed_qx': 1.0,
            'fixed_qy': 0.0,
            'fixed_qz': 0.0,
            'fixed_qw': 0.0,
            'normalize_quaternion': True,
        }],
    )


def generate_launch_description() -> LaunchDescription:
    """Start the complete physical planning path directly."""
    default_params = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_physical_pick_place_params.yaml',
    ])
    default_scene_params = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'physical_workcell_planning_scene.yaml',
    ])

    canonical_panda_xacro = str(
        Path(get_package_share_directory('adaptive_assembly_sim'))
        / 'urdf'
        / 'panda.urdf.xacro'
    )
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path=canonical_panda_xacro)
        .robot_description_semantic(file_path='config/panda.srdf')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=[
                'ompl',
                'chomp',
                'pilz_industrial_motion_planner',
                'stomp',
            ]
        )
        .to_moveit_configs()
    )
    gazebo_controller_parameters = {
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 1.2,
            'allowed_goal_duration_margin': 0.5,
            'allowed_start_tolerance': 0.01,
        },
        'moveit_controller_manager': (
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        ),
        'moveit_simple_controller_manager': {
            'controller_names': [
                'panda_arm_controller',
                'panda_gripper_controller',
            ],
            'panda_arm_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True,
                'joints': [f'panda_joint{index}' for index in range(1, 8)],
            },
            'panda_gripper_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': False,
                'joints': ['panda_finger_joint1', 'panda_finger_joint2'],
            },
        },
    }

    params_file = LaunchConfiguration('params_file')
    scene_params_file = LaunchConfiguration('static_planning_scene_params_file')

    declarations = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'static_planning_scene_params_file',
            default_value=default_scene_params,
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'stage_names',
            default_value='pre_grasp,grasp,lift,pre_place,place,retreat',
        ),
        DeclareLaunchArgument('end_effector_link', default_value='assembly_tcp'),
        DeclareLaunchArgument('planner_id', default_value=''),
        DeclareLaunchArgument('num_planning_attempts', default_value='1'),
        DeclareLaunchArgument('planning_time_sec', default_value='5.0'),
        DeclareLaunchArgument('position_tolerance', default_value='0.005'),
        DeclareLaunchArgument('orientation_tolerance', default_value='0.03'),
        DeclareLaunchArgument(
            'planning_scene_audit_expected_object_ids',
            default_value=(
                'work_table,target_support,assembly_socket_base,'
                'assembly_socket_left_wall,assembly_socket_right_wall,'
                'assembly_socket_back_wall,assembly_socket_front_wall'
            ),
        ),
        DeclareLaunchArgument(
            'pre_grasp_trajectory_topic', default_value='/pre_grasp_trajectory'
        ),
        DeclareLaunchArgument(
            'grasp_trajectory_topic', default_value='/grasp_trajectory'
        ),
        DeclareLaunchArgument(
            'lift_trajectory_topic', default_value='/lift_trajectory'
        ),
        DeclareLaunchArgument(
            'pre_place_trajectory_topic', default_value='/pre_place_trajectory'
        ),
        DeclareLaunchArgument(
            'place_trajectory_topic', default_value='/place_trajectory'
        ),
        DeclareLaunchArgument(
            'retreat_trajectory_topic', default_value='/retreat_trajectory'
        ),
    ]

    task_node = Node(
        package='adaptive_assembly_task',
        executable='assembly_task_node',
        name='assembly_task_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': _typed('use_sim_time', bool)}],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            gazebo_controller_parameters,
            {'use_sim_time': _typed('use_sim_time', bool)},
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    static_scene = Node(
        package='adaptive_assembly_planning',
        executable='static_planning_scene_node',
        name='static_planning_scene_node',
        output='screen',
        parameters=[scene_params_file],
    )

    scene_audit = Node(
        package='adaptive_assembly_planning',
        executable='planning_scene_audit_node',
        name='planning_scene_audit_node',
        output='screen',
        parameters=[{
            'expected_object_ids': LaunchConfiguration(
                'planning_scene_audit_expected_object_ids'
            ),
            'audit_period_sec': 2.0,
            'status_topic': '/planning_scene_audit_status',
            'ready_topic': '/planning_scene_audit_ready',
        }],
    )

    adapters = [_pose_adapter(*stage) for stage in STAGES]

    sequence_planner = Node(
        package='adaptive_assembly_planning',
        executable='assembly_sequence_planning_node',
        name='assembly_sequence_planning_node',
        output='screen',
        parameters=[{
            'pre_grasp_topic': '/panda_pre_grasp_pose',
            'grasp_topic': '/panda_grasp_pose',
            'lift_topic': '/panda_lift_pose',
            'pre_place_topic': '/panda_pre_place_pose',
            'place_topic': '/panda_place_pose',
            'retreat_topic': '/panda_retreat_pose',
            'assembly_topic': '/panda_assembly_pose',
            'stage_names_csv': LaunchConfiguration('stage_names'),
            'planning_group': 'panda_arm',
            'end_effector_link': LaunchConfiguration('end_effector_link'),
            'planner_id': LaunchConfiguration('planner_id'),
            'num_planning_attempts': _typed('num_planning_attempts', int),
            'planning_time_sec': _typed('planning_time_sec', float),
            'position_tolerance': _typed('position_tolerance', float),
            'orientation_tolerance': _typed('orientation_tolerance', float),
            'start_state_mode': 'fixed',
            'publish_diagnostics': True,
            'publish_trajectories': True,
            'require_grasp_pose': False,
            'require_place_sequence': False,
            'use_sim_time': _typed('use_sim_time', bool),
            'success_topic': '/assembly_sequence_plan_success',
            'status_topic': '/assembly_sequence_planning_status',
            'duration_topic': '/assembly_sequence_planning_duration_ms',
            'stage_status_topic': '/assembly_sequence_stage_status',
            'stage_success_topic': '/assembly_sequence_stage_success',
            'stage_duration_topic': '/assembly_sequence_stage_duration_ms',
            'trajectory_status_topic': '/assembly_sequence_trajectory_status',
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
            'assembly_trajectory_topic': '/assembly_trajectory',
        }],
    )

    return LaunchDescription(declarations + [
        LogInfo(msg=(
            'Launching the dedicated physical planning stack directly: '
            'assembly task, MoveIt move_group, physical static PlanningScene, '
            'PlanningScene audit, six Panda pose adapters, and six-stage '
            'sequence planning. Fake perception and fake controllers are not '
            'started by this launch.'
        )),
        task_node,
        move_group,
        static_scene,
        scene_audit,
        *adapters,
        sequence_planner,
    ])
