"""Launch full Gazebo Panda ros2_control execution of the planned sequence."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose Gazebo Panda controllers, planning, and execution bridge."""
    sim_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'launch',
        'adaptive_assembly_panda_gazebo.launch.py',
    ])
    execution_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_ros2_control_execution.launch.py',
    ])
    planning_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_sequence_planning_gazebo.launch.py',
    ])

    controller_action_name = LaunchConfiguration('controller_action_name')
    wait_for_controller_sec = LaunchConfiguration('wait_for_controller_sec')
    result_timeout_sec = LaunchConfiguration('result_timeout_sec')
    use_planning_scene_audit = LaunchConfiguration(
        'use_planning_scene_audit'
    )
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    params_file = LaunchConfiguration('params_file')
    require_grasp = LaunchConfiguration('require_grasp_trajectory')
    require_place = LaunchConfiguration('require_place_sequence')
    stage_names = LaunchConfiguration('stage_names')
    require_target_sync = LaunchConfiguration('require_target_sync_success')
    target_sync_topic = LaunchConfiguration('target_sync_status_topic')
    target_sync_timeout = LaunchConfiguration('target_sync_timeout_sec')
    launch_simulation = LaunchConfiguration('launch_simulation')
    enable_arm_collisions = LaunchConfiguration('enable_arm_collisions')
    default_world = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'worlds',
        'adaptive_assembly_workcell.sdf',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_simulation', default_value='true',
            description='Start Gazebo; false when an outer launch owns it.',
        ),
        DeclareLaunchArgument(
            'enable_arm_collisions', default_value='true',
            description='Forward simplified Gazebo arm collision setting.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('adaptive_assembly_bringup'), 'config',
                'adaptive_assembly_sequence_reachable_params.yaml',
            ]),
            description='Parameter YAML for perception and task nodes.',
        ),
        DeclareLaunchArgument(
            'require_grasp_trajectory', default_value='false',
            description='Plan and execute the intermediate grasp stage.',
        ),
        DeclareLaunchArgument('require_place_sequence', default_value='false'),
        DeclareLaunchArgument('stage_names', default_value=''),
        DeclareLaunchArgument(
            'require_target_sync_success', default_value='false',
            description='Optionally gate initial execution on target sync.',
        ),
        DeclareLaunchArgument(
            'target_sync_status_topic',
            default_value='/gazebo_target_sync_status',
        ),
        DeclareLaunchArgument(
            'target_sync_timeout_sec', default_value='10.0',
        ),
        DeclareLaunchArgument(
            'controller_action_name',
            default_value='/panda_arm_controller/follow_joint_trajectory',
            description='Gazebo ros2_control FollowJointTrajectory action.',
        ),
        DeclareLaunchArgument(
            'wait_for_controller_sec',
            default_value='20.0',
            description='Bounded wait for the Gazebo controller action.',
        ),
        DeclareLaunchArgument(
            'result_timeout_sec',
            default_value='30.0',
            description='Bounded wait for each controller trajectory result.',
        ),
        DeclareLaunchArgument(
            'use_planning_scene_audit',
            default_value='true',
            description='Include the read-only PlanningScene audit.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo SDF workcell world.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value=[world],
            description=(
                'Arguments passed to Gazebo Sim. Gazebo is paused by default; '
                'do not add -r until controllers are ready.'
            ),
        ),
        LogInfo(msg=(
            'Launching full simulator-only Gazebo Panda execution. This '
            'starts Gazebo, spawns the canonical Panda with ros2_control, '
            'plans the '
            'two-stage sequence, and sends both trajectories to the simulated '
            'controller. Real hardware, gripper control, object attachment, '
            'and contact-rich insertion are not enabled.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'world': world,
                'gz_args': gz_args,
                'enable_arm_collisions': enable_arm_collisions,
            }.items(),
            condition=IfCondition(launch_simulation),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch),
            launch_arguments={
                'params_file': params_file,
                'require_grasp_pose': require_grasp,
                'require_place_sequence': require_place,
                'stage_names': stage_names,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(execution_launch),
            launch_arguments={
                'controller_action_name': controller_action_name,
                'wait_for_controller_sec': wait_for_controller_sec,
                'result_timeout_sec': result_timeout_sec,
                'use_planning_scene_audit': use_planning_scene_audit,
                'launch_reachable_sequence': 'false',
                'grasp_trajectory_topic': '/grasp_trajectory',
                'require_grasp_trajectory': require_grasp,
                'require_place_sequence': require_place,
                'require_target_sync_success': require_target_sync,
                'target_sync_status_topic': target_sync_topic,
                'target_sync_timeout_sec': target_sync_timeout,
            }.items(),
        ),
    ])
