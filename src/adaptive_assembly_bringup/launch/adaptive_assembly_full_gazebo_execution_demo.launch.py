"""Launch full Gazebo Panda ros2_control execution of the planned sequence."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
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
    default_world = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'worlds',
        'adaptive_assembly_workcell.sdf',
    ])

    return LaunchDescription([
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
            'starts Gazebo, spawns a Panda-like ros2_control arm, plans the '
            'two-stage sequence, and sends both trajectories to the simulated '
            'controller. Real hardware, gripper control, object attachment, '
            'and contact-rich insertion are not enabled.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'world': world,
                'gz_args': gz_args,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(execution_launch),
            launch_arguments={
                'controller_action_name': controller_action_name,
                'wait_for_controller_sec': wait_for_controller_sec,
                'result_timeout_sec': result_timeout_sec,
                'use_planning_scene_audit': use_planning_scene_audit,
                'launch_reachable_sequence': 'false',
            }.items(),
        ),
    ])
