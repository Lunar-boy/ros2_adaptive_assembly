"""Launch Gazebo Harmonic with a Panda-like ros2_control arm."""

import os
import shutil

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _required_package_share(package_name: str, install_hint: str) -> str:
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError as error:
        raise RuntimeError(
            f"Required package '{package_name}' is unavailable. {install_hint}"
        ) from error


def _launch_setup(context, *args, **kwargs):
    """Create runtime actions after dependency checks pass."""
    ros_gz_sim_share = _required_package_share(
        'ros_gz_sim',
        'Install Gazebo integration with: sudo apt install ros-jazzy-ros-gz-sim',
    )
    _required_package_share(
        'gz_ros2_control',
        'Install ros2_control Gazebo support with: '
        'sudo apt install ros-jazzy-gz-ros2-control',
    )
    _required_package_share(
        'controller_manager',
        'Install ros2_control controller manager with: '
        'sudo apt install ros-jazzy-controller-manager',
    )
    _required_package_share(
        'ros_gz_bridge',
        'Install Gazebo ROS bridges with: sudo apt install ros-jazzy-ros-gz-bridge',
    )
    _required_package_share(
        'joint_state_broadcaster',
        'Install ros-jazzy-joint-state-broadcaster.',
    )
    _required_package_share(
        'joint_trajectory_controller',
        'Install ros-jazzy-joint-trajectory-controller.',
    )
    _required_package_share(
        'xacro',
        'Install ros-jazzy-xacro.',
    )
    if shutil.which('gz') is None:
        raise RuntimeError(
            "Gazebo's 'gz' executable is unavailable. Install Gazebo "
            'Harmonic before launching full simulated execution.'
        )

    gazebo_launch = os.path.join(
        ros_gz_sim_share, 'launch', 'gz_sim.launch.py'
    )

    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    model = LaunchConfiguration('model')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    enable_arm_collisions = LaunchConfiguration('enable_arm_collisions')
    controller_manager_name = LaunchConfiguration('controller_manager_name')
    world_name = LaunchConfiguration('world_name')
    controllers_file = os.path.join(
        get_package_share_directory('adaptive_assembly_sim'),
        'config',
        'panda_ros2_control.yaml',
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' controllers_file:=', controllers_file,
            ' enable_arm_collisions:=', enable_arm_collisions,
        ]),
        value_type=str,
    )

    spawn_panda = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_panda',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-topic', '/gazebo_panda/robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )
    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_panda_controllers',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'panda_arm_controller',
            '--controller-manager',
            controller_manager_name,
            '--controller-manager-timeout',
            '30',
            '--service-call-timeout',
            '10',
            '--switch-timeout',
            '60',
            '--inactive',
        ],
    )
    unpause_gazebo = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s',
            ['/world/', world_name, '/control'],
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', 'pause: false',
        ],
        name='unpause_gazebo_after_controller_configuration',
        output='screen',
    )
    activate_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--activate', 'joint_state_broadcaster', 'panda_arm_controller',
            '--strict',
            '--controller-manager', controller_manager_name,
        ],
        name='activate_panda_controllers',
        output='screen',
    )

    return [
        LogInfo(msg=(
            'Launching Gazebo Harmonic with a simulator-only Panda '
            'ros2_control arm. Real hardware support remains disabled.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': gz_args,
                'gz_version': '8',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='panda_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            remappings=[
                ('/robot_description', '/gazebo_panda/robot_description'),
            ],
        ),
        spawn_panda,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_panda,
                on_exit=[spawn_controllers],
            ),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_controllers,
                on_exit=[unpause_gazebo],
            ),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=unpause_gazebo,
                on_exit=[activate_controllers],
            ),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Start the workcell, spawn the Panda, and activate controllers."""
    default_world = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'worlds',
        'adaptive_assembly_workcell.sdf',
    ])
    default_model = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'urdf',
        'panda_gazebo_ros2_control.urdf.xacro',
    ])
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo SDF workcell world.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value=[world],
            description='Arguments passed to Gazebo Sim; paused by default.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Panda-like URDF/xacro model to spawn.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='panda',
            description='Gazebo entity name for the spawned Panda arm.',
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'enable_arm_collisions', default_value='true',
            description=(
                'Enable simplified Gazebo arm collision geometry. MoveIt '
                'collision checking is configured separately.'
            ),
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='adaptive_assembly_workcell',
            description='Gazebo world name used by the unpause service.',
        ),
        DeclareLaunchArgument(
            'controller_manager_name',
            default_value='/controller_manager',
            description='Controller manager provided by gz_ros2_control.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
