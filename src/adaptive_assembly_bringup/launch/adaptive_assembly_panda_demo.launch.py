"""Launch adaptive assembly topics with the standard Panda MoveIt2 demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    """Start fake perception, task pose generation, and Panda MoveIt2 demo."""
    adaptive_pipeline_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_pipeline.launch.py',
    ])
    default_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_params.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    use_standard_panda_demo = LaunchConfiguration('use_standard_panda_demo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    panda_demo_launch = PathJoinSubstitution([
        FindPackageShare('moveit_resources_panda_moveit_config'),
        'launch',
        'demo.launch.py',
    ])
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(
            file_path='config/panda.urdf.xacro',
            mappings={'ros2_control_hardware_type': 'mock_components'},
        )
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
                'joints': [
                    'panda_joint1',
                    'panda_joint2',
                    'panda_joint3',
                    'panda_joint4',
                    'panda_joint5',
                    'panda_joint6',
                    'panda_joint7',
                ],
            },
            'panda_gripper_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': False,
                'joints': [
                    'panda_finger_joint1',
                    'panda_finger_joint2',
                ],
            },
        },
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Parameter YAML for adaptive assembly pipeline nodes.',
        ),
        DeclareLaunchArgument(
            'use_standard_panda_demo',
            default_value='true',
            description=(
                'Use moveit_resources_panda_moveit_config demo.launch.py. '
                'Set false for Gazebo physical demos that already provide '
                '/controller_manager through gz_ros2_control.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'Use ROS simulation time for the direct Gazebo MoveIt path. '
                'The standard fake-control demo remains wall-time by default.'
            ),
        ),
        LogInfo(
            msg='Launching adaptive assembly Panda demo: fake perception, '
            'task pose generation, and Panda MoveIt2 planning.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(adaptive_pipeline_launch),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_demo_launch),
            condition=IfCondition(use_standard_panda_demo),
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            condition=UnlessCondition(use_standard_panda_demo),
            parameters=[
                moveit_config.to_dict(),
                gazebo_controller_parameters,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
