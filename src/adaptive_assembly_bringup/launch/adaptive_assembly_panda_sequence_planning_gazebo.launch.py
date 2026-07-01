"""Launch known-reachable planning without a mock ros2_control stack."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    """Start move_group and sequence planning against Gazebo joint states."""
    bringup_share = FindPackageShare('adaptive_assembly_bringup')
    planning_share = FindPackageShare('adaptive_assembly_planning')
    reachable_params = PathJoinSubstitution([
        bringup_share,
        'config',
        'adaptive_assembly_sequence_reachable_params.yaml',
    ])
    pipeline_launch = PathJoinSubstitution([
        bringup_share,
        'launch',
        'adaptive_assembly_pipeline.launch.py',
    ])
    static_scene_launch = PathJoinSubstitution([
        planning_share,
        'launch',
        'static_planning_scene.launch.py',
    ])
    pre_grasp_adapter_launch = PathJoinSubstitution([
        planning_share,
        'launch',
        'panda_pre_grasp_pose_adapter.launch.py',
    ])
    assembly_adapter_launch = PathJoinSubstitution([
        planning_share,
        'launch',
        'panda_assembly_pose_adapter.launch.py',
    ])
    sequence_planner_launch = PathJoinSubstitution([
        planning_share,
        'launch',
        'assembly_sequence_planning.launch.py',
    ])
    planning_scene_audit_launch = PathJoinSubstitution([
        planning_share,
        'launch',
        'planning_scene_audit.launch.py',
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
        .trajectory_execution(
            file_path='config/gripper_moveit_controllers.yaml'
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

    return LaunchDescription([
        LogInfo(msg=(
            'Launching Gazebo-compatible plan-only Panda sequence planning. '
            'move_group consumes Gazebo joint states; no mock ros2_control '
            'node or mock Panda controller is started.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pipeline_launch),
            launch_arguments={'params_file': reachable_params}.items(),
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[moveit_config.to_dict()],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_scene_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pre_grasp_adapter_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(assembly_adapter_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sequence_planner_launch),
            launch_arguments={
                'start_state_mode': 'fixed',
                'planning_time_sec': '5.0',
                'num_planning_attempts': '1',
                'position_tolerance': '0.01',
                'orientation_tolerance': '0.10',
                'pre_grasp_trajectory_topic': '/pre_grasp_trajectory',
                'assembly_trajectory_topic': '/assembly_trajectory',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_scene_audit_launch),
        ),
    ])
