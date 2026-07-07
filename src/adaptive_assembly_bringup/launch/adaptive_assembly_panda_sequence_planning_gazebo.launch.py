"""Launch known-reachable planning without a mock ros2_control stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    params_file = LaunchConfiguration('params_file')
    require_grasp_pose = LaunchConfiguration('require_grasp_pose')
    require_place_sequence = LaunchConfiguration('require_place_sequence')
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
    grasp_adapter_launch = PathJoinSubstitution([
        planning_share, 'launch', 'panda_grasp_pose_adapter.launch.py',
    ])
    assembly_adapter_launch = PathJoinSubstitution([
        planning_share,
        'launch',
        'panda_assembly_pose_adapter.launch.py',
    ])
    place_adapter_launches = [
        PathJoinSubstitution([
            planning_share, 'launch', f'panda_{name}_pose_adapter.launch.py'
        ])
        for name in ('pre_place', 'place', 'retreat')
    ]
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
        DeclareLaunchArgument(
            'params_file', default_value=reachable_params,
            description='Parameter YAML for perception and task nodes.',
        ),
        DeclareLaunchArgument(
            'require_grasp_pose', default_value='false',
            description='Enable the intermediate grasp planning stage.',
        ),
        DeclareLaunchArgument('require_place_sequence', default_value='false'),
        LogInfo(msg=(
            'Launching Gazebo-compatible plan-only Panda sequence planning. '
            'move_group consumes Gazebo joint states; no mock ros2_control '
            'node or mock Panda controller is started.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pipeline_launch),
            launch_arguments={'params_file': params_file}.items(),
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
            launch_arguments={
                'target_support_x': '0.35',
                'target_support_y': '0.18',
                'target_support_z': '0.025',
                'target_support_size_x': '0.18',
                'target_support_size_y': '0.18',
                'target_support_size_z': '0.05',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pre_grasp_adapter_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(grasp_adapter_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(assembly_adapter_launch),
        ),
        *[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
            for path in place_adapter_launches
        ],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sequence_planner_launch),
            launch_arguments={
                'start_state_mode': 'fixed',
                'planning_time_sec': '5.0',
                'num_planning_attempts': '1',
                'position_tolerance': '0.01',
                'orientation_tolerance': '0.10',
                'pre_grasp_trajectory_topic': '/pre_grasp_trajectory',
                'grasp_trajectory_topic': '/grasp_trajectory',
                'assembly_trajectory_topic': '/assembly_trajectory',
                'require_grasp_pose': require_grasp_pose,
                'require_place_sequence': require_place_sequence,
                **{
                    f'{name}_trajectory_topic': f'/{name}_trajectory'
                    for name in ('pre_place', 'place', 'retreat')
                },
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_scene_audit_launch),
        ),
    ])
