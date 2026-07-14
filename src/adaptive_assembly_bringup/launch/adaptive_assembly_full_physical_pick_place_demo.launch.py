"""Launch the simulator-only full physical pick-place demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start Gazebo, dedicated physical planning, and physical execution."""
    default_world = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'worlds',
        'adaptive_assembly_physical_workcell.sdf',
    ])
    sim_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'launch',
        'adaptive_assembly_panda_gazebo.launch.py',
    ])
    planning_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_physical_planning.launch.py',
    ])
    execution_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
    ])
    target_pose_adapter_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'launch',
        'gazebo_target_pose_adapter.launch.py',
    ])
    physical_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_physical_pick_place_params.yaml',
    ])
    physical_planning_scene_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'physical_workcell_planning_scene.yaml',
    ])

    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    params_file = LaunchConfiguration('params_file')
    enable_arm_collisions = LaunchConfiguration('enable_arm_collisions')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_fake_object_pose_node = LaunchConfiguration(
        'launch_fake_object_pose_node'
    )
    launch_object_pose_observer = LaunchConfiguration(
        'launch_object_pose_observer'
    )
    target_reference_z_offset = LaunchConfiguration(
        'target_reference_z_offset'
    )
    target_pose_output_frame_id = LaunchConfiguration(
        'target_pose_output_frame_id'
    )
    end_effector_link = LaunchConfiguration('end_effector_link')
    static_planning_scene_params_file = LaunchConfiguration(
        'static_planning_scene_params_file'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Physical Gazebo SDF workcell for contact verification.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value=[world],
            description=(
                'Arguments passed to Gazebo Sim. Use "-s <world>" for a '
                'server-only headless run.'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=physical_params_file,
            description=(
                'Task parameter YAML for the physical pick-place demo. The '
                'default places objects at the physical Gazebo socket.'
            ),
        ),
        DeclareLaunchArgument(
            'static_planning_scene_params_file',
            default_value=physical_planning_scene_params_file,
            description=(
                'Physical-workcell static PlanningScene geometry. The SDF '
                'world and panda_link0 are coincident in this demo.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_arm_collisions',
            default_value='true',
            description=(
                'Enable Panda collision geometry required by the physical '
                'finger contact path.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo /clock time domain for the physical demo.',
        ),
        DeclareLaunchArgument(
            'launch_fake_object_pose_node',
            default_value='false',
            description=(
                'Compatibility switch. The dedicated physical planning launch '
                'never starts fake perception.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_object_pose_observer',
            default_value='true',
            description=(
                'Launch the dedicated Gazebo target-object Pose bridge and '
                'observer. Keep true for the full physical demo.'
            ),
        ),
        DeclareLaunchArgument(
            'end_effector_link',
            default_value='assembly_tcp',
            description='Explicit MoveIt target link for every physical stage.',
        ),
        DeclareLaunchArgument(
            'target_reference_z_offset',
            default_value='0.0',
            description=(
                'Z offset from the Gazebo model center to the task reference '
                'pose. The physical default preserves the cylinder center.'
            ),
        ),
        DeclareLaunchArgument(
            'target_pose_output_frame_id',
            default_value='world',
            description=(
                'Output frame label for /target_pose; this does not perform '
                'a TF coordinate transform.'
            ),
        ),
        LogInfo(msg=(
            'Launching the simulator-only full physical pick-place demo with '
            'a dedicated physical planning stack. Legacy Panda planning demo '
            'wrappers, fake perception, and fake controllers are bypassed.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'world': world,
                'gz_args': gz_args,
                'world_name': 'adaptive_assembly_physical_workcell',
                'enable_arm_collisions': enable_arm_collisions,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(target_pose_adapter_launch),
            condition=UnlessCondition(launch_fake_object_pose_node),
            launch_arguments={
                'input_pose_topic': '/gazebo_target_object_pose',
                'output_pose_topic': '/target_pose',
                'target_reference_z_offset': target_reference_z_offset,
                'output_frame_id': target_pose_output_frame_id,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch),
            launch_arguments={
                'params_file': params_file,
                'static_planning_scene_params_file': (
                    static_planning_scene_params_file
                ),
                'use_sim_time': use_sim_time,
                'end_effector_link': end_effector_link,
                'stage_names': (
                    'pre_grasp,grasp,lift,pre_place,place,retreat'
                ),
                'position_tolerance': '0.005',
                'orientation_tolerance': '0.03',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(execution_launch),
            launch_arguments={
                'use_standard_panda_demo': 'false',
                'use_sim_time': use_sim_time,
                'launch_reachable_sequence': 'false',
                'launch_fake_object_pose_node': launch_fake_object_pose_node,
                'launch_object_pose_observer': launch_object_pose_observer,
                'target_object_gazebo_pose_topic': '/model/target_object/pose',
                'target_object_raw_pose_topic': (
                    '/gazebo_target_object_pose_raw'
                ),
                'object_pose_topic': '/gazebo_target_object_pose',
                'object_pose_available_topic': (
                    '/gazebo_target_object_pose_available'
                ),
                'params_file': params_file,
                'end_effector_link': end_effector_link,
                'static_planning_scene_params_file': (
                    static_planning_scene_params_file
                ),
            }.items(),
        ),
    ])
