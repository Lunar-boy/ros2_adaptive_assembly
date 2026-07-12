"""Launch the simulator-only full physical pick-place demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start Gazebo with the physical workcell and physical execution path."""
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

    world = LaunchConfiguration('world')
    params_file = LaunchConfiguration('params_file')
    enable_arm_collisions = LaunchConfiguration('enable_arm_collisions')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_fake_object_pose_node = LaunchConfiguration(
        'launch_fake_object_pose_node'
    )
    target_reference_z_offset = LaunchConfiguration(
        'target_reference_z_offset'
    )
    target_pose_output_frame_id = LaunchConfiguration(
        'target_pose_output_frame_id'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Physical Gazebo SDF workcell for contact verification.',
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
            description=(
                'Use the Gazebo /clock time domain for the physical demo. '
                'The Gazebo clock bridge is required.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_fake_object_pose_node',
            default_value='false',
            description=(
                'Disable fake perception so Gazebo is the only intended '
                '/target_pose source.'
            ),
        ),
        DeclareLaunchArgument(
            'target_reference_z_offset',
            default_value='0.05',
            description=(
                'Z offset from the Gazebo model center to the task reference '
                'pose. The default is half the 0.10 m target cylinder length.'
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
            'Launching simulator-only full physical pick-place demo with '
            'adaptive_assembly_physical_workcell.sdf, '
            'world_name=adaptive_assembly_physical_workcell, and '
            'enable_arm_collisions=true by default. This path is separate '
            'from the kinematic attach visual demo. Simulation time is '
            'enabled by default; /clock is expected from Gazebo. The observed '
            'Gazebo target pose is adapted to /target_pose and fake '
            'perception is disabled by default.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'world': world,
                'world_name': 'adaptive_assembly_physical_workcell',
                'enable_arm_collisions': enable_arm_collisions,
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
            PythonLaunchDescriptionSource(execution_launch),
            launch_arguments={
                'use_standard_panda_demo': 'false',
                'use_sim_time': use_sim_time,
                'launch_fake_object_pose_node': launch_fake_object_pose_node,
                'params_file': params_file,
            }.items(),
        ),
    ])
