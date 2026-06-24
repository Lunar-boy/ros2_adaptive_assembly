"""Launch the Panda MoveIt2 demo with adapted plan-only pre-grasp planning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start adaptive assembly, Panda demo, adapter, and planning bridge."""
    panda_demo_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_panda_demo.launch.py',
    ])
    default_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_params.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    static_planning_scene_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'static_planning_scene.launch.py',
    ])
    panda_pose_adapter_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'panda_pre_grasp_pose_adapter.launch.py',
    ])
    pre_grasp_planning_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_planning'),
        'launch',
        'pre_grasp_planning.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Parameter YAML for adaptive assembly pipeline nodes.',
        ),
        LogInfo(
            msg='Launching adaptive assembly Panda planning demo: fake '
            'perception, task pose generation, standard Panda MoveIt2 demo, '
            'static PlanningScene collision objects, Panda pre-grasp pose '
            'adapter, and plan-only planning bridge. Execution is disabled.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_demo_launch),
            launch_arguments={'params_file': params_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_planning_scene_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_pose_adapter_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pre_grasp_planning_launch),
        ),
    ])
