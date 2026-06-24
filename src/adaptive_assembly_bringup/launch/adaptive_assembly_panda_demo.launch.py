"""Launch adaptive assembly topics with the standard Panda MoveIt2 demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
    panda_demo_launch = PathJoinSubstitution([
        FindPackageShare('moveit_resources_panda_moveit_config'),
        'launch',
        'demo.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Parameter YAML for adaptive assembly pipeline nodes.',
        ),
        LogInfo(
            msg='Launching adaptive assembly Panda demo: fake perception, '
            'task pose generation, and the standard Panda MoveIt2 demo.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(adaptive_pipeline_launch),
            launch_arguments={'params_file': params_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(panda_demo_launch),
        ),
    ])
