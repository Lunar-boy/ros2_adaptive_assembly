"""Launch the Gazebo workcell with the optional topic-level pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose the static Gazebo workcell and non-MoveIt pipeline."""
    workcell_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'launch',
        'adaptive_assembly_workcell.launch.py',
    ])
    pipeline_launch = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_pipeline.launch.py',
    ])
    default_world = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_sim'),
        'worlds',
        'adaptive_assembly_workcell.sdf',
    ])
    default_params_file = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'config',
        'adaptive_assembly_params.yaml',
    ])

    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    launch_pipeline = LaunchConfiguration('launch_pipeline')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo SDF world.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value=['-r ', world],
            description='Arguments passed to Gazebo Sim.',
        ),
        DeclareLaunchArgument(
            'launch_pipeline',
            default_value='true',
            description=(
                'Launch fake perception and task pose generation alongside '
                'the static workcell.'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Parameter YAML for the optional adaptive pipeline.',
        ),
        LogInfo(
            msg=(
                'Launching the Gazebo workcell demo. The environment is '
                'static and no robot controller or execution bridge is '
                'included.'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(workcell_launch),
            launch_arguments={'world': world, 'gz_args': gz_args}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pipeline_launch),
            condition=IfCondition(launch_pipeline),
            launch_arguments={'params_file': params_file}.items(),
        ),
    ])
