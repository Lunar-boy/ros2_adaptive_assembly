"""Launch the deterministic recovery supervisor and retry orchestrator."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose the existing supervisor demo with simulated orchestration."""
    supervisor_demo = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_bringup'),
        'launch',
        'adaptive_assembly_recovery_supervisor_demo.launch.py',
    ])
    return LaunchDescription([
        LogInfo(msg=(
            'Launching deterministic simulator-only recovery orchestration. '
            'Retries reset logical scene state and republish fake perception; '
            'real hardware and trajectory execution are disabled.'
        )),
        Node(
            package='adaptive_assembly_recovery',
            executable='recovery_orchestrator_node',
            name='recovery_orchestrator_node',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(supervisor_demo),
            launch_arguments={'enable_service_calls': 'false'}.items(),
        ),
    ])
