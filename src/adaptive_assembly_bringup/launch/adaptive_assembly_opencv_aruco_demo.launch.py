"""Compose optional ArUco image perception with the task pipeline."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch a simulator-camera-only detector and task pose generation."""
    detector = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_perception'), 'launch',
        'opencv_aruco_perception.launch.py',
    ])
    task = PathJoinSubstitution([
        FindPackageShare('adaptive_assembly_task'), 'launch',
        'assembly_task.launch.py',
    ])
    return LaunchDescription([
        LogInfo(msg=(
            'Launching optional simulated-image OpenCV ArUco perception and '
            'the existing task pipeline. This launch has no camera hardware, '
            'real robot, or trajectory execution path.')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(detector)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(task)),
    ])
