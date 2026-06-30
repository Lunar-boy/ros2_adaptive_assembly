"""Launch the static adaptive assembly workcell in Gazebo Harmonic."""

import os
import shutil

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _required_package_share(package_name: str) -> str:
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError as error:
        raise RuntimeError(
            f"Required package '{package_name}' is unavailable. Install "
            'Gazebo integration with: sudo apt install ros-jazzy-ros-gz-sim'
        ) from error


def generate_launch_description() -> LaunchDescription:
    """Start Gazebo Sim 8 with the installed workcell world."""
    sim_share = _required_package_share('adaptive_assembly_sim')
    ros_gz_sim_share = _required_package_share('ros_gz_sim')
    if shutil.which('gz') is None:
        raise RuntimeError(
            "Gazebo's 'gz' executable is unavailable. Install Gazebo Harmonic "
            'and ros-jazzy-ros-gz-sim before launching this workcell.'
        )

    default_world = os.path.join(
        sim_share, 'worlds', 'adaptive_assembly_workcell.sdf'
    )
    gazebo_launch = os.path.join(
        ros_gz_sim_share, 'launch', 'gz_sim.launch.py'
    )
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo SDF world.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value=['-r ', world],
            description=(
                'Arguments passed to Gazebo Sim. The default starts the '
                'workcell immediately with the graphical client.'
            ),
        ),
        LogInfo(
            msg=[
                'Launching Gazebo Harmonic workcell from ',
                world,
                '. This is simulation visualization only; no robot or '
                'trajectory execution is started.',
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': gz_args,
                'gz_version': '8',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
    ])
