"""Sweep physical grasp heights against the canonical Panda collision model."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Run one deterministic collision-mesh calibration sweep."""
    panda_xacro = str(
        Path(get_package_share_directory('adaptive_assembly_sim'))
        / 'urdf' / 'panda.urdf.xacro'
    )
    config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path=panda_xacro)
        .robot_description_semantic(file_path='config/panda.srdf')
        .robot_description_kinematics()
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(
            package='adaptive_assembly_planning',
            executable='grasp_clearance_calibration_node',
            output='screen',
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
                {
                    'minimum_offset': 0.005,
                    'maximum_offset': 0.030,
                    'offset_step': 0.001,
                    'minimum_clearance': 0.005,
                    'target_x': 0.350,
                    'target_y': 0.180,
                    'target_z': 0.100,
                    'target_radius': 0.035,
                    'target_height': 0.10,
                },
            ],
        ),
    ])
