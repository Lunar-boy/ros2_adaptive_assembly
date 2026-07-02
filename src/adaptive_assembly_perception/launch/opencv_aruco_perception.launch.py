"""Launch optional simulator-only OpenCV ArUco perception."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Expose detector topics, marker model, and camera transform."""
    defaults = {
        'image_topic': '/camera/image_raw',
        'camera_info_topic': '/camera/camera_info',
        'target_pose_topic': '/target_pose',
        'perceived_pose_topic': '/perceived_target_pose',
        'status_topic': '/aruco_detection_status',
        'marker_detected_topic': '/aruco_marker_detected',
        'world_frame': 'world',
        'camera_frame': 'simulated_camera',
        'target_frame_id': 'target_object',
        'marker_id': '0',
        'marker_size_m': '0.05',
        'aruco_dictionary': 'DICT_4X4_50',
        'camera_x': '0.0', 'camera_y': '0.0', 'camera_z': '1.0',
        'camera_yaw': '0.0',
        'publish_tf': 'true',
        'fallback_to_emulator': 'true',
        'detection_timeout_sec': '2.0',
        'simulated_only': 'true',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        Node(
            package='adaptive_assembly_perception',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[{name: LaunchConfiguration(name)
                         for name in defaults}],
        ),
    ])
