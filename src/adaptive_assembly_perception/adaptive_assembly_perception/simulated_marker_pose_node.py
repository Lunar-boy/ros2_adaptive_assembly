"""Publish a deterministic simulator-only marker pose estimate."""

import math
import random

from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class SimulatedMarkerPoseNode(Node):
    """Emulate marker perception and publish the resulting target pose."""

    def __init__(self) -> None:
        """Declare and validate configuration, then start publishing."""
        super().__init__('simulated_marker_pose_node')
        defaults = {
            'target_pose_topic': '/target_pose',
            'perceived_pose_topic': '/perceived_target_pose',
            'status_topic': '/simulated_vision_perception_status',
            'world_frame': 'world',
            'camera_frame': 'simulated_camera',
            'target_frame_id': 'target_object',
            'marker_id': 0,
            'target_entity_name': 'target_object',
            'publish_period_sec': 1.0,
            'camera_x': 0.0,
            'camera_y': 0.0,
            'camera_z': 1.0,
            'camera_yaw': 0.0,
            'x': 0.45,
            'y': 0.0,
            'z': 0.15,
            'yaw': 0.0,
            'position_noise_std': 0.0,
            'yaw_noise_std': 0.0,
            'publish_immediately': True,
            'enable_camera_topics': False,
            'simulated_only': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self._values = {
            name: self.get_parameter(name).value for name in defaults
        }
        self._validate_parameters()

        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._target_publisher = self.create_publisher(
            PoseStamped, self._values['target_pose_topic'], 10
        )
        perceived_topic = self._values['perceived_pose_topic']
        self._perceived_publisher = (
            self.create_publisher(PoseStamped, perceived_topic, 10)
            if perceived_topic else None
        )
        self._status_publisher = self.create_publisher(
            String, self._values['status_topic'], retained
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        # A stable seed makes configured noise reproducible for a marker ID.
        self._random = random.Random(int(self._values['marker_id']))
        self._timer = self.create_timer(
            float(self._values['publish_period_sec']), self._publish_pose
        )

        if not self._values['enable_camera_topics']:
            self._publish_status(
                'event=skipped;mode=simulated_vision_perception;'
                'reason=camera_topics_disabled;source=marker_pose_emulator;'
                'simulated_only=true;real_hardware=false'
            )
        self.get_logger().info(
            'Simulated vision configured: source=marker_pose_emulator, '
            f"marker_id={self._values['marker_id']}, "
            f"camera_frame={self._values['camera_frame']}, "
            f"target_entity_name={self._values['target_entity_name']}"
        )
        if self._values['publish_immediately']:
            self._publish_pose()

    def _validate_parameters(self) -> None:
        """Reject unsafe or nonsensical simulator configuration."""
        if not self._values['simulated_only']:
            raise ValueError('simulated_only must remain true')
        if float(self._values['publish_period_sec']) <= 0.0:
            raise ValueError('publish_period_sec must be greater than zero')
        if float(self._values['position_noise_std']) < 0.0:
            raise ValueError('position_noise_std must be non-negative')
        if float(self._values['yaw_noise_std']) < 0.0:
            raise ValueError('yaw_noise_std must be non-negative')
        required = (
            'target_pose_topic', 'status_topic', 'world_frame',
            'camera_frame', 'target_frame_id', 'target_entity_name',
        )
        if any(not str(self._values[name]) for name in required):
            raise ValueError('topic, frame, and entity parameters must not be empty')

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)

    def _publish_pose(self) -> None:
        """Publish one emulated observation and matching TF."""
        position_noise = float(self._values['position_noise_std'])
        yaw = float(self._values['yaw']) + self._random.gauss(
            0.0, float(self._values['yaw_noise_std'])
        )
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._values['world_frame']
        pose.pose.position.x = float(self._values['x']) + self._random.gauss(
            0.0, position_noise
        )
        pose.pose.position.y = float(self._values['y']) + self._random.gauss(
            0.0, position_noise
        )
        pose.pose.position.z = float(self._values['z']) + self._random.gauss(
            0.0, position_noise
        )
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._target_publisher.publish(pose)

        if self._perceived_publisher is not None:
            perceived = PoseStamped()
            perceived.header.stamp = pose.header.stamp
            perceived.header.frame_id = self._values['camera_frame']
            camera_yaw = float(self._values['camera_yaw'])
            delta_x = pose.pose.position.x - float(self._values['camera_x'])
            delta_y = pose.pose.position.y - float(self._values['camera_y'])
            perceived.pose.position.x = (
                math.cos(camera_yaw) * delta_x
                + math.sin(camera_yaw) * delta_y
            )
            perceived.pose.position.y = (
                -math.sin(camera_yaw) * delta_x
                + math.cos(camera_yaw) * delta_y
            )
            perceived.pose.position.z = (
                pose.pose.position.z - float(self._values['camera_z'])
            )
            relative_yaw = yaw - camera_yaw
            perceived.pose.orientation.z = math.sin(relative_yaw / 2.0)
            perceived.pose.orientation.w = math.cos(relative_yaw / 2.0)
            self._perceived_publisher.publish(perceived)

        camera_transform = TransformStamped()
        camera_transform.header.stamp = pose.header.stamp
        camera_transform.header.frame_id = self._values['world_frame']
        camera_transform.child_frame_id = self._values['camera_frame']
        camera_transform.transform.translation.x = float(
            self._values['camera_x']
        )
        camera_transform.transform.translation.y = float(
            self._values['camera_y']
        )
        camera_transform.transform.translation.z = float(
            self._values['camera_z']
        )
        camera_yaw = float(self._values['camera_yaw'])
        camera_transform.transform.rotation.z = math.sin(camera_yaw / 2.0)
        camera_transform.transform.rotation.w = math.cos(camera_yaw / 2.0)
        self._tf_broadcaster.sendTransform(camera_transform)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self._values['target_frame_id']
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)
        self._publish_status(
            'event=success;mode=simulated_vision_perception;'
            'source=marker_pose_emulator;'
            f"perceived_frame={self._values['camera_frame']};"
            f"target_frame={self._values['target_frame_id']};"
            'simulated_only=true;real_hardware=false'
        )


def main(args=None) -> None:
    """Run the simulated marker pose node."""
    rclpy.init(args=args)
    node = None
    try:
        node = SimulatedMarkerPoseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
