"""Publish randomized target poses for the adaptive assembly simulation."""

import math
import random

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeObjectPoseNode(Node):
    """Publish a fake object pose and its matching TF transform."""

    def __init__(self) -> None:
        """Initialize parameters, publisher, TF broadcaster, and timer."""
        super().__init__('fake_object_pose_node')

        self.declare_parameter('publish_period_sec', 5.0)
        self.declare_parameter('x_min', 0.35)
        self.declare_parameter('x_max', 0.55)
        self.declare_parameter('y_min', -0.25)
        self.declare_parameter('y_max', 0.25)
        self.declare_parameter('z', 0.15)
        self.declare_parameter('random_seed', -1)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('target_frame_id', 'target_object')
        self.declare_parameter('yaw_min', -math.pi)
        self.declare_parameter('yaw_max', math.pi)
        self.declare_parameter('publish_immediately', True)

        self._publish_period_sec = (
            self.get_parameter('publish_period_sec').get_parameter_value()
            .double_value
        )
        self._x_min = self.get_parameter('x_min').value
        self._x_max = self.get_parameter('x_max').value
        self._y_min = self.get_parameter('y_min').value
        self._y_max = self.get_parameter('y_max').value
        self._z = self.get_parameter('z').value
        self._random_seed = self.get_parameter('random_seed').value
        self._frame_id = self.get_parameter('frame_id').value
        self._target_frame_id = self.get_parameter('target_frame_id').value
        self._yaw_min = self.get_parameter('yaw_min').value
        self._yaw_max = self.get_parameter('yaw_max').value
        self._publish_immediately = self.get_parameter('publish_immediately').value
        self._validate_parameters()
        self._random_generator = (
            random.Random(self._random_seed)
            if self._random_seed >= 0
            else random.Random()
        )

        self._pose_publisher = self.create_publisher(
            PoseStamped, '/target_pose', 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(
            self._publish_period_sec, self._publish_target_pose
        )

        self._log_startup_configuration()

        if self._publish_immediately:
            self._publish_target_pose()

    def _validate_parameters(self) -> None:
        """Reject parameter combinations that cannot produce valid poses."""
        if self._publish_period_sec <= 0.0:
            raise ValueError('publish_period_sec must be greater than zero')
        if self._x_min > self._x_max:
            raise ValueError('x_min must be less than or equal to x_max')
        if self._y_min > self._y_max:
            raise ValueError('y_min must be less than or equal to y_max')
        if self._yaw_min > self._yaw_max:
            raise ValueError('yaw_min must be less than or equal to yaw_max')
        if not self._frame_id:
            raise ValueError('frame_id must not be empty')
        if not self._target_frame_id:
            raise ValueError('target_frame_id must not be empty')

    def _log_startup_configuration(self) -> None:
        """Log fake perception configuration for reproducible runs."""
        deterministic = self._random_seed >= 0
        seed_text = str(self._random_seed) if deterministic else 'none'
        self.get_logger().info(
            'Fake object pose node configured: '
            f'deterministic={str(deterministic).lower()}, '
            f'random_seed={seed_text}, '
            f'frame_id={self._frame_id}, '
            f'target_frame_id={self._target_frame_id}, '
            f'x_range=[{self._x_min:.3f}, {self._x_max:.3f}], '
            f'y_range=[{self._y_min:.3f}, {self._y_max:.3f}], '
            f'yaw_range=[{self._yaw_min:.3f}, {self._yaw_max:.3f}], '
            f'publish_immediately={str(self._publish_immediately).lower()}'
        )

    def _publish_target_pose(self) -> None:
        """Publish a randomized pose and the equivalent TF transform."""
        stamp = self.get_clock().now().to_msg()
        yaw = self._random_generator.uniform(self._yaw_min, self._yaw_max)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = self._random_generator.uniform(
            self._x_min, self._x_max
        )
        pose.pose.position.y = self._random_generator.uniform(
            self._y_min, self._y_max
        )
        pose.pose.position.z = self._z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._pose_publisher.publish(pose)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._frame_id
        transform.child_frame_id = self._target_frame_id
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        self.get_logger().info(
            'Published target pose at '
            f'x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}, '
            f'z={pose.pose.position.z:.3f}'
        )


def main(args=None) -> None:
    """Run the fake object pose node."""
    rclpy.init(args=args)
    node = FakeObjectPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
