"""Adapt an observed Gazebo model-center pose for the task pipeline."""

import math
from typing import Any

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


def validate_adapter_parameters(
    input_pose_topic: str,
    output_pose_topic: str,
    target_reference_z_offset: float,
    output_frame_id: str,
) -> None:
    """Reject adapter settings that cannot produce a valid ROS pose."""
    for name, value in (
        ('input_pose_topic', input_pose_topic),
        ('output_pose_topic', output_pose_topic),
        ('output_frame_id', output_frame_id),
    ):
        if not value.strip():
            raise ValueError(f'{name} must not be empty')
    if not math.isfinite(target_reference_z_offset):
        raise ValueError('target_reference_z_offset must be finite')


def pose_is_valid(message: PoseStamped) -> bool:
    """Return whether all pose fields are finite with a usable quaternion."""
    pose = message.pose
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    norm_squared = sum(value * value for value in values[3:])
    return norm_squared > 0.0


def adapt_target_pose(
    source: PoseStamped,
    target_reference_z_offset: float,
    output_frame_id: str,
    output_stamp: Any,
) -> PoseStamped:
    """Copy a valid source pose and apply the task-reference adaptation."""
    if not math.isfinite(target_reference_z_offset):
        raise ValueError('target_reference_z_offset must be finite')
    if not output_frame_id.strip():
        raise ValueError('output_frame_id must not be empty')
    if not pose_is_valid(source):
        raise ValueError(
            'source pose must be finite with a nonzero quaternion'
        )

    adapted_z = source.pose.position.z + target_reference_z_offset
    if not math.isfinite(adapted_z):
        raise ValueError('adapted target Z position must be finite')

    output = PoseStamped()
    output.header.stamp = output_stamp
    output.header.frame_id = output_frame_id
    output.pose.position.x = source.pose.position.x
    output.pose.position.y = source.pose.position.y
    output.pose.position.z = adapted_z
    output.pose.orientation.x = source.pose.orientation.x
    output.pose.orientation.y = source.pose.orientation.y
    output.pose.orientation.z = source.pose.orientation.z
    output.pose.orientation.w = source.pose.orientation.w
    return output


class GazeboTargetPoseAdapterNode(Node):
    """Publish Gazebo target observations as task-compatible target poses."""

    def __init__(self, **kwargs: Any) -> None:
        """Configure validated topics and the model-center Z adaptation."""
        super().__init__('gazebo_target_pose_adapter_node', **kwargs)
        defaults = (
            ('input_pose_topic', '/gazebo_target_object_pose'),
            ('output_pose_topic', '/target_pose'),
            ('target_reference_z_offset', 0.05),
            ('output_frame_id', 'world'),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        self._input_topic = str(
            self.get_parameter('input_pose_topic').value
        )
        self._output_topic = str(
            self.get_parameter('output_pose_topic').value
        )
        self._z_offset = float(
            self.get_parameter('target_reference_z_offset').value
        )
        self._output_frame = str(
            self.get_parameter('output_frame_id').value
        )
        validate_adapter_parameters(
            self._input_topic,
            self._output_topic,
            self._z_offset,
            self._output_frame,
        )

        self._publisher = self.create_publisher(
            PoseStamped, self._output_topic, 10
        )
        self._subscription = self.create_subscription(
            PoseStamped, self._input_topic, self._on_source_pose, 10
        )
        self._published_once = False
        self.get_logger().info(
            'Gazebo target pose adapter configured: '
            f'input={self._input_topic}, output={self._output_topic}, '
            f'target_reference_z_offset={self._z_offset:.6f}, '
            f'output_frame={self._output_frame}. output_frame_id is a '
            'frame-label override only; no TF coordinate transform is applied.'
        )

    def _on_source_pose(self, source: PoseStamped) -> None:
        if not pose_is_valid(source):
            self.get_logger().warning(
                'Skipping invalid Gazebo target pose: pose values must be '
                'finite and the quaternion must be nonzero.',
                throttle_duration_sec=5.0,
            )
            return

        output = adapt_target_pose(
            source,
            self._z_offset,
            self._output_frame,
            self.get_clock().now().to_msg(),
        )
        self._publisher.publish(output)
        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                'Published first Gazebo-derived task target pose.'
            )
        else:
            self.get_logger().debug(
                'Published Gazebo-derived task target pose.'
            )


def main(args=None) -> None:
    """Run the Gazebo target-pose adapter."""
    rclpy.init(args=args)
    node = GazeboTargetPoseAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
