"""Publish one Gazebo entity pose as a ROS ``PoseStamped``."""

from typing import Any, Optional, Tuple, Type

from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

try:
    from ros_gz_interfaces.msg import Pose_V as GazeboPoseVector
    INPUT_MESSAGE_TYPE: Type[Any] = GazeboPoseVector
    INPUT_MESSAGE_NAME = 'ros_gz_interfaces/msg/Pose_V'
except ImportError:
    # ros_gz_interfaces 1.x in ROS 2 Jazzy bridges gz.msgs.Pose_V to
    # TFMessage rather than exposing a ROS Pose_V message.
    from tf2_msgs.msg import TFMessage
    INPUT_MESSAGE_TYPE = TFMessage
    INPUT_MESSAGE_NAME = 'tf2_msgs/msg/TFMessage'


def _entity_matches(name: str, target: str, exact: bool) -> bool:
    """Match exact names, or scoped Gazebo names when explicitly allowed."""
    if name == target:
        return True
    return not exact and name.replace('/', '::').split('::')[-1] == target


def extract_entity_pose(
    message: Any, target: str, exact: bool
) -> Tuple[Optional[Pose], Optional[str]]:
    """Extract an entity pose from supported Pose_V or TFMessage shapes."""
    poses = getattr(message, 'pose', None)
    if poses is not None:
        for candidate in poses:
            name = str(getattr(candidate, 'name', ''))
            if not _entity_matches(name, target, exact):
                continue
            position = getattr(candidate, 'position', None)
            orientation = getattr(candidate, 'orientation', None)
            if position is None or orientation is None:
                return None, 'unsupported_pose_structure'
            pose = Pose()
            pose.position = position
            pose.orientation = orientation
            return pose, None
        return None, 'entity_not_found'

    transforms = getattr(message, 'transforms', None)
    if transforms is not None:
        for candidate in transforms:
            name = str(getattr(candidate, 'child_frame_id', ''))
            if not _entity_matches(name, target, exact):
                continue
            transform = getattr(candidate, 'transform', None)
            if transform is None:
                return None, 'unsupported_pose_structure'
            translation = getattr(transform, 'translation', None)
            rotation = getattr(transform, 'rotation', None)
            if translation is None or rotation is None:
                return None, 'unsupported_pose_structure'
            pose = Pose()
            pose.position.x = translation.x
            pose.position.y = translation.y
            pose.position.z = translation.z
            pose.orientation = rotation
            return pose, None
        return None, 'entity_not_found'

    return None, 'unsupported_pose_structure'


class GazeboEntityPoseObserverNode(Node):
    """Observe and periodically republish one simulator entity pose."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('gazebo_entity_pose_observer_node', **kwargs)
        defaults = (
            ('pose_info_topic', '/world/adaptive_assembly_workcell/pose/info'),
            ('target_entity_name', 'target_object'),
            ('world_frame', 'world'),
            ('output_pose_topic', '/gazebo_target_object_pose'),
            ('status_topic', '/gazebo_target_object_pose_status'),
            ('available_topic', '/gazebo_target_object_pose_available'),
            ('pose_age_ms_topic', '/gazebo_target_object_pose_age_ms'),
            ('stale_timeout_sec', 2.0),
            ('publish_period_sec', 0.1),
            ('require_model_name_match', True),
            ('simulated_only', True),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        self._source_topic = str(self.get_parameter('pose_info_topic').value)
        self._entity = str(self.get_parameter('target_entity_name').value)
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._output_topic = str(self.get_parameter('output_pose_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._available_topic = str(self.get_parameter('available_topic').value)
        self._age_topic = str(self.get_parameter('pose_age_ms_topic').value)
        self._stale_timeout = float(
            self.get_parameter('stale_timeout_sec').value)
        self._publish_period = float(
            self.get_parameter('publish_period_sec').value)
        self._exact_match = bool(
            self.get_parameter('require_model_name_match').value)
        self._simulated_only = bool(
            self.get_parameter('simulated_only').value)
        self._validate_parameters()

        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pose_publisher = self.create_publisher(
            PoseStamped, self._output_topic, 10)
        self._status_publisher = self.create_publisher(
            String, self._status_topic, retained)
        self._available_publisher = self.create_publisher(
            Bool, self._available_topic, retained)
        self._age_publisher = self.create_publisher(
            Float64, self._age_topic, retained)
        self._subscription = self.create_subscription(
            INPUT_MESSAGE_TYPE, self._source_topic, self._on_pose_vector, 10)
        self._started_at = self.get_clock().now()
        self._last_stream_at = None
        self._latest_pose = None
        self._timer = self.create_timer(
            self._publish_period, self._on_publish_timer)
        self.get_logger().info(
            'Gazebo entity pose observer configured: '
            f'entity={self._entity}, source={self._source_topic}, '
            f'input_type={INPUT_MESSAGE_NAME}, output={self._output_topic}, '
            f'world_frame={self._world_frame}, simulated_only=true, '
            'real_hardware=false')

    def _validate_parameters(self) -> None:
        if not self._simulated_only:
            raise ValueError('simulated_only:=false is not supported')
        for name, value in (
            ('pose_info_topic', self._source_topic),
            ('target_entity_name', self._entity),
            ('world_frame', self._world_frame),
            ('output_pose_topic', self._output_topic),
            ('status_topic', self._status_topic),
            ('available_topic', self._available_topic),
            ('pose_age_ms_topic', self._age_topic),
        ):
            if not value:
                raise ValueError(f'{name} must not be empty')
        if self._stale_timeout <= 0.0:
            raise ValueError('stale_timeout_sec must be greater than zero')
        if self._publish_period <= 0.0:
            raise ValueError('publish_period_sec must be greater than zero')

    def _on_pose_vector(self, message: Any) -> None:
        self._last_stream_at = self.get_clock().now()
        pose, reason = extract_entity_pose(
            message, self._entity, self._exact_match)
        if pose is None:
            self._latest_pose = None
            self._publish_skipped(reason or 'unsupported_pose_structure')
            return
        self._latest_pose = pose
        self._publish_pose()

    def _on_publish_timer(self) -> None:
        now = self.get_clock().now()
        reference = self._last_stream_at or self._started_at
        age_sec = (now - reference).nanoseconds / 1e9
        if age_sec > self._stale_timeout:
            self._latest_pose = None
            self._publish_skipped('pose_stream_stale')
            return
        if self._latest_pose is not None:
            self._publish_pose()

    def _publish_pose(self) -> None:
        now = self.get_clock().now()
        output = PoseStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self._world_frame
        output.pose = self._latest_pose
        self._pose_publisher.publish(output)
        self._available_publisher.publish(Bool(data=True))
        age_ms = 0.0
        if self._last_stream_at is not None:
            age_ms = (now - self._last_stream_at).nanoseconds / 1e6
        self._age_publisher.publish(Float64(data=max(0.0, age_ms)))
        self._publish_status('success')

    def _publish_skipped(self, reason: str) -> None:
        self._available_publisher.publish(Bool(data=False))
        self._publish_status('skipped', reason)

    def _publish_status(
        self, event: str, reason: Optional[str] = None
    ) -> None:
        fields = [
            f'event={event}',
            'mode=gazebo_entity_pose_observer',
        ]
        if reason is not None:
            fields.append(f'reason={reason}')
        fields.extend([
            f'entity={self._entity}',
            f'source_topic={self._source_topic}',
        ])
        if event == 'success':
            fields.append(f'output_topic={self._output_topic}')
        fields.extend(['simulated_only=true', 'real_hardware=false'])
        self._status_publisher.publish(String(data=';'.join(fields)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboEntityPoseObserverNode()
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
