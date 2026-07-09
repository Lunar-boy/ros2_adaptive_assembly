"""Preflight checks for simulator-only physical Gazebo grasp verification."""

from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    from ros_gz_interfaces.msg import Contacts as ContactsMessage
except ImportError:  # pragma: no cover - only used without ros_gz_interfaces
    ContactsMessage = String


MODE = 'physical_grasp_preflight'
PHYSICAL_POSE_INFO_TOPIC = (
    '/world/adaptive_assembly_physical_workcell/pose/info'
)
STATIC_VISUAL_POSE_INFO_TOPIC = '/world/adaptive_assembly_workcell/pose/info'


def parse_status(status: str) -> Dict[str, str]:
    """Parse semicolon-delimited key/value status fields."""
    fields = {}
    for fragment in status.split(';'):
        if '=' not in fragment:
            continue
        key, value = fragment.split('=', 1)
        if key.strip():
            fields[key.strip()] = value.strip()
    return fields


def bool_text(value: bool) -> str:
    """Return ROS status-schema compatible boolean text."""
    return 'true' if value else 'false'


class PhysicalGraspPreflightNode(Node):
    """Publish bounded diagnostics for the physical Gazebo grasp path."""

    def __init__(self) -> None:
        super().__init__('physical_grasp_preflight_node')
        defaults = (
            ('pose_info_topic', PHYSICAL_POSE_INFO_TOPIC),
            ('object_pose_available_topic', '/gazebo_target_object_pose_available'),
            ('kinematic_attach_status_topic', '/gazebo_attach_detach_status'),
            ('left_contact_topic', '/panda_leftfinger_contact'),
            ('right_contact_topic', '/panda_rightfinger_contact'),
            ('contact_status_topic', '/grasp_contact_status'),
            ('status_topic', '/physical_grasp_preflight_status'),
            ('timeout_sec', 20.0),
            ('publish_period_sec', 0.1),
            ('simulated_only', True),
                # New: preflight should check infrastructure, not require grasp contact.
            ('require_physical_world', True),
            ('require_object_pose_available', True),
            ('require_no_kinematic_attach', True),
            ('require_contact_topics_observed', False),
            ('require_contact_status_observed', False),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        self._pose_info_topic = str(self.get_parameter('pose_info_topic').value)
        self._object_pose_available_topic = str(
            self.get_parameter('object_pose_available_topic').value
        )
        self._kinematic_attach_status_topic = str(
            self.get_parameter('kinematic_attach_status_topic').value
        )
        self._left_contact_topic = str(
            self.get_parameter('left_contact_topic').value
        )
        self._right_contact_topic = str(
            self.get_parameter('right_contact_topic').value
        )
        self._contact_status_topic = str(
            self.get_parameter('contact_status_topic').value
        )
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._publish_period_sec = float(
            self.get_parameter('publish_period_sec').value
        )
        self._simulated_only = bool(
            self.get_parameter('simulated_only').value
        )
        self._validate_parameters()

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, self._status_topic, retained_qos
        )
        self.create_subscription(
            Bool,
            self._object_pose_available_topic,
            self._object_pose_available_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            self._kinematic_attach_status_topic,
            self._kinematic_attach_status_callback,
            retained_qos,
        )
        self.create_subscription(
            ContactsMessage,
            self._left_contact_topic,
            self._left_contact_callback,
            10,
        )
        self.create_subscription(
            ContactsMessage,
            self._right_contact_topic,
            self._right_contact_callback,
            10,
        )
        self.create_subscription(
            String,
            self._contact_status_topic,
            self._contact_status_callback,
            retained_qos,
        )

        self._start_time = self.get_clock().now()
        self._object_pose_available = False
        self._kinematic_attach_active = False
        self._left_contact_observed = False
        self._right_contact_observed = False
        self._contact_status_observed = False
        self._terminal_event = ''
        self._terminal_reason = ''
        self._timer = self.create_timer(
            self._publish_period_sec, self._publish_status
        )
        self.get_logger().info(
            'Simulator-only physical grasp preflight ready: '
            f"pose_info_topic='{self._pose_info_topic}', "
            f"status_topic='{self._status_topic}', "
            f'timeout_sec={self._timeout_sec:.3f}, '
            'simulated_only=true, real_hardware=false.'
        )
        self._require_physical_world = bool(
            self.get_parameter('require_physical_world').value
        )
        self._require_object_pose_available = bool(
            self.get_parameter('require_object_pose_available').value
        )
        self._require_no_kinematic_attach = bool(
            self.get_parameter('require_no_kinematic_attach').value
        )
        self._require_contact_topics_observed = bool(
            self.get_parameter('require_contact_topics_observed').value
        )
        self._require_contact_status_observed = bool(
            self.get_parameter('require_contact_status_observed').value
        )

    def _validate_parameters(self) -> None:
        if not self._simulated_only:
            raise ValueError('simulated_only_false')
        if self._timeout_sec <= 0.0:
            raise ValueError('timeout_sec must be greater than zero')
        if self._publish_period_sec <= 0.0:
            raise ValueError('publish_period_sec must be greater than zero')

    def _object_pose_available_callback(self, message: Bool) -> None:
        self._object_pose_available = bool(message.data)

    def _kinematic_attach_status_callback(self, message: String) -> None:
        fields = parse_status(message.data)
        event = fields.get('event', '')
        attached = fields.get('attached', '').lower()
        if (
            fields.get('mode') == 'gazebo_attach_detach'
            and (event in ('attached', 'success') or attached == 'true')
        ):
            self._kinematic_attach_active = True

    def _left_contact_callback(self, _) -> None:
        self._left_contact_observed = True

    def _right_contact_callback(self, _) -> None:
        self._right_contact_observed = True

    def _contact_status_callback(self, _) -> None:
        self._contact_status_observed = True

    def _physical_world(self) -> bool:
        return self._pose_info_topic == PHYSICAL_POSE_INFO_TOPIC

    def _elapsed_sec(self) -> float:
        return (
            self.get_clock().now() - self._start_time
        ).nanoseconds * 1.0e-9


    def _failure_reasons(self) -> List[str]:
        reasons = []

        if self._require_physical_world and not self._physical_world():
            reasons.append('wrong_pose_info_topic')

        if (
            self._require_object_pose_available
            and not self._object_pose_available
        ):
            reasons.append('object_pose_unavailable')

        if self._require_no_kinematic_attach and self._kinematic_attach_active:
            reasons.append('kinematic_attach_node_active')

        # These are diagnostics only by default. Requiring actual contact before
        # the arm moves creates a startup deadlock: contact can only appear after
        # motion and gripper closure.
        if self._require_contact_topics_observed:
            if not self._left_contact_observed:
                reasons.append('left_contact_topic_unobserved')
            if not self._right_contact_observed:
                reasons.append('right_contact_topic_unobserved')

        if (
            self._require_contact_status_observed
            and not self._contact_status_observed
        ):
            reasons.append('contact_status_topic_unobserved')

        return reasons

    def _publish_status(self) -> None:
        if not self._terminal_event:
            reasons = self._failure_reasons()
            if not reasons:
                self._terminal_event = 'success'
                self._terminal_reason = 'ok'
            elif self._kinematic_attach_active:
                self._terminal_event = 'failure'
                self._terminal_reason = 'kinematic_attach_node_active'
            elif self._elapsed_sec() >= self._timeout_sec:
                self._terminal_event = 'failure'
                self._terminal_reason = ','.join(reasons)

        event = self._terminal_event or 'waiting'
        reason = self._terminal_reason or 'waiting_for_physical_grasp_inputs'
        message = String()
        message.data = (
            f'event={event};mode={MODE};'
            f'physical_world={bool_text(self._physical_world())};'
            f'object_pose_available={bool_text(self._object_pose_available)};'
            'kinematic_attach_active='
            f'{bool_text(self._kinematic_attach_active)};'
            'left_contact_observed='
            f'{bool_text(self._left_contact_observed)};'
            'right_contact_observed='
            f'{bool_text(self._right_contact_observed)};'
            'contact_status_observed='
            f'{bool_text(self._contact_status_observed)};'
            f'reason={reason};simulated_only=true;real_hardware=false'
        )
        self._status_publisher.publish(message)


def main(args=None) -> None:
    """Run the physical grasp preflight node."""
    rclpy.init(args=args)
    node = PhysicalGraspPreflightNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
