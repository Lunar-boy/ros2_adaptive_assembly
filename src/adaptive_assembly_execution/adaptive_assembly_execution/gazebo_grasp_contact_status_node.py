"""Publish simulator-only gripper contact status from Gazebo contacts."""

from typing import Any, Iterable, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    from ros_gz_interfaces.msg import Contacts as ContactsMessage
except ImportError:  # pragma: no cover - only used without ros_gz_interfaces
    ContactsMessage = String


MODE = 'gazebo_grasp_contact_status'


def _stringify(value: Any) -> str:
    return '' if value is None else str(value)


def _iter_contact_candidates(message: Any) -> Iterable[Any]:
    contacts = getattr(message, 'contacts', None)
    if contacts is not None:
        yield from contacts
        return
    contact = getattr(message, 'contact', None)
    if contact is not None:
        if isinstance(contact, (list, tuple)):
            yield from contact
        else:
            yield contact
        return
    if isinstance(message, str):
        for fragment in message.splitlines():
            if fragment.strip():
                yield fragment
        return
    yield message


def _collision_names_from_contact(contact: Any) -> List[str]:
    names: List[str] = []
    if isinstance(contact, str):
        text = contact
        for key in ('collision1', 'collision2', 'collision_1', 'collision_2'):
            marker = f'{key}:'
            if marker in text:
                value = text.split(marker, 1)[1].splitlines()[0]
                names.append(value.strip().strip('"'))
        if not names:
            names.append(text)
        return names

    for attr in (
        'collision1',
        'collision2',
        'collision_1',
        'collision_2',
        'collision1_name',
        'collision2_name',
        'collision_1_name',
        'collision_2_name',
    ):
        value = getattr(contact, attr, None)
        if value is not None:
            names.append(_stringify(value))

    for nested_attr in ('wrench', 'position', 'normal', 'depth'):
        nested = getattr(contact, nested_attr, None)
        if nested is not None and not names:
            continue
    return names


def contact_message_has_target_contact(
    message: Any, target_object_name: str, require_target_object_contact: bool
) -> Tuple[bool, str]:
    """Return whether a Gazebo contact message proves target contact."""
    candidates = list(_iter_contact_candidates(message))
    if not candidates:
        return False, 'no_contact'

    saw_supported_shape = False
    for contact in candidates:
        names = _collision_names_from_contact(contact)
        if not names:
            continue
        saw_supported_shape = True
        if not require_target_object_contact:
            return True, ''
        if any(target_object_name in name for name in names):
            return True, ''

    if not saw_supported_shape:
        return False, 'unsupported_contact_message'
    if require_target_object_contact:
        return False, 'no_target_object_contact'
    return True, ''


class GazeboGraspContactStatusNode(Node):
    """Aggregate left and right Gazebo contact sensor messages."""

    def __init__(self) -> None:
        super().__init__('gazebo_grasp_contact_status_node')
        defaults = (
            ('left_contact_topic', '/panda_leftfinger_contact'),
            ('right_contact_topic', '/panda_rightfinger_contact'),
            ('target_object_name', 'target_object'),
            ('left_contact_status_topic', '/left_gripper_contact_status'),
            ('right_contact_status_topic', '/right_gripper_contact_status'),
            ('aggregate_contact_status_topic', '/grasp_contact_status'),
            ('left_contact_detected_topic', '/left_gripper_contact_detected'),
            ('right_contact_detected_topic', '/right_gripper_contact_detected'),
            ('both_contacts_detected_topic', '/both_gripper_contacts_detected'),
            ('contact_stale_timeout_sec', 0.5),
            ('publish_period_sec', 0.05),
            ('require_target_object_contact', True),
            ('simulated_only', True),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        self._left_topic = str(self.get_parameter('left_contact_topic').value)
        self._right_topic = str(self.get_parameter('right_contact_topic').value)
        self._target = str(self.get_parameter('target_object_name').value)
        self._left_status_topic = str(
            self.get_parameter('left_contact_status_topic').value
        )
        self._right_status_topic = str(
            self.get_parameter('right_contact_status_topic').value
        )
        self._aggregate_topic = str(
            self.get_parameter('aggregate_contact_status_topic').value
        )
        self._left_detected_topic = str(
            self.get_parameter('left_contact_detected_topic').value
        )
        self._right_detected_topic = str(
            self.get_parameter('right_contact_detected_topic').value
        )
        self._both_detected_topic = str(
            self.get_parameter('both_contacts_detected_topic').value
        )
        self._stale_timeout = float(
            self.get_parameter('contact_stale_timeout_sec').value
        )
        self._publish_period = float(
            self.get_parameter('publish_period_sec').value
        )
        self._require_target = bool(
            self.get_parameter('require_target_object_contact').value
        )
        self._simulated_only = bool(
            self.get_parameter('simulated_only').value
        )
        self._validate_parameters()

        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._left_status_pub = self.create_publisher(
            String, self._left_status_topic, retained
        )
        self._right_status_pub = self.create_publisher(
            String, self._right_status_topic, retained
        )
        self._aggregate_pub = self.create_publisher(
            String, self._aggregate_topic, retained
        )
        self._left_detected_pub = self.create_publisher(
            Bool, self._left_detected_topic, retained
        )
        self._right_detected_pub = self.create_publisher(
            Bool, self._right_detected_topic, retained
        )
        self._both_detected_pub = self.create_publisher(
            Bool, self._both_detected_topic, retained
        )
        self.create_subscription(
            ContactsMessage, self._left_topic, self._left_callback, 10
        )
        self.create_subscription(
            ContactsMessage, self._right_topic, self._right_callback, 10
        )
        self._left_contact = False
        self._right_contact = False
        self._left_reason = 'no_left_contact'
        self._right_reason = 'no_right_contact'
        self._left_stamp = None
        self._right_stamp = None
        self.create_timer(self._publish_period, self._publish_status)
        self.get_logger().info(
            'Gazebo grasp contact status node ready: '
            f'left={self._left_topic}, right={self._right_topic}, '
            f'target_object_name={self._target}, simulated_only=true, '
            'real_hardware=false.'
        )

    def _validate_parameters(self) -> None:
        if not self._simulated_only:
            raise ValueError('simulated_only_false')
        if self._stale_timeout <= 0.0:
            raise ValueError('contact_stale_timeout_sec must be greater than zero')
        if self._publish_period <= 0.0:
            raise ValueError('publish_period_sec must be greater than zero')

    def _left_callback(self, message: Any) -> None:
        self._left_contact, self._left_reason = (
            contact_message_has_target_contact(
                message, self._target, self._require_target
            )
        )
        self._left_stamp = self.get_clock().now()

    def _right_callback(self, message: Any) -> None:
        self._right_contact, self._right_reason = (
            contact_message_has_target_contact(
                message, self._target, self._require_target
            )
        )
        self._right_stamp = self.get_clock().now()

    def _side_is_stale(self, stamp: Optional[Any]) -> bool:
        if stamp is None:
            return True
        age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
        return age > self._stale_timeout

    def _publish_status(self) -> None:
        left_stale = self._side_is_stale(self._left_stamp)
        right_stale = self._side_is_stale(self._right_stamp)
        left_contact = self._left_contact and not left_stale
        right_contact = self._right_contact and not right_stale
        both = left_contact and right_contact

        left_reason = 'left_contact_stale' if left_stale else self._left_reason
        right_reason = (
            'right_contact_stale' if right_stale else self._right_reason
        )
        self._left_detected_pub.publish(Bool(data=left_contact))
        self._right_detected_pub.publish(Bool(data=right_contact))
        self._both_detected_pub.publish(Bool(data=both))
        self._left_status_pub.publish(String(data=self._format_side_status(
            'left', left_contact, left_reason
        )))
        self._right_status_pub.publish(String(data=self._format_side_status(
            'right', right_contact, right_reason
        )))
        event = 'success' if both else 'waiting'
        reason = ''
        if not left_contact:
            reason = left_reason or 'no_left_contact'
        elif not right_contact:
            reason = right_reason or 'no_right_contact'
        fields = [
            f'event={event}',
            f'mode={MODE}',
            f'left_contact={str(left_contact).lower()}',
            f'right_contact={str(right_contact).lower()}',
            f'both_contacts={str(both).lower()}',
            f'target_object_name={self._target}',
        ]
        if reason:
            fields.append(f'reason={reason}')
        fields.extend(['simulated_only=true', 'real_hardware=false'])
        self._aggregate_pub.publish(String(data=';'.join(fields)))

    def _format_side_status(
        self, side: str, contact: bool, reason: str
    ) -> str:
        fields = [
            'event=success' if contact else 'event=waiting',
            f'mode={MODE}',
            f'{side}_contact={str(contact).lower()}',
            f'target_object_name={self._target}',
        ]
        if not contact:
            fields.append(f'reason={reason}')
        fields.extend(['simulated_only=true', 'real_hardware=false'])
        return ';'.join(fields)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboGraspContactStatusNode()
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
