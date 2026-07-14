"""Collect independent Gazebo finger contacts and publish normalized status."""

from dataclasses import dataclass
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


def _entity_name(value: Any) -> str:
    """Return an Entity name while retaining compatibility with plain strings."""
    if value is None:
        return ''
    name = getattr(value, 'name', None)
    return _stringify(name if name is not None else value)


def scoped_entity_tokens(entity_name: str) -> Tuple[str, ...]:
    """Split a Gazebo scoped entity name into exact non-empty tokens."""
    return tuple(token.strip() for token in entity_name.split('::') if token.strip())


def entity_name_matches_model(entity_name: str, model_name: str) -> bool:
    """
    Match the model field of a Gazebo scoped collision name.

    Gazebo collision names end in ``model::link::collision`` and may have one
    or more world or nested-model prefixes. The model is therefore the third
    token from the end. A bare model name is also accepted for normalized
    model-only observations. This rejects substrings and target-named links.
    """
    expected = model_name.strip()
    tokens = scoped_entity_tokens(entity_name)
    if not expected or not tokens:
        return False
    if len(tokens) == 1:
        return tokens[0] == expected
    if len(tokens) < 3:
        return False
    return tokens[-3] == expected


def entity_name_has_token(entity_name: str, token: str) -> bool:
    """Return whether an exact scope token identifies a configured finger."""
    expected = token.strip()
    return bool(expected and expected in scoped_entity_tokens(entity_name))


@dataclass(frozen=True)
class ContactObservation:
    """Normalized contents of one finger contact-sensor message."""

    target_contact: bool
    wrong_object_contact: bool
    entities: Tuple[str, ...]
    reason: str


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
            name = _entity_name(value)
            if name:
                names.append(name)

    for nested_attr in ('wrench', 'position', 'normal', 'depth'):
        nested = getattr(contact, nested_attr, None)
        if nested is not None and not names:
            continue
    return names


def analyze_contact_message(
    message: Any,
    target_object_name: str,
    finger_entity_name: str,
    require_target_object_contact: bool,
) -> ContactObservation:
    """
    Classify target and unrelated contacts from one dedicated sensor.

    A contact pair containing the configured finger token and an exact target
    model token is target contact. Any additional contact pair that lacks the
    target is conservatively classified as wrong-object contact.
    """
    candidates = list(_iter_contact_candidates(message))
    if not candidates:
        return ContactObservation(False, False, (), 'no_contact')

    saw_supported_shape = False
    target_contact = False
    wrong_object_contact = False
    contacted_entities = set()
    for contact in candidates:
        names = tuple(
            name for name in _collision_names_from_contact(contact) if name
        )
        if not names:
            continue
        saw_supported_shape = True
        external_names = tuple(
            name for name in names
            if not entity_name_has_token(name, finger_entity_name)
        )
        contacted_entities.update(external_names)
        if not require_target_object_contact:
            target_contact = True
            continue
        pair_has_target = any(
            entity_name_matches_model(name, target_object_name)
            for name in external_names
        )
        if pair_has_target:
            target_contact = True
        elif external_names:
            wrong_object_contact = True

    if not saw_supported_shape:
        return ContactObservation(
            False, False, (), 'unsupported_contact_message'
        )
    if wrong_object_contact:
        reason = 'wrong_object_contact'
    elif require_target_object_contact and not target_contact:
        reason = 'no_target_object_contact'
    else:
        reason = ''
    return ContactObservation(
        target_contact,
        wrong_object_contact,
        tuple(sorted(contacted_entities)),
        reason,
    )


def contact_message_has_target_contact(
    message: Any, target_object_name: str, require_target_object_contact: bool
) -> Tuple[bool, str]:
    """Compatibility wrapper returning target contact and reason."""
    observation = analyze_contact_message(
        message,
        target_object_name,
        '',
        require_target_object_contact,
    )
    valid = observation.target_contact and not observation.wrong_object_contact
    return valid, observation.reason


class GazeboGraspContactStatusNode(Node):
    """Aggregate left and right Gazebo contact sensor messages."""

    def __init__(self) -> None:
        """Declare contact topics and initialize independent finger state."""
        super().__init__('gazebo_grasp_contact_status_node')
        defaults = (
            ('left_contact_topic', '/panda_leftfinger_contact'),
            ('right_contact_topic', '/panda_rightfinger_contact'),
            ('target_object_name', 'target_object'),
            ('left_finger_entity_name', 'panda_leftfinger'),
            ('right_finger_entity_name', 'panda_rightfinger'),
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
        self._left_finger = str(
            self.get_parameter('left_finger_entity_name').value
        )
        self._right_finger = str(
            self.get_parameter('right_finger_entity_name').value
        )
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
        self._left_observation = ContactObservation(
            False, False, (), 'no_left_contact'
        )
        self._right_observation = ContactObservation(
            False, False, (), 'no_right_contact'
        )
        self._left_reason = 'no_left_contact'
        self._right_reason = 'no_right_contact'
        self._left_stamp = None
        self._right_stamp = None
        self._left_sensor_stamp_ns: Optional[int] = None
        self._right_sensor_stamp_ns: Optional[int] = None
        self._previous_transition_state = None
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
        if self._require_target and not self._target:
            raise ValueError(
                'target_object_name must not be empty when target contact is required'
            )
        if not self._left_finger or not self._right_finger:
            raise ValueError('finger entity names must not be empty')

    def _left_callback(self, message: Any) -> None:
        self._left_observation = analyze_contact_message(
            message, self._target, self._left_finger, self._require_target
        )
        self._left_reason = self._left_observation.reason
        self._left_stamp = self.get_clock().now()
        self._left_sensor_stamp_ns = self._message_stamp_ns(message)

    def _right_callback(self, message: Any) -> None:
        self._right_observation = analyze_contact_message(
            message, self._target, self._right_finger, self._require_target
        )
        self._right_reason = self._right_observation.reason
        self._right_stamp = self.get_clock().now()
        self._right_sensor_stamp_ns = self._message_stamp_ns(message)

    @staticmethod
    def _message_stamp_ns(message: Any) -> Optional[int]:
        header = getattr(message, 'header', None)
        stamp = getattr(header, 'stamp', None)
        if stamp is None:
            return None
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return stamp_ns if stamp_ns > 0 else None

    def _side_age_sec(self, stamp: Optional[Any]) -> Optional[float]:
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1.0e9

    def _publish_status(self) -> None:
        now = self.get_clock().now()
        left_age = self._side_age_sec(self._left_stamp)
        right_age = self._side_age_sec(self._right_stamp)
        left_stale = (
            left_age is None or left_age < 0.0 or left_age > self._stale_timeout
        )
        right_stale = (
            right_age is None or right_age < 0.0 or right_age > self._stale_timeout
        )
        left_contact = (
            self._left_observation.target_contact
            and not self._left_observation.wrong_object_contact
            and not left_stale
        )
        right_contact = (
            self._right_observation.target_contact
            and not self._right_observation.wrong_object_contact
            and not right_stale
        )
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
        if (
            not left_stale and self._left_observation.wrong_object_contact
        ) or (
            not right_stale and self._right_observation.wrong_object_contact
        ):
            reason = 'wrong_object_contact'
        elif not left_contact:
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
            'left_message_received='
            f'{str(self._left_stamp is not None).lower()}',
            'right_message_received='
            f'{str(self._right_stamp is not None).lower()}',
            'left_target_contact='
            f'{str(self._left_observation.target_contact).lower()}',
            'right_target_contact='
            f'{str(self._right_observation.target_contact).lower()}',
            'left_wrong_object_contact='
            f'{str(self._left_observation.wrong_object_contact).lower()}',
            'right_wrong_object_contact='
            f'{str(self._right_observation.wrong_object_contact).lower()}',
            f'left_contact_age_sec={self._format_age(left_age)}',
            f'right_contact_age_sec={self._format_age(right_age)}',
            f'left_receipt_stamp_ns={self._stamp_ns(self._left_stamp)}',
            f'right_receipt_stamp_ns={self._stamp_ns(self._right_stamp)}',
            f'left_sensor_stamp_ns={self._left_sensor_stamp_ns or 0}',
            f'right_sensor_stamp_ns={self._right_sensor_stamp_ns or 0}',
            'left_contact_entities='
            f'{",".join(self._left_observation.entities)}',
            'right_contact_entities='
            f'{",".join(self._right_observation.entities)}',
            f'status_stamp_ns={now.nanoseconds}',
        ]
        if reason:
            fields.append(f'reason={reason}')
        fields.extend(['simulated_only=true', 'real_hardware=false'])
        self._aggregate_pub.publish(String(data=';'.join(fields)))
        self._log_transition(left_contact, right_contact, both, reason)

    @staticmethod
    def _format_age(age: Optional[float]) -> str:
        return 'inf' if age is None else f'{age:.6f}'

    @staticmethod
    def _stamp_ns(stamp: Optional[Any]) -> int:
        return 0 if stamp is None else int(stamp.nanoseconds)

    def _log_transition(
        self, left: bool, right: bool, both: bool, reason: str
    ) -> None:
        state = (left, right, both, reason)
        previous = self._previous_transition_state
        if state == previous:
            return
        self._previous_transition_state = state
        if previous is not None:
            if left and not previous[0]:
                self.get_logger().info('left target contact acquired')
            if right and not previous[1]:
                self.get_logger().info('right target contact acquired')
            if previous[2] and not both:
                self.get_logger().warning('bilateral target contact lost')
        if reason == 'wrong_object_contact':
            self.get_logger().warning('wrong-object finger contact detected')

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
    """Run the Gazebo contact collector."""
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
