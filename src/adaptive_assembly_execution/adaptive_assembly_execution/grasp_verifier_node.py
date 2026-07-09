"""Deterministic simulator-only grasp and lift/slip verification."""

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String


MODE = 'grasp_verifier'


@dataclass
class ObjectPosition:
    """Minimal position sample used by deterministic verifier helpers."""

    x: float
    y: float
    z: float


@dataclass
class GraspVerifierState:
    """Pure state for grasp and lift verification checks."""

    gripper_success: bool = False
    gripper_closed: bool = False
    both_contacts: bool = False
    left_contact: bool = False
    right_contact: bool = False
    object_pose_available: bool = False
    baseline: Optional[ObjectPosition] = None
    grasp_verified: bool = False
    lift_verified: bool = False


def parse_status(status: str) -> Dict[str, str]:
    """Parse semicolon-delimited key/value status fields."""
    fields: Dict[str, str] = {}
    for fragment in status.split(';'):
        if '=' not in fragment:
            continue
        key, value = fragment.split('=', 1)
        if key.strip():
            fields[key.strip()] = value.strip()
    return fields


def bool_token(value: bool) -> str:
    return str(value).lower()


def compute_lift_and_slip(
    baseline: ObjectPosition, current: ObjectPosition
) -> Tuple[float, float]:
    """Compute vertical lift and horizontal slip from object positions."""
    lift_delta_m = current.z - baseline.z
    slip_distance_m = math.hypot(current.x - baseline.x, current.y - baseline.y)
    return lift_delta_m, slip_distance_m


def evaluate_grasp_request(
    state: GraspVerifierState,
    current_pose: Optional[ObjectPosition],
    require_both_contacts: bool,
    require_gripper_closed: bool,
    require_object_pose: bool,
    pose_stale: bool,
) -> Tuple[bool, str, Optional[ObjectPosition]]:
    """Evaluate one grasp verification request."""
    if not state.gripper_success:
        return False, 'gripper_not_successful', None
    if require_gripper_closed and not state.gripper_closed:
        return False, 'gripper_not_closed', None
    if require_both_contacts and not state.both_contacts:
        return False, 'missing_both_contacts', None
    if require_object_pose:
        if not state.object_pose_available or current_pose is None:
            return False, 'object_pose_unavailable', None
        if pose_stale:
            return False, 'object_pose_stale', None
    return True, '', current_pose


def evaluate_lift_request(
    baseline: Optional[ObjectPosition],
    current_pose: Optional[ObjectPosition],
    object_pose_available: bool,
    pose_stale: bool,
    min_lift_delta_m: float,
    max_slip_distance_m: float,
) -> Tuple[bool, str, Optional[float], Optional[float]]:
    """Evaluate lift and slip against a stored grasp baseline."""
    if baseline is None:
        return False, 'missing_grasp_baseline', None, None
    if not object_pose_available or current_pose is None:
        return False, 'object_pose_unavailable', None, None
    if pose_stale:
        return False, 'object_pose_stale', None, None
    lift_delta_m, slip_distance_m = compute_lift_and_slip(baseline, current_pose)
    if lift_delta_m < min_lift_delta_m:
        return False, 'insufficient_lift', lift_delta_m, slip_distance_m
    if slip_distance_m > max_slip_distance_m:
        return False, 'slip_too_large', lift_delta_m, slip_distance_m
    return True, '', lift_delta_m, slip_distance_m


class GraspVerifierNode(Node):
    """Verify grasp contacts, gripper state, object lift, and slip."""

    def __init__(self) -> None:
        super().__init__('grasp_verifier_node')
        defaults = (
            ('contact_status_topic', '/grasp_contact_status'),
            ('both_contacts_detected_topic', '/both_gripper_contacts_detected'),
            ('gripper_success_topic', '/physical_gripper_command_success'),
            ('gripper_closed_topic', '/physical_gripper_closed'),
            ('object_pose_topic', '/gazebo_target_object_pose'),
            ('object_pose_available_topic', '/gazebo_target_object_pose_available'),
            ('verifier_request_topic', '/grasp_verification_request'),
            ('verifier_status_topic', '/grasp_verification_status'),
            ('grasp_verified_topic', '/grasp_verified'),
            ('lift_verified_topic', '/lift_verified'),
            ('slip_distance_mm_topic', '/grasp_slip_distance_mm'),
            ('require_both_contacts', True),
            ('require_gripper_closed', True),
            ('require_object_pose', True),
            ('min_lift_delta_m', 0.02),
            ('max_slip_distance_m', 0.025),
            ('pose_stale_timeout_sec', 1.0),
            ('simulated_only', True),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        self._contact_status_topic = str(
            self.get_parameter('contact_status_topic').value
        )
        self._both_contacts_topic = str(
            self.get_parameter('both_contacts_detected_topic').value
        )
        self._gripper_success_topic = str(
            self.get_parameter('gripper_success_topic').value
        )
        self._gripper_closed_topic = str(
            self.get_parameter('gripper_closed_topic').value
        )
        self._object_pose_topic = str(
            self.get_parameter('object_pose_topic').value
        )
        self._object_pose_available_topic = str(
            self.get_parameter('object_pose_available_topic').value
        )
        self._request_topic = str(
            self.get_parameter('verifier_request_topic').value
        )
        self._status_topic = str(
            self.get_parameter('verifier_status_topic').value
        )
        self._grasp_verified_topic = str(
            self.get_parameter('grasp_verified_topic').value
        )
        self._lift_verified_topic = str(
            self.get_parameter('lift_verified_topic').value
        )
        self._slip_mm_topic = str(
            self.get_parameter('slip_distance_mm_topic').value
        )
        self._require_both_contacts = bool(
            self.get_parameter('require_both_contacts').value
        )
        self._require_gripper_closed = bool(
            self.get_parameter('require_gripper_closed').value
        )
        self._require_object_pose = bool(
            self.get_parameter('require_object_pose').value
        )
        self._min_lift_delta_m = float(
            self.get_parameter('min_lift_delta_m').value
        )
        self._max_slip_distance_m = float(
            self.get_parameter('max_slip_distance_m').value
        )
        self._pose_stale_timeout_sec = float(
            self.get_parameter('pose_stale_timeout_sec').value
        )
        self._simulated_only = bool(self.get_parameter('simulated_only').value)
        self._validate_parameters()

        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, self._status_topic, retained
        )
        self._grasp_verified_pub = self.create_publisher(
            Bool, self._grasp_verified_topic, retained
        )
        self._lift_verified_pub = self.create_publisher(
            Bool, self._lift_verified_topic, retained
        )
        self._slip_mm_pub = self.create_publisher(
            Float64, self._slip_mm_topic, retained
        )
        self.create_subscription(
            String, self._contact_status_topic,
            self._contact_status_callback, retained
        )
        self.create_subscription(
            Bool, self._both_contacts_topic,
            self._both_contacts_callback, retained
        )
        self.create_subscription(
            Bool, self._gripper_success_topic,
            self._gripper_success_callback, retained
        )
        self.create_subscription(
            Bool, self._gripper_closed_topic,
            self._gripper_closed_callback, retained
        )
        self.create_subscription(
            Bool, self._object_pose_available_topic,
            self._object_pose_available_callback, retained
        )
        self.create_subscription(
            PoseStamped, self._object_pose_topic,
            self._object_pose_callback, 10
        )
        self.create_subscription(
            String, self._request_topic, self._request_callback, 10
        )

        self._state = GraspVerifierState()
        self._current_pose: Optional[ObjectPosition] = None
        self._last_pose_stamp = None
        self.get_logger().info(
            'Grasp verifier ready: simulator-only contact/lift/slip checks '
            f'min_lift_delta_m={self._min_lift_delta_m:.6f}, '
            f'max_slip_distance_m={self._max_slip_distance_m:.6f}, '
            'simulated_only=true, real_hardware=false.'
        )

    def _validate_parameters(self) -> None:
        if not self._simulated_only:
            raise ValueError('simulated_only_false')
        if self._min_lift_delta_m < 0.0:
            raise ValueError('min_lift_delta_m must be greater than or equal to zero')
        if self._max_slip_distance_m < 0.0:
            raise ValueError('max_slip_distance_m must be greater than or equal to zero')
        if self._pose_stale_timeout_sec <= 0.0:
            raise ValueError('pose_stale_timeout_sec must be greater than zero')

    def _contact_status_callback(self, message: String) -> None:
        fields = parse_status(message.data)
        self._state.left_contact = fields.get('left_contact') == 'true'
        self._state.right_contact = fields.get('right_contact') == 'true'
        self._state.both_contacts = fields.get('both_contacts') == 'true'

    def _both_contacts_callback(self, message: Bool) -> None:
        self._state.both_contacts = bool(message.data)

    def _gripper_success_callback(self, message: Bool) -> None:
        self._state.gripper_success = bool(message.data)

    def _gripper_closed_callback(self, message: Bool) -> None:
        self._state.gripper_closed = bool(message.data)

    def _object_pose_available_callback(self, message: Bool) -> None:
        self._state.object_pose_available = bool(message.data)

    def _object_pose_callback(self, message: PoseStamped) -> None:
        position = message.pose.position
        self._current_pose = ObjectPosition(position.x, position.y, position.z)
        self._state.object_pose_available = True
        self._last_pose_stamp = self.get_clock().now()

    def _request_callback(self, message: String) -> None:
        fields = parse_status(message.data)
        if fields.get('real_hardware') == 'true':
            self._publish_status(
                'failure', fields.get('verification', 'unknown'),
                fields.get('stage', 'unknown'), 'unsupported_request'
            )
            return
        event = fields.get('event', '')
        if event == 'reset':
            self._reset(fields.get('reason', 'requested'))
            return
        if event != 'request':
            self._publish_status(
                'failure', fields.get('verification', 'unknown'),
                fields.get('stage', 'unknown'), 'unsupported_request'
            )
            return
        verification = fields.get('verification', '')
        stage = fields.get('stage', verification)
        if verification == 'grasp':
            self._handle_grasp_request(stage)
            return
        if verification == 'lift':
            self._handle_lift_request(stage)
            return
        self._publish_status(
            'failure', verification or 'unknown', stage, 'unsupported_request'
        )

    def _pose_is_stale(self) -> bool:
        if self._last_pose_stamp is None:
            return True
        age = (self.get_clock().now() - self._last_pose_stamp).nanoseconds / 1e9
        return age > self._pose_stale_timeout_sec

    def _handle_grasp_request(self, stage: str) -> None:
        ok, reason, baseline = evaluate_grasp_request(
            self._state,
            self._current_pose,
            self._require_both_contacts,
            self._require_gripper_closed,
            self._require_object_pose,
            self._pose_is_stale(),
        )
        self._state.grasp_verified = ok
        self._state.lift_verified = False
        if ok:
            self._state.baseline = baseline
        self._grasp_verified_pub.publish(Bool(data=ok))
        self._publish_status(
            'success' if ok else 'failure', 'grasp', stage, reason
        )

    def _handle_lift_request(self, stage: str) -> None:
        ok, reason, lift_delta_m, slip_distance_m = evaluate_lift_request(
            self._state.baseline if self._state.grasp_verified else None,
            self._current_pose,
            self._state.object_pose_available,
            self._pose_is_stale(),
            self._min_lift_delta_m,
            self._max_slip_distance_m,
        )
        self._state.lift_verified = ok
        self._lift_verified_pub.publish(Bool(data=ok))
        if slip_distance_m is not None:
            self._slip_mm_pub.publish(Float64(data=slip_distance_m * 1000.0))
        self._publish_status(
            'success' if ok else 'failure',
            'lift',
            stage,
            reason,
            lift_delta_m,
            slip_distance_m,
        )

    def _reset(self, reason: str) -> None:
        self._state.baseline = None
        self._state.grasp_verified = False
        self._state.lift_verified = False
        self._grasp_verified_pub.publish(Bool(data=False))
        self._lift_verified_pub.publish(Bool(data=False))
        self._publish_status('reset', 'grasp', 'reset', reason)

    def _publish_status(
        self,
        event: str,
        verification: str,
        stage: str,
        reason: str = '',
        lift_delta_m: Optional[float] = None,
        slip_distance_m: Optional[float] = None,
    ) -> None:
        fields = [
            f'event={event}',
            f'mode={MODE}',
            f'verification={verification}',
            f'stage={stage}',
            f'grasp_verified={bool_token(self._state.grasp_verified)}',
            f'lift_verified={bool_token(self._state.lift_verified)}',
            f'left_contact={bool_token(self._state.left_contact)}',
            f'right_contact={bool_token(self._state.right_contact)}',
            f'both_contacts={bool_token(self._state.both_contacts)}',
            f'gripper_closed={bool_token(self._state.gripper_closed)}',
            f'object_pose_available={bool_token(self._state.object_pose_available)}',
        ]
        if lift_delta_m is not None:
            fields.append(f'lift_delta_m={lift_delta_m:.6f}')
        if slip_distance_m is not None:
            fields.append(f'slip_distance_m={slip_distance_m:.6f}')
        if reason:
            fields.append(f'reason={reason}')
        fields.extend(['simulated_only=true', 'real_hardware=false'])
        self._status_pub.publish(String(data=';'.join(fields)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspVerifierNode()
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
