"""Bridge gripper commands to contact-aware simulator trajectory execution."""

import math
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from adaptive_assembly_manipulation.contact_aware_gripper import (
    ActionTerminalState,
    BilateralContactSnapshot,
    BilateralContactValidator,
    ContactAssessment,
    ContactState,
    evaluate_close_result,
    FingerContactSample,
    GripperCloseOutcome,
    GripperCloseResult,
)
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectoryPoint


PANDA_FINGER_JOINTS = (
    'panda_finger_joint1',
    'panda_finger_joint2',
)


def validate_simulator_joint_names(joint_names) -> list[str]:
    """Return the exact ordered simulator Panda finger-joint contract."""
    names = [str(name) for name in joint_names]
    if not names or any(not name for name in names):
        raise ValueError('joint_names must contain non-empty names')
    if len(names) != len(set(names)):
        raise ValueError('joint_names must not contain duplicates')
    if tuple(names) != PANDA_FINGER_JOINTS:
        raise ValueError(
            'simulator joint_names must be exactly '
            f'{list(PANDA_FINGER_JOINTS)!r} in that order'
        )
    return names


def make_gripper_goal(
    joint_names: list[str], position: float, goal_time_sec: float
) -> FollowJointTrajectory.Goal:
    """Build one equal-position trajectory goal for both Panda fingers."""
    point = JointTrajectoryPoint()
    point.positions = [position] * len(joint_names)
    point.time_from_start.sec = int(goal_time_sec)
    point.time_from_start.nanosec = int(
        (goal_time_sec - int(goal_time_sec)) * 1_000_000_000
    )
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joint_names
    goal.trajectory.points = [point]
    return goal


def parse_status(status: str) -> Dict[str, str]:
    """Parse semicolon-delimited key/value fields, ignoring invalid items."""
    fields = {}
    for fragment in status.split(';'):
        if '=' not in fragment:
            continue
        key, value = fragment.split('=', 1)
        if key.strip():
            fields[key.strip()] = value.strip()
    return fields


def _bool_field(fields: Dict[str, str], name: str) -> bool:
    return fields.get(name, '').lower() == 'true'


def _optional_int(value: str) -> Optional[int]:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


def _entities(value: str) -> Tuple[str, ...]:
    return tuple(item for item in value.split(',') if item)


def contact_snapshot_from_status(fields: Dict[str, str]) -> BilateralContactSnapshot:
    """Build a typed bilateral snapshot from the contact collector schema."""
    def side(name: str) -> FingerContactSample:
        return FingerContactSample(
            received=_bool_field(fields, f'{name}_message_received'),
            receipt_stamp_ns=_optional_int(
                fields.get(f'{name}_receipt_stamp_ns', '')
            ),
            sensor_stamp_ns=_optional_int(
                fields.get(f'{name}_sensor_stamp_ns', '')
            ),
            target_contact=_bool_field(fields, f'{name}_target_contact'),
            wrong_object_contact=_bool_field(
                fields, f'{name}_wrong_object_contact'
            ),
            entities=_entities(fields.get(f'{name}_contact_entities', '')),
        )

    return BilateralContactSnapshot(left=side('left'), right=side('right'))


def action_state_from_status(status: int) -> ActionTerminalState:
    """Map action_msgs terminal status constants to a typed state."""
    if status == GoalStatus.STATUS_SUCCEEDED:
        return ActionTerminalState.SUCCEEDED
    if status == GoalStatus.STATUS_ABORTED:
        return ActionTerminalState.ABORTED
    if status == GoalStatus.STATUS_CANCELED:
        return ActionTerminalState.CANCELED
    return ActionTerminalState.UNKNOWN


class GripperActionBridgeNode(Node):
    """Send finger trajectories and classify physical close completion."""

    def __init__(self) -> None:
        """Declare parameters and initialize action/contact state."""
        super().__init__('gripper_action_bridge_node')
        defaults = {
            'command_topic': '/gripper_command',
            'controller_action_name': (
                '/panda_gripper_controller/follow_joint_trajectory'
            ),
            'status_topic': '/physical_gripper_command_status',
            'success_topic': '/physical_gripper_command_success',
            'closed_topic': '/physical_gripper_closed',
            'contact_status_topic': '/grasp_contact_status',
            'expected_target_object': 'target_object',
            'joint_names': list(PANDA_FINGER_JOINTS),
            'open_position': 0.04,
            'close_position': 0.0,
            'goal_time_sec': 1.0,
            'wait_for_controller_sec': 5.0,
            'result_timeout_sec': 5.0,
            'contact_wait_timeout_sec': 1.0,
            'contact_freshness_timeout_sec': 0.25,
            'contact_settle_duration_sec': 0.20,
            'allow_contact_limited_close': False,
            'send_goals': True,
            'simulated_only': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self._load_parameters()
        self._validate_parameters()
        self._active_command: Optional[str] = None
        self._active_command_id = ''
        self._operation_counter = 0
        self._operation_start_ns: Optional[int] = None
        self._operation_deadline_ns: Optional[int] = None
        self._goal_accepted = False
        self._goal_handle = None
        self._result_timer = None
        self._contact_wait_timer = None
        self._contact_wait_deadline_ns: Optional[int] = None
        self._pending_action_state: Optional[ActionTerminalState] = None
        self._pending_action_status_code = GoalStatus.STATUS_UNKNOWN
        self._pending_error_code: Optional[int] = None
        self._pending_error_string = ''
        self._latest_finger_positions: Tuple[float, ...] = ()
        self._contact_validator = BilateralContactValidator(
            self._contact_freshness_timeout_sec,
            self._contact_settle_duration_sec,
        )
        self._last_contact_state: Optional[ContactState] = None

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, self.get_parameter('status_topic').value, retained_qos
        )
        self._success_publisher = self.create_publisher(
            Bool, self.get_parameter('success_topic').value, retained_qos
        )
        self._closed_publisher = self.create_publisher(
            Bool, self.get_parameter('closed_topic').value, retained_qos
        )
        self.create_subscription(
            String,
            self.get_parameter('command_topic').value,
            self._command_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('contact_status_topic').value,
            self._contact_status_callback,
            retained_qos,
        )
        self._action_client = ActionClient(
            self, FollowJointTrajectory, self._action_name
        )
        self._publish_status(
            'event=ready;mode=gripper_action_bridge;'
            f'controller={self._action_name}'
        )
        self.get_logger().info(
            'Simulator-only gripper action bridge ready: '
            f"controller='{self._action_name}', send_goals={self._send_goals}, "
            f'joint_names={self._joint_names}, '
            'allow_contact_limited_close='
            f'{self._allow_contact_limited_close}, '
            f"expected_target='{self._expected_target_object}'"
        )

    def _load_parameters(self) -> None:
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._action_name = str(
            self.get_parameter('controller_action_name').value
        )
        self._send_goals = bool(self.get_parameter('send_goals').value)
        self._open_position = float(self.get_parameter('open_position').value)
        self._close_position = float(self.get_parameter('close_position').value)
        self._goal_time_sec = float(self.get_parameter('goal_time_sec').value)
        self._wait_for_controller_sec = float(
            self.get_parameter('wait_for_controller_sec').value
        )
        self._result_timeout_sec = float(
            self.get_parameter('result_timeout_sec').value
        )
        self._contact_wait_timeout_sec = float(
            self.get_parameter('contact_wait_timeout_sec').value
        )
        self._contact_freshness_timeout_sec = float(
            self.get_parameter('contact_freshness_timeout_sec').value
        )
        self._contact_settle_duration_sec = float(
            self.get_parameter('contact_settle_duration_sec').value
        )
        self._allow_contact_limited_close = bool(
            self.get_parameter('allow_contact_limited_close').value
        )
        self._expected_target_object = str(
            self.get_parameter('expected_target_object').value
        ).strip()
        self._simulated_only = bool(
            self.get_parameter('simulated_only').value
        )

    def _validate_parameters(self) -> None:
        if not self._simulated_only:
            raise ValueError(
                'simulated_only must remain true; real hardware is not supported'
            )
        self._joint_names = validate_simulator_joint_names(self._joint_names)
        if not all(math.isfinite(value) for value in (
            self._open_position,
            self._close_position,
            self._goal_time_sec,
            self._wait_for_controller_sec,
            self._result_timeout_sec,
            self._contact_wait_timeout_sec,
            self._contact_freshness_timeout_sec,
            self._contact_settle_duration_sec,
        )):
            raise ValueError('gripper positions and durations must be finite')
        if self._goal_time_sec < 0.0 or self._wait_for_controller_sec < 0.0:
            raise ValueError('goal and controller wait durations must be nonnegative')
        if self._result_timeout_sec <= 0.0:
            raise ValueError('result_timeout_sec must be greater than zero')
        if self._contact_wait_timeout_sec < 0.0:
            raise ValueError('contact_wait_timeout_sec must be nonnegative')
        if self._contact_freshness_timeout_sec < 0.0:
            raise ValueError('contact_freshness_timeout_sec must be nonnegative')
        if self._contact_settle_duration_sec < 0.0:
            raise ValueError('contact_settle_duration_sec must be nonnegative')
        if self._allow_contact_limited_close:
            if self._contact_freshness_timeout_sec <= 0.0:
                raise ValueError(
                    'contact_freshness_timeout_sec must be positive when '
                    'contact-limited close is enabled'
                )
            if not self._expected_target_object:
                raise ValueError(
                    'expected_target_object must not be empty when '
                    'contact-limited close is enabled'
                )
            if self._contact_settle_duration_sec > min(
                self._contact_wait_timeout_sec,
                self._result_timeout_sec,
            ):
                raise ValueError(
                    'contact_settle_duration_sec must not exceed the contact '
                    'wait or gripper action timeout'
                )

    def _command_callback(self, message: String) -> None:
        fields = parse_status(message.data)
        if fields.get('event') != 'command':
            return
        command = fields.get('command', '')
        if command not in ('open', 'close'):
            self._publish_failure(
                GripperCloseResult.INTERNAL_ERROR,
                command=command or 'missing',
                reason='unknown_command',
            )
            return
        if self._active_command is not None:
            self._publish_failure(
                GripperCloseResult.INTERNAL_ERROR,
                command=command,
                reason='busy',
            )
            return
        requested_target = fields.get('expected_target_object', '')
        if (
            command == 'close'
            and requested_target
            and requested_target != self._expected_target_object
        ):
            self._publish_failure(
                GripperCloseResult.INTERNAL_ERROR,
                command=command,
                reason='expected_target_mismatch',
                requested_target=requested_target,
                expected_target_object=self._expected_target_object,
            )
            return

        self._operation_counter += 1
        self._active_command = command
        self._active_command_id = fields.get(
            'command_id', str(self._operation_counter)
        )
        now_ns = self.get_clock().now().nanoseconds
        self._operation_start_ns = now_ns
        self._operation_deadline_ns = (
            now_ns + int(self._result_timeout_sec * 1.0e9)
        )
        self._goal_accepted = False
        self._pending_action_state = None
        self._latest_finger_positions = ()
        self._contact_validator.start_operation(now_ns)
        self._last_contact_state = None

        position = (
            self._open_position if command == 'open' else self._close_position
        )
        if command == 'close':
            self.get_logger().info(
                'gripper close started: '
                f'position={position:.6f} '
                f'target={self._expected_target_object} '
                f'action_timeout_sec={self._result_timeout_sec:.3f} '
                'contact_freshness_timeout_sec='
                f'{self._contact_freshness_timeout_sec:.3f} '
                f'settle_sec={self._contact_settle_duration_sec:.3f} '
                'allow_contact_limited_close='
                f'{str(self._allow_contact_limited_close).lower()}'
            )

        if not self._send_goals:
            self._publish_normal_success(command, send_goals=False)
            self._finish_command()
            return

        self._publish_status(self._command_status('waiting'))
        if not self._action_client.wait_for_server(
            timeout_sec=self._wait_for_controller_sec
        ):
            self._publish_failure(
                GripperCloseResult.INTERNAL_ERROR,
                command=command,
                reason='controller_unavailable',
            )
            self._finish_command()
            return

        goal = make_gripper_goal(
            self._joint_names, position, self._goal_time_sec
        )
        self._publish_status(self._command_status('sending'))
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        command = self._active_command or 'unknown'
        try:
            goal_handle = future.result()
        except Exception as error:  # action transport errors are runtime failures
            self.get_logger().error(f'Gripper goal request failed: {error}')
            self._publish_failure(
                GripperCloseResult.INTERNAL_ERROR,
                command=command,
                reason='goal_request_failed',
            )
            self._finish_command()
            return
        if not goal_handle.accepted:
            self._publish_failure(
                GripperCloseResult.GOAL_REJECTED,
                command=command,
                reason=GripperCloseResult.GOAL_REJECTED.value,
                goal_accepted='false',
            )
            self._finish_command()
            return

        self._goal_handle = goal_handle
        self._goal_accepted = True
        self._publish_status(
            self._command_status('accepted', goal_accepted='true')
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        remaining_sec = self._remaining_operation_sec()
        if remaining_sec <= 0.0:
            self._result_timeout_callback()
            return
        self._result_timer = self.create_timer(
            remaining_sec, self._result_timeout_callback
        )

    def _feedback_callback(self, feedback_message) -> None:
        actual = getattr(feedback_message.feedback, 'actual', None)
        positions = tuple(getattr(actual, 'positions', ()))
        if positions and all(math.isfinite(value) for value in positions):
            self._latest_finger_positions = positions

    def _result_callback(self, future) -> None:
        if self._active_command is None:
            return
        self._cancel_result_timer()
        command = self._active_command
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
            result = wrapped_result.result
            error_code = int(result.error_code)
            error_string = str(result.error_string)
        except Exception as error:  # action transport errors are runtime failures
            self.get_logger().error(f'Gripper result request failed: {error}')
            self._publish_failure(
                GripperCloseResult.INTERNAL_ERROR,
                command=command,
                reason='result_error',
            )
            self._finish_command()
            return

        action_state = action_state_from_status(status)
        if (
            action_state == ActionTerminalState.SUCCEEDED
            and error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            action_state = ActionTerminalState.UNKNOWN
        if self._remaining_operation_sec() <= 0.0:
            self._complete_action_result(
                ActionTerminalState.TIMEOUT, status, error_code, error_string
            )
            return
        if command == 'open':
            if (
                action_state == ActionTerminalState.SUCCEEDED
                and error_code == FollowJointTrajectory.Result.SUCCESSFUL
            ):
                self._publish_normal_success(command, send_goals=True, status=status)
            else:
                result_class = {
                    ActionTerminalState.CANCELED: GripperCloseResult.ACTION_CANCELED,
                    ActionTerminalState.ABORTED: GripperCloseResult.ACTION_ABORTED,
                }.get(action_state, GripperCloseResult.INTERNAL_ERROR)
                self._publish_failure(
                    result_class,
                    command=command,
                    reason=result_class.value,
                    goal_accepted='true',
                    action_status=action_state.value,
                    action_status_code=status,
                    action_error_code=error_code,
                )
            self._finish_command()
            return

        if (
            action_state == ActionTerminalState.ABORTED
            and self._allow_contact_limited_close
            and error_code
            == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        ):
            self._pending_action_state = action_state
            self._pending_action_status_code = status
            self._pending_error_code = error_code
            self._pending_error_string = error_string
            assessment = self._contact_validator.assess(
                self.get_clock().now().nanoseconds
            )
            if assessment.state == ContactState.BILATERAL_SETTLED:
                self._complete_action_result(
                    action_state, status, error_code, error_string
                )
                return
            if assessment.state == ContactState.WRONG_OBJECT:
                self._complete_action_result(
                    action_state, status, error_code, error_string
                )
                return
            wait_sec = min(
                self._contact_wait_timeout_sec,
                self._remaining_operation_sec(),
            )
            if wait_sec <= 0.0:
                self._complete_action_result(
                    action_state, status, error_code, error_string
                )
                return
            now_ns = self.get_clock().now().nanoseconds
            self._contact_wait_deadline_ns = now_ns + int(wait_sec * 1.0e9)
            self._publish_status(self._command_status(
                'waiting_for_contact',
                goal_accepted='true',
                action_status=action_state.value,
                action_error_code=error_code,
                contact_state=assessment.state.value,
            ))
            self._contact_wait_timer = self.create_timer(
                min(0.05, wait_sec), self._contact_wait_timer_callback
            )
            return

        self._complete_action_result(
            action_state, status, error_code, error_string
        )

    def _contact_status_callback(self, message: String) -> None:
        fields = parse_status(message.data)
        if fields.get('mode') != 'gazebo_grasp_contact_status':
            return
        if self._active_command != 'close':
            return
        expected = fields.get('target_object_name', '')
        if expected != self._expected_target_object:
            return
        assessment = self._contact_validator.update(
            contact_snapshot_from_status(fields),
            self.get_clock().now().nanoseconds,
        )
        self._log_contact_transition(assessment)
        if self._pending_action_state is None:
            return
        if assessment.state in (
            ContactState.BILATERAL_SETTLED,
            ContactState.WRONG_OBJECT,
        ):
            self._complete_action_result(
                self._pending_action_state,
                self._pending_action_status_code,
                self._pending_error_code,
                self._pending_error_string,
            )

    def _log_contact_transition(self, assessment: ContactAssessment) -> None:
        current = assessment.state
        previous = self._last_contact_state
        if current == previous:
            return
        self._last_contact_state = current
        if current == ContactState.BILATERAL_SETTLING:
            self.get_logger().info('bilateral target contact settling started')
        elif current == ContactState.BILATERAL_SETTLED:
            self.get_logger().info('bilateral target contact settled')
        elif current == ContactState.WRONG_OBJECT:
            self.get_logger().warning('wrong-object contact detected')
        elif previous in (
            ContactState.BILATERAL_SETTLING,
            ContactState.BILATERAL_SETTLED,
        ):
            self.get_logger().warning('bilateral contact lost; settling reset')

    def _contact_wait_timer_callback(self) -> None:
        if self._pending_action_state is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        assessment = self._contact_validator.assess(now_ns)
        self._log_contact_transition(assessment)
        deadline = self._contact_wait_deadline_ns or now_ns
        if assessment.state in (
            ContactState.BILATERAL_SETTLED,
            ContactState.WRONG_OBJECT,
        ) or now_ns >= deadline or self._remaining_operation_sec() <= 0.0:
            self._complete_action_result(
                self._pending_action_state,
                self._pending_action_status_code,
                self._pending_error_code,
                self._pending_error_string,
            )

    def _complete_action_result(
        self,
        action_state: ActionTerminalState,
        action_status_code: int,
        error_code: Optional[int],
        error_string: str,
    ) -> None:
        if self._active_command != 'close':
            return
        self._cancel_contact_wait_timer()
        now_ns = self.get_clock().now().nanoseconds
        assessment = self._contact_validator.assess(now_ns)
        outcome = evaluate_close_result(
            goal_accepted=self._goal_accepted,
            action_state=action_state,
            action_error_code=error_code,
            goal_tolerance_error_code=(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            ),
            contact=assessment,
            allow_contact_limited_close=self._allow_contact_limited_close,
            expected_target_object=self._expected_target_object,
            finger_positions=self._latest_finger_positions,
            action_error_string=error_string,
        )
        self._publish_outcome(outcome, action_status_code)
        self._finish_command()

    def _publish_outcome(
        self, outcome: GripperCloseOutcome, action_status_code: int
    ) -> None:
        contact = outcome.contact
        fields = {
            'result': outcome.result.value,
            'command': 'close',
            'command_id': self._active_command_id,
            'goal_accepted': str(outcome.goal_accepted).lower(),
            'action_status': outcome.action_state.value,
            'action_status_code': action_status_code,
            'action_error_code': outcome.action_error_code,
            'action_error_string': self._sanitize_field(
                outcome.action_error_string
            ),
            'expected_target_object': outcome.expected_target_object,
            'left_contact': str(contact.left_target_contact).lower(),
            'right_contact': str(contact.right_target_contact).lower(),
            'left_contact_age_sec': self._format_optional(contact.left_age_sec),
            'right_contact_age_sec': self._format_optional(contact.right_age_sec),
            'left_contact_entities': ','.join(contact.left_entities),
            'right_contact_entities': ','.join(contact.right_entities),
            'settle_sec': f'{contact.settle_duration_sec:.6f}',
            'finger_positions': ','.join(
                f'{position:.6f}' for position in outcome.finger_positions
            ),
        }
        if outcome.succeeded:
            self._publish_status(self._format_status('success', fields))
            self._publish_bool(self._success_publisher, True)
            self._publish_bool(self._closed_publisher, True)
            log = self.get_logger().info
            prefix = 'gripper close completed'
        else:
            fields['reason'] = outcome.failure_reason
            self._publish_status(self._format_status('failure', fields))
            self._publish_bool(self._success_publisher, False)
            log = self.get_logger().warning
            prefix = 'gripper close failed'
        log(
            f'{prefix}: result={outcome.result.value} '
            f'target={outcome.expected_target_object} '
            f'left_contact={str(contact.left_target_contact).lower()} '
            f'right_contact={str(contact.right_target_contact).lower()} '
            f'settle_sec={contact.settle_duration_sec:.3f} '
            f'action_status={outcome.action_state.value}'
        )

    def _result_timeout_callback(self) -> None:
        if self._active_command is None:
            return
        command = self._active_command
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        if command == 'close':
            self._complete_action_result(
                ActionTerminalState.TIMEOUT,
                GoalStatus.STATUS_UNKNOWN,
                None,
                '',
            )
            return
        self._publish_failure(
            GripperCloseResult.ACTION_TIMEOUT,
            command=command,
            reason=GripperCloseResult.ACTION_TIMEOUT.value,
            goal_accepted=str(self._goal_accepted).lower(),
            action_status=ActionTerminalState.TIMEOUT.value,
        )
        self._finish_command()

    def _publish_normal_success(
        self, command: str, *, send_goals: bool, status: int = 0
    ) -> None:
        self._publish_status(self._format_status('success', {
            'command': command,
            'send_goals': str(send_goals).lower(),
            'result': GripperCloseResult.SUCCESS.value,
            'command_id': self._active_command_id,
            'goal_accepted': str(self._goal_accepted).lower(),
            'action_status': (
                ActionTerminalState.SUCCEEDED.value if send_goals else 'dry_run'
            ),
            'action_status_code': status,
        }))
        self._publish_bool(self._success_publisher, True)
        self._publish_bool(self._closed_publisher, command == 'close')

    def _publish_failure(
        self, result: GripperCloseResult, **fields
    ) -> None:
        details = {'result': result.value, **fields}
        if self._active_command_id:
            details.setdefault('command_id', self._active_command_id)
        self._publish_status(self._format_status('failure', details))
        self._publish_bool(self._success_publisher, False)

    def _command_status(self, event: str, **fields) -> str:
        return self._format_status(event, {
            'command': self._active_command or 'unknown',
            'command_id': self._active_command_id,
            **fields,
        })

    @staticmethod
    def _format_status(event: str, fields: Dict[str, object]) -> str:
        details = ';'.join(
            f'{key}={value}' for key, value in fields.items()
            if value is not None
        )
        return f'event={event};{details}'

    @staticmethod
    def _format_optional(value: Optional[float]) -> str:
        return 'inf' if value is None else f'{value:.6f}'

    @staticmethod
    def _sanitize_field(value: str) -> str:
        return ' '.join(value.replace(';', ' ').split())

    def _remaining_operation_sec(self) -> float:
        if self._operation_deadline_ns is None:
            return 0.0
        return max(
            0.0,
            (self._operation_deadline_ns - self.get_clock().now().nanoseconds)
            / 1.0e9,
        )

    def _finish_command(self) -> None:
        self._cancel_result_timer()
        self._cancel_contact_wait_timer()
        self._goal_handle = None
        self._active_command = None
        self._active_command_id = ''
        self._operation_start_ns = None
        self._operation_deadline_ns = None
        self._goal_accepted = False
        self._pending_action_state = None
        self._pending_error_code = None
        self._pending_error_string = ''

    def _cancel_result_timer(self) -> None:
        if self._result_timer is not None:
            self._result_timer.cancel()
            self.destroy_timer(self._result_timer)
            self._result_timer = None

    def _cancel_contact_wait_timer(self) -> None:
        if self._contact_wait_timer is not None:
            self._contact_wait_timer.cancel()
            self.destroy_timer(self._contact_wait_timer)
            self._contact_wait_timer = None
        self._contact_wait_deadline_ns = None

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = (
            f'{status};simulated_only=true;real_hardware=false'
        )
        self._status_publisher.publish(message)

    @staticmethod
    def _publish_bool(publisher, value: bool) -> None:
        message = Bool()
        message.data = value
        publisher.publish(message)


def main(args=None) -> None:
    """Run the simulator-only gripper bridge."""
    rclpy.init(args=args)
    node = GripperActionBridgeNode()
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
