"""Classify pipeline failures and publish deterministic recovery actions."""

from enum import Enum
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class RecoveryState(str, Enum):
    """States exposed by the recovery supervisor."""

    IDLE = 'IDLE'
    WAIT_FOR_PLAN = 'WAIT_FOR_PLAN'
    PLAN_SUCCEEDED = 'PLAN_SUCCEEDED'
    EXECUTION_SUCCEEDED = 'EXECUTION_SUCCEEDED'
    RECOVER_PLANNING_FAILURE = 'RECOVER_PLANNING_FAILURE'
    RECOVER_EXECUTION_FAILURE = 'RECOVER_EXECUTION_FAILURE'
    RECOVER_SCENE_FAILURE = 'RECOVER_SCENE_FAILURE'
    RECOVERY_ACTION_PUBLISHED = 'RECOVERY_ACTION_PUBLISHED'
    RECOVERY_EXHAUSTED = 'RECOVERY_EXHAUSTED'


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited key/value status string."""
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' not in item:
            continue
        key, value = item.split('=', 1)
        key = key.strip()
        if key:
            fields[key] = value.strip()
    return fields


class RecoverySupervisorNode(Node):
    """Observe pipeline status and expose recovery decisions only."""

    _SERVICE_TIMEOUT_SEC = 2.0
    _HEARTBEAT_PERIOD_SEC = 1.0

    def __init__(self) -> None:
        """Declare parameters and create recovery ROS interfaces."""
        super().__init__('recovery_supervisor_node')

        defaults = {
            'planning_status_topic': '/assembly_sequence_planning_status',
            'execution_status_topic': '/assembly_execution_status',
            'dynamic_scene_status_topic': '/dynamic_target_scene_status',
            'planning_scene_audit_status_topic': (
                '/planning_scene_audit_status'
            ),
            'recovery_status_topic': '/assembly_recovery_status',
            'recovery_action_topic': '/assembly_recovery_action',
            'recovery_success_topic': '/assembly_recovery_success',
            'clear_dynamic_scene_service': '/clear_dynamic_target_scene',
            'clear_static_scene_service': '/clear_static_planning_scene',
            'reapply_static_scene_service': '/reapply_static_planning_scene',
            'enable_service_calls': False,
            'max_recovery_attempts': 1,
            'publish_heartbeat': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self._enable_service_calls = bool(
            self.get_parameter('enable_service_calls').value
        )
        self._max_recovery_attempts = int(
            self.get_parameter('max_recovery_attempts').value
        )
        self._publish_heartbeat_enabled = bool(
            self.get_parameter('publish_heartbeat').value
        )
        if self._max_recovery_attempts < 0:
            raise ValueError('max_recovery_attempts must be non-negative')

        result_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String,
            self.get_parameter('recovery_status_topic').value,
            result_qos,
        )
        self._action_publisher = self.create_publisher(
            String,
            self.get_parameter('recovery_action_topic').value,
            result_qos,
        )
        self._success_publisher = self.create_publisher(
            Bool,
            self.get_parameter('recovery_success_topic').value,
            result_qos,
        )

        self.create_subscription(
            String,
            self.get_parameter('planning_status_topic').value,
            self._planning_callback,
            result_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter('execution_status_topic').value,
            self._execution_callback,
            result_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter('dynamic_scene_status_topic').value,
            self._dynamic_scene_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('planning_scene_audit_status_topic').value,
            self._audit_callback,
            10,
        )

        self._service_clients: Dict[str, Client] = {
            'clear_dynamic': self.create_client(
                Trigger,
                self.get_parameter('clear_dynamic_scene_service').value,
            ),
            'clear_static': self.create_client(
                Trigger,
                self.get_parameter('clear_static_scene_service').value,
            ),
            'reapply_static': self.create_client(
                Trigger,
                self.get_parameter('reapply_static_scene_service').value,
            ),
        }
        self._service_names = {
            key: client.srv_name
            for key, client in self._service_clients.items()
        }

        self._state = RecoveryState.IDLE
        self._attempt = 0
        self._active_service_sequence: List[str] = []
        self._service_generation = 0
        self._service_timeout_timer = None
        self._last_failure: Optional[Tuple[str, str, str]] = None

        self._publish_state('startup')
        self._state = RecoveryState.WAIT_FOR_PLAN
        self._publish_state('startup')
        if self._publish_heartbeat_enabled:
            self.create_timer(
                self._HEARTBEAT_PERIOD_SEC,
                self._publish_heartbeat,
            )

        self.get_logger().info(
            'Recovery supervisor ready: '
            f'enable_service_calls={self._enable_service_calls}, '
            f'max_recovery_attempts={self._max_recovery_attempts}, '
            'real_execution=false. No planning, trajectory execution, or '
            'robot command interfaces are used.'
        )

    def _planning_callback(self, message: String) -> None:
        """Classify aggregate two-stage planning status."""
        fields = parse_status(message.data)
        event = fields.get('event')
        if event == 'success':
            if self._state == RecoveryState.EXECUTION_SUCCEEDED:
                return
            self._last_failure = None
            self._state = RecoveryState.PLAN_SUCCEEDED
            self._publish_state('planning', {'failed_stage': 'none'})
            return
        if event != 'failure':
            return
        failed_stage = fields.get('failed_stage')
        if failed_stage not in {'pre_grasp', 'assembly'}:
            return
        if failed_stage == 'pre_grasp':
            action = 'reset_scene_and_retry'
            reason = 'pre_grasp_planning_failed'
        else:
            action = 'clear_dynamic_target_and_retry'
            reason = 'assembly_planning_failed'
        self._recover(
            RecoveryState.RECOVER_PLANNING_FAILURE,
            'planning',
            action,
            reason,
            {'failed_stage': failed_stage},
        )

    def _execution_callback(self, message: String) -> None:
        """Classify aggregate dry-run execution status."""
        fields = parse_status(message.data)
        event = fields.get('event')
        mode = fields.get('mode', 'unknown')
        if event == 'success' and mode == 'dry_run':
            self._last_failure = None
            self._state = RecoveryState.EXECUTION_SUCCEEDED
            self._publish_status(
                'event=success;state=EXECUTION_SUCCEEDED;'
                'source=execution;mode=dry_run;real_execution=false'
            )
            self._publish_success(True)
            return
        if event == 'failure':
            self._recover(
                RecoveryState.RECOVER_EXECUTION_FAILURE,
                'execution',
                'discard_trajectories_and_replan',
                'dry_run_execution_failed',
                {'mode': mode},
            )

    def _dynamic_scene_callback(self, message: String) -> None:
        """Classify dynamic PlanningScene update failures."""
        fields = parse_status(message.data)
        if fields.get('event') not in {'failed', 'clear_failed'}:
            return
        self._recover_scene('dynamic_scene', fields.get('event', 'unknown'))

    def _audit_callback(self, message: String) -> None:
        """Classify parseable audits that report expected missing objects."""
        fields = parse_status(message.data)
        if fields.get('event') != 'audit':
            return
        missing = fields.get('missing')
        all_present = fields.get('all_present')
        if not missing or all_present not in {'true', 'false'}:
            return
        if all_present == 'false' and missing != 'none':
            self._recover_scene('planning_scene_audit', missing)

    def _recover_scene(self, source: str, detail: str) -> None:
        self._recover(
            RecoveryState.RECOVER_SCENE_FAILURE,
            source,
            'reset_planning_scene',
            'scene_inconsistent',
            {'detail': detail},
        )

    def _recover(
        self,
        recovery_state: RecoveryState,
        source: str,
        action: str,
        reason: str,
        details: Dict[str, str],
    ) -> None:
        """Publish one recovery decision or an exhausted result."""
        failure_key = (source, action, reason + repr(sorted(details.items())))
        if failure_key == self._last_failure:
            return
        self._last_failure = failure_key

        if self._attempt >= self._max_recovery_attempts:
            self._state = RecoveryState.RECOVERY_EXHAUSTED
            self._publish_status(
                'event=exhausted;state=RECOVERY_EXHAUSTED;'
                'reason=max_recovery_attempts_reached;real_execution=false'
            )
            self._publish_success(False)
            return

        self._attempt += 1
        self._state = recovery_state
        state_fields = {
            'source': source,
            **details,
            'attempt': str(self._attempt),
            'max_attempts': str(self._max_recovery_attempts),
        }
        self._publish_state_fields(state_fields)

        self._state = RecoveryState.RECOVERY_ACTION_PUBLISHED
        action_status = (
            'event=recovery_action;state=RECOVERY_ACTION_PUBLISHED;'
            f'action={action};reason={reason};attempt={self._attempt};'
            f'service_calls={str(self._enable_service_calls).lower()};'
            'real_execution=false'
        )
        self._publish_action(action_status)
        self._publish_status(action_status)
        self._publish_success(False)

        if self._enable_service_calls:
            self._start_service_sequence(action)

    def _start_service_sequence(self, action: str) -> None:
        """Start the non-blocking service sequence for one action."""
        sequences = {
            'reset_scene_and_retry': [
                'clear_dynamic', 'clear_static', 'reapply_static'
            ],
            'clear_dynamic_target_and_retry': ['clear_dynamic'],
            'reset_planning_scene': [
                'clear_dynamic', 'clear_static', 'reapply_static'
            ],
        }
        self._service_generation += 1
        self._active_service_sequence = list(sequences.get(action, []))
        self._call_next_service(self._service_generation)

    def _call_next_service(self, generation: int) -> None:
        if generation != self._service_generation:
            return
        if not self._active_service_sequence:
            self._publish_service_status('sequence', 'complete')
            return

        service_key = self._active_service_sequence.pop(0)
        client = self._service_clients[service_key]
        if not client.service_is_ready():
            self._publish_service_status(service_key, 'unavailable')
            self._call_next_service(generation)
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda completed, key=service_key, token=generation:
            self._service_done(completed, key, token)
        )
        self._service_timeout_timer = self.create_timer(
            self._SERVICE_TIMEOUT_SEC,
            lambda key=service_key, token=generation:
            self._service_timed_out(key, token),
        )

    def _service_done(
        self, future: Future, service_key: str, generation: int
    ) -> None:
        if generation != self._service_generation:
            return
        self._cancel_service_timeout()
        try:
            response = future.result()
            result = 'success' if response.success else 'failure'
        except Exception as error:  # pragma: no cover - middleware guard
            result = 'error'
            self.get_logger().error(
                f"Recovery service '{self._service_names[service_key]}' "
                f'failed: {error}'
            )
        self._publish_service_status(service_key, result)
        self._call_next_service(generation)

    def _service_timed_out(self, service_key: str, generation: int) -> None:
        if generation != self._service_generation:
            return
        self._cancel_service_timeout()
        self._service_generation += 1
        next_generation = self._service_generation
        self._publish_service_status(service_key, 'timeout')
        self._call_next_service(next_generation)

    def _cancel_service_timeout(self) -> None:
        if self._service_timeout_timer is not None:
            self._service_timeout_timer.cancel()
            self.destroy_timer(self._service_timeout_timer)
            self._service_timeout_timer = None

    def _publish_service_status(self, service_key: str, result: str) -> None:
        service = self._service_names.get(service_key, service_key)
        self._publish_status(
            'event=service_call;state=RECOVERY_ACTION_PUBLISHED;'
            f'service={service};result={result};attempt={self._attempt};'
            'real_execution=false'
        )

    def _publish_heartbeat(self) -> None:
        if self._state in {
            RecoveryState.EXECUTION_SUCCEEDED,
            RecoveryState.RECOVERY_ACTION_PUBLISHED,
            RecoveryState.RECOVERY_EXHAUSTED,
        }:
            return
        self._publish_status(
            f'event=heartbeat;state={self._state.value};'
            f'attempt={self._attempt};real_execution=false'
        )

    def _publish_state(
        self, source: str, details: Optional[Dict[str, str]] = None
    ) -> None:
        fields = {'source': source}
        if details:
            fields.update(details)
        self._publish_state_fields(fields)

    def _publish_state_fields(self, fields: Dict[str, str]) -> None:
        suffix = ''.join(f';{key}={value}' for key, value in fields.items())
        self._publish_status(
            f'event=state_update;state={self._state.value}{suffix};'
            'real_execution=false'
        )

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status_publisher.publish(message)

    def _publish_action(self, action: str) -> None:
        message = String()
        message.data = action
        self._action_publisher.publish(message)

    def _publish_success(self, success: bool) -> None:
        message = Bool()
        message.data = success
        self._success_publisher.publish(message)


def main(args=None) -> None:
    """Run the recovery supervisor node."""
    rclpy.init(args=args)
    node = RecoverySupervisorNode()
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
