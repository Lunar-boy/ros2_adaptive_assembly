"""Turn recovery supervisor decisions into bounded simulated retries."""

from typing import Dict, List

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def parse_status(status: str) -> Dict[str, str]:
    """Parse semicolon-delimited key/value fields."""
    fields = {}
    for item in status.split(';'):
        if '=' in item:
            key, value = item.split('=', 1)
            if key.strip():
                fields[key.strip()] = value.strip()
    return fields


class RecoveryOrchestratorNode(Node):
    """Run reset services and request one fresh simulated target pose."""

    _SEQUENCES = {
        'clear_dynamic_target_and_retry': ['clear_dynamic', 'publish_target'],
        'reset_scene_and_retry': [
            'clear_dynamic', 'clear_static', 'reapply_static', 'publish_target'
        ],
        'reset_planning_scene': [
            'clear_dynamic', 'clear_static', 'reapply_static', 'publish_target'
        ],
        'discard_trajectories_and_replan': ['publish_target'],
    }

    def __init__(self) -> None:
        super().__init__('recovery_orchestrator_node')
        defaults = {
            'recovery_action_topic': '/assembly_recovery_action',
            'orchestration_status_topic': '/recovery_orchestration_status',
            'retry_requested_topic': '/recovery_retry_requested',
            'clear_dynamic_scene_service': '/clear_dynamic_target_scene',
            'clear_static_scene_service': '/clear_static_planning_scene',
            'reapply_static_scene_service': '/reapply_static_planning_scene',
            'publish_target_pose_service': '/publish_target_pose_once',
            'max_retry_attempts': 1,
            'service_timeout_sec': 2.0,
            'enable_service_calls': True,
            'simulated_only': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self._max_attempts = int(
            self.get_parameter('max_retry_attempts').value
        )
        self._timeout = float(self.get_parameter('service_timeout_sec').value)
        self._enabled = bool(self.get_parameter('enable_service_calls').value)
        simulated_only = bool(self.get_parameter('simulated_only').value)
        if not simulated_only:
            raise ValueError('simulated_only must remain true')
        if self._max_attempts < 0:
            raise ValueError('max_retry_attempts must be non-negative')
        if self._timeout <= 0.0:
            raise ValueError('service_timeout_sec must be greater than zero')

        retained_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String,
            self.get_parameter('orchestration_status_topic').value,
            retained_qos,
        )
        self._retry_publisher = self.create_publisher(
            Bool,
            self.get_parameter('retry_requested_topic').value,
            retained_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter('recovery_action_topic').value,
            self._action_callback,
            retained_qos,
        )
        service_parameters = {
            'clear_dynamic': 'clear_dynamic_scene_service',
            'clear_static': 'clear_static_scene_service',
            'reapply_static': 'reapply_static_scene_service',
            'publish_target': 'publish_target_pose_service',
        }
        self._service_clients: Dict[str, Client] = {
            key: self.create_client(
                Trigger, self.get_parameter(parameter).value
            )
            for key, parameter in service_parameters.items()
        }
        self._attempt = 0
        self._busy = False
        self._action = ''
        self._pending: List[str] = []
        self._services_ok = True
        self._target_triggered = False
        self._generation = 0
        self._timeout_timer = None
        self._publish_status('event=startup;attempt=0;services_ok=true;'
                             'target_pose_triggered=false')
        self.get_logger().info(
            'Recovery orchestrator ready: simulated_only=true, '
            'real_hardware=false, trajectory_execution=false'
        )

    def _action_callback(self, message: String) -> None:
        fields = parse_status(message.data)
        action = fields.get('action', '')
        if (
            fields.get('event') != 'recovery_action'
            or action not in self._SEQUENCES
        ):
            return
        if self._busy:
            self.get_logger().info(
                'Ignoring recovery action while retry is active'
            )
            return
        if self._attempt >= self._max_attempts:
            self._publish_status(
                'event=retry_exhausted;'
                f'action={action};attempt={self._attempt};'
                'services_ok=false;target_pose_triggered=false'
            )
            return

        self._attempt += 1
        self._busy = True
        self._action = action
        self._pending = list(self._SEQUENCES[action])
        self._services_ok = True
        self._target_triggered = False
        retry = Bool()
        retry.data = True
        self._retry_publisher.publish(retry)
        if action == 'discard_trajectories_and_replan':
            self._publish_status(
                'event=discard_and_replan;'
                f'action={action};attempt={self._attempt};'
                'services_ok=true;target_pose_triggered=false'
            )
        if not self._enabled:
            self._services_ok = False
            self._finish()
            return
        self._generation += 1
        self._call_next(self._generation)

    def _call_next(self, generation: int) -> None:
        if generation != self._generation:
            return
        if not self._pending:
            self._finish()
            return
        key = self._pending.pop(0)
        client = self._service_clients[key]
        if not client.service_is_ready() and not client.wait_for_service(
            timeout_sec=self._timeout
        ):
            self._services_ok = False
            self.get_logger().error(f"Service unavailable: {client.srv_name}")
            self._call_next(generation)
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda completed, service_key=key, token=generation:
            self._service_done(completed, service_key, token)
        )
        self._timeout_timer = self.create_timer(
            self._timeout,
            lambda service_key=key, token=generation:
            self._service_timeout(service_key, token),
        )

    def _service_done(self, future: Future, key: str, generation: int) -> None:
        if generation != self._generation:
            return
        self._cancel_timeout()
        try:
            response = future.result()
            success = bool(response.success)
        except Exception as error:  # pragma: no cover - middleware guard
            self.get_logger().error(f"Service '{key}' failed: {error}")
            success = False
        self._services_ok = self._services_ok and success
        if key == 'publish_target':
            self._target_triggered = success
        self._call_next(generation)

    def _service_timeout(self, key: str, generation: int) -> None:
        if generation != self._generation:
            return
        self._cancel_timeout()
        self._services_ok = False
        self.get_logger().error(
            f"Service timed out: {self._service_clients[key].srv_name}"
        )
        self._generation += 1
        self._call_next(self._generation)

    def _cancel_timeout(self) -> None:
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self.destroy_timer(self._timeout_timer)
            self._timeout_timer = None

    def _finish(self) -> None:
        self._publish_status(
            'event=retry_requested;'
            f'action={self._action};attempt={self._attempt};'
            f'services_ok={str(self._services_ok).lower()};'
            f'target_pose_triggered={str(self._target_triggered).lower()}'
        )
        self._busy = False

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status + ';simulated_only=true;real_hardware=false'
        self._status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryOrchestratorNode()
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
