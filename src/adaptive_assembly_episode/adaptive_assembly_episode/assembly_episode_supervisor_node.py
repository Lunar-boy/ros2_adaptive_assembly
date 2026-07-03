"""Passively aggregate an assembly episode into one terminal result."""

import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String


def parse_status(status: str) -> Dict[str, str]:
    """Parse semicolon-delimited key/value fields and ignore malformed data."""
    fields: Dict[str, str] = {}
    for fragment in status.split(';'):
        if '=' not in fragment:
            continue
        key, value = fragment.split('=', 1)
        key = key.strip()
        if key:
            fields[key] = value.strip()
    return fields


class AssemblyEpisodeSupervisorNode(Node):
    """Observe existing status topics without commanding the runtime."""

    _TERMINAL_FAILURE_EVENTS = {
        'failure', 'timeout', 'skipped', 'rejected', 'aborted', 'canceled'
    }

    def __init__(self) -> None:
        super().__init__('assembly_episode_supervisor_node')

        defaults = {
            'status_topic': '/assembly_episode_status',
            'success_topic': '/assembly_episode_success',
            'duration_topic': '/assembly_episode_duration_ms',
            'stage_status_topic': '/assembly_episode_stage_status',
            'failure_reason_topic': '/assembly_episode_failure_reason',
            'episode_timeout_sec': 120.0,
            'require_planning_success': True,
            'require_execution_success': True,
            'require_logical_grasp_released': True,
            'require_gazebo_attach_success': True,
            'require_insertion_success': True,
            'simulated_only': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self._timeout_sec = float(
            self.get_parameter('episode_timeout_sec').value
        )
        if self._timeout_sec <= 0.0:
            raise ValueError('episode_timeout_sec must be positive')
        if not bool(self.get_parameter('simulated_only').value):
            raise ValueError('simulated_only must remain true')

        self._requirements = {
            'planning': bool(
                self.get_parameter('require_planning_success').value
            ),
            'execution': bool(
                self.get_parameter('require_execution_success').value
            ),
            'logical_release': bool(
                self.get_parameter('require_logical_grasp_released').value
            ),
            'gazebo_attach': bool(
                self.get_parameter('require_gazebo_attach_success').value
            ),
            'insertion': bool(
                self.get_parameter('require_insertion_success').value
            ),
        }

        retained_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, self.get_parameter('status_topic').value, retained_qos
        )
        self._success_publisher = self.create_publisher(
            Bool, self.get_parameter('success_topic').value, retained_qos
        )
        self._duration_publisher = self.create_publisher(
            Float64, self.get_parameter('duration_topic').value, retained_qos
        )
        self._failure_reason_publisher = self.create_publisher(
            String,
            self.get_parameter('failure_reason_topic').value,
            retained_qos,
        )
        self._stage_publisher = self.create_publisher(
            String, self.get_parameter('stage_status_topic').value, 10
        )

        self._start_time = time.monotonic()
        self._terminal = False
        self._stage = 'init'
        self._planning_success = False
        self._pre_grasp_success = False
        self._assembly_success = False
        self._execution_success_signal = False
        self._execution_terminal_success = False
        self._execution_terminal_received = False
        self._logical_grasp_attached = False
        self._logical_grasp_released = False
        self._gazebo_attach_success = False
        self._gazebo_attach_failed = False
        self._insertion_status_received = False
        self._insertion_success = False
        self._insertion_error_mm: Optional[float] = None
        self._insertion_error_deg: Optional[float] = None
        self._gazebo_attach_pose_error_mm: Optional[float] = None

        # Volatile subscriptions accept both retained and non-retained upstream
        # publishers. Several stage-status topics are intentionally volatile.
        upstream_qos = 10
        subscriptions = (
            (String, '/assembly_sequence_planning_status', self._planning_cb),
            (String, '/assembly_sequence_stage_status', self._planning_stage_cb),
            (String, '/assembly_ros2_control_execution_status', self._execution_status_cb),
            (Bool, '/assembly_ros2_control_execution_success', self._execution_success_cb),
            (Float64, '/assembly_ros2_control_execution_duration_ms', self._execution_duration_cb),
            (String, '/assembly_ros2_control_execution_stage_status', self._execution_stage_cb),
            (String, '/logical_grasp_lifecycle_status', self._logical_grasp_cb),
            (Bool, '/object_grasp_attached', self._logical_attached_cb),
            (String, '/gazebo_attach_detach_status', self._gazebo_attach_cb),
            (Bool, '/gazebo_object_attached', self._gazebo_attached_cb),
            (Float64, '/gazebo_attach_pose_error_mm', self._gazebo_error_cb),
            (String, '/assembly_insertion_status', self._insertion_status_cb),
            (Bool, '/assembly_insertion_success', self._insertion_success_cb),
            (Float64, '/assembly_insertion_error_mm', self._insertion_mm_cb),
            (Float64, '/assembly_insertion_error_deg', self._insertion_deg_cb),
        )
        self._upstream_subscriptions = [
            self.create_subscription(msg_type, topic, callback, upstream_qos)
            for msg_type, topic, callback in subscriptions
        ]
        self._timer = self.create_timer(0.1, self._timeout_cb)
        self._publish_stage('ready', 'init')
        self.get_logger().info(
            'Passive assembly episode supervisor ready: '
            f'episode_timeout_sec={self._timeout_sec:.3f}, '
            f'requirements={self._requirements}, simulated_only=true, '
            'real_hardware=false. No command or service interfaces are used.'
        )

    def _planning_cb(self, message: String) -> None:
        fields = parse_status(message.data)
        event = fields.get('event', '').lower()
        successful = event == 'success' or (
            event in {'complete', 'completed', 'terminal'}
            and fields.get('success', '').lower() == 'true'
        )
        if successful:
            self._planning_success = True
            self._evaluate()
        elif event in self._TERMINAL_FAILURE_EVENTS:
            self._fail_if_required('planning', 'planning_failed')

    def _planning_stage_cb(self, message: String) -> None:
        del message

    def _execution_stage_cb(self, message: String) -> None:
        fields = parse_status(message.data)
        if fields.get('event') != 'success':
            return
        if fields.get('stage') == 'pre_grasp':
            self._pre_grasp_success = True
        elif fields.get('stage') == 'assembly':
            self._assembly_success = True
        self._evaluate()

    def _execution_status_cb(self, message: String) -> None:
        event = parse_status(message.data).get('event', '').lower()
        if event == 'success':
            self._execution_terminal_received = True
            self._execution_terminal_success = True
            self._evaluate()
        elif event in self._TERMINAL_FAILURE_EVENTS:
            self._execution_terminal_received = True
            self._fail_if_required('execution', 'execution_failed')

    def _execution_success_cb(self, message: Bool) -> None:
        self._execution_success_signal = bool(message.data)
        self._evaluate()

    def _execution_duration_cb(self, message: Float64) -> None:
        del message

    def _logical_grasp_cb(self, message: String) -> None:
        event = parse_status(message.data).get('event', '').lower()
        if event == 'attached':
            self._logical_grasp_attached = True
        elif event == 'released':
            self._logical_grasp_released = True
        elif event in self._TERMINAL_FAILURE_EVENTS:
            self._fail_if_required('logical_release', 'grasp_attach_failed')
            return
        self._evaluate()

    def _logical_attached_cb(self, message: Bool) -> None:
        if message.data:
            self._logical_grasp_attached = True
        elif self._logical_grasp_attached:
            self._logical_grasp_released = True
        self._evaluate()

    def _gazebo_attach_cb(self, message: String) -> None:
        event = parse_status(message.data).get('event', '').lower()
        if event == 'attached':
            self._gazebo_attach_success = True
            self._gazebo_attach_failed = False
            self._evaluate()
        elif event in self._TERMINAL_FAILURE_EVENTS:
            self._gazebo_attach_failed = True
            self._gazebo_attach_success = False
            self._fail_if_required('gazebo_attach', 'grasp_attach_failed')

    def _gazebo_attached_cb(self, message: Bool) -> None:
        del message  # Status evidence, not attachment-success evidence.

    def _gazebo_error_cb(self, message: Float64) -> None:
        self._gazebo_attach_pose_error_mm = float(message.data)

    def _insertion_status_cb(self, message: String) -> None:
        self._insertion_status_received = True
        fields = parse_status(message.data)
        event = fields.get('event', '').lower()
        if event in self._TERMINAL_FAILURE_EVENTS:
            self._fail_if_required('insertion', 'insertion_failed')

    def _insertion_success_cb(self, message: Bool) -> None:
        if message.data:
            self._insertion_success = True
            self._evaluate()
        elif self._insertion_status_received:
            self._fail_if_required('insertion', 'insertion_failed')

    def _insertion_mm_cb(self, message: Float64) -> None:
        self._insertion_error_mm = float(message.data)

    def _insertion_deg_cb(self, message: Float64) -> None:
        self._insertion_error_deg = float(message.data)

    @property
    def _execution_success(self) -> bool:
        return self._execution_success_signal and self._execution_terminal_success

    def _fail_if_required(self, requirement: str, reason: str) -> None:
        if self._requirements[requirement]:
            self._publish_terminal('failure', reason)

    def _evaluate(self) -> None:
        if self._terminal:
            return
        required_results = (
            (not self._requirements['planning'] or self._planning_success),
            (
                not self._requirements['execution']
                or (
                    self._pre_grasp_success
                    and self._assembly_success
                    and self._execution_success
                )
            ),
            (
                not self._requirements['logical_release']
                or self._logical_grasp_released
            ),
            (
                not self._requirements['gazebo_attach']
                or self._gazebo_attach_success
            ),
            (
                not self._requirements['insertion']
                or self._insertion_success
            ),
        )
        if all(required_results):
            self._publish_terminal('success', '')
            return
        self._update_stage()

    def _update_stage(self) -> None:
        if self._requirements['planning'] and not self._planning_success:
            stage = 'wait_planning'
        elif not self._pre_grasp_success and self._requirements['execution']:
            stage = 'wait_pre_grasp_success'
        elif (
            self._requirements['logical_release']
            and not self._logical_grasp_attached
        ):
            stage = 'wait_grasp_attached'
        elif not self._assembly_success and self._requirements['execution']:
            stage = 'wait_assembly_execution'
        elif (
            self._requirements['execution']
            and not self._execution_terminal_received
        ):
            stage = 'wait_execution_terminal'
        elif (
            self._requirements['logical_release']
            and not self._logical_grasp_released
        ):
            stage = 'wait_release'
        else:
            stage = 'wait_insertion_evaluation'
        if stage != self._stage:
            self._stage = stage
            self._publish_stage('stage_transition', stage)

    def _timeout_cb(self) -> None:
        if not self._terminal and self._elapsed_ms() >= self._timeout_sec * 1000.0:
            self._publish_terminal('timeout', 'episode_timeout')

    def _publish_stage(self, event: str, stage: str) -> None:
        message = String()
        message.data = (
            f'event={event};mode=assembly_episode;stage={stage};'
            'simulated_only=true;real_hardware=false'
        )
        self._stage_publisher.publish(message)

    def _elapsed_ms(self) -> float:
        return (time.monotonic() - self._start_time) * 1000.0

    @staticmethod
    def _bool(value: bool) -> str:
        return str(value).lower()

    def _publish_terminal(self, event: str, failure_reason: str) -> None:
        if self._terminal:
            return
        self._terminal = True
        self._stage = 'terminal'
        duration_ms = self._elapsed_ms()
        episode_success = event == 'success'
        fields = [
            f'event={event}',
            'mode=assembly_episode',
            'stage=terminal',
            f'episode_success={self._bool(episode_success)}',
            f'planning_success={self._bool(self._planning_success)}',
            f'pre_grasp_success={self._bool(self._pre_grasp_success)}',
            f'assembly_success={self._bool(self._assembly_success)}',
            f'execution_success={self._bool(self._execution_success)}',
            'logical_grasp_attached='
            f'{self._bool(self._logical_grasp_attached)}',
            'logical_grasp_released='
            f'{self._bool(self._logical_grasp_released)}',
            f'gazebo_attach_success={self._bool(self._gazebo_attach_success)}',
            f'insertion_success={self._bool(self._insertion_success)}',
            f'duration_ms={duration_ms:.6f}',
        ]
        if failure_reason:
            fields.append(f'failure_reason={failure_reason}')
        if self._insertion_error_mm is not None:
            fields.append(f'insertion_error_mm={self._insertion_error_mm:.6f}')
        if self._insertion_error_deg is not None:
            fields.append(f'insertion_error_deg={self._insertion_error_deg:.6f}')
        if self._gazebo_attach_pose_error_mm is not None:
            fields.append(
                'gazebo_attach_pose_error_mm='
                f'{self._gazebo_attach_pose_error_mm:.6f}'
            )
        fields.extend(('simulated_only=true', 'real_hardware=false'))

        status = String()
        status.data = ';'.join(fields)
        success = Bool()
        success.data = episode_success
        duration = Float64()
        duration.data = duration_ms
        reason = String()
        reason.data = failure_reason
        self._status_publisher.publish(status)
        self._success_publisher.publish(success)
        self._duration_publisher.publish(duration)
        self._failure_reason_publisher.publish(reason)
        self._publish_stage(event, 'terminal')
        self.get_logger().info(status.data)


def main(args=None) -> None:
    """Run the passive episode supervisor."""
    rclpy.init(args=args)
    node = AssemblyEpisodeSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
