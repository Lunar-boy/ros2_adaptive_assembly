"""Bridge logical gripper commands to a simulator-only trajectory action."""

from typing import Dict, Optional

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectoryPoint


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


class GripperActionBridgeNode(Node):
    """Send two-finger trajectories for logical open and close commands."""

    def __init__(self) -> None:
        super().__init__('gripper_action_bridge_node')
        defaults = {
            'command_topic': '/gripper_command',
            'controller_action_name': (
                '/panda_gripper_controller/follow_joint_trajectory'
            ),
            'status_topic': '/physical_gripper_command_status',
            'success_topic': '/physical_gripper_command_success',
            'closed_topic': '/physical_gripper_closed',
            'joint_names': [
                'panda_finger_joint1', 'panda_finger_joint2'
            ],
            'open_position': 0.04,
            'close_position': 0.0,
            'goal_time_sec': 1.0,
            'wait_for_controller_sec': 5.0,
            'result_timeout_sec': 5.0,
            'send_goals': True,
            'simulated_only': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        if not bool(self.get_parameter('simulated_only').value):
            raise ValueError(
                'simulated_only must remain true; real hardware is not supported'
            )
        self._joint_names = list(self.get_parameter('joint_names').value)
        if len(self._joint_names) != 2 or any(
                not str(name) for name in self._joint_names):
            raise ValueError('joint_names must contain exactly two non-empty names')
        self._action_name = str(
            self.get_parameter('controller_action_name').value
        )
        self._send_goals = bool(self.get_parameter('send_goals').value)
        self._active_command: Optional[str] = None
        self._goal_handle = None
        self._result_timer = None

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
        self._action_client = ActionClient(
            self, FollowJointTrajectory, self._action_name
        )
        self._publish_status(
            'event=ready;mode=gripper_action_bridge;'
            f'controller={self._action_name}'
        )
        self.get_logger().info(
            'Simulator-only gripper action bridge ready: '
            f"controller='{self._action_name}', send_goals={self._send_goals}"
        )

    def _command_callback(self, message: String) -> None:
        fields = parse_status(message.data)
        if fields.get('event') != 'command':
            return
        command = fields.get('command', '')
        if command not in ('open', 'close'):
            self._publish_failure('unknown_command', command=command or 'missing')
            return
        if self._active_command is not None:
            self._publish_failure('busy', command=command)
            return

        self._active_command = command
        if not self._send_goals:
            self._publish_status(
                f'event=success;command={command};send_goals=false'
            )
            self._publish_bool(self._success_publisher, True)
            self._publish_bool(self._closed_publisher, command == 'close')
            self._active_command = None
            return

        wait_sec = float(
            self.get_parameter('wait_for_controller_sec').value
        )
        self._publish_status(f'event=waiting;command={command}')
        if not self._action_client.wait_for_server(timeout_sec=wait_sec):
            self._publish_failure('controller_unavailable', command=command)
            self._active_command = None
            return

        position_parameter = (
            'open_position' if command == 'open' else 'close_position'
        )
        position = float(self.get_parameter(position_parameter).value)
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        goal_time = float(self.get_parameter('goal_time_sec').value)
        point.time_from_start.sec = int(goal_time)
        point.time_from_start.nanosec = int(
            (goal_time - int(goal_time)) * 1_000_000_000
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [point]
        self._publish_status(f'event=sending;command={command}')
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        command = self._active_command or 'unknown'
        try:
            goal_handle = future.result()
        except Exception as error:  # action transport errors are runtime failures
            self.get_logger().error(f'Gripper goal request failed: {error}')
            self._publish_failure('goal_request_failed', command=command)
            self._active_command = None
            return
        if not goal_handle.accepted:
            self._publish_failure('goal_rejected', command=command)
            self._active_command = None
            return

        self._goal_handle = goal_handle
        self._publish_status(f'event=accepted;command={command}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        timeout_sec = float(self.get_parameter('result_timeout_sec').value)
        self._result_timer = self.create_timer(
            timeout_sec, self._result_timeout_callback
        )

    def _result_callback(self, future) -> None:
        if self._active_command is None:
            return
        self._cancel_result_timer()
        command = self._active_command
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:  # action transport errors are runtime failures
            self.get_logger().error(f'Gripper result request failed: {error}')
            self._publish_failure('result_error', command=command)
            self._finish_command()
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_failure(
                'action_failed', command=command, action_status=status
            )
            self._finish_command()
            return

        self._publish_status(
            f'event=success;command={command};send_goals=true;'
            f'action_status={status}'
        )
        self._publish_bool(self._success_publisher, True)
        self._publish_bool(self._closed_publisher, command == 'close')
        self._finish_command()

    def _result_timeout_callback(self) -> None:
        if self._active_command is None:
            return
        command = self._active_command
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._publish_failure('result_timeout', command=command)
        self._finish_command()

    def _finish_command(self) -> None:
        self._cancel_result_timer()
        self._goal_handle = None
        self._active_command = None

    def _cancel_result_timer(self) -> None:
        if self._result_timer is not None:
            self._result_timer.cancel()
            self.destroy_timer(self._result_timer)
            self._result_timer = None

    def _publish_failure(self, reason: str, **fields) -> None:
        details = ';'.join(f'{key}={value}' for key, value in fields.items())
        suffix = f';{details}' if details else ''
        self._publish_status(f'event=failure;reason={reason}{suffix}')
        self._publish_bool(self._success_publisher, False)

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
