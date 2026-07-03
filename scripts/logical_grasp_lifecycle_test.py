"""Shared isolated validation for the logical grasp lifecycle."""

import signal
import subprocess
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String


def fields(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited status message."""
    return dict(
        item.split('=', 1) for item in status.split(';') if '=' in item
    )


class Checker(Node):
    """Publish synthetic execution events and collect lifecycle state."""

    def __init__(self, prefix: str) -> None:
        """Create isolated synthetic inputs and retained subscriptions."""
        super().__init__('logical_grasp_lifecycle_checker')
        self.command: Optional[str] = None
        self.command_status: Optional[str] = None
        self.object_state: Optional[str] = None
        self.attached: Optional[bool] = None
        self.lifecycle: Optional[str] = None
        self.stage = self.create_publisher(String, f'{prefix}/stage', 10)
        retained = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.execution = self.create_publisher(
            String, f'{prefix}/execution', retained
        )
        self.create_subscription(
            String, f'{prefix}/command', self._set_command, retained
        )
        self.create_subscription(
            String,
            f'{prefix}/command_status',
            self._set_command_status,
            retained,
        )
        self.create_subscription(
            String, f'{prefix}/object_state', self._set_object_state, retained
        )
        self.create_subscription(
            Bool, f'{prefix}/attached', self._set_attached, retained
        )
        self.create_subscription(
            String, f'{prefix}/lifecycle', self._set_lifecycle, retained
        )

    def _set_command(self, message: String) -> None:
        self.command = message.data

    def _set_command_status(self, message: String) -> None:
        self.command_status = message.data

    def _set_object_state(self, message: String) -> None:
        self.object_state = message.data

    def _set_attached(self, message: Bool) -> None:
        self.attached = message.data

    def _set_lifecycle(self, message: String) -> None:
        self.lifecycle = message.data


def _matches(value: Optional[str], expected: Dict[str, str]) -> bool:
    if value is None:
        return False
    parsed = fields(value)
    return all(parsed.get(key) == item for key, item in expected.items())


def _spin_until(node: Checker, predicate, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def _publish(publisher, value: str) -> None:
    message = String()
    message.data = value
    publisher.publish(message)


def run_check(mode: str) -> int:
    """Validate startup and either success release or failure retention."""
    prefix = f'/logical_grasp_{mode}_validation'
    attach_stage = 'grasp' if mode == 'place_release' else 'pre_grasp'
    extra_parameters = (
        ['-p', 'release_stage:=place'] if mode == 'place_release' else []
    )
    process = subprocess.Popen([
        'ros2', 'run', 'adaptive_assembly_manipulation',
        'logical_grasp_lifecycle_node', '--ros-args',
        '-r', f'__node:=logical_grasp_{mode}_validation',
        '-p', f'stage_status_topic:={prefix}/stage',
        '-p', f'execution_status_topic:={prefix}/execution',
        '-p', f'gripper_command_topic:={prefix}/command',
        '-p', f'gripper_command_status_topic:={prefix}/command_status',
        '-p', f'object_grasp_state_topic:={prefix}/object_state',
        '-p', f'object_grasp_attached_topic:={prefix}/attached',
        '-p', f'lifecycle_status_topic:={prefix}/lifecycle',
        '-p', f'attach_stage:={attach_stage}',
        *extra_parameters,
    ])
    rclpy.init()
    node = Checker(prefix)
    try:
        initial = _spin_until(node, lambda: (
            _matches(node.command, {'event': 'command', 'command': 'open'})
            and _matches(node.object_state, {
                'event': 'detached', 'parent': 'world',
                'gazebo_attach': 'false', 'real_hardware': 'false',
            })
            and node.attached is False
            and _matches(node.lifecycle, {'event': 'ready'})
        ))
        if not initial:
            print('FAIL: missing retained initial open/detached state')
            return 1

        _publish(
            node.stage,
            f'event=success;mode=ros2_control;stage={attach_stage};'
            'real_hardware=false',
        )
        attached = _spin_until(node, lambda: (
            _matches(node.command, {'command': 'close'})
            and _matches(node.command_status, {
                'event': 'success', 'command': 'close',
            })
            and _matches(node.object_state, {
                'event': 'attached', 'parent': 'panda_hand',
            })
            and node.attached is True
            and _matches(node.lifecycle, {'event': 'attached'})
        ))
        if not attached:
            print('FAIL: pre-grasp success did not close and attach')
            return 1

        if mode == 'place_release':
            _publish(
                node.stage,
                'event=success;mode=ros2_control;stage=place;'
                'real_hardware=false',
            )
            terminal = _spin_until(node, lambda: (
                _matches(node.command, {'command': 'open'})
                and _matches(node.object_state, {
                    'event': 'detached', 'trigger': 'place_success',
                })
                and node.attached is False
                and _matches(node.lifecycle, {
                    'event': 'released', 'release_stage': 'place',
                })
            ))
            description = 'place-stage release before aggregate success'
        elif mode == 'success':
            _publish(
                node.execution,
                'event=success;mode=ros2_control;execution=true;'
                'real_hardware=false',
            )
            terminal = _spin_until(node, lambda: (
                _matches(node.command, {'command': 'open'})
                and _matches(node.object_state, {
                    'event': 'detached', 'parent': 'world',
                })
                and node.attached is False
                and _matches(node.lifecycle, {'event': 'released'})
            ))
            description = 'open/detached release'
        else:
            _publish(
                node.execution,
                'event=failure;mode=ros2_control;stage=assembly;'
                'reason=result_timeout;execution=false;real_hardware=false',
            )
            terminal = _spin_until(node, lambda: (
                node.attached is True
                and _matches(node.object_state, {'event': 'attached'})
                and _matches(node.lifecycle, {
                    'event': 'failure', 'reason': 'result_timeout',
                    'attached': 'true',
                })
            ))
            description = 'failure with retained grasp state'
        if not terminal:
            print(f'FAIL: missing deterministic {description}')
            return 1
        if process.poll() is not None:
            print('FAIL: lifecycle node exited unexpectedly')
            return 1
        print(
            f'PASS: logical grasp {mode} path has deterministic '
            f'{description}'
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3.0)
