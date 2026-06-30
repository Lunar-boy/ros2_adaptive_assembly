#!/usr/bin/env python3
"""Validate a retained successful dry-run sequence execution result."""

import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float64, String


TIMEOUT_SEC = 60.0


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited key-value status string."""
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' in item:
            key, value = item.split('=', 1)
            fields[key] = value
    return fields


class DryRunExecutionStatusChecker(Node):
    """Collect the one-shot retained aggregate dry-run result."""

    def __init__(self) -> None:
        """Create transient-local result subscriptions."""
        super().__init__('dry_run_execution_status_checker')
        self.status_messages: List[str] = []
        self.success: Optional[bool] = None
        self.duration_ms: Optional[float] = None
        self.failure = ''

        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            '/assembly_execution_status',
            self._status_callback,
            result_qos,
        )
        self.create_subscription(
            Bool,
            '/assembly_execution_success',
            self._success_callback,
            result_qos,
        )
        self.create_subscription(
            Float64,
            '/assembly_execution_duration_ms',
            self._duration_callback,
            result_qos,
        )

    def _status_callback(self, message: String) -> None:
        """Validate the single aggregate status message."""
        self.status_messages.append(message.data)
        if len(self.status_messages) > 1:
            self.failure = (
                '/assembly_execution_status published more than once'
            )
            return

        fields = parse_status(message.data)
        expected = {
            'event': 'success',
            'mode': 'dry_run',
            'pre_grasp_received': 'true',
            'assembly_received': 'true',
            'pre_grasp_valid': 'true',
            'assembly_valid': 'true',
            'stage_count': '2',
            'execution': 'true',
            'real_execution': 'false',
        }
        for key, value in expected.items():
            if fields.get(key) != value:
                self.failure = (
                    f'expected {key}={value} in status: {message.data}'
                )
                return
        try:
            status_duration = float(fields['duration_ms'])
        except (KeyError, ValueError):
            self.failure = f'invalid duration_ms in status: {message.data}'
            return
        if status_duration < 0.0:
            self.failure = f'negative duration_ms in status: {message.data}'

    def _success_callback(self, message: Bool) -> None:
        """Record and validate aggregate success."""
        self.success = message.data
        if not message.data:
            self.failure = '/assembly_execution_success was false'

    def _duration_callback(self, message: Float64) -> None:
        """Record and validate aggregate duration."""
        self.duration_ms = message.data
        if message.data < 0.0:
            self.failure = '/assembly_execution_duration_ms was negative'

    def complete(self) -> bool:
        """Return whether every aggregate result has been received."""
        return (
            len(self.status_messages) == 1
            and self.success is True
            and self.duration_ms is not None
        )


def main() -> int:
    """Wait for and validate the known-reachable dry-run result."""
    rclpy.init()
    node = DryRunExecutionStatusChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.failure:
                print(f'FAIL: {node.failure}')
                return 1
            if node.complete():
                print('PASS: observed one successful dry-run execution result')
                print(f'      status: {node.status_messages[0]}')
                print(f'      duration_ms: {node.duration_ms:.6f}')
                return 0

        print('FAIL: timed out waiting for the dry-run execution result')
        print(f'      status count: {len(node.status_messages)}')
        print(f'      success: {node.success}')
        print(f'      duration_ms: {node.duration_ms}')
        print(
            'Start adaptive_assembly_panda_sequence_dry_run_execution.'
            'launch.py '
            'before running this checker.'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
