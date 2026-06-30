#!/usr/bin/env python3
"""Validate recovery supervision of the known-reachable dry-run path."""

import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String


TIMEOUT_SEC = 60.0


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited key/value status."""
    return dict(
        item.split('=', 1)
        for item in status.split(';')
        if '=' in item
    )


class RecoverySuccessChecker(Node):
    """Collect retained recovery status and success results."""

    def __init__(self) -> None:
        """Create recovery result subscriptions."""
        super().__init__('recovery_supervisor_success_checker')
        self.statuses: List[str] = []
        self.success: Optional[bool] = None
        self.failure = ''
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/assembly_recovery_status', self._status_callback, qos
        )
        self.create_subscription(
            Bool, '/assembly_recovery_success', self._success_callback, qos
        )

    def _status_callback(self, message: String) -> None:
        self.statuses.append(message.data)
        fields = parse_status(message.data)
        if fields.get('real_execution') != 'false':
            self.failure = (
                'recovery status omitted real_execution=false: '
                + message.data
            )

    def _success_callback(self, message: Bool) -> None:
        self.success = message.data

    def complete(self) -> bool:
        """Return true after the terminal success status and Bool arrive."""
        terminal = any(
            parse_status(status).get('event') == 'success'
            and parse_status(status).get('state') == 'EXECUTION_SUCCEEDED'
            and parse_status(status).get('mode') == 'dry_run'
            for status in self.statuses
        )
        return terminal and self.success is True


def main() -> int:
    """Wait for a supervised dry-run success."""
    rclpy.init()
    node = RecoverySuccessChecker()
    deadline = time.monotonic() + TIMEOUT_SEC
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.failure:
                print(f'FAIL: {node.failure}')
                return 1
            if node.complete():
                terminal = next(
                    status for status in reversed(node.statuses)
                    if parse_status(status).get('state')
                    == 'EXECUTION_SUCCEEDED'
                )
                print('PASS: recovery supervisor observed dry-run success')
                print(f'      status: {terminal}')
                return 0
        print('FAIL: timed out waiting for EXECUTION_SUCCEEDED')
        print(
            'Start adaptive_assembly_recovery_supervisor_demo.launch.py '
            'before running this checker.'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
