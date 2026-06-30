#!/usr/bin/env python3
"""Inject an assembly planning failure and validate the recovery action."""

import sys
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String


TIMEOUT_SEC = 20.0
EXPECTED_ACTION = 'clear_dynamic_target_and_retry'


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited key/value status."""
    return dict(
        item.split('=', 1)
        for item in status.split(';')
        if '=' in item
    )


class RecoveryFailureInjector(Node):
    """Publish a synthetic failure and observe the deterministic action."""

    def __init__(self) -> None:
        """Create a retained publisher and action/status subscriptions."""
        super().__init__('recovery_supervisor_failure_injector')
        self.actions: List[str] = []
        self.statuses: List[str] = []
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String, '/assembly_sequence_planning_status', qos
        )
        self.create_subscription(
            String,
            '/assembly_recovery_action',
            self._action_callback,
            qos,
        )
        self.create_subscription(
            String,
            '/assembly_recovery_status',
            self._status_callback,
            qos,
        )
        self.timer = self.create_timer(0.25, self._publish_failure)

    def _publish_failure(self) -> None:
        message = String()
        message.data = (
            'event=failure;failed_stage=assembly;planned_stage_count=1;'
            'execution=false;real_execution=false'
        )
        self.publisher.publish(message)

    def _action_callback(self, message: String) -> None:
        self.actions.append(message.data)

    def _status_callback(self, message: String) -> None:
        self.statuses.append(message.data)

    def matching_action(self) -> str:
        """Return the expected recovery action status when observed."""
        for action in reversed(self.actions):
            fields = parse_status(action)
            if (
                fields.get('event') == 'recovery_action'
                and fields.get('action') == EXPECTED_ACTION
                and fields.get('reason') == 'assembly_planning_failed'
                and fields.get('service_calls') == 'false'
                and fields.get('real_execution') == 'false'
            ):
                return action
        return ''


def main() -> int:
    """Publish until the supervisor returns the expected action."""
    rclpy.init()
    node = RecoveryFailureInjector()
    deadline = time.monotonic() + TIMEOUT_SEC
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            invalid = [
                status for status in node.statuses
                if parse_status(status).get('real_execution') != 'false'
            ]
            if invalid:
                print(
                    'FAIL: recovery status omitted real_execution=false: '
                    + invalid[0]
                )
                return 1
            action = node.matching_action()
            if action:
                print('PASS: observed deterministic assembly recovery action')
                print(f'      action: {action}')
                return 0
        print(
            'FAIL: timed out waiting for '
            f'action={EXPECTED_ACTION}'
        )
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
