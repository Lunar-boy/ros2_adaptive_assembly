#!/usr/bin/env python3
"""Validate one plan-only assembly sequence status event."""

import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TIMEOUT_SEC = 20.0
REQUIRED_KEYS = {
    'event',
    'failed_stage',
    'planned_stage_count',
    'total_duration_ms',
    'execution',
}


class AssemblySequenceStatusChecker(Node):
    """Capture one assembly sequence planning status message."""

    def __init__(self) -> None:
        """Create the status subscription."""
        super().__init__('assembly_sequence_status_checker')
        self.status: Optional[str] = None
        self.create_subscription(
            String,
            '/assembly_sequence_planning_status',
            self._status_callback,
            10,
        )

    def _status_callback(self, message: String) -> None:
        self.status = message.data


def _parse_status(status: str) -> Dict[str, str]:
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' in item:
            key, value = item.split('=', 1)
            fields[key] = value
    return fields


def _validate(status: str) -> int:
    fields = _parse_status(status)
    failures = []
    missing = sorted(REQUIRED_KEYS - fields.keys())
    if missing:
        failures.append(f'missing keys: {", ".join(missing)}')

    if fields.get('event') not in {'success', 'failure'}:
        failures.append(f"unexpected event: {fields.get('event')}")
    if fields.get('failed_stage') not in {'none', 'pre_grasp', 'assembly'}:
        failures.append(
            f"unexpected failed_stage: {fields.get('failed_stage')}"
        )
    if fields.get('execution') != 'false':
        failures.append('execution must be false')

    try:
        stage_count = int(fields.get('planned_stage_count', ''))
        if stage_count < 0 or stage_count > 2:
            failures.append('planned_stage_count must be between 0 and 2')
    except ValueError:
        failures.append('planned_stage_count is not an integer')

    try:
        if float(fields.get('total_duration_ms', '')) < 0.0:
            failures.append('total_duration_ms must be non-negative')
    except ValueError:
        failures.append('total_duration_ms is not a float')

    if failures:
        print('FAIL: invalid /assembly_sequence_planning_status message')
        print(f'      message: {status}')
        for failure in failures:
            print(f'      {failure}')
        return 1

    print('PASS: assembly sequence planning status format is valid')
    print(f'      message: {status}')
    return 0


def main() -> int:
    """Wait for and validate one status event."""
    rclpy.init()
    node = AssemblySequenceStatusChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status is not None:
                return _validate(node.status)

        print('FAIL: timed out waiting for /assembly_sequence_planning_status')
        print(
            'Start adaptive_assembly_panda_sequence_planning_demo.launch.py '
            'before running this checker.'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
