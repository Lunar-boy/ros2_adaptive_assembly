#!/usr/bin/env python3
"""Validate one PlanningScene audit status message."""

import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TIMEOUT_SEC = 20.0
REQUIRED_KEYS = {
    'event',
    'expected',
    'present',
    'missing',
    'all_present',
}


class PlanningSceneAuditStatusChecker(Node):
    """Subscribe once to the PlanningScene audit status topic."""

    def __init__(self) -> None:
        """Create the status subscription."""
        super().__init__('planning_scene_audit_status_checker')
        self.status: Optional[str] = None
        self.create_subscription(
            String,
            '/planning_scene_audit_status',
            self._status_callback,
            10,
        )

    def _status_callback(self, message: String) -> None:
        self.status = message.data


def _parse_status(status: str) -> Dict[str, str]:
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' not in item:
            continue
        key, value = item.split('=', 1)
        fields[key] = value
    return fields


def _validate_status(status: str) -> int:
    fields = _parse_status(status)
    missing = sorted(REQUIRED_KEYS - fields.keys())
    failures = []

    if missing:
        failures.append(f'missing keys: {", ".join(missing)}')

    if fields.get('event') != 'audit':
        failures.append(f'event must be audit, got: {fields.get("event")}')

    all_present = fields.get('all_present')
    if all_present not in {'true', 'false'}:
        failures.append(f'all_present must be true or false, got: {all_present}')

    if not fields.get('expected'):
        failures.append('expected must not be empty')

    if failures:
        print('FAIL: /planning_scene_audit_status has invalid format')
        print(f'      message: {status}')
        for failure in failures:
            print(f'      {failure}')
        return 1

    print('PASS: /planning_scene_audit_status format is valid')
    print(f'      message: {status}')
    return 0


def main() -> int:
    """Wait for one audit status message and validate it."""
    rclpy.init()
    node = PlanningSceneAuditStatusChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status is not None:
                return _validate_status(node.status)

        print('FAIL: timed out waiting for /planning_scene_audit_status')
        print(
            'Start the Panda planning demo first with: ros2 launch '
            'adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
