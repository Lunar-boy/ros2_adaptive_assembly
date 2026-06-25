#!/usr/bin/env python3
"""Validate the semicolon-separated planning status message format."""

import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TIMEOUT_SEC = 20.0
REQUIRED_KEYS = {
    'event',
    'frame',
    'x',
    'y',
    'z',
    'distance_from_last_plan',
    'min_replan_distance',
    'duration_ms',
    'execution',
    'planner_id',
    'num_planning_attempts',
    'max_velocity_scaling_factor',
    'max_acceleration_scaling_factor',
    'guard_enabled',
    'guard_passed',
    'guard_reason',
}
VALID_EVENTS = {
    'success',
    'failure',
    'skipped_small_motion',
    'guard_rejected',
}


class PlanningStatusChecker(Node):
    """Subscribe once to the planning status topic."""

    def __init__(self) -> None:
        """Create the status subscription."""
        super().__init__('planning_status_format_checker')
        self.status: Optional[str] = None
        self.create_subscription(
            String,
            '/pre_grasp_planning_status',
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


def _is_float(value: str) -> bool:
    try:
        float(value)
    except ValueError:
        return False
    return True


def _validate_status(status: str) -> int:
    fields = _parse_status(status)
    missing_keys = sorted(REQUIRED_KEYS - fields.keys())
    failures = []

    if missing_keys:
        failures.append(f'missing keys: {", ".join(missing_keys)}')

    event = fields.get('event')
    if event is not None and event not in VALID_EVENTS:
        failures.append(f'event has unexpected value: {event}')

    execution = fields.get('execution')
    if execution is not None and execution != 'false':
        failures.append(f'execution must be false, got: {execution}')

    for key in ('guard_enabled', 'guard_passed'):
        value = fields.get(key)
        if value is not None and value not in {'true', 'false'}:
            failures.append(f'{key} must be true or false, got: {value}')

    guard_reason = fields.get('guard_reason')
    if guard_reason is not None and guard_reason == '':
        failures.append('guard_reason must not be empty')

    for key in (
        'x',
        'y',
        'z',
        'duration_ms',
        'max_velocity_scaling_factor',
        'max_acceleration_scaling_factor',
    ):
        value = fields.get(key)
        if value is not None and not _is_float(value):
            failures.append(f'{key} is not parseable as float: {value}')

    attempts = fields.get('num_planning_attempts')
    if attempts is not None:
        try:
            if int(attempts) < 1:
                failures.append(
                    f'num_planning_attempts must be >= 1, got: {attempts}'
                )
        except ValueError:
            failures.append(
                f'num_planning_attempts is not parseable as int: {attempts}'
            )

    if failures:
        print('FAIL: /pre_grasp_planning_status has invalid format')
        print(f'      message: {status}')
        for failure in failures:
            print(f'      {failure}')
        return 1

    print('PASS: /pre_grasp_planning_status format is valid')
    print(f'      message: {status}')
    return 0


def main() -> int:
    """Wait for one planning status message and validate it."""
    rclpy.init()
    node = PlanningStatusChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status is not None:
                return _validate_status(node.status)

        print('FAIL: timed out waiting for /pre_grasp_planning_status')
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
