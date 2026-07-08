#!/usr/bin/env python3
"""Validate successful pre-grasp and assembly stage diagnostics."""

import sys
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TIMEOUT_SEC = 60.0
REQUIRED_KEYS = {
    'event',
    'stage',
    'stage_index',
    'requested_stage_count',
    'duration_ms',
    'planning_group',
    'planner_id',
    'num_planning_attempts',
    'planning_time_sec',
    'position_tolerance',
    'orientation_tolerance',
    'start_state_mode',
    'execution',
}


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited key-value status string."""
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' in item:
            key, value = item.split('=', 1)
            fields[key] = value
    return fields


class AssemblySequenceStageStatusChecker(Node):
    """Collect successful diagnostics for both sequence stages."""

    def __init__(self) -> None:
        """Create the stage status subscription."""
        super().__init__('assembly_sequence_stage_status_checker')
        self.success_statuses: Dict[str, str] = {}
        self.invalid_status = ''
        self.create_subscription(
            String,
            '/assembly_sequence_stage_status',
            self.status_callback,
            10,
        )

    def status_callback(self, message: String) -> None:
        """Validate one event and retain successful stage messages."""
        fields = parse_status(message.data)
        missing = REQUIRED_KEYS - fields.keys()
        if missing:
            self.invalid_status = (
                f'missing keys {sorted(missing)} in message: {message.data}'
            )
            return

        stage = fields['stage']
        if stage not in {'pre_grasp', 'assembly'}:
            self.invalid_status = f'invalid stage in message: {message.data}'
            return
        if fields['execution'] != 'false':
            self.invalid_status = f'execution must be false: {message.data}'
            return
        if not fields['start_state_mode']:
            self.invalid_status = f'start_state_mode is empty: {message.data}'
            return
        try:
            duration_ms = float(fields['duration_ms'])
        except ValueError:
            self.invalid_status = f'duration_ms is not parseable: {message.data}'
            return
        if duration_ms < 0.0:
            self.invalid_status = f'duration_ms is negative: {message.data}'
            return

        if fields['event'] == 'success':
            self.success_statuses[stage] = message.data


def main() -> int:
    """Wait for successful diagnostics from both reachable sequence stages."""
    rclpy.init()
    node = AssemblySequenceStageStatusChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.invalid_status:
                print(f'FAIL: {node.invalid_status}')
                return 1
            if {'pre_grasp', 'assembly'} <= node.success_statuses.keys():
                print('PASS: successful diagnostics received for both stages')
                print(
                    '      pre_grasp: '
                    f"{node.success_statuses['pre_grasp']}"
                )
                print(
                    '      assembly: '
                    f"{node.success_statuses['assembly']}"
                )
                return 0

        print('FAIL: timed out waiting for successful diagnostics from both stages')
        print(f'      stages received: {sorted(node.success_statuses)}')
        print(
            'Start adaptive_assembly_panda_sequence_planning_reachable.launch.py '
            'before running this checker.'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
