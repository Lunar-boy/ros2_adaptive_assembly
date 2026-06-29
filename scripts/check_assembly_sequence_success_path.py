#!/usr/bin/env python3
"""Wait for a successful fixed-start two-stage assembly planning event."""

import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TIMEOUT_SEC = 60.0


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-separated key-value status string."""
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' in item:
            key, value = item.split('=', 1)
            fields[key] = value
    return fields


class AssemblySequenceSuccessChecker(Node):
    """Capture sequence events until the required success event arrives."""

    def __init__(self) -> None:
        """Create the planning status subscription."""
        super().__init__('assembly_sequence_success_path_checker')
        self.success_status: Optional[str] = None
        self.last_status: Optional[str] = None
        self.create_subscription(
            String,
            '/assembly_sequence_planning_status',
            self.status_callback,
            10,
        )

    def status_callback(self, message: String) -> None:
        """Record status and retain it when all success criteria match."""
        self.last_status = message.data
        fields = parse_status(message.data)
        if (
            fields.get('event') == 'success'
            and fields.get('failed_stage') == 'none'
            and fields.get('planned_stage_count') == '2'
            and fields.get('start_state_mode') == 'fixed'
            and fields.get('execution') == 'false'
        ):
            self.success_status = message.data


def main() -> int:
    """Wait for a successful sequence event or return a clear failure."""
    rclpy.init()
    node = AssemblySequenceSuccessChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.success_status is not None:
                print('PASS: observed successful fixed-start assembly sequence')
                print(f'      message: {node.success_status}')
                return 0

        print('FAIL: no successful fixed-start assembly sequence within 60 seconds')
        if node.last_status is not None:
            print(f'      last message: {node.last_status}')
        else:
            print('      no /assembly_sequence_planning_status message received')
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
