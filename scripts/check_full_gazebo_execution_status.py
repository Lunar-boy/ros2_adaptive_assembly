#!/usr/bin/env python3
"""Validate the retained terminal status from full Gazebo execution."""

import argparse
import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float64, String


def parse_status(status: str) -> Dict[str, str]:
    """Parse semicolon-delimited key/value status fields."""
    return dict(
        item.split('=', 1) for item in status.split(';') if '=' in item
    )


class FullGazeboExecutionStatusChecker(Node):
    """Collect the retained aggregate execution result."""

    def __init__(self) -> None:
        super().__init__('full_gazebo_execution_status_checker')
        self.status: Optional[str] = None
        self.success: Optional[bool] = None
        self.duration_ms: Optional[float] = None
        self.failure = ''
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            '/assembly_ros2_control_execution_status',
            self._status_callback,
            qos,
        )
        self.create_subscription(
            Bool,
            '/assembly_ros2_control_execution_success',
            self._success_callback,
            qos,
        )
        self.create_subscription(
            Float64,
            '/assembly_ros2_control_execution_duration_ms',
            self._duration_callback,
            qos,
        )

    def _status_callback(self, message: String) -> None:
        if self.status is not None:
            self.failure = 'aggregate status published more than once'
            return
        self.status = message.data
        fields = parse_status(message.data)
        if fields.get('event') not in {'success', 'failure', 'skipped'}:
            self.failure = f'invalid terminal event: {message.data}'
        elif fields.get('mode') != 'ros2_control':
            self.failure = f'missing mode=ros2_control: {message.data}'
        elif fields.get('simulated_execution_only') != 'true':
            self.failure = (
                f'missing simulated_execution_only=true: {message.data}'
            )
        elif fields.get('real_hardware') != 'false':
            self.failure = f'missing real_hardware=false: {message.data}'

    def _success_callback(self, message: Bool) -> None:
        self.success = message.data

    def _duration_callback(self, message: Float64) -> None:
        self.duration_ms = message.data
        if message.data < 0.0:
            self.failure = 'execution duration was negative'

    def complete(self) -> bool:
        return (
            self.status is not None
            and self.success is not None
            and self.duration_ms is not None
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-sec', type=float, default=90.0)
    parser.add_argument(
        '--allow-non-success',
        action='store_true',
        help='Accept deterministic failure/skipped terminal states.',
    )
    args = parser.parse_args()

    rclpy.init()
    node = FullGazeboExecutionStatusChecker()
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.failure:
                print(f'FAIL: {node.failure}')
                return 1
            if node.complete():
                fields = parse_status(node.status)
                expected_success = fields['event'] == 'success'
                if node.success != expected_success:
                    print('FAIL: success Bool conflicts with terminal event')
                    return 1
                if not args.allow_non_success and not expected_success:
                    print('FAIL: full Gazebo execution did not succeed')
                    print(f'      status: {node.status}')
                    return 1
                print('PASS: observed a valid full Gazebo terminal result')
                print(f'      status: {node.status}')
                print(f'      duration_ms: {node.duration_ms:.6f}')
                return 0
        print('FAIL: timed out waiting for full Gazebo execution result')
        print(
            'Start adaptive_assembly_full_gazebo_execution_demo.launch.py '
            'before running this checker.'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
