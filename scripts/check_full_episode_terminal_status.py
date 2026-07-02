#!/usr/bin/env python3
"""Validate the full assembly episode's terminal status contract."""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String


TERMINAL_EVENTS = {'success', 'failure', 'timeout', 'skipped'}


def parse_status(value: str):
    fields = {}
    for fragment in value.split(';'):
        if '=' in fragment:
            key, field_value = fragment.split('=', 1)
            fields[key.strip()] = field_value.strip()
    return fields


class TerminalStatusCheck(Node):
    """Collect retained episode outputs and supporting runtime statuses."""

    def __init__(self):
        super().__init__('check_full_episode_terminal_status')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status = None
        self.success = None
        self.duration_ms = None
        self.insertion_status = None
        self.pose_status = None
        self.subscriptions = [
            self.create_subscription(String, '/assembly_episode_status', self._status, qos),
            self.create_subscription(Bool, '/assembly_episode_success', self._success, qos),
            self.create_subscription(
                Float64, '/assembly_episode_duration_ms', self._duration, qos
            ),
            self.create_subscription(String, '/assembly_insertion_status', self._insertion, qos),
            self.create_subscription(String, '/gazebo_target_object_pose_status', self._pose, qos),
        ]

    def _status(self, message):
        fields = parse_status(message.data)
        if fields.get('event') in TERMINAL_EVENTS:
            self.status = fields

    def _success(self, message):
        self.success = bool(message.data)

    def _duration(self, message):
        self.duration_ms = float(message.data)

    def _insertion(self, message):
        self.insertion_status = message.data

    def _pose(self, message):
        self.pose_status = message.data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-sec', type=float, default=180.0)
    parser.add_argument('--allow-non-success', action='store_true')
    args = parser.parse_args()
    if args.timeout_sec <= 0.0:
        parser.error('--timeout-sec must be positive')

    rclpy.init()
    node = TerminalStatusCheck()
    deadline = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.status is not None
                and node.success is not None
                and node.duration_ms is not None
            ):
                break
        else:
            print('FAIL: timed out waiting for complete terminal episode status')
            return 1

        fields = node.status
        event = fields.get('event')
        expected = {
            'mode': 'assembly_episode',
            'simulated_only': 'true',
            'real_hardware': 'false',
        }
        for key, value in expected.items():
            if fields.get(key) != value:
                print(f'FAIL: terminal status requires {key}={value}')
                return 1
        if event not in TERMINAL_EVENTS:
            print(f'FAIL: invalid terminal event: {event}')
            return 1
        if node.success != (event == 'success'):
            print('FAIL: /assembly_episode_success does not match terminal event')
            return 1
        if node.duration_ms < 0.0:
            print('FAIL: episode duration must be non-negative')
            return 1
        print(f'Terminal status: event={event}; duration_ms={node.duration_ms:.3f}')
        if event != 'success' and not args.allow_non_success:
            print('FAIL: episode did not succeed (use --allow-non-success when expected)')
            return 1
        print('PASS: full episode terminal status is valid')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
