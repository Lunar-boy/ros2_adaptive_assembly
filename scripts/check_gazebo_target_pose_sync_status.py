#!/usr/bin/env python3
"""Validate one retained Gazebo target synchronization status message."""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String


class StatusChecker(Node):
    """Receive the retained synchronization status."""

    def __init__(self, topic: str) -> None:
        """Subscribe to the configured status topic."""
        super().__init__('gazebo_target_pose_sync_status_checker')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status = None
        self.create_subscription(String, topic, self._callback, qos)

    def _callback(self, message: String) -> None:
        self.status = message.data


def main() -> int:
    """Wait for and validate one retained status message."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/gazebo_target_sync_status')
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = StatusChecker(args.topic)
    end = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while node.status is None and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    status = node.status
    node.destroy_node()
    rclpy.shutdown()
    if status is None:
        print(f'FAIL: no retained status received on {args.topic}')
        return 1
    fields = dict(
        part.split('=', 1) for part in status.split(';') if '=' in part
    )
    required = {
        'mode': 'gazebo_target_sync',
        'simulated_only': 'true',
        'real_hardware': 'false',
    }
    if fields.get('event') not in {'success', 'skipped', 'failure'}:
        print(f'FAIL: invalid event in status: {status}')
        return 1
    for key, expected in required.items():
        if fields.get(key) != expected:
            print(f'FAIL: expected {key}={expected}: {status}')
            return 1
    print(f'PASS: valid retained target sync status: {status}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
