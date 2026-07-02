#!/usr/bin/env python3
"""Validate retained simulated vision status."""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Checker(Node):
    """Capture one retained status message."""

    def __init__(self, topic: str) -> None:
        super().__init__('simulated_vision_status_checker')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status = None
        self.create_subscription(String, topic, self._callback, qos)

    def _callback(self, message: String) -> None:
        self.status = message.data


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--topic', default='/simulated_vision_perception_status'
    )
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = Checker(args.topic)
    end = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while node.status is None and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    status = node.status
    node.destroy_node()
    rclpy.shutdown()
    if status is None:
        print(f'FAIL: no retained status on {args.topic}')
        return 1
    fields = dict(part.split('=', 1) for part in status.split(';'))
    expected = {
        'event': 'success',
        'mode': 'simulated_vision_perception',
        'source': 'marker_pose_emulator',
        'perceived_frame': 'simulated_camera',
        'target_frame': 'target_object',
        'simulated_only': 'true',
        'real_hardware': 'false',
    }
    if any(fields.get(key) != value for key, value in expected.items()):
        print(f'FAIL: invalid retained status: {status}')
        return 1
    print(f'PASS: valid retained simulated vision status: {status}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
