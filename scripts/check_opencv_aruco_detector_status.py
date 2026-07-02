#!/usr/bin/env python3
"""Validate the retained optional ArUco detector status and bool."""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String


class Checker(Node):
    """Capture retained detector diagnostics."""

    def __init__(self, status_topic, detected_topic):
        """Subscribe to the configured retained topics."""
        super().__init__('opencv_aruco_status_checker')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status = None
        self.detected = None
        self.create_subscription(String, status_topic, self._status, qos)
        self.create_subscription(Bool, detected_topic, self._detected, qos)

    def _status(self, message):
        self.status = message.data

    def _detected(self, message):
        self.detected = message.data


def main():
    """Wait for and validate one retained detector state."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--status-topic', default='/aruco_detection_status')
    parser.add_argument('--detected-topic', default='/aruco_marker_detected')
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = Checker(args.status_topic, args.detected_topic)
    end = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while (node.status is None or node.detected is None) and (
            node.get_clock().now().nanoseconds < end):
        rclpy.spin_once(node, timeout_sec=0.1)
    status, detected = node.status, node.detected
    node.destroy_node()
    rclpy.shutdown()
    if status is None or detected is None:
        print('FAIL: retained detector status or detected bool was not received')
        return 1
    try:
        fields = dict(part.split('=', 1) for part in status.split(';'))
    except ValueError:
        print(f'FAIL: malformed status: {status}')
        return 1
    if (fields.get('mode') != 'opencv_aruco_detector'
            or fields.get('simulated_only') != 'true'
            or fields.get('real_hardware') != 'false'
            or fields.get('event') not in ('success', 'skipped', 'failure')):
        print(f'FAIL: invalid detector status: {status}')
        return 1
    if fields['event'] == 'success' and not detected:
        print('FAIL: success status has a false marker-detected value')
        return 1
    print(f'PASS: valid retained detector state: {status};detected={detected}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
