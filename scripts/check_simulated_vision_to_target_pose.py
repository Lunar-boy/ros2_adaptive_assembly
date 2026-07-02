#!/usr/bin/env python3
"""Check that simulated vision publishes a valid target pose."""

import argparse
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class Checker(Node):
    """Capture target and camera-frame perceived poses."""

    def __init__(self, topic: str, perceived_topic: str) -> None:
        super().__init__('simulated_vision_target_pose_checker')
        self.pose = None
        self.perceived_pose = None
        self.create_subscription(PoseStamped, topic, self._callback, 10)
        self.create_subscription(
            PoseStamped, perceived_topic, self._perceived_callback, 10
        )

    def _callback(self, message: PoseStamped) -> None:
        self.pose = message

    def _perceived_callback(self, message: PoseStamped) -> None:
        self.perceived_pose = message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/target_pose')
    parser.add_argument('--perceived-topic', default='/perceived_target_pose')
    parser.add_argument('--expected-frame', default='world')
    parser.add_argument('--expected-camera-frame', default='simulated_camera')
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = Checker(args.topic, args.perceived_topic)
    end = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while (node.pose is None or node.perceived_pose is None) and (
            node.get_clock().now().nanoseconds < end):
        rclpy.spin_once(node, timeout_sec=0.1)
    pose = node.pose
    perceived_pose = node.perceived_pose
    node.destroy_node()
    rclpy.shutdown()
    if pose is None or perceived_pose is None:
        print('FAIL: target or perceived PoseStamped was not received')
        return 1
    values = [
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        pose.pose.orientation.x, pose.pose.orientation.y,
        pose.pose.orientation.z, pose.pose.orientation.w,
    ]
    norm = math.sqrt(sum(value * value for value in values[3:]))
    if (pose.header.frame_id != args.expected_frame
            or perceived_pose.header.frame_id != args.expected_camera_frame
            or not all(math.isfinite(value) for value in values)
            or abs(norm - 1.0) > 1e-6):
        print(
            'FAIL: invalid pose frames or target pose: '
            f'target={pose.header.frame_id}, '
            f'perceived={perceived_pose.header.frame_id}'
        )
        return 1
    print(
        'PASS: perceived pose is in simulated_camera and target pose is in world'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
