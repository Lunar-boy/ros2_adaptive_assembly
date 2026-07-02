#!/usr/bin/env python3
"""Compare simulated target pose with its world-to-target TF."""

import argparse
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Checker(Node):
    """Capture the pose and query its matching transform."""

    def __init__(self, topic: str) -> None:
        super().__init__('simulated_vision_tf_consistency_checker')
        self.pose = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PoseStamped, topic, self._callback, 10)

    def _callback(self, message: PoseStamped) -> None:
        self.pose = message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/target_pose')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--target-frame', default='target_object')
    parser.add_argument('--tolerance', type=float, default=1e-5)
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = Checker(args.topic)
    transform = None
    end = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None:
            continue
        try:
            transform = node.buffer.lookup_transform(
                args.world_frame, args.target_frame, Time()
            )
            break
        except TransformException:
            pass
    pose = node.pose
    node.destroy_node()
    rclpy.shutdown()
    if pose is None or transform is None:
        print('FAIL: target pose or TF was not received')
        return 1
    p = pose.pose
    t = transform.transform
    errors = [
        p.position.x - t.translation.x,
        p.position.y - t.translation.y,
        p.position.z - t.translation.z,
        p.orientation.x - t.rotation.x,
        p.orientation.y - t.rotation.y,
        p.orientation.z - t.rotation.z,
        p.orientation.w - t.rotation.w,
    ]
    if math.sqrt(sum(value * value for value in errors)) > args.tolerance:
        print(f'FAIL: pose and TF differ: errors={errors}')
        return 1
    print('PASS: /target_pose and world -> target_object TF are consistent')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
