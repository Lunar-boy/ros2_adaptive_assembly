#!/usr/bin/env python3
"""Check detector pose frames and world-to-target TF consistency."""

import argparse
import math

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener


class Checker(Node):
    """Capture detector poses and target TF."""

    def __init__(self, perceived_topic, target_topic):
        """Subscribe to poses and start a TF listener."""
        super().__init__('opencv_aruco_pose_checker')
        self.perceived = None
        self.target = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            PoseStamped, perceived_topic, self._perceived, 10)
        self.create_subscription(PoseStamped, target_topic, self._target, 10)

    def _perceived(self, message):
        self.perceived = message

    def _target(self, message):
        self.target = message


def main():
    """Validate pose frames and target TF equivalence."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--perceived-topic', default='/perceived_target_pose')
    parser.add_argument('--target-topic', default='/target_pose')
    parser.add_argument('--camera-frame', default='simulated_camera')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--target-frame', default='target_object')
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    parser.add_argument('--tolerance', type=float, default=1e-5)
    args = parser.parse_args()
    rclpy.init()
    node = Checker(args.perceived_topic, args.target_topic)
    transform = None
    end = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.target is None or node.perceived is None:
            continue
        try:
            transform = node.buffer.lookup_transform(
                args.world_frame, args.target_frame, Time())
            break
        except TransformException:
            pass
    perceived, target = node.perceived, node.target
    node.destroy_node()
    rclpy.shutdown()
    if perceived is None or target is None or transform is None:
        print('FAIL: detector poses or target TF were not received')
        return 1
    tf = transform.transform
    pose = target.pose
    errors = [pose.position.x - tf.translation.x,
              pose.position.y - tf.translation.y,
              pose.position.z - tf.translation.z,
              pose.orientation.x - tf.rotation.x,
              pose.orientation.y - tf.rotation.y,
              pose.orientation.z - tf.rotation.z,
              pose.orientation.w - tf.rotation.w]
    if (perceived.header.frame_id != args.camera_frame
            or target.header.frame_id != args.world_frame
            or not all(math.isfinite(value) for value in errors)
            or math.sqrt(sum(value * value for value in errors))
            > args.tolerance):
        print('FAIL: invalid frames or target pose/TF mismatch')
        return 1
    print('PASS: camera/world pose frames and world -> target TF are consistent')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
