#!/usr/bin/env python3
"""Check fixed-socket object-place and backward-compatible assembly poses."""

import math
import subprocess
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class FixedSocketChecker(Node):
    def __init__(self):
        super().__init__('fixed_socket_assembly_pose_checker')
        self.assembly_pose = None
        self.object_place_pose = None
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.create_subscription(
            PoseStamped, '/assembly_pose', self._on_assembly_pose, 10
        )
        self.create_subscription(
            PoseStamped, '/object_place_pose', self._on_object_place_pose, 10
        )

    def _on_assembly_pose(self, message):
        self.assembly_pose = message

    def _on_object_place_pose(self, message):
        self.object_place_pose = message


def main():
    rclpy.init()
    node = FixedSocketChecker()
    process = subprocess.Popen([
        'ros2', 'run', 'adaptive_assembly_task', 'assembly_task_node',
        '--ros-args', '-p', 'assembly_pose_mode:=fixed_socket',
        '-p', 'socket_x:=0.62', '-p', 'socket_y:=-0.18',
        '-p', 'socket_z:=0.10', '-p', 'socket_yaw:=0.6',
    ])
    try:
        deadline = time.monotonic() + 10.0
        source = PoseStamped()
        source.header.frame_id = 'world'
        source.pose.position.x = 0.10
        source.pose.position.y = 0.20
        source.pose.position.z = 0.15
        source.pose.orientation.w = 1.0
        while time.monotonic() < deadline and (
            node.assembly_pose is None or node.object_place_pose is None
        ):
            source.header.stamp = node.get_clock().now().to_msg()
            node.publisher.publish(source)
            rclpy.spin_once(node, timeout_sec=0.1)
        poses = (node.assembly_pose, node.object_place_pose)
        valid = all(pose is not None and all((
            math.isclose(pose.pose.position.x, 0.62, abs_tol=1e-6),
            math.isclose(pose.pose.position.y, -0.18, abs_tol=1e-6),
            math.isclose(pose.pose.position.z, 0.10, abs_tol=1e-6),
            pose.header.frame_id == 'world',
            math.isclose(
                pose.pose.orientation.z, math.sin(0.3), abs_tol=1e-6
            ),
            math.isclose(
                pose.pose.orientation.w, math.cos(0.3), abs_tol=1e-6
            ),
        )) for pose in poses)
        print('PASS: fixed socket object-place and assembly poses' if valid else
              'FAIL: fixed socket object-place or assembly pose')
        return 0 if valid else 1
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
