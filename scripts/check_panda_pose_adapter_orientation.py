#!/usr/bin/env python3
"""Validate the default fixed orientation of the Panda-adapted pose."""

import sys
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


EXPECTED_QX = 1.0
EXPECTED_QY = 0.0
EXPECTED_QZ = 0.0
EXPECTED_QW = 0.0
TOLERANCE = 1e-6
TIMEOUT_SEC = 15.0


class PandaPoseOrientationChecker(Node):
    """Subscribe once to the Panda-adapted pre-grasp pose."""

    def __init__(self) -> None:
        """Create the pose subscription."""
        super().__init__('panda_pose_orientation_checker')
        self.pose: Optional[PoseStamped] = None
        self.create_subscription(
            PoseStamped,
            '/panda_pre_grasp_pose',
            self._pose_callback,
            10,
        )

    def _pose_callback(self, message: PoseStamped) -> None:
        self.pose = message


def _close(actual: float, expected: float) -> bool:
    return abs(actual - expected) <= TOLERANCE


def _validate_orientation(pose: PoseStamped) -> int:
    orientation = pose.pose.orientation
    checks = [
        ('x', orientation.x, EXPECTED_QX),
        ('y', orientation.y, EXPECTED_QY),
        ('z', orientation.z, EXPECTED_QZ),
        ('w', orientation.w, EXPECTED_QW),
    ]
    failures = [
        f'{name}: actual={actual:.6f}, expected={expected:.6f}'
        for name, actual, expected in checks
        if not _close(actual, expected)
    ]

    if failures:
        print('FAIL: /panda_pre_grasp_pose orientation does not match fixed default')
        for failure in failures:
            print(f'      {failure}')
        return 1

    print(
        'PASS: /panda_pre_grasp_pose orientation matches fixed default '
        '(x=1.0, y=0.0, z=0.0, w=0.0)'
    )
    return 0


def main() -> int:
    """Wait for one adapted pose and validate its orientation."""
    rclpy.init()
    node = PandaPoseOrientationChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pose is not None:
                return _validate_orientation(node.pose)

        print('FAIL: timed out waiting for /panda_pre_grasp_pose')
        print(
            'Start the Panda planning demo first with: ros2 launch '
            'adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
