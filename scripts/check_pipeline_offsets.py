#!/usr/bin/env python3
"""Validate z offsets in the current adaptive assembly pipeline."""

import sys
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


PRE_GRASP_OFFSET = 0.20
ASSEMBLY_OFFSET = 0.05
TOLERANCE = 1e-6
TIMEOUT_SEC = 15.0


class PipelineOffsetChecker(Node):
    """Subscribe to pipeline pose topics and validate derived z offsets."""

    def __init__(self) -> None:
        """Create subscriptions for the current pipeline pose topics."""
        super().__init__('pipeline_offset_checker')
        self.target_pose: Optional[PoseStamped] = None
        self.pre_grasp_pose: Optional[PoseStamped] = None
        self.assembly_pose: Optional[PoseStamped] = None

        self.create_subscription(
            PoseStamped,
            '/target_pose',
            self._target_pose_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/pre_grasp_pose',
            self._pre_grasp_pose_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/assembly_pose',
            self._assembly_pose_callback,
            10,
        )

    def received_all_messages(self) -> bool:
        """Return whether one message has arrived from every pose topic."""
        return (
            self.target_pose is not None
            and self.pre_grasp_pose is not None
            and self.assembly_pose is not None
        )

    def _target_pose_callback(self, message: PoseStamped) -> None:
        self.target_pose = message

    def _pre_grasp_pose_callback(self, message: PoseStamped) -> None:
        self.pre_grasp_pose = message

    def _assembly_pose_callback(self, message: PoseStamped) -> None:
        self.assembly_pose = message


def _within_tolerance(actual: float, expected: float) -> bool:
    return abs(actual - expected) <= TOLERANCE


def _validate_offsets(node: PipelineOffsetChecker) -> int:
    target_z = node.target_pose.pose.position.z
    pre_grasp_z = node.pre_grasp_pose.pose.position.z
    assembly_z = node.assembly_pose.pose.position.z

    expected_pre_grasp_z = target_z + PRE_GRASP_OFFSET
    expected_assembly_z = target_z + ASSEMBLY_OFFSET

    pre_grasp_ok = _within_tolerance(pre_grasp_z, expected_pre_grasp_z)
    assembly_ok = _within_tolerance(assembly_z, expected_assembly_z)

    if pre_grasp_ok:
        print(
            'PASS: /pre_grasp_pose z offset is correct '
            f'({pre_grasp_z:.6f} == {expected_pre_grasp_z:.6f})'
        )
    else:
        print(
            'FAIL: /pre_grasp_pose z offset is incorrect '
            f'({pre_grasp_z:.6f} != {expected_pre_grasp_z:.6f})'
        )

    if assembly_ok:
        print(
            'PASS: /assembly_pose z offset is correct '
            f'({assembly_z:.6f} == {expected_assembly_z:.6f})'
        )
    else:
        print(
            'FAIL: /assembly_pose z offset is incorrect '
            f'({assembly_z:.6f} != {expected_assembly_z:.6f})'
        )

    if pre_grasp_ok and assembly_ok:
        print('PASS: all pipeline offset checks passed')
        return 0

    return 1


def main() -> int:
    """Wait for one message on each topic and validate z offsets."""
    rclpy.init()
    node = PipelineOffsetChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.received_all_messages():
                return _validate_offsets(node)

        missing_topics = []
        if node.target_pose is None:
            missing_topics.append('/target_pose')
        if node.pre_grasp_pose is None:
            missing_topics.append('/pre_grasp_pose')
        if node.assembly_pose is None:
            missing_topics.append('/assembly_pose')

        print(
            'FAIL: timed out waiting for pipeline messages from '
            f'{", ".join(missing_topics)}'
        )
        print(
            'Start the pipeline first with: ros2 launch '
            'adaptive_assembly_bringup adaptive_assembly_pipeline.launch.py'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
