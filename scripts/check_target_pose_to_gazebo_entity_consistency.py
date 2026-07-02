#!/usr/bin/env python3
"""Exercise request conversion and deterministic no-Gazebo sync behavior."""

import math
import subprocess
import sys
import time

from adaptive_assembly_sim.gazebo_target_pose_sync_node import (
    pose_stamped_to_gazebo_request,
)

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import Float64, String


class Fixture(Node):
    """Publish a fixture pose and collect retained synchronization outputs."""

    def __init__(self) -> None:
        """Create fixture publishers and subscribers."""
        super().__init__('gazebo_target_pose_consistency_fixture')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = None
        self.error_mm = None
        self.error_deg = None
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.create_subscription(
            String, '/gazebo_target_sync_status',
            lambda msg: setattr(self, 'status', msg.data), qos,
        )
        self.create_subscription(
            Float64, '/gazebo_target_pose_error_mm',
            lambda msg: setattr(self, 'error_mm', msg.data), qos,
        )
        self.create_subscription(
            Float64, '/gazebo_target_pose_error_deg',
            lambda msg: setattr(self, 'error_deg', msg.data), qos,
        )


def fixture_pose() -> PoseStamped:
    """Return a deterministic, valid world-frame test pose."""
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = 0.42
    pose.pose.position.y = -0.08
    pose.pose.position.z = 0.15
    pose.pose.orientation.z = math.sin(0.25)
    pose.pose.orientation.w = math.cos(0.25)
    return pose


def main() -> int:
    """Validate conversion and the no-Gazebo execution path."""
    pose = fixture_pose()
    request = pose_stamped_to_gazebo_request(pose, 'target_object')
    if request.entity.name != 'target_object' or request.entity.type != 2:
        print('FAIL: adapter did not select the target model entity')
        return 1
    if request.pose != pose.pose:
        print('FAIL: adapter changed the requested pose')
        return 1

    process = subprocess.Popen([
        'ros2', 'run', 'adaptive_assembly_sim',
        'gazebo_target_pose_sync_node', '--ros-args',
        '-p', 'enable_service_calls:=false',
    ])
    rclpy.init()
    node = Fixture()
    try:
        end = time.monotonic() + 8.0
        while (
            (node.status is None or node.error_mm is None or
             node.error_deg is None)
            and time.monotonic() < end
        ):
            pose.header.stamp = node.get_clock().now().to_msg()
            node.publisher.publish(pose)
            rclpy.spin_once(node, timeout_sec=0.2)
        expected = (
            'event=skipped;mode=gazebo_target_sync;'
            'reason=service_calls_disabled'
        )
        if node.status is None or not node.status.startswith(expected):
            print(f'FAIL: unexpected deterministic status: {node.status}')
            return 1
        if node.error_mm is None or node.error_deg is None:
            print('FAIL: pose-error diagnostics were not published')
            return 1
        if not math.isnan(node.error_mm) or not math.isnan(node.error_deg):
            print('FAIL: unconfirmed Gazebo pose errors must be NaN')
            return 1
        print(
            'PASS: adapter preserves the pose and disabled calls skip '
            'deterministically'
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == '__main__':
    sys.exit(main())
