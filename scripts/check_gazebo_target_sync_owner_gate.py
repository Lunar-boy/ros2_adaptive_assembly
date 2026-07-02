#!/usr/bin/env python3
"""Verify target sync skips pose writes while another owner is active."""

import os
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


class Fixture(Node):
    def __init__(self, prefix: str) -> None:
        super().__init__('gazebo_target_sync_owner_gate_fixture')
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.owner_pub = self.create_publisher(String, prefix + '/owner', qos)
        self.pose_pub = self.create_publisher(PoseStamped, prefix + '/pose', 10)
        self.statuses = []
        self.errors = []
        self.create_subscription(
            String, prefix + '/status',
            lambda message: self.statuses.append(message.data), qos,
        )
        self.create_subscription(
            Float64, prefix + '/error',
            lambda message: self.errors.append(message.data), qos,
        )


def spin_until(node, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def main() -> int:
    prefix = '/gazebo_target_sync_owner_validation'
    process = subprocess.Popen([
        'ros2', 'run', 'adaptive_assembly_sim',
        'gazebo_target_pose_sync_node', '--ros-args',
        '-p', f'target_pose_topic:={prefix}/pose',
        '-p', f'control_owner_topic:={prefix}/owner',
        '-p', f'status_topic:={prefix}/status',
        '-p', f'pose_error_mm_topic:={prefix}/error',
        '-p', f'pose_error_deg_topic:={prefix}/error_deg',
        '-p', 'enable_service_calls:=false',
    ], start_new_session=True)
    rclpy.init()
    node = Fixture(prefix)
    try:
        if not spin_until(node, lambda: (
            node.owner_pub.get_subscription_count() > 0
            and node.pose_pub.get_subscription_count() > 0
        )):
            raise RuntimeError('target sync subscriptions were not discovered')
        node.owner_pub.publish(String(data='gripper_attach'))
        # Ownership and pose use separate topics; allow the owner callback to
        # run before publishing the pose under test.
        time.sleep(0.2)
        message = PoseStamped()
        message.header.frame_id = 'world'
        message.pose.orientation.w = 1.0
        node.pose_pub.publish(message)
        expected = (
            'event=skipped;mode=gazebo_target_sync;reason=not_owner;'
            'owner=gripper_attach;entity=target_object;'
            f'source_topic={prefix}/pose;simulated_only=true;'
            'real_hardware=false'
        )
        if not spin_until(node, lambda: (
            expected in node.statuses and bool(node.errors)
        )):
            raise RuntimeError(f'not_owner status missing: {node.statuses}')
        if not node.errors or not all(value != value for value in node.errors):
            raise RuntimeError('skipped ownership event did not publish NaN')
        print('PASS: target sync skips writes owned by gripper_attach')
        return 0
    except RuntimeError as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            process.kill()


if __name__ == '__main__':
    raise SystemExit(main())
