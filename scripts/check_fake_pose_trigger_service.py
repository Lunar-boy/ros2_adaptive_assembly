#!/usr/bin/env python3
"""Validate the fake-perception one-shot Trigger service in isolation."""

import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


SERVICE = '/pr37_test/publish_target_pose_once'
TOPIC = '/target_pose'


class Checker(Node):
    def __init__(self) -> None:
        super().__init__('fake_pose_trigger_checker')
        self.poses = 0
        self.create_subscription(PoseStamped, TOPIC, self._pose, 10)
        self.client = self.create_client(Trigger, SERVICE)

    def _pose(self, _message: PoseStamped) -> None:
        self.poses += 1


def main() -> int:
    process = subprocess.Popen([
        'ros2', 'run', 'adaptive_assembly_perception', 'fake_object_pose_node',
        '--ros-args', '-p', 'publish_immediately:=false',
        '-p', 'publish_period_sec:=60.0', '-p', 'random_seed:=37',
        '-p', f'publish_target_pose_service:={SERVICE}',
    ], start_new_session=True)
    rclpy.init()
    node = Checker()
    try:
        if not node.client.wait_for_service(timeout_sec=10.0):
            print('FAIL: fake pose Trigger service was not available')
            return 1
        future = node.client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if future.done() and node.poses == 1:
                if future.result().success:
                    print(
                        'PASS: Trigger published exactly one fresh target pose'
                    )
                    return 0
                break
        print(f'FAIL: service response/pose missing; poses={node.poses}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


if __name__ == '__main__':
    sys.exit(main())
