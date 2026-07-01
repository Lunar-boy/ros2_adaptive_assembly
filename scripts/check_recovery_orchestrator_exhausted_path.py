#!/usr/bin/env python3
"""Verify duplicate recovery actions cannot exceed the retry bound."""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


PREFIX = '/pr37_exhausted'


class Fixture(Node):
    def __init__(self) -> None:
        super().__init__('recovery_orchestrator_exhausted_fixture')
        self.target_calls = 0
        self.statuses = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, f'{PREFIX}/action', qos)
        self.create_subscription(String, f'{PREFIX}/status',
                                 self.statuses.append, qos)
        self.create_service(Trigger, f'{PREFIX}/target', self._trigger)

    def _trigger(self, _request, response):
        self.target_calls += 1
        response.success = True
        return response


def event(message):
    return dict(item.split('=', 1) for item in message.data.split(';')
                if '=' in item).get('event')


def main() -> int:
    rclpy.init()
    node = Fixture()
    process = subprocess.Popen([
        'ros2', 'run', 'adaptive_assembly_recovery',
        'recovery_orchestrator_node',
        '--ros-args', '-p', f'recovery_action_topic:={PREFIX}/action',
        '-p', f'orchestration_status_topic:={PREFIX}/status',
        '-p', f'retry_requested_topic:={PREFIX}/retry',
        '-p', f'publish_target_pose_service:={PREFIX}/target',
        '-p', 'max_retry_attempts:=1',
    ], start_new_session=True)
    try:
        message = String()
        message.data = ('event=recovery_action;'
                        'action=discard_trajectories_and_replan')
        deadline = time.monotonic() + 15.0
        first_complete = False
        while rclpy.ok() and time.monotonic() < deadline:
            node.publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
            events = [event(status) for status in node.statuses]
            if 'retry_requested' in events:
                first_complete = True
            if first_complete and 'retry_exhausted' in events:
                settle = time.monotonic() + 1.0
                while time.monotonic() < settle:
                    node.publisher.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.05)
                if node.target_calls == 1:
                    print(
                        'PASS: duplicate actions were exhausted after one retry'
                    )
                    return 0
                print(f'FAIL: target trigger called {node.target_calls} times')
                return 1
        print('FAIL: timed out waiting for retry and exhausted statuses')
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
