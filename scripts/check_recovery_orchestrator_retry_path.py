#!/usr/bin/env python3
"""Validate one isolated reset-and-retry orchestration sequence."""

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


PREFIX = '/pr37_retry'


class Fixture(Node):
    def __init__(self) -> None:
        super().__init__('recovery_orchestrator_retry_fixture')
        self.calls = []
        self.statuses = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, f'{PREFIX}/action', qos)
        self.create_subscription(String, f'{PREFIX}/status',
                                 self.statuses.append, qos)
        for key in ('dynamic', 'static', 'reapply', 'target'):
            self.create_service(Trigger, f'{PREFIX}/{key}',
                                lambda request, response, name=key:
                                self._service(name, request, response))

    def _service(self, name, _request, response):
        self.calls.append(name)
        response.success = True
        response.message = name
        return response


def fields(message):
    return dict(item.split('=', 1) for item in message.data.split(';')
                if '=' in item)


def main() -> int:
    rclpy.init()
    node = Fixture()
    process = subprocess.Popen([
        'ros2', 'run', 'adaptive_assembly_recovery',
        'recovery_orchestrator_node',
        '--ros-args', '-p', f'recovery_action_topic:={PREFIX}/action',
        '-p', f'orchestration_status_topic:={PREFIX}/status',
        '-p', f'retry_requested_topic:={PREFIX}/retry',
        '-p', f'clear_dynamic_scene_service:={PREFIX}/dynamic',
        '-p', f'clear_static_scene_service:={PREFIX}/static',
        '-p', f'reapply_static_scene_service:={PREFIX}/reapply',
        '-p', f'publish_target_pose_service:={PREFIX}/target',
    ], start_new_session=True)
    try:
        deadline = time.monotonic() + 15.0
        message = String()
        message.data = ('event=recovery_action;action=reset_scene_and_retry;'
                        'attempt=1;real_execution=false')
        while rclpy.ok() and time.monotonic() < deadline:
            node.publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
            matches = [fields(status) for status in node.statuses
                       if fields(status).get('event') == 'retry_requested']
            if matches:
                status = matches[-1]
                expected = ['dynamic', 'static', 'reapply', 'target']
                if (
                    node.calls == expected
                    and status.get('services_ok') == 'true'
                    and status.get('target_pose_triggered') == 'true'
                ):
                    print(
                        'PASS: reset services ran in order and target trigger '
                        'ran once'
                    )
                    return 0
                print(f'FAIL: calls={node.calls}, status={status}')
                return 1
        print('FAIL: timed out waiting for retry_requested status')
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
