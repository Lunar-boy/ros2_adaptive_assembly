#!/usr/bin/env python3
"""Exercise close/open bridge commands without a controller or Gazebo."""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class DryRunProbe(Node):
    def __init__(self) -> None:
        super().__init__('gripper_action_bridge_dry_run_probe')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.statuses = []
        self.closed_values = []
        self.create_subscription(
            String, '/physical_gripper_command_status',
            lambda message: self.statuses.append(message.data), qos,
        )
        self.create_subscription(
            Bool, '/physical_gripper_closed',
            lambda message: self.closed_values.append(message.data), qos,
        )
        self.publisher = self.create_publisher(String, '/gripper_command', qos)

    def publish_command(self, command: str) -> None:
        message = String()
        message.data = (
            f'event=command;command={command};gripper=panda_hand;'
            'simulated=true;real_hardware=false'
        )
        self.publisher.publish(message)


def wait_for(node: DryRunProbe, predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main() -> int:
    process = subprocess.Popen(
        [
            'ros2', 'run', 'adaptive_assembly_manipulation',
            'gripper_action_bridge_node', '--ros-args',
            '-p', 'send_goals:=false',
        ],
        start_new_session=True,
    )
    rclpy.init()
    node = DryRunProbe()
    try:
        if not wait_for(
            node,
            lambda: any('event=ready;' in item for item in node.statuses),
            10.0,
        ):
            print('FAIL: bridge did not publish retained ready status')
            return 1

        node.publish_command('close')
        if not wait_for(
            node,
            lambda: (
                any(
                    'event=success;command=close;send_goals=false' in item
                    for item in node.statuses
                ) and node.closed_values and node.closed_values[-1] is True
            ),
            5.0,
        ):
            print('FAIL: close dry-run did not report success and closed=true')
            return 1

        node.publish_command('open')
        if not wait_for(
            node,
            lambda: (
                any(
                    'event=success;command=open;send_goals=false' in item
                    for item in node.statuses
                ) and node.closed_values and node.closed_values[-1] is False
            ),
            5.0,
        ):
            print('FAIL: open dry-run did not report success and closed=false')
            return 1
        print('PASS: bridge dry-run close/open statuses and closed state are valid')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)


if __name__ == '__main__':
    sys.exit(main())
