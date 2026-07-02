#!/usr/bin/env python3
"""Verify retained ownership across the Gazebo grasp lifecycle."""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Fixture(Node):
    def __init__(self, prefix: str) -> None:
        super().__init__('gazebo_attach_owner_transition_fixture')
        qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, prefix + '/state', qos)
        self.owners = []
        self.create_subscription(
            String, prefix + '/owner',
            lambda message: self.owners.append(message.data), qos,
        )


def spin_until(node, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def main() -> int:
    prefix = '/gazebo_attach_owner_validation'
    process = subprocess.Popen([
        'ros2', 'run', 'adaptive_assembly_manipulation',
        'gazebo_attach_detach_node', '--ros-args',
        '-p', f'object_grasp_state_topic:={prefix}/state',
        '-p', f'object_grasp_attached_topic:={prefix}/logical_attached',
        '-p', f'control_owner_topic:={prefix}/owner',
        '-p', f'status_topic:={prefix}/status',
        '-p', f'gazebo_object_attached_topic:={prefix}/attached',
        '-p', f'pose_error_mm_topic:={prefix}/error',
        '-p', 'enable_service_calls:=false',
    ], start_new_session=True)
    rclpy.init()
    node = Fixture(prefix)
    try:
        if not spin_until(node, lambda: 'target_sync' in node.owners):
            raise RuntimeError(f'startup target_sync missing: {node.owners}')
        node.state_pub.publish(String(data=(
            'event=attached;object=target_object;trigger=grasp_confirmed'
        )))
        if not spin_until(node, lambda: 'gripper_attach' in node.owners):
            raise RuntimeError(f'gripper_attach missing: {node.owners}')
        node.state_pub.publish(String(data=(
            'event=detached;object=target_object;'
            'trigger=execution_success'
        )))
        if not spin_until(node, lambda: 'released' in node.owners):
            raise RuntimeError(f'released missing: {node.owners}')
        expected = ['target_sync', 'gripper_attach', 'released']
        positions = [node.owners.index(value) for value in expected]
        if positions != sorted(positions):
            raise RuntimeError(f'owner transitions out of order: {node.owners}')
        node.state_pub.publish(String(data=(
            'event=attached;object=target_object;trigger=grasp_confirmed'
        )))
        if not spin_until(node, lambda: node.owners[-1:] == ['gripper_attach']):
            raise RuntimeError('second attach did not reclaim ownership')
        node.state_pub.publish(String(data=(
            'event=detached;object=target_object;'
            'trigger=execution_failure'
        )))
        if not spin_until(node, lambda: node.owners[-1:] == ['released']):
            raise RuntimeError('execution failure did not release ownership')
        print('PASS: attach ownership transitions are deterministic')
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
