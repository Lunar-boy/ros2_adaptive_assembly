#!/usr/bin/env python3
"""Shared synthetic status publisher for episode supervisor checks."""

import os
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String


def parse_status(value):
    """Parse status fields used by the checks."""
    result = {}
    for fragment in value.split(';'):
        if '=' in fragment:
            key, item = fragment.split('=', 1)
            if key.strip():
                result[key.strip()] = item.strip()
    return result


class EpisodeCheckNode(Node):
    """Publish synthetic upstream evidence and collect one terminal result."""

    def __init__(self, suffix):
        super().__init__('assembly_episode_check_' + suffix)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publishers_by_name = {
            'planning': self.create_publisher(
                String, '/assembly_sequence_planning_status', qos
            ),
            'execution_status': self.create_publisher(
                String, '/assembly_ros2_control_execution_status', qos
            ),
            'execution_success': self.create_publisher(
                Bool, '/assembly_ros2_control_execution_success', qos
            ),
            'execution_duration': self.create_publisher(
                Float64, '/assembly_ros2_control_execution_duration_ms', qos
            ),
            'execution_stage': self.create_publisher(
                String, '/assembly_ros2_control_execution_stage_status', qos
            ),
            'logical_status': self.create_publisher(
                String, '/logical_grasp_lifecycle_status', qos
            ),
            'logical_attached': self.create_publisher(
                Bool, '/object_grasp_attached', qos
            ),
            'gazebo_status': self.create_publisher(
                String, '/gazebo_attach_detach_status', qos
            ),
            'gazebo_error': self.create_publisher(
                Float64, '/gazebo_attach_pose_error_mm', qos
            ),
            'insertion_status': self.create_publisher(
                String, '/assembly_insertion_status', qos
            ),
            'insertion_success': self.create_publisher(
                Bool, '/assembly_insertion_success', qos
            ),
            'insertion_mm': self.create_publisher(
                Float64, '/assembly_insertion_error_mm', qos
            ),
            'insertion_deg': self.create_publisher(
                Float64, '/assembly_insertion_error_deg', qos
            ),
        }
        self.result = None
        self.create_subscription(
            String, '/assembly_episode_test_status_' + suffix,
            self._result_callback, qos
        )

    def _result_callback(self, message):
        self.result = message.data

    def publish(self, name, value):
        publisher = self.publishers_by_name[name]
        if isinstance(value, bool):
            message = Bool()
        elif isinstance(value, float):
            message = Float64()
        else:
            message = String()
        message.data = value
        publisher.publish(message)


def _spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def _wait_for_discovery(node, timeout=5.0):
    deadline = time.monotonic() + timeout
    required = ('planning', 'execution_status', 'execution_stage')
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if all(
            node.publishers_by_name[name].get_subscription_count() > 0
            for name in required
        ):
            return True
    return False


def _start_supervisor(suffix, timeout_sec):
    output = '/assembly_episode_test_status_' + suffix
    command = [
        'ros2', 'run', 'adaptive_assembly_episode',
        'assembly_episode_supervisor_node', '--ros-args',
        '-r', '__node:=assembly_episode_supervisor_test_' + suffix,
        '-p', 'status_topic:=' + output,
        '-p', 'success_topic:=/assembly_episode_test_success_' + suffix,
        '-p', 'duration_topic:=/assembly_episode_test_duration_' + suffix,
        '-p', 'failure_reason_topic:=/assembly_episode_test_reason_' + suffix,
        '-p', 'stage_status_topic:=/assembly_episode_test_stage_' + suffix,
        '-p', f'episode_timeout_sec:={timeout_sec}',
    ]
    return subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def _publish_success(node):
    node.publish('planning', 'event=success;mode=assembly_sequence')
    node.publish(
        'execution_stage', 'event=success;mode=ros2_control;stage=pre_grasp'
    )
    node.publish('logical_status', 'event=attached;attached=true')
    node.publish('logical_attached', True)
    node.publish('gazebo_status', 'event=attached;simulated_only=true')
    node.publish('gazebo_error', 0.6)
    _spin_for(node, 0.2)
    node.publish(
        'execution_stage', 'event=success;mode=ros2_control;stage=assembly'
    )
    node.publish('execution_success', True)
    node.publish('execution_duration', 250.0)
    node.publish('logical_status', 'event=released;attached=false')
    node.publish('logical_attached', False)
    node.publish('insertion_status', 'event=success;mode=contact_lite')
    node.publish('insertion_mm', 1.2)
    node.publish('insertion_deg', 0.8)
    node.publish('insertion_success', True)
    # Publish terminal execution last so all supporting evidence is present.
    node.publish('execution_status', 'event=success;mode=ros2_control')


def _publish_failure(node):
    node.publish('planning', 'event=success;mode=assembly_sequence')
    node.publish(
        'execution_status',
        'event=failure;mode=ros2_control;stage=assembly;reason=test_failure',
    )


def run_check(mode):
    """Run one isolated success, failure, or timeout check."""
    suffix = mode + '_' + uuid.uuid4().hex[:8]
    timeout_sec = 0.8 if mode == 'timeout' else 8.0
    process = _start_supervisor(suffix, timeout_sec)
    rclpy.init()
    node = EpisodeCheckNode(suffix)
    try:
        if not _wait_for_discovery(node):
            print('FAIL: supervisor subscriptions were not discovered')
            return 1
        if mode == 'success':
            _publish_success(node)
        elif mode == 'failure':
            _publish_failure(node)

        deadline = time.monotonic() + 5.0
        while node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.result is None:
            print(f'FAIL: no terminal {mode} status received')
            return 1

        fields = parse_status(node.result)
        expected_event = 'timeout' if mode == 'timeout' else mode
        if fields.get('event') != expected_event:
            print(
                f"FAIL: expected event={expected_event}, got '{node.result}'"
            )
            return 1
        if fields.get('mode') != 'assembly_episode':
            print(f"FAIL: invalid mode in '{node.result}'")
            return 1
        if mode == 'success' and fields.get('episode_success') != 'true':
            print(f"FAIL: success result is invalid: '{node.result}'")
            return 1
        expected_reason = {
            'failure': 'execution_failed',
            'timeout': 'episode_timeout',
        }.get(mode)
        if expected_reason and fields.get('failure_reason') != expected_reason:
            print(f"FAIL: invalid failure reason in '{node.result}'")
            return 1
        print(f'PASS: assembly episode supervisor {mode} path')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)


if __name__ == '__main__':
    if len(sys.argv) != 2 or sys.argv[1] not in {'success', 'failure', 'timeout'}:
        print('usage: assembly_episode_supervisor_test.py MODE')
        raise SystemExit(2)
    raise SystemExit(run_check(sys.argv[1]))
