#!/usr/bin/env python3
"""Fixture test support for Gazebo attach/detach validation scripts."""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class Fixture(Node):
    """Drive grasp events and collect retained attachment outputs."""

    def __init__(self, prefix: str) -> None:
        super().__init__('gazebo_attach_detach_fixture')
        qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, prefix + '/state', qos)
        self.statuses = []
        self.attached = []
        self.create_subscription(
            String, prefix + '/status',
            lambda message: self.statuses.append(message.data), qos,
        )
        self.create_subscription(
            Bool, prefix + '/attached',
            lambda message: self.attached.append(message.data), qos,
        )


def spin_until(node, predicate, timeout=5.0):
    """Spin until predicate succeeds or a bounded timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def run_check(mode: str) -> int:
    """Run the offline success case or missing-service failure case."""
    prefix = '/gazebo_attach_detach_' + mode + '_validation'
    enable_calls = mode == 'failure'
    command = [
        'ros2', 'run', 'adaptive_assembly_manipulation',
        'gazebo_attach_detach_node', '--ros-args',
        '-r', '__node:=gazebo_attach_detach_' + mode + '_validation',
        '-p', f'object_grasp_state_topic:={prefix}/state',
        '-p', f'object_grasp_attached_topic:={prefix}/logical_attached',
        '-p', f'status_topic:={prefix}/status',
        '-p', f'gazebo_object_attached_topic:={prefix}/attached',
        '-p', f'pose_error_mm_topic:={prefix}/error',
        '-p', f'enable_service_calls:={str(enable_calls).lower()}',
        '-p', 'attach_update_period_sec:=0.05',
    ]
    process = subprocess.Popen(command, start_new_session=True)
    tf_process = subprocess.Popen([
        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        '--x', '0.4', '--y', '0.0', '--z', '0.5',
        '--frame-id', 'world', '--child-frame-id', 'panda_hand',
    ], start_new_session=True, stdout=subprocess.DEVNULL,
       stderr=subprocess.DEVNULL)
    rclpy.init()
    node = Fixture(prefix)
    try:
        if not spin_until(node, lambda: len(node.statuses) > 0):
            raise RuntimeError('node did not publish retained startup status')
        time.sleep(0.5)
        node.publisher.publish(String(data=(
            'event=attached;object=target_object;parent=panda_hand'
        )))
        expected_reason = (
            'gazebo_service_unavailable' if enable_calls
            else 'service_calls_disabled'
        )
        if not spin_until(node, lambda: (
            True in node.attached and any(
                'event=attached' in value for value in node.statuses
            ) and any(
                f'reason={expected_reason}' in value
                for value in node.statuses
            )
        )):
            raise RuntimeError(
                'attach event, attached=true, or deterministic status missing'
            )
        node.publisher.publish(String(data=(
            'event=detached;object=target_object;parent=world'
        )))
        if not spin_until(node, lambda: (
            False in node.attached and any(
                'event=detached' in value for value in node.statuses
            )
        )):
            raise RuntimeError('detach event or attached=false missing')
        print(f'PASS: Gazebo attach/detach {mode} fixture')
        return 0
    except RuntimeError as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for child in (process, tf_process):
            try:
                os.killpg(child.pid, signal.SIGINT)
                child.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                child.kill()
