#!/usr/bin/env python3
"""Exercise aborted close classification over real ROS topics and actions."""

import threading
import time

from adaptive_assembly_manipulation.gripper_action_bridge_node import (
    GripperActionBridgeNode,
)
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class AbortingGripperServer(Node):
    """Accept one goal and abort with the structured goal-tolerance code."""

    def __init__(self, action_name):
        """Start a synthetic action server at ``action_name``."""
        super().__init__('contact_close_aborting_server')
        self.goal_executed = False
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
        )

    def _execute(self, goal_handle):
        self.goal_executed = True
        time.sleep(0.35)
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = result.GOAL_TOLERANCE_VIOLATED
        result.error_string = 'synthetic contact-limited final position'
        return result


class ContactCloseProbe(Node):
    """Publish normalized contact evidence and collect bridge status."""

    def __init__(self, suffix, bilateral):
        """Create unique topics and periodic contact publication."""
        super().__init__(f'contact_close_probe_{suffix}')
        self._bilateral = bilateral
        self.statuses = []
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_publisher = self.create_publisher(
            String, f'/test_gripper_command_{suffix}', qos
        )
        self.contact_publisher = self.create_publisher(
            String, f'/test_gripper_contact_{suffix}', qos
        )
        self.create_subscription(
            String,
            f'/test_gripper_status_{suffix}',
            lambda message: self.statuses.append(message.data),
            qos,
        )
        self.create_timer(0.04, self._publish_contact)

    def publish_command(self, command):
        """Publish one gripper command with a deterministic ID."""
        self.command_publisher.publish(String(
            data=f'event=command;command={command};command_id=1'
        ))

    def _publish_contact(self):
        now_ns = self.get_clock().now().nanoseconds
        right_target = str(self._bilateral).lower()
        right_entities = (
            'target_object::link::collision' if self._bilateral else ''
        )
        self.contact_publisher.publish(String(data=(
            'event=success;mode=gazebo_grasp_contact_status;'
            'target_object_name=target_object;'
            'left_message_received=true;right_message_received=true;'
            'left_target_contact=true;'
            f'right_target_contact={right_target};'
            'left_wrong_object_contact=false;'
            'right_wrong_object_contact=false;'
            f'left_receipt_stamp_ns={now_ns};'
            f'right_receipt_stamp_ns={now_ns};'
            'left_contact_entities=target_object::link::collision;'
            f'right_contact_entities={right_entities}'
        )))


def run_case(suffix, bilateral, expected_result, command='close'):
    """Run one bounded action/contact integration case."""
    action_name = f'/test_gripper_action_{suffix}'
    args = [
        '--ros-args',
        '-p', f'controller_action_name:={action_name}',
        '-p', f'command_topic:=/test_gripper_command_{suffix}',
        '-p', f'status_topic:=/test_gripper_status_{suffix}',
        '-p', f'contact_status_topic:=/test_gripper_contact_{suffix}',
        '-p', 'allow_contact_limited_close:=true',
        '-p', 'expected_target_object:=target_object',
        '-p', 'result_timeout_sec:=2.0',
        '-p', 'contact_wait_timeout_sec:=0.4',
        '-p', 'contact_freshness_timeout_sec:=0.2',
        '-p', 'contact_settle_duration_sec:=0.2',
    ]
    rclpy.init(args=args)
    server = AbortingGripperServer(action_name)
    bridge = GripperActionBridgeNode()
    probe = ContactCloseProbe(suffix, bilateral)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (server, bridge, probe):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(0.3)
        probe.publish_command(command)
        deadline = time.monotonic() + 4.0
        expected = f'result={expected_result}'
        while time.monotonic() < deadline:
            if any(expected in status for status in probe.statuses):
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(
                f'{suffix} did not produce {expected_result}: {probe.statuses}'
            )
        if not server.goal_executed:
            raise RuntimeError(f'{suffix} action goal was not executed')
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        for node in (probe, bridge, server):
            node.destroy_node()
        rclpy.shutdown()


def main():
    """Validate bilateral success and unilateral failure regressions."""
    run_case('bilateral', True, 'contact_limited_success')
    run_case('unilateral', False, 'unilateral_contact')
    run_case('open_strict', True, 'action_aborted', command='open')
    print(
        'PASS contact-aware gripper close integration: '
        'bilateral abort accepted, unilateral abort rejected, and open strict'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
