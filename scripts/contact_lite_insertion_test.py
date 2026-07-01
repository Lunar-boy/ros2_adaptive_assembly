#!/usr/bin/env python3
"""Synthetic pose checks for the contact-lite insertion evaluator."""

import math
import sys
import time
from typing import Dict, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from adaptive_assembly_benchmark.contact_lite_insertion_evaluator_node import (
    ContactLiteInsertionEvaluatorNode,
)


TIMEOUT_SEC = 5.0


def _target_topic(mode: str) -> str:
    return f'/synthetic_insertion_{mode}_target_pose'


def _achieved_topic(mode: str) -> str:
    return f'/synthetic_insertion_{mode}_achieved_pose'


def _parse_status(status: str) -> Dict[str, str]:
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' not in item:
            continue
        key, value = item.split('=', 1)
        fields[key] = value
    return fields


def _pose(x: float, yaw_deg: float = 0.0) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = 'panda_link0'
    message.header.stamp.sec = 1
    message.pose.position.x = x
    message.pose.position.y = 0.0
    message.pose.position.z = 0.2
    yaw_rad = math.radians(yaw_deg)
    message.pose.orientation.z = math.sin(yaw_rad / 2.0)
    message.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return message


class SyntheticInsertionHarness(Node):
    """Publish synthetic inputs and capture evaluator outputs."""

    def __init__(self, mode: str) -> None:
        """Create publishers and subscriptions for one check mode."""
        super().__init__(f'contact_lite_insertion_{mode}_checker')
        self.mode = mode
        self.status: Optional[str] = None
        self.success: Optional[bool] = None
        self.position_error_mm: Optional[float] = None
        self.orientation_error_deg: Optional[float] = None
        self._target_publisher = self.create_publisher(
            PoseStamped, _target_topic(mode), 10
        )
        self._achieved_publisher = self.create_publisher(
            PoseStamped, _achieved_topic(mode), 10
        )
        self.create_subscription(
            String,
            f'/synthetic_insertion_{mode}_status',
            self._status_callback,
            10,
        )
        self.create_subscription(
            Bool,
            f'/synthetic_insertion_{mode}_success',
            self._success_callback,
            10,
        )
        self.create_subscription(
            Float64,
            f'/synthetic_insertion_{mode}_error_mm',
            self._position_error_callback,
            10,
        )
        self.create_subscription(
            Float64,
            f'/synthetic_insertion_{mode}_error_deg',
            self._orientation_error_callback,
            10,
        )
        self._timer = self.create_timer(0.1, self._publish_inputs)

    def _publish_inputs(self) -> None:
        self._target_publisher.publish(_pose(0.0))
        if self.mode == 'success':
            self._achieved_publisher.publish(_pose(0.002, yaw_deg=2.0))
        else:
            self._achieved_publisher.publish(_pose(0.010, yaw_deg=8.0))

    def _status_callback(self, message: String) -> None:
        self.status = message.data

    def _success_callback(self, message: Bool) -> None:
        self.success = message.data

    def _position_error_callback(self, message: Float64) -> None:
        self.position_error_mm = message.data

    def _orientation_error_callback(self, message: Float64) -> None:
        self.orientation_error_deg = message.data


def _result_ready(harness: SyntheticInsertionHarness) -> bool:
    return (
        harness.status is not None
        and harness.success is not None
        and harness.position_error_mm is not None
        and harness.orientation_error_deg is not None
    )


def _validate(mode: str, harness: SyntheticInsertionHarness) -> int:
    if not _result_ready(harness):
        print('FAIL: evaluator did not publish complete insertion outputs')
        return 1

    fields = _parse_status(harness.status or '')
    expected_success = mode == 'success'
    checks = {
        'event': fields.get('event') == 'insertion_evaluated',
        'mode': fields.get('mode') == 'contact_lite_insertion',
        'success': harness.success is expected_success,
        'status_success': (
            fields.get('success') == str(expected_success).lower()
        ),
        'position_tolerance': (
            fields.get('position_tolerance_mm') == '5.000000'
        ),
        'orientation_tolerance': (
            fields.get('orientation_tolerance_deg') == '5.000000'
        ),
        'execution_required': fields.get('execution_required') == 'false',
        'achieved_pose_source': fields.get(
            'achieved_pose_source'
        ) == 'synthetic_pose',
        'real_hardware': fields.get('real_hardware') == 'false',
    }
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        print('FAIL: insertion status did not match expected fields')
        print(f'      failed checks: {", ".join(failed)}')
        print(f'      status: {harness.status}')
        return 1

    if expected_success:
        metric_ok = (
            1.5 <= (harness.position_error_mm or 0.0) <= 2.5
            and 1.5 <= (harness.orientation_error_deg or 0.0) <= 2.5
        )
    else:
        metric_ok = (
            (harness.position_error_mm or 0.0) > 5.0
            and (harness.orientation_error_deg or 0.0) > 5.0
        )
    if not metric_ok:
        print(
            'FAIL: insertion metrics were outside expected deterministic range'
        )
        print(f'      position_error_mm={harness.position_error_mm}')
        print(f'      orientation_error_deg={harness.orientation_error_deg}')
        return 1

    print(f'PASS: contact-lite insertion {mode} path is deterministic')
    print(f'      status: {harness.status}')
    return 0


def run_check(mode: str) -> int:
    """Run a synthetic insertion success or failure check."""
    if mode not in {'success', 'failure'}:
        print(f'FAIL: unsupported check mode: {mode}')
        return 1

    rclpy.init(args=[
        '--ros-args',
        '-p', f'target_pose_topic:={_target_topic(mode)}',
        '-p', f'achieved_pose_topic:={_achieved_topic(mode)}',
        '-p', 'position_tolerance_mm:=5.0',
        '-p', 'orientation_tolerance_deg:=5.0',
        '-p', 'require_execution_success:=false',
        '-p', 'achieved_pose_source:=synthetic_pose',
        '-p', f'status_topic:=/synthetic_insertion_{mode}_status',
        '-p', f'success_topic:=/synthetic_insertion_{mode}_success',
        '-p', f'position_error_topic:=/synthetic_insertion_{mode}_error_mm',
        '-p',
        f'orientation_error_topic:=/synthetic_insertion_{mode}_error_deg',
    ])
    evaluator = ContactLiteInsertionEvaluatorNode()
    harness = SyntheticInsertionHarness(mode)
    executor = SingleThreadedExecutor()
    executor.add_node(evaluator)
    executor.add_node(harness)
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            if _result_ready(harness):
                return _validate(mode, harness)

        print('FAIL: timed out waiting for contact-lite insertion output')
        return 1
    finally:
        executor.remove_node(harness)
        executor.remove_node(evaluator)
        harness.destroy_node()
        evaluator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(run_check(sys.argv[1] if len(sys.argv) > 1 else 'success'))
