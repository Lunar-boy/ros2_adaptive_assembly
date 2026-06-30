#!/usr/bin/env python3
"""Exercise the ros2_control bridge with a deliberately missing controller."""

import signal
import subprocess
import sys
import time
from typing import Dict, Optional

from moveit_msgs.msg import RobotTrajectory

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float64, String

from trajectory_msgs.msg import JointTrajectoryPoint


TIMEOUT_SEC = 10.0
PREFIX = '/ros2_control_unavailable_validation'


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited status message."""
    return dict(
        item.split('=', 1) for item in status.split(';') if '=' in item
    )


def make_trajectory(offset: float) -> RobotTrajectory:
    """Create a minimal valid Panda trajectory for interface validation."""
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = [
        f'panda_joint{index}' for index in range(1, 8)
    ]
    point = JointTrajectoryPoint()
    point.positions = [offset] * 7
    point.time_from_start.sec = 1
    trajectory.joint_trajectory.points.append(point)
    return trajectory


class UnavailablePathChecker(Node):
    """Publish two trajectories and collect the retained skipped result."""

    def __init__(self) -> None:
        """Create isolated validation interfaces."""
        super().__init__('ros2_control_unavailable_path_checker')
        self.status: Optional[str] = None
        self.success: Optional[bool] = None
        self.duration_ms: Optional[float] = None
        self.pre_grasp_publisher = self.create_publisher(
            RobotTrajectory, f'{PREFIX}/pre_grasp_trajectory', 10
        )
        self.assembly_publisher = self.create_publisher(
            RobotTrajectory, f'{PREFIX}/assembly_trajectory', 10
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, f'{PREFIX}/status', self._status_callback, qos
        )
        self.create_subscription(
            Bool, f'{PREFIX}/success', self._success_callback, qos
        )
        self.create_subscription(
            Float64, f'{PREFIX}/duration_ms', self._duration_callback, qos
        )

    def _status_callback(self, message: String) -> None:
        self.status = message.data

    def _success_callback(self, message: Bool) -> None:
        self.success = message.data

    def _duration_callback(self, message: Float64) -> None:
        self.duration_ms = message.data

    def bridge_discovered(self) -> bool:
        """Return whether both bridge input subscriptions are discovered."""
        return (
            self.pre_grasp_publisher.get_subscription_count() > 0
            and self.assembly_publisher.get_subscription_count() > 0
        )

    def complete(self) -> bool:
        """Return whether all terminal outputs arrived."""
        return (
            self.status is not None
            and self.success is not None
            and self.duration_ms is not None
        )


def start_bridge() -> subprocess.Popen:
    """Start an isolated bridge configured with a nonexistent action name."""
    command = [
        'ros2',
        'run',
        'adaptive_assembly_execution',
        'ros2_control_sequence_executor_node',
        '--ros-args',
        '-r',
        '__node:=ros2_control_unavailable_validation_executor',
        '-p',
        f'pre_grasp_trajectory_topic:={PREFIX}/pre_grasp_trajectory',
        '-p',
        f'assembly_trajectory_topic:={PREFIX}/assembly_trajectory',
        '-p',
        'controller_action_name:=/missing_validation_controller/'
        'follow_joint_trajectory',
        '-p',
        f'status_topic:={PREFIX}/status',
        '-p',
        f'success_topic:={PREFIX}/success',
        '-p',
        f'duration_topic:={PREFIX}/duration_ms',
        '-p',
        f'stage_status_topic:={PREFIX}/stage_status',
        '-p',
        'wait_for_controller_sec:=0.25',
    ]
    return subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def main() -> int:
    """Verify missing-controller handling without Gazebo or MoveIt."""
    rclpy.init()
    node = UnavailablePathChecker()
    process = start_bridge()
    deadline = time.monotonic() + TIMEOUT_SEC
    last_publish = 0.0

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is not None:
                print(
                    'FAIL: ros2_control executor exited unexpectedly with '
                    f'code {process.returncode}'
                )
                return 1
            now = time.monotonic()
            if node.bridge_discovered() and now - last_publish >= 0.2:
                node.pre_grasp_publisher.publish(make_trajectory(0.0))
                node.assembly_publisher.publish(make_trajectory(0.1))
                last_publish = now
            if node.complete():
                fields = parse_status(node.status)
                expected = {
                    'event': 'skipped',
                    'mode': 'ros2_control',
                    'reason': 'controller_unavailable',
                    'execution': 'false',
                    'simulated_execution_only': 'true',
                    'real_hardware': 'false',
                }
                for key, value in expected.items():
                    if fields.get(key) != value:
                        print(
                            f'FAIL: expected {key}={value}: {node.status}'
                        )
                        return 1
                if node.success is not False:
                    print('FAIL: unavailable-controller success was not false')
                    return 1
                if node.duration_ms < 0.0:
                    print('FAIL: unavailable-controller duration was negative')
                    return 1
                print('PASS: missing controller produced a skipped result')
                print(f'      status: {node.status}')
                return 0

        print('FAIL: timed out waiting for unavailable-controller result')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3.0)


if __name__ == '__main__':
    sys.exit(main())
