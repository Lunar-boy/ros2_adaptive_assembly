"""Shared lightweight ros2_control execution-path validation helper."""

import signal
import subprocess
import time
from typing import Dict, Optional

from moveit_msgs.msg import RobotTrajectory

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float64, String

from trajectory_msgs.msg import JointTrajectoryPoint


def _fields(status: str) -> Dict[str, str]:
    return dict(
        item.split('=', 1) for item in status.split(';') if '=' in item
    )


def _trajectory(offset: float) -> RobotTrajectory:
    message = RobotTrajectory()
    message.joint_trajectory.joint_names = [
        f'panda_joint{index}' for index in range(1, 8)
    ]
    point = JointTrajectoryPoint()
    point.positions = [offset] * 7
    point.time_from_start.sec = 1
    message.joint_trajectory.points.append(point)
    return message


class Checker(Node):
    """Publish trajectories and collect retained terminal outputs."""

    def __init__(self, prefix: str) -> None:
        """Create isolated topics for one validation mode."""
        super().__init__('ros2_control_execution_path_checker')
        self.status: Optional[str] = None
        self.success: Optional[bool] = None
        self.duration: Optional[float] = None
        self.pre = self.create_publisher(
            RobotTrajectory, f'{prefix}/pre_grasp', 10
        )
        self.assembly = self.create_publisher(
            RobotTrajectory, f'{prefix}/assembly', 10
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, f'{prefix}/status', lambda msg: self._status(msg), qos
        )
        self.create_subscription(
            Bool, f'{prefix}/success', lambda msg: self._success(msg), qos
        )
        self.create_subscription(
            Float64, f'{prefix}/duration', lambda msg: self._duration(msg), qos
        )

    def _status(self, message: String) -> None:
        self.status = message.data

    def _success(self, message: Bool) -> None:
        self.success = message.data

    def _duration(self, message: Float64) -> None:
        self.duration = message.data


def _start(command):
    return subprocess.Popen(command)


def run_check(mode: str) -> int:
    """Run either the success or timeout action-level check."""
    prefix = f'/ros2_control_{mode}_validation'
    action = f'{prefix}/follow_joint_trajectory'
    server = _start([
        'ros2', 'run', 'adaptive_assembly_execution',
        'simulated_follow_joint_trajectory_server_node', '--ros-args',
        '-r', f'__node:=simulated_trajectory_{mode}_validation',
        '-p', f'action_name:={action}', '-p', f'result_mode:={mode}',
        '-p', 'result_delay_sec:=0.05',
    ])
    bridge = _start([
        'ros2', 'run', 'adaptive_assembly_execution',
        'ros2_control_sequence_executor_node', '--ros-args',
        '-r', f'__node:=ros2_control_{mode}_validation_executor',
        '-p', f'pre_grasp_trajectory_topic:={prefix}/pre_grasp',
        '-p', f'assembly_trajectory_topic:={prefix}/assembly',
        '-p', f'controller_action_name:={action}',
        '-p', f'status_topic:={prefix}/status',
        '-p', f'success_topic:={prefix}/success',
        '-p', f'duration_topic:={prefix}/duration',
        '-p', f'stage_status_topic:={prefix}/stage_status',
        '-p', 'wait_for_controller_sec:=2.0',
        '-p', 'result_timeout_sec:=0.4',
        '-p', 'cancel_on_timeout:=true',
    ])
    rclpy.init()
    node = Checker(prefix)
    deadline = time.monotonic() + 10.0
    last_publish = 0.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if server.poll() is not None or bridge.poll() is not None:
                print('FAIL: validation process exited unexpectedly')
                return 1
            now = time.monotonic()
            if (
                node.pre.get_subscription_count() > 0
                and node.assembly.get_subscription_count() > 0
                and now - last_publish > 0.2
            ):
                node.pre.publish(_trajectory(0.0))
                node.assembly.publish(_trajectory(0.1))
                last_publish = now
            if all(value is not None for value in (
                node.status, node.success, node.duration
            )):
                expected = {
                    'mode': 'ros2_control',
                    'simulated_execution_only': 'true',
                    'real_hardware': 'false',
                }
                if mode == 'success':
                    expected.update({'event': 'success', 'execution': 'true'})
                    expected_success = True
                else:
                    expected.update({
                        'event': 'failure', 'reason': 'result_timeout'
                    })
                    expected_success = False
                parsed = _fields(node.status)
                failures = [
                    f'{key}={value}' for key, value in expected.items()
                    if parsed.get(key) != value
                ]
                if failures or node.success is not expected_success:
                    print(f'FAIL: unexpected terminal status: {node.status}')
                    return 1
                print(f'PASS: ros2_control {mode} path is deterministic')
                print(f'      status: {node.status}')
                return 0
        print(f'FAIL: timed out waiting for {mode} terminal result')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in (bridge, server):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=3.0)
