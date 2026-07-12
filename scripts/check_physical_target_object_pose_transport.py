#!/usr/bin/env python3
"""Validate the dedicated physical-workcell target pose transport path."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
os.environ.setdefault('GZ_PARTITION', f'adaptive_assembly_pr74_{os.getpid()}')


ROOT = Path(__file__).resolve().parents[1]
WORLD = (
    ROOT / 'install/adaptive_assembly_sim/share/adaptive_assembly_sim/worlds'
    / 'adaptive_assembly_physical_workcell.sdf'
)
GZ_TOPIC = '/model/target_object/pose'
RAW_TOPIC = '/gazebo_target_object_pose_raw'
OUTPUT_TOPIC = '/gazebo_target_object_pose'
AVAILABLE_TOPIC = '/gazebo_target_object_pose_available'
EXPECTED_POSITION = (0.35, 0.18, 0.10)
POSITION_TOLERANCE_M = 0.03


class Checker(Node):
    """Collect raw and observed target poses plus retained availability."""

    def __init__(self) -> None:
        super().__init__('physical_target_object_pose_transport_checker')
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.raw_pose = None
        self.observed_pose = None
        self.available = False
        self.create_subscription(
            PoseStamped, RAW_TOPIC,
            lambda message: setattr(self, 'raw_pose', message), 10,
        )
        self.create_subscription(
            PoseStamped, OUTPUT_TOPIC,
            lambda message: setattr(self, 'observed_pose', message), 10,
        )
        self.create_subscription(
            Bool, AVAILABLE_TOPIC,
            lambda message: setattr(self, 'available', message.data), retained,
        )


def _start(command, log_path: Path) -> subprocess.Popen:
    log = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        command,
        cwd=ROOT,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    process._validation_log = log  # type: ignore[attr-defined]
    return process


def _stop(processes) -> None:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    deadline = time.monotonic() + 5.0
    for process in reversed(processes):
        if process.poll() is None:
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
        process._validation_log.close()  # type: ignore[attr-defined]


def _gazebo_topic_exists(timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        result = subprocess.run(
            ['gz', 'topic', '-l'],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
        if GZ_TOPIC in result.stdout.splitlines():
            return True
        time.sleep(0.2)
    return False


def _position_matches(message: PoseStamped) -> bool:
    actual = (
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
    )
    return all(
        abs(value - expected) <= POSITION_TOLERANCE_M
        for value, expected in zip(actual, EXPECTED_POSITION)
    )


def main() -> int:
    if not WORLD.exists():
        print(
            f'FAIL: installed physical world is missing: {WORLD}. '
            'Run colcon build --symlink-install and source install/setup.bash.'
        )
        return 1

    processes = []
    with tempfile.TemporaryDirectory(prefix='pr74_pose_transport_') as temp:
        log_dir = Path(temp)
        try:
            processes.append(_start(
                ['gz', 'sim', '-s', '-r', str(WORLD)],
                log_dir / 'gazebo.log',
            ))
            if not _gazebo_topic_exists(20.0):
                print(
                    f'FAIL: dedicated Gazebo topic {GZ_TOPIC} did not appear; '
                    f'logs={log_dir}'
                )
                return 1

            processes.append(_start([
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'{GZ_TOPIC}@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
                '--ros-args', '-r', f'{GZ_TOPIC}:={RAW_TOPIC}',
            ], log_dir / 'bridge.log'))
            processes.append(_start([
                'ros2', 'run', 'adaptive_assembly_sim',
                'gazebo_entity_pose_observer_node', '--ros-args',
                '-p', f'pose_info_topic:={RAW_TOPIC}',
                '-p', 'input_message_type:=pose_stamped',
                '-p', 'world_frame:=world',
                '-p', 'stale_timeout_sec:=1.0',
            ], log_dir / 'observer.log'))

            rclpy.init()
            checker = Checker()
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                rclpy.spin_once(checker, timeout_sec=0.1)
                if (
                    checker.raw_pose is not None
                    and checker.observed_pose is not None
                    and checker.available
                    and _position_matches(checker.observed_pose)
                ):
                    break

            raw_pose = checker.raw_pose
            observed_pose = checker.observed_pose
            available = checker.available
            checker.destroy_node()
            rclpy.shutdown()

            if raw_pose is None:
                print('FAIL: raw ROS PoseStamped topic produced no message')
                return 1
            if not available:
                print('FAIL: observed target pose availability did not become true')
                return 1
            if observed_pose is None or not _position_matches(observed_pose):
                print(
                    'FAIL: observed target position was not approximately '
                    f'{EXPECTED_POSITION}; observed={observed_pose}'
                )
                return 1
            actual = (
                observed_pose.pose.position.x,
                observed_pose.pose.position.y,
                observed_pose.pose.position.z,
            )
            print(
                'PASS: dedicated Gazebo Pose and raw ROS PoseStamped are live; '
                f'availability=true; observed_position={actual}'
            )
            return 0
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            _stop(processes)


if __name__ == '__main__':
    raise SystemExit(main())
