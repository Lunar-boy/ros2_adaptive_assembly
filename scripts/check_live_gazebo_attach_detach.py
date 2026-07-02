#!/usr/bin/env python3
"""Run a bounded, headless live Gazebo attach/detach validation."""

import os
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, String


SERVICE = '/world/adaptive_assembly_workcell/set_pose'
STATUS_TOPIC = '/gazebo_attach_detach_status'
ATTACHED_TOPIC = '/gazebo_object_attached'
STATE_TOPIC = '/object_grasp_state'


class LiveFixture(Node):
    """Publish grasp events and observe the live attachment outputs."""

    def __init__(self) -> None:
        super().__init__('live_gazebo_attach_detach_checker')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.statuses = []
        self.attached = []
        self.state_publisher = self.create_publisher(String, STATE_TOPIC, qos)
        self.create_subscription(
            String, STATUS_TOPIC, lambda msg: self.statuses.append(msg.data), qos
        )
        self.create_subscription(
            Bool, ATTACHED_TOPIC, lambda msg: self.attached.append(msg.data), qos
        )
        self.service_client = self.create_client(SetEntityPose, SERVICE)


def wait_for(node: Node, description: str, predicate, timeout: float) -> None:
    """Spin until a condition is true or raise a diagnostic timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    statuses = getattr(node, 'statuses', [])
    detail = f'; recent statuses={statuses[-8:]}' if statuses else ''
    raise RuntimeError(f'timed out waiting for {description}{detail}')


def start(command, log_file):
    """Start one process in a group so all children can be terminated."""
    return subprocess.Popen(
        command,
        start_new_session=True,
        stdout=log_file,
        stderr=subprocess.STDOUT,
    )


def stop(process) -> None:
    """Bounded process-group shutdown."""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=2.0)


def main() -> int:
    """Exercise the real Gazebo SetEntityPose service path."""
    processes = []
    node = None
    log_path = None
    try:
        sim_share = get_package_share_directory('adaptive_assembly_sim')
        world = os.path.join(
            sim_share, 'worlds', 'adaptive_assembly_workcell.sdf'
        )
        with tempfile.NamedTemporaryFile(
            prefix='live_gazebo_attach_detach_', suffix='.log', delete=False
        ) as log_file:
            log_path = log_file.name
            processes.append(start([
                'ros2', 'launch', 'adaptive_assembly_sim',
                'adaptive_assembly_workcell.launch.py',
                f'gz_args:=-r -s {world}',
            ], log_file))
            processes.append(start([
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0.45', '--y', '0.0', '--z', '0.55',
                '--frame-id', 'world', '--child-frame-id', 'panda_hand',
            ], log_file))
            processes.append(start([
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                SERVICE + '@ros_gz_interfaces/srv/SetEntityPose'
                '@gz.msgs.Pose@gz.msgs.Boolean',
            ], log_file))
            processes.append(start([
                'ros2', 'run', 'adaptive_assembly_manipulation',
                'gazebo_attach_detach_node', '--ros-args',
                '-p', 'enable_service_calls:=true',
                '-p', 'service_timeout_sec:=2.0',
                '-p', 'attach_update_period_sec:=0.05',
            ], log_file))

        rclpy.init()
        node = LiveFixture()
        if not node.service_client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f'Gazebo service unavailable: {SERVICE}')
        wait_for(
            node, 'attach/detach node startup status',
            lambda: any('event=ready' in value for value in node.statuses), 8.0
        )

        status_start = len(node.statuses)
        attached_start = len(node.attached)
        node.state_publisher.publish(String(data=(
            'event=attached;object=target_object;parent=panda_hand'
        )))

        # One attached status is emitted for the logical transition and a
        # second only after SetEntityPose returns success.
        wait_for(
            node, 'successful live Gazebo attach request',
            lambda: sum(
                'event=attached' in value
                for value in node.statuses[status_start:]
            ) >= 2,
            10.0,
        )
        attach_statuses = node.statuses[status_start:]
        rejected_reasons = (
            'reason=service_calls_disabled',
            'reason=gazebo_service_unavailable',
        )
        if any(
            reason in value for value in attach_statuses
            for reason in rejected_reasons
        ):
            raise RuntimeError(
                f'attach used a non-live path: {attach_statuses}'
            )
        if any('event=failure' in value for value in attach_statuses):
            raise RuntimeError(f'Gazebo attach request failed: {attach_statuses}')
        if True not in node.attached[attached_start:]:
            raise RuntimeError(f'{ATTACHED_TOPIC} did not become true')

        status_start = len(node.statuses)
        attached_start = len(node.attached)
        node.state_publisher.publish(String(data=(
            'event=detached;object=target_object;parent=world'
        )))
        wait_for(
            node, 'detach status and attached=false',
            lambda: (
                any(
                    'event=detached' in value
                    for value in node.statuses[status_start:]
                )
                and False in node.attached[attached_start:]
            ),
            8.0,
        )
        print('PASS: live Gazebo attach request succeeded and detach completed')
        return 0
    except Exception as error:
        print(f'FAIL: {error}', file=sys.stderr)
        if log_path:
            print(f'Gazebo validation log: {log_path}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            stop(process)


if __name__ == '__main__':
    raise SystemExit(main())
