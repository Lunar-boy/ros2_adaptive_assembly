#!/usr/bin/env python3
"""Verify the live Gazebo visual grasp-attach demo status sequence."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class DemoStatusChecker(Node):
    """Collect the bounded set of success events emitted by one demo run."""

    def __init__(self) -> None:
        super().__init__('gazebo_grasp_attach_demo_status_checker')
        transient_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.seen = {
            'target_sync': False,
            'grasp': False,
            'place': False,
            'attached': False,
            'detached': False,
            'gazebo_attached': False,
            'gazebo_detached': False,
            'gazebo_attached_bool': False,
            'gazebo_detached_bool': False,
            'final_success': False,
        }
        self.last_message = {
            '/gazebo_target_sync_status': '<none>',
            '/assembly_ros2_control_execution_stage_status': '<none>',
            '/object_grasp_state': '<none>',
            '/gazebo_attach_detach_status': '<none>',
            '/gazebo_object_attached': '<none>',
            '/assembly_ros2_control_execution_status': '<none>',
        }
        self.create_subscription(
            String, '/gazebo_target_sync_status', self._target_sync,
            transient_qos,
        )
        self.create_subscription(
            String, '/assembly_ros2_control_execution_stage_status',
            self._stage, transient_qos,
        )
        self.create_subscription(
            String, '/object_grasp_state', self._grasp_state, transient_qos,
        )
        self.create_subscription(
            String, '/gazebo_attach_detach_status', self._gazebo_status,
            transient_qos,
        )
        self.create_subscription(
            Bool, '/gazebo_object_attached', self._gazebo_attached,
            transient_qos,
        )
        self.create_subscription(
            String, '/assembly_ros2_control_execution_status',
            self._execution_status, transient_qos,
        )

    def _target_sync(self, message: String) -> None:
        self.last_message['/gazebo_target_sync_status'] = message.data
        if 'event=success' in message.data and 'mode=gazebo_target_sync' in message.data:
            self.seen['target_sync'] = True

    def _stage(self, message: String) -> None:
        self.last_message[
            '/assembly_ros2_control_execution_stage_status'
        ] = message.data
        if 'event=success' not in message.data:
            return
        if 'stage=grasp' in message.data:
            self.seen['grasp'] = True
        if 'stage=place' in message.data:
            self.seen['place'] = True

    def _grasp_state(self, message: String) -> None:
        self.last_message['/object_grasp_state'] = message.data
        if 'event=attached' in message.data and 'parent=panda_hand' in message.data:
            self.seen['attached'] = True
        if 'event=detached' in message.data and 'parent=world' in message.data:
            self.seen['detached'] = True

    def _gazebo_status(self, message: String) -> None:
        self.last_message['/gazebo_attach_detach_status'] = message.data
        if 'event=attached' in message.data and 'parent=panda_hand' in message.data:
            self.seen['gazebo_attached'] = True
        if 'event=detached' in message.data and 'parent=world' in message.data:
            self.seen['gazebo_detached'] = True

    def _gazebo_attached(self, message: Bool) -> None:
        self.last_message['/gazebo_object_attached'] = str(message.data)
        if message.data:
            self.seen['gazebo_attached_bool'] = True
        elif self.seen['gazebo_attached_bool']:
            # Ignore the retained startup False until an attach was observed.
            self.seen['gazebo_detached_bool'] = True

    def _execution_status(self, message: String) -> None:
        self.last_message['/assembly_ros2_control_execution_status'] = message.data
        if (
            'event=success' in message.data
            and 'mode=ros2_control' in message.data
            and 'execution=true' in message.data
        ):
            self.seen['final_success'] = True


def main() -> int:
    """Wait for the complete live sequence or return a bounded failure."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-sec', type=float, default=90.0)
    args = parser.parse_args()

    rclpy.init()
    node = DemoStatusChecker()
    deadline = time.monotonic() + args.timeout_sec
    try:
        while time.monotonic() < deadline and not all(node.seen.values()):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        missing = [name for name, observed in node.seen.items() if not observed]
        node.destroy_node()
        rclpy.shutdown()

    if missing:
        print('FAIL: missing Gazebo grasp-attach demo events: ' + ', '.join(missing))
        print('Observed conditions:')
        for name, observed in node.seen.items():
            print(f'  {name}: {observed}')
        print('Last messages:')
        for topic, message in node.last_message.items():
            print(f'  {topic}: {message}')
        return 1
    print('PASS: target sync, grasp, attach, place, detach, and final execution succeeded')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
