#!/usr/bin/env python3
"""Validate bounded stale-stream diagnostics without Gazebo."""

import os
import time

os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from adaptive_assembly_sim.gazebo_entity_pose_observer_node import (
    GazeboEntityPoseObserverNode,
)


class Checker(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_entity_pose_observer_stale_checker')
        self.status = None
        self.available = None
        retained = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/gazebo_target_object_pose_status',
            lambda message: setattr(self, 'status', message.data), retained)
        self.create_subscription(
            Bool, '/gazebo_target_object_pose_available',
            lambda message: setattr(self, 'available', message.data), retained)


def main() -> int:
    rclpy.init()
    observer = GazeboEntityPoseObserverNode(parameter_overrides=[
        Parameter('stale_timeout_sec', value=0.3),
        Parameter('publish_period_sec', value=0.05),
    ])
    checker = Checker()
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    executor.add_node(checker)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if (checker.status is not None
                and 'reason=pose_stream_stale' in checker.status
                and checker.available is False):
            break
    status, available = checker.status, checker.available
    executor.shutdown()
    checker.destroy_node()
    observer.destroy_node()
    rclpy.shutdown()
    if (status is None or 'event=skipped' not in status
            or 'reason=pose_stream_stale' not in status
            or available is not False):
        print(f'FAIL: stale state not observed; status={status}, '
              f'available={available}')
        return 1
    print('PASS: absent Gazebo pose stream reports stale and unavailable')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
