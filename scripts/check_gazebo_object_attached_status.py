#!/usr/bin/env python3
"""Check the retained public Gazebo attachment state and status topics."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class Checker(Node):
    """Wait for both retained attachment outputs."""

    def __init__(self) -> None:
        super().__init__('gazebo_object_attached_status_checker')
        qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state = None
        self.status = None
        self.create_subscription(
            Bool, '/gazebo_object_attached',
            lambda message: setattr(self, 'state', message.data), qos,
        )
        self.create_subscription(
            String, '/gazebo_attach_detach_status',
            lambda message: setattr(self, 'status', message.data), qos,
        )


def main() -> int:
    """Return success when valid retained messages arrive."""
    rclpy.init()
    node = Checker()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and (
        node.state is None or node.status is None
    ):
        rclpy.spin_once(node, timeout_sec=0.1)
    valid = (
        node.state is not None and node.status is not None
        and 'mode=gazebo_attach_detach' in node.status
        and 'simulated_only=true' in node.status
        and 'real_hardware=false' in node.status
    )
    node.destroy_node()
    rclpy.shutdown()
    print(
        ('PASS' if valid else 'FAIL')
        + ': retained Gazebo attachment status'
    )
    return 0 if valid else 1


if __name__ == '__main__':
    raise SystemExit(main())
