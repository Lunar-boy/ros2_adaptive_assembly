#!/usr/bin/env python3
"""Exercise the dedicated PoseStamped observer input and stale handling."""

import os
import time

from adaptive_assembly_sim.gazebo_entity_pose_observer_node import (
    GazeboEntityPoseObserverNode,
)
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))


RAW_TOPIC = '/gazebo_target_object_pose_raw_test'
OUTPUT_TOPIC = '/gazebo_target_object_pose_test'
STATUS_TOPIC = '/gazebo_target_object_pose_status_test'
AVAILABLE_TOPIC = '/gazebo_target_object_pose_available_test'
AGE_TOPIC = '/gazebo_target_object_pose_age_ms_test'


class Fixture(Node):
    """Publish raw poses and collect the observer's public contract."""

    def __init__(self) -> None:
        super().__init__('gazebo_pose_stamped_observer_fixture')
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PoseStamped, RAW_TOPIC, 10)
        self.pose = None
        self.status = None
        self.available = None
        self.create_subscription(
            PoseStamped, OUTPUT_TOPIC,
            lambda message: setattr(self, 'pose', message), 10,
        )
        self.create_subscription(
            String, STATUS_TOPIC,
            lambda message: setattr(self, 'status', message.data), retained,
        )
        self.create_subscription(
            Bool, AVAILABLE_TOPIC,
            lambda message: setattr(self, 'available', message.data), retained,
        )


def _spin_until(executor, predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def _valid_pose() -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = 'source_frame_is_not_forwarded'
    message.pose.position.x = 0.35
    message.pose.position.y = 0.18
    message.pose.position.z = 0.10
    message.pose.orientation.x = 0.1
    message.pose.orientation.y = 0.2
    message.pose.orientation.z = 0.3
    message.pose.orientation.w = 0.9
    return message


def main() -> int:
    rclpy.init()
    observer = GazeboEntityPoseObserverNode(parameter_overrides=[
        Parameter('pose_info_topic', value=RAW_TOPIC),
        Parameter('input_message_type', value='pose_stamped'),
        Parameter('world_frame', value='world'),
        Parameter('output_pose_topic', value=OUTPUT_TOPIC),
        Parameter('status_topic', value=STATUS_TOPIC),
        Parameter('available_topic', value=AVAILABLE_TOPIC),
        Parameter('pose_age_ms_topic', value=AGE_TOPIC),
        Parameter('stale_timeout_sec', value=0.25),
        Parameter('publish_period_sec', value=0.02),
    ])
    fixture = Fixture()
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    executor.add_node(fixture)

    try:
        message = _valid_pose()
        for _ in range(10):
            fixture.publisher.publish(message)
            executor.spin_once(timeout_sec=0.03)
            if fixture.pose is not None and fixture.available is True:
                break
        valid = fixture.pose is not None and fixture.available is True
        if valid:
            pose = fixture.pose
            actual = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
            expected = (0.35, 0.18, 0.10, 0.1, 0.2, 0.3, 0.9)
            valid = (
                pose.header.frame_id == 'world'
                and all(abs(a - b) < 1e-9 for a, b in zip(actual, expected))
            )
        if not valid:
            print(
                'FAIL: valid PoseStamped was not preserved and made '
                f'available; status={fixture.status}, pose={fixture.pose}'
            )
            return 1

        stale = _spin_until(
            executor,
            lambda: fixture.available is False
            and fixture.status is not None
            and 'reason=pose_stream_stale' in fixture.status,
            1.0,
        )
        if not stale:
            print(
                'FAIL: stopped PoseStamped stream did not become stale; '
                f'status={fixture.status}, available={fixture.available}'
            )
            return 1

        invalid = _valid_pose()
        invalid.pose.position.x = float('nan')
        fixture.publisher.publish(invalid)
        rejected = _spin_until(
            executor,
            lambda: fixture.available is False
            and fixture.status is not None
            and 'reason=pose_non_finite' in fixture.status,
            1.0,
        )
        if not rejected:
            print(
                'FAIL: non-finite PoseStamped was not rejected; '
                f'status={fixture.status}, available={fixture.available}'
            )
            return 1

        print(
            'PASS: PoseStamped input preserves pose/frame contract, becomes '
            'available, becomes stale, and rejects non-finite data'
        )
        return 0
    finally:
        executor.shutdown()
        fixture.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
