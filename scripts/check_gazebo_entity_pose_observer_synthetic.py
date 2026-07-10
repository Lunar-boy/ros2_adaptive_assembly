#!/usr/bin/env python3
"""Validate the observer with a synthetic Jazzy Gazebo pose message."""

import os
import time

os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from adaptive_assembly_sim.gazebo_entity_pose_observer_node import (
    GazeboEntityPoseObserverNode,
    INPUT_MESSAGE_TYPE,
)


class Fixture(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_entity_pose_observer_synthetic_fixture')
        self.pose = None
        self.status = None
        self.available = None
        self.publisher = self.create_publisher(
            INPUT_MESSAGE_TYPE,
            '/world/adaptive_assembly_workcell/pose/info', 10)
        self.create_subscription(
            PoseStamped, '/gazebo_target_object_pose', self._on_pose, 10)
        retained = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/gazebo_target_object_pose_status',
            lambda message: setattr(self, 'status', message.data), retained)
        self.create_subscription(
            Bool, '/gazebo_target_object_pose_available',
            lambda message: setattr(self, 'available', message.data), retained)
        self.timer = self.create_timer(0.1, self._publish)

    def _on_pose(self, message: PoseStamped) -> None:
        self.pose = message

    def _publish(self) -> None:
        message = INPUT_MESSAGE_TYPE()
        if hasattr(message, 'pose'):
            candidate = message.pose.add()
            candidate.name = (
                'world::adaptive_assembly_physical_workcell::target_object'
            )
            candidate.position.x = 0.41
            candidate.position.y = -0.12
            candidate.position.z = 0.73
            candidate.orientation.z = 0.25
            candidate.orientation.w = 0.9682458366
        else:
            from geometry_msgs.msg import TransformStamped
            candidate = TransformStamped()
            candidate.child_frame_id = (
                'world/adaptive_assembly_physical_workcell/model/target_object'
            )
            candidate.transform.translation.x = 0.41
            candidate.transform.translation.y = -0.12
            candidate.transform.translation.z = 0.73
            candidate.transform.rotation.z = 0.25
            candidate.transform.rotation.w = 0.9682458366
            message.transforms.append(candidate)
        self.publisher.publish(message)


def main() -> int:
    rclpy.init()
    observer = GazeboEntityPoseObserverNode(parameter_overrides=[
        Parameter('require_model_name_match', value=False),
    ])
    fixture = Fixture()
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    executor.add_node(fixture)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if (fixture.pose is not None and fixture.available is True
                and fixture.status is not None
                and fixture.status.startswith('event=success;')):
            break

    pose, status, available = fixture.pose, fixture.status, fixture.available
    executor.shutdown()
    fixture.destroy_node()
    observer.destroy_node()
    rclpy.shutdown()
    if pose is None or status is None or available is not True:
        print(f'FAIL: missing output; status={status}, available={available}')
        return 1
    fields = dict(field.split('=', 1) for field in status.split(';'))
    expected = {
        'event': 'success',
        'mode': 'gazebo_entity_pose_observer',
        'entity': 'target_object',
        'simulated_only': 'true',
        'real_hardware': 'false',
    }
    if any(fields.get(key) != value for key, value in expected.items()):
        print(f'FAIL: invalid success status: {status}')
        return 1
    values = (pose.pose.position.x, pose.pose.position.y,
              pose.pose.position.z, pose.pose.orientation.z,
              pose.pose.orientation.w)
    wanted = (0.41, -0.12, 0.73, 0.25, 0.9682458366)
    if pose.header.frame_id != 'world' or any(
            abs(actual - expected_value) > 1e-6
            for actual, expected_value in zip(values, wanted)):
        print(f'FAIL: output pose does not match synthetic input: {pose}')
        return 1
    print(
        'PASS: scoped synthetic Gazebo entity pose produced matching '
        'observer output'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
