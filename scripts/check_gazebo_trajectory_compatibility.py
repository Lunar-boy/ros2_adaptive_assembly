#!/usr/bin/env python3
"""Validate Gazebo Panda controller, state, and planned trajectories."""

import argparse
import math
import sys
import time

from moveit_msgs.msg import RobotTrajectory

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient

from sensor_msgs.msg import JointState


EXPECTED_JOINTS = [f'panda_joint{index}' for index in range(1, 8)]


class CompatibilityChecker(Node):
    """Collect one current state and both planned trajectories."""

    def __init__(self) -> None:
        """Create controller parameter and trajectory/state clients."""
        super().__init__('gazebo_trajectory_compatibility_checker')
        self.joint_state = None
        self.trajectories = {}
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )
        self.create_subscription(
            RobotTrajectory,
            '/pre_grasp_trajectory',
            lambda message: self._trajectory_callback('pre_grasp', message),
            10,
        )
        self.create_subscription(
            RobotTrajectory,
            '/assembly_trajectory',
            lambda message: self._trajectory_callback('assembly', message),
            10,
        )
        self.parameter_client = AsyncParameterClient(
            self, '/panda_arm_controller'
        )

    def _joint_state_callback(self, message: JointState) -> None:
        self.joint_state = message

    def _trajectory_callback(
        self, stage: str, message: RobotTrajectory
    ) -> None:
        self.trajectories[stage] = message


def point_time(point) -> float:
    """Convert a trajectory point duration to seconds."""
    return (
        float(point.time_from_start.sec)
        + float(point.time_from_start.nanosec) * 1.0e-9
    )


def validate_trajectory(stage: str, message: RobotTrajectory) -> list[str]:
    """Print and validate one controller-bound trajectory."""
    trajectory = message.joint_trajectory
    failures = []
    times = [point_time(point) for point in trajectory.points]
    first = list(trajectory.points[0].positions) if trajectory.points else []
    print(f'{stage} trajectory joint_names: {list(trajectory.joint_names)}')
    print(f'{stage} first point positions: {first}')
    print(
        f'{stage} timing summary: point_count={len(times)}, '
        f'first={times[0] if times else None}, '
        f'last={times[-1] if times else None}, strictly_increasing='
        f'{all(a < b for a, b in zip(times, times[1:]))}'
    )
    if list(trajectory.joint_names) != EXPECTED_JOINTS:
        failures.append(f'{stage} trajectory/controller joint names differ')
    if not trajectory.points:
        failures.append(f'{stage} trajectory has no points')
    for index, point in enumerate(trajectory.points):
        if len(point.positions) != 7:
            failures.append(
                f'{stage} point {index} has {len(point.positions)} positions'
            )
        if not all(math.isfinite(value) for value in point.positions):
            failures.append(f'{stage} point {index} has non-finite positions')
    if any(a >= b for a, b in zip(times, times[1:])):
        failures.append(f'{stage} time_from_start is not strictly increasing')
    return failures


def main() -> int:
    """Run the bounded live compatibility check."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = CompatibilityChecker()
    failures = []
    try:
        if not node.parameter_client.wait_for_services(args.timeout_sec):
            print('FAIL: /panda_arm_controller parameter service unavailable')
            return 1
        future = node.parameter_client.get_parameters(['joints'])
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=args.timeout_sec
        )
        if not future.done() or future.result() is None:
            print('FAIL: could not read panda_arm_controller joints parameter')
            return 1
        controller_joints = list(
            future.result().values[0].string_array_value
        )
        print(f'Active controller joint list: {controller_joints}')
        if controller_joints != EXPECTED_JOINTS:
            failures.append(
                'active controller joint list is not panda_joint1..7'
            )

        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.joint_state is not None and len(node.trajectories) == 2:
                break

        if node.joint_state is None:
            failures.append('no /joint_states sample received')
        else:
            positions = dict(zip(
                node.joint_state.name, node.joint_state.position
            ))
            current = [positions.get(name) for name in EXPECTED_JOINTS]
            print(f'Current /joint_states: {current}')
            if any(value is None for value in current):
                failures.append('/joint_states is missing a Panda arm joint')

        for stage in ('pre_grasp', 'assembly'):
            if stage not in node.trajectories:
                failures.append(f'no /{stage}_trajectory sample received')
                continue
            failures.extend(
                validate_trajectory(stage, node.trajectories[stage])
            )

        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1
        print('PASS: controller, joint state, and trajectories are compatible')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
