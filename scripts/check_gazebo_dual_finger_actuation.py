#!/usr/bin/env python3
"""Bounded live check of symmetric Panda dual-finger Gazebo actuation."""

from __future__ import annotations

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


FINGER_JOINTS = ('panda_finger_joint1', 'panda_finger_joint2')


class CheckFailure(RuntimeError):
    """One actionable runtime contract failure."""


class DualFingerCheck(Node):
    """Query controller ownership, command both joints, and observe motion."""

    def __init__(self, timeout_sec: float, tolerance: float) -> None:
        super().__init__('check_gazebo_dual_finger_actuation')
        self.timeout_sec = timeout_sec
        self.tolerance = tolerance
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._state, 10)
        self.list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/panda_gripper_controller/follow_joint_trajectory',
        )

    def _state(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        for name, position in zip(message.name, message.position):
            if name in FINGER_JOINTS and math.isfinite(position):
                self.positions[name] = position

    def _spin_future(self, future, label: str):
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.timeout_sec
        )
        if not future.done():
            raise CheckFailure(f'timed out waiting for {label}')
        error = future.exception()
        if error is not None:
            raise CheckFailure(f'{label} failed: {error}')
        return future.result()

    def check_controller(self) -> None:
        if not self.list_client.wait_for_service(timeout_sec=self.timeout_sec):
            raise CheckFailure('controller manager list service is unavailable')
        response = self._spin_future(
            self.list_client.call_async(ListControllers.Request()),
            'controller list',
        )
        matches = [
            controller for controller in response.controller
            if controller.name == 'panda_gripper_controller'
        ]
        if len(matches) != 1:
            raise CheckFailure(
                'expected exactly one panda_gripper_controller, found '
                f'{len(matches)}'
            )
        controller = matches[0]
        if controller.state != 'active':
            raise CheckFailure(
                f'panda_gripper_controller state is {controller.state!r}'
            )
        expected = {f'{joint}/position' for joint in FINGER_JOINTS}
        claimed = set(controller.claimed_interfaces)
        if claimed != expected:
            raise CheckFailure(
                f'claimed interfaces are {sorted(claimed)!r}; expected '
                f'{sorted(expected)!r}'
            )

    def command_and_observe(self, target: float) -> tuple[float, float]:
        if not self.action_client.wait_for_server(timeout_sec=self.timeout_sec):
            raise CheckFailure('gripper trajectory action is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(FINGER_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [target, target]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        handle = self._spin_future(
            self.action_client.send_goal_async(goal), f'{target:.3f} goal'
        )
        if not handle.accepted:
            raise CheckFailure(f'{target:.3f} goal was rejected')
        wrapped = self._spin_future(
            handle.get_result_async(), f'{target:.3f} result'
        )
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise CheckFailure(
                f'{target:.3f} goal failed: status={wrapped.status}, '
                f'error_code={wrapped.result.error_code}, '
                f'error={wrapped.result.error_string!r}'
            )

        deadline = time.monotonic() + self.timeout_sec
        last = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not all(joint in self.positions for joint in FINGER_JOINTS):
                continue
            last = tuple(self.positions[joint] for joint in FINGER_JOINTS)
            if (
                all(abs(position - target) <= self.tolerance for position in last)
                and abs(last[0] - last[1]) <= 0.001
            ):
                return last
        raise CheckFailure(
            f'joint positions did not converge to {target:.3f}; '
            f'last={last!r}, tolerance={self.tolerance:.6f}'
        )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=10.0)
    parser.add_argument('--position-tolerance', type=float, default=0.002)
    args = parser.parse_args(argv)
    if args.timeout_sec <= 0.0 or args.position_tolerance <= 0.0:
        parser.error('timeouts and tolerances must be positive')

    rclpy.init()
    node = DualFingerCheck(args.timeout_sec, args.position_tolerance)
    samples: dict[str, tuple[float, float]] = {}
    try:
        node.check_controller()
        samples['open'] = node.command_and_observe(0.04)
        samples['intermediate_close'] = node.command_and_observe(0.02)
        samples['reopened'] = node.command_and_observe(0.04)
    except CheckFailure as error:
        print(f'FAIL: {error}')
        return_code = 1
        try:
            samples['reopen_after_failure'] = node.command_and_observe(0.04)
        except CheckFailure as reopen_error:
            print(f'  reopen also failed: {reopen_error}')
    else:
        differences = {
            name: abs(values[0] - values[1])
            for name, values in samples.items()
        }
        maximum = max(differences.values())
        for name, values in samples.items():
            print(
                f'{name}: q1={values[0]:.6f} m; q2={values[1]:.6f} m; '
                f'abs_difference={differences[name]:.6f} m'
            )
        print(f'max_abs_difference={maximum:.6f} m')
        print('PASS: both Gazebo Panda fingers track equal position commands')
        return_code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == '__main__':
    sys.exit(main())
