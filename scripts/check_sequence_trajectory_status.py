#!/usr/bin/env python3
"""Validate trajectory export from the known-reachable sequence profile."""

import sys
import time
from typing import Dict, List

import rclpy
from moveit_msgs.msg import RobotTrajectory
from rclpy.node import Node
from std_msgs.msg import String


TIMEOUT_SEC = 60.0
PANDA_JOINTS = {f'panda_joint{index}' for index in range(1, 8)}
TRAJECTORY_TOPICS = {
    'pre_grasp': '/pre_grasp_trajectory',
    'assembly': '/assembly_trajectory',
}


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited key-value status string."""
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' in item:
            key, value = item.split('=', 1)
            fields[key] = value
    return fields


class SequenceTrajectoryChecker(Node):
    """Collect one trajectory and one published status for each stage."""

    def __init__(self) -> None:
        """Create trajectory and status subscriptions."""
        super().__init__('sequence_trajectory_status_checker')
        self.trajectories: Dict[str, List[RobotTrajectory]] = {
            stage: [] for stage in TRAJECTORY_TOPICS
        }
        self.statuses: Dict[str, str] = {}
        self.failure = ''

        self.create_subscription(
            RobotTrajectory,
            TRAJECTORY_TOPICS['pre_grasp'],
            lambda message: self.trajectory_callback('pre_grasp', message),
            10,
        )
        self.create_subscription(
            RobotTrajectory,
            TRAJECTORY_TOPICS['assembly'],
            lambda message: self.trajectory_callback('assembly', message),
            10,
        )
        self.create_subscription(
            String,
            '/assembly_sequence_trajectory_status',
            self.status_callback,
            10,
        )

    def trajectory_callback(
        self, stage: str, message: RobotTrajectory
    ) -> None:
        """Validate and retain a trajectory for one stage."""
        messages = self.trajectories[stage]
        messages.append(message)
        if len(messages) > 1:
            self.failure = f'{TRAJECTORY_TOPICS[stage]} published more than once'
            return

        joint_trajectory = message.joint_trajectory
        if not joint_trajectory.points:
            self.failure = f'{TRAJECTORY_TOPICS[stage]} has no trajectory points'
            return
        missing_joints = PANDA_JOINTS - set(joint_trajectory.joint_names)
        if missing_joints:
            self.failure = (
                f'{TRAJECTORY_TOPICS[stage]} is missing Panda joints: '
                f'{sorted(missing_joints)}'
            )

    def status_callback(self, message: String) -> None:
        """Validate and retain published trajectory status events."""
        fields = parse_status(message.data)
        if fields.get('event') != 'published':
            return

        stage = fields.get('stage', '')
        if stage not in TRAJECTORY_TOPICS:
            self.failure = f'invalid published trajectory stage: {message.data}'
            return
        if fields.get('execution') != 'false':
            self.failure = f'execution must be false: {message.data}'
            return
        if fields.get('topic') != TRAJECTORY_TOPICS[stage]:
            self.failure = f'unexpected trajectory topic in status: {message.data}'
            return
        try:
            point_count = int(fields['point_count'])
            joint_count = int(fields['joint_count'])
        except (KeyError, ValueError):
            self.failure = f'invalid trajectory counts in status: {message.data}'
            return
        if point_count < 1 or joint_count < len(PANDA_JOINTS):
            self.failure = f'empty or incomplete trajectory status: {message.data}'
            return

        self.statuses[stage] = message.data

    def complete(self) -> bool:
        """Return whether both stages have one trajectory and published status."""
        return all(
            len(self.trajectories[stage]) == 1 and stage in self.statuses
            for stage in TRAJECTORY_TOPICS
        )


def main() -> int:
    """Wait for valid exported trajectories from the reachable profile."""
    rclpy.init()
    node = SequenceTrajectoryChecker()
    deadline = time.monotonic() + TIMEOUT_SEC

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.failure:
                print(f'FAIL: {node.failure}')
                return 1
            if node.complete():
                print('PASS: one valid trajectory received for each sequence stage')
                print(f"      pre_grasp: {node.statuses['pre_grasp']}")
                print(f"      assembly: {node.statuses['assembly']}")
                return 0

        print('FAIL: timed out waiting for both exported sequence trajectories')
        print(
            '      trajectory counts: '
            + ', '.join(
                f'{stage}={len(messages)}'
                for stage, messages in node.trajectories.items()
            )
        )
        print(f'      published statuses: {sorted(node.statuses)}')
        print(
            'Start adaptive_assembly_panda_sequence_planning_reachable.launch.py '
            'before running this checker.'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
