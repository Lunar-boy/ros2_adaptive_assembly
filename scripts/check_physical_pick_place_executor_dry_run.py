#!/usr/bin/env python3
"""Run a bounded message-only dry run of the PR66 executor."""

import subprocess
import time

from moveit_msgs.msg import RobotTrajectory

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


STAGES = ('pre_grasp', 'grasp', 'lift', 'pre_place', 'place', 'retreat')
JOINTS = [f'panda_joint{index}' for index in range(1, 8)]


class DryRunHarness(Node):
    """Publish deterministic inputs and wait for terminal success."""

    def __init__(self) -> None:
        super().__init__('physical_pick_place_executor_dry_run_harness')
        self.success = False
        self.failure = ''
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.trajectory_publishers = {
            stage: self.create_publisher(
                RobotTrajectory, f'/{stage}_trajectory', qos
            )
            for stage in STAGES
        }
        self.joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', qos
        )
        self.create_subscription(
            String,
            '/physical_pick_place_execution_status',
            self._status_callback,
            qos,
        )

    def _status_callback(self, message: String) -> None:
        if 'event=success' in message.data:
            self.success = True
        elif 'event=failure' in message.data:
            if 'reason=physical_grasp_preflight_failed' in message.data:
                return
            self.failure = message.data

    def publish_inputs(self) -> None:
        joint_state = JointState()
        joint_state.name = JOINTS
        joint_state.position = [0.0] * len(JOINTS)
        self.joint_state_publisher.publish(joint_state)
        for stage_index, stage in enumerate(STAGES):
            trajectory = RobotTrajectory()
            trajectory.joint_trajectory.joint_names = JOINTS
            point = JointTrajectoryPoint()
            point.positions = [0.01 * stage_index] * len(JOINTS)
            point.velocities = [0.0] * len(JOINTS)
            point.accelerations = [0.0] * len(JOINTS)
            point.time_from_start.sec = 1
            trajectory.joint_trajectory.points = [point]
            self.trajectory_publishers[stage].publish(trajectory)


def main() -> int:
    command = [
        'ros2', 'run', 'adaptive_assembly_execution',
        'physical_pick_place_executor_node',
        '--ros-args',
        '-p', 'send_arm_goals:=false',
        '-p', 'send_gripper_commands:=false',
        '-p', 'require_physical_grasp_preflight:=false',
        '-p', 'require_grasp_verification:=false',
        '-p', 'require_lift_verification:=false',
        '-p', 'require_joint_state:=true',
    ]
    process = subprocess.Popen(command)
    rclpy.init()
    node = DryRunHarness()
    deadline = time.monotonic() + 10.0
    try:
        while time.monotonic() < deadline and not node.success and not node.failure:
            node.publish_inputs()
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()

    if node.success:
        print('PASS physical pick-place executor dry run')
        return 0
    if node.failure:
        print('FAIL physical pick-place executor dry run')
        print(node.failure)
        return 1
    print('FAIL physical pick-place executor dry run: timed out')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
