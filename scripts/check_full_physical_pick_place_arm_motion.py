#!/usr/bin/env python3
"""Prove the full physical demo accepts a goal and starts Panda arm motion."""

import argparse
import math
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory

from control_msgs.action import FollowJointTrajectory

from controller_manager_msgs.srv import ListControllers

from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import RobotTrajectory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool, String


ROOT = Path(__file__).resolve().parents[1]
STAGES = ('pre_grasp', 'grasp', 'lift', 'pre_place', 'place', 'retreat')
PANDA_JOINTS = tuple(f'panda_joint{index}' for index in range(1, 8))
STATUS_TOPICS = (
    '/gazebo_target_object_pose_status',
    '/physical_grasp_preflight_status',
    '/assembly_sequence_planning_status',
    '/assembly_sequence_trajectory_status',
    '/physical_pick_place_stage_status',
    '/physical_pick_place_execution_status',
    '/grasp_verification_status',
)
REJECTED_STATUS_TOKENS = (
    'entity_names_unavailable',
    'object_pose_unavailable',
    'arm_controller_unavailable',
    'arm_goal_rejected',
)
MILESTONE_NAMES = (
    'gazebo_controllers_active',
    'valid_panda_joint_states',
    'target_object_pose_available',
    'physical_grasp_preflight_success',
    'six_stage_sequence_planning_success',
    'six_non_empty_trajectories_received',
    'physical_executor_entered_pre_grasp',
    'arm_follow_joint_trajectory_goal_accepted',
    'panda_arm_joint_motion_observed',
)


def parse_status(value: str) -> Dict[str, str]:
    """Parse the repository's semicolon-delimited status schema."""
    fields = {}
    for fragment in value.split(';'):
        if '=' in fragment:
            key, field_value = fragment.split('=', 1)
            fields[key.strip()] = field_value.strip()
    return fields


class ArmMotionChecker(Node):
    """Collect ordered evidence without changing the running demo."""

    def __init__(self, motion_threshold: float) -> None:
        """Subscribe to acceptance evidence and controller readiness."""
        super().__init__('full_physical_pick_place_arm_motion_checker')
        self.motion_threshold = motion_threshold
        self.statuses: Dict[str, List[str]] = {
            topic: [] for topic in STATUS_TOPICS
        }
        self.trajectories: Dict[str, RobotTrajectory] = {}
        self.controllers: Dict[str, str] = {}
        self.initial_joints: Optional[Dict[str, float]] = None
        self.latest_joints: Optional[Dict[str, float]] = None
        self.max_joint_delta = 0.0
        self.valid_joint_state = False
        self.target_pose_received = False
        self.target_pose_available = False
        self.preflight_success = False
        self.planning_success = False
        self.executor_pre_grasp = False
        self.arm_goal_accepted = False
        self.failure_reason = ''
        self.milestones: List[str] = []
        self._controller_future = None
        self._last_controller_request = 0.0

        for topic in STATUS_TOPICS:
            self.create_subscription(
                String,
                topic,
                lambda message, name=topic: self._status_callback(
                    name, message
                ),
                50,
            )
        self.create_subscription(
            Bool,
            '/gazebo_target_object_pose_available',
            self._availability_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/gazebo_target_object_pose',
            self._pose_callback,
            10,
        )
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 50
        )
        for stage in STAGES:
            self.create_subscription(
                RobotTrajectory,
                f'/{stage}_trajectory',
                lambda message, name=stage: self._trajectory_callback(
                    name, message
                ),
                10,
            )

        self.controller_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory',
        )

    def _status_callback(self, topic: str, message: String) -> None:
        value = message.data
        self.statuses[topic].append(value)
        self.statuses[topic] = self.statuses[topic][-100:]
        if not self.failure_reason:
            for token in REJECTED_STATUS_TOKENS:
                if token in value:
                    self.failure_reason = f'rejected_status:{token}'
                    return

        fields = parse_status(value)
        event = fields.get('event', '')
        if topic == '/physical_grasp_preflight_status':
            if event == 'success' and fields.get('reason') == 'ok':
                self.preflight_success = True
            elif event == 'failure':
                self.failure_reason = (
                    'physical_preflight_failed:'
                    + fields.get('reason', 'unknown')
                )
        elif topic == '/assembly_sequence_planning_status':
            self.planning_success = (
                event == 'success'
                and fields.get('planned_stage_count') == '6'
                and fields.get('requested_stage_count') == '6'
            )
            if event == 'failure':
                self.failure_reason = (
                    'six_stage_planning_failed:'
                    + fields.get(
                        'reason', fields.get('failed_stage', 'unknown')
                    )
                )
        elif topic == '/physical_pick_place_stage_status':
            if (
                event == 'sending'
                and fields.get('stage') == 'pre_grasp'
                and fields.get('action') == 'arm'
            ):
                self.executor_pre_grasp = True
            if (
                event == 'accepted'
                and fields.get('stage') == 'pre_grasp'
                and fields.get('action') == 'arm'
                and fields.get('controller_goal_accepted') == 'true'
            ):
                self.arm_goal_accepted = True
            if event == 'failure' and fields.get('stage') == 'pre_grasp':
                self.failure_reason = (
                    'pre_grasp_execution_failed:'
                    + fields.get('reason', 'unknown')
                )

    def _availability_callback(self, message: Bool) -> None:
        self.target_pose_available = bool(message.data)

    def _pose_callback(self, message: PoseStamped) -> None:
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        self.target_pose_received = (
            bool(message.header.frame_id)
            and all(math.isfinite(value) for value in values)
        )

    def _joint_state_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        positions = dict(zip(message.name, message.position))
        if not all(
            name in positions and math.isfinite(float(positions[name]))
            for name in PANDA_JOINTS
        ):
            return
        sample = {name: float(positions[name]) for name in PANDA_JOINTS}
        self.valid_joint_state = True
        if self.initial_joints is None:
            self.initial_joints = sample
        self.latest_joints = sample
        self.max_joint_delta = max(
            self.max_joint_delta,
            max(
                abs(sample[name] - self.initial_joints[name])
                for name in PANDA_JOINTS
            ),
        )

    def _trajectory_callback(
        self, stage: str, message: RobotTrajectory
    ) -> None:
        trajectory = message.joint_trajectory
        if not trajectory.joint_names or not trajectory.points:
            self.failure_reason = f'empty_trajectory:{stage}'
            return
        if tuple(trajectory.joint_names) != PANDA_JOINTS:
            self.failure_reason = f'invalid_panda_trajectory_joints:{stage}'
            return
        self.trajectories.setdefault(stage, message)

    def poll_controllers(self) -> None:
        """Poll controller manager without blocking the ROS executor."""
        now = time.monotonic()
        if (
            self._controller_future is not None
            or now - self._last_controller_request < 0.5
            or not self.controller_client.service_is_ready()
        ):
            return
        self._last_controller_request = now
        self._controller_future = self.controller_client.call_async(
            ListControllers.Request()
        )
        self._controller_future.add_done_callback(
            self._controller_response_callback
        )

    def _controller_response_callback(self, future) -> None:
        self._controller_future = None
        try:
            response = future.result()
        except Exception:
            return
        self.controllers = {
            controller.name: controller.state
            for controller in response.controller
        }

    def controllers_ready(self) -> bool:
        """Require both state broadcaster and Panda arm controller active."""
        return (
            self.controllers.get('joint_state_broadcaster') == 'active'
            and self.controllers.get('panda_arm_controller') == 'active'
            and self.action_client.server_is_ready()
        )

    def update_ordered_milestones(self) -> None:
        """Advance only through the required acceptance sequence."""
        predicates = (
            self.controllers_ready(),
            self.valid_joint_state,
            self.target_pose_available and self.target_pose_received,
            self.preflight_success,
            self.planning_success,
            set(self.trajectories) == set(STAGES),
            self.executor_pre_grasp,
            self.arm_goal_accepted,
            self.max_joint_delta >= self.motion_threshold,
        )
        while (
            len(self.milestones) < len(MILESTONE_NAMES)
            and predicates[len(self.milestones)]
        ):
            name = MILESTONE_NAMES[len(self.milestones)]
            self.milestones.append(name)
            self.get_logger().info(f'PASS milestone: {name}')

    def diagnostic_text(self) -> str:
        """Return collected evidence suitable for a failure artifact."""
        lines = [
            f'milestones={",".join(self.milestones) or "none"}',
            f'controllers={self.controllers}',
            f'action_server_available={self.action_client.server_is_ready()}',
            f'valid_joint_state={self.valid_joint_state}',
            f'target_pose_received={self.target_pose_received}',
            f'target_pose_available={self.target_pose_available}',
            f'preflight_success={self.preflight_success}',
            f'planning_success={self.planning_success}',
            f'trajectories={sorted(self.trajectories)}',
            f'executor_pre_grasp={self.executor_pre_grasp}',
            f'arm_goal_accepted={self.arm_goal_accepted}',
            f'max_joint_delta_rad={self.max_joint_delta:.6f}',
        ]
        for topic, values in self.statuses.items():
            lines.append(f'[{topic}]')
            lines.extend(values or ['<no messages>'])
        return '\n'.join(lines) + '\n'


def parse_args():
    """Parse bounded runtime-check arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=240.0)
    parser.add_argument(
        '--joint-motion-threshold-rad', type=float, default=0.01
    )
    parser.add_argument('--run-dir', type=Path)
    return parser.parse_args()


def stop_launch(process: subprocess.Popen) -> None:
    """Stop the launched process group, escalating after a short grace."""
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)


def timeout_reason(checker: ArmMotionChecker) -> str:
    """Map the first missing milestone to one concrete acceptance failure."""
    missing = MILESTONE_NAMES[len(checker.milestones)]
    reasons = {
        'gazebo_controllers_active': 'arm_controller_unavailable',
        'valid_panda_joint_states': 'missing_valid_panda_joint_states',
        'target_object_pose_available': 'missing_target_pose',
        'physical_grasp_preflight_success': 'missing_preflight_success',
        'six_stage_sequence_planning_success': 'missing_planning_success',
        'six_non_empty_trajectories_received': 'missing_trajectories:'
        + ','.join(sorted(set(STAGES) - set(checker.trajectories))),
        'physical_executor_entered_pre_grasp': (
            'executor_never_entered_pre_grasp'
        ),
        'arm_follow_joint_trajectory_goal_accepted': 'arm_goal_not_accepted',
        'panda_arm_joint_motion_observed': 'arm_joint_motion_below_threshold',
    }
    return reasons[missing]


def main() -> int:
    """Launch the headless full demo and require bounded arm-motion proof."""
    args = parse_args()
    if args.timeout_sec <= 0.0 or args.joint_motion_threshold_rad <= 0.0:
        print('FAIL: timeout and joint-motion threshold must be positive')
        return 2

    timestamp = time.strftime('%Y%m%d_%H%M%S')
    run_dir = args.run_dir or ROOT / 'runs' / f'pr75_arm_motion_{timestamp}'
    run_dir.mkdir(parents=True, exist_ok=True)
    launch_log_path = run_dir / 'launch.log'
    status_log_path = run_dir / 'status_topics.log'
    world = (
        Path(get_package_share_directory('adaptive_assembly_sim'))
        / 'worlds/adaptive_assembly_physical_workcell.sdf'
    )
    command = [
        'ros2', 'launch', 'adaptive_assembly_bringup',
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'use_sim_time:=true',
        'launch_fake_object_pose_node:=false',
        'launch_object_pose_observer:=true',
        f'gz_args:=-s {world}',
    ]

    launch_log = launch_log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        command,
        cwd=ROOT,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    checker = None
    failure = ''
    try:
        rclpy.init()
        checker = ArmMotionChecker(args.joint_motion_threshold_rad)
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                failure = f'launch_exited_early:{process.returncode}'
                break
            rclpy.spin_once(checker, timeout_sec=0.1)
            checker.poll_controllers()
            checker.update_ordered_milestones()
            if checker.failure_reason:
                failure = checker.failure_reason
                break
            if len(checker.milestones) == len(MILESTONE_NAMES):
                break
        if not failure and len(checker.milestones) != len(MILESTONE_NAMES):
            failure = timeout_reason(checker)
        status_log_path.write_text(
            checker.diagnostic_text(), encoding='utf-8'
        )
        if failure:
            print(
                f'FAIL: {failure}; collected_statuses={status_log_path}; '
                f'launch_log={launch_log_path}'
            )
            print(checker.diagnostic_text(), end='')
            return 1
        print(
            'PASS: Panda pre_grasp goal accepted and arm motion observed; '
            f'max_joint_delta_rad={checker.max_joint_delta:.6f}; '
            f'collected_statuses={status_log_path}; '
            f'launch_log={launch_log_path}'
        )
        return 0
    finally:
        if checker is not None:
            checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_launch(process)
        launch_log.close()


if __name__ == '__main__':
    raise SystemExit(main())
