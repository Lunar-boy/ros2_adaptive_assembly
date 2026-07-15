#!/usr/bin/env python3
"""Bounded headless regression for LIN planning and the physical plan lock."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]
STAGES = ('pre_grasp', 'grasp', 'lift', 'pre_place', 'place', 'retreat')
TRAJECTORY_TOPICS = {stage: f'/{stage}_trajectory' for stage in STAGES}


def parse_status(text):
    """Parse the repository's semicolon-delimited status schema."""
    result = {}
    for fragment in text.split(';'):
        if '=' in fragment:
            key, value = fragment.split('=', 1)
            result[key.strip()] = value.strip()
    return result


class PlanLockChecker(Node):
    """Observe one immutable physical plan and its executor handoff."""

    def __init__(self):
        super().__init__('physical_plan_lock_runtime_checker')
        volatile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        retained = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.lock_events = []
        self.stage_statuses = []
        self.trajectory_statuses = []
        self.executor_statuses = []
        self.scene_statuses = []
        self.trajectory_counts = {stage: 0 for stage in STAGES}
        self.planning_started_ids = []
        self.locked_plan_id = None
        self.lock_time = None
        self.grasp_metrics = None
        self.scene_locked_pose = None
        self.scene_locked_plan_id = None
        self.gazebo_pose = None
        self.gazebo_pose_count_after_lock = 0
        self.close_pose = None
        self.failure = ''
        self.create_subscription(
            String, '/assembly_sequence_plan_lock_status',
            self._lock, volatile,
        )
        self.create_subscription(
            String, '/assembly_sequence_stage_status',
            self._stage, 20,
        )
        self.create_subscription(
            String, '/assembly_sequence_trajectory_status',
            self._trajectory_status, 20,
        )
        self.create_subscription(
            String, '/physical_pick_place_stage_status',
            self._executor, retained,
        )
        self.create_subscription(
            String, '/physical_target_planning_scene_status',
            self._scene, retained,
        )
        self.create_subscription(
            PoseStamped, '/gazebo_target_object_pose', self._gazebo_pose, 20
        )
        self._trajectory_subscriptions = [
            self.create_subscription(
                RobotTrajectory, topic,
                lambda _, name=stage: self._count_trajectory(name), 20,
            )
            for stage, topic in TRAJECTORY_TOPICS.items()
        ]

    def _lock(self, message):
        fields = parse_status(message.data)
        self.lock_events.append(fields)
        event = fields.get('event')
        if event == 'planning_started':
            self.planning_started_ids.append(fields.get('plan_id'))
            if self.locked_plan_id is not None:
                self.failure = 'replanning_after_lock'
        elif event == 'locked':
            plan_id = fields.get('plan_id')
            if self.locked_plan_id is not None and plan_id != self.locked_plan_id:
                self.failure = 'multiple_locked_plan_ids'
            self.locked_plan_id = plan_id
            self.lock_time = time.monotonic()
        elif event == 'failure':
            self.failure = (
                'planning_failed:' + fields.get('reason', 'unknown')
            )

    def _stage(self, message):
        fields = parse_status(message.data)
        self.stage_statuses.append(fields)
        if fields.get('stage') == 'grasp' and fields.get('event') == 'success':
            self.grasp_metrics = fields

    def _trajectory_status(self, message):
        self.trajectory_statuses.append(parse_status(message.data))

    def _executor(self, message):
        fields = parse_status(message.data)
        self.executor_statuses.append(fields)
        if (
            fields.get('stage') == 'grasp'
            and fields.get('action') == 'gripper'
            and fields.get('command') == 'close'
            and fields.get('event') == 'sending'
            and self.close_pose is None
        ):
            self.close_pose = self.gazebo_pose

    def _scene(self, message):
        fields = parse_status(message.data)
        self.scene_statuses.append(fields)
        if fields.get('event') == 'locked' and fields.get('locked') == 'true':
            try:
                self.scene_locked_pose = tuple(
                    float(fields[key]) for key in ('target_x', 'target_y', 'target_z')
                )
            except (KeyError, ValueError):
                self.failure = 'locked_scene_pose_missing'
            self.scene_locked_plan_id = fields.get('plan_id')

    def _gazebo_pose(self, message):
        self.gazebo_pose = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        if self.lock_time is not None:
            self.gazebo_pose_count_after_lock += 1

    def _count_trajectory(self, stage):
        self.trajectory_counts[stage] += 1
        if self.trajectory_counts[stage] > 1:
            self.failure = f'duplicate_trajectory:{stage}'

    def complete(self):
        return (
            self.locked_plan_id is not None
            and self.grasp_metrics is not None
            and self.scene_locked_pose is not None
            and self.close_pose is not None
            and all(self.trajectory_counts.values())
            and self.lock_time is not None
            and time.monotonic() - self.lock_time >= 2.0
        )

    def validate(self):
        if self.failure:
            return self.failure, {}
        if len(set(self.planning_started_ids)) != 1:
            return 'planning_attempt_count_not_one', {}
        if any(count != 1 for count in self.trajectory_counts.values()):
            return 'trajectory_publication_count_invalid', {}
        metrics = self.grasp_metrics or {}
        if metrics.get('planning_pipeline_id') != 'pilz_industrial_motion_planner':
            return 'grasp_pipeline_invalid', metrics
        if metrics.get('planner_id') != 'LIN':
            return 'grasp_planner_invalid', metrics
        bounds = {
            'max_lateral_deviation': 0.002,
            'max_orientation_deviation': 0.01,
            'endpoint_position_error': 0.002,
            'endpoint_orientation_error': 0.01,
            'path_length_ratio': 1.02,
        }
        try:
            for name, bound in bounds.items():
                if not math.isfinite(float(metrics[name])) or float(metrics[name]) > bound:
                    return f'linear_metric_failed:{name}', metrics
        except (KeyError, ValueError):
            return 'linear_metrics_missing', metrics
        if metrics.get('monotonic_progress') != 'true':
            return 'linear_progress_non_monotonic', metrics
        if self.scene_locked_plan_id != self.locked_plan_id:
            return 'target_scene_plan_id_mismatch', metrics
        executor_ids = {
            status.get('plan_id') for status in self.executor_statuses
            if status.get('event') in ('sending', 'accepted', 'success')
        }
        if executor_ids != {self.locked_plan_id}:
            return 'executor_plan_id_mismatch', metrics
        if self.gazebo_pose_count_after_lock < 5:
            return 'gazebo_pose_stream_stopped_after_scene_lock', metrics
        if self.scene_locked_pose is None or self.close_pose is None:
            return 'target_displacement_unavailable', metrics
        dx = self.close_pose[0] - self.scene_locked_pose[0]
        dy = self.close_pose[1] - self.scene_locked_pose[1]
        dz = self.close_pose[2] - self.scene_locked_pose[2]
        xy = math.hypot(dx, dy)
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        result = {
            'plan_id': self.locked_plan_id,
            'trajectory_counts': self.trajectory_counts,
            'grasp_metrics': metrics,
            'gazebo_pose_count_after_lock': self.gazebo_pose_count_after_lock,
            'target_xy_displacement_before_close_m': xy,
            'target_3d_displacement_before_close_m': distance,
            'planning_started_ids': self.planning_started_ids,
        }
        if xy > 0.002:
            return 'target_xy_displacement_before_close', result
        if distance > 0.003:
            return 'target_3d_displacement_before_close', result
        return '', result


def stop_launch(process):
    """Stop the complete launch process group without leaving Gazebo behind."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=240.0)
    parser.add_argument('--run-dir', type=Path)
    args = parser.parse_args()
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    run_dir = args.run_dir or ROOT / 'runs' / f'plan_lock_{timestamp}'
    run_dir.mkdir(parents=True, exist_ok=True)
    world = Path(get_package_share_directory('adaptive_assembly_sim')) / (
        'worlds/adaptive_assembly_physical_workcell.sdf'
    )
    command = [
        'ros2', 'launch', 'adaptive_assembly_bringup',
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'use_sim_time:=true', 'launch_fake_object_pose_node:=false',
        'launch_object_pose_observer:=true', 'end_effector_link:=assembly_tcp',
        f'gz_args:=-s {world}',
    ]
    launch_log_path = run_dir / 'launch.log'
    with launch_log_path.open('w', encoding='utf-8') as launch_log:
        process = subprocess.Popen(
            command, cwd=ROOT, stdout=launch_log, stderr=subprocess.STDOUT,
            start_new_session=True, text=True,
        )
        checker = None
        try:
            rclpy.init()
            checker = PlanLockChecker()
            deadline = time.monotonic() + args.timeout_sec
            while time.monotonic() < deadline and not checker.complete():
                if process.poll() is not None:
                    checker.failure = f'launch_exited_early:{process.returncode}'
                    break
                rclpy.spin_once(checker, timeout_sec=0.05)
                if checker.failure:
                    break
            if not checker.complete() and not checker.failure:
                checker.failure = 'runtime_timeout'
            reason, result = checker.validate()
            result['passed'] = not reason
            result['reason'] = reason or 'success'
            (run_dir / 'result.json').write_text(
                json.dumps(result, indent=2, sort_keys=True) + '\n',
                encoding='utf-8',
            )
            if reason:
                print(f'FAIL: {reason}; artifacts={run_dir}')
                return 1
            print(
                'PASS: one locked LIN plan; '
                f'plan_id={result["plan_id"]}; '
                f'trajectory_counts={result["trajectory_counts"]}; '
                'target_xy_displacement_before_close_m='
                f'{result["target_xy_displacement_before_close_m"]:.6f}; '
                'target_3d_displacement_before_close_m='
                f'{result["target_3d_displacement_before_close_m"]:.6f}; '
                f'artifacts={run_dir}'
            )
            return 0
        finally:
            if checker is not None:
                checker.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            stop_launch(process)


if __name__ == '__main__':
    raise SystemExit(main())
