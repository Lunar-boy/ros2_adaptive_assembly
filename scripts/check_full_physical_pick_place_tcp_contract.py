#!/usr/bin/env python3
"""Prove pre-grasp and grasp execution reaches the physical assembly TCP."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


ROOT = Path(__file__).resolve().parents[1]
STAGES = ('pre_grasp', 'grasp', 'lift', 'pre_place', 'place', 'retreat')
MEASURED_STAGES = ('pre_grasp', 'grasp')
PANDA_JOINTS = tuple(f'panda_joint{index}' for index in range(1, 8))
END_EFFECTOR_LINK = 'assembly_tcp'
STATUS_TOPICS = (
    '/gazebo_target_object_pose_status',
    '/physical_grasp_preflight_status',
    '/grasp_planning_status',
    '/grasp_trajectory_status',
    '/physical_pick_place_stage_status',
    '/physical_pick_place_execution_status',
)
MILESTONES = (
    'gazebo_controllers_active',
    'valid_panda_joint_states',
    'gazebo_target_pose_available',
    'target_pose_semantics_valid',
    'physical_grasp_preflight_success',
    'two_grasp_trajectories_planned',
    'pre_grasp_goal_accepted',
    'pre_grasp_arm_stage_succeeded',
    'pre_grasp_tcp_reached',
    'grasp_goal_accepted',
    'grasp_arm_stage_succeeded',
    'grasp_tcp_reached',
)
EXPECTED_STAGE_EVENTS = (
    ('pre_grasp', 'accepted'),
    ('pre_grasp', 'success'),
    ('grasp', 'accepted'),
    ('grasp', 'success'),
)


def parse_status(value: str) -> dict[str, str]:
    """Parse the repository's stable semicolon-delimited status schema."""
    fields = {}
    for fragment in value.split(';'):
        if '=' in fragment:
            key, field_value = fragment.split('=', 1)
            fields[key.strip()] = field_value.strip()
    return fields


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def normalize_quaternion(quaternion) -> tuple[float, float, float, float]:
    """Return a finite unit quaternion or raise for unusable input."""
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4 or not _finite(values):
        raise ValueError('quaternion must contain four finite values')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm must be nonzero')
    return tuple(value / norm for value in values)


def position_error(target, actual) -> float:
    """Compute Euclidean Cartesian position error in metres."""
    target_values = tuple(float(value) for value in target)
    actual_values = tuple(float(value) for value in actual)
    if len(target_values) != 3 or len(actual_values) != 3:
        raise ValueError('positions must contain three values')
    if not _finite(target_values + actual_values):
        raise ValueError('positions must be finite')
    return math.sqrt(sum(
        (left - right) ** 2
        for left, right in zip(target_values, actual_values)
    ))


def orientation_error(target, actual) -> float:
    """Compute the shortest relative quaternion rotation angle in radians."""
    target_unit = normalize_quaternion(target)
    actual_unit = normalize_quaternion(actual)
    dot = abs(sum(
        left * right for left, right in zip(target_unit, actual_unit)
    ))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def within_tolerance(
    position_error_m: float,
    orientation_error_rad: float,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> bool:
    """Use inclusive tolerance boundaries for runtime acceptance."""
    values = (
        position_error_m,
        orientation_error_rad,
        position_tolerance_m,
        orientation_tolerance_rad,
    )
    return (
        _finite(values)
        and position_error_m >= 0.0
        and orientation_error_rad >= 0.0
        and position_tolerance_m >= 0.0
        and orientation_tolerance_rad >= 0.0
        and position_error_m <= position_tolerance_m
        and orientation_error_rad <= orientation_tolerance_rad
    )


def pose_values(message: PoseStamped) -> dict:
    """Extract and validate one stamped pose for artifact serialization."""
    position = (
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
    )
    orientation = normalize_quaternion((
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ))
    if not message.header.frame_id or not _finite(position):
        raise ValueError('pose frame and finite position are required')
    return {
        'frame_id': message.header.frame_id,
        'position': list(position),
        'orientation': list(orientation),
    }


class StageOrderTracker:
    """Validate accepted/success ordering for the two measured arm stages."""

    def __init__(self) -> None:
        self.index = 0

    def observe(self, fields: dict[str, str]) -> str:
        """Advance an expected arm event or report an out-of-order event."""
        if fields.get('action') != 'arm':
            return ''
        event = (fields.get('stage', ''), fields.get('event', ''))
        if event not in EXPECTED_STAGE_EVENTS:
            return ''
        event_index = EXPECTED_STAGE_EVENTS.index(event)
        if event_index < self.index:
            return ''
        if event_index > self.index:
            return 'stage_order_invalid'
        self.index += 1
        return ''


def missing_tf_reason(tf_received: bool) -> str:
    """Classify a missing runtime transform deterministically."""
    return 'tf_unavailable' if not tf_received else ''


class TcpContractChecker(Node):
    """Collect physical execution status and authoritative runtime TF."""

    def __init__(
        self, position_tolerance_m: float, orientation_tolerance_rad: float
    ) -> None:
        super().__init__('full_physical_pick_place_tcp_contract_checker')
        self.position_tolerance_m = position_tolerance_m
        self.orientation_tolerance_rad = orientation_tolerance_rad
        self.statuses = {topic: [] for topic in STATUS_TOPICS}
        self.controllers = {}
        self.trajectories = set()
        self.targets = {}
        self.target_rows = []
        self.actual_rows = []
        self.best = {stage: None for stage in MEASURED_STAGES}
        self.gazebo_pose = None
        self.task_target_pose = None
        self.target_available = False
        self.valid_joint_state = False
        self.preflight_success = False
        self.planning_success = False
        self.goal_accepted = {stage: False for stage in MEASURED_STAGES}
        self.stage_success = {stage: False for stage in MEASURED_STAGES}
        self.failure_reason = ''
        self.passed_milestones = []
        self.tf_received = False
        self.order = StageOrderTracker()
        self._controller_future = None
        self._last_controller_request = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for topic in STATUS_TOPICS:
            self.create_subscription(
                String, topic,
                lambda message, name=topic: self._status(name, message),
                50,
            )
        self.create_subscription(
            Bool, '/gazebo_target_object_pose_available',
            lambda message: setattr(
                self, 'target_available', bool(message.data)
            ), 10,
        )
        self.create_subscription(
            PoseStamped, '/gazebo_target_object_pose',
            lambda message: self._set_reference_pose('gazebo', message), 10,
        )
        self.create_subscription(
            PoseStamped, '/target_pose',
            lambda message: self._set_reference_pose('target', message), 10,
        )
        for stage in MEASURED_STAGES:
            self.create_subscription(
                PoseStamped, f'/panda_{stage}_pose',
                lambda message, name=stage: self._target(name, message), 10,
            )
        self.create_subscription(
            JointState, '/joint_states', self._joint_state, 50
        )
        for stage in STAGES:
            self.create_subscription(
                RobotTrajectory, f'/{stage}_trajectory',
                lambda message, name=stage: self._trajectory(name, message), 10,
            )
        self.controller_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory',
        )

    def _status(self, topic: str, message: String) -> None:
        self.statuses[topic].append(message.data)
        self.statuses[topic] = self.statuses[topic][-200:]
        fields = parse_status(message.data)
        event = fields.get('event', '')
        if topic == '/physical_grasp_preflight_status':
            if event == 'success' and fields.get('reason') == 'ok':
                self.preflight_success = True
            elif event == 'failure':
                self.failure_reason = 'physical_grasp_preflight_failed'
        elif topic == '/grasp_planning_status':
            if event == 'success':
                valid_success = (
                    fields.get('planned_stage_count') == '2'
                    and fields.get('requested_stage_count') == '2'
                    and fields.get('end_effector_link') == END_EFFECTOR_LINK
                )
                self.planning_success = self.planning_success or valid_success
                if fields.get('end_effector_link') != END_EFFECTOR_LINK:
                    self.failure_reason = 'configured_end_effector_link_invalid'
            elif event == 'failure' and not self.planning_success:
                self.failure_reason = fields.get('reason', 'planning_failed')
        elif topic == '/physical_pick_place_stage_status':
            ordering_failure = self.order.observe(fields)
            if ordering_failure:
                self.failure_reason = ordering_failure
                return
            stage = fields.get('stage')
            if stage in MEASURED_STAGES and fields.get('action') == 'arm':
                if (
                    event == 'accepted'
                    and fields.get('controller_goal_accepted') == 'true'
                ):
                    self.goal_accepted[stage] = True
                elif event == 'success':
                    self.stage_success[stage] = True
                elif event == 'failure':
                    self.failure_reason = f'{stage}_arm_stage_failed'

    def _set_reference_pose(self, name: str, message: PoseStamped) -> None:
        try:
            values = pose_values(message)
        except ValueError:
            self.failure_reason = 'target_pose_semantics_mismatch'
            return
        if name == 'gazebo':
            self.gazebo_pose = values
        else:
            self.task_target_pose = values

    def _target(self, stage: str, message: PoseStamped) -> None:
        try:
            values = pose_values(message)
        except ValueError:
            self.failure_reason = 'target_pose_semantics_mismatch'
            return
        if values['frame_id'] != 'panda_link0':
            self.failure_reason = 'target_pose_semantics_mismatch'
            return
        self.target_rows.append({
            'time_monotonic': time.monotonic(),
            'stage': stage,
            **_flat_pose(values),
        })
        if not self.goal_accepted[stage]:
            self.targets[stage] = values

    def _joint_state(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        positions = dict(zip(message.name, message.position))
        self.valid_joint_state = all(
            name in positions and math.isfinite(float(positions[name]))
            for name in PANDA_JOINTS
        )

    def _trajectory(self, stage: str, message: RobotTrajectory) -> None:
        trajectory = message.joint_trajectory
        if (
            tuple(trajectory.joint_names) != PANDA_JOINTS
            or not trajectory.points
        ):
            self.failure_reason = f'empty_or_invalid_trajectory:{stage}'
            return
        self.trajectories.add(stage)

    def target_semantics_valid(self) -> bool:
        """Require physical /target_pose to equal the observed model center."""
        if self.gazebo_pose is None or self.task_target_pose is None:
            return False
        if (
            self.gazebo_pose['frame_id'] != 'world'
            or self.task_target_pose['frame_id'] != 'world'
        ):
            return False
        return (
            position_error(
                self.gazebo_pose['position'],
                self.task_target_pose['position'],
            ) <= 1.0e-6
            and orientation_error(
                self.gazebo_pose['orientation'],
                self.task_target_pose['orientation'],
            ) <= 1.0e-6
        )

    def poll_controllers(self) -> None:
        """Poll controller state without blocking ROS callbacks."""
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
        self._controller_future.add_done_callback(self._controllers_response)

    def _controllers_response(self, future) -> None:
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
        return (
            self.controllers.get('joint_state_broadcaster') == 'active'
            and self.controllers.get('panda_arm_controller') == 'active'
            and self.action_client.server_is_ready()
        )

    def sample_tcp(self) -> None:
        """Measure authoritative panda_link0-to-assembly_tcp TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'panda_link0', END_EFFECTOR_LINK, Time()
            )
        except TransformException:
            return
        position = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        orientation = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        try:
            orientation = normalize_quaternion(orientation)
            if not _finite(position):
                raise ValueError('nonfinite transform')
        except ValueError:
            self.failure_reason = 'tf_unavailable'
            return
        self.tf_received = True
        for stage in MEASURED_STAGES:
            if not self.goal_accepted[stage] or stage not in self.targets:
                continue
            if stage == 'pre_grasp' and self.goal_accepted['grasp']:
                continue
            target = self.targets[stage]
            pos_error = position_error(target['position'], position)
            rot_error = orientation_error(target['orientation'], orientation)
            sample = {
                'frame_id': 'panda_link0',
                'position': list(position),
                'orientation': list(orientation),
                'position_error_m': pos_error,
                'orientation_error_rad': rot_error,
                'time_monotonic': time.monotonic(),
            }
            self.actual_rows.append({
                'stage': stage,
                **_flat_pose(sample),
                'position_error_m': pos_error,
                'orientation_error_rad': rot_error,
                'time_monotonic': sample['time_monotonic'],
            })
            score = pos_error / self.position_tolerance_m + (
                rot_error / self.orientation_tolerance_rad
            )
            previous = self.best[stage]
            if previous is None or score < previous['score']:
                sample['score'] = score
                self.best[stage] = sample

    def stage_reached(self, stage: str) -> bool:
        sample = self.best[stage]
        return sample is not None and within_tolerance(
            sample['position_error_m'],
            sample['orientation_error_rad'],
            self.position_tolerance_m,
            self.orientation_tolerance_rad,
        )

    def update_milestones(self) -> None:
        predicates = (
            self.controllers_ready(),
            self.valid_joint_state,
            self.target_available and self.gazebo_pose is not None,
            self.target_semantics_valid(),
            self.preflight_success,
            self.planning_success and set(MEASURED_STAGES).issubset(self.trajectories),
            self.goal_accepted['pre_grasp'],
            self.stage_success['pre_grasp'],
            self.stage_reached('pre_grasp'),
            self.goal_accepted['grasp'],
            self.stage_success['grasp'],
            self.stage_reached('grasp'),
        )
        while (
            len(self.passed_milestones) < len(MILESTONES)
            and predicates[len(self.passed_milestones)]
        ):
            milestone = MILESTONES[len(self.passed_milestones)]
            self.passed_milestones.append(milestone)
            self.get_logger().info(f'PASS milestone: {milestone}')


def _flat_pose(values: dict) -> dict:
    position = values['position']
    orientation = values['orientation']
    return {
        'frame_id': values['frame_id'],
        'x': position[0], 'y': position[1], 'z': position[2],
        'qx': orientation[0], 'qy': orientation[1],
        'qz': orientation[2], 'qw': orientation[3],
    }


def terminal_reason(checker: TcpContractChecker) -> str:
    """Map the first missing milestone to an actionable terminal reason."""
    if not checker.tf_received and (
        checker.goal_accepted['pre_grasp'] or checker.goal_accepted['grasp']
    ):
        return 'tf_unavailable'
    missing = MILESTONES[len(checker.passed_milestones)]
    reasons = {
        'gazebo_controllers_active': 'arm_controller_unavailable',
        'valid_panda_joint_states': 'missing_valid_panda_joint_states',
        'gazebo_target_pose_available': 'missing_target_pose',
        'target_pose_semantics_valid': 'target_pose_semantics_mismatch',
        'physical_grasp_preflight_success': 'missing_preflight_success',
        'two_grasp_trajectories_planned': 'missing_grasp_trajectories',
        'pre_grasp_goal_accepted': 'pre_grasp_goal_not_accepted',
        'pre_grasp_arm_stage_succeeded': 'pre_grasp_arm_stage_failed',
        'pre_grasp_tcp_reached': _tcp_failure(checker, 'pre_grasp'),
        'grasp_goal_accepted': 'grasp_goal_not_accepted',
        'grasp_arm_stage_succeeded': 'grasp_arm_stage_failed',
        'grasp_tcp_reached': _tcp_failure(checker, 'grasp'),
    }
    return reasons[missing]


def _tcp_failure(checker: TcpContractChecker, stage: str) -> str:
    sample = checker.best[stage]
    if sample is None:
        return 'tf_unavailable'
    if sample['position_error_m'] > checker.position_tolerance_m:
        return f'{stage}_tcp_position_error'
    return f'{stage}_tcp_orientation_error'


def result_payload(checker: TcpContractChecker, reason: str) -> dict:
    """Build the stable machine-readable runtime artifact."""
    measurements = {}
    for stage in MEASURED_STAGES:
        actual = checker.best[stage]
        if actual is not None:
            actual = {key: value for key, value in actual.items() if key != 'score'}
        measurements[stage] = {
            'target': checker.targets.get(stage),
            'actual': actual,
            'position_error_m': (
                actual['position_error_m'] if actual is not None else None
            ),
            'orientation_error_rad': (
                actual['orientation_error_rad'] if actual is not None else None
            ),
        }
    return {
        'schema_version': 1,
        'passed': not reason,
        'terminal_reason': reason or 'success',
        'selected_end_effector_link': END_EFFECTOR_LINK,
        'base_frame': 'panda_link0',
        'tolerances': {
            'position_m': checker.position_tolerance_m,
            'orientation_rad': checker.orientation_tolerance_rad,
        },
        'passed_milestones': checker.passed_milestones,
        'measurements': measurements,
        'controllers': checker.controllers,
        'trajectory_stages': sorted(checker.trajectories),
    }


def stop_launch(process: subprocess.Popen) -> None:
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


def write_csv(path: Path, rows: list[dict], fields: tuple[str, ...]) -> None:
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=240.0)
    parser.add_argument('--position-tolerance-m', type=float, default=0.02)
    parser.add_argument('--orientation-tolerance-rad', type=float, default=0.10)
    parser.add_argument('--run-dir', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not _finite((
        args.timeout_sec,
        args.position_tolerance_m,
        args.orientation_tolerance_rad,
    )) or min(
        args.timeout_sec,
        args.position_tolerance_m,
        args.orientation_tolerance_rad,
    ) <= 0.0:
        print('FAIL: timeout and Cartesian tolerances must be finite and positive')
        return 2

    timestamp = time.strftime('%Y%m%d_%H%M%S')
    run_dir = args.run_dir or ROOT / 'runs' / f'tcp_contract_{timestamp}'
    run_dir.mkdir(parents=True, exist_ok=True)
    launch_log_path = run_dir / 'launch.log'
    status_log_path = run_dir / 'status_topics.log'
    targets_path = run_dir / 'tcp_targets.csv'
    actual_path = run_dir / 'tcp_actual.csv'
    result_path = run_dir / 'result.json'
    world = Path(get_package_share_directory('adaptive_assembly_sim')) / (
        'worlds/adaptive_assembly_physical_workcell.sdf'
    )
    command = [
        'ros2', 'launch', 'adaptive_assembly_bringup',
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'use_sim_time:=true',
        'launch_fake_object_pose_node:=false',
        'launch_object_pose_observer:=true',
        'end_effector_link:=assembly_tcp',
        f'gz_args:=-s {world}',
    ]

    launch_log = launch_log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        command, cwd=ROOT, stdout=launch_log, stderr=subprocess.STDOUT,
        start_new_session=True, text=True,
    )
    checker = None
    failure = ''
    try:
        rclpy.init()
        checker = TcpContractChecker(
            args.position_tolerance_m, args.orientation_tolerance_rad
        )
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                failure = f'launch_exited_early:{process.returncode}'
                break
            rclpy.spin_once(checker, timeout_sec=0.05)
            checker.poll_controllers()
            checker.sample_tcp()
            checker.update_milestones()
            if checker.failure_reason:
                failure = checker.failure_reason
                break
            if len(checker.passed_milestones) == len(MILESTONES):
                break
        if not failure and len(checker.passed_milestones) != len(MILESTONES):
            failure = terminal_reason(checker)

        status_lines = []
        for topic, values in checker.statuses.items():
            status_lines.append(f'[{topic}]')
            status_lines.extend(values or ['<no messages>'])
        status_log_path.write_text('\n'.join(status_lines) + '\n', encoding='utf-8')
        common_pose_fields = (
            'time_monotonic', 'stage', 'frame_id', 'x', 'y', 'z',
            'qx', 'qy', 'qz', 'qw',
        )
        write_csv(targets_path, checker.target_rows, common_pose_fields)
        write_csv(
            actual_path, checker.actual_rows,
            common_pose_fields + (
                'position_error_m', 'orientation_error_rad',
            ),
        )
        result = result_payload(checker, failure)
        result_path.write_text(
            json.dumps(result, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        if failure:
            print(f'FAIL: {failure}; artifacts={run_dir}')
            return 1
        pre = result['measurements']['pre_grasp']
        grasp = result['measurements']['grasp']
        print(
            'PASS: authoritative assembly_tcp reached pre_grasp and grasp; '
            f'pre_grasp_position_error_m={pre["position_error_m"]:.6f}; '
            f'pre_grasp_orientation_error_rad={pre["orientation_error_rad"]:.6f}; '
            f'grasp_position_error_m={grasp["position_error_m"]:.6f}; '
            f'grasp_orientation_error_rad={grasp["orientation_error_rad"]:.6f}; '
            f'artifacts={run_dir}'
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
