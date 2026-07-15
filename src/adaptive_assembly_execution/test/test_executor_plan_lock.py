"""Focused tests for the physical executor's volatile plan-lock contract."""

from unittest.mock import patch

from adaptive_assembly_execution.physical_pick_place_executor_node import (
    DEFAULT_STAGE_NAMES,
    parse_stage_names,
    PhysicalPickPlaceExecutorNode,
)
from moveit_msgs.msg import RobotTrajectory
import rclpy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [f'panda_joint{index}' for index in range(1, 8)]
STAGES = parse_stage_names(DEFAULT_STAGE_NAMES)


def _trajectory(position=0.0):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = [position] * 7
    point.time_from_start.sec = 1
    result.joint_trajectory.points = [point]
    return result


def _lock(event, plan_id=1, stages=DEFAULT_STAGE_NAMES, **fields):
    message = String()
    values = {
        'event': event,
        'mode': 'sequence_plan_lock',
        'plan_id': str(plan_id),
        'stage_sequence': stages,
        'snapshot_stamp_ns': '100',
        'locked': 'true' if event == 'locked' else 'false',
        'planned_stage_count': str(len(STAGES) if event == 'locked' else 0),
        'execution': 'false',
        **{key: str(value) for key, value in fields.items()},
    }
    message.data = ';'.join(f'{key}={value}' for key, value in values.items())
    return message


def _executor(require_lock=True):
    rclpy.init(args=[
        '--ros-args',
        '-p', f'require_plan_lock:={str(require_lock).lower()}',
        '-p', 'require_joint_state:=false',
        '-p', 'require_physical_grasp_preflight:=false',
        '-p', 'send_arm_goals:=false',
        '-p', 'send_gripper_commands:=false',
        '-p', 'require_grasp_verification:=false',
        '-p', 'require_lift_verification:=false',
    ])
    return PhysicalPickPlaceExecutorNode()


def _close(node):
    node.destroy_node()
    rclpy.shutdown()


def test_trajectories_before_lock_cannot_start_and_matching_lock_can():
    node = _executor()
    try:
        with patch.object(node, '_try_start_sequence') as start:
            for stage in STAGES:
                node._trajectory_callback(stage, _trajectory())
        assert len(node._trajectories) == len(STAGES)
        assert node._started is False
        node._try_start_sequence()
        assert node._started is False
        with patch.object(node, '_try_start_sequence') as start:
            node._plan_lock_status_callback(_lock('locked'))
        assert node._plan_lock_valid is True
        assert node._locked_plan_id == 1
        start.assert_called_once_with()
    finally:
        _close(node)


def test_lock_before_last_trajectory_waits_for_complete_set():
    node = _executor()
    try:
        node._plan_lock_status_callback(_lock('planning_started', plan_id=7))
        with patch.object(node, '_try_start_sequence'):
            node._plan_lock_status_callback(_lock('locked', plan_id=7))
            for stage in STAGES[:-1]:
                node._trajectory_callback(stage, _trajectory())
        assert node._started is False
        assert node._locked_plan_id == 7
        with patch.object(node, '_try_start_sequence') as start:
            node._trajectory_callback(STAGES[-1], _trajectory())
        start.assert_called_once_with()
    finally:
        _close(node)


def test_new_attempt_clears_buffers_and_failure_cannot_execute_them():
    node = _executor()
    try:
        with patch.object(node, '_try_start_sequence'):
            for stage in STAGES:
                node._trajectory_callback(stage, _trajectory())
        node._plan_lock_status_callback(_lock('planning_started', plan_id=2))
        assert node._trajectories == {}
        with patch.object(node, '_try_start_sequence'):
            node._trajectory_callback('pre_grasp', _trajectory())
        node._plan_lock_status_callback(_lock('failure', plan_id=2))
        assert node._trajectories == {}
        assert node._plan_lock_valid is False
        assert node._completed is False
    finally:
        _close(node)


def test_mismatch_fails_and_legacy_mode_remains_backward_compatible():
    node = _executor()
    try:
        node._plan_lock_status_callback(_lock('locked', stages='grasp,place'))
        assert node._completed is True
        assert node._state == 'FAILURE'
    finally:
        _close(node)

    legacy = _executor(require_lock=False)
    try:
        with patch.object(legacy, '_try_start_sequence'):
            for stage in STAGES:
                legacy._trajectory_callback(stage, _trajectory())
        assert legacy._normal_prerequisites_ready() is True
    finally:
        _close(legacy)


def test_locked_duplicate_and_execution_updates_are_ignored():
    node = _executor()
    try:
        with patch.object(node, '_try_start_sequence'):
            node._trajectory_callback('grasp', _trajectory(0.0))
            node._plan_lock_status_callback(_lock('locked'))
            node._trajectory_callback('grasp', _trajectory(1.0))
        assert node._trajectories['grasp'].joint_trajectory.points[0].positions[0] == 0.0
        node._started = True
        node._trajectory_set_frozen = True
        node._trajectory_callback('lift', _trajectory(1.0))
        assert 'lift' not in node._trajectories
    finally:
        _close(node)
