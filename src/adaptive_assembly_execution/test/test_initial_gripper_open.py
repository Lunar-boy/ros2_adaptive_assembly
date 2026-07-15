"""Focused state-machine tests for the physical executor's initial open."""

import time
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


def _trajectory():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = [0.0] * len(JOINTS)
    point.time_from_start.sec = 1
    trajectory.joint_trajectory.points = [point]
    return trajectory


def _make_executor(initial_open=True, require_gripper_success=True):
    rclpy.init(args=[
        '--ros-args',
        '-p', 'require_physical_grasp_preflight:=false',
        '-p', 'send_arm_goals:=false',
        '-p', 'send_gripper_commands:=true',
        '-p', 'require_gripper_success:='
        f'{str(require_gripper_success).lower()}',
        '-p', 'require_grasp_verification:=false',
        '-p', 'require_lift_verification:=false',
        '-p', 'open_gripper_before_first_arm_stage:='
        f'{str(initial_open).lower()}',
    ])
    node = PhysicalPickPlaceExecutorNode()
    for stage in parse_stage_names(DEFAULT_STAGE_NAMES):
        node._trajectories[stage] = _trajectory()
    node._current_joint_positions = {joint: 0.0 for joint in JOINTS}
    node._joint_state_received = True
    return node


def _status(event, command_id='1', result='success'):
    message = String()
    message.data = (
        f'event={event};command=open;command_id={command_id};'
        f'result={result};reason={result};action_status=succeeded'
    )
    return message


def test_initial_open_is_requested_before_first_arm_goal():
    """Enter the initial-open transition before dispatching pre_grasp."""
    node = _make_executor()
    calls = []
    try:
        with patch.object(
            node, '_send_gripper_command',
            side_effect=lambda stage, command: calls.append(
                ('gripper', stage, command)
            ),
        ), patch.object(
            node, '_send_current_arm_stage',
            side_effect=lambda: calls.append(('arm', 'pre_grasp')),
        ):
            node._try_start_sequence()
        assert calls == [('gripper', 'initial_open', 'open')]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_successful_matching_initial_open_advances_to_pre_grasp():
    """Use the matching bridge command ID before advancing to arm motion."""
    node = _make_executor()
    try:
        node._started = True
        node._send_gripper_command('initial_open', 'open')
        with patch.object(node, '_send_current_arm_stage') as send_arm:
            node._gripper_status_callback(_status('success'))
        send_arm.assert_called_once_with()
        assert node._current_stage_index == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_initial_open_failure_terminates_execution():
    """Reject an aborted/rejected initial open as a clear terminal failure."""
    node = _make_executor()
    try:
        node._started = True
        node._send_gripper_command('initial_open', 'open')
        with patch.object(
            node, '_publish_final_result', wraps=node._publish_final_result
        ) as publish_result:
            node._gripper_status_callback(
                _status('failure', result='action_aborted')
            )
        assert node._completed is True
        assert 'reason=initial_gripper_open_failed' in (
            publish_result.call_args.args[1]
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_initial_open_failure_cannot_be_ignored_by_optional_close_policy():
    """Always require initial open even when later gripper failures are optional."""
    node = _make_executor(require_gripper_success=False)
    try:
        node._started = True
        node._send_gripper_command('initial_open', 'open')
        node._gripper_status_callback(
            _status('failure', result='goal_rejected')
        )
        assert node._completed is True
        assert node._state == 'FAILURE'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_initial_open_timeout_terminates_execution():
    """Fail deterministically when the bridge does not return in time."""
    node = _make_executor()
    try:
        node._started = True
        node._send_gripper_command('initial_open', 'open')
        node._gripper_deadline = time.monotonic() - 1.0
        with patch.object(
            node, '_publish_final_result', wraps=node._publish_final_result
        ) as publish_result:
            node._timer_callback()
        assert node._completed is True
        assert 'reason=initial_gripper_open_timeout' in (
            publish_result.call_args.args[1]
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_disabling_initial_open_preserves_prior_start_sequence():
    """Dispatch pre_grasp directly when the profile option is disabled."""
    node = _make_executor(initial_open=False)
    try:
        with patch.object(node, '_send_current_arm_stage') as send_arm, \
                patch.object(node, '_send_gripper_command') as send_gripper:
            node._try_start_sequence()
        send_arm.assert_called_once_with()
        send_gripper.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stale_or_missing_command_ids_cannot_satisfy_initial_open():
    """Ignore retained and mismatched bridge results from earlier commands."""
    node = _make_executor()
    try:
        node._started = True
        node._send_gripper_command('initial_open', 'open')
        with patch.object(node, '_send_current_arm_stage') as send_arm:
            node._gripper_status_callback(_status('success', command_id='99'))
            node._gripper_status_callback(_status('success', command_id=''))
        send_arm.assert_not_called()
        assert node._active_gripper_command == 'open'
        assert node._completed is False
    finally:
        node.destroy_node()
        rclpy.shutdown()
