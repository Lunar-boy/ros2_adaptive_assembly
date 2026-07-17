"""Focused state-machine tests for payload-aware two-phase execution."""

from unittest.mock import patch

from adaptive_assembly_execution.physical_pick_place_executor_node import (
    PhysicalPickPlaceExecutorNode,
)
from moveit_msgs.msg import RobotTrajectory
import rclpy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [f'panda_joint{index}' for index in range(1, 8)]


def _trajectory(position=0.0):
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = [position] * 7
    point.time_from_start.sec = 1
    trajectory.joint_trajectory.points = [point]
    return trajectory


def _node():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'two_phase_planning:=true',
        '-p', 'require_plan_lock:=true',
        '-p', 'require_joint_state:=false',
        '-p', 'require_physical_grasp_preflight:=false',
        '-p', 'send_arm_goals:=false',
        '-p', 'send_gripper_commands:=false',
        '-p', 'require_grasp_verification:=true',
        '-p', 'require_lift_verification:=true',
    ])
    return PhysicalPickPlaceExecutorNode()


def _close(node):
    node.destroy_node()
    rclpy.shutdown()


def _lock(phase, event='locked', plan_id=1):
    stages = (
        'pre_grasp,grasp'
        if phase == 'grasp' else 'lift,pre_place,place,retreat'
    )
    message = String()
    message.data = (
        f'event={event};mode=sequence_plan_lock;phase={phase};'
        f'plan_id={plan_id};stage_sequence={stages};locked=true;'
        f'planned_stage_count={len(stages.split(","))}'
    )
    return message


def _payload(operation, event='success', command_id='1', reason=''):
    message = String()
    message.data = (
        f'event={event};mode=payload_attachment;operation={operation};'
        f'command_id={command_id};payload_state='
        f'{"attached" if operation == "attach" else "world"};'
        f'reason={reason}'
    )
    return message


def test_transport_trajectory_and_lock_cannot_start_before_attachment():
    node = _node()
    try:
        node._trajectory_callback('lift', _trajectory())
        assert node._phase_trajectories['transport'] == {}
        node._phase_plan_lock_status_callback(
            'transport', _lock('transport', 'planning_started', 1000001)
        )
        assert node._completed is True
        assert node._state == 'FAILURE'
    finally:
        _close(node)


def test_distinct_phase_plan_ids_and_lock_before_trajectories_are_supported():
    node = _node()
    try:
        with patch.object(node, '_try_start_sequence'):
            node._phase_plan_lock_status_callback('grasp', _lock('grasp', plan_id=1))
            node._trajectory_callback('pre_grasp', _trajectory())
            node._trajectory_callback('grasp', _trajectory())
        node._payload_state = 'attached'
        with patch.object(node, '_try_start_transport') as start:
            node._phase_plan_lock_status_callback(
                'transport', _lock('transport', plan_id=1000001)
            )
            for stage in ('lift', 'pre_place', 'place', 'retreat'):
                node._trajectory_callback(stage, _trajectory())
        assert node._phase_plan_ids == {'grasp': 1, 'transport': 1000001}
        assert start.call_count >= 1
    finally:
        _close(node)


def test_attachment_failure_prevents_lift():
    node = _node()
    try:
        node._started = True
        node._current_stage = 'grasp'
        node._request_payload_operation('attach')
        with patch.object(node, '_send_current_arm_stage') as send_arm:
            node._payload_status_callback(
                _payload('attach', 'failure', reason='world_object_missing')
            )
        send_arm.assert_not_called()
        assert node._completed is True
        assert node._state == 'FAILURE'
    finally:
        _close(node)


def test_open_requests_detach_and_retreat_waits_for_success():
    node = _node()
    try:
        node._started = True
        node._payload_state = 'attached'
        node._current_stage_index = 4
        node._active_gripper_stage = 'place'
        node._active_gripper_command = 'open'
        node._active_gripper_command_id = '2'
        with patch.object(node, '_request_payload_operation') as request:
            node._finish_gripper_command(True, result='success')
        request.assert_called_once_with('detach')

        node._active_payload_operation = 'detach'
        node._active_payload_command_id = '1'
        with patch.object(node, '_send_current_arm_stage') as send_retreat:
            node._payload_status_callback(_payload('detach'))
        assert node._payload_state == 'world'
        assert node._current_stage_index == 5
        send_retreat.assert_called_once_with()
    finally:
        _close(node)


def test_detach_failure_and_transport_start_mismatch_prevent_motion():
    node = _node()
    try:
        node._started = True
        node._payload_state = 'attached'
        node._active_payload_operation = 'detach'
        node._active_payload_command_id = '1'
        with patch.object(node, '_send_current_arm_stage') as send_arm:
            node._payload_status_callback(
                _payload('detach', 'failure', reason='gazebo_pose_stale')
            )
        send_arm.assert_not_called()
        assert node._completed is True
    finally:
        _close(node)

    node = _node()
    try:
        node._started = True
        node._payload_state = 'attached'
        node._current_stage_index = 2
        node._phase_lock_valid['transport'] = True
        node._phase_plan_ids['transport'] = 1000001
        node._joint_state_received = True
        node._current_joint_positions = {joint: 0.0 for joint in JOINTS}
        for stage in ('lift', 'pre_place', 'place', 'retreat'):
            node._phase_trajectories['transport'][stage] = _trajectory(0.2)
        with patch.object(node, '_send_current_arm_stage') as send_arm:
            node._try_start_transport()
        send_arm.assert_not_called()
        assert node._completed is True
        assert node._state == 'FAILURE'
    finally:
        _close(node)
