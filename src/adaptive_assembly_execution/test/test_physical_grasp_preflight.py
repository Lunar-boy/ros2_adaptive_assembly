"""Tests for physical grasp preflight parsing and executor gating."""

import time
from unittest.mock import patch

from adaptive_assembly_execution.physical_grasp_preflight_node import (
    bool_text,
    parse_status,
    PHYSICAL_POSE_INFO_TOPIC,
)
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


def _trajectory() -> RobotTrajectory:
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = [0.0] * len(JOINTS)
    point.velocities = [0.0] * len(JOINTS)
    point.accelerations = [0.0] * len(JOINTS)
    point.time_from_start.sec = 1
    trajectory.joint_trajectory.points = [point]
    return trajectory


def _ready_executor(node: PhysicalPickPlaceExecutorNode) -> None:
    for stage in parse_stage_names(DEFAULT_STAGE_NAMES):
        node._trajectories[stage] = _trajectory()
    node._current_joint_positions = {joint: 0.0 for joint in JOINTS}
    node._joint_state_received = True


def _make_executor(*, require_preflight: bool) -> PhysicalPickPlaceExecutorNode:
    rclpy.init(args=[
        '--ros-args',
        '-p', f'require_physical_grasp_preflight:={str(require_preflight).lower()}',
        '-p', 'physical_grasp_preflight_timeout_sec:=0.1',
        '-p', 'send_arm_goals:=false',
        '-p', 'send_gripper_commands:=false',
        '-p', 'require_grasp_verification:=false',
        '-p', 'require_lift_verification:=false',
    ])
    return PhysicalPickPlaceExecutorNode()


def test_preflight_status_parser_and_booleans():
    """Parse semicolon status and preserve physical world constants."""
    fields = parse_status(
        'event=failure;mode=physical_grasp_preflight;'
        'reason=kinematic_attach_node_active'
    )
    assert fields['event'] == 'failure'
    assert fields['mode'] == 'physical_grasp_preflight'
    assert fields['reason'] == 'kinematic_attach_node_active'
    assert bool_text(True) == 'true'
    assert bool_text(False) == 'false'
    assert PHYSICAL_POSE_INFO_TOPIC.endswith(
        '/adaptive_assembly_physical_workcell/pose/info'
    )


def test_executor_waits_for_physical_grasp_preflight_success():
    """Do not start arm execution while required preflight is pending."""
    node = _make_executor(require_preflight=True)
    try:
        _ready_executor(node)
        node._try_start_sequence()

        assert node._started is False
        assert node._completed is False
        assert node._state == 'WAIT_PREFLIGHT'
        assert node._preflight_deadline is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_executor_fails_on_physical_grasp_preflight_failure():
    """Turn a preflight failure status into a terminal executor failure."""
    node = _make_executor(require_preflight=True)
    try:
        _ready_executor(node)
        node._try_start_sequence()
        message = String()
        message.data = (
            'event=failure;mode=physical_grasp_preflight;'
            'reason=object_pose_unavailable;'
            'simulated_only=true;real_hardware=false'
        )
        with patch.object(
            node,
            '_publish_final_result',
            wraps=node._publish_final_result,
        ) as publish_result:
            node._physical_grasp_preflight_status_callback(message)

        assert node._started is False
        assert node._completed is True
        assert node._state == 'FAILURE'
        assert node._preflight_failure_reason == 'object_pose_unavailable'
        terminal_fields = parse_status(publish_result.call_args.args[1])
        assert terminal_fields['reason'] == 'physical_grasp_preflight_failed'
        assert terminal_fields['preflight_reason'] == 'object_pose_unavailable'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_executor_starts_after_physical_grasp_preflight_success():
    """Allow normal sequence startup after all prerequisites are ready."""
    node = _make_executor(require_preflight=True)
    try:
        _ready_executor(node)
        node._try_start_sequence()
        message = String()
        message.data = (
            'event=success;mode=physical_grasp_preflight;reason=ok;'
            'simulated_only=true;real_hardware=false'
        )
        node._physical_grasp_preflight_status_callback(message)

        assert node._preflight_success is True
        assert node._started is True
        assert node._completed is True
        assert node._state == 'SUCCESS'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_executor_times_out_waiting_for_physical_grasp_preflight():
    """Fail deterministically when required preflight never succeeds."""
    node = _make_executor(require_preflight=True)
    try:
        _ready_executor(node)
        node._try_start_sequence()
        node._preflight_deadline = time.monotonic() - 1.0
        node._timer_callback()

        assert node._started is False
        assert node._completed is True
        assert node._state == 'FAILURE'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_executor_preserves_old_behavior_when_preflight_is_disabled():
    """A message-only dry run can still execute without preflight."""
    node = _make_executor(require_preflight=False)
    try:
        _ready_executor(node)
        node._try_start_sequence()

        assert node._started is True
        assert node._completed is True
        assert node._state == 'SUCCESS'
    finally:
        node.destroy_node()
        rclpy.shutdown()
