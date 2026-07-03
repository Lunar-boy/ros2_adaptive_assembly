"""Tests for ros2_control target-sync status parsing."""

import time

from adaptive_assembly_execution.ros2_control_sequence_executor_node import (
    Ros2ControlSequenceExecutorNode,
    parse_status,
)

from moveit_msgs.msg import RobotTrajectory

import rclpy


def test_parse_target_sync_success_status():
    """Parse the fields used to open the execution gate."""
    fields = parse_status(
        'event=success;mode=gazebo_target_sync;entity=target_object'
    )
    assert fields['event'] == 'success'
    assert fields['mode'] == 'gazebo_target_sync'
    assert fields['entity'] == 'target_object'


def test_parse_status_ignores_non_key_value_fragments():
    """Ignore malformed fragments without losing valid fields."""
    assert parse_status('malformed;event=failure;reason=invalid_pose') == {
        'event': 'failure',
        'reason': 'invalid_pose',
    }


def test_ready_sequence_times_out_without_starting_execution():
    """Keep `_started` false and terminate when sync never succeeds."""
    rclpy.init(args=[
        '--ros-args',
        '-p', 'require_target_sync_success:=true',
        '-p', 'target_sync_timeout_sec:=0.1',
    ])
    node = Ros2ControlSequenceExecutorNode()
    try:
        node._pre_grasp_trajectory = RobotTrajectory()
        node._assembly_trajectory = RobotTrajectory()
        node._joint_state_received = True
        node._try_start_sequence()

        assert node._started is False
        assert node._target_sync_deadline is not None

        node._target_sync_deadline = time.monotonic() - 1.0
        node._target_sync_timeout_callback()

        assert node._started is False
        assert node._completed is True
    finally:
        node.destroy_node()
        rclpy.shutdown()
