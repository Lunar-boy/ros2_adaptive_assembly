"""Component integration tests for executor contact-aware close handling."""

from unittest.mock import patch

from adaptive_assembly_execution.physical_pick_place_executor_node import (
    parse_status,
    PhysicalPickPlaceExecutorNode,
)
import rclpy
from std_msgs.msg import String


def _make_executor():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'send_arm_goals:=false',
        '-p', 'require_physical_grasp_preflight:=false',
        '-p', 'require_grasp_verification:=true',
        '-p', 'require_lift_verification:=true',
    ])
    return PhysicalPickPlaceExecutorNode()


def _activate_close(node):
    node._started = True
    node._state = 'WAIT_GRIPPER_RESULT'
    node._active_gripper_stage = 'grasp'
    node._active_gripper_command = 'close'
    node._active_gripper_command_id = '1'


def test_contact_limited_close_advances_only_to_grasp_verification():
    """Continue after close without marking the complete task successful."""
    node = _make_executor()
    try:
        _activate_close(node)
        message = String()
        message.data = (
            'event=success;command=close;command_id=1;'
            'result=contact_limited_success;goal_accepted=true;'
            'action_status=aborted;action_error_code=-5;'
            'expected_target_object=target_object;left_contact=true;'
            'right_contact=true;settle_sec=0.210000;'
            'simulated_only=true;real_hardware=false'
        )
        with patch.object(node, '_request_verification') as request:
            node._gripper_status_callback(message)

        request.assert_called_once_with('grasp', 'grasp')
        assert node._completed is False
        assert node._active_gripper_command is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_unilateral_close_failure_does_not_start_verification():
    """Preserve classified contact failure and prevent the lift path."""
    node = _make_executor()
    try:
        _activate_close(node)
        message = String()
        message.data = (
            'event=failure;command=close;command_id=1;'
            'result=unilateral_contact;reason=unilateral_contact;'
            'goal_accepted=true;action_status=aborted;'
            'left_contact=true;right_contact=false;'
            'simulated_only=true;real_hardware=false'
        )
        with patch.object(node, '_request_verification') as request:
            with patch.object(
                node,
                '_publish_final_result',
                wraps=node._publish_final_result,
            ) as publish_result:
                node._gripper_status_callback(message)

        request.assert_not_called()
        assert node._completed is True
        terminal = parse_status(publish_result.call_args.args[1])
        assert terminal['reason'] == 'gripper_command_failed'
        assert terminal['gripper_result'] == 'unilateral_contact'
    finally:
        node.destroy_node()
        rclpy.shutdown()
