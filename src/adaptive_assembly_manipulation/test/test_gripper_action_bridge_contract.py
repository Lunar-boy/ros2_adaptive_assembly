"""Tests for the simulator-only dual-finger bridge contract."""

from adaptive_assembly_manipulation.gripper_action_bridge_node import (
    make_gripper_goal,
    PANDA_FINGER_JOINTS,
    validate_simulator_joint_names,
)
import pytest


def test_default_contract_builds_equal_open_and_close_goals():
    """Send the exact two-joint names and equal positions for both commands."""
    names = validate_simulator_joint_names(PANDA_FINGER_JOINTS)

    open_goal = make_gripper_goal(names, 0.04, 1.0)
    close_goal = make_gripper_goal(names, 0.0, 1.0)

    assert list(open_goal.trajectory.joint_names) == list(PANDA_FINGER_JOINTS)
    assert list(open_goal.trajectory.points[0].positions) == [0.04, 0.04]
    assert list(close_goal.trajectory.joint_names) == list(PANDA_FINGER_JOINTS)
    assert list(close_goal.trajectory.points[0].positions) == [0.0, 0.0]


@pytest.mark.parametrize('names', [
    [],
    ['panda_finger_joint1', 'panda_finger_joint1'],
    ['panda_finger_joint1'],
    ['panda_finger_joint2'],
    ['panda_finger_joint2', 'panda_finger_joint1'],
    ['panda_finger_joint1', 'unexpected_finger_joint'],
])
def test_invalid_simulator_joint_contracts_are_rejected(names):
    """Reject empty, duplicate, partial, reordered, and unexpected lists."""
    with pytest.raises(ValueError):
        validate_simulator_joint_names(names)
