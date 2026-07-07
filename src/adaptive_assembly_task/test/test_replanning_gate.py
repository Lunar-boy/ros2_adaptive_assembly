"""Unit tests for the target-pose replanning gate."""

from adaptive_assembly_task.replanning_gate import should_replan


def test_positive_threshold_skips_motion_at_or_below_threshold():
    assert not should_replan(0.02, 0.03)
    assert not should_replan(0.03, 0.03)


def test_positive_threshold_accepts_motion_above_threshold():
    assert should_replan(0.030001, 0.03)


def test_non_positive_threshold_disables_gate():
    assert should_replan(0.0, 0.0)
    assert should_replan(0.0, -0.01)
