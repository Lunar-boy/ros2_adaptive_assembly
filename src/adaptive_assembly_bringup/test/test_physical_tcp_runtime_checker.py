"""Unit tests for the bounded physical TCP runtime checker."""

import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / (
    'check_full_physical_pick_place_tcp_contract.py'
)
SPEC = importlib.util.spec_from_file_location('tcp_contract_checker', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_position_error_and_inclusive_threshold_boundary():
    """Use Euclidean distance and accept exact configured boundaries."""
    error = MODULE.position_error((0.0, 0.0, 0.0), (0.012, 0.016, 0.0))

    assert error == pytest.approx(0.02)
    assert MODULE.within_tolerance(error, 0.10, 0.02, 0.10)
    assert not MODULE.within_tolerance(error + 1.0e-9, 0.10, 0.02, 0.10)
    assert not MODULE.within_tolerance(error, 0.10 + 1.0e-9, 0.02, 0.10)


def test_orientation_error_uses_shortest_relative_quaternion_angle():
    """Treat q and -q equally and measure a relative rotation angle."""
    half_angle = 0.05
    rotated = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))

    assert MODULE.orientation_error((0.0, 0.0, 0.0, 1.0), rotated) == pytest.approx(0.10)
    assert MODULE.orientation_error(rotated, tuple(-v for v in rotated)) == pytest.approx(0.0)


def test_status_parsing_and_stage_ordering():
    """Require accepted then success for pre-grasp before grasp."""
    tracker = MODULE.StageOrderTracker()
    accepted = MODULE.parse_status(
        'event=accepted;mode=physical_pick_place;stage=pre_grasp;'
        'action=arm;controller_goal_accepted=true'
    )
    success = MODULE.parse_status(
        'event=success;stage=pre_grasp;action=arm'
    )

    assert accepted['controller_goal_accepted'] == 'true'
    assert tracker.observe(accepted) == ''
    assert tracker.observe(success) == ''
    out_of_order = {'stage': 'grasp', 'event': 'success', 'action': 'arm'}
    assert tracker.observe(out_of_order) == 'stage_order_invalid'


def test_missing_tf_and_nonfinite_inputs_are_rejected():
    """Classify missing TF and reject nonfinite pose/transform math."""
    assert MODULE.missing_tf_reason(False) == 'tf_unavailable'
    assert MODULE.missing_tf_reason(True) == ''
    with pytest.raises(ValueError, match='finite'):
        MODULE.position_error((0.0, 0.0, math.nan), (0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match='finite'):
        MODULE.orientation_error((0.0, 0.0, 0.0, math.inf), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match='nonzero'):
        MODULE.orientation_error((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_result_artifact_schema_contains_targets_actuals_and_errors():
    """Lock the machine-readable evidence fields used by CI artifacts."""
    checker = type('Checker', (), {})()
    checker.position_tolerance_m = 0.02
    checker.orientation_tolerance_rad = 0.10
    checker.targets = {
        stage: {
            'frame_id': 'panda_link0',
            'position': [0.4, 0.1, 0.3],
            'orientation': [1.0, 0.0, 0.0, 0.0],
        }
        for stage in MODULE.MEASURED_STAGES
    }
    checker.best = {
        stage: {
            'frame_id': 'panda_link0',
            'position': [0.4, 0.1, 0.3],
            'orientation': [1.0, 0.0, 0.0, 0.0],
            'position_error_m': 0.0,
            'orientation_error_rad': 0.0,
            'time_monotonic': 1.0,
            'score': 0.0,
        }
        for stage in MODULE.MEASURED_STAGES
    }
    checker.passed_milestones = list(MODULE.MILESTONES)
    checker.controllers = {'panda_arm_controller': 'active'}
    checker.trajectories = set(MODULE.STAGES)

    payload = MODULE.result_payload(checker, '')

    assert payload['passed'] is True
    assert payload['selected_end_effector_link'] == 'assembly_tcp'
    assert payload['tolerances'] == {
        'position_m': 0.02, 'orientation_rad': 0.10,
    }
    for stage in MODULE.MEASURED_STAGES:
        measurement = payload['measurements'][stage]
        assert measurement['target']['frame_id'] == 'panda_link0'
        assert measurement['actual']['frame_id'] == 'panda_link0'
        assert measurement['position_error_m'] == 0.0
        assert measurement['orientation_error_rad'] == 0.0
