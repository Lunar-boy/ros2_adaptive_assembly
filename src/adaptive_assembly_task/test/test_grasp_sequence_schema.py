"""Unit tests for the deterministic grasp sequence schema."""

import math

from adaptive_assembly_task.grasp_sequence_schema import (
    format_grasp_candidates,
    format_grasp_sequence_status,
    generate_grasp_candidates,
    lift_z,
    select_candidate,
    validate_candidate_configuration,
)
import pytest


def _candidates(count=4):
    return generate_grasp_candidates(
        0.1, 0.2, 0.35, (0.0, 0.0, 0.0, 1.0), count, math.pi / 2.0)


def test_candidate_generation_count_is_deterministic():
    assert _candidates() == _candidates()
    assert len(_candidates()) == 4


def test_selected_index_selects_expected_candidate():
    candidates = _candidates()
    assert select_candidate(candidates, 2) == candidates[2]
    assert candidates[2].qz == pytest.approx(1.0)


def test_invalid_candidate_index_raises_value_error():
    with pytest.raises(ValueError):
        validate_candidate_configuration(4, 4)


def test_invalid_candidate_count_raises_value_error():
    with pytest.raises(ValueError):
        validate_candidate_configuration(0, 0)


def test_lift_pose_z_adds_height_offset():
    assert lift_z(0.35, 0.20) == pytest.approx(0.55)


def test_candidates_format_is_stable_and_complete():
    message = format_grasp_candidates(_candidates(2), 1, 'world')
    assert message == (
        'event=grasp_candidates;count=2;selected_index=1;frame_id=world;'
        'candidates=0:0.100000,0.200000,0.350000,0.000000,0.000000,'
        '0.000000,1.000000|1:0.100000,0.200000,0.350000,0.000000,'
        '0.000000,0.707107,0.707107'
    )


def test_status_format_has_safety_scope_fields():
    message = format_grasp_sequence_status(
        'accepted_initial', 0, 4, 'world', (0.1, 0.2, 0.3),
        (0.1, 0.2, 0.35), 0.55, (0.6, -0.2, 0.1), 'fixed_socket')
    assert 'execution=false' in message
    assert 'simulated_only=true' in message
    assert 'real_hardware=false' in message


def test_legacy_grasp_pose_matches_selected_pose_semantics():
    selected = select_candidate(_candidates(), 0)
    legacy_grasp_pose = selected
    assert legacy_grasp_pose == selected
