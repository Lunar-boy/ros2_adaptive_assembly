"""Integration test for the canonical Panda collision-mesh calibration."""

import math
import re
import subprocess


def _fields(line):
    return dict(
        fragment.split('=', 1)
        for fragment in line.split(';')
        if '=' in fragment
    )


def test_actual_panda_model_selects_smallest_clear_candidate():
    """Exercise IK, Panda STL collision meshes, cylinder, and finger-only ACM."""
    completed = subprocess.run(
        [
            'ros2', 'launch', 'adaptive_assembly_planning',
            'grasp_clearance_calibration.launch.py',
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=45.0,
    )
    output = completed.stdout + completed.stderr
    assert completed.returncode == 0, output
    candidate_lines = [
        line[line.index('event=candidate;'):]
        for line in output.splitlines()
        if 'event=candidate;' in line
    ]
    assert len(candidate_lines) == 26, output
    candidates = [_fields(line) for line in candidate_lines]
    by_offset = {round(float(item['offset']), 3): item for item in candidates}
    assert by_offset[0.005]['nearest_disallowed_link'] == 'panda_hand'
    assert by_offset[0.005]['state_valid'] == 'false'
    assert by_offset[0.005]['valid'] == 'false'
    assert 'panda_hand<->target_object' in by_offset[0.005]['collision_pairs']
    assert by_offset[0.017]['reason'] == 'grasp_clearance_below_minimum'
    assert by_offset[0.018]['valid'] == 'true'
    assert by_offset[0.018]['state_valid'] == 'true'
    assert math.isfinite(float(by_offset[0.018]['minimum_target_to_robot_distance']))
    assert float(by_offset[0.018]['minimum_disallowed_clearance']) >= 0.005
    assert by_offset[0.018]['left_finger_target_geometry_valid'] == 'true'
    assert by_offset[0.018]['right_finger_target_geometry_valid'] == 'true'
    selected = re.search(r'event=selected;grasp_height_offset=([0-9.]+)', output)
    assert selected is not None, output
    assert float(selected.group(1)) == 0.018
