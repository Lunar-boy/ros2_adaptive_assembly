#!/usr/bin/env python3
"""Static checks for the PR67 grasp verifier node."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT / 'src/adaptive_assembly_execution/adaptive_assembly_execution'
    / 'grasp_verifier_node.py'
)
SETUP = ROOT / 'src/adaptive_assembly_execution/setup.py'


def main() -> int:
    failures = []
    source = SOURCE.read_text(encoding='utf-8') if SOURCE.exists() else ''
    setup = SETUP.read_text(encoding='utf-8') if SETUP.exists() else ''
    if not SOURCE.exists():
        failures.append('grasp_verifier_node.py is missing')
    if 'grasp_verifier_node = ' not in setup:
        failures.append('setup.py does not register grasp_verifier_node')

    required_tokens = [
        'contact_status_topic',
        'both_contacts_detected_topic',
        'gripper_success_topic',
        'gripper_closed_topic',
        'object_pose_topic',
        'object_pose_available_topic',
        'verifier_request_topic',
        'verifier_status_topic',
        'grasp_verified_topic',
        'lift_verified_topic',
        'slip_distance_mm_topic',
        'require_both_contacts',
        'require_gripper_closed',
        'require_object_pose',
        'min_lift_delta_m',
        'max_slip_distance_m',
        'pose_stale_timeout_sec',
        "MODE = 'grasp_verifier'",
        'gripper_not_successful',
        'gripper_not_closed',
        'missing_both_contacts',
        'object_pose_unavailable',
        'object_pose_stale',
        'missing_grasp_baseline',
        'insufficient_lift',
        'slip_too_large',
        'unsupported_request',
        'simulated_only_false',
        'simulated_only=true',
        'real_hardware=false',
        'compute_lift_and_slip',
        'evaluate_grasp_request',
        'evaluate_lift_request',
    ]
    for token in required_tokens:
        if token not in source:
            failures.append(f'missing grasp verifier token: {token}')

    if failures:
        print('FAIL grasp verifier static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS grasp verifier static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
