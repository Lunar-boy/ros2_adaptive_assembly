#!/usr/bin/env python3
"""Deterministic helper-level dry run for PR67 grasp verification."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/adaptive_assembly_execution'))

from adaptive_assembly_execution.grasp_verifier_node import (  # noqa: E402
    GraspVerifierState,
    ObjectPosition,
    evaluate_grasp_request,
    evaluate_lift_request,
)


def _expect(condition: bool, label: str, failures: list[str]) -> None:
    if not condition:
        failures.append(label)


def main() -> int:
    failures: list[str] = []
    baseline = ObjectPosition(0.35, 0.18, 0.10)

    state = GraspVerifierState(
        gripper_success=True,
        gripper_closed=True,
        both_contacts=False,
        object_pose_available=True,
    )
    ok, reason, stored = evaluate_grasp_request(
        state, baseline, True, True, True, False
    )
    _expect(not ok and reason == 'missing_both_contacts' and stored is None,
            'grasp should fail without both contacts', failures)

    state.both_contacts = True
    state.left_contact = True
    state.right_contact = True
    ok, reason, stored = evaluate_grasp_request(
        state, baseline, True, True, True, False
    )
    _expect(ok and reason == '' and stored == baseline,
            'grasp should succeed with contacts, closure, and pose', failures)

    ok, reason, lift_delta, slip_distance = evaluate_lift_request(
        baseline, ObjectPosition(0.35, 0.18, 0.11),
        True, False, 0.02, 0.025
    )
    _expect(
        not ok and reason == 'insufficient_lift'
        and lift_delta is not None and slip_distance is not None,
        'lift should fail with insufficient z delta',
        failures,
    )

    ok, reason, lift_delta, slip_distance = evaluate_lift_request(
        baseline, ObjectPosition(0.39, 0.18, 0.13),
        True, False, 0.02, 0.025
    )
    _expect(
        not ok and reason == 'slip_too_large'
        and lift_delta is not None and slip_distance is not None,
        'lift should fail with excessive slip',
        failures,
    )

    ok, reason, lift_delta, slip_distance = evaluate_lift_request(
        baseline, ObjectPosition(0.355, 0.18, 0.13),
        True, False, 0.02, 0.025
    )
    _expect(
        ok and reason == '' and lift_delta is not None
        and lift_delta >= 0.02 and slip_distance is not None
        and slip_distance <= 0.025,
        'lift should succeed with sufficient z delta and bounded slip',
        failures,
    )

    if failures:
        print('FAIL grasp verifier dry run')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS grasp verifier dry run')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
