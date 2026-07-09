#!/usr/bin/env python3
"""Static checks for PR67 executor grasp/lift verification integration."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT / 'src/adaptive_assembly_execution/adaptive_assembly_execution'
    / 'physical_pick_place_executor_node.py'
)


def main() -> int:
    failures = []
    source = SOURCE.read_text(encoding='utf-8') if SOURCE.exists() else ''
    if not SOURCE.exists():
        failures.append('physical_pick_place_executor_node.py is missing')

    if 'reason=pr67_out_of_scope' in source:
        failures.append('old pr67_out_of_scope hook is still present')

    required_tokens = [
        'require_grasp_verification',
        'require_lift_verification',
        'grasp_verification_request_topic',
        'grasp_verification_status_topic',
        'grasp_verified_topic',
        'lift_verified_topic',
        'verification_timeout_sec',
        'event=request;verification={verification}',
        "_request_verification(stage, 'grasp')",
        "_request_verification(stage, 'lift')",
        'grasp_verification_failed',
        'grasp_verification_timeout',
        'lift_verification_failed',
        'lift_verification_timeout',
        'require_grasp_verification_false',
        'require_lift_verification_false',
        'WAIT_VERIFICATION_RESULT',
        'mode=physical_pick_place',
        'simulated=true',
        'real_hardware=false',
    ]
    for token in required_tokens:
        if token not in source:
            failures.append(f'missing executor verification token: {token}')

    if failures:
        print('FAIL physical pick-place executor verification static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS physical pick-place executor verification static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
