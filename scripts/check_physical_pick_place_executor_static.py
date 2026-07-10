#!/usr/bin/env python3
"""Static checks for the PR66 physical pick-place executor."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT / 'src/adaptive_assembly_execution/adaptive_assembly_execution'
    / 'physical_pick_place_executor_node.py'
)
SETUP = ROOT / 'src/adaptive_assembly_execution/setup.py'


def main() -> int:
    failures = []
    if not SOURCE.exists():
        failures.append('physical_pick_place_executor_node.py is missing')
        source_text = ''
    else:
        source_text = SOURCE.read_text(encoding='utf-8')

    setup_text = SETUP.read_text(encoding='utf-8')
    if 'physical_pick_place_executor_node = ' not in setup_text:
        failures.append('setup.py does not register physical_pick_place_executor_node')

    required_tokens = [
        '/pre_grasp_trajectory',
        '/grasp_trajectory',
        '/lift_trajectory',
        '/pre_place_trajectory',
        '/place_trajectory',
        '/retreat_trajectory',
        '/gripper_command',
        '/physical_gripper_command_status',
        '/physical_pick_place_execution_status',
        'mode=physical_pick_place',
        'real_hardware=false',
        'simulated_execution_only',
        'raise ValueError',
        'require_grasp_verification',
        'require_lift_verification',
        'require_physical_grasp_preflight',
        'physical_grasp_preflight_status_topic',
        'physical_grasp_preflight_timeout_sec',
        '/physical_grasp_preflight_status',
        'physical_grasp_preflight_failed',
        'preflight_reason=',
        'physical_grasp_preflight_timeout',
        'verification_skipped',
    ]
    for token in required_tokens:
        if token not in source_text:
            failures.append(f'missing executor token: {token}')

    forbidden_tokens = [
        'force_control',
        'tactile',
        'MoveItServo',
        'real_hardware=true',
        'hardware_interface',
    ]
    for token in forbidden_tokens:
        if token in source_text:
            failures.append(f'forbidden PR66 executor token present: {token}')

    if 'if not self._simulated_execution_only' not in source_text:
        failures.append('simulated_execution_only=false rejection is missing')

    if failures:
        print('FAIL physical pick-place executor static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS physical pick-place executor static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
