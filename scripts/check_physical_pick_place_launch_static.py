#!/usr/bin/env python3
"""Static checks for the PR66 bringup launch file."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = (
    ROOT / 'src/adaptive_assembly_bringup/launch'
    / 'adaptive_assembly_physical_pick_place_execution.launch.py'
)


def main() -> int:
    failures = []
    if not LAUNCH.exists():
        failures.append('physical pick-place launch file is missing')
        text = ''
    else:
        text = LAUNCH.read_text(encoding='utf-8')

    required = [
        'physical_pick_place_executor_node',
        'gripper_action_bridge_node',
        'stage_names',
        'lift_trajectory_topic',
        'send_gripper_commands',
        'simulated_execution_only',
        'launch_reachable_sequence',
        'launch_gripper_bridge',
    ]
    for token in required:
        if token not in text:
            failures.append(f'missing launch token: {token}')

    forbidden = [
        'enable_real_hardware',
        'real_hardware:=true',
        'hardware_driver',
        'camera',
        'contact_sensor',
    ]
    for token in forbidden:
        if token in text:
            failures.append(f'forbidden launch token present: {token}')

    if failures:
        print('FAIL physical pick-place launch static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS physical pick-place launch static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
