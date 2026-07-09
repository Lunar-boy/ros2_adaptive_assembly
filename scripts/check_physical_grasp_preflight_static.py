#!/usr/bin/env python3
"""Static checks for physical grasp preflight diagnostics."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT / 'src/adaptive_assembly_execution/adaptive_assembly_execution'
    / 'physical_grasp_preflight_node.py'
)
SETUP = ROOT / 'src/adaptive_assembly_execution/setup.py'
WRAPPER = (
    ROOT / 'src/adaptive_assembly_bringup/launch'
    / 'adaptive_assembly_full_physical_pick_place_demo.launch.py'
)


def main() -> int:
    failures = []
    source = SOURCE.read_text(encoding='utf-8') if SOURCE.exists() else ''
    setup = SETUP.read_text(encoding='utf-8') if SETUP.exists() else ''
    wrapper = WRAPPER.read_text(encoding='utf-8') if WRAPPER.exists() else ''

    if not SOURCE.exists():
        failures.append('physical_grasp_preflight_node.py is missing')
    if 'physical_grasp_preflight_node = ' not in setup:
        failures.append('setup.py does not register physical_grasp_preflight_node')
    if not WRAPPER.exists():
        failures.append('full physical pick-place wrapper launch is missing')

    required_source_tokens = [
        '/physical_grasp_preflight_status',
        '/world/adaptive_assembly_physical_workcell/pose/info',
        '/world/adaptive_assembly_workcell/pose/info',
        '/gazebo_target_object_pose_available',
        '/gazebo_attach_detach_status',
        '/panda_leftfinger_contact',
        '/panda_rightfinger_contact',
        '/grasp_contact_status',
        'kinematic_attach_node_active',
        'left_contact_topic_unobserved',
        'right_contact_topic_unobserved',
        'contact_status_topic_unobserved',
        'event={event};mode={MODE}',
        'simulated_only=true;real_hardware=false',
    ]
    for token in required_source_tokens:
        if token not in source:
            failures.append(f'missing preflight token: {token}')

    required_wrapper_tokens = [
        'adaptive_assembly_physical_workcell.sdf',
        'adaptive_assembly_physical_workcell',
        'enable_arm_collisions',
        "default_value='true'",
        'adaptive_assembly_physical_pick_place_execution.launch.py',
    ]
    for token in required_wrapper_tokens:
        if token not in wrapper:
            failures.append(f'missing wrapper token: {token}')

    forbidden = [
        'real_hardware=true',
        'hardware_driver',
        'MoveItServo',
    ]
    for token in forbidden:
        if token in source or token in wrapper:
            failures.append(f'forbidden physical preflight token present: {token}')

    if failures:
        print('FAIL physical grasp preflight static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS physical grasp preflight static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
