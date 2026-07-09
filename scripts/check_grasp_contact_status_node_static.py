#!/usr/bin/env python3
"""Static checks for the PR67 Gazebo contact status node."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT / 'src/adaptive_assembly_execution/adaptive_assembly_execution'
    / 'gazebo_grasp_contact_status_node.py'
)
SETUP = ROOT / 'src/adaptive_assembly_execution/setup.py'


def main() -> int:
    failures = []
    source = SOURCE.read_text(encoding='utf-8') if SOURCE.exists() else ''
    setup = SETUP.read_text(encoding='utf-8') if SETUP.exists() else ''
    if not SOURCE.exists():
        failures.append('gazebo_grasp_contact_status_node.py is missing')
    if 'gazebo_grasp_contact_status_node = ' not in setup:
        failures.append('setup.py does not register contact status node')

    required_tokens = [
        'left_contact_topic',
        'right_contact_topic',
        'target_object_name',
        'left_contact_status_topic',
        'right_contact_status_topic',
        'aggregate_contact_status_topic',
        'left_contact_detected_topic',
        'right_contact_detected_topic',
        'both_contacts_detected_topic',
        'contact_stale_timeout_sec',
        'publish_period_sec',
        'require_target_object_contact',
        'simulated_only',
        '/panda_leftfinger_contact',
        '/panda_rightfinger_contact',
        '/grasp_contact_status',
        '/both_gripper_contacts_detected',
        "MODE = 'gazebo_grasp_contact_status'",
        'left_contact_stale',
        'right_contact_stale',
        'no_left_contact',
        'no_right_contact',
        'no_target_object_contact',
        'unsupported_contact_message',
        'simulated_only_false',
        'simulated_only=true',
        'real_hardware=false',
        'ros_gz_interfaces.msg',
        'Contacts',
    ]
    for token in required_tokens:
        if token not in source:
            failures.append(f'missing contact status token: {token}')

    if failures:
        print('FAIL grasp contact status node static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS grasp contact status node static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
