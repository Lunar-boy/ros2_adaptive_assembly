#!/usr/bin/env python3
"""Validate gripper action bridge source and package registration."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/adaptive_assembly_manipulation'
SOURCE = PACKAGE / 'adaptive_assembly_manipulation/gripper_action_bridge_node.py'


def main() -> int:
    failures = []
    if not SOURCE.is_file():
        failures.append(f'missing {SOURCE.relative_to(ROOT)}')
        source_text = ''
    else:
        source_text = SOURCE.read_text(encoding='utf-8')
    required_source = {
        'FollowJointTrajectory import': 'from control_msgs.action import FollowJointTrajectory',
        'controller action default': '/panda_gripper_controller/follow_joint_trajectory',
        'canonical primary finger joint': 'panda_finger_joint1',
    }
    for description, text in required_source.items():
        if text not in source_text:
            failures.append(f'bridge source lacks {description}')

    setup_text = (PACKAGE / 'setup.py').read_text(encoding='utf-8')
    if 'gripper_action_bridge_node:main' not in setup_text:
        failures.append('setup.py does not register gripper_action_bridge_node')
    package_text = (PACKAGE / 'package.xml').read_text(encoding='utf-8')
    for dependency in ('control_msgs', 'trajectory_msgs'):
        if f'<depend>{dependency}</depend>' not in package_text:
            failures.append(f'package.xml lacks {dependency} dependency')

    if failures:
        print('FAIL: gripper action bridge static validation failed')
        for failure in failures:
            print(f'  - {failure}')
        return 1
    print('PASS: gripper action bridge source and package interfaces are present')
    return 0


if __name__ == '__main__':
    sys.exit(main())
