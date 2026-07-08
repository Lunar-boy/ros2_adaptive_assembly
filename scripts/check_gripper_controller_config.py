#!/usr/bin/env python3
"""Validate the simulator-only Panda gripper controller configuration."""

from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/adaptive_assembly_sim/config/panda_ros2_control.yaml'
GRIPPER_JOINTS = {'panda_finger_joint1', 'panda_finger_joint2'}


def main() -> int:
    if not CONFIG.is_file():
        print(f'FAIL: missing {CONFIG.relative_to(ROOT)}')
        return 1
    try:
        data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        print(f'FAIL: could not parse controller configuration: {error}')
        return 1

    failures = []
    manager = data.get('controller_manager', {}).get('ros__parameters', {})
    expected_type = 'joint_trajectory_controller/JointTrajectoryController'
    gripper_manager = manager.get('panda_gripper_controller', {})
    if gripper_manager.get('type') != expected_type:
        failures.append('panda_gripper_controller has the wrong or missing type')
    arm_manager = manager.get('panda_arm_controller', {})
    if arm_manager.get('type') != expected_type:
        failures.append('panda_arm_controller was removed or has the wrong type')

    arm = data.get('panda_arm_controller', {}).get('ros__parameters', {})
    expected_arm_joints = {f'panda_joint{number}' for number in range(1, 8)}
    if set(arm.get('joints', [])) != expected_arm_joints:
        failures.append('panda_arm_controller joint list is missing or corrupt')

    gripper = data.get('panda_gripper_controller', {}).get(
        'ros__parameters', {}
    )
    if not GRIPPER_JOINTS.issubset(set(gripper.get('joints', []))):
        failures.append('gripper controller does not contain both finger joints')
    if 'position' not in gripper.get('command_interfaces', []):
        failures.append('gripper controller lacks the position command interface')
    state_interfaces = set(gripper.get('state_interfaces', []))
    if not {'position', 'velocity'}.issubset(state_interfaces):
        failures.append('gripper controller lacks position/velocity state interfaces')

    if failures:
        print('FAIL: gripper controller configuration validation failed')
        for failure in failures:
            print(f'  - {failure}')
        return 1
    print('PASS: Panda arm and gripper controller configuration is valid')
    return 0


if __name__ == '__main__':
    sys.exit(main())
