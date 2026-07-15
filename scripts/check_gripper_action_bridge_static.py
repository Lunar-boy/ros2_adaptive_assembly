#!/usr/bin/env python3
"""Validate the simulator gripper bridge's two-joint goal contract."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/adaptive_assembly_manipulation'))

from adaptive_assembly_manipulation.gripper_action_bridge_node import (  # noqa: E402
    make_gripper_goal,
    PANDA_FINGER_JOINTS,
    validate_simulator_joint_names,
)


def main() -> int:
    failures = []
    expected = ['panda_finger_joint1', 'panda_finger_joint2']
    try:
        names = validate_simulator_joint_names(PANDA_FINGER_JOINTS)
        for command, position, expected_positions in (
            ('open', 0.04, [0.04, 0.04]),
            ('close', 0.0, [0.0, 0.0]),
        ):
            goal = make_gripper_goal(names, position, 1.0)
            if list(goal.trajectory.joint_names) != expected:
                failures.append(f'{command} goal has wrong joint names')
            if len(goal.trajectory.points) != 1:
                failures.append(f'{command} goal must have one point')
            elif list(goal.trajectory.points[0].positions) != expected_positions:
                failures.append(f'{command} goal has wrong positions')
        for invalid in (
            [],
            ['panda_finger_joint1'],
            ['panda_finger_joint1', 'panda_finger_joint1'],
            ['panda_finger_joint2', 'panda_finger_joint1'],
            ['panda_finger_joint1', 'unexpected'],
        ):
            try:
                validate_simulator_joint_names(invalid)
            except ValueError:
                continue
            failures.append(f'invalid joint list was accepted: {invalid!r}')
    except Exception as error:
        failures.append(f'bridge contract check raised: {error}')

    if failures:
        print('FAIL: gripper action bridge contract validation failed')
        for failure in failures:
            print(f'  - {failure}')
        return 1
    print(
        'PASS: gripper bridge open and close goals command both Panda '
        'finger joints with equal positions'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
