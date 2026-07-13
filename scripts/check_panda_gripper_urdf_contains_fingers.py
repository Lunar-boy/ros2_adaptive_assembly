#!/usr/bin/env python3
"""Validate the simulator-only Panda gripper links and ros2_control joints."""

from pathlib import Path
import sys
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'src/adaptive_assembly_sim/urdf/panda_gazebo_ros2_control.urdf.xacro'
REQUIRED_LINKS = ('panda_hand', 'panda_leftfinger', 'panda_rightfinger')
REQUIRED_JOINTS = ('panda_finger_joint1', 'panda_finger_joint2')


def fail(message: str) -> int:
    print(f'FAIL: {message}')
    return 1


def attr_equals(element: ET.Element, name: str, expected: str) -> bool:
    return element.get(name) == expected


def find_named(elements: list[ET.Element], name: str) -> ET.Element | None:
    for element in elements:
        if element.get('name') == name:
            return element
    return None


def main() -> int:
    if not URDF.is_file():
        return fail(f'missing URDF/xacro: {URDF.relative_to(ROOT)}')

    try:
        expanded = subprocess.run(
            ['xacro', str(URDF)],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        root = ET.fromstring(expanded)
    except (ET.ParseError, subprocess.CalledProcessError) as exc:
        return fail(f'URDF/xacro expansion failed: {exc}')

    failures: list[str] = []
    links = root.findall('.//link')
    joints = root.findall('.//joint')

    for link_name in REQUIRED_LINKS:
        if find_named(links, link_name) is None:
            failures.append(f'missing link {link_name}')

    for joint_name in REQUIRED_JOINTS:
        joint = find_named(joints, joint_name)
        if joint is None:
            failures.append(f'missing joint {joint_name}')
            continue
        if not attr_equals(joint, 'type', 'prismatic'):
            failures.append(f'{joint_name} is not type="prismatic"')
        limit = joint.find('limit')
        if limit is None:
            failures.append(f'{joint_name} is missing a limit element')
            continue
        for attr in ('lower', 'upper', 'effort', 'velocity'):
            if attr not in limit.attrib:
                failures.append(f'{joint_name} limit is missing {attr}')

    mimic_joint = find_named(joints, 'panda_finger_joint2')
    mimic = mimic_joint.find('mimic') if mimic_joint is not None else None
    if mimic is None or mimic.get('joint') != 'panda_finger_joint1':
        failures.append(
            'panda_finger_joint2 does not mimic panda_finger_joint1'
        )

    ros2_control = find_named(root.findall('.//ros2_control'), 'GazeboSystem')
    if ros2_control is None:
        failures.append('missing ros2_control block named GazeboSystem')
    else:
        control_joints = ros2_control.findall('joint')
        for joint_name in REQUIRED_JOINTS:
            control_joint = find_named(control_joints, joint_name)
            if control_joint is None:
                failures.append(f'{joint_name} missing from ros2_control block')
                continue
            command_names = [
                item.get('name') for item in control_joint.findall('command_interface')
            ]
            state_names = [
                item.get('name') for item in control_joint.findall('state_interface')
            ]
            if (
                joint_name == 'panda_finger_joint1'
                and 'position' not in command_names
            ):
                failures.append(
                    f'{joint_name} missing position command_interface'
                )
            if joint_name == 'panda_finger_joint2' and command_names:
                failures.append(
                    'mimic panda_finger_joint2 must not expose command interfaces'
                )
            for state_name in ('position', 'velocity'):
                if state_name not in state_names:
                    failures.append(
                        f'{joint_name} missing {state_name} state_interface'
                    )

    if failures:
        print('FAIL: Panda gripper URDF validation failed')
        for failure in failures:
            print(f'  - {failure}')
        return 1

    print(
        'PASS: Panda gripper links, prismatic finger joints, and '
        'ros2_control interfaces are present'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
