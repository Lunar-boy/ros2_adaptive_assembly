#!/usr/bin/env python3
"""Validate the simulator-only Panda gripper links and ros2_control joints."""

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'src/adaptive_assembly_sim/urdf/panda_gazebo_ros2_control.urdf.xacro'
CANONICAL_URDF = ROOT / 'src/adaptive_assembly_sim/urdf/panda.urdf.xacro'
REQUIRED_LINKS = ('panda_hand', 'panda_leftfinger', 'panda_rightfinger')
REQUIRED_JOINTS = ('panda_finger_joint1', 'panda_finger_joint2')
ARM_JOINTS = tuple(f'panda_joint{index}' for index in range(1, 8))

sys.path.insert(0, str(ROOT / 'src/adaptive_assembly_sim'))
from adaptive_assembly_sim.gazebo_robot_description_renderer import (  # noqa: E402
    GazeboRobotDescriptionError,
    render_gazebo_robot_description,
)


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
        expanded = render_gazebo_robot_description(str(URDF))
        root = ET.fromstring(expanded)
        canonical = subprocess.run(
            ['xacro', str(CANONICAL_URDF)], check=True,
            capture_output=True, text=True,
        ).stdout
        canonical_root = ET.fromstring(canonical)
    except (
        ET.ParseError, subprocess.CalledProcessError,
        GazeboRobotDescriptionError,
    ) as exc:
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

    for joint_name in REQUIRED_JOINTS:
        joint = find_named(joints, joint_name)
        if joint is not None and joint.find('mimic') is not None:
            failures.append(f'Gazebo-rendered {joint_name} contains mimic')

    canonical_mimic = canonical_root.find(
        "./joint[@name='panda_finger_joint2']/mimic"
    )
    if (
        canonical_mimic is None
        or canonical_mimic.get('joint') != 'panda_finger_joint1'
    ):
        failures.append(
            'canonical planning description lost the standard finger mimic'
        )

    systems = [
        item for item in root.findall('.//ros2_control')
        if item.get('name') == 'GazeboSystem'
    ]
    if len(systems) != 1:
        failures.append(
            f'expected exactly one GazeboSystem, found {len(systems)}'
        )
    ros2_control = systems[0] if len(systems) == 1 else None
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
            if command_names != ['position']:
                failures.append(
                    f'{joint_name} must expose only position command_interface'
                )
            for state_name in ('position', 'velocity'):
                if state_name not in state_names:
                    failures.append(
                        f'{joint_name} missing {state_name} state_interface'
                    )
            position_state = next(
                (item for item in control_joint.findall('state_interface')
                 if item.get('name') == 'position'), None
            )
            initial = (
                position_state.find("./param[@name='initial_value']")
                if position_state is not None else None
            )
            if initial is None or float(initial.text) != 0.04:
                failures.append(
                    f'{joint_name} initial position must be 0.04'
                )
        for joint_name in ARM_JOINTS:
            control_joint = find_named(control_joints, joint_name)
            if control_joint is None:
                failures.append(f'{joint_name} missing from GazeboSystem')
                continue
            command_names = [
                item.get('name')
                for item in control_joint.findall('command_interface')
            ]
            state_names = {
                item.get('name')
                for item in control_joint.findall('state_interface')
            }
            if command_names != ['position']:
                failures.append(f'{joint_name} command interfaces changed')
            if state_names != {'position', 'velocity'}:
                failures.append(f'{joint_name} state interfaces changed')

    if failures:
        print('FAIL: Panda gripper URDF validation failed')
        for failure in failures:
            print(f'  - {failure}')
        return 1

    print(
        'PASS: canonical planning mimic is preserved; Gazebo renders two '
        'independently position-commanded Panda fingers in one GazeboSystem'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
