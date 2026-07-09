#!/usr/bin/env python3
"""Static checks for PR67 Gazebo contact sensor plumbing."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'src/adaptive_assembly_sim'
WORKCELL = SIM / 'worlds/adaptive_assembly_workcell.sdf'
PHYSICAL_WORKCELL = SIM / 'worlds/adaptive_assembly_physical_workcell.sdf'
XACRO = SIM / 'urdf/panda_gazebo_ros2_control.urdf.xacro'


def _read(path: Path) -> str:
    return path.read_text(encoding='utf-8') if path.exists() else ''


def main() -> int:
    failures = []
    for path in (WORKCELL, PHYSICAL_WORKCELL, XACRO):
        if not path.exists():
            failures.append(f'missing file: {path.relative_to(ROOT)}')

    physical_text = _read(PHYSICAL_WORKCELL)
    xacro_text = _read(XACRO)
    try:
        physical_root = ET.fromstring(physical_text)
    except ET.ParseError as error:
        failures.append(f'physical workcell XML parse failed: {error}')
        physical_root = None

    required_world_tokens = [
        'gz-sim-contact-system',
        'gz::sim::systems::Contact',
        '<world name="adaptive_assembly_physical_workcell">',
        '<model name="target_object">',
        '<static>false</static>',
    ]
    for token in required_world_tokens:
        if token not in physical_text:
            failures.append(f'missing physical world token: {token}')

    if physical_root is not None:
        target = physical_root.find(".//model[@name='target_object']/static")
        if target is None or (target.text or '').strip() != 'false':
            failures.append('target_object is not dynamic in physical workcell')

    required_xacro_tokens = [
        'panda_leftfinger_collision',
        'panda_rightfinger_collision',
        'sensor name="panda_leftfinger_contact_sensor"',
        'sensor name="panda_rightfinger_contact_sensor"',
        '<topic>/panda_leftfinger_contact</topic>',
        '<topic>/panda_rightfinger_contact</topic>',
        '<collision>panda_leftfinger_collision</collision>',
        '<collision>panda_rightfinger_collision</collision>',
    ]
    for token in required_xacro_tokens:
        if token not in xacro_text:
            failures.append(f'missing contact sensor token: {token}')

    if failures:
        print('FAIL Gazebo contact sensor static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS Gazebo contact sensor static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
