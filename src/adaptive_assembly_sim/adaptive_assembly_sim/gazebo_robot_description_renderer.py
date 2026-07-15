"""Render the canonical Panda URDF for independent Gazebo finger control."""

from __future__ import annotations

import argparse
import sys
from typing import Mapping, Sequence
import xml.etree.ElementTree as ElementTree

import xacro


SECOND_FINGER_JOINT = 'panda_finger_joint2'
MIMIC_SOURCE_JOINT = 'panda_finger_joint1'


class GazeboRobotDescriptionError(RuntimeError):
    """Raised when the Gazebo-only Panda transformation is unsafe."""


def remove_second_finger_mimic(xml_text: str) -> str:
    """Remove only the expected Panda second-finger mimic element."""
    try:
        root = ElementTree.fromstring(xml_text)
    except ElementTree.ParseError as error:
        raise GazeboRobotDescriptionError(
            f'failed to parse expanded Panda URDF: {error}'
        ) from error

    joints = root.findall(f"./joint[@name='{SECOND_FINGER_JOINT}']")
    if not joints:
        raise GazeboRobotDescriptionError(
            f'missing required joint {SECOND_FINGER_JOINT!r}'
        )
    if len(joints) != 1:
        raise GazeboRobotDescriptionError(
            f'found {len(joints)} joints named {SECOND_FINGER_JOINT!r}; expected 1'
        )

    joint = joints[0]
    mimics = joint.findall('mimic')
    if not mimics:
        raise GazeboRobotDescriptionError(
            f'{SECOND_FINGER_JOINT!r} is missing its expected mimic relation'
        )
    if len(mimics) != 1:
        raise GazeboRobotDescriptionError(
            f'{SECOND_FINGER_JOINT!r} has {len(mimics)} mimic elements; expected 1'
        )
    mimic = mimics[0]
    if mimic.get('joint') != MIMIC_SOURCE_JOINT:
        raise GazeboRobotDescriptionError(
            f'{SECOND_FINGER_JOINT!r} mimics {mimic.get("joint")!r}; '
            f'expected {MIMIC_SOURCE_JOINT!r}'
        )
    joint.remove(mimic)

    try:
        return ElementTree.tostring(
            root, encoding='unicode', short_empty_elements=True
        )
    except (TypeError, ValueError) as error:
        raise GazeboRobotDescriptionError(
            f'failed to serialize Gazebo Panda URDF: {error}'
        ) from error


def render_gazebo_robot_description(
    xacro_path: str,
    mappings: Mapping[str, str] | None = None,
) -> str:
    """Expand one xacro and apply the narrowly scoped Gazebo transformation."""
    try:
        document = xacro.process_file(xacro_path, mappings=dict(mappings or {}))
        expanded = document.toxml()
    except Exception as error:
        raise GazeboRobotDescriptionError(
            f'failed to expand Panda xacro {xacro_path!r}: {error}'
        ) from error
    return remove_second_finger_mimic(expanded)


def main(argv: Sequence[str] | None = None) -> int:
    """Render a Gazebo Panda description to stdout for diagnostics."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('xacro_path')
    parser.add_argument(
        'mappings', nargs='*', metavar='NAME:=VALUE',
        help='xacro mappings passed to the normal installed xacro processor',
    )
    arguments = parser.parse_args(argv)
    mappings: dict[str, str] = {}
    for item in arguments.mappings:
        if ':=' not in item:
            parser.error(f'invalid mapping {item!r}; expected NAME:=VALUE')
        name, value = item.split(':=', 1)
        if not name or name in mappings:
            parser.error(f'invalid or duplicate mapping name in {item!r}')
        mappings[name] = value
    try:
        rendered = render_gazebo_robot_description(
            arguments.xacro_path, mappings
        )
    except GazeboRobotDescriptionError as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1
    sys.stdout.write(rendered)
    if not rendered.endswith('\n'):
        sys.stdout.write('\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
