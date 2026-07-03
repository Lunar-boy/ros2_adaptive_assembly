#!/usr/bin/env python3
"""Check attachment offset math and launch wiring without a running ROS graph."""

import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / 'src/adaptive_assembly_manipulation' \
    / 'adaptive_assembly_manipulation/attachment_math.py'

spec = importlib.util.spec_from_file_location('attachment_math', MODULE)
attachment_math = importlib.util.module_from_spec(spec)
spec.loader.exec_module(attachment_math)
rotate = attachment_math.rotate_vector_by_quaternion


def add(position, offset, quaternion):
    rotated = rotate(offset, quaternion)
    return tuple(a + b for a, b in zip(position, rotated))


hand = (1.0, 2.0, 3.0)
identity = (0.0, 0.0, 0.0, 1.0)
assert add(hand, (0.0, 0.0, 0.0), identity) == hand
assert add(hand, (0.0, 0.0, 0.1), identity) == (1.0, 2.0, 3.1)
quarter_turn_z = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
rotated = rotate((1.0, 0.0, 0.0), quarter_turn_z)
assert all(math.isclose(a, b, abs_tol=1e-12)
           for a, b in zip(rotated, (0.0, 1.0, 0.0)))

attachment_launch = (ROOT / 'src/adaptive_assembly_manipulation/launch'
                     / 'gazebo_attach_detach.launch.py').read_text()
visual_launch = (ROOT / 'src/adaptive_assembly_bringup/launch'
                 / 'adaptive_assembly_full_episode_visual_demo.launch.py').read_text()
for name in ('attached_object_offset_x', 'attached_object_offset_y',
             'attached_object_offset_z',
             'attached_object_use_hand_orientation'):
    assert name in attachment_launch, f'missing launch argument: {name}'
assert "'attached_object_offset_z': '0.10'" in visual_launch
print('PASS: Gazebo attachment offset math and launch wiring')
