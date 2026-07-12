#!/usr/bin/env python3
"""Validate physical Gazebo workcell boxes against PlanningScene parameters."""

import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import yaml


TOLERANCE = 1e-6
ROOT = Path(__file__).resolve().parents[1]
PHYSICAL_WORLD = (
    ROOT / 'src/adaptive_assembly_sim/worlds'
    / 'adaptive_assembly_physical_workcell.sdf'
)
PHYSICAL_PARAMETERS = (
    ROOT / 'src/adaptive_assembly_bringup/config'
    / 'physical_workcell_planning_scene.yaml'
)

SDF_OBJECTS = {
    'work_table': ('work_table', 'work_surface_link', 'work_surface_collision'),
    'target_support': (
        'target_support', 'target_support_link', 'target_support_collision'
    ),
    'assembly_socket_base': (
        'assembly_socket_fixture', 'socket_base_link', 'socket_base_collision'
    ),
    'assembly_socket_left_wall': (
        'assembly_socket_fixture', 'socket_left_wall_link',
        'socket_left_wall_collision'
    ),
    'assembly_socket_right_wall': (
        'assembly_socket_fixture', 'socket_right_wall_link',
        'socket_right_wall_collision'
    ),
    'assembly_socket_back_wall': (
        'assembly_socket_fixture', 'socket_back_wall_link',
        'socket_back_wall_collision'
    ),
    'assembly_socket_front_wall': (
        'assembly_socket_fixture', 'socket_front_wall_link',
        'socket_front_wall_collision'
    ),
}


def fail(message: str) -> None:
    print(f'FAIL: {message}')
    raise SystemExit(1)


def parse_pose(element):
    """Return an SDF pose as translation and rotation matrix."""
    text = element.findtext('pose', default='0 0 0 0 0 0')
    values = [float(value) for value in text.split()]
    if len(values) != 6:
        fail(f'expected six pose values for {element.tag}, got {text!r}')
    x, y, z, roll, pitch, yaw = values
    cosine_roll, sine_roll = math.cos(roll), math.sin(roll)
    cosine_pitch, sine_pitch = math.cos(pitch), math.sin(pitch)
    cosine_yaw, sine_yaw = math.cos(yaw), math.sin(yaw)
    rotation = (
        (
            cosine_yaw * cosine_pitch,
            cosine_yaw * sine_pitch * sine_roll - sine_yaw * cosine_roll,
            cosine_yaw * sine_pitch * cosine_roll + sine_yaw * sine_roll,
        ),
        (
            sine_yaw * cosine_pitch,
            sine_yaw * sine_pitch * sine_roll + cosine_yaw * cosine_roll,
            sine_yaw * sine_pitch * cosine_roll - cosine_yaw * sine_roll,
        ),
        (-sine_pitch, cosine_pitch * sine_roll, cosine_pitch * cosine_roll),
    )
    return (x, y, z), rotation


def transform_point(rotation, point):
    return tuple(
        sum(rotation[row][column] * point[column] for column in range(3))
        for row in range(3)
    )


def multiply_rotation(left, right):
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def compose(parent, child):
    """Compose parent and child SDF poses, including any local rotations."""
    parent_translation, parent_rotation = parent
    child_translation, child_rotation = child
    rotated_translation = transform_point(parent_rotation, child_translation)
    translation = tuple(
        parent_translation[index] + rotated_translation[index]
        for index in range(3)
    )
    return translation, multiply_rotation(parent_rotation, child_rotation)


def sdf_geometry(root, model_name, link_name, collision_name):
    """Read a collision box center and size in the physical SDF world frame."""
    model = root.find(f".//model[@name='{model_name}']")
    if model is None:
        fail(f'physical SDF model is missing: {model_name}')
    link = model.find(f"link[@name='{link_name}']")
    if link is None:
        fail(f'physical SDF link is missing: {model_name}/{link_name}')
    collision = link.find(f"collision[@name='{collision_name}']")
    if collision is None:
        fail(f'physical SDF collision is missing: {collision_name}')
    size_text = collision.findtext('geometry/box/size')
    if size_text is None:
        fail(f'physical SDF collision is not a box: {collision_name}')
    size = tuple(float(value) for value in size_text.split())
    if len(size) != 3:
        fail(f'physical SDF box has invalid size: {collision_name}')
    center, _ = compose(compose(parse_pose(model), parse_pose(link)),
                        parse_pose(collision))
    return center, size


def planning_scene_geometry(parameters):
    """Evaluate static_planning_scene_node's parameterized box formulas."""
    required_flags = ('add_work_table', 'add_target_support', 'add_socket_fixture')
    for name in required_flags:
        if parameters.get(name) is not True:
            fail(f'physical PlanningScene must enable {name}')

    def vector(*names):
        try:
            return tuple(float(parameters[name]) for name in names)
        except KeyError as error:
            fail(f'physical PlanningScene parameter is missing: {error.args[0]}')

    socket_x, socket_y, socket_z = vector('socket_x', 'socket_y', 'socket_z')
    base_z = float(parameters['socket_base_center_z_offset'])
    wall_z = float(parameters['socket_wall_center_z_offset'])
    side_offset = float(parameters['socket_side_wall_y_offset'])
    end_offset = float(parameters['socket_back_front_wall_x_offset'])
    wall_height = float(parameters['socket_wall_height'])
    wall_thickness = float(parameters['socket_wall_thickness'])
    side_length = float(parameters['socket_side_wall_length'])
    end_length = float(parameters['socket_back_front_wall_length'])
    return {
        'work_table': (
            vector('table_x', 'table_y', 'table_z'),
            vector('table_size_x', 'table_size_y', 'table_size_z'),
        ),
        'target_support': (
            vector('target_support_x', 'target_support_y', 'target_support_z'),
            vector(
                'target_support_size_x', 'target_support_size_y',
                'target_support_size_z'
            ),
        ),
        'assembly_socket_base': (
            (socket_x, socket_y, socket_z + base_z),
            vector(
                'socket_base_size_x', 'socket_base_size_y',
                'socket_base_size_z'
            ),
        ),
        'assembly_socket_left_wall': (
            (socket_x, socket_y + side_offset, socket_z + wall_z),
            (side_length, wall_thickness, wall_height),
        ),
        'assembly_socket_right_wall': (
            (socket_x, socket_y - side_offset, socket_z + wall_z),
            (side_length, wall_thickness, wall_height),
        ),
        'assembly_socket_back_wall': (
            (socket_x - end_offset, socket_y, socket_z + wall_z),
            (wall_thickness, end_length, wall_height),
        ),
        'assembly_socket_front_wall': (
            (socket_x + end_offset, socket_y, socket_z + wall_z),
            (wall_thickness, end_length, wall_height),
        ),
    }


def close_enough(actual, expected):
    return all(
        math.isclose(value, reference, abs_tol=TOLERANCE)
        for value, reference in zip(actual, expected)
    )


def main() -> int:
    if not PHYSICAL_WORLD.is_file():
        fail(f'physical SDF is missing: {PHYSICAL_WORLD}')
    if not PHYSICAL_PARAMETERS.is_file():
        fail(f'physical PlanningScene YAML is missing: {PHYSICAL_PARAMETERS}')

    profile = yaml.safe_load(PHYSICAL_PARAMETERS.read_text(encoding='utf-8'))
    try:
        parameters = profile['static_planning_scene_node']['ros__parameters']
    except (KeyError, TypeError):
        fail('physical PlanningScene YAML has no static_planning_scene_node parameters')
    if parameters.get('planning_frame') != 'panda_link0':
        fail('physical PlanningScene frame must be panda_link0')

    root = ElementTree.parse(PHYSICAL_WORLD).getroot()
    expected = {
        object_id: sdf_geometry(root, *sdf_path)
        for object_id, sdf_path in SDF_OBJECTS.items()
    }
    actual = planning_scene_geometry(parameters)
    for object_id in SDF_OBJECTS:
        center, dimensions = actual[object_id]
        sdf_center, sdf_dimensions = expected[object_id]
        if not close_enough(center, sdf_center):
            fail(
                f'{object_id} center differs from SDF: '
                f'{center} != {sdf_center} (tolerance {TOLERANCE:g})'
            )
        if not close_enough(dimensions, sdf_dimensions):
            fail(
                f'{object_id} dimensions differ from SDF: '
                f'{dimensions} != {sdf_dimensions} (tolerance {TOLERANCE:g})'
            )

    print(
        'PASS: physical PlanningScene geometry matches the Gazebo SDF for '
        f'{len(SDF_OBJECTS)} objects (tolerance {TOLERANCE:g})'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
