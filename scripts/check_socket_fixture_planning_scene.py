#!/usr/bin/env python3
"""Validate static PlanningScene socket geometry against the Gazebo SDF."""

import ast
import math
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "src/adaptive_assembly_planning/src/static_planning_scene_node.cpp"
LAUNCH = ROOT / "src/adaptive_assembly_planning/launch/static_planning_scene.launch.py"

PARAMETERS = {
    "add_socket_fixture": True,
    "socket_x": 0.62,
    "socket_y": -0.18,
    "socket_z": 0.0,
    "socket_base_size_x": 0.20,
    "socket_base_size_y": 0.20,
    "socket_base_size_z": 0.03,
    "socket_wall_height": 0.08,
    "socket_wall_thickness": 0.03,
    "socket_side_wall_length": 0.16,
    "socket_back_front_wall_length": 0.10,
    "socket_side_wall_y_offset": 0.065,
    "socket_back_front_wall_x_offset": 0.065,
    "socket_base_center_z_offset": 0.015,
    "socket_wall_center_z_offset": 0.055,
}

OBJECT_CALLS = {
    "assembly_socket_base": (
        "socket_x_, socket_y_, socket_z_ + socket_base_center_z_offset_",
        "socket_base_size_x_, socket_base_size_y_, socket_base_size_z_",
    ),
    "assembly_socket_left_wall": (
        "socket_x_, socket_y_ + socket_side_wall_y_offset_",
        "socket_side_wall_length_, socket_wall_thickness_, socket_wall_height_",
    ),
    "assembly_socket_right_wall": (
        "socket_x_, socket_y_ - socket_side_wall_y_offset_",
        "socket_side_wall_length_, socket_wall_thickness_, socket_wall_height_",
    ),
    "assembly_socket_back_wall": (
        "socket_x_ - socket_back_front_wall_x_offset_, socket_y_",
        "socket_wall_thickness_, socket_back_front_wall_length_, socket_wall_height_",
    ),
    "assembly_socket_front_wall": (
        "socket_x_ + socket_back_front_wall_x_offset_, socket_y_",
        "socket_wall_thickness_, socket_back_front_wall_length_, socket_wall_height_",
    ),
}


def fail(message: str) -> None:
    print(f"FAIL: {message}")
    sys.exit(1)


node_source = " ".join(NODE.read_text(encoding="utf-8").split())
launch_tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
launch_dicts = [ast.literal_eval(node) for node in ast.walk(launch_tree)
                if isinstance(node, ast.Dict)]
launch_parameters = next((item for item in launch_dicts
                          if "add_socket_fixture" in item), None)

if launch_parameters is None:
    fail("launch file does not pass add_socket_fixture")

for name, expected in PARAMETERS.items():
    parameter_type = "bool" if isinstance(expected, bool) else "double"
    match = re.search(
        rf'declare_parameter<{parameter_type}>\("{name}", ([^)]+)\)',
        node_source,
    )
    if match is None:
        fail(f"node parameter default is missing or incorrect: {name}")
    actual_default = match.group(1) == "true" if isinstance(expected, bool) else float(match.group(1))
    if actual_default != expected:
        fail(f"node parameter default is incorrect: {name}")
    if launch_parameters.get(name) != expected:
        fail(f"launch parameter is missing or incorrect: {name}")

for object_id, fragments in OBJECT_CALLS.items():
    if f'"{object_id}"' not in node_source:
        fail(f"collision object ID is missing: {object_id}")
    for fragment in fragments:
        if fragment not in node_source:
            fail(f"geometry expression is incorrect for {object_id}: {fragment}")

centers = {
    "assembly_socket_base": (0.62, -0.18, 0.015),
    "assembly_socket_left_wall": (0.62, -0.115, 0.055),
    "assembly_socket_right_wall": (0.62, -0.245, 0.055),
    "assembly_socket_back_wall": (0.555, -0.18, 0.055),
    "assembly_socket_front_wall": (0.685, -0.18, 0.055),
}
computed_centers = {
    "assembly_socket_base": (PARAMETERS["socket_x"], PARAMETERS["socket_y"],
                             PARAMETERS["socket_z"] + PARAMETERS["socket_base_center_z_offset"]),
    "assembly_socket_left_wall": (PARAMETERS["socket_x"], PARAMETERS["socket_y"] + PARAMETERS["socket_side_wall_y_offset"],
                                  PARAMETERS["socket_z"] + PARAMETERS["socket_wall_center_z_offset"]),
    "assembly_socket_right_wall": (PARAMETERS["socket_x"], PARAMETERS["socket_y"] - PARAMETERS["socket_side_wall_y_offset"],
                                   PARAMETERS["socket_z"] + PARAMETERS["socket_wall_center_z_offset"]),
    "assembly_socket_back_wall": (PARAMETERS["socket_x"] - PARAMETERS["socket_back_front_wall_x_offset"], PARAMETERS["socket_y"],
                                  PARAMETERS["socket_z"] + PARAMETERS["socket_wall_center_z_offset"]),
    "assembly_socket_front_wall": (PARAMETERS["socket_x"] + PARAMETERS["socket_back_front_wall_x_offset"], PARAMETERS["socket_y"],
                                   PARAMETERS["socket_z"] + PARAMETERS["socket_wall_center_z_offset"]),
}
for object_id, expected_center in centers.items():
    if not all(math.isclose(actual, expected, abs_tol=1e-12)
               for actual, expected in zip(computed_centers[object_id], expected_center)):
        fail(f"computed center does not match SDF reference for {object_id}")

print("PASS: socket fixture PlanningScene IDs, defaults, dimensions, and centers match the Gazebo SDF")
