"""Regression tests for physical PlanningScene launch composition."""

import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_DIR / 'launch'
PLANNING_LAUNCH = (
    PACKAGE_DIR.parents[0]
    / 'adaptive_assembly_planning'
    / 'launch'
    / 'static_planning_scene.launch.py'
)
PLANNING_NODE = (
    PACKAGE_DIR.parents[0]
    / 'adaptive_assembly_planning'
    / 'src'
    / 'static_planning_scene_node.cpp'
)
PANDA_GAZEBO_LAUNCH = (
    PACKAGE_DIR.parents[0]
    / 'adaptive_assembly_sim'
    / 'launch'
    / 'adaptive_assembly_panda_gazebo.launch.py'
)
PANDA_MODEL = (
    PACKAGE_DIR.parents[0]
    / 'adaptive_assembly_sim'
    / 'urdf'
    / 'panda_gazebo_ros2_control.urdf.xacro'
)
PHYSICAL_CONFIGURATION = 'physical_workcell_planning_scene.yaml'
PARAMETER_ARGUMENT = 'static_planning_scene_params_file'
PHYSICAL_OBJECT_IDS = {
    'work_table',
    'target_support',
    'assembly_socket_base',
    'assembly_socket_left_wall',
    'assembly_socket_right_wall',
    'assembly_socket_back_wall',
    'assembly_socket_front_wall',
}


def _load_launch(filename):
    spec = importlib.util.spec_from_file_location(
        filename, LAUNCH_DIR / filename
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _declaration(description, name):
    return next(
        action for action in description.entities
        if isinstance(action, DeclareLaunchArgument) and action.name == name
    )


def _default_text(declaration):
    return ''.join(
        substitution.text for substitution in declaration.default_value
        if hasattr(substitution, 'text')
    )


def _include_argument_names(description):
    return [
        set(dict(action.launch_arguments))
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]


def _source_tree(path):
    return ast.parse(path.read_text(encoding='utf-8'))


def test_physical_configuration_is_installed_and_contains_all_objects():
    """Install the physical profile with all static workcell collision boxes."""
    source_profile = PACKAGE_DIR / 'config' / PHYSICAL_CONFIGURATION
    installed_profile = (
        Path(get_package_share_directory('adaptive_assembly_bringup'))
        / 'config'
        / PHYSICAL_CONFIGURATION
    )
    assert source_profile.is_file()
    assert installed_profile.is_file()

    parameters = yaml.safe_load(source_profile.read_text())[
        'static_planning_scene_node'
    ]['ros__parameters']
    assert parameters['add_work_table'] is True
    assert parameters['add_target_support'] is True
    assert parameters['add_socket_fixture'] is True
    node_source = PLANNING_NODE.read_text(encoding='utf-8')
    for object_id in PHYSICAL_OBJECT_IDS:
        assert f'"{object_id}"' in node_source


def test_only_full_physical_demo_defaults_to_the_physical_profile():
    """Keep plan-only and reusable launch defaults on their current geometry."""
    full = _load_launch('adaptive_assembly_full_physical_pick_place_demo.launch.py')
    assert PARAMETER_ARGUMENT in {
        action.name for action in full.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    full_tree = _source_tree(
        LAUNCH_DIR / 'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    )
    assert any(
        isinstance(node, ast.Constant) and node.value == PHYSICAL_CONFIGURATION
        for node in ast.walk(full_tree)
    )

    nonphysical = (
        'adaptive_assembly_panda_planning_demo.launch.py',
        'adaptive_assembly_panda_sequence_planning_demo.launch.py',
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
    )
    for filename in nonphysical:
        declaration = _declaration(_load_launch(filename), PARAMETER_ARGUMENT)
        assert _default_text(declaration) == ''


def test_physical_parameter_file_reaches_one_static_scene_launch():
    """Forward the profile through the physical chain without a second node."""
    chain = (
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
        'adaptive_assembly_panda_sequence_planning_demo.launch.py',
        'adaptive_assembly_panda_planning_demo.launch.py',
    )
    for filename in chain:
        assert any(
            PARAMETER_ARGUMENT in names
            for names in _include_argument_names(_load_launch(filename))
        )

    planning_tree = _source_tree(PLANNING_LAUNCH)
    node_calls = [
        node for node in ast.walk(planning_tree)
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node'
    ]
    assert len(node_calls) == 1
    assert any(
        isinstance(node, ast.Constant) and node.value == PARAMETER_ARGUMENT
        for node in ast.walk(planning_tree)
    )


def test_full_physical_demo_uses_identity_world_to_panda_link0():
    """Use SDF coordinates only because this launch keeps the frames aligned."""
    model = ElementTree.parse(PANDA_MODEL).getroot()
    world_joint = model.find("joint[@name='panda_world_joint']")
    assert world_joint is not None
    origin = world_joint.find('origin')
    assert origin is not None
    assert origin.get('xyz') == '0 0 0'
    assert origin.get('rpy') == '0 0 0'

    gazebo_tree = _source_tree(PANDA_GAZEBO_LAUNCH)
    defaults = {}
    for node in ast.walk(gazebo_tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, 'id', None) != 'DeclareLaunchArgument':
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg == 'default_value' and isinstance(
                    keyword.value, ast.Constant):
                defaults[node.args[0].value] = keyword.value.value
    assert {name: defaults[name] for name in (
        'spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw'
    )} == {
        'spawn_x': '0.0',
        'spawn_y': '0.0',
        'spawn_z': '0.0',
        'spawn_yaw': '0.0',
    }
