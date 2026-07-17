"""Regression tests for the dedicated physical planning launch."""

import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_DIR / 'launch'
SOURCE_ROOT = PACKAGE_DIR.parent
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
    spec = importlib.util.spec_from_file_location(filename, LAUNCH_DIR / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _nodes(description):
    return [entity for entity in description.entities if isinstance(entity, Node)]


def _executable(node):
    return node.__dict__['_Node__node_executable']


def _default_text(description, name):
    declaration = next(
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == name
    )
    return perform_substitutions(
        LaunchContext(),
        declaration.default_value,
    )


def test_physical_configuration_is_installed_and_contains_all_objects():
    source_profile = PACKAGE_DIR / 'config' / PHYSICAL_CONFIGURATION
    installed_profile = (
        Path(get_package_share_directory('adaptive_assembly_bringup'))
        / 'config' / PHYSICAL_CONFIGURATION
    )
    assert source_profile.is_file()
    assert installed_profile.is_file()

    parameters = yaml.safe_load(source_profile.read_text(encoding='utf-8'))[
        'static_planning_scene_node'
    ]['ros__parameters']
    assert parameters['add_work_table'] is True
    assert parameters['add_target_support'] is True
    assert parameters['add_socket_fixture'] is True

    node_source = (
        SOURCE_ROOT / 'adaptive_assembly_planning' / 'src'
        / 'static_planning_scene_node.cpp'
    ).read_text(encoding='utf-8')
    for object_id in PHYSICAL_OBJECT_IDS:
        assert f'"{object_id}"' in node_source


def test_dedicated_planning_launch_starts_required_nodes_directly():
    description = _load_launch('adaptive_assembly_physical_planning.launch.py')
    executables = [_executable(node) for node in _nodes(description)]

    assert executables.count('assembly_task_node') == 1
    assert executables.count('move_group') == 1
    assert executables.count('static_planning_scene_node') == 1
    assert executables.count('planning_scene_audit_node') == 1
    assert executables.count('panda_pre_grasp_pose_adapter_node') == 6
    assert executables.count('assembly_sequence_planning_node') == 2
    assert executables.count('payload_planning_scene_manager_node') == 1
    assert not any(
        isinstance(entity, IncludeLaunchDescription)
        for entity in description.entities
    )


def test_full_demo_includes_dedicated_planning_and_disables_legacy_sequence():
    source = (
        LAUNCH_DIR / 'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    ).read_text(encoding='utf-8')
    assert 'adaptive_assembly_physical_planning.launch.py' in source
    assert 'launch_reachable_sequence' not in source
    for legacy in (
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
        'adaptive_assembly_panda_sequence_planning_demo.launch.py',
        'adaptive_assembly_panda_planning_demo.launch.py',
        'adaptive_assembly_panda_demo.launch.py',
        'adaptive_assembly_pipeline.launch.py',
    ):
        assert legacy not in source


def test_physical_planning_defaults_to_physical_profile():
    description = _load_launch('adaptive_assembly_physical_planning.launch.py')
    assert PHYSICAL_CONFIGURATION in _default_text(
        description, PARAMETER_ARGUMENT
    )
    assert _default_text(description, 'end_effector_link') == 'assembly_tcp'


def test_full_physical_demo_uses_identity_world_to_panda_link0():
    model_path = (
        SOURCE_ROOT / 'adaptive_assembly_sim' / 'urdf'
        / 'panda_gazebo_ros2_control.urdf.xacro'
    )
    model = ElementTree.parse(model_path).getroot()
    world_joint = model.find("joint[@name='panda_world_joint']")
    assert world_joint is not None
    origin = world_joint.find('origin')
    assert origin is not None
    assert origin.get('xyz') == '0 0 0'
    assert origin.get('rpy') == '0 0 0'

    gazebo_launch = (
        SOURCE_ROOT / 'adaptive_assembly_sim' / 'launch'
        / 'adaptive_assembly_panda_gazebo.launch.py'
    )
    tree = ast.parse(gazebo_launch.read_text(encoding='utf-8'))
    defaults = {}
    for node in ast.walk(tree):
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
