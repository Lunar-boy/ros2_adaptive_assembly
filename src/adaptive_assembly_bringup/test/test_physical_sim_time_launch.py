"""Regression tests for physical Gazebo simulation-time launch wiring."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'


def _load_launch(filename):
    spec = importlib.util.spec_from_file_location(filename, LAUNCH_DIR / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _declaration(description, name):
    return next(
        action for action in description.entities
        if isinstance(action, DeclareLaunchArgument) and action.name == name
    )


def _default_value(declaration):
    return ''.join(substitution.text for substitution in declaration.default_value)


def _include_arguments(description):
    return [
        dict(action.launch_arguments)
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]


def _nodes(description):
    return [action for action in description.entities if isinstance(action, Node)]


def _executable(node):
    return node.__dict__['_Node__node_executable']


def _parameter(node, name):
    for parameter_set in node.__dict__['_Node__parameters']:
        if not isinstance(parameter_set, dict):
            continue
        for key, value in parameter_set.items():
            key_name = ''.join(substitution.text for substitution in key)
            if key_name == name:
                return value
    raise AssertionError(f'Parameter {name} is not configured.')


def _assert_typed_sim_time(node):
    parameter = _parameter(node, 'use_sim_time')
    assert isinstance(parameter, ParameterValue)
    assert parameter.__dict__['_ParameterValue__value_type'] is bool


def test_full_physical_launch_defaults_to_simulation_time():
    description = _load_launch(
        'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    )
    assert _default_value(_declaration(description, 'use_sim_time')) == 'true'
    include_arguments = _include_arguments(description)
    assert any(
        arguments.get('world_name') == 'adaptive_assembly_physical_workcell'
        and 'use_sim_time' in arguments
        for arguments in include_arguments
    )
    assert any(
        arguments.get('stage_names')
        == 'pre_grasp,grasp,lift,pre_place,place,retreat'
        and 'use_sim_time' in arguments
        for arguments in include_arguments
    )


def test_dedicated_physical_planning_defaults_to_simulation_time():
    planning = _load_launch('adaptive_assembly_physical_planning.launch.py')
    assert _default_value(_declaration(planning, 'use_sim_time')) == 'true'


def test_direct_physical_nodes_receive_typed_simulation_time():
    planning = _load_launch('adaptive_assembly_physical_planning.launch.py')
    for node in _nodes(planning):
        if _executable(node) in (
            'assembly_task_node',
            'move_group',
            'panda_pre_grasp_pose_adapter_node',
            'assembly_sequence_planning_node',
        ):
            _assert_typed_sim_time(node)


def test_execution_time_sensitive_nodes_receive_typed_simulation_time():
    description = _load_launch(
        'adaptive_assembly_physical_pick_place_execution.launch.py'
    )
    for executable in (
        'physical_pick_place_executor_node',
        'physical_grasp_preflight_node',
        'grasp_verifier_node',
        'gazebo_grasp_contact_status_node',
        'gazebo_entity_pose_observer_node',
    ):
        node = next(
            item for item in _nodes(description)
            if _executable(item) == executable
        )
        _assert_typed_sim_time(node)


def test_full_demo_disables_legacy_planning_include():
    source = (
        LAUNCH_DIR / 'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    ).read_text(encoding='utf-8')
    assert 'launch_reachable_sequence' not in source
    assert 'adaptive_assembly_physical_planning.launch.py' in source
