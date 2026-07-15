"""Static regression tests for the physical fixed-socket launch profile."""

import ast
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_DIR / 'launch'


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


def _source_tree(filename):
    return ast.parse((LAUNCH_DIR / filename).read_text(encoding='utf-8'))


def _references_profile(filename):
    return any(
        isinstance(node, ast.Constant)
        and node.value == 'adaptive_assembly_physical_pick_place_params.yaml'
        for node in ast.walk(_source_tree(filename))
    )


def _default_text(description, name):
    return perform_substitutions(
        LaunchContext(), _declaration(description, name).default_value
    )


def _node_parameter_names(filename):
    """Return names referenced by each Node parameters expression."""
    parameter_names = []
    for node in ast.walk(_source_tree(filename)):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, 'id', None) != 'Node':
            continue
        parameters = next(
            (keyword.value for keyword in node.keywords
             if keyword.arg == 'parameters'),
            None,
        )
        if parameters is not None:
            parameter_names.append({
                child.id for child in ast.walk(parameters)
                if isinstance(child, ast.Name)
            })
    return parameter_names


def test_physical_profile_is_declared_and_forwarded_to_both_stacks():
    """Forward one overridable physical profile to planning and execution."""
    filenames = (
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'adaptive_assembly_physical_planning.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
    )

    for filename in filenames:
        description = _load_launch(filename)
        _declaration(description, 'params_file')

    full_description = _load_launch(
        'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    )
    include_arguments = [
        dict(action.launch_arguments)
        for action in full_description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]

    profile_consumers = [
        arguments for arguments in include_arguments
        if 'params_file' in arguments
    ]
    assert len(profile_consumers) == 2
    assert any('end_effector_link' in item for item in profile_consumers)
    assert any(
        item.get('target_object_gazebo_pose_topic')
        == '/model/target_object/pose'
        for item in profile_consumers
    )

    for filename in filenames[1:]:
        assert any(
            'params_file' in names
            for names in _node_parameter_names(filename)
        )


def test_all_physical_launches_default_to_the_physical_profile():
    """Use the physical task profile in every physical launch entrypoint."""
    physical_launches = (
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'adaptive_assembly_physical_planning.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
    )

    for filename in physical_launches:
        assert _references_profile(filename)
        default = _default_text(_load_launch(filename), 'params_file')
        assert default.endswith(
            'adaptive_assembly_physical_pick_place_params.yaml'
        )
