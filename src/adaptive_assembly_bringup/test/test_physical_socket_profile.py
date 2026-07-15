"""Static regression tests for the physical fixed-socket launch profile."""

import ast
import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


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


def _launch_includes_params_file(filename):
    description = _load_launch(filename)
    return any(
        'params_file' in dict(action.launch_arguments)
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    )


def _source_tree(filename):
    return ast.parse((LAUNCH_DIR / filename).read_text())


def _declares_default_variable(filename, argument, variable):
    for node in ast.walk(_source_tree(filename)):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, 'id', None) != 'DeclareLaunchArgument':
            continue
        if not node.args or getattr(node.args[0], 'value', None) != argument:
            continue
        for keyword in node.keywords:
            if keyword.arg == 'default_value' and getattr(
                keyword.value, 'id', None
            ) == variable:
                return True
    return False


def _references_profile(filename):
    return any(
        isinstance(node, ast.Constant)
        and node.value == 'adaptive_assembly_physical_pick_place_params.yaml'
        for node in ast.walk(_source_tree(filename))
    )


def test_physical_chain_propagates_an_overridable_params_file():
    """Route the physical default profile through every nested task wrapper."""
    chain = (
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
        'adaptive_assembly_physical_planning.launch.py',
    )
    for filename in chain:
        description = _load_launch(filename)
        _declaration(description, 'params_file')
    for filename in chain[:-1]:
        assert _launch_includes_params_file(filename)


def test_only_physical_launches_reference_the_physical_profile():
    """Leave generic and plan-only launch defaults on their existing profiles."""
    physical_launches = (
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
        'adaptive_assembly_physical_planning.launch.py',
    )
    for filename in physical_launches:
        assert _references_profile(filename)
        assert _declares_default_variable(
            filename, 'params_file', 'physical_params_file'
        )
    for filename in nonphysical_launches:
        assert not _references_profile(filename)

        tree = _source_tree(filename)
        assert any(
            isinstance(node, ast.Call)
            and getattr(node.func, 'id', None) == 'DeclareLaunchArgument'
            for node in ast.walk(tree)
        )

