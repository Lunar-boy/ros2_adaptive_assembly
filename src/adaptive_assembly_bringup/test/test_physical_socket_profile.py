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

def _source_tree(filename):
    return ast.parse((LAUNCH_DIR / filename).read_text())

def _references_profile(filename):
    return any(
        isinstance(node, ast.Constant)
        and node.value == 'adaptive_assembly_physical_pick_place_params.yaml'
        for node in ast.walk(_source_tree(filename))
    )


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

    assert sum(
        'params_file' in arguments
        for arguments in include_arguments
    ) == 2

    planning_source = (
        LAUNCH_DIR / 'adaptive_assembly_physical_planning.launch.py'
    ).read_text(encoding='utf-8')
    execution_source = (
        LAUNCH_DIR
        / 'adaptive_assembly_physical_pick_place_execution.launch.py'
    ).read_text(encoding='utf-8')

    assert 'parameters=[params_file,' in planning_source
    assert 'parameters=[params_file,' in execution_source


def test_all_physical_launches_default_to_the_physical_profile():
    """Use the physical task profile in every physical launch entrypoint."""
    physical_launches = (
        'adaptive_assembly_full_physical_pick_place_demo.launch.py',
        'adaptive_assembly_physical_planning.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
    )

    for filename in physical_launches:
        assert _references_profile(filename)
        _declaration(_load_launch(filename), 'params_file')
