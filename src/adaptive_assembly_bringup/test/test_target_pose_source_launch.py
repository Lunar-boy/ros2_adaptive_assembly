"""Regression tests for fake and Gazebo target-pose launch composition."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


BRINGUP_LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
SIM_LAUNCH_DIR = (
    Path(__file__).resolve().parents[2] / 'adaptive_assembly_sim' / 'launch'
)
FAKE_SWITCH = 'launch_fake_object_pose_node'


def _load_launch(filename, launch_dir=BRINGUP_LAUNCH_DIR):
    spec = importlib.util.spec_from_file_location(
        filename, launch_dir / filename
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _declaration(description, name):
    return next(
        action for action in description.entities
        if isinstance(action, DeclareLaunchArgument) and action.name == name
    )


def _default_value(declaration):
    return ''.join(
        substitution.text for substitution in declaration.default_value
    )


def _includes(description):
    return [
        action for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]


def _nodes(description):
    return [
        action for action in description.entities if isinstance(action, Node)
    ]


def _executable(node):
    return node.__dict__['_Node__node_executable']


def test_pipeline_fake_source_is_conditional_and_task_is_unconditional():
    """Keep the task node active while making only fake perception optional."""
    description = _load_launch('adaptive_assembly_pipeline.launch.py')
    nodes = {_executable(node): node for node in _nodes(description)}

    assert _default_value(_declaration(description, FAKE_SWITCH)) == 'true'
    assert isinstance(nodes['fake_object_pose_node'].condition, IfCondition)
    assert nodes['assembly_task_node'].condition is None


def test_fake_source_switch_propagates_with_nonphysical_defaults():
    """Reusable wrappers default to fake perception and pass its value."""
    wrapper_files = (
        'adaptive_assembly_panda_demo.launch.py',
        'adaptive_assembly_panda_planning_demo.launch.py',
        'adaptive_assembly_panda_sequence_planning_demo.launch.py',
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
    )
    for filename in wrapper_files:
        description = _load_launch(filename)
        assert _default_value(_declaration(description, FAKE_SWITCH)) == 'true'
        assert any(
            FAKE_SWITCH in dict(include.launch_arguments)
            for include in _includes(description)
        )


def test_physical_demo_selects_gazebo_adapter_without_duplicate_source():
    """Use inverse conditions so the target publishers cannot coexist."""
    description = _load_launch(
        'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    )
    assert _default_value(_declaration(description, FAKE_SWITCH)) == 'false'

    adapter_include = next(
        include for include in _includes(description)
        if dict(include.launch_arguments).get('output_pose_topic')
        == '/target_pose'
    )
    assert isinstance(adapter_include.condition, UnlessCondition)
    assert dict(adapter_include.launch_arguments)['input_pose_topic'] == (
        '/gazebo_target_object_pose'
    )
    assert any(
        FAKE_SWITCH in dict(include.launch_arguments)
        for include in _includes(description)
    )


def test_adapter_launch_has_expected_topics_and_reference_offset():
    """Expose the explicit Gazebo-center to task-reference conversion."""
    description = _load_launch(
        'gazebo_target_pose_adapter.launch.py', SIM_LAUNCH_DIR
    )
    nodes = {_executable(node): node for node in _nodes(description)}

    assert 'gazebo_target_pose_adapter_node' in nodes
    assert _default_value(
        _declaration(description, 'input_pose_topic')
    ) == '/gazebo_target_object_pose'
    assert _default_value(
        _declaration(description, 'output_pose_topic')
    ) == '/target_pose'
    assert _default_value(
        _declaration(description, 'target_reference_z_offset')
    ) == '0.05'
    assert _default_value(
        _declaration(description, 'output_frame_id')
    ) == 'world'
