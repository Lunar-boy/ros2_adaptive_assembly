"""Regression tests for physical Gazebo simulation-time launch wiring."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
PLANNING_LAUNCH_DIR = (
    Path(__file__).resolve().parents[2]
    / 'adaptive_assembly_planning'
    / 'launch'
)


def _load_launch(filename, launch_dir=LAUNCH_DIR):
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


def _include_arguments(description):
    return [
        dict(action.launch_arguments)
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]


def _node(description, executable):
    return next(
        action for action in description.entities
        if isinstance(action, Node)
        and action.__dict__['_Node__node_executable'] == executable
    )


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
    """The full physical entry point must opt into Gazebo time."""
    description = _load_launch(
        'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    )

    assert _default_value(_declaration(description, 'use_sim_time')) == 'true'
    include_arguments = _include_arguments(description)
    assert any(
        arguments.get('use_standard_panda_demo') == 'false'
        and 'use_sim_time' in arguments
        for arguments in include_arguments
    )


def test_simulation_time_propagates_to_physical_execution_and_sequence():
    """The full path must retain the simulation-time launch value."""
    physical = _load_launch(
        'adaptive_assembly_physical_pick_place_execution.launch.py'
    )
    reachable = _load_launch(
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py'
    )
    sequence = _load_launch(
        'adaptive_assembly_panda_sequence_planning_demo.launch.py'
    )

    assert _default_value(_declaration(physical, 'use_sim_time')) == 'false'
    assert any(
        'use_sim_time' in arguments
        for arguments in _include_arguments(physical)
    )
    assert any(
        'use_sim_time' in arguments
        for arguments in _include_arguments(reachable)
    )
    assert any(
        'use_sim_time' in arguments
        for arguments in _include_arguments(sequence)
    )


def test_direct_move_group_receives_typed_simulation_time():
    """The direct Gazebo MoveIt node must receive a Boolean parameter."""
    description = _load_launch('adaptive_assembly_panda_demo.launch.py')

    move_group = _node(description, 'move_group')
    _assert_typed_sim_time(move_group)


def test_physical_time_sensitive_nodes_receive_typed_simulation_time():
    """Physical stale-data and planning nodes must share the time domain."""
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
        _assert_typed_sim_time(_node(description, executable))

    planning = _load_launch(
        'assembly_sequence_planning.launch.py', PLANNING_LAUNCH_DIR
    )
    _assert_typed_sim_time(_node(planning, 'assembly_sequence_planning_node'))


def test_non_gazebo_planning_defaults_to_wall_time_and_no_fake_controller():
    """Plan-only defaults stay wall-time without a controller manager."""
    planning = _load_launch('adaptive_assembly_panda_planning_demo.launch.py')
    panda = _load_launch('adaptive_assembly_panda_demo.launch.py')

    assert _default_value(_declaration(planning, 'use_sim_time')) == 'false'
    assert _default_value(_declaration(panda, 'use_sim_time')) == 'false'
    assert _default_value(
        _declaration(panda, 'use_standard_panda_demo')
    ) == 'true'
    assert not any(
        action.__dict__['_Node__node_executable'] == 'ros2_control_node'
        for action in panda.entities if isinstance(action, Node)
    )
