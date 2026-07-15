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

def test_physical_planning_starts_task_without_fake_perception():
    """Start the physical task directly without fake perception."""
    description = _load_launch(
        'adaptive_assembly_physical_planning.launch.py'
    )
    executables = [_executable(node) for node in _nodes(description)]

    assert executables.count('assembly_task_node') == 1
    assert 'fake_object_pose_node' not in executables

def test_physical_demo_selects_gazebo_adapter_without_duplicate_source():
    """Use inverse conditions so the target publishers cannot coexist."""
    description = _load_launch(
        'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    )
    assert _default_value(_declaration(description, FAKE_SWITCH)) == 'false'
    assert _default_value(
        _declaration(description, 'launch_object_pose_observer')
    ) == 'true'

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
    execution_arguments = next(
        dict(include.launch_arguments)
        for include in _includes(description)
        if dict(include.launch_arguments).get('use_standard_panda_demo')
        == 'false'
    )
    assert 'launch_object_pose_observer' in execution_arguments
    assert execution_arguments['target_object_gazebo_pose_topic'] == (
        '/model/target_object/pose'
    )
    assert execution_arguments['target_object_raw_pose_topic'] == (
        '/gazebo_target_object_pose_raw'
    )
    assert execution_arguments['object_pose_topic'] == (
        '/gazebo_target_object_pose'
    )


def test_physical_execution_has_one_dedicated_pose_source_and_no_pose_vector():
    """Bridge one model Pose and never identify the target through Pose_V."""
    description = _load_launch(
        'adaptive_assembly_physical_pick_place_execution.launch.py'
    )
    assert _default_value(
        _declaration(description, 'send_arm_goals')
    ) == 'true'
    nodes = _nodes(description)
    bridges = [
        node for node in nodes if _executable(node) == 'parameter_bridge'
    ]
    pose_bridges = [
        node for node in bridges
        if node.__dict__['_Node__node_name']
        == 'physical_target_object_pose_bridge'
    ]
    observers = [
        node for node in nodes
        if node.__dict__['_Node__node_name']
        == 'physical_target_object_pose_observer'
    ]

    assert len(pose_bridges) == 1
    assert len(observers) == 1
    source = (
        BRINGUP_LAUNCH_DIR
        / 'adaptive_assembly_physical_pick_place_execution.launch.py'
    ).read_text(encoding='utf-8')
    assert '@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' in source
    assert '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' not in source


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
