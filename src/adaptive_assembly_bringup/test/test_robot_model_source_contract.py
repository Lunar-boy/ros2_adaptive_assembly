"""Static guard for the physical demo's two robot-model sources."""

import ast
from pathlib import Path


BRINGUP_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = BRINGUP_DIR.parent
BRINGUP_LAUNCH = BRINGUP_DIR / 'launch'
SIM_LAUNCH = WORKSPACE_SRC / 'adaptive_assembly_sim' / 'launch'


def _tree(path):
    return ast.parse(path.read_text(encoding='utf-8'), filename=str(path))


def _assignment_strings(path, variable):
    """Collect string constants only from one named assignment expression."""
    for node in ast.walk(_tree(path)):
        if not isinstance(node, ast.Assign):
            continue
        if any(
            isinstance(target, ast.Name) and target.id == variable
            for target in node.targets
        ):
            return {
                child.value for child in ast.walk(node.value)
                if isinstance(child, ast.Constant)
                and isinstance(child.value, str)
            }
    raise AssertionError(f'assignment {variable!r} not found in {path}')


def _find_call(path, method_name):
    for node in ast.walk(_tree(path)):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if isinstance(function, ast.Name) and function.id == method_name:
            return node
        if isinstance(function, ast.Attribute) and function.attr == method_name:
            return node
    raise AssertionError(f'call {method_name!r} not found in {path}')


def _keyword_string(call, keyword_name):
    keyword = next(
        item for item in call.keywords if item.arg == keyword_name
    )
    assert isinstance(keyword.value, ast.Constant)
    return keyword.value.value


def test_physical_demo_model_sources_match_the_expected_parity_contract():
    """Require model-source changes to update diagnostics and regressions."""
    full_demo = (
        BRINGUP_LAUNCH
        / 'adaptive_assembly_full_physical_pick_place_demo.launch.py'
    )
    physical_execution = (
        BRINGUP_LAUNCH
        / 'adaptive_assembly_physical_pick_place_execution.launch.py'
    )
    panda_demo = (
        BRINGUP_LAUNCH / 'adaptive_assembly_panda_demo.launch.py'
    )
    gazebo_launch = (
        SIM_LAUNCH / 'adaptive_assembly_panda_gazebo.launch.py'
    )

    assert 'adaptive_assembly_panda_gazebo.launch.py' in (
        _assignment_strings(full_demo, 'sim_launch')
    )
    assert 'adaptive_assembly_physical_pick_place_execution.launch.py' in (
        _assignment_strings(full_demo, 'execution_launch')
    )
    assert 'adaptive_assembly_panda_sequence_planning_reachable.launch.py' in (
        _assignment_strings(physical_execution, 'reachable_sequence_launch')
    )

    builder = _find_call(panda_demo, 'MoveItConfigsBuilder')
    assert isinstance(builder.args[0], ast.Constant)
    assert builder.args[0].value == 'moveit_resources_panda'
    robot_description = _find_call(panda_demo, 'robot_description')
    assert _keyword_string(robot_description, 'file_path') == (
        'config/panda.urdf.xacro'
    )

    assert 'adaptive_assembly_sim' in _assignment_strings(
        gazebo_launch, 'default_model'
    )
    assert 'panda_gazebo_ros2_control.urdf.xacro' in _assignment_strings(
        gazebo_launch, 'default_model'
    )
