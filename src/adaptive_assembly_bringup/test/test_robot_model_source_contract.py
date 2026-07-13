"""Guard the physical demo's unified canonical Panda model source."""

import ast
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from adaptive_assembly_sim.robot_model_parity import CURRENT_PANDA_TOOL_LINK


BRINGUP_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = BRINGUP_DIR.parent
BRINGUP_LAUNCH = BRINGUP_DIR / 'launch'
SIM_LAUNCH = WORKSPACE_SRC / 'adaptive_assembly_sim' / 'launch'
SIM_PACKAGE = WORKSPACE_SRC / 'adaptive_assembly_sim'
GAZEBO_XACRO = SIM_PACKAGE / 'urdf' / 'panda_gazebo_ros2_control.urdf.xacro'
CANONICAL_XACRO = SIM_PACKAGE / 'urdf' / 'panda.urdf.xacro'
PARITY_SOURCE = (
    SIM_PACKAGE / 'adaptive_assembly_sim' / 'robot_model_parity.py'
)
XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'


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
    """Use canonical MoveIt kinematics and Gazebo controllers together."""
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
    file_path = next(
        item for item in robot_description.keywords if item.arg == 'file_path'
    ).value
    assert isinstance(file_path, ast.Name)
    assert file_path.id == 'canonical_panda_xacro'
    assert {'adaptive_assembly_sim', 'panda.urdf.xacro'}.issubset(
        _assignment_strings(panda_demo, 'canonical_panda_xacro')
    )

    assert 'adaptive_assembly_sim' in _assignment_strings(
        gazebo_launch, 'default_model'
    )
    assert 'panda_gazebo_ros2_control.urdf.xacro' in _assignment_strings(
        gazebo_launch, 'default_model'
    )

    full_demo_text = full_demo.read_text(encoding='utf-8')
    physical_execution_text = physical_execution.read_text(encoding='utf-8')
    assert 'adaptive_assembly_panda_demo.launch.py' not in full_demo_text
    assert 'adaptive_assembly_panda_demo.launch.py' not in physical_execution_text


def test_gazebo_wrapper_includes_canonical_description_without_arm_chain():
    """Reject a second local declaration of any canonical Panda arm joint."""
    root = ET.parse(GAZEBO_XACRO).getroot()
    includes = root.findall(f'{{{XACRO_NAMESPACE}}}include')
    assert [include.get('filename') for include in includes] == [
        '$(find adaptive_assembly_sim)/urdf/panda.urdf.xacro'
    ]

    canonical_root = ET.parse(CANONICAL_XACRO).getroot()
    canonical_includes = canonical_root.findall(
        f'{{{XACRO_NAMESPACE}}}include'
    )
    assert [include.get('filename') for include in canonical_includes] == [
        '$(find moveit_resources_panda_description)/urdf/panda.urdf.xacro'
    ]

    local_kinematic_joints = {
        joint.get('name') for joint in root.findall('./joint')
    }
    canonical_arm_joints = {f'panda_joint{number}' for number in range(1, 8)}
    assert local_kinematic_joints.isdisjoint(canonical_arm_joints)

    expanded = subprocess.run(
        ['xacro', str(GAZEBO_XACRO)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    expanded_root = ET.fromstring(expanded)
    expanded_kinematic_names = [
        joint.get('name') for joint in expanded_root.findall('./joint')
    ]
    for joint_name in canonical_arm_joints:
        assert expanded_kinematic_names.count(joint_name) == 1
    assert len(expanded_root.findall('./ros2_control')) == 1

    links = {link.get('name') for link in expanded_root.findall('./link')}
    assert {
        'panda_link8', 'panda_hand',
        'panda_leftfinger', 'panda_rightfinger',
    }.issubset(links)
    mimic = expanded_root.find(
        "./joint[@name='panda_finger_joint2']/mimic"
    )
    assert mimic is not None
    assert mimic.get('joint') == 'panda_finger_joint1'


def test_current_parity_preset_uses_one_canonical_tool_endpoint():
    """Compare panda_link8 on both sides without compensating tool offsets."""
    assert CURRENT_PANDA_TOOL_LINK == 'panda_link8'
    parity_text = PARITY_SOURCE.read_text(encoding='utf-8')
    assert "'panda_hand' if arguments.current_panda_models" not in parity_text
