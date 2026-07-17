"""Static geometry and launch tests for the physical assembly TCP contract."""

import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_DIR.parent
PHYSICAL_PROFILE = PACKAGE_DIR / 'config' / (
    'adaptive_assembly_physical_pick_place_params.yaml'
)
PHYSICAL_WORLD = SOURCE_ROOT / 'adaptive_assembly_sim' / 'worlds' / (
    'adaptive_assembly_physical_workcell.sdf'
)
FULL_LAUNCH = PACKAGE_DIR / 'launch' / (
    'adaptive_assembly_full_physical_pick_place_demo.launch.py'
)
PLANNING_LAUNCH = PACKAGE_DIR / 'launch' / (
    'adaptive_assembly_physical_planning.launch.py'
)
EXECUTION_LAUNCH = PACKAGE_DIR / 'launch' / (
    'adaptive_assembly_physical_pick_place_execution.launch.py'
)
PANDA_XACRO = SOURCE_ROOT / 'adaptive_assembly_sim' / 'urdf' / 'panda.urdf.xacro'


def _physical_geometry():
    root = ElementTree.parse(PHYSICAL_WORLD).getroot()
    target = root.find(".//model[@name='target_object']")
    assert target is not None
    center_z = float(target.findtext('pose').split()[2])
    cylinder = target.find('.//collision/geometry/cylinder')
    assert cylinder is not None
    length = float(cylinder.findtext('length'))
    radius = float(cylinder.findtext('radius'))
    return center_z, radius, length


def _literal_launch_parameters(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    values = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if isinstance(key, ast.Constant) and isinstance(key.value, str):
                try:
                    values[key.value] = ast.literal_eval(value)
                except (ValueError, TypeError):
                    pass
    return values


def _launch_argument_defaults(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    values = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != 'DeclareLaunchArgument':
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg == 'default_value' and isinstance(keyword.value, ast.Constant):
                values[node.args[0].value] = keyword.value.value
    return values


def test_physical_target_and_task_offsets_preserve_calibrated_approach():
    _, cylinder_radius, cylinder_length = _physical_geometry()
    parameters = yaml.safe_load(PHYSICAL_PROFILE.read_text(encoding='utf-8'))[
        'assembly_task_node'
    ]['ros__parameters']

    assert cylinder_radius == 0.035
    assert cylinder_length == 0.10
    assert math.isfinite(parameters['grasp_height_offset'])
    assert 0.005 <= parameters['grasp_height_offset'] <= 0.030
    assert parameters['grasp_height_offset'] > 0.0
    assert math.isclose(
        parameters['pre_grasp_height_offset']
        - parameters['grasp_height_offset'],
        0.20,
        abs_tol=1.0e-12,
    )
    assert parameters['lift_height_offset'] == 0.20
    assert parameters['place_height_offset'] == 0.0

    full_parameters = _launch_argument_defaults(FULL_LAUNCH)
    assert full_parameters['target_reference_z_offset'] == '0.0'


def test_assembly_tcp_and_clearance_contract_remain_exact():
    root = ElementTree.parse(PANDA_XACRO).getroot()
    joint = root.find("joint[@name='panda_hand_to_assembly_tcp']")
    assert joint is not None
    assert joint.find('parent').attrib['link'] == 'panda_hand'
    assert joint.find('child').attrib['link'] == 'assembly_tcp'
    assert joint.find('origin').attrib == {
        'xyz': '0 0 0.1034',
        'rpy': '0 0 0',
    }

    profile = yaml.safe_load(PHYSICAL_PROFILE.read_text(encoding='utf-8'))
    clearance = profile['assembly_sequence_planning_node']['ros__parameters']
    assert clearance['require_grasp_clearance_validation'] is True
    assert clearance['grasp_min_disallowed_clearance'] >= 0.005
    assert clearance['grasp_clearance_target_object_id'] == 'target_object'
    assert clearance['grasp_allowed_contact_links_csv'].split(',') == [
        'panda_leftfinger', 'panda_rightfinger'
    ]

    planning_source = PLANNING_LAUNCH.read_text(encoding='utf-8')
    assert "'require_grasp_clearance_validation': True" in planning_source
    assert "'grasp_min_disallowed_clearance': 0.005" in planning_source
    assert "'grasp_clearance_target_object_id': 'target_object'" in planning_source
    assert (
        "'grasp_allowed_contact_links_csv': (" in planning_source
        and "'panda_leftfinger,panda_rightfinger'" in planning_source
    )


def test_physical_planning_owns_the_assembly_tcp_contract():
    full_source = FULL_LAUNCH.read_text(encoding='utf-8')
    planning_source = PLANNING_LAUNCH.read_text(encoding='utf-8')
    execution_source = EXECUTION_LAUNCH.read_text(encoding='utf-8')

    assert "default_value='assembly_tcp'" in full_source
    assert "default_value='assembly_tcp'" in planning_source

    assert "'end_effector_link': end_effector_link" in full_source
    assert (
        "'end_effector_link': "
        "LaunchConfiguration('end_effector_link')"
    ) in planning_source

    assert 'end_effector_link' not in execution_source
    assert 'position_tolerance' not in execution_source
    assert 'orientation_tolerance' not in execution_source


def test_dedicated_planning_launch_has_six_physical_adapters():
    source = PLANNING_LAUNCH.read_text(encoding='utf-8')
    for stage in (
        'pre_grasp', 'grasp', 'lift', 'pre_place', 'place', 'retreat'
    ):
        assert f"('{stage}', '/{stage}_pose', '/panda_{stage}_pose')" in source
    assert "'output_frame_id': 'panda_link0'" in source
    assert "'target_frame_id': 'panda_link0'" in source
    assert "'fixed_qx': 1.0" in source
    assert "'fixed_qy': 0.0" in source
    assert "'fixed_qz': 0.0" in source
    assert "'fixed_qw': 0.0" in source
    assert "'normalize_quaternion': True" in source


def test_physical_planning_bypasses_legacy_wrappers():
    source = FULL_LAUNCH.read_text(encoding='utf-8')
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
