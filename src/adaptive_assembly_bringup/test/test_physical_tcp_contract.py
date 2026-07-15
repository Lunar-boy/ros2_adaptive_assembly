"""Static geometry and launch tests for the physical assembly TCP contract."""

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


def _physical_geometry():
    root = ElementTree.parse(PHYSICAL_WORLD).getroot()
    target = root.find(".//model[@name='target_object']")
    assert target is not None
    center_z = float(target.findtext('pose').split()[2])
    cylinder = target.find('.//collision/geometry/cylinder')
    assert cylinder is not None
    length = float(cylinder.findtext('length'))
    return center_z, length


def test_physical_target_and_task_offsets_end_at_cylinder_grasp_center():
    center_z, cylinder_length = _physical_geometry()
    parameters = yaml.safe_load(PHYSICAL_PROFILE.read_text(encoding='utf-8'))[
        'assembly_task_node'
    ]['ros__parameters']
    launch_source = FULL_LAUNCH.read_text(encoding='utf-8')

    assert cylinder_length == 0.10
    assert "'target_reference_z_offset',\n            default_value='0.0'" in launch_source
    final_grasp_z = center_z + parameters['grasp_height_offset']
    assert final_grasp_z == center_z
    assert parameters['pre_grasp_height_offset'] == 0.20
    assert parameters['lift_height_offset'] == 0.20
    assert parameters['place_height_offset'] == 0.0


def test_physical_launch_chain_uses_one_assembly_tcp_argument():
    sources = [
        FULL_LAUNCH.read_text(encoding='utf-8'),
        PLANNING_LAUNCH.read_text(encoding='utf-8'),
        EXECUTION_LAUNCH.read_text(encoding='utf-8'),
    ]

    assert "default_value='assembly_tcp'" in sources[0]
    assert "default_value='assembly_tcp'" in sources[1]
    assert "'end_effector_link': 'assembly_tcp'" in sources[2]
    assert "default_value='0.005'" in sources[2]
    assert "default_value='0.03'" in sources[2]
    for source in sources:
        assert 'end_effector_link' in source


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
