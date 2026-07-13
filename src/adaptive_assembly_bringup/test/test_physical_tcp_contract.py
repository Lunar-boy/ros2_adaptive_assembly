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
EXECUTION_LAUNCH = PACKAGE_DIR / 'launch' / (
    'adaptive_assembly_physical_pick_place_execution.launch.py'
)
REACHABLE_LAUNCH = PACKAGE_DIR / 'launch' / (
    'adaptive_assembly_panda_sequence_planning_reachable.launch.py'
)
SEQUENCE_LAUNCH = PACKAGE_DIR / 'launch' / (
    'adaptive_assembly_panda_sequence_planning_demo.launch.py'
)
PLANNER_LAUNCH = SOURCE_ROOT / 'adaptive_assembly_planning' / 'launch' / (
    'assembly_sequence_planning.launch.py'
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
    """Lock object center, observed reference, task grasp, and TCP together."""
    center_z, cylinder_length = _physical_geometry()
    parameters = yaml.safe_load(PHYSICAL_PROFILE.read_text(encoding='utf-8'))[
        'assembly_task_node'
    ]['ros__parameters']
    launch_source = FULL_LAUNCH.read_text(encoding='utf-8')

    assert cylinder_length == 0.10
    assert "'target_reference_z_offset',\n            default_value='0.0'" in launch_source
    target_reference_offset = 0.0
    final_grasp_z = (
        center_z + target_reference_offset
        + parameters['grasp_height_offset']
    )
    assert final_grasp_z == center_z
    assert parameters['pre_grasp_height_offset'] == 0.20
    assert parameters['lift_height_offset'] == 0.20
    assert parameters['place_height_offset'] == 0.0


def test_generic_nonphysical_profiles_retain_existing_offsets():
    """Keep the physical geometry correction scoped to the physical demo."""
    task_node = SOURCE_ROOT / 'adaptive_assembly_task' / (
        'adaptive_assembly_task/assembly_task_node.py'
    )
    adapter = SOURCE_ROOT / 'adaptive_assembly_sim' / 'launch' / (
        'gazebo_target_pose_adapter.launch.py'
    )

    assert "declare_parameter('grasp_height_offset', 0.05)" in (
        task_node.read_text(encoding='utf-8')
    )
    assert "('target_reference_z_offset', '0.05')" in adapter.read_text(
        encoding='utf-8'
    )


def test_physical_launch_chain_propagates_one_assembly_tcp_argument():
    """Require explicit assembly_tcp propagation through every wrapper."""
    sources = [
        FULL_LAUNCH.read_text(encoding='utf-8'),
        EXECUTION_LAUNCH.read_text(encoding='utf-8'),
        REACHABLE_LAUNCH.read_text(encoding='utf-8'),
        SEQUENCE_LAUNCH.read_text(encoding='utf-8'),
        PLANNER_LAUNCH.read_text(encoding='utf-8'),
    ]

    assert "default_value='assembly_tcp'" in sources[0]
    assert "'end_effector_link': 'assembly_tcp'" in sources[1]
    assert "default_value='0.005'" in sources[1]
    assert "default_value='0.03'" in sources[1]
    for source in sources:
        assert 'end_effector_link' in source
    for source in sources[:-1]:
        assert "'end_effector_link':" in source


def test_all_physical_stage_adapters_publish_normalized_base_frame_targets():
    """Keep every Panda stage pose deterministic in panda_link0."""
    launch_dir = SOURCE_ROOT / 'adaptive_assembly_planning' / 'launch'
    adapters = (
        'panda_pre_grasp_pose_adapter.launch.py',
        'panda_grasp_pose_adapter.launch.py',
        'panda_lift_pose_adapter.launch.py',
        'panda_pre_place_pose_adapter.launch.py',
        'panda_place_pose_adapter.launch.py',
        'panda_retreat_pose_adapter.launch.py',
    )
    for name in adapters:
        source = (launch_dir / name).read_text(encoding='utf-8')
        assert "'output_frame_id': 'panda_link0'" in source
        assert "'target_frame_id': 'panda_link0'" in source
        assert "'fixed_qx': 1.0" in source
        assert "'fixed_qy': 0.0" in source
        assert "'fixed_qz': 0.0" in source
        assert "'fixed_qw': 0.0" in source
        assert "'normalize_quaternion': True" in source
