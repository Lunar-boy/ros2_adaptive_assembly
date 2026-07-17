"""Static contracts for exact payload scene transitions and phase planning."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PLANNING = ROOT / 'adaptive_assembly_planning'
BRINGUP = ROOT / 'adaptive_assembly_bringup'


def test_manager_preserves_exact_cylinder_link_touch_links_and_exclusivity():
    source = (
        PLANNING / 'src' / 'payload_planning_scene_manager_node.cpp'
    ).read_text(encoding='utf-8')
    assert 'grasp_not_verified' in source
    assert 'world.size() != 1 || !attached.empty()' in source
    assert '!world.empty() || attached.size() != 1' in source
    assert 'payload.object = world.front()' in source
    assert 'link_name = attachment_link_' in source
    assert 'payload.touch_links = touch_links_' in source
    assert 'panda_leftfinger,panda_rightfinger' in source
    assert 'CYLINDER_RADIUS] - 0.035' in source
    assert 'CYLINDER_HEIGHT] - 0.10' in source
    assert 'link_inverse * global_from_object_frame * object_pose' in source
    assert 'exact_finger_only_acm' in source


def test_detach_requires_fresh_observed_pose_and_verifies_world_only():
    source = (
        PLANNING / 'src' / 'payload_planning_scene_manager_node.cpp'
    ).read_text(encoding='utf-8')
    assert 'gazebo_pose_missing' in source
    assert 'age > freshness_timeout_' in source
    assert 'tf_buffer_.transform(*latest_gazebo_pose_, planning_frame_' in source
    assert 'restored.primitive_poses.assign(1, observed.pose)' in source
    assert 'verified_world.size() != 1 || !verified_attached.empty()' in source


def test_physical_launch_has_distinct_gated_immutable_generations():
    source = (
        BRINGUP / 'launch' / 'adaptive_assembly_physical_planning.launch.py'
    ).read_text(encoding='utf-8')
    assert "'stage_names_csv': 'pre_grasp,grasp'" in source
    assert "'stage_names_csv': 'lift,pre_place,place,retreat'" in source
    assert "'plan_phase': 'grasp'" in source
    assert "'plan_phase': 'transport'" in source
    assert "'initial_plan_id': 1000000" in source
    assert "'/payload_attachment_ready'" in source
    assert "'start_state_mode': 'current'" in source
    assert source.count("'lock_after_successful_sequence': True") == 1
    assert "'linear_stage_names_csv': 'grasp'" in source
    assert "'grasp_min_disallowed_clearance': 0.005" in source
