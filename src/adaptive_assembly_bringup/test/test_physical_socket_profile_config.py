"""Static tests for the physical fixed-socket parameter profile."""

from pathlib import Path
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PHYSICAL_PROFILE = (
    PACKAGE_DIR / 'config' / 'adaptive_assembly_physical_pick_place_params.yaml'
)
PHYSICAL_WORLD = (
    PACKAGE_DIR.parents[0]
    / 'adaptive_assembly_sim'
    / 'worlds'
    / 'adaptive_assembly_physical_workcell.sdf'
)
PHYSICAL_EXECUTION_LAUNCH = (
    PACKAGE_DIR / 'launch'
    / 'adaptive_assembly_physical_pick_place_execution.launch.py'
)


def test_physical_profile_matches_the_gazebo_socket_fixture():
    """Keep the physical task target tied to the physical workcell fixture."""
    profile = yaml.safe_load(PHYSICAL_PROFILE.read_text())
    parameters = profile['assembly_task_node']['ros__parameters']
    root = ElementTree.parse(PHYSICAL_WORLD).getroot()
    fixture = root.find(".//model[@name='assembly_socket_fixture']")
    assert fixture is not None
    fixture_x, fixture_y, fixture_z, *_ = [
        float(value) for value in fixture.findtext('pose').split()
    ]

    assert parameters['assembly_pose_mode'] == 'fixed_socket'
    assert parameters['grasp_height_offset'] == 0.0
    assert parameters['lift_height_offset'] == 0.20
    assert parameters['replan_distance_threshold'] == 0.03
    assert parameters['place_height_offset'] == 0.0
    assert parameters['socket_x'] == fixture_x
    assert parameters['socket_y'] == fixture_y
    assert parameters['socket_z'] == 0.10
    assert fixture_z == 0.0
    assert parameters['socket_frame_id'] == 'world'


def test_physical_profile_enables_bounded_contact_aware_close():
    """Keep physical close parameters explicit and internally consistent."""
    profile = yaml.safe_load(PHYSICAL_PROFILE.read_text())
    bridge = profile['gripper_action_bridge_node']['ros__parameters']
    contacts = profile[
        'gazebo_grasp_contact_status_node'
    ]['ros__parameters']
    executor = profile[
        'physical_pick_place_executor_node'
    ]['ros__parameters']

    assert bridge['open_position'] == 0.04
    assert bridge['close_position'] == 0.0
    assert bridge['result_timeout_sec'] == 5.0
    assert bridge['contact_wait_timeout_sec'] == 1.0
    assert bridge['contact_freshness_timeout_sec'] == 0.25
    assert bridge['contact_settle_duration_sec'] == 0.20
    assert bridge['allow_contact_limited_close'] is True
    assert bridge['expected_target_object'] == contacts['target_object_name']
    assert executor['expected_target_object'] == contacts['target_object_name']
    assert contacts['contact_stale_timeout_sec'] == 0.25
    assert bridge['contact_settle_duration_sec'] <= (
        bridge['contact_wait_timeout_sec']
    )
    assert bridge['result_timeout_sec'] == (
        executor['gripper_command_timeout_sec']
    )


def test_physical_close_parameters_reach_runtime_nodes():
    """Load the physical YAML into bridge, contact, and executor nodes."""
    launch_text = PHYSICAL_EXECUTION_LAUNCH.read_text()
    assert launch_text.count('parameters=[params_file,') >= 3
    assert "'expected_target_object': LaunchConfiguration(" in launch_text
    assert "'target_object_name'" in launch_text
