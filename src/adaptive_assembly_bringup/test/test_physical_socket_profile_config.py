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
