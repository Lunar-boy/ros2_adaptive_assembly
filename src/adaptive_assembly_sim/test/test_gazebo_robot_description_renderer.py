"""Unit tests for the Gazebo-only Panda mimic transformation."""

import xml.etree.ElementTree as ElementTree

from adaptive_assembly_sim.gazebo_robot_description_renderer import (
    GazeboRobotDescriptionError,
    remove_second_finger_mimic,
    render_gazebo_robot_description,
)
import pytest


VALID_XML = """<robot name="panda">
  <link name="unrelated"><inertial><mass value="1.2"/></inertial></link>
  <joint name="panda_finger_joint1" type="prismatic">
    <origin xyz="0 0 0.0584" rpy="0 0 0"/>
    <axis xyz="0 1 0"/><limit lower="0" upper="0.04" effort="20" velocity="0.2"/>
  </joint>
  <joint name="panda_finger_joint2" type="prismatic">
    <origin xyz="0 0 0.0584" rpy="0 0 3.14159"/>
    <axis xyz="0 1 0"/><limit lower="0" upper="0.04" effort="20" velocity="0.2"/>
    <mimic joint="panda_finger_joint1" multiplier="1" offset="0"/>
  </joint>
  <transmission name="finger_transmission"><type>example</type></transmission>
  <ros2_control name="GazeboSystem" type="system">
    <hardware><plugin>example</plugin></hardware>
  </ros2_control>
</robot>"""


def test_removes_only_expected_mimic_and_preserves_unrelated_structure():
    """Preserve links, joint fields, and control blocks byte-for-byte by tree."""
    original = ElementTree.fromstring(VALID_XML)
    expected_joint = original.find("./joint[@name='panda_finger_joint2']")
    expected_joint.remove(expected_joint.find('mimic'))

    rendered = ElementTree.fromstring(remove_second_finger_mimic(VALID_XML))

    assert ElementTree.tostring(rendered) == ElementTree.tostring(original)
    assert rendered.find("./joint[@name='panda_finger_joint1']/mimic") is None
    assert rendered.find("./joint[@name='panda_finger_joint2']/mimic") is None


@pytest.mark.parametrize(
    ('xml_text', 'message'),
    [
        ('<robot/>', 'missing required joint'),
        (
            VALID_XML.replace(
                '<mimic joint="panda_finger_joint1" '
                'multiplier="1" offset="0"/>', ''
            ),
            'missing its expected mimic',
        ),
        (
            VALID_XML.replace(
                'joint="panda_finger_joint1" multiplier',
                'joint="wrong_joint" multiplier',
            ),
            "mimics 'wrong_joint'",
        ),
        (
            VALID_XML.replace(
                '</robot>',
                '<joint name="panda_finger_joint2" '
                'type="prismatic"/></robot>',
            ),
            'found 2 joints',
        ),
        ('<robot><joint></robot>', 'failed to parse'),
    ],
)
def test_invalid_expanded_descriptions_fail_loudly(xml_text, message):
    """Reject missing, wrong, duplicate, and malformed finger descriptions."""
    with pytest.raises(GazeboRobotDescriptionError, match=message):
        remove_second_finger_mimic(xml_text)


def test_xacro_expansion_failure_is_wrapped(tmp_path):
    """Report xacro failures rather than returning an unmodified model."""
    broken = tmp_path / 'broken.urdf.xacro'
    broken.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:missing/></robot>'
    )
    with pytest.raises(GazeboRobotDescriptionError, match='failed to expand'):
        render_gazebo_robot_description(str(broken))
