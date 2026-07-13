"""Expected-pass regression for the current planning and Gazebo models."""

from pathlib import Path
import xml.etree.ElementTree as ElementTree

from adaptive_assembly_sim.robot_model_parity import (
    builtin_panda_samples,
    compare_sources,
    DEFAULT_ARM_JOINTS,
    expand_urdf_source,
    load_urdf_model,
)


SIM_PACKAGE_DIR = Path(__file__).resolve().parents[1]
MOVEIT_XACRO = SIM_PACKAGE_DIR / 'urdf' / 'panda.urdf.xacro'
GAZEBO_XACRO = (
    SIM_PACKAGE_DIR / 'urdf' / 'panda_gazebo_ros2_control.urdf.xacro'
)


def test_canonical_assembly_tcp_has_derived_fixed_hand_transform():
    """Lock the project TCP to the symmetric Panda finger-contact midpoint."""
    model = load_urdf_model(MOVEIT_XACRO)
    tcp_joint = model.child_to_joint['assembly_tcp']

    assert list(model.links).count('assembly_tcp') == 1
    assert tcp_joint.name == 'panda_hand_to_assembly_tcp'
    assert tcp_joint.joint_type == 'fixed'
    assert tcp_joint.parent == 'panda_hand'
    assert tcp_joint.child == 'assembly_tcp'
    assert tcp_joint.origin_xyz == (0.0, 0.0, 0.1034)
    assert tcp_joint.origin_rpy == (0.0, 0.0, 0.0)

    for source in (MOVEIT_XACRO, GAZEBO_XACRO):
        root = ElementTree.fromstring(expand_urdf_source(source))
        assert len(root.findall("./link[@name='assembly_tcp']")) == 1
        assert len(root.findall(
            "./joint[@name='panda_hand_to_assembly_tcp']"
        )) == 1


def test_current_moveit_and_gazebo_models_have_kinematic_parity():
    """Require canonical structure and FK parity at all nonzero samples."""
    result = compare_sources(
        MOVEIT_XACRO,
        GAZEBO_XACRO,
        reference_base_link='panda_link0',
        candidate_base_link='panda_link0',
        reference_tool_link='panda_link8',
        candidate_tool_link='panda_link8',
        arm_joints=DEFAULT_ARM_JOINTS,
        samples=builtin_panda_samples(),
    )

    assert result.passed is True
    assert result.structural_mismatches == ()
    assert result.fk_mismatch_count == 0
    assert len(result.fk_results) == 3
    assert all(sample.passed for sample in result.fk_results)


def test_current_moveit_and_gazebo_models_have_assembly_tcp_parity():
    """Require the fixed grasp-center chain and FK to match exactly."""
    result = compare_sources(
        MOVEIT_XACRO,
        GAZEBO_XACRO,
        reference_base_link='panda_link0',
        candidate_base_link='panda_link0',
        reference_tool_link='assembly_tcp',
        candidate_tool_link='assembly_tcp',
        arm_joints=DEFAULT_ARM_JOINTS,
        samples=builtin_panda_samples(),
    )

    assert result.passed is True
    assert result.structural_mismatches == ()
    assert len(result.fk_results) == 3
    assert all(sample.passed for sample in result.fk_results)
