"""Expected-mismatch regression for the current planning and Gazebo models."""

from pathlib import Path

from adaptive_assembly_sim.robot_model_parity import (
    builtin_panda_samples,
    compare_sources,
    DEFAULT_ARM_JOINTS,
)
import pytest


SIM_PACKAGE_DIR = Path(__file__).resolve().parents[1]
GAZEBO_XACRO = (
    SIM_PACKAGE_DIR / 'urdf' / 'panda_gazebo_ros2_control.urdf.xacro'
)


def _moveit_panda_xacro():
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        pytest.skip('ament_index_python is unavailable; ROS 2 is not sourced')
    try:
        share = Path(get_package_share_directory(
            'moveit_resources_panda_moveit_config'
        ))
    except PackageNotFoundError:
        pytest.skip(
            'moveit_resources_panda_moveit_config is not installed; '
            'skipping only the repository model integration regression'
        )
    source = share / 'config' / 'panda.urdf.xacro'
    if not source.is_file():
        pytest.skip(f'installed MoveIt Panda xacro is missing: {source}')
    return source


def test_current_moveit_and_gazebo_models_are_expected_to_mismatch_until_pr2():
    """Prove PR 1 diagnoses the mismatch that a follow-up PR must remove."""
    result = compare_sources(
        _moveit_panda_xacro(),
        GAZEBO_XACRO,
        reference_xacro_args=('ros2_control_hardware_type:=mock_components',),
        reference_base_link='panda_link0',
        candidate_base_link='panda_link0',
        reference_tool_link='panda_link8',
        candidate_tool_link='panda_hand',
        arm_joints=DEFAULT_ARM_JOINTS,
        samples=builtin_panda_samples(),
    )

    categories = {
        mismatch.category for mismatch in result.structural_mismatches
    }
    assert result.passed is False
    assert result.structural_mismatches
    assert 'origin_translation_mismatch' in categories
    assert 'origin_rotation_mismatch' in categories
    assert 'axis_mismatch' in categories
    assert result.fk_mismatch_count >= 1
