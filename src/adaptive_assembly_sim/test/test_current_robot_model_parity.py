"""Expected-pass regression for the current planning and Gazebo models."""

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


def test_current_moveit_and_gazebo_models_have_kinematic_parity():
    """Require canonical structure and FK parity at all nonzero samples."""
    result = compare_sources(
        _moveit_panda_xacro(),
        GAZEBO_XACRO,
        reference_xacro_args=('ros2_control_hardware_type:=mock_components',),
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
