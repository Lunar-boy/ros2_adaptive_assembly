"""Static contract tests for explicit sequence-planner target links."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE_DIR / 'src' / 'assembly_sequence_planning_node.cpp'
LAUNCH = PACKAGE_DIR / 'launch' / 'assembly_sequence_planning.launch.py'


def test_planner_declares_validates_and_explicitly_targets_configured_link():
    """Prevent fallback to MoveGroup's implicit end-effector selection."""
    source = SOURCE.read_text(encoding='utf-8')

    assert '"end_effector_link", "panda_link8"' in source
    assert 'hasLinkModel(end_effector_link_)' in source
    assert 'setEndEffectorLink(end_effector_link_)' in source
    assert 'setPoseTarget(snapshot[i].second, end_effector_link_)' in source
    assert 'configured_end_effector_link_invalid' in source
    assert ';end_effector_link=' in source
    assert 'setPoseTarget(pose);' not in source


def test_planner_launch_exposes_one_end_effector_link_argument():
    """Keep one launch name from the public wrapper to the node parameter."""
    source = LAUNCH.read_text(encoding='utf-8')

    declaration = "DeclareLaunchArgument(\n            'end_effector_link'"
    assert source.count(declaration) == 1
    assert "'end_effector_link': end_effector_link" in source
