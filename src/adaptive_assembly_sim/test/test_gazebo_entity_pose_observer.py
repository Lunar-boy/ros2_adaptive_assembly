"""Unit tests for Gazebo scoped-name matching and pose extraction."""

from types import SimpleNamespace

from adaptive_assembly_sim.gazebo_entity_pose_observer_node import (
    _entity_matches,
    candidate_entity_names,
    extract_entity_pose,
    extract_pose_stamped,
    pose_is_finite,
)
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
import pytest
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize(
    ('name', 'matches'),
    [
        ('target_object', True),
        ('world::target_object', False),
        ('target_object_visual', False),
    ],
)
def test_exact_entity_matching(name, matches):
    """Strict matching preserves exact-name behavior."""
    assert _entity_matches(name, 'target_object', exact=True) is matches


@pytest.mark.parametrize(
    'name',
    [
        'target_object',
        'model::target_object',
        'adaptive_assembly_physical_workcell::target_object',
        'world::adaptive_assembly_physical_workcell::target_object',
        'adaptive_assembly_physical_workcell/target_object',
        'world/adaptive_assembly_physical_workcell/model/target_object',
        '/world/adaptive_assembly_physical_workcell/model/target_object',
    ],
)
def test_scoped_entity_matching(name):
    """Non-exact matching accepts complete Gazebo scope components."""
    assert _entity_matches(name, 'target_object', exact=False)


@pytest.mark.parametrize(
    'name',
    [
        'target_object_visual',
        'target_object_collision',
        'other_target_object',
        'target_object_backup',
    ],
)
def test_scoped_entity_matching_rejects_false_positives(name):
    """Non-exact matching never falls back to a substring match."""
    assert not _entity_matches(name, 'target_object', exact=False)


def test_extract_scoped_tf_message_pose():
    """Select a scoped TF child and copy its full transform."""
    ignored = TransformStamped()
    ignored.child_frame_id = 'target_object_visual'
    ignored.transform.translation.x = 99.0

    target = TransformStamped()
    target.child_frame_id = (
        'world/adaptive_assembly_physical_workcell/model/target_object'
    )
    target.transform.translation.x = 0.41
    target.transform.translation.y = -0.12
    target.transform.translation.z = 0.73
    target.transform.rotation.x = 0.1
    target.transform.rotation.y = 0.2
    target.transform.rotation.z = 0.3
    target.transform.rotation.w = 0.9

    pose, reason = extract_entity_pose(
        TFMessage(transforms=[ignored, target]),
        'target_object',
        exact=False,
    )

    assert reason is None
    assert pose is not None
    assert (pose.position.x, pose.position.y, pose.position.z) == (
        0.41, -0.12, 0.73
    )
    assert (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) == (0.1, 0.2, 0.3, 0.9)


def test_extract_pose_vector_shape():
    """Support the ros_gz_interfaces Pose_V field representation."""
    ignored = SimpleNamespace(
        name='target_object_collision',
        position=Point(x=99.0),
        orientation=Quaternion(w=1.0),
    )
    target = SimpleNamespace(
        name='world::adaptive_assembly_physical_workcell::target_object',
        position=Point(x=0.2, y=0.3, z=0.4),
        orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9),
    )

    pose, reason = extract_entity_pose(
        SimpleNamespace(pose=[ignored, target]),
        'target_object',
        exact=False,
    )

    assert reason is None
    assert pose is not None
    assert (pose.position.x, pose.position.y, pose.position.z) == (
        0.2, 0.3, 0.4
    )
    assert pose.orientation.w == 0.9


def test_extract_dedicated_pose_stamped_preserves_pose():
    """Dedicated input is already entity-selected and preserves its pose."""
    message = PoseStamped()
    message.header.frame_id = 'gazebo_ignored_source_frame'
    message.pose.position.x = 0.35
    message.pose.position.y = 0.18
    message.pose.position.z = 0.10
    message.pose.orientation.x = 0.1
    message.pose.orientation.y = 0.2
    message.pose.orientation.z = 0.3
    message.pose.orientation.w = 0.9

    pose, reason = extract_pose_stamped(message)

    assert reason is None
    assert pose is not None
    assert (pose.position.x, pose.position.y, pose.position.z) == (
        0.35, 0.18, 0.10
    )
    assert (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) == (0.1, 0.2, 0.3, 0.9)


@pytest.mark.parametrize('field', ['x', 'y', 'z'])
def test_non_finite_pose_position_is_rejected(field):
    """Non-finite position data cannot make the observer available."""
    message = PoseStamped()
    message.pose.orientation.w = 1.0
    setattr(message.pose.position, field, float('nan'))

    pose, reason = extract_pose_stamped(message)

    assert reason is None
    assert pose is not None
    assert not pose_is_finite(pose)


def test_extract_reports_malformed_matching_candidate():
    """Report malformed data after selecting the intended target."""
    candidate = SimpleNamespace(
        child_frame_id='world::target_object',
        transform=SimpleNamespace(translation=None, rotation=Quaternion()),
    )

    pose, reason = extract_entity_pose(
        SimpleNamespace(transforms=[candidate]),
        'target_object',
        exact=False,
    )

    assert pose is None
    assert reason == 'unsupported_pose_structure'


def test_extract_reports_target_absent():
    """Report entity_not_found when no semantic target component exists."""
    candidate = TransformStamped()
    candidate.child_frame_id = 'world::target_object_backup'

    pose, reason = extract_entity_pose(
        TFMessage(transforms=[candidate]),
        'target_object',
        exact=False,
    )

    assert pose is None
    assert reason == 'entity_not_found'


def test_extract_reports_names_unavailable():
    """Identify TF bridge output that omits every entity/frame name."""
    candidate = TransformStamped()

    pose, reason = extract_entity_pose(
        TFMessage(transforms=[candidate]),
        'target_object',
        exact=False,
    )

    assert pose is None
    assert reason == 'entity_names_unavailable'


def test_candidate_diagnostics_are_bounded():
    """Cap diagnostic candidate count and individual name length."""
    transforms = []
    for index in range(8):
        candidate = TransformStamped()
        candidate.child_frame_id = f'world::{index}::' + 'x' * 120
        transforms.append(candidate)

    names = candidate_entity_names(TFMessage(transforms=transforms))

    assert len(names) == 5
    assert all(len(name) <= 96 for name in names)


def test_pose_stamped_has_no_vector_candidate_names():
    """A single PoseStamped pose is not mistaken for a Pose_V array."""
    assert candidate_entity_names(PoseStamped()) == ()
