"""Tests for Gazebo model-center to task-reference pose adaptation."""

from adaptive_assembly_sim.gazebo_target_pose_adapter_node import (
    adapt_target_pose,
    GazeboTargetPoseAdapterNode,
    pose_is_valid,
    validate_adapter_parameters,
)
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def _source_pose() -> PoseStamped:
    source = PoseStamped()
    source.header.frame_id = 'gazebo_world'
    source.pose.position.x = 0.35
    source.pose.position.y = 0.18
    source.pose.position.z = 0.10
    source.pose.orientation.x = 0.1
    source.pose.orientation.y = -0.2
    source.pose.orientation.z = 0.3
    source.pose.orientation.w = 0.9
    return source


def test_adaptation_preserves_xy_and_adds_reference_z_offset():
    """Convert the cylinder center at z=0.10 to its top at z=0.15."""
    output = adapt_target_pose(_source_pose(), 0.05, 'world', Time(sec=7))

    assert output.pose.position.x == pytest.approx(0.35)
    assert output.pose.position.y == pytest.approx(0.18)
    assert output.pose.position.z == pytest.approx(0.15)


def test_adaptation_preserves_orientation():
    """Keep the complete observed Gazebo orientation unchanged."""
    source = _source_pose()
    output = adapt_target_pose(source, 0.05, 'world', Time())

    assert output.pose.orientation == source.pose.orientation


def test_adaptation_overrides_frame_label_and_uses_supplied_stamp():
    """Apply the configured label without claiming a TF transformation."""
    stamp = Time(sec=12, nanosec=34)
    output = adapt_target_pose(_source_pose(), 0.05, 'world', stamp)

    assert output.header.frame_id == 'world'
    assert output.header.stamp == stamp


@pytest.mark.parametrize(
    ('input_topic', 'output_topic', 'offset', 'frame'),
    [
        ('', '/target_pose', 0.05, 'world'),
        ('/gazebo_target_object_pose', '', 0.05, 'world'),
        ('/gazebo_target_object_pose', '/target_pose', float('nan'), 'world'),
        ('/gazebo_target_object_pose', '/target_pose', float('inf'), 'world'),
        ('/gazebo_target_object_pose', '/target_pose', 0.05, ''),
    ],
)
def test_invalid_parameters_are_rejected(
    input_topic, output_topic, offset, frame
):
    """Reject empty names and non-finite reference offsets."""
    with pytest.raises(ValueError):
        validate_adapter_parameters(
            input_topic, output_topic, offset, frame
        )


def test_invalid_source_pose_is_rejected_before_conversion():
    """Do not create output from a source pose with invalid coordinates."""
    source = _source_pose()
    source.pose.position.x = float('nan')

    assert not pose_is_valid(source)
    with pytest.raises(ValueError):
        adapt_target_pose(source, 0.05, 'world', Time())


def test_node_publishes_nothing_before_source_pose():
    """Remain silent until Gazebo provides the first valid observation."""
    context = Context()
    rclpy.init(context=context)
    adapter = GazeboTargetPoseAdapterNode(
        context=context,
        parameter_overrides=[
            Parameter('input_pose_topic', value='/test_gazebo_source_pose'),
            Parameter('output_pose_topic', value='/test_task_target_pose'),
        ],
    )
    listener = Node(
        'gazebo_target_pose_adapter_test_listener', context=context
    )
    received = []
    subscription = listener.create_subscription(
        PoseStamped,
        '/test_task_target_pose',
        received.append,
        10,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(adapter)
    executor.add_node(listener)

    try:
        executor.spin_once(timeout_sec=0.2)
        assert not received
    finally:
        executor.remove_node(listener)
        executor.remove_node(adapter)
        listener.destroy_subscription(subscription)
        listener.destroy_node()
        adapter.destroy_node()
        executor.shutdown()
        context.shutdown()
