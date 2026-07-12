"""Regression tests for fixed-socket task pose generation."""

from adaptive_assembly_task.assembly_task_node import AssemblyTaskNode
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy


class _PoseRecorder:
    """Capture poses published by the task node without spinning ROS."""

    def __init__(self):
        """Start with no captured pose."""
        self.messages = []

    def publish(self, message):
        """Store each published pose message."""
        self.messages.append(message)


def _target_pose():
    """Return a target deliberately separated from the assembly socket."""
    message = PoseStamped()
    message.header.frame_id = 'world'
    message.pose.position.x = 0.442
    message.pose.position.y = 0.148
    message.pose.position.z = 0.15
    message.pose.orientation.w = 1.0
    return message


def _last(recorder):
    """Return the most recently captured pose."""
    assert recorder.messages
    return recorder.messages[-1]


def test_fixed_socket_keeps_grasp_at_target_and_places_at_socket():
    """Keep target-based grasp stages separate from socket placement stages."""
    rclpy.init(args=[
        '--ros-args',
        '-p', 'assembly_pose_mode:=fixed_socket',
        '-p', 'socket_x:=0.62',
        '-p', 'socket_y:=-0.18',
        '-p', 'socket_z:=0.10',
        '-p', 'socket_yaw:=0.0',
        '-p', 'socket_frame_id:=world',
    ])
    node = AssemblyTaskNode()
    try:
        recorders = {
            name: _PoseRecorder() for name in (
                '_grasp_publisher', '_pre_grasp_publisher', '_lift_publisher',
                '_assembly_publisher', '_object_place_publisher',
                '_pre_place_publisher', '_place_publisher', '_retreat_publisher',
            )
        }
        for name, recorder in recorders.items():
            setattr(node, name, recorder)

        target = _target_pose()
        node._target_pose_callback(target)

        grasp = _last(recorders['_grasp_publisher']).pose.position
        pre_grasp = _last(recorders['_pre_grasp_publisher']).pose.position
        lift = _last(recorders['_lift_publisher']).pose.position
        assembly = _last(recorders['_assembly_publisher']).pose.position
        object_place = _last(recorders['_object_place_publisher']).pose.position
        pre_place = _last(recorders['_pre_place_publisher']).pose.position
        place = _last(recorders['_place_publisher']).pose.position
        retreat = _last(recorders['_retreat_publisher']).pose.position

        assert grasp.x == pytest.approx(target.pose.position.x)
        assert grasp.y == pytest.approx(target.pose.position.y)
        assert pre_grasp.x == pytest.approx(target.pose.position.x)
        assert pre_grasp.y == pytest.approx(target.pose.position.y)
        assert lift.x == pytest.approx(target.pose.position.x)
        assert lift.y == pytest.approx(target.pose.position.y)
        assert place.x == pytest.approx(0.62)
        assert place.y == pytest.approx(-0.18)
        assert object_place.x == pytest.approx(place.x)
        assert object_place.y == pytest.approx(place.y)
        assert assembly.x == pytest.approx(place.x)
        assert assembly.y == pytest.approx(place.y)
        assert pre_place.x == pytest.approx(place.x)
        assert pre_place.y == pytest.approx(place.y)
        assert retreat.x == pytest.approx(place.x)
        assert retreat.y == pytest.approx(place.y)
        assert pre_place.z > place.z
        assert retreat.z > place.z
        assert (grasp.x, grasp.y) != pytest.approx((place.x, place.y))
    finally:
        node.destroy_node()
        rclpy.shutdown()
