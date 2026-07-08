"""Compute task-level poses from perceived assembly target poses."""

import math

from adaptive_assembly_task.grasp_sequence_schema import (
    format_grasp_candidates,
    format_grasp_sequence_status,
    generate_grasp_candidates,
    lift_z,
    select_candidate,
    validate_candidate_configuration,
)
from adaptive_assembly_task.replanning_gate import should_replan
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AssemblyTaskNode(Node):
    """Publish pre-grasp, grasp, and assembly poses for each target."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, and target pose subscription."""
        super().__init__('assembly_task_node')

        self.declare_parameter('pre_grasp_height_offset', 0.20)
        self.declare_parameter('grasp_height_offset', 0.05)
        self.declare_parameter('grasp_pose_topic', '/grasp_pose')
        self.declare_parameter('grasp_candidates_topic', '/grasp_candidates')
        self.declare_parameter('selected_grasp_pose_topic', '/selected_grasp_pose')
        self.declare_parameter('lift_pose_topic', '/lift_pose')
        self.declare_parameter('grasp_sequence_status_topic', '/grasp_sequence_status')
        self.declare_parameter('grasp_candidate_count', 4)
        self.declare_parameter('grasp_candidate_yaw_step_rad', math.pi / 2.0)
        self.declare_parameter('selected_grasp_candidate_index', 0)
        self.declare_parameter('lift_height_offset', 0.20)
        self.declare_parameter('preserve_target_orientation_for_candidates', False)
        self.declare_parameter('assembly_height_offset', 0.05)
        self.declare_parameter('replan_distance_threshold', 0.03)
        self.declare_parameter('assembly_pose_mode', 'target_offset')
        self.declare_parameter('object_place_pose_topic', '/object_place_pose')
        self.declare_parameter('pre_place_pose_topic', '/pre_place_pose')
        self.declare_parameter('place_pose_topic', '/place_pose')
        self.declare_parameter('retreat_pose_topic', '/retreat_pose')
        self.declare_parameter('pre_place_height_offset', 0.20)
        self.declare_parameter('place_height_offset', 0.00)
        self.declare_parameter('retreat_height_offset', 0.20)
        self.declare_parameter('socket_x', 0.62)
        self.declare_parameter('socket_y', -0.18)
        self.declare_parameter('socket_z', 0.10)
        self.declare_parameter('socket_yaw', 0.0)
        self.declare_parameter('socket_frame_id', 'world')

        self._pre_grasp_height_offset = self.get_parameter(
            'pre_grasp_height_offset'
        ).value
        self._assembly_height_offset = self.get_parameter(
            'assembly_height_offset'
        ).value
        self._grasp_height_offset = self.get_parameter(
            'grasp_height_offset'
        ).value
        self._grasp_pose_topic = str(
            self.get_parameter('grasp_pose_topic').value
        )
        self._grasp_candidates_topic = str(
            self.get_parameter('grasp_candidates_topic').value)
        self._selected_grasp_pose_topic = str(
            self.get_parameter('selected_grasp_pose_topic').value)
        self._lift_pose_topic = str(self.get_parameter('lift_pose_topic').value)
        self._grasp_sequence_status_topic = str(
            self.get_parameter('grasp_sequence_status_topic').value)
        self._grasp_candidate_count = int(
            self.get_parameter('grasp_candidate_count').value)
        self._grasp_candidate_yaw_step_rad = float(
            self.get_parameter('grasp_candidate_yaw_step_rad').value)
        self._selected_grasp_candidate_index = int(
            self.get_parameter('selected_grasp_candidate_index').value)
        self._lift_height_offset = float(
            self.get_parameter('lift_height_offset').value)
        self._preserve_target_orientation_for_candidates = bool(
            self.get_parameter('preserve_target_orientation_for_candidates').value)
        self._replan_distance_threshold = self.get_parameter(
            'replan_distance_threshold'
        ).value
        self._assembly_pose_mode = str(
            self.get_parameter('assembly_pose_mode').value
        )
        self._object_place_pose_topic = str(
            self.get_parameter('object_place_pose_topic').value
        )
        self._pre_place_pose_topic = str(self.get_parameter('pre_place_pose_topic').value)
        self._place_pose_topic = str(self.get_parameter('place_pose_topic').value)
        self._retreat_pose_topic = str(self.get_parameter('retreat_pose_topic').value)
        self._pre_place_height_offset = float(self.get_parameter('pre_place_height_offset').value)
        self._place_height_offset = float(self.get_parameter('place_height_offset').value)
        self._retreat_height_offset = float(self.get_parameter('retreat_height_offset').value)
        self._socket_x = float(self.get_parameter('socket_x').value)
        self._socket_y = float(self.get_parameter('socket_y').value)
        self._socket_z = float(self.get_parameter('socket_z').value)
        self._socket_yaw = float(self.get_parameter('socket_yaw').value)
        self._socket_frame_id = str(
            self.get_parameter('socket_frame_id').value
        )
        self._validate_parameters()

        self._pre_grasp_publisher = self.create_publisher(
            PoseStamped, '/pre_grasp_pose', 10
        )
        self._assembly_publisher = self.create_publisher(
            PoseStamped, '/assembly_pose', 10
        )
        self._object_place_publisher = self.create_publisher(
            PoseStamped, self._object_place_pose_topic, 10
        )
        self._grasp_publisher = self.create_publisher(
            PoseStamped, self._grasp_pose_topic, 10
        )
        self._grasp_candidates_publisher = self.create_publisher(
            String, self._grasp_candidates_topic, 10)
        self._selected_grasp_publisher = self.create_publisher(
            PoseStamped, self._selected_grasp_pose_topic, 10)
        self._lift_publisher = self.create_publisher(
            PoseStamped, self._lift_pose_topic, 10)
        self._grasp_sequence_status_publisher = self.create_publisher(
            String, self._grasp_sequence_status_topic, 10)
        self._pre_place_publisher = self.create_publisher(
            PoseStamped, self._pre_place_pose_topic, 10
        )
        self._place_publisher = self.create_publisher(
            PoseStamped, self._place_pose_topic, 10
        )
        self._retreat_publisher = self.create_publisher(
            PoseStamped, self._retreat_pose_topic, 10
        )
        self._status_publisher = self.create_publisher(String, '~/status', 10)
        self._target_subscription = self.create_subscription(
            PoseStamped, '/target_pose', self._target_pose_callback, 10
        )
        self._last_accepted_target_pose = None
        self.get_logger().info(
            f'Assembly task configured: assembly_pose_mode='
            f'{self._assembly_pose_mode}, object_place_pose_topic='
            f'{self._object_place_pose_topic}'
        )

    def _validate_parameters(self) -> None:
        """Validate task pose configuration parameters."""
        if self._grasp_height_offset < 0.0:
            raise ValueError(
                'grasp_height_offset must be greater than or equal to zero'
            )
        if not self._grasp_pose_topic:
            raise ValueError('grasp_pose_topic must not be empty')
        topic_names = (
            self._grasp_candidates_topic, self._selected_grasp_pose_topic,
            self._lift_pose_topic, self._grasp_sequence_status_topic,
        )
        if not all(topic_names):
            raise ValueError('grasp sequence topic names must not be empty')
        validate_candidate_configuration(
            self._grasp_candidate_count, self._selected_grasp_candidate_index)
        if not math.isfinite(self._grasp_candidate_yaw_step_rad):
            raise ValueError('grasp_candidate_yaw_step_rad must be finite')
        if self._lift_height_offset < 0.0:
            raise ValueError('lift_height_offset must be greater than or equal to zero')
        if self._assembly_pose_mode not in ('target_offset', 'fixed_socket'):
            raise ValueError(
                'assembly_pose_mode must be target_offset or fixed_socket'
            )
        if not self._object_place_pose_topic:
            raise ValueError('object_place_pose_topic must not be empty')
        if not all((self._pre_place_pose_topic, self._place_pose_topic, self._retreat_pose_topic)):
            raise ValueError('place sequence pose topics must not be empty')
        if not self._socket_frame_id:
            raise ValueError('socket_frame_id must not be empty')

    def _target_pose_callback(self, target_pose: PoseStamped) -> None:
        """Compute and publish task poses for a received target pose."""
        position = target_pose.pose.position
        self.get_logger().info(
            'Received target pose: '
            f'x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}'
        )

        distance = self._target_displacement(target_pose)
        if distance is not None and not should_replan(
            distance, self._replan_distance_threshold
        ):
            self._publish_status(
                'skipped_small_motion', target_pose, distance
            )
            self._publish_skipped_grasp_sequence_status(target_pose)
            self.get_logger().info(
                f'Target moved {distance:.3f} m; target change is small and '
                'planning targets were not updated'
            )
            return

        if distance is None:
            status = 'accepted_initial'
            self.get_logger().info(
                'First target pose accepted; initial planning is required'
            )
        elif self._replan_distance_threshold <= 0.0:
            status = 'accepted_threshold_disabled'
            self.get_logger().info(
                'Replanning distance gate is disabled; target accepted'
            )
        else:
            status = 'accepted_replan'
            self.get_logger().info(
                f'Target moved {distance:.3f} m; replanning is required'
            )

        candidates = generate_grasp_candidates(
            position.x, position.y, position.z + self._grasp_height_offset,
            (
                target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                target_pose.pose.orientation.z, target_pose.pose.orientation.w,
            ),
            self._grasp_candidate_count, self._grasp_candidate_yaw_step_rad,
            self._preserve_target_orientation_for_candidates,
        )
        selected = select_candidate(
            candidates, self._selected_grasp_candidate_index)
        grasp_pose = self._candidate_pose(target_pose, selected)
        pre_grasp_pose = self._offset_pose(
            grasp_pose,
            self._pre_grasp_height_offset - self._grasp_height_offset)
        lift_pose = self._offset_pose(grasp_pose, self._lift_height_offset)
        if self._assembly_pose_mode == 'fixed_socket':
            object_place_pose = self._fixed_socket_pose(target_pose)
            pre_place_pose = self._offset_pose(object_place_pose, self._pre_place_height_offset)
            place_pose = self._offset_pose(object_place_pose, self._place_height_offset)
            retreat_pose = self._offset_pose(object_place_pose, self._retreat_height_offset)
            assembly_pose = place_pose
        else:
            assembly_pose = self._offset_pose(
                target_pose, self._assembly_height_offset
            )
            object_place_pose = assembly_pose
            pre_place_pose = self._offset_pose(assembly_pose, self._pre_place_height_offset)
            place_pose = self._offset_pose(assembly_pose, self._place_height_offset)
            retreat_pose = self._offset_pose(assembly_pose, self._retreat_height_offset)

        self._pre_grasp_publisher.publish(pre_grasp_pose)
        candidates_message = String()
        candidates_message.data = format_grasp_candidates(
            candidates, self._selected_grasp_candidate_index,
            target_pose.header.frame_id)
        self._grasp_candidates_publisher.publish(candidates_message)
        self._selected_grasp_publisher.publish(grasp_pose)
        self._grasp_publisher.publish(grasp_pose)
        self._lift_publisher.publish(lift_pose)
        self._assembly_publisher.publish(assembly_pose)
        self._object_place_publisher.publish(object_place_pose)
        self._pre_place_publisher.publish(pre_place_pose)
        self._place_publisher.publish(place_pose)
        self._retreat_publisher.publish(retreat_pose)

        self.get_logger().info(
            'Computed grasp pose: '
            f'x={grasp_pose.pose.position.x:.3f}, '
            f'y={grasp_pose.pose.position.y:.3f}, '
            f'z={grasp_pose.pose.position.z:.3f}'
        )
        self.get_logger().info(
            'Computed pre-grasp pose: '
            f'x={pre_grasp_pose.pose.position.x:.3f}, '
            f'y={pre_grasp_pose.pose.position.y:.3f}, '
            f'z={pre_grasp_pose.pose.position.z:.3f}'
        )
        self.get_logger().info(
            f'Computed {self._assembly_pose_mode} assembly pose: '
            f'x={assembly_pose.pose.position.x:.3f}, '
            f'y={assembly_pose.pose.position.y:.3f}, '
            f'z={assembly_pose.pose.position.z:.3f}'
        )
        self.get_logger().info(
            f'Computed object place pose: '
            f'x={object_place_pose.pose.position.x:.3f}, '
            f'y={object_place_pose.pose.position.y:.3f}, '
            f'z={object_place_pose.pose.position.z:.3f}, '
            f'frame={object_place_pose.header.frame_id}'
        )
        for name, pose in (
            ('pre-place', pre_place_pose), ('place', place_pose),
            ('retreat', retreat_pose),
        ):
            self.get_logger().info(
                f'Computed {name} pose: x={pose.pose.position.x:.3f}, '
                f'y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}, '
                f'frame={pose.header.frame_id}'
            )

        self._last_accepted_target_pose = target_pose
        self._publish_status(status, target_pose, distance)
        self._publish_grasp_sequence_status(
            status, target_pose, grasp_pose, lift_pose, object_place_pose)

    def _publish_grasp_sequence_status(
        self, status, target_pose, selected_pose, lift_pose, object_place_pose
    ) -> None:
        """Publish the explicit schema status for an accepted sequence."""
        target = target_pose.pose.position
        selected = selected_pose.pose.position
        object_place = object_place_pose.pose.position
        message = String()
        message.data = format_grasp_sequence_status(
            status, self._selected_grasp_candidate_index,
            self._grasp_candidate_count, target_pose.header.frame_id,
            (target.x, target.y, target.z),
            (selected.x, selected.y, selected.z), lift_pose.pose.position.z,
            (object_place.x, object_place.y, object_place.z),
            self._assembly_pose_mode)
        self._grasp_sequence_status_publisher.publish(message)

    def _publish_skipped_grasp_sequence_status(self, target_pose) -> None:
        """Explain a skipped update without republishing sequence poses."""
        target = target_pose.pose.position
        last = self._last_accepted_target_pose
        selected_z = last.pose.position.z + self._grasp_height_offset
        if self._assembly_pose_mode == 'fixed_socket':
            object_place = (self._socket_x, self._socket_y, self._socket_z)
        else:
            object_place = (
                last.pose.position.x, last.pose.position.y,
                last.pose.position.z + self._assembly_height_offset)
        message = String()
        message.data = format_grasp_sequence_status(
            'skipped_small_motion', self._selected_grasp_candidate_index,
            self._grasp_candidate_count, target_pose.header.frame_id,
            (target.x, target.y, target.z),
            (last.pose.position.x, last.pose.position.y, selected_z),
            lift_z(selected_z, self._lift_height_offset), object_place,
            self._assembly_pose_mode)
        self._grasp_sequence_status_publisher.publish(message)

    def _fixed_socket_pose(self, target_pose: PoseStamped) -> PoseStamped:
        """Return the configured fixed socket pose with a yaw quaternion."""
        result = PoseStamped()
        result.header.stamp = target_pose.header.stamp
        result.header.frame_id = self._socket_frame_id
        result.pose.position.x = self._socket_x
        result.pose.position.y = self._socket_y
        result.pose.position.z = self._socket_z
        result.pose.orientation.z = math.sin(self._socket_yaw / 2.0)
        result.pose.orientation.w = math.cos(self._socket_yaw / 2.0)
        return result

    def _target_displacement(self, target_pose: PoseStamped):
        """Return displacement from the last published target, if any."""
        if self._last_accepted_target_pose is None:
            return None

        current = target_pose.pose.position
        previous = self._last_accepted_target_pose.pose.position
        return math.sqrt(
            (current.x - previous.x) ** 2
            + (current.y - previous.y) ** 2
            + (current.z - previous.z) ** 2
        )

    def _publish_status(
        self, status: str, target_pose: PoseStamped, distance
    ) -> None:
        """Publish the target acceptance decision in a stable text format."""
        distance_field = 'n/a' if distance is None else f'{distance:.6f}'
        message = String()
        message.data = (
            f'status={status};distance={distance_field};'
            f'threshold={self._replan_distance_threshold:.6f};'
            f'frame_id={target_pose.header.frame_id}'
        )
        self._status_publisher.publish(message)

    @staticmethod
    def _offset_pose(
        target_pose: PoseStamped, height_offset: float
    ) -> PoseStamped:
        """Return a pose with a vertical offset and copied orientation."""
        result = PoseStamped()
        result.header = target_pose.header
        result.pose.position.x = target_pose.pose.position.x
        result.pose.position.y = target_pose.pose.position.y
        result.pose.position.z = target_pose.pose.position.z + height_offset
        result.pose.orientation = target_pose.pose.orientation
        return result

    @staticmethod
    def _candidate_pose(target_pose, candidate) -> PoseStamped:
        """Convert one schema candidate into a PoseStamped message."""
        result = PoseStamped()
        result.header = target_pose.header
        result.pose.position.x = candidate.x
        result.pose.position.y = candidate.y
        result.pose.position.z = candidate.z
        result.pose.orientation.x = candidate.qx
        result.pose.orientation.y = candidate.qy
        result.pose.orientation.z = candidate.qz
        result.pose.orientation.w = candidate.qw
        return result


def main(args=None) -> None:
    """Run the assembly task node."""
    rclpy.init(args=args)
    node = AssemblyTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
