"""Compute task-level poses from perceived assembly target poses."""

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class AssemblyTaskNode(Node):
    """Publish pre-grasp, grasp, and assembly poses for each target."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, and target pose subscription."""
        super().__init__('assembly_task_node')

        self.declare_parameter('pre_grasp_height_offset', 0.20)
        self.declare_parameter('grasp_height_offset', 0.05)
        self.declare_parameter('grasp_pose_topic', '/grasp_pose')
        self.declare_parameter('assembly_height_offset', 0.05)
        self.declare_parameter('replan_distance_threshold', 0.03)
        self.declare_parameter('assembly_pose_mode', 'target_offset')
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
        self._replan_distance_threshold = self.get_parameter(
            'replan_distance_threshold'
        ).value
        self._assembly_pose_mode = str(
            self.get_parameter('assembly_pose_mode').value
        )
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
        self._grasp_publisher = self.create_publisher(
            PoseStamped, self._grasp_pose_topic, 10
        )
        self._target_subscription = self.create_subscription(
            PoseStamped, '/target_pose', self._target_pose_callback, 10
        )
        self._previous_target_pose = None
        self.get_logger().info(
            f'Assembly task configured: assembly_pose_mode='
            f'{self._assembly_pose_mode}'
        )

    def _validate_parameters(self) -> None:
        """Reject a negative replanning distance threshold."""
        if self._replan_distance_threshold < 0.0:
            raise ValueError(
                'replan_distance_threshold must be greater than or equal to '
                'zero'
            )
        if self._grasp_height_offset < 0.0:
            raise ValueError(
                'grasp_height_offset must be greater than or equal to zero'
            )
        if not self._grasp_pose_topic:
            raise ValueError('grasp_pose_topic must not be empty')
        if self._assembly_pose_mode not in ('target_offset', 'fixed_socket'):
            raise ValueError(
                'assembly_pose_mode must be target_offset or fixed_socket'
            )
        if not self._socket_frame_id:
            raise ValueError('socket_frame_id must not be empty')

    def _target_pose_callback(self, target_pose: PoseStamped) -> None:
        """Compute and publish task poses for a received target pose."""
        position = target_pose.pose.position
        self.get_logger().info(
            'Received target pose: '
            f'x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}'
        )

        self._log_replanning_decision(target_pose)

        pre_grasp_pose = self._offset_pose(
            target_pose, self._pre_grasp_height_offset
        )
        grasp_pose = self._offset_pose(target_pose, self._grasp_height_offset)
        if self._assembly_pose_mode == 'fixed_socket':
            assembly_pose = self._fixed_socket_pose(target_pose)
        else:
            assembly_pose = self._offset_pose(
                target_pose, self._assembly_height_offset
            )

        self._pre_grasp_publisher.publish(pre_grasp_pose)
        self._grasp_publisher.publish(grasp_pose)
        self._assembly_publisher.publish(assembly_pose)

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

        self._previous_target_pose = target_pose

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

    def _log_replanning_decision(self, target_pose: PoseStamped) -> None:
        """Log whether target movement requires planning or replanning."""
        if self._previous_target_pose is None:
            self.get_logger().info(
                'First target pose accepted; initial planning is required'
            )
            return

        current = target_pose.pose.position
        previous = self._previous_target_pose.pose.position
        distance = math.sqrt(
            (current.x - previous.x) ** 2
            + (current.y - previous.y) ** 2
            + (current.z - previous.z) ** 2
        )

        if distance > self._replan_distance_threshold:
            self.get_logger().info(
                'Target moved '
                f'{distance:.3f} m; replanning is required'
            )
        else:
            self.get_logger().info(
                'Target moved '
                f'{distance:.3f} m; target change is small and no replanning '
                'is required'
            )

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
