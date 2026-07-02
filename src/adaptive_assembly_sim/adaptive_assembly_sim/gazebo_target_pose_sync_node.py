"""Synchronize the fake-perception target pose into Gazebo Sim."""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from std_msgs.msg import Float64, String


def pose_stamped_to_gazebo_request(
    message: PoseStamped, entity_name: str
) -> SetEntityPose.Request:
    """Convert a validated ROS pose into a Gazebo model pose request."""
    request = SetEntityPose.Request()
    request.entity.name = entity_name
    request.entity.type = Entity.MODEL
    request.pose = message.pose
    return request


class GazeboTargetPoseSyncNode(Node):
    """Mirror `/target_pose` into one simulator-only Gazebo entity."""

    def __init__(self) -> None:
        """Declare configuration and create the ROS/Gazebo interfaces."""
        super().__init__('gazebo_target_pose_sync_node')
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('target_entity_name', 'target_object')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('status_topic', '/gazebo_target_sync_status')
        self.declare_parameter(
            'pose_error_mm_topic', '/gazebo_target_pose_error_mm'
        )
        self.declare_parameter(
            'pose_error_deg_topic', '/gazebo_target_pose_error_deg'
        )
        self.declare_parameter('service_timeout_sec', 2.0)
        self.declare_parameter('simulated_only', True)
        self.declare_parameter('enable_service_calls', True)

        self._target_pose_topic = self.get_parameter('target_pose_topic').value
        self._entity_name = self.get_parameter('target_entity_name').value
        self._world_frame = self.get_parameter('world_frame').value
        self._status_topic = self.get_parameter('status_topic').value
        self._error_mm_topic = self.get_parameter('pose_error_mm_topic').value
        self._error_deg_topic = self.get_parameter(
            'pose_error_deg_topic'
        ).value
        self._timeout = float(self.get_parameter('service_timeout_sec').value)
        self._simulated_only = self.get_parameter('simulated_only').value
        self._enable_calls = self.get_parameter('enable_service_calls').value
        self._validate_parameters()

        retained_qos = QoSProfile(depth=1)
        retained_qos.reliability = ReliabilityPolicy.RELIABLE
        retained_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_publisher = self.create_publisher(
            String, self._status_topic, retained_qos
        )
        self._error_mm_publisher = self.create_publisher(
            Float64, self._error_mm_topic, retained_qos
        )
        self._error_deg_publisher = self.create_publisher(
            Float64, self._error_deg_topic, retained_qos
        )
        self._service_name = '/world/adaptive_assembly_workcell/set_pose'
        self._client = self.create_client(SetEntityPose, self._service_name)
        self._pending_future = None
        self._deadline = None
        self._subscription = self.create_subscription(
            PoseStamped, self._target_pose_topic, self._on_target_pose, 10
        )
        self._timer = self.create_timer(0.05, self._check_pending_call)
        self.get_logger().info(
            'Gazebo target sync configured: '
            f'entity={self._entity_name}, source={self._target_pose_topic}, '
            f'world_frame={self._world_frame}, service={self._service_name}, '
            f'enable_service_calls={str(self._enable_calls).lower()}, '
            'simulated_only=true, real_hardware=false'
        )

    def _validate_parameters(self) -> None:
        if not self._simulated_only:
            raise ValueError('simulated_only:=false is not supported')
        for name, value in (
            ('target_pose_topic', self._target_pose_topic),
            ('target_entity_name', self._entity_name),
            ('world_frame', self._world_frame),
            ('status_topic', self._status_topic),
            ('pose_error_mm_topic', self._error_mm_topic),
            ('pose_error_deg_topic', self._error_deg_topic),
        ):
            if not value:
                raise ValueError(f'{name} must not be empty')
        if self._timeout <= 0.0:
            raise ValueError('service_timeout_sec must be greater than zero')

    def _on_target_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id != self._world_frame:
            self._publish_terminal('failure', 'invalid_frame')
            return
        if not self._valid_pose(message):
            self._publish_terminal('failure', 'invalid_pose')
            return
        request = pose_stamped_to_gazebo_request(message, self._entity_name)
        if not self._enable_calls:
            self._publish_terminal('skipped', 'service_calls_disabled')
            return
        if self._pending_future is not None:
            self._publish_terminal('skipped', 'request_in_progress')
            return
        if not self._client.service_is_ready():
            self._publish_terminal('skipped', 'gazebo_service_unavailable')
            return
        self._pending_future = self._client.call_async(request)
        self._deadline = self.get_clock().now() + Duration(
            seconds=self._timeout
        )

    @staticmethod
    def _valid_pose(message: PoseStamped) -> bool:
        values = (
            message.pose.position.x, message.pose.position.y,
            message.pose.position.z, message.pose.orientation.x,
            message.pose.orientation.y, message.pose.orientation.z,
            message.pose.orientation.w,
        )
        norm = sum(value * value for value in values[3:])
        return all(math.isfinite(value) for value in values) and norm > 1e-12

    def _check_pending_call(self) -> None:
        if self._pending_future is None:
            return
        if self._pending_future.done():
            future = self._pending_future
            self._pending_future = None
            try:
                response = future.result()
                if response.success:
                    self._publish_terminal('success')
                else:
                    self._publish_terminal(
                        'failure', 'gazebo_request_rejected'
                    )
            except Exception as error:
                self.get_logger().error(f'Gazebo pose service failed: {error}')
                self._publish_terminal('failure', 'gazebo_service_error')
            return
        if self.get_clock().now() >= self._deadline:
            self._pending_future.cancel()
            self._pending_future = None
            self._publish_terminal('failure', 'gazebo_service_timeout')

    def _publish_terminal(
        self, event: str, reason: Optional[str] = None
    ) -> None:
        fields = [f'event={event}', 'mode=gazebo_target_sync']
        if reason:
            fields.append(f'reason={reason}')
        fields.extend([
            f'entity={self._entity_name}',
            f'source_topic={self._target_pose_topic}',
            'simulated_only=true',
            'real_hardware=false',
        ])
        status = String(data=';'.join(fields))
        self._status_publisher.publish(status)
        error = 0.0 if event == 'success' else math.nan
        self._error_mm_publisher.publish(Float64(data=error))
        self._error_deg_publisher.publish(Float64(data=error))
        log = (
            self.get_logger().info if event != 'failure'
            else self.get_logger().error
        )
        log(status.data)


def main(args=None) -> None:
    """Run the Gazebo target pose synchronization node."""
    rclpy.init(args=args)
    node = GazeboTargetPoseSyncNode()
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
