"""Kinematically attach a Gazebo object to a gripper TF frame."""

import math
from typing import Dict, Optional

from geometry_msgs.msg import Pose
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener


def parse_status(value: str) -> Dict[str, str]:
    """Parse a semicolon-delimited project status message."""
    result = {}
    for item in value.split(';'):
        if '=' in item:
            key, field_value = item.split('=', 1)
            if key.strip():
                result[key.strip()] = field_value.strip()
    return result


def transform_to_pose(transform) -> Pose:
    """Convert a stamped TF transform to a Gazebo world pose."""
    pose = Pose()
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    pose.orientation = transform.transform.rotation
    return pose


class GazeboAttachDetachNode(Node):
    """Follow the configured gripper in Gazebo while logically attached."""

    def __init__(self) -> None:
        super().__init__('gazebo_attach_detach_node')
        defaults = {
            'object_grasp_state_topic': '/object_grasp_state',
            'object_grasp_attached_topic': '/object_grasp_attached',
            'status_topic': '/gazebo_attach_detach_status',
            'gazebo_object_attached_topic': '/gazebo_object_attached',
            'pose_error_mm_topic': '/gazebo_attach_pose_error_mm',
            'target_entity_name': 'target_object',
            'world_frame': 'world',
            'gripper_frame': 'panda_hand',
            'attach_update_period_sec': 0.05,
            'service_timeout_sec': 2.0,
            'enable_service_calls': True,
            'simulated_only': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self._values = {
            name: self.get_parameter(name).value for name in defaults
        }
        self._validate_parameters()

        retained = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, self._values['status_topic'], retained
        )
        self._attached_pub = self.create_publisher(
            Bool, self._values['gazebo_object_attached_topic'], retained
        )
        self._error_pub = self.create_publisher(
            Float64, self._values['pose_error_mm_topic'], retained
        )
        self.create_subscription(
            String, self._values['object_grasp_state_topic'],
            self._on_grasp_state, retained,
        )
        attached_topic = str(self._values['object_grasp_attached_topic'])
        if attached_topic:
            self.create_subscription(
                Bool, attached_topic, self._on_attached_bool, retained
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._service_name = '/world/adaptive_assembly_workcell/set_pose'
        self._client = self.create_client(SetEntityPose, self._service_name)
        self._attached = False
        self._pending = None
        self._deadline = None
        self._last_pose = None
        self.create_timer(
            float(self._values['attach_update_period_sec']), self._update
        )
        self.create_timer(0.05, self._check_pending)
        self._publish_attached(False)
        self._publish_status('ready')
        self.get_logger().info(
            'Gazebo attach/detach ready: '
            f"object={self._values['target_entity_name']}, "
            f"gripper={self._values['gripper_frame']}, "
            'enable_service_calls='
            f"{str(self._values['enable_service_calls']).lower()}, "
            'simulated_only=true, real_hardware=false'
        )

    def _validate_parameters(self) -> None:
        if not bool(self._values['simulated_only']):
            raise ValueError('simulated_only:=false is not supported')
        required = (
            'object_grasp_state_topic', 'status_topic',
            'gazebo_object_attached_topic', 'pose_error_mm_topic',
            'target_entity_name', 'world_frame', 'gripper_frame',
        )
        for name in required:
            if not str(self._values[name]):
                raise ValueError(f'{name} must not be empty')
        if float(self._values['attach_update_period_sec']) <= 0.0:
            raise ValueError(
                'attach_update_period_sec must be greater than zero'
            )
        if float(self._values['service_timeout_sec']) <= 0.0:
            raise ValueError('service_timeout_sec must be greater than zero')

    def _on_grasp_state(self, message: String) -> None:
        fields = parse_status(message.data)
        event_object = fields.get(
            'object', self._values['target_entity_name']
        )
        if event_object != self._values['target_entity_name']:
            return
        event = fields.get('event')
        if event == 'attached':
            self._set_attached(True)
        elif event == 'detached':
            self._set_attached(False)

    def _on_attached_bool(self, message: Bool) -> None:
        # The string event is authoritative; this optional input only repairs
        # state if a producer supplies the bool without its companion event.
        if bool(message.data) != self._attached:
            self._set_attached(bool(message.data))

    def _set_attached(self, attached: bool) -> None:
        if attached == self._attached:
            return
        self._attached = attached
        self._publish_attached(attached)
        if attached:
            self._publish_status(
                'attached', parent=self._values['gripper_frame']
            )
            self._update()
        else:
            self._last_pose = None
            self._publish_status(
                'detached', parent=self._values['world_frame']
            )
            self._error_pub.publish(Float64(data=0.0))

    def _update(self) -> None:
        if not self._attached or self._pending is not None:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._values['world_frame'], self._values['gripper_frame'],
                Time()
            )
        except TransformException as error:
            self.get_logger().debug(f'Gripper TF unavailable: {error}')
            self._publish_status('skipped', 'tf_unavailable')
            self._error_pub.publish(Float64(data=math.nan))
            return
        pose = transform_to_pose(transform)
        self._last_pose = pose
        if not bool(self._values['enable_service_calls']):
            self._publish_status('skipped', 'service_calls_disabled')
            self._error_pub.publish(Float64(data=0.0))
            return
        if not self._client.service_is_ready():
            self._publish_status('skipped', 'gazebo_service_unavailable')
            self._error_pub.publish(Float64(data=math.nan))
            return
        request = SetEntityPose.Request()
        request.entity.name = self._values['target_entity_name']
        request.entity.type = Entity.MODEL
        request.pose = pose
        self._pending = self._client.call_async(request)
        self._deadline = self.get_clock().now() + Duration(
            seconds=float(self._values['service_timeout_sec'])
        )

    def _check_pending(self) -> None:
        if self._pending is None:
            return
        if self._pending.done():
            future = self._pending
            self._pending = None
            try:
                if future.result().success:
                    self._publish_status(
                        'attached', parent=self._values['gripper_frame']
                    )
                    self._error_pub.publish(Float64(data=0.0))
                else:
                    self._publish_status('failure', 'gazebo_request_rejected')
                    self._error_pub.publish(Float64(data=math.nan))
            except Exception as error:
                self.get_logger().error(
                    f'Gazebo set-pose call failed: {error}'
                )
                self._publish_status('failure', 'gazebo_service_error')
            return
        if self.get_clock().now() >= self._deadline:
            self._pending.cancel()
            self._pending = None
            self._publish_status('failure', 'gazebo_service_timeout')
            self._error_pub.publish(Float64(data=math.nan))

    def _publish_attached(self, value: bool) -> None:
        self._attached_pub.publish(Bool(data=value))

    def _publish_status(
        self, event: str, reason: Optional[str] = None,
        parent: Optional[str] = None,
    ) -> None:
        fields = [f'event={event}', 'mode=gazebo_attach_detach']
        if reason:
            fields.append(f'reason={reason}')
        fields.append(f"object={self._values['target_entity_name']}")
        if parent:
            fields.append(f'parent={parent}')
        fields.extend(['simulated_only=true', 'real_hardware=false'])
        value = ';'.join(fields)
        self._status_pub.publish(String(data=value))
        if event in ('failure',):
            self.get_logger().error(value)
        elif event not in ('skipped',):
            self.get_logger().info(value)


def main(args=None) -> None:
    """Run the simulator-only Gazebo attach/detach node."""
    rclpy.init(args=args)
    node = GazeboAttachDetachNode()
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
