"""Publish when the simulator Panda controller is fully usable."""

import math
import time
from typing import Dict, Optional

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class GazeboControllerReadyNode(Node):
    """Bounded, simulator-only readiness probe for Gazebo ros2_control."""

    def __init__(self) -> None:
        super().__init__('wait_for_gazebo_controller_ready_node')
        defaults = {
            'controller_manager_name': '/controller_manager',
            'joint_state_broadcaster_name': 'joint_state_broadcaster',
            'arm_controller_name': 'panda_arm_controller',
            'action_name': '/panda_arm_controller/follow_joint_trajectory',
            'joint_state_topic': '/joint_states',
            'expected_joint_prefix': 'panda_joint',
            'expected_joint_count': 7,
            'timeout_sec': 60.0,
            'status_topic': '/gazebo_controller_ready_status',
            'poll_period_sec': 0.2,
            'simulated_only': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if not bool(self.get_parameter('simulated_only').value):
            raise ValueError('simulated_only:=false is not supported')

        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._poll_period = float(self.get_parameter('poll_period_sec').value)
        self._expected_joints = {
            f"{self.get_parameter('expected_joint_prefix').value}{index}"
            for index in range(
                1, int(self.get_parameter('expected_joint_count').value) + 1
            )
        }
        if self._timeout_sec <= 0.0 or self._poll_period <= 0.0:
            raise ValueError('timeout_sec and poll_period_sec must be positive')
        if not self._expected_joints:
            raise ValueError('expected_joint_count must be positive')

        retained_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), retained_qos
        )
        manager = str(self.get_parameter('controller_manager_name').value)
        service_name = f"{manager.rstrip('/')}/list_controllers"
        self._controller_client = self.create_client(
            ListControllers, service_name
        )
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter('action_name').value),
        )
        self._joint_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self._joint_state_callback,
            10,
        )
        self._joint_state_broadcaster = str(
            self.get_parameter('joint_state_broadcaster_name').value
        )
        self._arm_controller = str(
            self.get_parameter('arm_controller_name').value
        )
        self._controllers: Dict[str, str] = {}
        self._list_future = None
        self._joint_states_received = False
        self._joint_stamp = 'unobserved'
        self._terminal = False
        self._last_reason: Optional[str] = None
        self._started = time.monotonic()
        self._timer = self.create_timer(self._poll_period, self._poll)
        self._publish('waiting', 'waiting_for_controller_manager')
        self.get_logger().info(
            'Gazebo controller readiness gate started: '
            f'service={service_name}, timeout_sec={self._timeout_sec:.3f}, '
            'simulated_only=true, real_hardware=false'
        )

    def _joint_state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        self._joint_states_received = all(
            name in positions and math.isfinite(float(positions[name]))
            for name in self._expected_joints
        )
        self._joint_stamp = f'{message.header.stamp.sec}.{message.header.stamp.nanosec:09d}'

    def _controllers_callback(self, future) -> None:
        self._list_future = None
        try:
            response = future.result()
        except Exception as error:  # service can disappear during Gazebo startup
            self.get_logger().debug(f'ListControllers request failed: {error}')
            return
        self._controllers = {
            controller.name: controller.state for controller in response.controller
        }

    def _poll(self) -> None:
        if self._terminal:
            return
        if time.monotonic() - self._started >= self._timeout_sec:
            self._terminal = True
            self._publish('failure', 'timeout')
            return
        if self._controller_client.service_is_ready() and self._list_future is None:
            self._list_future = self._controller_client.call_async(
                ListControllers.Request()
            )
            self._list_future.add_done_callback(self._controllers_callback)

        broadcaster_active = (
            self._controllers.get(self._joint_state_broadcaster) == 'active'
        )
        arm_active = self._controllers.get(self._arm_controller) == 'active'
        action_available = self._action_client.server_is_ready()
        if not self._controller_client.service_is_ready():
            reason = 'waiting_for_controller_manager'
        elif not broadcaster_active:
            reason = 'waiting_for_joint_state_broadcaster'
        elif not arm_active:
            reason = 'waiting_for_panda_arm_controller'
        elif not action_available:
            reason = 'waiting_for_action_server'
        elif not self._joint_states_received:
            reason = 'waiting_for_joint_states'
        else:
            self._terminal = True
            self._publish('success', '')
            return
        if reason != self._last_reason:
            self._publish('waiting', reason)

    def _publish(self, event: str, reason: str) -> None:
        broadcaster_active = (
            self._controllers.get(self._joint_state_broadcaster) == 'active'
        )
        arm_active = self._controllers.get(self._arm_controller) == 'active'
        fields = [
            f'event={event}',
            'mode=gazebo_controller_ready',
        ]
        if reason:
            fields.append(f'reason={reason}')
        fields.extend((
            f'joint_state_broadcaster_active={str(broadcaster_active).lower()}',
            f'panda_arm_controller_active={str(arm_active).lower()}',
            'controllers_active='
            f'{str(broadcaster_active and arm_active).lower()}',
            'action_server_available='
            f'{str(self._action_client.server_is_ready()).lower()}',
            f'joint_states_received={str(self._joint_states_received).lower()}',
            f'joint_state_stamp={self._joint_stamp}',
            'simulated_only=true',
            'real_hardware=false',
        ))
        message = String()
        message.data = ';'.join(fields)
        self._publisher.publish(message)
        if reason != self._last_reason or event != 'waiting':
            self.get_logger().info(message.data)
        self._last_reason = reason


def main(args=None) -> None:
    """Run the readiness gate and retain its terminal publisher."""
    rclpy.init(args=args)
    node = GazeboControllerReadyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
