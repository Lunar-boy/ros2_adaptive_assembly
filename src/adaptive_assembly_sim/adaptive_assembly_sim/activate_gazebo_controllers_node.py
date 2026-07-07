"""Activate and verify the simulator-only Panda controllers."""

import sys
import time

from controller_manager_msgs.srv import ListControllers, SwitchController
import rclpy
from rclpy.node import Node


class ControllerActivator(Node):
    """Use controller-manager services without relying on ROS CLI discovery."""

    def __init__(self) -> None:
        super().__init__('activate_gazebo_controllers_node')
        self.declare_parameter('controller_manager_name', '/controller_manager')
        self.declare_parameter(
            'controllers', ['joint_state_broadcaster', 'panda_arm_controller']
        )
        self.declare_parameter('timeout_sec', 60.0)

    def run(self) -> bool:
        manager = str(self.get_parameter('controller_manager_name').value).rstrip('/')
        controllers = list(self.get_parameter('controllers').value)
        timeout_sec = float(self.get_parameter('timeout_sec').value)
        deadline = time.monotonic() + timeout_sec
        switch_client = self.create_client(
            SwitchController, f'{manager}/switch_controller'
        )
        list_client = self.create_client(
            ListControllers, f'{manager}/list_controllers'
        )
        while time.monotonic() < deadline:
            if switch_client.wait_for_service(timeout_sec=0.2) and \
                    list_client.wait_for_service(timeout_sec=0.2):
                break
        else:
            self.get_logger().error(
                f'Controller manager services unavailable at {manager}'
            )
            return False

        request = SwitchController.Request()
        request.activate_controllers = controllers
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        remaining = max(0.0, deadline - time.monotonic())
        request.timeout.sec = int(remaining)
        request.timeout.nanosec = int((remaining % 1.0) * 1_000_000_000)
        future = switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done() or future.result() is None or not future.result().ok:
            self.get_logger().error('Failed to activate Panda controllers')
            return False

        future = list_client.call_async(ListControllers.Request())
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done() or future.result() is None:
            self.get_logger().error('Failed to verify Panda controller states')
            return False
        states = {
            controller.name: controller.state
            for controller in future.result().controller
        }
        inactive = [name for name in controllers if states.get(name) != 'active']
        if inactive:
            self.get_logger().error(
                f'Controllers not active after switch: {inactive}; states={states}'
            )
            return False
        self.get_logger().info(
            'Activated and verified simulator controllers: '
            + ', '.join(controllers)
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerActivator()
    try:
        result = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if result else 1)


if __name__ == '__main__':
    main()
