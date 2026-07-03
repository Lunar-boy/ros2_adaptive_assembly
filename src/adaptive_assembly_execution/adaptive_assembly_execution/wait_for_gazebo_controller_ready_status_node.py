"""Exit successfully only after retained controller-ready success status."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def parse_status(text: str) -> dict[str, str]:
    """Parse deterministic semicolon-delimited status fields."""
    return dict(
        fragment.split('=', 1) for fragment in text.split(';') if '=' in fragment
    )


class ControllerReadyStatusWaiter(Node):
    """Provide a bounded process-exit event for ROS launch ordering."""

    def __init__(self) -> None:
        super().__init__('wait_for_gazebo_controller_ready_status_node')
        self.declare_parameter(
            'status_topic', '/gazebo_controller_ready_status'
        )
        self.declare_parameter('timeout_sec', 65.0)
        self.result = 1
        self._started = time.monotonic()
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscription = self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._callback,
            qos,
        )
        self._timer = self.create_timer(0.1, self._timeout)

    def _callback(self, message: String) -> None:
        fields = parse_status(message.data)
        if fields.get('mode') != 'gazebo_controller_ready':
            return
        event = fields.get('event')
        if event == 'success':
            self.result = 0
            rclpy.shutdown()
        elif event == 'failure':
            self.get_logger().error(message.data)
            rclpy.shutdown()

    def _timeout(self) -> None:
        if time.monotonic() - self._started >= float(
            self.get_parameter('timeout_sec').value
        ):
            self.get_logger().error('controller readiness status wait timed out')
            rclpy.shutdown()


def main(args=None) -> None:
    """Wait for success and expose it as process exit code zero."""
    rclpy.init(args=args)
    node = ControllerReadyStatusWaiter()
    try:
        rclpy.spin(node)
    finally:
        result = node.result
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(result)


if __name__ == '__main__':
    main()
