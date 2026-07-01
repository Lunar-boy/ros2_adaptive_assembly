"""Map simulator execution events to a logical gripper lifecycle."""

from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String


def parse_status(status: str) -> Dict[str, str]:
    """Parse the project's semicolon-delimited status format."""
    fields = {}
    for item in status.split(';'):
        if '=' not in item:
            continue
        key, value = item.split('=', 1)
        if key.strip():
            fields[key.strip()] = value.strip()
    return fields


class LogicalGraspLifecycleNode(Node):
    """Publish deterministic logical gripper and object-grasp states."""

    def __init__(self) -> None:
        """Declare parameters, create interfaces, and publish open state."""
        super().__init__('logical_grasp_lifecycle_node')
        defaults = {
            'stage_status_topic': (
                '/assembly_ros2_control_execution_stage_status'
            ),
            'execution_status_topic': (
                '/assembly_ros2_control_execution_status'
            ),
            'gripper_command_topic': '/gripper_command',
            'gripper_command_status_topic': '/gripper_command_status',
            'object_grasp_state_topic': '/object_grasp_state',
            'object_grasp_attached_topic': '/object_grasp_attached',
            'lifecycle_status_topic': '/logical_grasp_lifecycle_status',
            'object_id': 'target_object',
            'gripper_id': 'panda_hand',
            'attach_parent_frame': 'panda_hand',
            'release_parent_frame': 'world',
            'detach_on_failure': False,
            'simulated_only': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self._object_id = str(self.get_parameter('object_id').value)
        self._gripper_id = str(self.get_parameter('gripper_id').value)
        self._attach_parent = str(
            self.get_parameter('attach_parent_frame').value
        )
        self._release_parent = str(
            self.get_parameter('release_parent_frame').value
        )
        self._detach_on_failure = bool(
            self.get_parameter('detach_on_failure').value
        )
        if not bool(self.get_parameter('simulated_only').value):
            raise ValueError(
                'simulated_only must remain true; real hardware is not '
                'supported'
            )
        for name in (
            'object_id', 'gripper_id', 'attach_parent_frame',
            'release_parent_frame',
        ):
            if not str(self.get_parameter(name).value):
                raise ValueError(f'{name} must not be empty')

        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._command_publisher = self.create_publisher(
            String,
            self.get_parameter('gripper_command_topic').value,
            state_qos,
        )
        self._command_status_publisher = self.create_publisher(
            String,
            self.get_parameter('gripper_command_status_topic').value,
            state_qos,
        )
        self._object_state_publisher = self.create_publisher(
            String,
            self.get_parameter('object_grasp_state_topic').value,
            state_qos,
        )
        self._attached_publisher = self.create_publisher(
            Bool,
            self.get_parameter('object_grasp_attached_topic').value,
            state_qos,
        )
        self._lifecycle_publisher = self.create_publisher(
            String,
            self.get_parameter('lifecycle_status_topic').value,
            state_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter('stage_status_topic').value,
            self._stage_status_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('execution_status_topic').value,
            self._execution_status_callback,
            state_qos,
        )

        self._attached = False
        self._terminal = False
        self._publish_command('open', 'startup')
        self._publish_detached('startup')
        self._publish_lifecycle('ready', 'startup')
        self.get_logger().info(
            'Logical grasp lifecycle ready: '
            f"object='{self._object_id}', gripper='{self._gripper_id}', "
            f'detach_on_failure={self._detach_on_failure}, '
            'simulated_only=true, gazebo_attach=false, real_hardware=false.'
        )

    def _stage_status_callback(self, message: String) -> None:
        """Close and logically attach after pre-grasp stage success."""
        fields = parse_status(message.data)
        if (
            self._terminal
            or self._attached
            or fields.get('event') != 'success'
            or fields.get('stage') != 'pre_grasp'
        ):
            return
        self._publish_command('close', 'pre_grasp_success')
        self._attached = True
        self._publish_object_state(
            'attached', self._attach_parent, 'pre_grasp_success'
        )
        self._publish_attached(True)
        self._publish_lifecycle('attached', 'pre_grasp_success')

    def _execution_status_callback(self, message: String) -> None:
        """Release on success or report a deterministic terminal failure."""
        if self._terminal:
            return
        fields = parse_status(message.data)
        event = fields.get('event')
        if event == 'success':
            self._terminal = True
            self._publish_command('open', 'execution_success')
            self._attached = False
            self._publish_object_state(
                'detached', self._release_parent, 'execution_success'
            )
            self._publish_attached(False)
            self._publish_lifecycle('released', 'execution_success')
        elif event in ('failure', 'timeout'):
            self._terminal = True
            reason = fields.get('reason', event)
            if self._detach_on_failure and self._attached:
                self._publish_command('open', 'execution_failure')
                self._attached = False
                self._publish_object_state(
                    'detached', self._release_parent, 'execution_failure'
                )
                self._publish_attached(False)
            self._publish_lifecycle('failure', reason)

    def _publish_command(self, command: str, trigger: str) -> None:
        command_text = (
            f'event=command;command={command};gripper={self._gripper_id};'
            'simulated=true;real_hardware=false'
        )
        self._publish_string(self._command_publisher, command_text)
        status = (
            f'event=success;command={command};gripper={self._gripper_id};'
            f'trigger={trigger};logical=true;simulated=true;'
            'real_hardware=false'
        )
        self._publish_string(self._command_status_publisher, status)

    def _publish_detached(self, trigger: str) -> None:
        self._publish_object_state(
            'detached', self._release_parent, trigger
        )
        self._publish_attached(False)

    def _publish_object_state(
        self, event: str, parent: str, trigger: str
    ) -> None:
        status = (
            f'event={event};object={self._object_id};parent={parent};'
            f'trigger={trigger};logical=true;gazebo_attach=false;'
            'real_hardware=false'
        )
        self._publish_string(self._object_state_publisher, status)

    def _publish_attached(self, attached: bool) -> None:
        message = Bool()
        message.data = attached
        self._attached_publisher.publish(message)

    def _publish_lifecycle(self, event: str, reason: str) -> None:
        status = (
            f'event={event};reason={reason};object={self._object_id};'
            f'gripper={self._gripper_id};attached='
            f'{str(self._attached).lower()};logical=true;'
            'simulated_only=true;gazebo_attach=false;real_hardware=false'
        )
        self._publish_string(self._lifecycle_publisher, status)

    @staticmethod
    def _publish_string(publisher, value: str) -> None:
        message = String()
        message.data = value
        publisher.publish(message)


def main(args=None) -> None:
    """Run the simulator-only logical grasp lifecycle."""
    rclpy.init(args=args)
    node = LogicalGraspLifecycleNode()
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
