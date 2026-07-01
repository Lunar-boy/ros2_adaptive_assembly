"""Evaluate final insertion pose accuracy without contact simulation."""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String


class ContactLiteInsertionEvaluatorNode(Node):
    """Publish deterministic geometric insertion results."""

    def __init__(self) -> None:
        """Declare parameters and create ROS interfaces."""
        super().__init__('contact_lite_insertion_evaluator_node')
        defaults = {
            'target_pose_topic': '/panda_assembly_pose',
            'achieved_pose_topic': '/panda_assembly_pose',
            'position_tolerance_mm': 5.0,
            'orientation_tolerance_deg': 5.0,
            'require_execution_success': False,
            'achieved_pose_source': 'planned_pose',
            'status_topic': '/assembly_insertion_status',
            'success_topic': '/assembly_insertion_success',
            'position_error_topic': '/assembly_insertion_error_mm',
            'orientation_error_topic': '/assembly_insertion_error_deg',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self._position_tolerance_mm = float(values['position_tolerance_mm'])
        self._orientation_tolerance_deg = float(
            values['orientation_tolerance_deg']
        )
        self._require_execution_success = bool(
            values['require_execution_success']
        )
        self._achieved_pose_source = str(values['achieved_pose_source'])
        if self._position_tolerance_mm < 0.0:
            raise ValueError('position_tolerance_mm must be non-negative')
        if self._orientation_tolerance_deg < 0.0:
            raise ValueError('orientation_tolerance_deg must be non-negative')
        if not self._achieved_pose_source:
            raise ValueError('achieved_pose_source must not be empty')

        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, str(values['status_topic']), result_qos
        )
        self._success_publisher = self.create_publisher(
            Bool, str(values['success_topic']), result_qos
        )
        self._position_publisher = self.create_publisher(
            Float64, str(values['position_error_topic']), result_qos
        )
        self._orientation_publisher = self.create_publisher(
            Float64, str(values['orientation_error_topic']), result_qos
        )

        self._target_pose: Optional[PoseStamped] = None
        self._achieved_pose: Optional[PoseStamped] = None
        self._execution_success: Optional[bool] = None
        self._planning_status = 'unavailable'
        self._execution_status = 'unavailable'
        target_topic = str(values['target_pose_topic'])
        achieved_topic = str(values['achieved_pose_topic'])
        if target_topic == achieved_topic:
            self.create_subscription(
                PoseStamped, target_topic, self._shared_pose_callback, 10
            )
        else:
            self.create_subscription(
                PoseStamped, target_topic, self._target_pose_callback, 10
            )
            self.create_subscription(
                PoseStamped, achieved_topic, self._achieved_pose_callback, 10
            )
        self.create_subscription(
            String,
            '/assembly_sequence_planning_status',
            self._planning_status_callback,
            result_qos,
        )
        self.create_subscription(
            String,
            '/assembly_ros2_control_execution_status',
            self._execution_status_callback,
            result_qos,
        )
        self.create_subscription(
            Bool,
            '/assembly_ros2_control_execution_success',
            self._execution_success_callback,
            result_qos,
        )
        self.get_logger().info(
            'Contact-lite insertion evaluator ready: '
            f"target_topic='{target_topic}', "
            f"achieved_topic='{achieved_topic}', "
            f'position_tolerance_mm={self._position_tolerance_mm:.3f}, '
            'orientation_tolerance_deg='
            f'{self._orientation_tolerance_deg:.3f}, '
            'execution_required='
            f'{str(self._require_execution_success).lower()}, '
            f'achieved_pose_source={self._achieved_pose_source}, '
            'real_hardware=false'
        )

    def _shared_pose_callback(self, message: PoseStamped) -> None:
        """Use one planned pose as target and achieved pose."""
        self._target_pose = message
        self._achieved_pose = message
        self._evaluate()

    def _target_pose_callback(self, message: PoseStamped) -> None:
        self._target_pose = message
        self._evaluate()

    def _achieved_pose_callback(self, message: PoseStamped) -> None:
        self._achieved_pose = message
        self._evaluate()

    def _planning_status_callback(self, message: String) -> None:
        self._planning_status = message.data

    def _execution_status_callback(self, message: String) -> None:
        self._execution_status = message.data

    def _execution_success_callback(self, message: Bool) -> None:
        self._execution_success = message.data
        self._evaluate()

    @staticmethod
    def _orientation_error_deg(
        target: PoseStamped,
        achieved: PoseStamped,
    ) -> float:
        target_q = target.pose.orientation
        achieved_q = achieved.pose.orientation
        target_norm = math.sqrt(
            target_q.x ** 2 + target_q.y ** 2
            + target_q.z ** 2 + target_q.w ** 2
        )
        achieved_norm = math.sqrt(
            achieved_q.x ** 2 + achieved_q.y ** 2
            + achieved_q.z ** 2 + achieved_q.w ** 2
        )
        if target_norm == 0.0 or achieved_norm == 0.0:
            return 180.0
        dot = (
            target_q.x * achieved_q.x + target_q.y * achieved_q.y
            + target_q.z * achieved_q.z + target_q.w * achieved_q.w
        ) / (target_norm * achieved_norm)
        dot = max(-1.0, min(1.0, abs(dot)))
        return math.degrees(2.0 * math.acos(dot))

    def _evaluate(self) -> None:
        if self._target_pose is None or self._achieved_pose is None:
            return
        if self._require_execution_success and self._execution_success is None:
            return
        target = self._target_pose.pose.position
        achieved = self._achieved_pose.pose.position
        position_error_mm = 1000.0 * math.sqrt(
            (target.x - achieved.x) ** 2
            + (target.y - achieved.y) ** 2
            + (target.z - achieved.z) ** 2
        )
        orientation_error_deg = self._orientation_error_deg(
            self._target_pose, self._achieved_pose
        )
        execution_ok = (
            not self._require_execution_success
            or self._execution_success is True
        )
        success = (
            position_error_mm <= self._position_tolerance_mm
            and orientation_error_deg <= self._orientation_tolerance_deg
            and execution_ok
        )
        execution_success = (
            'unavailable' if self._execution_success is None
            else str(self._execution_success).lower()
        )
        status = (
            'event=insertion_evaluated;mode=contact_lite_insertion;'
            f'success={str(success).lower()};'
            f'position_error_mm={position_error_mm:.6f};'
            f'orientation_error_deg={orientation_error_deg:.6f};'
            f'position_tolerance_mm={self._position_tolerance_mm:.6f};'
            f'orientation_tolerance_deg={self._orientation_tolerance_deg:.6f};'
            'execution_required='
            f'{str(self._require_execution_success).lower()};'
            f'execution_success={execution_success};'
            f'achieved_pose_source={self._achieved_pose_source};'
            'real_hardware=false'
        )
        self._status_publisher.publish(String(data=status))
        self._success_publisher.publish(Bool(data=success))
        self._position_publisher.publish(Float64(data=position_error_mm))
        self._orientation_publisher.publish(
            Float64(data=orientation_error_deg)
        )
        self.get_logger().info(status)


def main(args=None) -> None:
    """Run the contact-lite insertion evaluator."""
    rclpy.init(args=args)
    node = ContactLiteInsertionEvaluatorNode()
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
