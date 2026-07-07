from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node


class FakePandaFingerJointStateNode(Node):
    def __init__(self):
        super().__init__('fake_panda_finger_joint_state_node')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_fingers)

    def publish_fingers(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['panda_finger_joint1', 'panda_finger_joint2']
        msg.position = [0.04, 0.04]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakePandaFingerJointStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
