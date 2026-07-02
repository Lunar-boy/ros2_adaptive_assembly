#!/usr/bin/env python3
"""Publish a deterministic synthetic marker and validate detector output."""

import time


def main():
    """Run the generated-image check, or skip without OpenCV ArUco."""
    try:
        import cv2
        import numpy as np
        if not hasattr(cv2, 'aruco'):
            raise ImportError
    except (ImportError, ModuleNotFoundError):
        print('SKIP: OpenCV ArUco support is unavailable')
        return 0

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    from adaptive_assembly_perception.aruco_detector_node import ArucoDetectorNode

    class Fixture(Node):
        """Publish a calibrated synthetic camera stream and capture output."""

        def __init__(self):
            """Create fixture publishers, subscribers, and marker image."""
            super().__init__('synthetic_aruco_camera_fixture')
            self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
            self.info_pub = self.create_publisher(
                CameraInfo, '/camera/camera_info', 10)
            self.perceived = None
            self.target = None
            self.status = None
            self.transform = None
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.create_subscription(
                PoseStamped, '/perceived_target_pose',
                lambda message: setattr(self, 'perceived', message), 10)
            self.create_subscription(
                PoseStamped, '/target_pose',
                lambda message: setattr(self, 'target', message), 10)
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                String, '/aruco_detection_status',
                lambda message: setattr(self, 'status', message.data), qos)
            dictionary = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50)
            if hasattr(cv2.aruco, 'generateImageMarker'):
                marker = cv2.aruco.generateImageMarker(dictionary, 0, 200)
            else:
                marker = np.zeros((200, 200), dtype=np.uint8)
                cv2.aruco.drawMarker(dictionary, 0, 200, marker, 1)
            self.canvas = np.full((480, 640), 255, dtype=np.uint8)
            self.canvas[140:340, 220:420] = marker
            self.timer = self.create_timer(0.1, self.publish_fixture)

        def publish_fixture(self):
            """Publish one matched CameraInfo and mono image pair."""
            stamp = self.get_clock().now().to_msg()
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = 'simulated_camera'
            info.width = 640
            info.height = 480
            info.k = [600.0, 0.0, 320.0,
                      0.0, 600.0, 240.0,
                      0.0, 0.0, 1.0]
            info.d = [0.0] * 5
            image = Image()
            image.header = info.header
            image.width = 640
            image.height = 480
            image.encoding = 'mono8'
            image.step = 640
            image.data = self.canvas.tobytes()
            self.info_pub.publish(info)
            self.image_pub.publish(image)

    rclpy.init()
    detector = ArucoDetectorNode()
    fixture = Fixture()
    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(fixture)
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if fixture.target is not None:
            try:
                fixture.transform = fixture.buffer.lookup_transform(
                    'world', 'target_object', Time())
            except TransformException:
                pass
        if (fixture.status is not None
                and fixture.status.startswith('event=success;')
                and fixture.perceived is not None and fixture.target is not None
                and fixture.transform is not None):
            break
    status = fixture.status
    perceived = fixture.perceived
    target = fixture.target
    transform = fixture.transform
    executor.shutdown()
    fixture.destroy_node()
    detector.destroy_node()
    rclpy.shutdown()
    if (status is None or not status.startswith('event=success;')
            or perceived is None or target is None or transform is None):
        print(f'FAIL: synthetic marker was not detected; status={status}')
        return 1
    if (perceived.header.frame_id != 'simulated_camera'
            or target.header.frame_id != 'world'):
        print('FAIL: synthetic detection used incorrect pose frames')
        return 1
    tf = transform.transform
    pose = target.pose
    errors = [pose.position.x - tf.translation.x,
              pose.position.y - tf.translation.y,
              pose.position.z - tf.translation.z,
              pose.orientation.x - tf.rotation.x,
              pose.orientation.y - tf.rotation.y,
              pose.orientation.z - tf.rotation.z,
              pose.orientation.w - tf.rotation.w]
    if sum(value * value for value in errors) ** 0.5 > 1e-5:
        print('FAIL: synthetic target pose and TF differ')
        return 1
    print('PASS: synthetic ArUco image produced consistent poses and TF')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
