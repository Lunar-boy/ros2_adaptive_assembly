"""Detect ArUco markers in simulator camera images when OpenCV is present."""

import math
import time

from geometry_msgs.msg import PoseStamped, TransformStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image

from std_msgs.msg import Bool, String

from tf2_ros import TransformBroadcaster


def _quaternion_from_matrix(matrix):
    """Return an xyzw quaternion for a 3x3 rotation matrix."""
    trace = float(matrix[0, 0] + matrix[1, 1] + matrix[2, 2])
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale, scale / 4.0)
    diagonal = [matrix[0, 0], matrix[1, 1], matrix[2, 2]]
    index = max(range(3), key=lambda item: diagonal[item])
    if index == 0:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1]
                          - matrix[2, 2]) * 2.0
        return (scale / 4.0, (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale)
    if index == 1:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0]
                          - matrix[2, 2]) * 2.0
        return ((matrix[0, 1] + matrix[1, 0]) / scale, scale / 4.0,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale)
    scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0]
                      - matrix[1, 1]) * 2.0
    return ((matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[1, 2] + matrix[2, 1]) / scale, scale / 4.0,
            (matrix[1, 0] - matrix[0, 1]) / scale)


class ArucoDetectorNode(Node):
    """Provide an optional, simulator-only OpenCV ArUco perception path."""

    def __init__(self) -> None:
        """Declare configuration and activate detection when available."""
        super().__init__('aruco_detector_node')
        defaults = {
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'target_pose_topic': '/target_pose',
            'perceived_pose_topic': '/perceived_target_pose',
            'status_topic': '/aruco_detection_status',
            'marker_detected_topic': '/aruco_marker_detected',
            'world_frame': 'world',
            'camera_frame': 'simulated_camera',
            'target_frame_id': 'target_object',
            'marker_id': 0,
            'marker_size_m': 0.05,
            'aruco_dictionary': 'DICT_4X4_50',
            'camera_x': 0.0, 'camera_y': 0.0, 'camera_z': 1.0,
            'camera_yaw': 0.0,
            'publish_tf': True,
            'fallback_to_emulator': True,
            'detection_timeout_sec': 2.0,
            'simulated_only': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self._values = {name: self.get_parameter(name).value
                        for name in defaults}
        self._validate()

        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            String, self._values['status_topic'], retained)
        self._detected_pub = self.create_publisher(
            Bool, self._values['marker_detected_topic'], retained)
        self._perceived_pub = self.create_publisher(
            PoseStamped, self._values['perceived_pose_topic'], 10)
        self._target_pub = self.create_publisher(
            PoseStamped, self._values['target_pose_topic'], 10)
        self._tf = TransformBroadcaster(self)
        self._camera_matrix = None
        self._distortion = None
        self._last_image_time = None
        self._last_detection_time = None
        self._last_timeout_reason = None
        self._cv2 = None
        self._np = None
        self._dictionary = None
        self._detector = None

        try:
            import cv2  # Optional dependency; intentionally imported here.
            import numpy
            if not hasattr(cv2, 'aruco'):
                raise ImportError('cv2.aruco is unavailable')
            dictionary_name = str(self._values['aruco_dictionary'])
            dictionary_id = getattr(cv2.aruco, dictionary_name, None)
            if dictionary_id is None:
                raise ValueError(
                    f'unknown aruco_dictionary: {dictionary_name}')
            self._cv2 = cv2
            self._np = numpy
            self._dictionary = cv2.aruco.getPredefinedDictionary(
                dictionary_id)
            if hasattr(cv2.aruco, 'ArucoDetector'):
                parameters = cv2.aruco.DetectorParameters()
                self._detector = cv2.aruco.ArucoDetector(
                    self._dictionary, parameters)
        except (ImportError, ModuleNotFoundError):
            self._publish_detected(False)
            self._publish_status('skipped', 'opencv_unavailable')
            self.get_logger().warning(
                'OpenCV ArUco unavailable; use simulated_marker_pose_node '
                'for the default headless fallback.')
            return
        except ValueError as error:
            self._publish_detected(False)
            self._publish_status('failure', 'invalid_aruco_dictionary')
            self.get_logger().error(str(error))
            return

        self.create_subscription(
            CameraInfo, self._values['camera_info_topic'],
            self._camera_info_callback, 10)
        self.create_subscription(
            Image, self._values['image_topic'], self._image_callback, 10)
        self.create_timer(0.25, self._check_timeout)
        self._publish_detected(False)
        self._publish_status('skipped', 'camera_info_unavailable')
        self.get_logger().info(
            'Optional simulator OpenCV ArUco detector ready: '
            f"marker_id={self._values['marker_id']}, "
            f"dictionary={self._values['aruco_dictionary']}")

    def _validate(self) -> None:
        if not self._values['simulated_only']:
            raise ValueError('simulated_only must remain true')
        if float(self._values['marker_size_m']) <= 0.0:
            raise ValueError('marker_size_m must be greater than zero')
        if float(self._values['detection_timeout_sec']) <= 0.0:
            raise ValueError('detection_timeout_sec must be greater than zero')
        required = ('image_topic', 'camera_info_topic', 'target_pose_topic',
                    'perceived_pose_topic', 'status_topic',
                    'marker_detected_topic', 'world_frame', 'camera_frame',
                    'target_frame_id', 'aruco_dictionary')
        if any(not str(self._values[name]) for name in required):
            raise ValueError('topic, frame, and dictionary values cannot be empty')

    def _publish_status(self, event: str, reason: str = '') -> None:
        fields = [f'event={event}', 'mode=opencv_aruco_detector']
        if reason:
            fields.append(f'reason={reason}')
        fields.extend(('simulated_only=true', 'real_hardware=false'))
        message = String()
        message.data = ';'.join(fields)
        self._status_pub.publish(message)

    def _publish_detected(self, value: bool) -> None:
        message = Bool()
        message.data = value
        self._detected_pub.publish(message)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        matrix = self._np.asarray(message.k, dtype=float).reshape(3, 3)
        if (not self._np.isfinite(matrix).all() or matrix[0, 0] <= 0.0
                or matrix[1, 1] <= 0.0 or matrix[2, 2] == 0.0):
            self._camera_matrix = None
            self._publish_detected(False)
            self._publish_status('failure', 'invalid_camera_model')
            return
        self._camera_matrix = matrix
        self._distortion = self._np.asarray(message.d, dtype=float)

    def _image_callback(self, message: Image) -> None:
        self._last_image_time = time.monotonic()
        if self._camera_matrix is None:
            self._publish_detected(False)
            self._publish_status('skipped', 'camera_info_unavailable')
            return
        image = self._decode_image(message)
        if image is None:
            self._publish_detected(False)
            self._publish_status('failure', 'unsupported_image_encoding')
            return
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(image)
        else:
            corners, ids, _ = self._cv2.aruco.detectMarkers(
                image, self._dictionary)
        if ids is None or int(self._values['marker_id']) not in ids.flatten():
            self._publish_detected(False)
            self._publish_status('skipped', 'marker_not_detected')
            return
        index = list(ids.flatten()).index(int(self._values['marker_id']))
        object_points = self._np.asarray([
            [-0.5, 0.5, 0.0], [0.5, 0.5, 0.0],
            [0.5, -0.5, 0.0], [-0.5, -0.5, 0.0],
        ], dtype=self._np.float32) * float(self._values['marker_size_m'])
        ok, rvec, tvec = self._cv2.solvePnP(
            object_points, corners[index].reshape(4, 2), self._camera_matrix,
            self._distortion, flags=self._cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not self._np.isfinite(tvec).all():
            self._publish_detected(False)
            self._publish_status('failure', 'pose_estimation_failed')
            return
        rotation, _ = self._cv2.Rodrigues(rvec)
        quaternion = _quaternion_from_matrix(rotation)
        self._publish_pose(message, tvec.reshape(3), quaternion)

    def _decode_image(self, message: Image):
        channels = {'mono8': 1, '8UC1': 1, 'bgr8': 3, 'rgb8': 3}.get(
            message.encoding)
        if channels is None or message.height == 0 or message.width == 0:
            return None
        row_bytes = int(message.width) * channels
        if int(message.step) < row_bytes:
            return None
        data = self._np.frombuffer(message.data, dtype=self._np.uint8)
        needed = int(message.step) * int(message.height)
        if data.size < needed:
            return None
        rows = data[:needed].reshape(int(message.height), int(message.step))
        compact = rows[:, :row_bytes]
        if channels == 1:
            return compact.reshape(int(message.height), int(message.width))
        image = compact.reshape(int(message.height), int(message.width), 3)
        if message.encoding == 'rgb8':
            return self._cv2.cvtColor(image, self._cv2.COLOR_RGB2BGR)
        return image

    def _publish_pose(self, image: Image, translation, quaternion) -> None:
        stamp = image.header.stamp
        perceived = PoseStamped()
        perceived.header.stamp = stamp
        perceived.header.frame_id = self._values['camera_frame']
        perceived.pose.position.x = float(translation[0])
        perceived.pose.position.y = float(translation[1])
        perceived.pose.position.z = float(translation[2])
        (perceived.pose.orientation.x, perceived.pose.orientation.y,
         perceived.pose.orientation.z, perceived.pose.orientation.w) = quaternion
        self._perceived_pub.publish(perceived)

        yaw = float(self._values['camera_yaw'])
        cy, sy = math.cos(yaw), math.sin(yaw)
        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = self._values['world_frame']
        target.pose.position.x = float(self._values['camera_x']) + (
            cy * translation[0] - sy * translation[1])
        target.pose.position.y = float(self._values['camera_y']) + (
            sy * translation[0] + cy * translation[1])
        target.pose.position.z = float(self._values['camera_z']) + translation[2]
        half = yaw / 2.0
        qz, qw = math.sin(half), math.cos(half)
        qx, qy, mz, mw = quaternion
        target.pose.orientation.x = qw * qx - qz * qy
        target.pose.orientation.y = qw * qy + qz * qx
        target.pose.orientation.z = qw * mz + qz * mw
        target.pose.orientation.w = qw * mw - qz * mz
        self._target_pub.publish(target)
        if self._values['publish_tf']:
            self._broadcast_transforms(target)
        self._last_detection_time = time.monotonic()
        self._last_timeout_reason = None
        self._publish_detected(True)
        message = String()
        message.data = (
            'event=success;mode=opencv_aruco_detector;'
            'source=simulated_camera_image;'
            f"marker_id={self._values['marker_id']};"
            f"target_frame={self._values['target_frame_id']};"
            'simulated_only=true;real_hardware=false')
        self._status_pub.publish(message)

    def _broadcast_transforms(self, target: PoseStamped) -> None:
        camera = TransformStamped()
        camera.header = target.header
        camera.child_frame_id = self._values['camera_frame']
        camera.transform.translation.x = float(self._values['camera_x'])
        camera.transform.translation.y = float(self._values['camera_y'])
        camera.transform.translation.z = float(self._values['camera_z'])
        half = float(self._values['camera_yaw']) / 2.0
        camera.transform.rotation.z = math.sin(half)
        camera.transform.rotation.w = math.cos(half)
        marker = TransformStamped()
        marker.header = target.header
        marker.child_frame_id = self._values['target_frame_id']
        marker.transform.translation.x = target.pose.position.x
        marker.transform.translation.y = target.pose.position.y
        marker.transform.translation.z = target.pose.position.z
        marker.transform.rotation = target.pose.orientation
        self._tf.sendTransform([camera, marker])

    def _check_timeout(self) -> None:
        now = time.monotonic()
        timeout = float(self._values['detection_timeout_sec'])
        if self._last_image_time is None or now - self._last_image_time > timeout:
            reason = 'image_input_unavailable'
        elif (self._last_detection_time is None
              or now - self._last_detection_time > timeout):
            reason = 'marker_not_detected'
        else:
            return
        if reason != self._last_timeout_reason:
            self._last_timeout_reason = reason
            self._publish_detected(False)
            self._publish_status('skipped', reason)


def main(args=None) -> None:
    """Run the optional ArUco detector node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
