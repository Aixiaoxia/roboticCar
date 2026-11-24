"""Interactive time-aligned data recorder for camera-to-base calibration."""

import json
import os
import threading

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


class ExtrinsicRecorder:
    def __init__(self):
        self.node = rclpy.create_node('bunker_extrinsic_recorder')
        self.node.declare_parameter('image_topic', '/camera/color/image_raw')
        self.node.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.node.declare_parameter('detections_topic', '/apriltag/detections')
        self.node.declare_parameter('odom_frame', 'odom')
        self.node.declare_parameter('base_frame', 'base_link')
        self.node.declare_parameter('camera_link_frame', 'camera_link')
        self.node.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.node.declare_parameter('tag_frame', 'target_tag')
        self.node.declare_parameter('tag_id', 0)
        self.node.declare_parameter('output', 'camera_extrinsic_samples.jsonl')
        self.node.declare_parameter('tag_size', 0.15)
        # AprilTag processing can lag the newest camera image by several frames.
        # Pair against the detection timestamp and use that timestamp for both TFs.
        self.node.declare_parameter('max_sync_error', 0.25)
        self.node.declare_parameter('min_decision_margin', 20.0)

        p = self.node.get_parameters([
            'image_topic', 'camera_info_topic', 'detections_topic',
            'odom_frame', 'base_frame', 'camera_link_frame', 'camera_frame', 'tag_frame',
            'tag_id', 'output', 'tag_size',
            'max_sync_error', 'min_decision_margin'])
        self.image_topic, self.camera_info_topic = p[0].value, p[1].value
        self.detections_topic = p[2].value
        self.odom_frame, self.base_frame = p[3].value, p[4].value
        self.camera_link_frame, self.camera_frame = p[5].value, p[6].value
        self.tag_frame = p[7].value
        self.tag_id, self.output = p[8].value, os.path.expanduser(p[9].value)
        self.tag_size = float(p[10].value)
        self.max_sync_error, self.min_margin = p[11].value, p[12].value

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image = None
        self.image_stamp = None
        self.image_encoding = ''
        self.camera_matrix = None
        self.status = '等待图像；窗口获得焦点后按 Enter 采集'
        self.detections = {}
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        # TF must keep updating while a capture waits for an exact-time lookup.
        # Use a separate node/thread so the image and detection callbacks can
        # continue to be serviced by the UI loop at the same time.
        self.tf_node = rclpy.create_node('bunker_extrinsic_tf_listener')
        self.tf_listener = TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.node.create_subscription(Image, self.image_topic, self.on_image, 5)
        self.node.create_subscription(CameraInfo, self.camera_info_topic,
                                      self.on_camera_info, 5)
        self.node.create_subscription(AprilTagDetectionArray, self.detections_topic,
                                      self.on_detections, 10)

        output_dir = os.path.dirname(os.path.abspath(self.output))
        os.makedirs(output_dir, exist_ok=True)
        self.file = open(self.output, 'a', encoding='utf-8')
        self.count = sum(1 for line in open(self.output, encoding='utf-8') if line.strip())

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # malformed or unsupported frame
            self.node.get_logger().warning(f'Cannot convert camera image: {exc}')
            return
        ns = self.stamp_ns(msg.header.stamp)
        with self.lock:
            self.image, self.image_stamp = image, ns
            self.image_encoding = msg.encoding

    def on_camera_info(self, msg):
        # Match apriltag_ros: use the projection matrix of the paired camera model.
        p = msg.p
        if p[0] > 0.0 and p[5] > 0.0:
            self.camera_matrix = np.asarray(
                [[p[0], 0.0, p[2]], [0.0, p[5], p[6]], [0.0, 0.0, 1.0]],
                dtype=np.float64)
        elif msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)

    @staticmethod
    def rotation_to_quaternion(rotation):
        # Return ROS xyzw quaternion from a 3x3 rotation matrix.
        trace = float(np.trace(rotation))
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            q = [(rotation[2, 1] - rotation[1, 2]) / s,
                 (rotation[0, 2] - rotation[2, 0]) / s,
                 (rotation[1, 0] - rotation[0, 1]) / s,
                 0.25 * s]
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            q = [0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s,
                 (rotation[0, 2] + rotation[2, 0]) / s,
                 (rotation[2, 1] - rotation[1, 2]) / s]
        elif rotation[1, 1] > rotation[2, 2]:
            s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            q = [(rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s,
                 (rotation[1, 2] + rotation[2, 1]) / s,
                 (rotation[0, 2] - rotation[2, 0]) / s]
        else:
            s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            q = [(rotation[0, 2] + rotation[2, 0]) / s,
                 (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s,
                 (rotation[1, 0] - rotation[0, 1]) / s]
        q = np.asarray(q, dtype=np.float64)
        q /= np.linalg.norm(q)
        return q.tolist()

    def estimate_camera_tag(self, corners, camera_matrix):
        if camera_matrix is None:
            raise ValueError('未收到有效 camera_info 内参')
        half = self.tag_size / 2.0
        object_points = np.asarray([
            [-half, -half, 0.0], [half, -half, 0.0],
            [half, half, 0.0], [-half, half, 0.0]], dtype=np.float64)
        image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, np.zeros(5),
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or float(tvec[2, 0]) <= 0.0:
            raise ValueError('solvePnP 未得到有效的正向 Tag 位姿')
        rotation, _ = cv2.Rodrigues(rvec)
        return {
            'translation_m': [float(v) for v in tvec.reshape(3)],
            'quaternion_xyzw': self.rotation_to_quaternion(rotation),
        }

    def on_detections(self, msg):
        ns = self.stamp_ns(msg.header.stamp)
        accepted = {
            d.id: {
                'margin': float(d.decision_margin),
                'corners': [[float(point.x), float(point.y)] for point in d.corners],
            }
            for d in msg.detections
        }
        with self.lock:
            self.detections[ns] = accepted
            # Keep the small cache bounded while allowing delayed Enter presses.
            if len(self.detections) > 300:
                for old in sorted(self.detections)[:-200]:
                    self.detections.pop(old, None)

    @staticmethod
    def transform_record(tf):
        t, q = tf.transform.translation, tf.transform.rotation
        return {'translation_m': [t.x, t.y, t.z],
                'quaternion_xyzw': [q.x, q.y, q.z, q.w]}

    def capture(self):
        with self.lock:
            if self.image_stamp is None:
                self.set_status('尚未收到相机图像')
                return
            image_stamp_ns = self.image_stamp
            nearest = min(self.detections,
                          key=lambda n: abs(n - image_stamp_ns), default=None)
            if nearest is None or abs(nearest - image_stamp_ns) > self.max_sync_error * 1e9:
                self.set_status('未记录：当前画面没有同步的 AprilTag 检测')
                return
            # The detection header stamp is the source image's acquisition time.
            # Query odometry and tag TF at this exact time, not at the newer image.
            stamp_ns = nearest
            detection = self.detections[nearest].get(self.tag_id)
            if detection is None or detection['margin'] < self.min_margin:
                self.set_status(f'未记录：未检测到 Tag {self.tag_id} 或检测质量过低')
                return
            margin = detection['margin']
            camera_matrix = None if self.camera_matrix is None else self.camera_matrix.copy()

        try:
            camera_tag = self.estimate_camera_tag(detection['corners'], camera_matrix)
        except (ValueError, cv2.error) as exc:
            self.set_status(f'未记录：无法从 Tag 角点计算位姿（{exc}）')
            return

        camera_link_optical = None
        try:
            link_optical_tf = self.tf_buffer.lookup_transform(
                self.camera_link_frame, self.camera_frame, Time(),
                timeout=Duration(seconds=0.25))
            camera_link_optical = self.transform_record(link_optical_tf)
        except TransformException:
            # Keep the optical-frame solution usable even if the driver does
            # not publish the internal camera_link transform.
            pass

        stamp = Time(nanoseconds=stamp_ns)
        odom_stamp_ns = stamp_ns
        odom_offset_sec = 0.0
        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, stamp, timeout=Duration(seconds=0.75))
        except TransformException:
            # A camera stamp can be a little ahead of the newest odometry TF.
            # While the robot is stopped, the newest pose is a safe near-time
            # sample; store its real stamp and reject it if the gap is too large.
            try:
                base_tf = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, Time(), timeout=Duration(seconds=0.25))
            except TransformException as exc:
                self.set_status(f'未记录：odom TF 尚不可用（{str(exc)[:90]}）')
                return
            odom_stamp_ns = self.stamp_ns(base_tf.header.stamp)
            odom_offset_sec = (odom_stamp_ns - stamp_ns) / 1e9
            if abs(odom_offset_sec) > 0.25:
                self.set_status(
                    f'未记录：odom 与 Tag 时间差 {odom_offset_sec:+.3f}s，超过 0.25s；请停车后重试')
                return

        row = {
            'sample': self.count + 1,
            'timestamp_ns': stamp_ns,
            'timestamp_sec': stamp_ns / 1e9,
            'odom_timestamp_ns': odom_stamp_ns,
            'odom_time_offset_sec': odom_offset_sec,
            'tag_id': self.tag_id,
            'decision_margin': margin,
            'odom_frame': self.odom_frame,
            'base_frame': self.base_frame,
            'camera_frame': self.camera_frame,
            'camera_link_frame': self.camera_link_frame,
            'tag_frame': self.tag_frame,
            'odom_T_base': self.transform_record(base_tf),
            'camera_T_tag': camera_tag,
            'camera_link_T_optical': camera_link_optical,
        }
        self.file.write(json.dumps(row, ensure_ascii=False) + '\n')
        self.file.flush()
        os.fsync(self.file.fileno())
        self.count += 1
        self.set_status(f"已记录第 {self.count} 组 | Tag margin={margin:.1f}")

    def set_status(self, message):
        self.status = message
        print(message)

    def run(self):
        print(f'图像: {self.image_topic} | 检测: {self.detections_topic}')
        print(f'标定数据: {self.output} | Tag ID: {self.tag_id}')
        print('Enter：采集当前画面；q 或 Esc：退出。请固定 Tag 板，手动移动小车采集不同位置和朝向。')
        window = 'Astra extrinsic calibration recorder'
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.01)
                with self.lock:
                    image = None if self.image is None else self.image.copy()
                    stamp_ns = self.image_stamp
                    detection_stamp = min(
                        self.detections,
                        key=lambda n: abs(n - stamp_ns),
                        default=None) if stamp_ns is not None else None
                    tag_detection = (
                        self.detections[detection_stamp].get(self.tag_id)
                        if detection_stamp is not None and stamp_ns is not None and
                        abs(detection_stamp - stamp_ns) <= self.max_sync_error * 1e9
                        else None)
                    tag_detected = (tag_detection is not None and
                                    tag_detection['margin'] >= self.min_margin)
                if image is None:
                    frame = None
                else:
                    if tag_detected:
                        corners = np.asarray(tag_detection['corners'], dtype=np.float32).reshape(4, 2)
                        polygon = np.round(corners).astype(np.int32)
                        cv2.polylines(image, [polygon], True, (0, 255, 0), 3, cv2.LINE_AA)
                        x, y = polygon[0]
                        cv2.putText(image, f"ID {self.tag_id} margin {tag_detection['margin']:.1f}",
                                    (int(x), max(24, int(y) - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.65, (0, 255, 0), 2, cv2.LINE_AA)
                    label = f'Tag {self.tag_id}: ' + ('DETECTED' if tag_detected else 'not detected')
                    cv2.putText(image, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (0, 220, 0) if tag_detected else (0, 0, 255), 2)
                    cv2.putText(image, 'ENTER: capture    q/ESC: quit', (15, 62),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
                    cv2.putText(image, self.status[:100], (15, 94),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                    frame = image
                if frame is not None:
                    cv2.imshow(window, frame)
                key = cv2.waitKey(10) & 0xFF
                if key in (10, 13):
                    self.capture()
                elif key in (ord('q'), 27):
                    break
        finally:
            self.file.close()
            cv2.destroyAllWindows()
            self.tf_listener.unregister()
            if hasattr(self.tf_listener, 'executor'):
                self.tf_listener.executor.shutdown(timeout_sec=1.0)
                self.tf_listener.dedicated_listener_thread.join(timeout=1.0)
            self.tf_node.destroy_node()
            self.node.destroy_node()


def main():
    rclpy.init()
    recorder = ExtrinsicRecorder()
    try:
        try:
            recorder.run()
        except KeyboardInterrupt:
            pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
