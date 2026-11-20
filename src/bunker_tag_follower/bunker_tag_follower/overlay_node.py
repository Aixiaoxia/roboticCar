#!/usr/bin/env python3
"""Draw synchronized AprilTag corners and TF axes without a cv_bridge dependency."""
from collections import OrderedDict
from array import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformException


def stamp_key(header):
    return (header.frame_id, header.stamp.sec, header.stamp.nanosec)


def to_bgr(msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}
    if msg.encoding not in channels:
        raise ValueError('Unsupported image encoding: ' + msg.encoding)
    count = channels[msg.encoding]
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * count].reshape(msg.height, msg.width, count)
    if msg.encoding == 'bgr8':
        return pixels.copy()
    conversion = {'rgb8': cv2.COLOR_RGB2BGR, 'mono8': cv2.COLOR_GRAY2BGR,
                  'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR}
    return cv2.cvtColor(pixels, conversion[msg.encoding])


def rotation_vector(q):
    x, y, z, w = np.array([q.x, q.y, q.z, q.w], dtype=float) / np.linalg.norm(
        [q.x, q.y, q.z, q.w])
    matrix = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return cv2.Rodrigues(matrix)[0]


class AprilTagOverlay(Node):
    def __init__(self):
        super().__init__('apriltag_overlay')
        defaults = {'image_topic': '/camera/color/image_raw',
                    'camera_info_topic': '/camera/color/camera_info',
                    'detections_topic': '/apriltag/detections',
                    'output_topic': '/apriltag/image_overlay',
                    'tag_frame': 'target_tag', 'tag_id': 0, 'axis_length': 0.075}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        self.images = OrderedDict()
        self.detections = OrderedDict()
        self.info = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Image, self.params['output_topic'], 2)
        self.subscriptions_kept = [
            self.create_subscription(Image, self.params['image_topic'], self.on_image,
                                     qos_profile_sensor_data),
            self.create_subscription(CameraInfo, self.params['camera_info_topic'],
                                     self.on_info, qos_profile_sensor_data),
            self.create_subscription(AprilTagDetectionArray, self.params['detections_topic'],
                                     self.on_detections, qos_profile_sensor_data)]
        self.timer = self.create_timer(0.02, self.render_pending)
        self.get_logger().info('Overlay output: ' + self.params['output_topic'])

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        self.images[stamp_key(msg.header)] = (msg, time.monotonic())
        while len(self.images) > 8:
            self.images.popitem(last=False)

    def on_detections(self, msg):
        self.detections[stamp_key(msg.header)] = msg
        while len(self.detections) > 30:
            self.detections.popitem(last=False)

    def render_pending(self):
        for key in reversed(list(self.images)):
            image, arrived = self.images[key]
            detection = self.detections.get(key)
            age = time.monotonic() - arrived
            if detection is None and age < 0.12:
                continue
            targets = [] if detection is None else [
                d for d in detection.detections
                if d.id == self.params['tag_id'] and d.family == 'tag36h11']
            transform = None
            if targets:
                try:
                    transform = self.buffer.lookup_transform(
                        image.header.frame_id, self.params['tag_frame'],
                        Time.from_msg(image.header.stamp)).transform
                except TransformException:
                    if age < 0.05:
                        continue
            try:
                frame = to_bgr(image)
                status = 'No ID 0' if detection is not None else 'Waiting for synchronized detections'
                for tag in targets:
                    corners = np.array([[p.x, p.y] for p in tag.corners])
                    if not np.isfinite(corners).all():
                        continue
                    cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 2)
                    for index, point in enumerate(corners.astype(int)):
                        cv2.circle(frame, tuple(point), 4, (0, 255, 255), -1)
                    center = (int(tag.centre.x), int(tag.centre.y))
                    cv2.putText(frame, 'ID 0', center, cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 2)
                    status = 'ID 0 detected; waiting for matching TF'
                    info = self.info
                    if (transform is not None and info is not None
                            and info.header.frame_id == image.header.frame_id
                            and (info.width, info.height) == (image.width, image.height)):
                        t = transform.translation
                        status = f'ID 0 | camera: x={t.x:.2f} y={t.y:.2f} z={t.z:.2f} m'
                        # Detector uses rectified-image P. Current Astra model has D=0.
                        if any(abs(d) > 1e-9 for d in info.d):
                            status = 'Nonzero distortion: use rectified image/info for axes'
                        elif t.z > 0:
                            k = np.array(info.p).reshape(3, 4)[:, :3].copy()
                            cv2.drawFrameAxes(frame, k, np.zeros(5),
                                              rotation_vector(transform.rotation),
                                              np.array([t.x, t.y, t.z]),
                                              self.params['axis_length'], 3)
                cv2.rectangle(frame, (0, 0), (frame.shape[1], 28), (0, 0, 0), -1)
                cv2.putText(frame, status, (6, 19), cv2.FONT_HERSHEY_SIMPLEX,
                            0.47, (255, 255, 255), 1)
                output = Image()
                output.header = image.header
                output.height, output.width = frame.shape[:2]
                output.encoding = 'bgr8'
                output.step = output.width * 3
                output.data = array('B', frame.tobytes())
                self.publisher.publish(output)
            except (ValueError, cv2.error) as error:
                self.get_logger().error(str(error))
            # Never publish an older image after this one. Drop queued work,
            # rather than rendering a backlog when detection or TF is delayed.
            for old_key in list(self.images):
                if self.images[old_key][1] <= arrived:
                    del self.images[old_key]
                    self.detections.pop(old_key, None)
            return


def main():
    rclpy.init()
    node = AprilTagOverlay()
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
