"""Adapt fresh AprilTag observations into preview commands; never drive hardware."""
import json
import math
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformException

from .kalman import PlanarKalmanFilter
from .runtime import initialize_runtime
from .policy import Command, FollowConfig, follow_target, optical_to_planar


def twist(command):
    msg = Twist()
    msg.linear.x, msg.angular.z = command.linear, command.angular
    return msg


class TagFollower(Node):
    def __init__(self):
        super().__init__('tag_follower')
        defaults = dict(mode='handheld', camera_frame='camera_color_optical_frame',
                        base_frame='base_link', tag_frame='target_tag', tag_id=0,
                        detections_topic='/apriltag/detections', observation_timeout=0.25,
                        minimum_margin=20.0, acquire_frames=1, control_rate=20.0,
                        centered_forward_mount=False)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        if self.settings['mode'] not in ('handheld', 'mounted'):
            raise ValueError('mode must be handheld or mounted')
        if (self.settings['acquire_frames'] < 1 or self.settings['observation_timeout'] <= 0
                or self.settings['control_rate'] <= 0):
            raise ValueError('Invalid tracking timing parameters')
        values = {}
        for name, value in vars(FollowConfig()).items():
            self.declare_parameter(name, value)
            values[name] = self.get_parameter(name).value
        self.config = FollowConfig(**values)
        for name, value in {'kf.enabled': True, 'kf.position_std': .04,
                            'kf.acceleration_std': 2., 'kf.initial_velocity_std': 1.}.items():
            self.declare_parameter(name, value)
        self.kf_enabled = self.get_parameter('kf.enabled').value
        self.kf = PlanarKalmanFilter(
            position_std=self.get_parameter('kf.position_std').value,
            acceleration_std=self.get_parameter('kf.acceleration_std').value,
            initial_velocity_std=self.get_parameter('kf.initial_velocity_std').value,
            max_gap=self.settings['observation_timeout'])
        self.raw_position = None
        self.relative_velocity = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = None
        self.observation = None
        self.last_seen_stamp = -1
        self.last_accepted_stamp = -1
        self.stable_count = 0
        self.reason = 'waiting_for_target'
        self.sub = self.create_subscription(
            AprilTagDetectionArray, self.settings['detections_topic'], self.on_detection,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.candidate_pub = self.create_publisher(TwistStamped, '/follow/candidate', 1)
        self.preview_pub = self.create_publisher(Twist, '/follow/cmd_vel_preview', 1)
        self.target_pub = self.create_publisher(PoseStamped, '/follow/target', 1)
        self.filtered_target_pub = self.create_publisher(PoseStamped, '/follow/filtered_target', 1)
        self.state_pub = self.create_publisher(String, '/follow/state', 1)
        self.timer = self.create_timer(1.0/self.settings['control_rate'], self.tick)
        self.get_logger().info('Mode: %s; output is preview only' % self.settings['mode'])

    def reset_target(self, reason):
        self.pending = self.observation = None
        self.kf.reset()
        self.raw_position = self.relative_velocity = None
        self.stable_count = 0
        self.reason = reason

    def on_detection(self, msg):
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not -0.05 <= age <= self.settings['observation_timeout']:
            self.reset_target('stale_detection')
            return
        if stamp <= self.last_seen_stamp:
            return  # Repeated messages must not keep a stale target alive.
        self.last_seen_stamp = stamp
        valid = [d for d in msg.detections if d.id == self.settings['tag_id']
                 and d.family == 'tag36h11' and d.hamming == 0
                 and math.isfinite(d.decision_margin)
                 and d.decision_margin >= self.settings['minimum_margin']]
        if len(valid) != 1 or msg.header.frame_id != self.settings['camera_frame']:
            self.reset_target('target_missing_or_rejected')
            return
        self.pending = (msg.header, time.monotonic())

    def update_target(self):
        if self.pending is None:
            return
        header, received = self.pending
        stamp = header.stamp.sec*10**9 + header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        timeout = self.settings['observation_timeout']
        if (not -0.05 <= (now-stamp)/1e9 <= timeout
                or time.monotonic()-received > timeout):
            self.reset_target('observation_timeout')
            return
        optical_planar = (self.settings['mode'] == 'handheld'
                          or self.settings['centered_forward_mount'])
        frame = header.frame_id if optical_planar else self.settings['base_frame']
        try:
            transform = self.buffer.lookup_transform(
                frame, self.settings['tag_frame'], Time.from_msg(header.stamp)).transform
        except TransformException:
            self.reason = 'waiting_for_matching_tf'
            return
        t = transform.translation
        forward, left = optical_to_planar(t.x, t.y, t.z) if optical_planar else (t.x, t.y)
        if not all(math.isfinite(v) for v in (forward, left, t.z)):
            self.reset_target('invalid_position')
            return
        if self.last_accepted_stamp >= 0 and (stamp-self.last_accepted_stamp)/1e9 > timeout:
            self.stable_count = 0
        self.raw_position = (forward, left)
        if self.kf_enabled:
            try:
                forward, left, vf, vl = self.kf.update(forward, left, stamp/1e9)
            except ValueError:
                self.reset_target('invalid_filter_measurement')
                return
            self.relative_velocity = (vf, vl)
        else:
            self.relative_velocity = None
        self.last_accepted_stamp = stamp
        self.stable_count += 1
        self.observation = (forward, left, header, received)
        self.pending = None
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
        pose.pose.orientation = transform.rotation
        self.target_pub.publish(pose)
        filtered_pose = deepcopy(pose)
        if optical_planar:
            filtered_pose.pose.position.x = -left
            filtered_pose.pose.position.z = forward
        else:
            filtered_pose.pose.position.x = forward
            filtered_pose.pose.position.y = left
        self.filtered_target_pub.publish(filtered_pose)

    def tick(self):
        self.update_target()
        command = Command()
        forward = left = None
        stamp = self.get_clock().now().to_msg()
        if self.observation:
            forward, left, header, received = self.observation
            age = (self.get_clock().now().nanoseconds-
                   header.stamp.sec*10**9-header.stamp.nanosec)/1e9
            if (not -0.05 <= age <= self.settings['observation_timeout']
                    or time.monotonic()-received > self.settings['observation_timeout']):
                self.reset_target('target_timeout')
            elif self.stable_count < self.settings['acquire_frames']:
                self.reason = 'acquiring'
            else:
                command, self.reason = follow_target(forward, left, self.config)
                # Smoothing must not delay raw close/range stops or large-angle alignment.
                raw_command, raw_reason = follow_target(*self.raw_position, self.config)
                if raw_reason in ('invalid_or_behind', 'too_close', 'out_of_range',
                                  'holding_distance', 'aligning'):
                    command, self.reason = raw_command, raw_reason
                stamp = header.stamp
        candidate = TwistStamped()
        candidate.header.stamp = stamp
        candidate.header.frame_id = 'handheld_preview' if self.settings['mode'] == 'handheld' else self.settings['base_frame']
        candidate.twist = twist(command)
        self.candidate_pub.publish(candidate)
        self.preview_pub.publish(candidate.twist)
        raw_forward, raw_left = self.raw_position or (None, None)
        vf, vl = self.relative_velocity or (None, None)
        state = dict(mode=self.settings['mode'], kf_enabled=self.kf_enabled,
                     raw_forward_m=raw_forward, raw_left_m=raw_left,
                     relative_forward_velocity_mps=vf, relative_left_velocity_mps=vl,
                     centered_forward_mount=self.settings['centered_forward_mount'], state=self.reason,
                     forward_m=forward, left_m=left, acquired_frames=self.stable_count,
                     linear_mps=command.linear, angular_rps=command.angular)
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main():
    initialize_runtime()
    node = TagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.candidate_pub.publish(TwistStamped())
        node.preview_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
