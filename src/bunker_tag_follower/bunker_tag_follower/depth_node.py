"""Fail-closed depth proximity heartbeat for supervised, low-speed commissioning."""
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from .runtime import initialize_runtime
from .depth_policy import assess_depth


class DepthClearance(Node):
    def __init__(self):
        super().__init__('follow_depth_clearance')
        for key, value in dict(depth_topic='/camera/depth/image_raw',
                              stop_distance=0.6, minimum_valid_fraction=0.5,
                              minimum_tile_valid_fraction=0.15, check_blind_sectors=True,
                              timeout=0.2, check_depth_timeout=True).items():
            self.declare_parameter(key, value)
        self.stop_distance = self.get_parameter('stop_distance').value
        self.valid_fraction = self.get_parameter('minimum_valid_fraction').value
        self.tile_valid_fraction = self.get_parameter('minimum_tile_valid_fraction').value
        self.check_blind_sectors = self.get_parameter('check_blind_sectors').value
        self.timeout = self.get_parameter('timeout').value
        self.check_depth_timeout = self.get_parameter('check_depth_timeout').value
        if not (self.stop_distance > 0 and 0 < self.valid_fraction <= 1
                and 0 < self.tile_valid_fraction <= 1 and self.timeout > 0):
            raise ValueError('Invalid depth monitor parameters')
        self.last_sample = None
        self.last_stamp = -1
        self.status = dict(clear=False, reason='waiting_for_depth')
        self.pub = self.create_publisher(Bool, '/follow/path_clear', 1)
        self.state_pub = self.create_publisher(String, '/follow/depth_state', 1)
        self.sub = self.create_subscription(Image, self.get_parameter('depth_topic').value,
            self.on_depth, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(.05, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_depth(self, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp <= self.last_stamp:
            return  # Duplicate frames do not refresh the stored sample timestamp.
        self.last_stamp = stamp
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if self.check_depth_timeout and not -.05 <= age <= self.timeout:
            self.status = dict(clear=False, reason='stale_depth')
            self.last_sample = None
            return
        try:
            if msg.encoding == '16UC1':
                dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
                scale = .001
            elif msg.encoding == '32FC1':
                dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
                scale = 1.0
            else:
                raise ValueError('Unsupported depth encoding')
            if msg.step % dtype.itemsize:
                raise ValueError('Invalid row stride')
            data = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step//dtype.itemsize)
            if data.shape[1] < msg.width or msg.width == 0:
                raise ValueError('Invalid image width')
            clear, reason, fraction, nearest = assess_depth(
                data[:, :msg.width]*scale, self.stop_distance, self.valid_fraction,
                self.tile_valid_fraction, self.check_blind_sectors)
            self.status = dict(clear=clear, reason=reason, valid_fraction=fraction,
                               nearest_m=nearest, stop_distance_m=self.stop_distance,
                               minimum_tile_valid_fraction=self.tile_valid_fraction,
                               check_blind_sectors=self.check_blind_sectors)
            self.last_sample = (stamp, time.monotonic())
        except (ValueError, TypeError):
            self.status = dict(clear=False, reason='invalid_depth_image')
            self.last_sample = None

    def tick(self):
        status = dict(self.status)
        status['check_depth_timeout'] = self.check_depth_timeout
        if self.last_sample is None:
            status['clear'] = False
        else:
            stamp, received = self.last_sample
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            fresh = (-.05 <= age <= self.timeout
                     and time.monotonic()-received <= self.timeout)
            status['depth_fresh'] = fresh
            status['depth_age_seconds'] = age
            if not fresh and self.check_depth_timeout:
                status.update(clear=False, reason='depth_timeout')
            elif not fresh and status['clear']:
                status['reason'] = 'clear_from_cached_depth'
        self.pub.publish(Bool(data=status['clear']))
        self.state_pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main():
    initialize_runtime()
    node = DepthClearance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
