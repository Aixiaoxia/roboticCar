#!/usr/bin/python3
"""Verify depth parser and expiry in an isolated graph, without a robot driver."""
import os
import signal
import subprocess
import time
import sys
from array import array
os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def main():
    disabled = '--disable-timeout' in sys.argv
    rclpy.init(args=[])
    node = rclpy.create_node('depth_pipeline_test')
    received = []
    sub = node.create_subscription(Bool, '/follow/path_clear', lambda m: received.append(m.data), 1)
    pub = node.create_publisher(Image, '/test/depth', 1)
    process = subprocess.Popen(['ros2', 'run', 'bunker_tag_follower', 'depth_clearance',
                                '--ros-args', '-p', 'depth_topic:=/test/depth',
                                '-p', 'check_depth_timeout:=' + ('false' if disabled else 'true')],
                               start_new_session=True, stdout=subprocess.DEVNULL)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.01)
    def feed(depth):
        for _ in range(12):
            msg = Image()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.width, msg.height, msg.step = 80, 60, 160
            msg.encoding = '16UC1'
            msg.data = array('B', np.full((60,80), depth, dtype='<u2').tobytes())
            pub.publish(msg)
            spin(.04)
    try:
        spin(1.5)
        feed(2000)
        assert received[-1], 'Valid clear depth was rejected'
        feed(400)
        assert not received[-1], 'Close object was not rejected'
        feed(0)
        assert not received[-1], 'Invalid depth was not rejected'
        feed(2000)
        assert received[-1]
        spin(.4)
        assert received[-1] == disabled, 'Depth timeout switch did not take effect'
        print('PASS: clear depth, near object, invalid depth; timeout protection:', not disabled)
    finally:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=4)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
