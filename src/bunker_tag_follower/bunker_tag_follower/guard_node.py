"""Separate process that stops commands if the follower freezes or exits."""
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .runtime import initialize_runtime
from .policy import Command, VelocityGuard


class GuardNode(Node):
    def __init__(self):
        super().__init__('follow_velocity_guard')
        defaults = dict(hardware=False, mode='handheld', timeout=0.3,
                        max_linear=0.05, max_angular=0.1,
                        linear_accel=0.03, angular_accel=0.2)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        if p['hardware'] and p['mode'] != 'mounted':
            raise ValueError('Hardware output is forbidden in handheld mode')
        self.expected_frame = 'base_link' if p['mode'] == 'mounted' else 'handheld_preview'
        self.declare_parameter('base_frame', 'base_link')
        if p['mode'] == 'mounted':
            self.expected_frame = self.get_parameter('base_frame').value
        self.guard = VelocityGuard(**{k: p[k] for k in (
            'timeout', 'max_linear', 'max_angular', 'linear_accel', 'angular_accel')},
            require_clearance=p['hardware'])
        self.output_topic = '/cmd_vel' if p['hardware'] else '/follow/cmd_vel_guarded_preview'
        self.output_pub = self.create_publisher(Twist, self.output_topic, 1)
        self.status_pub = self.create_publisher(String, '/follow/guard_state', 1)
        self.command_sub = self.create_subscription(TwistStamped, '/follow/candidate', self.on_command, 1)
        self.clearance_sub = self.create_subscription(Bool, '/follow/path_clear',
            lambda msg: self.guard.path_clear(msg.data, time.monotonic()), 1)
        self.service = self.create_service(SetBool, '/follow/enable', self.on_enable)
        self.timer = self.create_timer(0.05, self.tick,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_state = None
        self.get_logger().info('Disabled on startup; output: ' + self.output_topic)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_command(self, msg):
        if msg.header.frame_id != self.expected_frame:
            self.guard.enabled = False
            self.guard.sample = None
            return
        c = Command(msg.twist.linear.x, msg.twist.angular.z)
        self.guard.receive(c, msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, time.monotonic())

    def on_enable(self, request, response):
        response.success, response.message = self.guard.arm(request.data, self.now(), time.monotonic())
        return response

    def tick(self):
        command, state = self.guard.tick(self.now(), time.monotonic())
        msg = Twist()
        msg.linear.x, msg.angular.z = command.linear, command.angular
        self.output_pub.publish(msg)
        self.status_pub.publish(String(data=state))
        if state != self.last_state:
            self.get_logger().info('Guard state: ' + state)
            self.last_state = state


def main():
    initialize_runtime()
    node = GuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.output_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
