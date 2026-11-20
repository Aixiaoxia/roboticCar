#!/usr/bin/env python3
"""Preview or execute small BUNKER step tests and write timestamped CSV data."""
import argparse
import csv
import json
import math
from pathlib import Path
import signal
import time
from datetime import datetime


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--axis', choices=['linear', 'angular'], required=True)
    p.add_argument('--amplitudes', type=float, nargs='+', default=None)
    p.add_argument('--hold', type=float, default=3.0)
    p.add_argument('--rest', type=float, default=4.0)
    p.add_argument('--repeats', type=int, default=3)
    p.add_argument('--rate', type=float, default=30.0)
    p.add_argument('--feedback-timeout', type=float, default=0.5)
    p.add_argument('--imu-topic', help='Optional sensor_msgs/msg/Imu topic')
    p.add_argument('--output', type=Path)
    p.add_argument('--execute', action='store_true', help='Actually publish motion commands')
    a = p.parse_args()
    if a.amplitudes is None:
        a.amplitudes = [0.05, 0.10] if a.axis == 'linear' else [0.05, -0.05, 0.10, -0.10]
    if any(not math.isfinite(x) for x in a.amplitudes):
        p.error('Amplitudes must be finite')
    limit = 0.3 if a.axis == 'linear' else 0.5
    if any(abs(x) > limit or x == 0 for x in a.amplitudes):
        p.error(f'{a.axis} amplitudes must be nonzero with absolute value <= {limit} '
                f"{'m/s' if a.axis == 'linear' else 'rad/s'}")
    if any(not math.isfinite(x) or x <= 0 for x in
           [a.hold, a.rest, a.rate, a.feedback_timeout]) or a.repeats < 1:
        p.error('Durations, rate, timeout and repeats must be positive')
    if a.rate > 100 or a.hold > 10 or a.rest < 2:
        p.error('Use rate <= 100 Hz, hold <= 10 s and rest >= 2 s')
    return a


def sequence(a):
    plan = [('baseline', 0.0, 2.0)]
    for repeat in range(a.repeats):
        for value in a.amplitudes:
            plan += [(f'step_{repeat + 1}_{value:+.3f}', value, a.hold),
                     ('rest', 0.0, a.rest)]
    return plan


def default_output(a):
    unit = 'mps' if a.axis == 'linear' else 'radps'
    amplitudes = '_'.join(format(value, '.12g') for value in a.amplitudes)
    name = (f"{a.axis}_amp_{amplitudes}{unit}"
            f"_initial2s_hold{a.hold:.12g}s_rest{a.rest:.12g}s"
            f"_repeat{a.repeats}_rate{a.rate:.12g}Hz"
            f"_timeout{a.feedback_timeout:.12g}s"
            f"_imu{'on' if a.imu_topic else 'off'}"
            f"__{datetime.now():%Y%m%d_%H%M%S_%f}")
    return Path('data/system_identification') / name


def main():
    a = arguments()
    plan = sequence(a)
    output = a.output or default_output(a)
    print(f"Output directory: {output.resolve()}")
    print(f'Axis: {a.axis}; duration: {sum(x[2] for x in plan):.1f} s')
    for name, value, duration in plan:
        print(f'  {name:24s} {value:+.3f} for {duration:.1f} s')
    if not a.execute:
        print('Preview only. Add --execute to publish to /cmd_vel.')
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from bunker_msgs.msg import BunkerStatus
    from sensor_msgs.msg import Imu

    output.mkdir(parents=True, exist_ok=False)
    metadata = vars(a).copy()
    metadata['output'] = str(output)
    metadata['plan'] = plan
    metadata['result'] = 'initializing'
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2))
    rclpy.init()
    node = Node('bunker_system_identification')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    files = []
    def writer(name, header):
        f = (output / name).open('w', newline='', buffering=1)
        files.append(f)
        w = csv.writer(f)
        w.writerow(header)
        return w
    commands = writer('commands.csv', ['elapsed_s', 'ros_time_ns', 'phase', 'linear_x', 'angular_z'])
    feedback = writer('feedback.csv', ['elapsed_s', 'ros_receive_ns', 'header_ns', 'linear_velocity', 'angular_velocity'])
    imu = writer('imu.csv', ['elapsed_s', 'ros_receive_ns', 'header_ns', 'angular_velocity_z'])
    start = time.monotonic()
    last_feedback = None
    stop_requested = False
    def stamp_ns(msg):
        return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    def on_feedback(msg):
        nonlocal last_feedback
        last_feedback = time.monotonic()
        feedback.writerow([last_feedback-start, node.get_clock().now().nanoseconds,
                           stamp_ns(msg), msg.linear_velocity, msg.angular_velocity])
    def on_imu(msg):
        imu.writerow([time.monotonic()-start, node.get_clock().now().nanoseconds,
                      stamp_ns(msg), msg.angular_velocity.z])
    sub = node.create_subscription(BunkerStatus, '/bunker_status', on_feedback, 50)
    imu_sub = node.create_subscription(Imu, a.imu_topic, on_imu, qos_profile_sensor_data) if a.imu_topic else None
    def stop_handler(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    def send(value, phase):
        msg = Twist()
        if a.axis == 'linear':
            msg.linear.x = value
        else:
            msg.angular.z = value
        pub.publish(msg)
        commands.writerow([time.monotonic()-start, node.get_clock().now().nanoseconds,
                           phase, msg.linear.x, msg.angular.z])
    result = 'aborted'
    try:
        print(f'Logging to {output.resolve()}. Waiting for feedback and ROS discovery...')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
        if stop_requested:
            raise RuntimeError('Interrupted before test')
        if last_feedback is None or time.monotonic()-last_feedback > a.feedback_timeout:
            raise RuntimeError('No fresh /bunker_status; motion not started')
        if pub.get_subscription_count() == 0:
            raise RuntimeError('No /cmd_vel subscriber')
        for phase, value, duration in plan:
            print(f'{phase}: {value:+.3f}, {duration:.1f} s', flush=True)
            end = time.monotonic() + duration
            next_tick = time.monotonic()
            while time.monotonic() < end:
                if stop_requested:
                    raise RuntimeError('Interrupted by operator')
                if node.count_publishers('/cmd_vel') != 1:
                    raise RuntimeError('Other /cmd_vel publishers detected; stop their nodes first')
                if last_feedback is None or time.monotonic()-last_feedback > a.feedback_timeout:
                    raise RuntimeError('Feedback timeout')
                send(value, phase)
                next_tick += 1/a.rate
                while time.monotonic() < min(next_tick, end) and not stop_requested:
                    rclpy.spin_once(node, timeout_sec=min(0.01, max(0, min(next_tick, end)-time.monotonic())))
        result = 'completed'
    except Exception as exc:
        result = f'aborted: {exc}'
        print(result, flush=True)
    finally:
        # Best-effort stop; physical emergency stop remains necessary on process/communication failure.
        if rclpy.ok():
            until = time.monotonic() + 1.0
            while time.monotonic() < until:
                send(0.0, 'shutdown_stop')
                rclpy.spin_once(node, timeout_sec=0.03)
        metadata['result'] = result
        (output / 'metadata.json').write_text(json.dumps(metadata, indent=2))
        for f in files:
            f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'{result}. Data: {output.resolve()}')
    if result != 'completed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
