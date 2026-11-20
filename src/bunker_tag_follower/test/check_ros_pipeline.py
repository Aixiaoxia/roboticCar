#!/usr/bin/python3
"""Exercise real ROS processes in a private domain without any hardware output."""
import os
import json
import signal
import subprocess
import tempfile
import time
from pathlib import Path

# Do not discover or publish to the real robot graph.
os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rcl_interfaces.srv import GetParameters
import yaml
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def main():
    config = Path(__file__).resolve().parents[1] / 'config' / 'follow.yaml'
    processes = []
    logs = []
    rclpy.init()
    node = rclpy.create_node('follow_integration_driver')
    preview, guarded, statuses, states = [], [], [], []
    subs = [
        node.create_subscription(Twist, '/follow/cmd_vel_preview', preview.append, 10),
        node.create_subscription(Twist, '/follow/cmd_vel_guarded_preview', guarded.append, 10),
        node.create_subscription(String, '/follow/guard_state', lambda m: statuses.append(m.data), 10),
        node.create_subscription(String, '/follow/state', lambda m: states.append(m.data), 10)]
    pub = node.create_publisher(AprilTagDetectionArray, '/apriltag/detections', 1)
    broadcaster = TransformBroadcaster(node)
    client = node.create_client(SetBool, '/follow/enable')

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def feed(x=0., z=2., visible=True, seconds=0.5):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'camera_color_optical_frame'
            tf.child_frame_id = 'target_tag'
            tf.transform.translation.x, tf.transform.translation.z = x, z
            tf.transform.rotation.w = 1.
            broadcaster.sendTransform(tf)
            msg = AprilTagDetectionArray()
            msg.header = tf.header
            if visible:
                d = AprilTagDetection()
                d.family, d.id, d.decision_margin = 'tag36h11', 0, 80.
                msg.detections = [d]
            pub.publish(msg)
            spin(0.035)

    try:
        for executable in ('follower', 'velocity_guard'):
            log = tempfile.TemporaryFile(mode='w+')
            logs.append(log)
            processes.append(subprocess.Popen(
                ['ros2', 'run', 'bunker_tag_follower', executable, '--ros-args',
                 '--params-file', str(config)], stdout=log, stderr=log,
                start_new_session=True))
        assert client.wait_for_service(timeout_sec=8), 'Guard service missing'
        ready = time.monotonic()+8
        while time.monotonic() < ready and (not states or pub.get_subscription_count() == 0):
            spin(.05)
        # A single shared YAML limit must reach both actual ROS nodes.
        shared = yaml.safe_load(config.read_text())['/**']['ros__parameters']
        for name in ('tag_follower', 'follow_velocity_guard'):
            params = node.create_client(GetParameters, '/' + name + '/get_parameters')
            assert params.wait_for_service(timeout_sec=3), name
            request = GetParameters.Request()
            request.names = ['max_linear', 'max_angular']
            result = params.call_async(request)
            end = time.monotonic()+3
            while not result.done() and time.monotonic()<end:
                spin(.02)
            assert result.done(), name
            assert [v.double_value for v in result.result().values] == [
                shared['max_linear'], shared['max_angular']], name
        feed(seconds=1.5)
        assert preview and preview[-1].linear.x > 0, 'Centered target did not produce forward preview: ' + str(states[-6:])
        assert guarded and guarded[-1].linear.x == 0, 'Guard was enabled at startup'
        feed(visible=False, seconds=.3)
        preview.clear()
        feed(seconds=.001)  # Exactly one fresh detection and matching transform.
        spin(.12)
        assert any(m.linear.x > 0 for m in preview), 'One valid frame did not produce preview motion'
        state = json.loads(states[-1])
        assert state['kf_enabled'] and state['acquired_frames'] == 1
        assert abs(state['forward_m']-state['raw_forward_m']) < 1e-8
        assert abs(state['relative_forward_velocity_mps']) < 1e-8
        spin(.35)  # No more detections: the filter must not keep driving by prediction.
        assert preview[-1].linear.x == preview[-1].angular.z == 0
        assert json.loads(states[-1])['raw_forward_m'] is None
        feed(x=-1.)
        assert preview[-1].linear.x == 0 and preview[-1].angular.z > 0, 'Left sign/turn-only failed'
        feed(x=1.)
        assert preview[-1].linear.x == 0 and preview[-1].angular.z < 0, 'Right sign/turn-only failed'
        feed(z=.5)
        assert preview[-1].linear.x == preview[-1].angular.z == 0, 'Distance hold failed'
        feed(z=2., seconds=.4)
        feed(z=.2, seconds=.001)
        spin(.08)
        assert preview[-1].linear.x == preview[-1].angular.z == 0, 'KF delayed raw close-distance braking'
        feed(visible=False)
        assert preview[-1].linear.x == preview[-1].angular.z == 0, 'Occlusion failed to stop'
        feed()
        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        for _ in range(10):
            feed(seconds=.05)
            if future.done():
                break
        assert future.done() and future.result().success, 'Preview arm failed'
        feed(seconds=.3)
        assert guarded[-1].linear.x > 0, 'Guarded preview did not accelerate'
        # Abrupt follower death: no cooperative shutdown/zero command allowed.
        os.killpg(processes[0].pid, signal.SIGKILL)
        processes[0].wait(timeout=3)
        spin(.7)
        assert guarded[-1].linear.x == guarded[-1].angular.z == 0, 'Watchdog failed to brake'
        assert statuses[-1] in ('command_watchdog', 'stale_command_stamp'), statuses[-1]
        # Mounted mode must wait for a real camera-to-base transform.
        log = tempfile.TemporaryFile(mode='w+')
        logs.append(log)
        processes.append(subprocess.Popen(
            ['ros2', 'run', 'bunker_tag_follower', 'follower', '--ros-args',
             '--params-file', str(config), '-p', 'mode:=mounted'],
            stdout=log, stderr=log, start_new_session=True))
        feed(seconds=1.2)
        assert preview[-1].linear.x == preview[-1].angular.z == 0, 'Missing extrinsics did not stop'
        static = StaticTransformBroadcaster(node)
        mount = TransformStamped()
        mount.header.stamp = node.get_clock().now().to_msg()
        mount.header.frame_id = 'base_link'
        mount.child_frame_id = 'camera_color_optical_frame'
        # Synthetic optical -> base rotation, not an installation calibration.
        mount.transform.rotation.x = -0.5
        mount.transform.rotation.y = 0.5
        mount.transform.rotation.z = -0.5
        mount.transform.rotation.w = 0.5
        static.sendTransform(mount)
        feed(seconds=2.)
        assert preview[-1].linear.x > 0, 'Mounted transform did not produce forward preview: ' + str(states[-5:])
        assert node.count_publishers('/cmd_vel') == 0, 'Preview unexpectedly exposes hardware output'
        print('PASS: KF first frame/reset/close stop, steering, hold, loss, arm, crash watchdog, mounted TF requirement; no /cmd_vel publisher')
    except BaseException:
        for log in logs:
            log.seek(0)
            print(log.read())
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
