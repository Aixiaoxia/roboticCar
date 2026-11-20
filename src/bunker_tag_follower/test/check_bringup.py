#!/usr/bin/python3
"""Exercise the combined launch and Qt panel away from the real robot graph."""
import os
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import rclpy
from std_msgs.msg import String


def main():
    args = ['ros2', 'launch', 'bunker_tag_follower', 'bringup.launch.py',
            'start_camera:=false', 'start_base:=false', 'hardware:=false']
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen(args, stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('check_total_bringup')
    states = []
    sub = node.create_subscription(String, '/follow/guard_state', lambda m: states.append(m.data), 1)
    try:
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as invalid_file:
            invalid_file.write('tag_follower:\n  ros__parameters: {desired_distance: 0.3, minimum_distance: 0.35}\n')
            invalid_file.flush()
            invalid = subprocess.run(args + ['config:=' + invalid_file.name],
                                     text=True, capture_output=True, timeout=8)
            assert invalid.returncode != 0
            assert '距离参数冲突' in invalid.stdout+invalid.stderr
            assert 'process started with pid' not in invalid.stdout+invalid.stderr
        required = {'apriltag', 'apriltag_overlay', 'tag_follower', 'follow_velocity_guard',
                    'follow_depth_clearance', 'follow_control_panel'}
        end = time.monotonic()+12
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if required <= set(node.get_node_names()) and states:
                break
            assert process.poll() is None, 'Combined launch exited early'
        assert required <= set(node.get_node_names()), node.get_node_names()
        assert all(state != 'enabled' for state in states), 'Motion was automatically enabled'
        assert node.count_publishers('/cmd_vel') == 0, 'Preview exposed hardware output'
        # Exercise the actual Qt button slots against the isolated guard service.
        from python_qt_binding.QtWidgets import QApplication
        from bunker_tag_follower.control_panel import ControlPanel
        from sensor_msgs.msg import Image
        from array import array
        application = QApplication([])
        panel = ControlPanel(node)
        end = time.monotonic()+3
        while time.monotonic()<end and not panel.enable.isEnabled():
            panel.poll()
            application.processEvents()
            time.sleep(.01)
        assert panel.enable.isEnabled(), 'Enable button never became available'
        panel.enable.click()
        end = time.monotonic()+2
        while time.monotonic()<end and states[-1] != 'enabled':
            panel.poll()
            application.processEvents()
            time.sleep(.01)
        assert states[-1] == 'enabled', 'Enable button did not call the service'
        panel.stop.click()
        end = time.monotonic()+2
        while time.monotonic()<end and states[-1] != 'disabled':
            panel.poll()
            application.processEvents()
            time.sleep(.01)
        assert states[-1] == 'disabled', 'Stop button did not disable the guard'
        image = Image()
        image.width, image.height, image.step = 20, 20, 60
        image.encoding = 'bgr8'
        image.data = array('B', [100]) * 1200
        panel.on_image(image)
        assert panel.picture.pixmap() is not None, 'Overlay image did not reach Qt'
        panel.close()
        second = subprocess.run(args, text=True, capture_output=True, timeout=8)
        assert second.returncode != 0 and 'Already running:' in second.stdout+second.stderr
        # Kill the guard process only: its exit handler must stop the rest of the stack.
        kill = subprocess.run(['pgrep', '-f',
            '/install/bunker_tag_follower/lib/bunker_tag_follower/velocity_guard'],
            text=True, capture_output=True)
        # Select the descendant belonging to this isolated launch, not the live robot.
        def descendants(pid):
            result = subprocess.run(['pgrep', '-P', str(pid)], text=True, capture_output=True)
            children = [int(x) for x in result.stdout.split()]
            return children + [child for p in children for child in descendants(p)]
        owned = set(descendants(process.pid))
        candidates = owned & {int(x) for x in kill.stdout.split()}
        assert len(candidates) == 1, candidates
        os.kill(candidates.pop(), signal.SIGTERM)
        process.wait(timeout=8)
        print('PASS: combined nodes, Qt image and enable/stop buttons, disabled startup, duplicate prevention, guard-exit shutdown')
    except BaseException:
        log.seek(0)
        print(log.read())
        raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == '__main__':
    main()
