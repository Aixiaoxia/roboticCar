"""Single-window image, status and explicit enable/stop controls."""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout)


class ControlPanel(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Bunker AprilTag 跟随')
        self.resize(800, 720)
        self.client = node.create_client(SetBool, '/follow/enable')
        self.pending = None
        node.declare_parameter('hardware', False)
        self.warning = QLabel('实车模式：使能后车辆可能立即移动' if
                              node.get_parameter('hardware').value else '预览模式：仅计算速度')
        self.warning.setStyleSheet('font-size: 18px; font-weight: bold;')
        self.picture = QLabel('等待 /apriltag/image_overlay')
        self.picture.setAlignment(Qt.AlignCenter)
        self.picture.setMinimumSize(640, 480)
        self.guard = QLabel('保护：等待状态')
        self.follow = QLabel('目标：等待状态')
        self.depth = QLabel('深度：等待状态')
        self.result = QLabel('启动后默认未使能')
        self.enable = QPushButton('使能跟随')
        self.stop = QPushButton('停止 / 禁用')
        self.stop.setStyleSheet('background-color: #c83232; color: white; font-size: 20px;')
        self.enable.setMinimumHeight(45)
        self.stop.setMinimumHeight(45)
        self.enable.clicked.connect(lambda: self.request(True))
        self.stop.clicked.connect(lambda: self.request(False))
        buttons = QHBoxLayout()
        buttons.addWidget(self.enable)
        buttons.addWidget(self.stop)
        layout = QVBoxLayout(self)
        for widget in (self.warning, self.picture, self.guard, self.follow, self.depth, self.result):
            layout.addWidget(widget)
        layout.addLayout(buttons)
        self.last = {}
        self.subscriptions = [
            node.create_subscription(String, '/follow/guard_state', self.on_guard, 1),
            node.create_subscription(String, '/follow/state', self.on_follow, 1),
            node.create_subscription(String, '/follow/depth_state', self.on_depth, 1),
            node.create_subscription(Image, '/apriltag/image_overlay', self.on_image,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))]
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(20)

    def request(self, enabled):
        if not self.client.service_is_ready():
            self.result.setText('控制服务尚未就绪，请检查跟随节点')
            return None
        request = SetBool.Request()
        request.data = enabled
        future = self.client.call_async(request)
        self.result.setText('正在请求使能' if enabled else '正在请求停止')
        def done(result):
            try:
                response = result.result()
                self.result.setText(('成功：' if response.success else '未通过：') + response.message)
            except Exception as error:
                self.result.setText('服务调用失败：' + str(error))
        future.add_done_callback(done)
        return future

    def on_guard(self, msg):
        self.last['guard'] = time.monotonic()
        self.guard.setText('保护：' + msg.data)

    def on_follow(self, msg):
        self.last['follow'] = time.monotonic()
        try:
            state = json.loads(msg.data)
            forward = state.get('forward_m')
            distance = '--' if forward is None else f'{forward:.2f}'
            position_label = 'KF前方' if state.get('kf_enabled') else '前方'
            self.follow.setText(f"目标：{state['state']} | {position_label} {distance} m | "
                                f"期望前进 {state['linear_mps']:.3f} m/s | "
                                f"转向 {state['angular_rps']:.3f} rad/s")
        except (ValueError, KeyError, TypeError):
            self.follow.setText('目标状态格式异常')

    def on_depth(self, msg):
        self.last['depth'] = time.monotonic()
        try:
            state = json.loads(msg.data)
            nearest = state.get('nearest_m')
            text = '--' if nearest is None else f'{nearest:.2f} m'
            self.depth.setText(f"深度：{'通过' if state['clear'] else '停车'} | "
                               f"{state['reason']} | 最近 {text}")
        except (ValueError, KeyError, TypeError):
            self.depth.setText('深度状态格式异常')

    def on_image(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8'):
            return
        self.last['image'] = time.monotonic()
        image = QImage(bytes(msg.data), msg.width, msg.height, msg.step,
                       QImage.Format_RGB888)
        image = image.rgbSwapped() if msg.encoding == 'bgr8' else image.copy()
        self.picture.setPixmap(QPixmap.fromImage(image).scaled(
            self.picture.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def poll(self):
        if not rclpy.ok():
            self.timer.stop()
            self.close()
            return
        for _ in range(6):
            rclpy.spin_once(self.node, timeout_sec=0)
        now = time.monotonic()
        self.enable.setEnabled(self.client.service_is_ready()
                               and now-self.last.get('guard', 0) < 1)
        for key, widget in [('guard', self.guard), ('follow', self.follow), ('depth', self.depth)]:
            if key in self.last and now-self.last[key] > 1:
                widget.setText(key + '：数据已过期')
        if 'image' in self.last and now-self.last['image'] > 1:
            self.picture.clear()
            self.picture.setText('画面已停止更新')

    def closeEvent(self, event):
        # Closing the panel requests a stop; the terminal owns node lifetimes.
        self.timer.stop()
        future = self.request(False) if rclpy.ok() else None
        end = time.monotonic()+1.0
        while future is not None and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.02)
        event.accept()


def main():
    rclpy.init()
    node = None
    try:
        application = QApplication(sys.argv)
        node = Node('follow_control_panel')
        panel = ControlPanel(node)
        panel.show()
        application.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
