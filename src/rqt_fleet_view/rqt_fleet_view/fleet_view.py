import math
import rclpy
from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter, QColor, QPen, QBrush
from python_qt_binding.QtCore import Qt

from nav_msgs.msg import Odometry

class FleetWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.leader_pos = None
        self.follower_pos = None

    def update_positions(self, leader, follower):
        self.leader_pos = leader
        self.follower_pos = follower
        self.update()  # GUI 재렌더링

    ## ---- 실제 그림 그리는 부분 ---- ##
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 배경
        painter.fillRect(self.rect(), QColor(240, 240, 240))

        center_x = self.width() / 2
        center_y = self.height() / 2

        scale = 20.0   # 1m = 20px

        # follower (내 차량)
        if self.follower_pos:
            x = center_x + self.follower_pos[0] * scale
            y = center_y - self.follower_pos[1] * scale
            painter.setBrush(QBrush(Qt.blue))
            painter.drawEllipse(x - 6, y - 6, 12, 12)
            painter.drawText(x + 10, y, "Follower")

        # leader
        if self.leader_pos:
            x = center_x + self.leader_pos[0] * scale
            y = center_y - self.leader_pos[1] * scale
            painter.setBrush(QBrush(Qt.red))
            painter.drawEllipse(x - 6, y - 6, 12, 12)
            painter.drawText(x + 10, y, "Leader")

        # 연관선
        if self.follower_pos and self.leader_pos:
            painter.setPen(QPen(Qt.darkGray, 2, Qt.DashLine))
            fx = center_x + self.follower_pos[0] * scale
            fy = center_y - self.follower_pos[1] * scale
            lx = center_x + self.leader_pos[0] * scale
            ly = center_y - self.leader_pos[1] * scale
            painter.drawLine(fx, fy, lx, ly)


class FleetViewPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("FleetViewPlugin")

        self._widget = FleetWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle()
                + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        # ROS2 node
        self.node = rclpy.create_node("fleet_view_rqt")

        # Subscribers
        self.sub_leader = self.node.create_subscription(
            Odometry, "/leader/odom", self.cb_leader, 10
        )
        self.sub_follower = self.node.create_subscription(
            Odometry, "/odom", self.cb_follower, 10
        )

        self.leader_pos = None
        self.follower_pos = None

    def cb_leader(self, msg):
        self.leader_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._widget.update_positions(self.leader_pos, self.follower_pos)

    def cb_follower(self, msg):
        self.follower_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._widget.update_positions(self.leader_pos, self.follower_pos)

    def shutdown_plugin(self):
        self.node.destroy_node()
