#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from PyQt5.QtWidgets import (
    QApplication, QGraphicsView, QGraphicsScene,
    QGraphicsEllipseItem
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPen, QBrush


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_visualizer')

        self.robot1_pos = (0.0, 0.0)
        self.robot2_pos = (0.0, 0.0)

        # Subscribe to topics
        self.create_subscription(Odometry, '/odom', self.odom1_cb, 10)
        self.create_subscription(Odometry, '/leader/odom', self.odom2_cb, 10)

        self.get_logger().info("Odom visualizer is running...")

    def odom1_cb(self, msg):
        self.robot1_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def odom2_cb(self, msg):
        self.robot2_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )


class OdomGUI(QGraphicsView):
    def __init__(self, node: OdomNode):
        super().__init__()

        self.node = node

        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Scale factor (meter → pixel)
        self.scale_factor = 50

        # Draw robot dots
        self.robot1_item = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.robot1_item.setBrush(QBrush(Qt.blue))
        self.scene.addItem(self.robot1_item)

        self.robot2_item = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.robot2_item.setBrush(QBrush(Qt.red))
        self.scene.addItem(self.robot2_item)

        # Timer for updating graphic positions
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_positions)
        self.timer.start(30)

        self.setWindowTitle("ROS2 Odometry Viewer (PyQt)")
        self.setGeometry(100, 100, 800, 600)

    def update_positions(self):
        x1, y1 = self.node.robot1_pos
        x2, y2 = self.node.robot2_pos

        # Convert meters → pixels
        self.robot1_item.setPos(x1 * self.scale_factor, -y1 * self.scale_factor)
        self.robot2_item.setPos(x2 * self.scale_factor, -y2 * self.scale_factor)


def main():
    rclpy.init()

    node = OdomNode()

    app = QApplication(sys.argv)
    gui = OdomGUI(node)
    gui.show()

    # PyQt와 ROS2를 동시에 돌리는 핵심
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
