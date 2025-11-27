#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveLeaderRobot(Node):
    def __init__(self):
        super().__init__('move_leader_robot')

        # robot2 publisher
        self.pub2 = self.create_publisher(
            Twist,
            '/robot2/cmd_vel',
            10
        )

        # Timer: 0.1초마다 두 로봇 모두에 명령 publish
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()

        # robot2 → 직진
        msg.linear.x = 0.5
        msg.angular.z = 0.0

        self.pub2.publish(msg)

        self.get_logger().info("Moving leader...")


def main(args=None):
    rclpy.init(args=args)
    node = MoveLeaderRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
