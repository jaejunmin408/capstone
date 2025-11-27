#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveTwoRobots(Node):
    def __init__(self):
        super().__init__('move_two_robots')

        # robot1 publisher
        self.pub1 = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )

        # robot2 publisher
        self.pub2 = self.create_publisher(
            Twist,
            '/robot2/cmd_vel',
            10
        )

        # Timer: 0.1초마다 두 로봇 모두에 명령 publish
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg1 = Twist()
        msg2 = Twist()

        # robot1 → 직진
        msg1.linear.x = 0.3
        msg1.angular.z = 0.0

        # robot2 → 직진 + 약간 회전
        msg2.linear.x = 0.3
        msg2.angular.z = 0.0

        self.pub1.publish(msg1)
        self.pub2.publish(msg2)

        self.get_logger().info("Moving robot1 & robot2...")


def main(args=None):
    rclpy.init(args=args)
    node = MoveTwoRobots()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
