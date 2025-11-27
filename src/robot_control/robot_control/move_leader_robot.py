#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

class MoveLeaderRobot(Node):
    def __init__(self):
        super().__init__('move_leader_robot')

        # robot2 publisher
        self.pub2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # flag for emergency stop
        self.stop_flag = False

        # Timer: 0.1초마다 robot2에 명령 publish
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 스레드로 입력 감시
        input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        input_thread.start()

    def keyboard_listener(self):
        while True:
            key = input("Press 1 to stop robot2: ")
            if key.strip() == "1":
                self.stop_flag = True
                self.get_logger().info("Emergency STOP triggered!")

    def timer_callback(self):
        msg = Twist()

        if self.stop_flag:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            # robot2 → 직진
            msg.linear.x = 1.5
            msg.angular.z = 0.0

        self.pub2.publish(msg)
        #self.get_logger().info(f"robot2 cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveLeaderRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
