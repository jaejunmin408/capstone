#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class FollowLeader(Node):
    def __init__(self):
        super().__init__('follow_leader')

        # Publisher: robot1 cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        # Subscriber: leader odom (robot2)
        self.create_subscription(Odometry, '/robot2/odom', self.leader_callback, 10)
        # Subscriber: follower odom (robot1)
        self.create_subscription(Odometry, '/robot1/odom', self.follower_callback, 10)

        self.leader_pose = None
        self.follower_pose = None

        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.target_distance = 2.0     # 2m 유지
        self.max_speed = 1.5           # m/s
        self.max_angular = 2.0         # rad/s

    def leader_callback(self, msg):
        self.leader_pose = msg.pose.pose

    def follower_callback(self, msg):
        self.follower_pose = msg.pose.pose

    def get_yaw(self, orientation):
        """Quaternion → yaw 변환"""
        import math
        q = orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def control_loop(self):
        if self.leader_pose is None or self.follower_pose is None:
            return

        # Leader position
        lx = self.leader_pose.position.x
        ly = self.leader_pose.position.y
        lyaw = self.get_yaw(self.leader_pose.orientation)

        # Follower position
        fx = self.follower_pose.position.x
        fy = self.follower_pose.position.y

        # -------------------------
        # 1) Target point: leader 뒤 2m 지점
        # -------------------------
        target_x = lx - self.target_distance * math.cos(lyaw)
        target_y = ly - self.target_distance * math.sin(lyaw)

        # -------------------------
        # 2) Follower → target 방향/거리 계산
        # -------------------------
        dx = target_x - fx
        dy = target_y - fy

        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_target = math.atan2(dy, dx)

        follower_yaw = self.get_yaw(self.follower_pose.orientation)
        yaw_error = angle_to_target - follower_yaw

        # normalize
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # -------------------------
        # 3) Control (Pure Pursuit 비슷한 단순제어)
        # -------------------------
        cmd = Twist()
        cmd.linear.x  = min(self.max_speed, distance * 0.7)
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, yaw_error * 1.5))

        # publish
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowLeader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
