#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class FollowLeader(Node):
    def __init__(self):
        super().__init__('follow_leader')

        # Publisher: robot1 cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Odometry, '/robot2/odom', self.leader_callback, 10)
        self.create_subscription(Odometry, '/robot1/odom', self.follower_callback, 10)

        self.leader_pose = None
        self.follower_pose = None

        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        # Target parameters
        self.target_distance = 2.0     # 유지 거리
        self.max_speed = 0.4           # 속도 제한
        self.max_angular = 1.0         # 회전 제한

        # -----------------------------
        # PID parameters (distance PID)
        # -----------------------------
        self.Kp = 1.2
        self.Ki = 0.1
        self.Kd = 0.4

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def leader_callback(self, msg):
        self.leader_pose = msg.pose.pose

    def follower_callback(self, msg):
        self.follower_pose = msg.pose.pose

    def get_yaw(self, orientation):
        q = orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # ======================================================
    #                  PID Controller
    # ======================================================
    def pid_distance(self, distance_error):
        cur_time = time.time()
        dt = cur_time - self.prev_time
        self.prev_time = cur_time

        # Integral (Anti-windup)
        self.integral += distance_error * dt
        self.integral = max(min(self.integral, 3.0), -3.0)

        # Derivative
        derivative = (distance_error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = distance_error

        # PID output
        output = (self.Kp * distance_error) + (self.Ki * self.integral) + (self.Kd * derivative)
        return output

    # ======================================================
    #                  CONTROL LOOP
    # ======================================================
    def control_loop(self):
        if self.leader_pose is None or self.follower_pose is None:
            return

        # Leader pose
        lx = self.leader_pose.position.x
        ly = self.leader_pose.position.y
        lyaw = self.get_yaw(self.leader_pose.orientation)

        # Follower pose
        fx = self.follower_pose.position.x
        fy = self.follower_pose.position.y
        fyaw = self.get_yaw(self.follower_pose.orientation)

        # Calculate target point 2m behind leader
        target_x = lx - self.target_distance * math.cos(lyaw)
        target_y = ly - self.target_distance * math.sin(lyaw)

        dx = target_x - fx
        dy = target_y - fy

        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_target = math.atan2(dy, dx)

        # yaw error
        yaw_error = angle_to_target - fyaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # -------------------------------------------
        # 1) Angular control (pure pursuit style)
        # -------------------------------------------
        angular_cmd = max(-self.max_angular,
                          min(self.max_angular, yaw_error * 2.0))

        # -------------------------------------------
        # 2) Linear control (PID distance control)
        # distance_error <0 → 너무 가까이 있음 → 뒤로 감
        # -------------------------------------------
        distance_error = distance  # NOT distance - target_distance (target 자체를 뒤로 설정했음)
        linear_cmd = self.pid_distance(distance_error)

        # limit
        linear_cmd = max(-self.max_speed, min(self.max_speed, linear_cmd))

        # -------------------------------------------
        # Publish
        # -------------------------------------------
        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowLeader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
