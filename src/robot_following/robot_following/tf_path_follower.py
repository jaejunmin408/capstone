# robot_follow_odom_follower_with_robot1_odom.py
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RobotFollower(Node):
    def __init__(self):
        super().__init__('robot_follower')

        # === 파라미터 ===
        self.declare_parameter('linear_speed', 0.7)
        self.declare_parameter('k_ang', 1.5)
        self.declare_parameter('max_ang_vel', 1.2)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('follow_distance', 1.0)  # robot1-robot2 거리 유지

        self.linear_speed = self.get_parameter('linear_speed').value
        self.k_ang = self.get_parameter('k_ang').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.follow_distance = self.get_parameter('follow_distance').value

        # robot2 odom subscriber
        self.robot2_odom_sub = self.create_subscription(
            Odometry,
            '/robot2/odom',
            self.robot2_odom_callback,
            10
        )

        # robot1 odom subscriber
        self.robot1_odom_sub = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.robot1_odom_callback,
            10
        )

        # robot2 위치
        self.target_x = None
        self.target_y = None
        self.target_yaw = None

        # robot1 위치
        self.robot1_x = None
        self.robot1_y = None
        self.robot1_yaw = None

        # cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        # 제어 주기
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('RobotFollower node started. Following robot2 via /odom.')

    def robot2_odom_callback(self, msg):
        self.target_x = msg.pose.pose.position.x
        self.target_y = msg.pose.pose.position.y
        self.target_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def robot1_odom_callback(self, msg):
        self.robot1_x = msg.pose.pose.position.x
        self.robot1_y = msg.pose.pose.position.y
        self.robot1_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def timer_callback(self):
        # robot1, robot2 위치 아직 안 들어왔으면 return
        if None in [self.target_x, self.target_y, self.robot1_x, self.robot1_y, self.robot1_yaw]:
            return

        # robot2 상대 위치
        dx = self.target_x - self.robot1_x
        dy = self.target_y - self.robot1_y
        dist_to_target = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_target - self.robot1_yaw)

        # 목표 근처 정지
        if dist_to_target < self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            return

        # 선속도: follow_distance 유지
        v = self.linear_speed
        if dist_to_target < self.follow_distance:
            v *= dist_to_target / self.follow_distance  # 근접하면 감속
        if abs(heading_error) > math.radians(80.0):
            v = 0.0  # 너무 각도 차 크면 회전만

        # 각속도
        omega = self.k_ang * heading_error
        omega = max(min(omega, self.max_ang_vel), -self.max_ang_vel)

        # cmd_vel publish
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_pub.publish(twist)

        # 디버깅
        self.get_logger().debug(
            f'dx={dx:.2f}, dy={dy:.2f}, dist={dist_to_target:.2f}, '
            f'heading_err={math.degrees(heading_error):.1f}, v={v:.2f}, omega={omega:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down RobotFollower node')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
