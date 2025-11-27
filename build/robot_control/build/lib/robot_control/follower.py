import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Follower(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Leader odom subscriber (robot2)
        self.create_subscription(Odometry, '/robot2/odom', self.leader_callback, 10)

        # Follower cmd_vel publisher (robot1)
        self.cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        # 저장 변수
        self.leader_x = 0.0
        self.leader_y = 0.0
        self.follower_x = 0.0
        self.follower_y = 0.0

        # follower odom
        self.create_subscription(Odometry, '/robot1/odom', self.follower_callback, 10)

        self.timer = self.create_timer(0.1, self.follow_logic)

    def leader_callback(self, msg):
        self.leader_x = msg.pose.pose.position.x
        self.leader_y = msg.pose.pose.position.y

    def follower_callback(self, msg):
        self.follower_x = msg.pose.pose.position.x
        self.follower_y = msg.pose.pose.position.y

    def follow_logic(self):
        # Leader와 follower 거리 계산
        dx = self.leader_x - self.follower_x
        dy = self.leader_y - self.follower_y
        dist = math.sqrt(dx*dx + dy*dy)

        # P 제어
        linear_gain = 0.5
        angular_gain = 2.0

        # 목표 방향
        target_yaw = math.atan2(dy, dx)

        # follower의 yaw 추출
        # orientation에서 yaw만 구함
        # quaternion → yaw
        yaw = self.get_yaw()

        yaw_error = target_yaw - yaw

        # 메시지 생성
        cmd = Twist()

        # Leader와의 거리가 0.5m 이상일 때 따라가기
        if dist > 0.5:
            cmd.linear.x = linear_gain * dist
            cmd.angular.z = angular_gain * yaw_error
        else:
            # 가까우면 정지
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # publish
        self.cmd_pub.publish(cmd)

    def get_yaw(self):
        # follower orientation quaternion → yaw 변환
        try:
            msg = self.latest_follower_odom
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except:
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
