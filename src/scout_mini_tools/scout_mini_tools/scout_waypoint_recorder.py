import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # 파라미터: 최소 간격 (m)
        self.declare_parameter('min_dist', 0.5)
        self.min_dist = self.get_parameter('min_dist').value

        # tf2 버퍼 & 리스너 (sim_time 사용하니까 queue_time 크게)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.last_x = None
        self.last_y = None

        # 10 Hz로 pose 체크
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"WaypointRecorder started. min_dist = {self.min_dist} m")

    def timer_callback(self):
        try:
            # 최신 transform: map -> base_link
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', now
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            if self.last_x is None:
                # 첫 번째 포인트는 무조건 저장
                self._add_waypoint(x, y)
                self.last_x = x
                self.last_y = y
                return

            dist = math.hypot(x - self.last_x, y - self.last_y)

            if dist >= self.min_dist:
                self._add_waypoint(x, y)
                self.last_x = x
                self.last_y = y

        except (LookupException, ConnectivityException, ExtrapolationException):
            # tf 아직 준비 안 됐을 때
            self.get_logger().warn('TF (odom -> base_link) 를 아직 가져오지 못했습니다.')
            return

    def _add_waypoint(self, x, y):
        self.waypoints.append([float(x), float(y)])
        self.get_logger().info(f"Add waypoint #{len(self.waypoints)-1}: x={x:.3f}, y={y:.3f}")

    def save_waypoints(self, filename='waypoints.yaml'):
        data = {'waypoints': self.waypoints}
        with open(filename, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 파일로 저장
        node.save_waypoints('waypoints_gazebo_floor5_fixed.yaml')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
