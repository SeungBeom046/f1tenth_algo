"""AEB (Automatic Emergency Braking) ROS2 node for real F1TENTH car."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math


class AEBRealNode(Node):
    """
    실차용 자동 긴급제동 노드
    TTC (Time To Collision) 기반으로 충돌 위험 감지 시 즉시 제동.
    wall_follow_real 노드와 독립적으로 동작.
    """

    def __init__(self):
        super().__init__('aeb_real_node')

        # ============ 튜닝 파라미터 ============
        self.ttc_threshold = 0.5   # TTC 임계값 (초)
        self.min_speed = 0.1       # 이 속도 이하면 AEB 비활성화
        # ======================================

        self.current_speed = 0.0

        # QoS 설정 (pointcloud_to_laserscan과 호환)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # LiDAR 구독 (QoS 적용)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)

        # 실차 오도메트리 구독 (VESC에서 발행)
        self.odom_sub = self.create_subscription(
            Odometry, '/vesc/odom', self.odom_callback, 10)

        # VESC 속도 명령
        self.speed_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)

        self.get_logger().info('AEB Real Node 시작!')

    def odom_callback(self, msg):
        """현재 속도 업데이트"""
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        """
        TTC 계산 후 긴급제동 판단
        TTC = 거리 / 속도
        TTC < 임계값이면 즉시 제동
        """
        if abs(self.current_speed) < self.min_speed:
            return

        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if math.isnan(r) or math.isinf(r):
                angle += scan_msg.angle_increment
                continue

            speed_component = self.current_speed * math.cos(angle)

            if speed_component > 0:
                ttc = r / speed_component
                if ttc < self.ttc_threshold:
                    self.emergency_brake()
                    print(
                        f'⚠️ AEB 작동! TTC: {ttc:.2f}s | '
                        f'거리: {r:.2f}m | '
                        f'속도: {self.current_speed:.2f}m/s',
                        flush=True
                    )
                    return

            angle += scan_msg.angle_increment

    def emergency_brake(self):
        """긴급 제동 - 속도 0으로"""
        speed_msg = Float64()
        speed_msg.data = 0.0
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AEBRealNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Float64()
        stop_msg.data = 0.0
        node.speed_pub.publish(stop_msg)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()