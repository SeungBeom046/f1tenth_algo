"""AEB (Automatic Emergency Braking) ROS2 node."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math


class AEBNode(Node):
    """
    자동 긴급제동 노드.
    TTC (Time To Collision) 기반으로 충돌 위험 감지 시 즉시 제동.
    Wall Following 노드와 독립적으로 동작.
    """

    def __init__(self):
        super().__init__('aeb_node')

        # ============ 튜닝 파라미터 ============
        self.ttc_threshold = 0.5   # TTC 임계값 (초) - 이 시간 이내 충돌 예상 시 제동
        self.min_speed = 0.1       # 이 속도 이하면 AEB 비활성화 (정지 상태)
        # ======================================

        self.current_speed = 0.0   # 현재 속도

        # LiDAR 구독
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # 속도 구독 (오도메트리)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # 제동 명령 발행
        self.publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('AEB Node 시작!')

    def odom_callback(self, msg):
        """현재 속도 업데이트"""
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        """
        TTC (Time To Collision) 계산
        
        TTC = 거리 / 속도
        
        각 LiDAR 빔에 대해:
        - 빔 방향의 속도 성분 = current_speed * cos(각도)
        - TTC = 거리 / 속도 성분
        - TTC < 임계값이면 충돌 위험 → 즉시 제동
        """
        if abs(self.current_speed) < self.min_speed:
            return  # 거의 정지 상태면 AEB 불필요

        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            # 유효한 거리값만 처리
            if math.isnan(r) or math.isinf(r):
                angle += scan_msg.angle_increment
                continue

            # 해당 방향의 속도 성분 (전방 기준)
            # cos(angle) > 0 이면 전방 방향 빔
            speed_component = self.current_speed * math.cos(angle)

            if speed_component > 0:  # 전방으로 이동 중인 빔만
                ttc = r / speed_component

                if ttc < self.ttc_threshold:
                    # 충돌 위험! 즉시 제동
                    self.emergency_brake()
                    print(
                        f'⚠️ AEB 작동! TTC: {ttc:.2f}s | '
                        f'거리: {r:.2f}m | 속도: {self.current_speed:.2f}m/s',
                        flush=True
                    )
                    return

            angle += scan_msg.angle_increment

    def emergency_brake(self):
        """긴급 제동 명령 발행"""
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        brake_msg.drive.steering_angle = 0.0
        self.publisher.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()