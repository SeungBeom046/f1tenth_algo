"""AEB (Automatic Emergency Braking) ROS2 node for real F1TENTH car."""

import rclpy
from rclpy.duration import Duration
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
        self.ttc_threshold = 0.5   # 보존용 파라미터. 현재 실차 AEB는 근거리 기준만 사용.
        self.min_speed = 0.1
        self.lidar_to_bumper_dist = 0.10
        self.close_obstacle_clearance = 0.20
        self.close_obstacle_dist = (
            self.close_obstacle_clearance + self.lidar_to_bumper_dist
        )
        self.front_angle_limit = math.radians(60.0)
        self.brake_hold_sec = 0.2
        # LiDAR is mounted 90 deg clockwise from the datasheet frame:
        # vehicle front is +90 deg in the raw LiDAR/LaserScan frame.
        self.lidar_yaw_offset = math.radians(90.0)
        # ======================================

        self.current_speed = 0.0
        self.brake_until = self.get_clock().now()

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
        self.brake_timer = self.create_timer(0.02, self.brake_timer_callback)

        self.get_logger().info('AEB Real Node 시작!')

    def odom_callback(self, msg):
        """현재 속도 업데이트"""
        self.current_speed = msg.twist.twist.linear.x

    def brake_timer_callback(self):
        """브레이크 래치 중이면 wall_follow 명령을 덮어쓰도록 0 속도를 반복 발행"""
        if self.get_clock().now() < self.brake_until:
            self.publish_zero_speed()

    def lidar_to_vehicle_angle(self, lidar_angle):
        """실제 LiDAR scan 각도를 차량 기준 각도(전방 0도)로 변환"""
        angle = lidar_angle - self.lidar_yaw_offset
        return math.atan2(math.sin(angle), math.cos(angle))

    def sanitize_range(self, scan_msg, r):
        """LiDAR 값을 장애물 판단에 안전한 거리값으로 변환"""
        if math.isnan(r) or math.isinf(r):
            return None
        if r <= 0.0:
            return 0.0
        return min(r, scan_msg.range_max)

    def scan_callback(self, scan_msg):
        """
        주행방향 정면 120도 안에서 범퍼 기준 20cm 이하 장애물만 긴급제동.
        """
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            vehicle_angle = self.lidar_to_vehicle_angle(angle)
            r = self.sanitize_range(scan_msg, r)
            if r is None:
                angle += scan_msg.angle_increment
                continue

            if (
                abs(vehicle_angle) <= self.front_angle_limit
                and r <= self.close_obstacle_dist
            ):
                clearance = max(0.0, r - self.lidar_to_bumper_dist)
                self.emergency_brake()
                print(
                    f'⚠️ AEB 근거리 제동! 거리: {r:.2f}m | '
                    f'범퍼여유: {clearance:.2f}m | '
                    f'속도: {self.current_speed:.2f}m/s',
                    flush=True
                )
                return

            angle += scan_msg.angle_increment

    def emergency_brake(self):
        """긴급 제동 - 속도 0으로"""
        self.brake_until = (
            self.get_clock().now()
            + Duration(seconds=self.brake_hold_sec)
        )
        self.publish_zero_speed()

    def publish_zero_speed(self):
        """속도 0 명령 발행"""
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
