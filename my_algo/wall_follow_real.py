"""Real car wall-following node for F1TENTH with Livox Mid-360."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math


class WallFollowRealNode(Node):
    """
    실차용 Wall Following 노드
    - LiDAR: Livox Mid-360 (PointCloud2 → LaserScan 변환 후 수신)
    - 제어: VESC (UART, /commands/motor/speed + /commands/servo/position)
    """

    def __init__(self):
        super().__init__('wall_follow_real_node')

        # ============ 튜닝 파라미터 ============
        self.kp = 1.2
        self.kd = 0.05
        self.safety_kp = 2.8
        self.target_dist = 1.0
        self.min_wall_dist = 0.8
        self.hard_wall_dist = 0.45
        self.front_slow_dist = 1.8
        self.front_stop_dist = 0.75
        self.lookahead = 0.65
        self.max_steer = 0.42

        # ============ 실차 변환 파라미터 ============
        # 속도 변환: m/s → ERPM
        # ERPM = 속도(m/s) * ERPM_GAIN
        # ERPM_GAIN = (60 * 모터KV * 배터리전압) / (2π * 바퀴반경 * 기어비)
        # → 실측으로 구하는 게 정확함 (아래 캘리브레이션 방법 참고)
        self.ERPM_GAIN = 4614.0  # 실측 후 수정 필요

        # 조향 변환: 라디안 → 서보 위치 (0.0 ~ 1.0)
        # 0.5 = 중앙, 0.0 = 최대 우회전, 1.0 = 최대 좌회전
        self.SERVO_CENTER = 0.5  # 실측 후 수정 필요
        self.SERVO_GAIN = 0.4    # 실측 후 수정 필요
        # ========================================

        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # LiDAR 구독 (pointcloud_to_laserscan 변환 후)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # VESC 속도 명령 (ERPM)
        self.speed_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)

        # VESC 조향 명령 (서보 위치 0.0~1.0)
        self.servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)

        self.get_logger().info('Wall Follow Real Node 시작!')
        self.get_logger().info(
            f'ERPM_GAIN: {self.ERPM_GAIN} | '
            f'SERVO_CENTER: {self.SERVO_CENTER} | '
            f'SERVO_GAIN: {self.SERVO_GAIN}'
        )

    def get_range(self, scan_msg, angle_deg):
        """특정 각도(도)의 LiDAR 거리값 반환"""
        angle_rad = math.radians(angle_deg)
        span = scan_msg.angle_max - scan_msg.angle_min
        if span >= 2.0 * math.pi - 0.01:
            angle_rad = (
                (angle_rad - scan_msg.angle_min) % span
            ) + scan_msg.angle_min

        index = int((angle_rad - scan_msg.angle_min)
                    / scan_msg.angle_increment)
        index = max(0, min(index, len(scan_msg.ranges) - 1))
        r = scan_msg.ranges[index]
        if math.isnan(r) or math.isinf(r):
            return scan_msg.range_max
        return max(scan_msg.range_min, min(r, scan_msg.range_max))

    def get_sector_min(self, scan_msg, center_deg, width_deg, step_deg=2):
        """각도 구간 최솟값"""
        start = center_deg - width_deg / 2.0
        end = center_deg + width_deg / 2.0
        min_r = scan_msg.range_max
        angle = start
        while angle <= end:
            min_r = min(min_r, self.get_range(scan_msg, angle))
            angle += step_deg
        return min_r

    def get_wall_distance(self, scan_msg, side='right'):
        """벽까지 수직 거리 계산"""
        theta = 50
        if side == 'right':
            a = self.get_range(scan_msg, -90 + theta)
            b = self.get_range(scan_msg, -90)
        else:
            a = self.get_range(scan_msg, 90 - theta)
            b = self.get_range(scan_msg, 90)

        theta_rad = math.radians(theta)
        alpha = math.atan2(
            a * math.cos(theta_rad) - b,
            a * math.sin(theta_rad)
        )
        dist = b * math.cos(alpha)
        return dist + self.lookahead * math.sin(alpha)

    def publish_command(self, steering_rad, speed_ms):
        """
        조향각(라디안)과 속도(m/s)를 VESC 명령으로 변환해서 발행

        [속도 변환]
        ERPM = speed(m/s) * ERPM_GAIN

        [조향 변환]
        servo = SERVO_CENTER - steering_rad * SERVO_GAIN
        (+steering = 왼쪽, -steering = 오른쪽)
        servo 범위: 0.0(우) ~ 0.5(중앙) ~ 1.0(좌)
        """
        # 속도 변환 (m/s → ERPM)
        erpm = speed_ms * self.ERPM_GAIN
        speed_msg = Float64()
        speed_msg.data = erpm
        self.speed_pub.publish(speed_msg)

        # 조향 변환 (라디안 → 서보 위치)
        servo_pos = self.SERVO_CENTER - steering_rad * self.SERVO_GAIN
        servo_pos = max(0.0, min(1.0, servo_pos))  # 0.0 ~ 1.0 제한
        servo_msg = Float64()
        servo_msg.data = servo_pos
        self.servo_pub.publish(servo_msg)

    def stop(self):
        """긴급 정지"""
        speed_msg = Float64()
        speed_msg.data = 0.0
        self.speed_pub.publish(speed_msg)

        servo_msg = Float64()
        servo_msg.data = self.SERVO_CENTER  # 조향 중앙으로
        self.servo_pub.publish(servo_msg)
        print('차 정지!')

    def scan_callback(self, scan_msg):
        """LiDAR 데이터 → VESC 제어 명령 변환"""

        # 1. 거리 측정
        right_dist = self.get_wall_distance(scan_msg, side='right')
        left_dist = self.get_wall_distance(scan_msg, side='left')
        front_min = self.get_sector_min(scan_msg, 0, 40)
        right_min = self.get_sector_min(scan_msg, -90, 70)
        left_min = self.get_sector_min(scan_msg, 90, 70)

        # 2. 오차 계산 (중앙 유지)
        center_error = left_dist - right_dist

        # 3. 시간 간격
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        dt = max(dt, 1e-3)

        # 4. PD 제어
        p_term = self.kp * center_error
        derivative = (center_error - self.prev_error) / dt
        derivative = max(-5.0, min(5.0, derivative))
        d_term = self.kd * derivative

        # 5. 벽 안전거리 보정
        safety_error = 0.0
        if right_min < self.min_wall_dist:
            safety_error += self.min_wall_dist - right_min
        if left_min < self.min_wall_dist:
            safety_error -= self.min_wall_dist - left_min

        steering = p_term + d_term + self.safety_kp * safety_error

        # 6. 전방 장애물 보정
        if front_min < self.front_slow_dist:
            turn_to_open = 1.0 if left_min > right_min else -1.0
            ratio = (self.front_slow_dist - front_min) / (
                self.front_slow_dist - self.front_stop_dist)
            steering += turn_to_open * max(0.0, min(1.0, ratio)) * 0.28

        # 조향각 제한
        steering = max(-self.max_steer, min(self.max_steer, steering))

        # 7. 속도 결정 (실차는 시뮬보다 훨씬 보수적으로)
        abs_steer = abs(steering)
        nearest_wall = min(left_min, right_min)

        if front_min < self.front_stop_dist:
            speed = 0.0   # 전방 장애물 → 즉시 정지
        elif nearest_wall < self.hard_wall_dist:
            speed = 0.3   # 벽 너무 가까움 → 극저속
        elif front_min < self.front_slow_dist or abs_steer > 0.25:
            speed = 0.5   # 코너 → 저속
        elif abs_steer > 0.12:
            speed = 0.8   # 완만한 코너 → 중속
        else:
            speed = 1.2   # 직선 → 중고속
            # ⚠️ 처음엔 1.2 이상 올리지 말 것!
            # 실차 테스트 안정 확인 후 조금씩 올리기

        # 8. VESC에 명령 발행
        self.publish_command(steering, speed)

        # 9. 로그
        print(
            f'R: {right_dist:.2f}m | L: {left_dist:.2f}m | '
            f'Fmin: {front_min:.2f}m | '
            f'center_err: {center_error:.2f} | '
            f'steer: {math.degrees(steering):.1f}deg | '
            f'speed: {speed:.1f}m/s | '
            f'ERPM: {speed * self.ERPM_GAIN:.0f} | '
            f'servo: {self.SERVO_CENTER - steering * self.SERVO_GAIN:.3f}',
            flush=True
        )

        self.prev_error = center_error
        self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowRealNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()