"""Real car wall-following node for F1TENTH with Livox Mid-360."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

from my_algo.vesc_utils import (
    ERPM_GAIN,
    MIN_DRIVE_ERPM,
    MIN_DRIVE_SPEED_MS,
    apply_min_drive_speed,
    speed_to_erpm,
)


class WallFollowRealNode(Node):
    """
    실차용 Wall Following 노드
    - LiDAR: Livox Mid-360 (PointCloud2 → LaserScan 변환 후 수신)
    - 제어: VESC (UART, /commands/motor/speed + /commands/servo/position)
    - 조이스틱 제어 중이면 자동으로 양보
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
        self.front_slow_dist = 1.0
        self.front_stop_dist = 0.45
        self.close_obstacle_dist = 0.35
        self.lookahead = 0.65
        self.max_steer = 0.42
        # LiDAR is mounted 90 deg clockwise from the datasheet frame:
        # vehicle front is +90 deg in the raw LiDAR/LaserScan frame.
        self.lidar_yaw_offset_deg = 90.0

        # ============ 실차 변환 파라미터 ============
        self.ERPM_GAIN = ERPM_GAIN
        self.SERVO_CENTER = 0.5   # 실측 후 수정 필요
        self.SERVO_GAIN = 0.4     # 실측 후 수정 필요
        # ==========================================

        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # 조이스틱/자율주행 모드 상태
        self.joy_active = False
        self.auto_mode = False

        # QoS 설정 (pointcloud_to_laserscan과 호환)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # LiDAR 구독 (QoS 적용)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)

        # VESC 제어 발행
        self.speed_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)

        # 조이스틱 활성 상태 구독
        self.joy_active_sub = self.create_subscription(
            Bool, '/joy_active', self.joy_active_callback, 10)

        # 자율주행 모드 구독
        self.auto_mode_sub = self.create_subscription(
            Bool, '/autonomous_mode', self.auto_mode_callback, 10)

        self.get_logger().info('Wall Follow Real Node 시작!')
        self.get_logger().info(
            f'ERPM_GAIN: {self.ERPM_GAIN} | '
            f'MIN_DRIVE: {MIN_DRIVE_SPEED_MS:.2f}m/s '
            f'({MIN_DRIVE_ERPM:.0f} ERPM) | '
            f'SERVO_CENTER: {self.SERVO_CENTER} | '
            f'SERVO_GAIN: {self.SERVO_GAIN}'
        )

    # ============ 모드 콜백 ============

    def joy_active_callback(self, msg):
        """조이스틱 활성 상태 업데이트"""
        self.joy_active = msg.data

    def auto_mode_callback(self, msg):
        """자율주행 모드 상태 업데이트"""
        prev = self.auto_mode
        self.auto_mode = msg.data
        if self.auto_mode != prev:
            mode_str = '자율주행 ON' if self.auto_mode else '자율주행 OFF'
            print(f'[WallFollow] {mode_str}', flush=True)

    # ============ LiDAR 유틸 ============

    def vehicle_to_lidar_angle(self, vehicle_angle_deg):
        """차량 기준 각도(전방 0도)를 실제 LiDAR scan 각도로 변환"""
        return math.radians(vehicle_angle_deg + self.lidar_yaw_offset_deg)

    def normalize_scan_angle(self, scan_msg, angle_rad):
        """scan 범위 안으로 각도를 정규화"""
        span = scan_msg.angle_max - scan_msg.angle_min
        if span >= 2.0 * math.pi - 0.01:
            return ((angle_rad - scan_msg.angle_min) % span) + scan_msg.angle_min
        return angle_rad

    def sanitize_range(self, scan_msg, r):
        """LiDAR 값을 장애물 판단에 안전한 거리값으로 변환"""
        if math.isnan(r) or math.isinf(r):
            return None
        if r <= 0.0:
            return 0.0
        return min(r, scan_msg.range_max)

    def get_range(self, scan_msg, angle_deg):
        """특정 각도(도)의 LiDAR 거리값 반환"""
        angle_rad = self.normalize_scan_angle(
            scan_msg, self.vehicle_to_lidar_angle(angle_deg))

        index = int((angle_rad - scan_msg.angle_min)
                    / scan_msg.angle_increment)
        index = max(0, min(index, len(scan_msg.ranges) - 1))
        r = self.sanitize_range(scan_msg, scan_msg.ranges[index])
        if r is None:
            return scan_msg.range_max
        return r

    def angle_in_sector(self, angle, center, half_width):
        """라디안 각도가 섹터 안에 있는지 확인"""
        diff = math.atan2(math.sin(angle - center), math.cos(angle - center))
        return abs(diff) <= half_width

    def get_sector_min(self, scan_msg, center_deg, width_deg):
        """
        각도 구간 최솟값 반환.

        모든 LaserScan bin을 검사해서 2도 샘플링 때문에 작은/가까운 장애물을
        건너뛰지 않도록 한다. range_min보다 작은 유효값도 가까운 장애물로 본다.
        """
        center = self.vehicle_to_lidar_angle(center_deg)
        half_width = math.radians(width_deg / 2.0)
        min_r = scan_msg.range_max
        found = False

        for i, r in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            if not self.angle_in_sector(angle, center, half_width):
                continue
            r = self.sanitize_range(scan_msg, r)
            if r is None:
                continue
            if r <= scan_msg.range_max:
                min_r = min(min_r, r)
                found = True

        return min_r if found else scan_msg.range_max

    def get_wall_distance(self, scan_msg, side='right'):
        """
        벽까지 수직 거리 계산
        두 빔(a, b)과 사이각도(theta)로 삼각함수 계산:
        alpha = atan2(a*cos(θ) - b, a*sin(θ))
        수직거리 D = b * cos(alpha)
        lookahead: 코너에서 미리 반응
        """
        theta = 50
        if side == 'right':
            a = self.get_range(scan_msg, -90 + theta)  # -40도
            b = self.get_range(scan_msg, -90)           # -90도
        else:
            a = self.get_range(scan_msg, 90 - theta)   # +40도
            b = self.get_range(scan_msg, 90)            # +90도

        theta_rad = math.radians(theta)
        alpha = math.atan2(
            a * math.cos(theta_rad) - b,
            a * math.sin(theta_rad)
        )
        dist = b * math.cos(alpha)
        return dist + self.lookahead * math.sin(alpha)

    def clamp(self, value, low, high):
        """값 범위 제한"""
        return max(low, min(value, high))

    # ============ VESC 제어 ============

    def publish_command(self, steering_rad, speed_ms):
        """
        조향각(라디안)과 속도(m/s)를 VESC 명령으로 변환해서 발행

        [속도 변환]
        ERPM = speed(m/s) * ERPM_GAIN
        최소 구동 속도 적용 (모터 탈조 방지)

        [조향 변환]
        servo = SERVO_CENTER - steering_rad * SERVO_GAIN
        (+steering = 왼쪽, -steering = 오른쪽)
        servo 범위: 0.0(우) ~ 0.5(중앙) ~ 1.0(좌)
        """
        # 속도 변환 (m/s → ERPM, 최소 구동속도 적용)
        adjusted_speed_ms = apply_min_drive_speed(speed_ms)
        erpm = speed_to_erpm(adjusted_speed_ms)
        speed_msg = Float64()
        speed_msg.data = erpm
        self.speed_pub.publish(speed_msg)

        # 조향 변환 (라디안 → 서보 위치)
        servo_pos = self.SERVO_CENTER - steering_rad * self.SERVO_GAIN
        servo_pos = self.clamp(servo_pos, 0.0, 1.0)
        servo_msg = Float64()
        servo_msg.data = servo_pos
        self.servo_pub.publish(servo_msg)

    def stop(self):
        """긴급 정지"""
        speed_msg = Float64()
        speed_msg.data = 0.0
        self.speed_pub.publish(speed_msg)

        servo_msg = Float64()
        servo_msg.data = self.SERVO_CENTER
        self.servo_pub.publish(servo_msg)
        print('[WallFollow] 차 정지!', flush=True)

    # ============ 메인 콜백 ============

    def scan_callback(self, scan_msg):
        """LiDAR 데이터 → VESC 제어 명령 변환"""

        # 조이스틱 제어 중이거나 자율주행 모드 아니면 스킵
        if self.joy_active or not self.auto_mode:
            return

        # 1. 거리 측정
        right_dist = self.get_wall_distance(scan_msg, side='right')
        left_dist = self.get_wall_distance(scan_msg, side='left')
        front_min = self.get_sector_min(scan_msg, 0, 50)
        right_min = self.get_sector_min(scan_msg, -90, 70)
        left_min = self.get_sector_min(scan_msg, 90, 70)

        # 2. 오차 계산 (중앙 유지)
        # left > right → 오른쪽 벽이 가까움 → 왼쪽으로 꺾어야 함
        center_error = left_dist - right_dist

        # 3. 시간 간격
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        dt = max(dt, 1e-3)

        # 4. PD 제어
        p_term = self.kp * center_error
        derivative = (center_error - self.prev_error) / dt
        derivative = self.clamp(derivative, -5.0, 5.0)
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
            steering += turn_to_open * self.clamp(ratio, 0.0, 1.0) * 0.28

        # 조향각 제한
        steering = self.clamp(steering, -self.max_steer, self.max_steer)

        # 7. 속도 결정
        # ⚠️ 실차는 시뮬보다 훨씬 보수적으로 시작!
        abs_steer = abs(steering)
        nearest_wall = min(left_min, right_min)

        if front_min <= max(self.front_stop_dist, self.close_obstacle_dist):
            speed = 0.0    # 전방 장애물 → 즉시 정지
        elif nearest_wall < self.hard_wall_dist:
            speed = 0.4    # 벽 너무 가까움 → 최소 안정 구동속도
        elif front_min < self.front_slow_dist or abs_steer > 0.25:
            speed = 0.5    # 코너 → 저속
        elif abs_steer > 0.12:
            speed = 0.8    # 완만한 코너 → 중속
        else:
            speed = 1.2    # 직선 → 중고속

        # 8. VESC에 명령 발행
        self.publish_command(steering, speed)
        adjusted_speed = apply_min_drive_speed(speed)

        # 9. 로그
        print(
            f'R: {right_dist:.2f}m | L: {left_dist:.2f}m | '
            f'Fmin: {front_min:.2f}m | '
            f'center_err: {center_error:.2f} | '
            f'steer: {math.degrees(steering):.1f}deg | '
            f'speed: {adjusted_speed:.1f}m/s | '
            f'ERPM: {speed_to_erpm(adjusted_speed):.0f} | '
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
