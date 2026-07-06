"""Wall-following ROS2 node with distance safety guards."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math


class WallFollowNode(Node):
    """Follow the track center while keeping a minimum wall distance."""

    def __init__(self):
        """Initialize ROS interfaces and wall-following parameters."""
        super().__init__('wall_follow_node')

        # ============ 튜닝 파라미터 ============
        self.kp = 0.95             # 중앙 유지 P 게인
        self.kd = 0.11             # 중앙 유지 D 게인
        self.safety_kp = 3.2       # 벽 회피 P 게인
        self.target_dist = 1.0     # 벽까지 목표 거리 (m)
        self.min_wall_dist = 0.8
        self.hard_wall_dist = 0.45  # 너무 가까우면 거의 정지
        self.front_slow_dist = 3.2  # 전방 장애물 감속 시작 거리
        self.front_stop_dist = 0.70
        self.rear_slow_dist = 1.0
        self.rear_stop_dist = 0.45
        self.lookahead = 1.10      # 코너 예측 거리 (m)
        self.max_steer = 0.42
        self.max_speed = 5.0
        self.fast_corner_speed = 3.2
        self.medium_corner_speed = 1.75
        self.tight_corner_speed = 0.95
        self.crawl_speed = 0.25
        self.full_throttle_clear = 4.1
        self.full_throttle_release_clear = 2.7
        self.full_throttle_steer = 0.24
        self.full_throttle_release_steer = 0.34
        self.high_speed_steer = 0.33
        self.trail_brake_steer = 0.08
        self.apex_steer = 0.32
        self.accel_limit = 8.0
        self.brake_limit = 9.0
        self.high_speed_error_deadband = 0.06
        self.stuck_front_dist = 0.85
        self.stuck_side_dist = 1.15
        self.side_open_dist = 1.8
        self.recovery_front_clear = 1.35
        self.reverse_duration = 3.0
        self.escape_duration = 1.2
        self.reverse_speed = -0.35
        self.escape_speed = 0.35
        # ======================================

        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        self.recovery_mode = 'normal'
        self.recovery_until = 0.0
        self.recovery_turn = 1.0
        self.last_debug_time = 0.0
        self.debug_period = 0.25
        self.prev_speed = 0.0
        self.prev_abs_steer = 0.0
        self.straight_mode = False

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Wall Follow Node 시작!')

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

    def get_min_range(self, scan_msg, start_deg, end_deg, step_deg=2):
        """각도 구간 안에서 가장 가까운 거리 반환"""
        if start_deg > end_deg:
            start_deg, end_deg = end_deg, start_deg

        min_range = scan_msg.range_max
        angle = start_deg
        while angle <= end_deg:
            min_range = min(min_range, self.get_range(scan_msg, angle))
            angle += step_deg
        return min_range

    def get_max_range(self, scan_msg, start_deg, end_deg, step_deg=2):
        """각도 구간 안에서 가장 먼 거리 반환"""
        if start_deg > end_deg:
            start_deg, end_deg = end_deg, start_deg

        max_range = scan_msg.range_min
        angle = start_deg
        while angle <= end_deg:
            max_range = max(max_range, self.get_range(scan_msg, angle))
            angle += step_deg
        return max_range

    def get_sector_min(self, scan_msg, center_deg, width_deg, step_deg=2):
        """각도 중심과 폭으로 가장 가까운 거리 반환."""
        start_deg = center_deg - width_deg / 2.0
        end_deg = center_deg + width_deg / 2.0
        return self.get_min_range(scan_msg, start_deg, end_deg, step_deg)

    def get_sector_max(self, scan_msg, center_deg, width_deg, step_deg=2):
        """각도 중심과 폭으로 가장 먼 거리 반환."""
        start_deg = center_deg - width_deg / 2.0
        end_deg = center_deg + width_deg / 2.0
        return self.get_max_range(scan_msg, start_deg, end_deg, step_deg)

    def clamp(self, value, low, high):
        """Limit a value to the inclusive [low, high] range."""
        return max(low, min(value, high))

    def now_sec(self):
        """Return current ROS time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def publish_drive(self, steering, speed):
        """Publish one Ackermann drive command."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed
        self.publisher.publish(drive_msg)

    def get_race_speed(self, front_clear, nearest_wall, abs_steer, dt):
        """F1-like speed plan: full throttle, late brake, trail to apex."""
        if front_clear < self.front_stop_dist:
            self.straight_mode = False
            return 0.0
        if nearest_wall < self.hard_wall_dist:
            self.straight_mode = False
            return self.crawl_speed

        if self.straight_mode:
            if (
                front_clear < self.full_throttle_release_clear
                or abs_steer > self.full_throttle_release_steer
            ):
                self.straight_mode = False
        elif (
            front_clear > self.full_throttle_clear
            and abs_steer < self.full_throttle_steer
        ):
            self.straight_mode = True

        if self.straight_mode:
            target_speed = self.max_speed
        else:
            brake_zone = self.full_throttle_clear - self.front_stop_dist
            clearance = self.clamp(
                (front_clear - self.front_stop_dist) / brake_zone,
                0.0,
                1.0,
            )
            late_brake = (1.0 - clearance) ** 2.2
            entry_speed = (
                self.max_speed
                - late_brake * (self.max_speed - self.tight_corner_speed)
            )

            trail_ratio = self.clamp(
                (abs_steer - self.trail_brake_steer)
                / (self.apex_steer - self.trail_brake_steer),
                0.0,
                1.0,
            )
            corner_speed = (
                self.fast_corner_speed - trail_ratio
                * (self.fast_corner_speed - self.tight_corner_speed)
            )
            trail_speed = self.max_speed - (
                trail_ratio ** 1.35
            ) * (self.max_speed - corner_speed)
            target_speed = min(entry_speed, trail_speed)

            if (
                front_clear > self.front_slow_dist
                and abs_steer < self.high_speed_steer
            ):
                straightish_speed = self.max_speed - (
                    (abs_steer / self.high_speed_steer) ** 1.6
                ) * (self.max_speed - self.fast_corner_speed)
                target_speed = max(target_speed, straightish_speed)

        steer_releasing = abs_steer < self.prev_abs_steer
        accel_limit = self.accel_limit * (1.45 if steer_releasing else 1.0)
        max_step_up = accel_limit * dt
        max_step_down = self.brake_limit * dt
        speed = self.clamp(
            target_speed,
            self.prev_speed - max_step_down,
            self.prev_speed + max_step_up,
        )

        return self.clamp(speed, 0.0, self.max_speed)

    def start_recovery(self, left_open, right_open):
        """Begin a reverse maneuver toward the more open side."""
        open_side = 1.0 if left_open > right_open else -1.0
        self.recovery_turn = open_side
        self.recovery_mode = 'reverse'
        self.recovery_until = self.now_sec() + self.reverse_duration

    def run_recovery(
        self,
        front_min,
        left_min,
        right_min,
        rear_min,
        left_open,
        right_open,
    ):
        """Run reverse/escape commands when the car is boxed in."""
        now = self.now_sec()
        side_open = max(left_open, right_open)
        is_stuck = (
            front_min < self.stuck_front_dist
            and left_min < self.stuck_side_dist
            and right_min < self.stuck_side_dist
        )

        if self.recovery_mode == 'normal' and is_stuck:
            self.start_recovery(left_open, right_open)

        if self.recovery_mode == 'normal':
            return False

        if self.recovery_mode == 'reverse':
            if rear_min < self.rear_stop_dist:
                self.publish_drive(0.0, 0.0)
                print(
                    f'RECOVERY blocked rear | Rear: {rear_min:.2f}m',
                    flush=True
                )
                return True

            if left_open > right_open:
                self.recovery_turn = 1.0
            else:
                self.recovery_turn = -1.0

            found_exit = (
                side_open > self.side_open_dist
                and front_min > self.recovery_front_clear
            )
            if found_exit or now >= self.recovery_until:
                self.recovery_mode = 'escape'
                self.recovery_until = now + self.escape_duration
            else:
                steering = 0.0
                if rear_min < self.rear_slow_dist:
                    speed = self.reverse_speed * 0.45
                else:
                    speed = self.reverse_speed
                self.publish_drive(steering, speed)
                print(
                    f'RECOVERY reverse | Fmin: {front_min:.2f}m | '
                    f'Rear: {rear_min:.2f}m | '
                    f'Lmin: {left_min:.2f}m | Rmin: {right_min:.2f}m | '
                    f'Lopen: {left_open:.2f}m | '
                    f'Ropen: {right_open:.2f}m | '
                    f'steer: {math.degrees(steering):.1f}deg | '
                    f'speed: {speed:.1f}m/s',
                    flush=True
                )
                return True

        if self.recovery_mode == 'escape':
            if now >= self.recovery_until:
                if front_min > self.front_stop_dist:
                    self.recovery_mode = 'normal'
                    return False

                self.start_recovery(left_open, right_open)
                steering = 0.0
                self.publish_drive(steering, self.reverse_speed)
                return True

            steering = self.recovery_turn * self.max_steer
            self.publish_drive(steering, self.escape_speed)
            print(
                f'RECOVERY escape | Fmin: {front_min:.2f}m | '
                f'Lmin: {left_min:.2f}m | Rmin: {right_min:.2f}m | '
                f'Lopen: {left_open:.2f}m | '
                f'Ropen: {right_open:.2f}m | '
                f'steer: {math.degrees(steering):.1f}deg | '
                f'speed: {self.escape_speed:.1f}m/s',
                flush=True
            )
            return True

        self.recovery_mode = 'normal'
        return False

    def get_wall_distance(self, scan_msg, side='right'):
        """
        한쪽 벽까지의 수직 거리 계산
        두 빔(a, b)과 사이각도(theta)로 삼각함수 계산:
        alpha = atan2(a*cos(θ) - b, a*sin(θ))
        수직거리 D = b * cos(alpha)
        lookahead: 앞을 미리 내다봐서 코너에서 일찍 반응
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
        future_dist = dist + self.lookahead * math.sin(alpha)
        return future_dist

    def scan_callback(self, scan_msg):
        """Convert each laser scan into steering and speed commands."""
        # 1. 양쪽 벽까지 거리 측정
        right_dist = self.get_wall_distance(scan_msg, side='right')
        left_dist = self.get_wall_distance(scan_msg, side='left')
        front_min = self.get_sector_min(scan_msg, 0, 45)
        front_speed_clear = self.get_sector_min(scan_msg, 0, 18)
        right_min = self.get_sector_min(scan_msg, -90, 70)
        left_min = self.get_sector_min(scan_msg, 90, 70)
        rear_min = self.get_sector_min(scan_msg, 180, 50)
        right_open = self.get_sector_max(scan_msg, -75, 105)
        left_open = self.get_sector_max(scan_msg, 75, 105)

        if self.run_recovery(
            front_min,
            left_min,
            right_min,
            rear_min,
            left_open,
            right_open,
        ):
            return

        # 2. 오차 계산
        # AckermannDrive 기준: +steering은 왼쪽, -steering은 오른쪽.
        # 왼쪽이 더 넓으면 오른쪽 벽이 가까운 상태다.
        center_error = left_dist - right_dist
        if (
            front_speed_clear > self.full_throttle_release_clear
            and abs(center_error) < self.high_speed_error_deadband
            and self.prev_speed > self.fast_corner_speed
        ):
            center_error = 0.0

        # 3. 시간 간격(dt) 계산
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        dt = max(dt, 1e-3)

        # 4. PD 제어
        # P항: 현재 오차에 비례 (많이 벗어났으면 많이 꺾기)
        p_term = self.kp * center_error
        # D항: 오차 변화율에 비례 (급격한 변화 억제)
        derivative = (center_error - self.prev_error) / dt
        derivative = self.clamp(derivative, -5.0, 5.0)
        d_term = self.kd * derivative

        # 벽 안전거리 보정: 가까운 벽 반대쪽으로 밀어낸다.
        safety_error = 0.0
        if right_min < self.min_wall_dist:
            safety_error += self.min_wall_dist - right_min
        if left_min < self.min_wall_dist:
            safety_error -= self.min_wall_dist - left_min

        steering = p_term + d_term + self.safety_kp * safety_error

        # 전방이 막히면 열린 쪽으로 먼저 돌린다.
        if front_min < self.front_slow_dist:
            turn_to_open_side = 1.0 if left_min > right_min else -1.0
            front_ratio = (
                (self.front_slow_dist - front_min)
                / (self.front_slow_dist - self.front_stop_dist)
            )
            steering += (
                turn_to_open_side
                * self.clamp(front_ratio, 0.0, 1.0)
                * 0.34
            )

        # 조향각 제한 (-0.42 ~ 0.42 라디안 = 약 ±24도)
        steering = self.clamp(steering, -self.max_steer, self.max_steer)

        # 5. 속도 결정 (조향각/전방 여유 기반 레이스용 프로파일)
        abs_steer = abs(steering)
        nearest_wall = min(left_min, right_min)
        speed = self.get_race_speed(
            front_speed_clear,
            nearest_wall,
            abs_steer,
            dt,
        )

        # 6. 드라이브 명령 발행
        self.publish_drive(steering, speed)

        # 7. 디버깅 로그
        now_sec = self.now_sec()
        if now_sec - self.last_debug_time >= self.debug_period:
            print(
                f'R: {right_dist:.2f}m | L: {left_dist:.2f}m | '
                f'Rmin: {right_min:.2f}m | Lmin: {left_min:.2f}m | '
                f'Fmin: {front_min:.2f}m | '
                f'Fspd: {front_speed_clear:.2f}m | '
                f'Rear: {rear_min:.2f}m | '
                f'center_error: {center_error:.2f} | '
                f'safety_error: {safety_error:.2f} | '
                f'steer: {math.degrees(steering):.1f}deg | '
                f'speed: {speed:.1f}m/s',
                flush=True
            )
            self.last_debug_time = now_sec

        self.prev_error = center_error
        self.prev_time = now
        self.prev_speed = speed
        self.prev_abs_steer = abs_steer

    def stop(self):
        """노드 종료 시 차 정지"""
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        self.publisher.publish(stop_msg)
        print('차 정지!')


def main(args=None):
    """Run the wall-following node."""
    rclpy.init(args=args)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()       # 종료 시 정지 명령
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
