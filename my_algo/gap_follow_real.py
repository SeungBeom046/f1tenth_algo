"""Real car follow-the-gap node for F1TENTH with Livox Mid-360."""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

from my_algo.vesc_utils import (
    ERPM_GAIN,
    MIN_DRIVE_ERPM,
    MIN_DRIVE_SPEED_MS,
    apply_min_drive_speed,
    speed_to_erpm,
)


class GapFollowRealNode(Node):
    """
    실차용 Follow The Gap 노드.

    - LiDAR: Livox Mid-360 (PointCloud2 -> LaserScan 변환 후 수신)
    - 제어: VESC (/commands/motor/speed + /commands/servo/position)
    - 안전: 25cm 이하 전방 장애물은 즉시 정지 후 짧은 후진 탈출 시도
    - 최후 제동: aeb_real 노드가 20cm 이하에서 0속도를 덮어쓴다
    """

    def __init__(self):
        super().__init__('gap_follow_real_node')

        # ============ LiDAR/차량 파라미터 ============
        self.lidar_yaw_offset_deg = 90.0
        self.lidar_to_bumper_dist = 0.10
        self.forward_fov_deg = 190.0
        self.guard_fov_deg = 120.0
        self.corridor_fov_deg = 36.0
        self.max_considered_range = 6.0
        self.vehicle_half_width = 0.18
        self.bubble_margin = 0.18
        self.bubble_trigger_clearance = 0.85
        self.gap_min_clearance = 0.42
        # ============================================

        # ============ 속도/조향 튜닝 ============
        self.emergency_clearance = 0.25
        self.decel_clearance = 0.70
        self.fast_clearance = 2.0
        self.open_space_erpm = 15000.0
        self.open_space_speed = self.open_space_erpm / ERPM_GAIN
        self.base_speed = 2.2
        self.corner_speed = 1.5
        self.slow_speed = max(0.5, 1850.0 / ERPM_GAIN)
        self.reverse_speed = -0.55
        self.speed_ramp_rate = 2.8
        self.max_steer = 0.34
        self.steering_filter_alpha = 0.35
        self.steering_deadband = 0.025
        self.angle_gain = 1.0
        self.center_bias = 0.18
        # =======================================

        # ============ 실차 변환 파라미터 ============
        self.ERPM_GAIN = ERPM_GAIN
        self.SERVO_CENTER = 0.5
        self.SERVO_GAIN = 0.30
        # ==========================================

        self.prev_steering = 0.0
        self.current_speed_cmd = 0.0
        self.prev_time = self.get_clock().now()
        self.escape_until = self.get_clock().now()
        self.escape_steering = 0.0

        self.joy_active = False
        self.auto_mode = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)

        self.speed_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)

        self.joy_active_sub = self.create_subscription(
            Bool, '/joy_active', self.joy_active_callback, 10)
        self.auto_mode_sub = self.create_subscription(
            Bool, '/autonomous_mode', self.auto_mode_callback, 10)

        self.get_logger().info('Gap Follow Real Node 시작!')
        self.get_logger().info(
            f'ERPM_GAIN: {self.ERPM_GAIN} | '
            f'MIN_DRIVE: {MIN_DRIVE_SPEED_MS:.2f}m/s '
            f'({MIN_DRIVE_ERPM:.0f} ERPM) | '
            f'OPEN: {self.open_space_erpm:.0f} ERPM | '
            f'SERVO_CENTER: {self.SERVO_CENTER} | '
            f'SERVO_GAIN: {self.SERVO_GAIN}'
        )

    # ============ 모드 콜백 ============

    def joy_active_callback(self, msg):
        self.joy_active = msg.data

    def auto_mode_callback(self, msg):
        prev = self.auto_mode
        self.auto_mode = msg.data
        if self.auto_mode != prev:
            mode_str = '자율주행 ON' if self.auto_mode else '자율주행 OFF'
            print(f'[GapFollow] {mode_str}', flush=True)

    # ============ LiDAR 유틸 ============

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def lidar_to_vehicle_angle(self, lidar_angle):
        angle = lidar_angle - math.radians(self.lidar_yaw_offset_deg)
        return math.atan2(math.sin(angle), math.cos(angle))

    def sanitize_range(self, scan_msg, r):
        if math.isnan(r):
            return None
        if math.isinf(r):
            return scan_msg.range_max
        if r <= 0.0:
            return 0.0
        return min(r, scan_msg.range_max, self.max_considered_range)

    def get_bumper_clearance(self, scan_range):
        return max(0.0, scan_range - self.lidar_to_bumper_dist)

    def build_forward_samples(self, scan_msg):
        half_fov = math.radians(self.forward_fov_deg / 2.0)
        samples = []

        for i, raw_r in enumerate(scan_msg.ranges):
            lidar_angle = scan_msg.angle_min + i * scan_msg.angle_increment
            vehicle_angle = self.lidar_to_vehicle_angle(lidar_angle)
            if abs(vehicle_angle) > half_fov:
                continue

            r = self.sanitize_range(scan_msg, raw_r)
            if r is None:
                continue

            clearance = self.get_bumper_clearance(r)
            samples.append({
                'angle': vehicle_angle,
                'range': r,
                'clearance': clearance,
                'masked': False,
            })

        samples.sort(key=lambda item: item['angle'])
        return samples

    def sector_min_clearance(self, samples, half_width_deg):
        half_width = math.radians(half_width_deg)
        values = [
            s['clearance'] for s in samples
            if abs(s['angle']) <= half_width
        ]
        return min(values) if values else self.max_considered_range

    def sector_percentile_clearance(self, samples, half_width_deg, percentile):
        half_width = math.radians(half_width_deg)
        values = sorted(
            s['clearance'] for s in samples
            if abs(s['angle']) <= half_width
        )
        if not values:
            return self.max_considered_range
        index = int((len(values) - 1) * self.clamp(percentile, 0.0, 1.0))
        return values[index]

    def sector_percentile_at(self, samples, center_deg, width_deg, percentile):
        center = math.radians(center_deg)
        half_width = math.radians(width_deg / 2.0)
        values = sorted(
            s['clearance'] for s in samples
            if abs(s['angle'] - center) <= half_width
        )
        if not values:
            return self.max_considered_range
        index = int((len(values) - 1) * self.clamp(percentile, 0.0, 1.0))
        return values[index]

    def mask_safety_bubbles(self, samples):
        for obs in samples:
            clearance = obs['clearance']
            if clearance <= 0.0 or clearance > self.bubble_trigger_clearance:
                continue

            bubble_radius = self.vehicle_half_width + self.bubble_margin
            angle_width = math.asin(
                self.clamp(bubble_radius / max(obs['range'], 0.05), 0.0, 1.0)
            )
            for sample in samples:
                if abs(sample['angle'] - obs['angle']) <= angle_width:
                    sample['masked'] = True

    def find_best_gap(self, samples):
        best_gap = []
        current_gap = []

        for sample in samples:
            is_clear = (
                not sample['masked']
                and sample['clearance'] >= self.gap_min_clearance
            )
            if is_clear:
                current_gap.append(sample)
            else:
                if len(current_gap) > len(best_gap):
                    best_gap = current_gap
                current_gap = []

        if len(current_gap) > len(best_gap):
            best_gap = current_gap

        if best_gap:
            return best_gap

        return [max(samples, key=lambda s: s['clearance'])] if samples else []

    def choose_target_angle(self, gap):
        best_sample = None
        best_score = -float('inf')

        for sample in gap:
            clearance_score = min(sample['clearance'], self.max_considered_range)
            center_score = -abs(sample['angle']) * self.center_bias
            score = clearance_score + center_score
            if score > best_score:
                best_score = score
                best_sample = sample

        return best_sample['angle'] if best_sample else 0.0

    # ============ 제어 유틸 ============

    def apply_steering_filter(self, steering):
        if abs(steering) < self.steering_deadband:
            steering = 0.0
        filtered = (
            self.prev_steering
            + self.steering_filter_alpha * (steering - self.prev_steering)
        )
        return self.clamp(filtered, -self.max_steer, self.max_steer)

    def ramp_speed(self, target_speed, dt):
        if target_speed <= self.current_speed_cmd:
            self.current_speed_cmd = target_speed
            return self.current_speed_cmd

        max_step = self.speed_ramp_rate * dt
        self.current_speed_cmd = min(
            self.current_speed_cmd + max_step,
            target_speed,
        )
        return self.current_speed_cmd

    def compute_target_speed(self, steering, corridor_clearance, front_clearance):
        abs_steer = abs(steering)

        if corridor_clearance <= self.emergency_clearance:
            return 0.0
        if corridor_clearance < self.decel_clearance:
            ratio = (
                (corridor_clearance - self.emergency_clearance)
                / (self.decel_clearance - self.emergency_clearance)
            )
            return self.slow_speed + self.clamp(ratio, 0.0, 1.0) * (
                self.corner_speed - self.slow_speed
            )
        if abs_steer > 0.24:
            return self.corner_speed
        if abs_steer > 0.12:
            return max(self.corner_speed, self.base_speed)
        if front_clearance >= self.fast_clearance:
            return self.open_space_speed
        return self.base_speed

    def publish_command(self, steering_rad, speed_ms):
        adjusted_speed_ms = apply_min_drive_speed(speed_ms)
        speed_msg = Float64()
        speed_msg.data = speed_to_erpm(adjusted_speed_ms)
        self.speed_pub.publish(speed_msg)

        servo_pos = self.SERVO_CENTER - steering_rad * self.SERVO_GAIN
        servo_pos = self.clamp(servo_pos, 0.0, 1.0)
        servo_msg = Float64()
        servo_msg.data = servo_pos
        self.servo_pub.publish(servo_msg)

    def stop(self):
        self.current_speed_cmd = 0.0
        self.publish_command(0.0, 0.0)
        print('[GapFollow] 차 정지!', flush=True)

    def start_escape(self, samples):
        left_clear = self.sector_percentile_at(samples, 30, 50, 0.35)
        right_clear = self.sector_percentile_at(samples, -30, 50, 0.35)
        self.escape_steering = -0.18 if left_clear >= right_clear else 0.18
        self.escape_until = (
            self.get_clock().now() + Duration(seconds=0.7)
        )
        self.current_speed_cmd = 0.0
        self.publish_command(0.0, 0.0)

    def handle_escape(self):
        if self.get_clock().now() >= self.escape_until:
            return False
        self.publish_command(self.escape_steering, self.reverse_speed)
        return True

    # ============ 메인 콜백 ============

    def scan_callback(self, scan_msg):
        if self.joy_active or not self.auto_mode:
            return

        if self.handle_escape():
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        dt = max(dt, 1e-3)

        samples = self.build_forward_samples(scan_msg)
        if not samples:
            self.stop()
            return

        front_min = self.sector_min_clearance(
            samples, self.guard_fov_deg / 2.0)
        corridor_min = self.sector_min_clearance(
            samples, self.corridor_fov_deg / 2.0)
        corridor_p20 = self.sector_percentile_clearance(
            samples, self.corridor_fov_deg / 2.0, 0.20)
        front_p20 = self.sector_percentile_clearance(
            samples, 45.0, 0.20)

        if corridor_min <= self.emergency_clearance:
            self.start_escape(samples)
            print(
                f'[GapFollow] 전방 250mm 이하 감지 -> 정지 후 후진 | '
                f'Cmin: {corridor_min:.2f}m',
                flush=True
            )
            return

        self.mask_safety_bubbles(samples)
        gap = self.find_best_gap(samples)
        target_angle = self.choose_target_angle(gap)

        steering = self.clamp(
            target_angle * self.angle_gain,
            -self.max_steer,
            self.max_steer,
        )
        steering = self.apply_steering_filter(steering)

        target_speed = self.compute_target_speed(
            steering, corridor_p20, front_p20)
        speed = self.ramp_speed(target_speed, dt)
        self.publish_command(steering, speed)
        adjusted_speed = apply_min_drive_speed(speed)

        print(
            f'Gap: {len(gap)} | '
            f'Fmin: {front_min:.2f}m | Cmin: {corridor_min:.2f}m | '
            f'Cp20: {corridor_p20:.2f}m | Fp20: {front_p20:.2f}m | '
            f'target: {math.degrees(target_angle):.1f}deg | '
            f'steer: {math.degrees(steering):.1f}deg | '
            f'speed: {adjusted_speed:.1f}m/s | '
            f'ERPM: {speed_to_erpm(adjusted_speed):.0f} | '
            f'servo: {self.SERVO_CENTER - steering * self.SERVO_GAIN:.3f}',
            flush=True
        )

        self.prev_steering = steering
        self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowRealNode()
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
