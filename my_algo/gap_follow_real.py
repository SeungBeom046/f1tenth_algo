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
    - 안전: 주행 노드는 gap을 따라 탈출하고, 최후 제동은 AEB에 맡긴다
    - 최후 제동: aeb_real 노드가 20cm 이하에서 0속도를 덮어쓴다
    """

    def __init__(self):
        super().__init__('gap_follow_real_node')

        # ============ LiDAR/차량 파라미터 ============
        self.lidar_yaw_offset_deg = 90.0
        self.lidar_to_bumper_dist = 0.15
        self.forward_fov_deg = 260.0
        self.rear_fov_deg = 50.0
        self.guard_fov_deg = 120.0
        self.corridor_fov_deg = 24.0
        self.emergency_fov_deg = 12.0
        self.max_considered_range = 6.0
        self.vehicle_half_width = 0.18
        self.bubble_margin = 0.08
        self.bubble_trigger_clearance = 0.65
        self.gap_min_clearance = 0.30
        # ============================================

        # ============ 속도/조향 튜닝 ============
        self.escape_clearance = 0.12
        self.decel_clearance = 1.05
        self.fast_clearance = 2.0
        self.full_output_erpm = 15000.0
        self.output_limit_ratio = 0.60
        self.open_space_erpm = self.full_output_erpm * self.output_limit_ratio
        self.open_space_speed = self.open_space_erpm / ERPM_GAIN
        self.base_speed = min(2.0, self.open_space_speed)
        self.corner_speed = 0.90
        self.sharp_corner_speed = 0.55
        self.slow_speed = max(0.5, 1850.0 / ERPM_GAIN)
        self.reverse_speed = -0.50
        self.speed_ramp_rate = 2.8
        self.max_steer = 0.62
        self.cruise_max_steer = 0.24
        self.steering_filter_alpha = 0.62
        self.steering_deadband = 0.015
        self.angle_gain = 1.35
        self.corner_angle_gain = 1.75
        self.center_bias = 0.04
        self.turn_commit_angle = math.radians(35.0)
        self.sharp_turn_angle = math.radians(50.0)
        self.corner_min_steer_ratio = 0.78
        self.sharp_min_steer_ratio = 0.95
        # =======================================

        # ============ 실차 변환 파라미터 ============
        self.ERPM_GAIN = ERPM_GAIN
        self.SERVO_CENTER = 0.5
        self.SERVO_GAIN = 0.34
        # ==========================================

        self.prev_steering = 0.0
        self.current_speed_cmd = 0.0
        self.prev_time = self.get_clock().now()
        self.escape_until = self.get_clock().now()
        self.escape_steering = 0.0
        self.escape_active = False
        self.escape_started = self.get_clock().now()
        self.corner_active = False
        self.close_center_count = 0

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
        self.escape_pub = self.create_publisher(
            Bool, '/gap_escape_active', 10)
        self.escape_timer = self.create_timer(0.02, self.escape_timer_callback)

        self.joy_active_sub = self.create_subscription(
            Bool, '/joy_active', self.joy_active_callback, 10)
        self.auto_mode_sub = self.create_subscription(
            Bool, '/autonomous_mode', self.auto_mode_callback, 10)

        self.get_logger().info('Gap Follow Real Node 시작!')
        self.get_logger().info(
            f'ERPM_GAIN: {self.ERPM_GAIN} | '
            f'MIN_DRIVE: {MIN_DRIVE_SPEED_MS:.2f}m/s '
            f'({MIN_DRIVE_ERPM:.0f} ERPM) | '
            f'LIMIT: {self.output_limit_ratio * 100:.0f}% | '
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

    def rear_min_clearance(self, scan_msg):
        half_width = math.radians(self.rear_fov_deg / 2.0)
        values = []

        for i, raw_r in enumerate(scan_msg.ranges):
            lidar_angle = scan_msg.angle_min + i * scan_msg.angle_increment
            vehicle_angle = self.lidar_to_vehicle_angle(lidar_angle)
            rear_diff = abs(abs(vehicle_angle) - math.pi)
            if rear_diff > half_width:
                continue
            r = self.sanitize_range(scan_msg, raw_r)
            if r is None:
                continue
            values.append(self.get_bumper_clearance(r))

        return min(values) if values else self.max_considered_range

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

    def find_best_gap(self, samples, front_blocked=False):
        best_gap = []
        best_score = -float('inf')
        current_gap = []

        def score_gap(gap):
            if not gap:
                return -float('inf')
            width = abs(gap[-1]['angle'] - gap[0]['angle'])
            clearances = sorted(s['clearance'] for s in gap)
            p70 = clearances[int((len(clearances) - 1) * 0.70)]
            center = (gap[0]['angle'] + gap[-1]['angle']) / 2.0
            turn_bonus = abs(center) * 0.55 if front_blocked else 0.0
            return (
                width * 2.6
                + min(p70, self.max_considered_range) * 1.2
                - abs(center) * self.center_bias
                + turn_bonus
            )

        for sample in samples:
            is_clear = (
                not sample['masked']
                and sample['clearance'] >= self.gap_min_clearance
            )
            if is_clear:
                current_gap.append(sample)
            else:
                score = score_gap(current_gap)
                if score > best_score:
                    best_gap = current_gap
                    best_score = score
                current_gap = []

        score = score_gap(current_gap)
        if score > best_score:
            best_gap = current_gap

        if best_gap:
            return best_gap

        return [max(samples, key=lambda s: s['clearance'])] if samples else []

    def choose_target_angle(self, gap, front_blocked=False):
        best_sample = None
        best_score = -float('inf')

        if not gap:
            return 0.0

        for sample in gap:
            clearance_score = min(sample['clearance'], self.max_considered_range)
            center_score = -abs(sample['angle']) * self.center_bias
            turn_score = 0.0
            if front_blocked:
                turn_score = abs(sample['angle']) * 0.35
            score = clearance_score + center_score + turn_score
            if score > best_score:
                best_score = score
                best_sample = sample

        gap_center = (gap[0]['angle'] + gap[-1]['angle']) / 2.0
        best_angle = best_sample['angle'] if best_sample else gap_center
        target_angle = 0.65 * gap_center + 0.35 * best_angle
        if front_blocked:
            target_angle = 0.80 * gap_center + 0.20 * best_angle

        if front_blocked and abs(target_angle) < self.turn_commit_angle:
            direction = 1.0 if target_angle >= 0.0 else -1.0
            target_angle = direction * self.turn_commit_angle
        return target_angle

    def is_corner(self, target_angle, front_blocked, corridor_clearance, path_quality):
        if front_blocked:
            return True
        if abs(target_angle) >= self.turn_commit_angle:
            return True
        return corridor_clearance < self.decel_clearance and path_quality < 0.70

    def is_sharp_corner(self, target_angle, front_blocked, corridor_clearance):
        if abs(target_angle) >= self.sharp_turn_angle:
            return True
        return front_blocked and corridor_clearance < 0.95

    def score_path_quality(self, gap, front_clearance, corridor_clearance):
        if not gap:
            return 0.0
        width = abs(gap[-1]['angle'] - gap[0]['angle'])
        gap_clearances = sorted(s['clearance'] for s in gap)
        gap_p60 = gap_clearances[int((len(gap_clearances) - 1) * 0.60)]
        width_score = self.clamp(width / math.radians(70.0), 0.0, 1.0)
        gap_score = self.clamp(gap_p60 / self.fast_clearance, 0.0, 1.0)
        front_score = self.clamp(front_clearance / self.fast_clearance, 0.0, 1.0)
        corridor_score = self.clamp(corridor_clearance / self.decel_clearance, 0.0, 1.0)
        return (
            0.35 * width_score
            + 0.30 * gap_score
            + 0.20 * front_score
            + 0.15 * corridor_score
        )

    # ============ 제어 유틸 ============

    def apply_steering_filter(self, steering):
        if abs(steering) < self.steering_deadband:
            steering = 0.0
        filtered = (
            self.prev_steering
            + self.steering_filter_alpha * (steering - self.prev_steering)
        )
        return self.clamp(filtered, -self.max_steer, self.max_steer)

    def limit_steering_for_context(self, steering, corner_active):
        limit = self.max_steer if corner_active else self.cruise_max_steer
        return self.clamp(steering, -limit, limit)

    def enforce_corner_steering(self, steering, target_angle, corner_active, sharp_corner):
        """코너에서는 최대 조향에 가깝게 밀어 넣는다."""
        if not corner_active:
            return steering

        direction = 1.0 if target_angle >= 0.0 else -1.0
        ratio = self.sharp_min_steer_ratio if sharp_corner else self.corner_min_steer_ratio
        min_abs_steer = self.max_steer * ratio
        if abs(steering) < min_abs_steer:
            steering = direction * min_abs_steer
        return self.clamp(steering, -self.max_steer, self.max_steer)

    def filter_steering_for_context(self, steering, corner_active, sharp_corner):
        """직진은 부드럽게, 코너는 빠르게 조향한다."""
        if sharp_corner:
            return steering
        if corner_active:
            alpha = 0.88
            filtered = self.prev_steering + alpha * (steering - self.prev_steering)
            return self.clamp(filtered, -self.max_steer, self.max_steer)
        return self.apply_steering_filter(steering)

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

    def compute_target_speed(
        self, steering, corridor_clearance, front_clearance, path_quality,
        corner_active=False, sharp_corner=False
    ):
        abs_steer = abs(steering)

        if corridor_clearance < self.decel_clearance:
            ratio = (
                (corridor_clearance - self.escape_clearance)
                / (self.decel_clearance - self.escape_clearance)
            )
            return self.slow_speed + self.clamp(ratio, 0.0, 1.0) * (
                self.corner_speed - self.slow_speed
            )
        if sharp_corner:
            return self.sharp_corner_speed
        if corner_active:
            exit_factor = self.clamp(path_quality, 0.0, 1.0)
            return self.sharp_corner_speed + exit_factor * (
                self.corner_speed - self.sharp_corner_speed
            )

        steer_factor = self.clamp(abs_steer / self.max_steer, 0.0, 1.0)
        clear_factor = self.clamp(front_clearance / self.fast_clearance, 0.0, 1.0)
        quality = self.clamp(path_quality, 0.0, 1.0)
        race_factor = 0.15 + 0.55 * quality + 0.30 * clear_factor
        target = self.slow_speed + race_factor * (
            self.open_space_speed - self.slow_speed
        )
        corner_limit = self.corner_speed + (1.0 - steer_factor) * (
            self.open_space_speed - self.corner_speed
        )
        return min(target, corner_limit, self.open_space_speed)

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

    def publish_escape_active(self, active):
        msg = Bool()
        msg.data = active
        self.escape_pub.publish(msg)

    def escape_timer_callback(self):
        if not self.escape_active:
            return
        if self.joy_active or not self.auto_mode:
            self.escape_active = False
            self.publish_escape_active(False)
            return
        if self.get_clock().now() >= self.escape_until:
            self.escape_active = False
            self.current_speed_cmd = 0.0
            self.publish_escape_active(False)
            self.publish_command(0.0, 0.0)
            return

        self.publish_escape_active(True)
        self.publish_command(self.get_escape_steering(), self.reverse_speed)

    def stop(self):
        self.current_speed_cmd = 0.0
        self.publish_escape_active(False)
        self.publish_command(0.0, 0.0)
        print('[GapFollow] 차 정지!', flush=True)

    def start_escape(self, samples):
        left_clear = self.sector_percentile_at(samples, 30, 50, 0.35)
        right_clear = self.sector_percentile_at(samples, -30, 50, 0.35)
        self.escape_steering = -0.24 if left_clear >= right_clear else 0.24
        self.escape_until = (
            self.get_clock().now() + Duration(seconds=1.2)
        )
        self.escape_started = self.get_clock().now()
        self.current_speed_cmd = 0.0
        self.escape_active = True
        self.publish_escape_active(True)
        self.publish_command(0.0, 0.0)

    def handle_escape(self):
        if not self.escape_active:
            return False
        if self.get_clock().now() >= self.escape_until:
            self.escape_active = False
            self.publish_escape_active(False)
            return False
        self.publish_escape_active(True)
        self.publish_command(self.get_escape_steering(), self.reverse_speed)
        return True

    def get_escape_steering(self):
        elapsed = (self.get_clock().now() - self.escape_started).nanoseconds / 1e9
        wiggle = 0.18 * math.sin(elapsed * 10.0)
        return self.clamp(
            self.escape_steering + wiggle,
            -self.max_steer,
            self.max_steer,
        )

    # ============ 메인 콜백 ============

    def scan_callback(self, scan_msg):
        if self.joy_active or not self.auto_mode:
            self.publish_escape_active(False)
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
        central_min = self.sector_min_clearance(
            samples, self.emergency_fov_deg / 2.0)
        corridor_min = self.sector_min_clearance(
            samples, self.corridor_fov_deg / 2.0)
        corridor_p20 = self.sector_percentile_clearance(
            samples, self.corridor_fov_deg / 2.0, 0.20)
        front_p20 = self.sector_percentile_clearance(
            samples, 45.0, 0.20)
        front_blocked = corridor_p20 < 1.10 or front_p20 < 0.85

        if central_min <= self.escape_clearance:
            self.close_center_count += 1
        else:
            self.close_center_count = 0

        if self.close_center_count >= 4:
            self.start_escape(samples)
            print(
                f'[GapFollow] 중앙 초근접 감지 -> 정지 후 후진 | '
                f'Center: {central_min:.2f}m',
                flush=True
            )
            return

        self.mask_safety_bubbles(samples)
        gap = self.find_best_gap(samples, front_blocked)
        target_angle = self.choose_target_angle(gap, front_blocked)
        path_quality = self.score_path_quality(gap, front_p20, corridor_p20)
        corner_active = self.is_corner(
            target_angle, front_blocked, corridor_p20, path_quality)
        sharp_corner = self.is_sharp_corner(
            target_angle, front_blocked, corridor_p20)

        steer_gain = self.corner_angle_gain if corner_active else self.angle_gain
        steering = self.clamp(
            target_angle * steer_gain,
            -self.max_steer,
            self.max_steer,
        )
        steering = self.limit_steering_for_context(steering, corner_active)
        steering = self.enforce_corner_steering(
            steering, target_angle, corner_active, sharp_corner)
        steering = self.filter_steering_for_context(
            steering, corner_active, sharp_corner)

        target_speed = self.compute_target_speed(
            steering,
            corridor_p20,
            front_p20,
            path_quality,
            corner_active,
            sharp_corner,
        )
        speed = self.ramp_speed(target_speed, dt)
        self.publish_command(steering, speed)
        adjusted_speed = apply_min_drive_speed(speed)

        print(
            f'Gap: {len(gap)} | '
            f'Fmin: {front_min:.2f}m | Center: {central_min:.2f}m | '
            f'Cmin: {corridor_min:.2f}m | '
            f'Cp20: {corridor_p20:.2f}m | Fp20: {front_p20:.2f}m | '
            f'blocked: {int(front_blocked)} | '
            f'corner: {int(corner_active)} | sharp: {int(sharp_corner)} | '
            f'Q: {path_quality:.2f} | '
            f'closeN: {self.close_center_count} | '
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
