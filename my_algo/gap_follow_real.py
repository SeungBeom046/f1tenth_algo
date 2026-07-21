"""
Gap-following controller for the real F1TENTH car.

This version is tuned for a cone course and a Livox Mid-360 style 360 deg scan.
The LiDAR is mounted 90 deg clockwise from its datasheet frame, so vehicle front
is +90 deg in the raw LaserScan frame.
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

from my_algo.vesc_utils import (
    ERPM_GAIN,
    apply_min_drive_speed,
    print_event_line,
    print_status_line,
    speed_to_erpm,
)


class GapFollowRealNode(Node):
    """Real-car gap following with cone-course scoring and reverse escape."""

    def __init__(self):
        super().__init__('gap_follow_real_node')

        # LiDAR / geometry
        self.lidar_yaw_offset = math.radians(90.0)
        self.lidar_to_bumper_dist = 0.15
        self.forward_fov_deg = 240.0          # vehicle front +/-120 deg
        self.rear_fov_deg = 70.0
        self.guard_fov_deg = 120.0
        self.corridor_fov_deg = 24.0
        self.center_fov_deg = 10.0
        self.max_considered_range = 6.0
        self.vehicle_half_width = 0.18
        self.bubble_margin = 0.08
        self.bubble_trigger_clearance = 0.70
        self.gap_min_clearance = 0.30

        # Safety / obstacle distance is bumper clearance, not raw LiDAR range.
        self.escape_clearance = 0.11
        self.decel_clearance = 1.05
        self.fast_clearance = 2.00
        self.front_blocked_corridor_clearance = 1.20
        self.front_blocked_front_clearance = 1.00
        self.sharp_blocked_clearance = 1.05

        # Speed. Test runs use 60% of the 15000 ERPM target ceiling.
        self.full_output_erpm = 15000.0
        self.output_limit_ratio = 0.60
        self.open_space_erpm = self.full_output_erpm * self.output_limit_ratio
        self.open_space_speed = self.open_space_erpm / ERPM_GAIN
        self.base_speed = min(2.0, self.open_space_speed)
        self.corner_speed = 0.72
        self.sharp_corner_speed = 0.42
        self.slow_speed = max(0.50, 1850.0 / ERPM_GAIN)
        self.reverse_speed = -0.50
        self.speed_ramp_rate = 2.8

        # Steering. Use the servo's practical limit in corners/U-turns.
        self.max_steer = 0.78
        self.cruise_max_steer = 0.22
        self.angle_gain = 1.35
        self.corner_angle_gain = 1.95
        self.steering_deadband = 0.012
        self.steering_filter_alpha = 0.62
        self.corner_filter_alpha = 0.94
        self.turn_commit_angle = math.radians(32.0)
        self.sharp_turn_angle = math.radians(50.0)
        self.corner_min_steer_ratio = 0.82
        self.sharp_min_steer_ratio = 1.00

        # Gap scoring. Large gaps matter, but a far target inside the gap wins.
        self.gap_width_weight = 3.0
        self.gap_clearance_weight = 1.35
        self.gap_turn_bonus_weight = 1.10
        self.target_center_weight = 0.72
        self.target_best_weight = 0.28
        self.blocked_target_center_weight = 0.90
        self.blocked_target_best_weight = 0.10
        self.blocked_turn_sample_weight = 0.60

        # Path quality/speed scoring
        self.path_width_weight = 0.42
        self.path_gap_weight = 0.28
        self.path_front_weight = 0.18
        self.path_corridor_weight = 0.12
        self.corner_path_quality_threshold = 0.78

        # Servo / VESC conversion
        self.SERVO_CENTER = 0.5
        self.SERVO_GAIN = 0.60
        self.SERVO_MIN = 0.03
        self.SERVO_MAX = 0.97

        # Mode/state
        self.joy_active = False
        self.auto_mode = False
        self.prev_steering = 0.0
        self.current_speed_cmd = 0.0
        self.prev_time = self.get_clock().now()
        self.escape_until = self.get_clock().now()
        self.escape_steering = 0.0
        self.escape_active = False
        self.escape_started = False
        self.close_center_count = 0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)
        self.joy_active_sub = self.create_subscription(
            Bool, '/joy_active', self.joy_active_callback, 10)
        self.auto_mode_sub = self.create_subscription(
            Bool, '/autonomous_mode', self.auto_mode_callback, 10)

        self.speed_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)
        self.escape_pub = self.create_publisher(
            Bool, '/gap_escape_active', 10)

        self.escape_timer = self.create_timer(0.02, self.escape_timer_callback)

        self.get_logger().info(
            'Gap Follow Real Node 시작: LiDAR +90deg 보정, 전방 +/-120deg, '
            f'최대조향 {self.max_steer:.2f}rad, 출력제한 {self.output_limit_ratio:.0%}'
        )

    def joy_active_callback(self, msg):
        self.joy_active = msg.data
        if self.joy_active:
            self.publish_escape_active(False)

    def auto_mode_callback(self, msg):
        prev = self.auto_mode
        self.auto_mode = msg.data
        if not self.auto_mode:
            self.publish_escape_active(False)
            self.close_center_count = 0
        if self.auto_mode != prev:
            mode_str = '자율주행 ON' if self.auto_mode else '자율주행 OFF'
            print_event_line(f'[GapFollow] {mode_str}')

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def lidar_to_vehicle_angle(self, lidar_angle):
        angle = lidar_angle - self.lidar_yaw_offset
        return math.atan2(math.sin(angle), math.cos(angle))

    def sanitize_range(self, scan_msg, r):
        if math.isnan(r):
            return None
        if math.isinf(r):
            return min(scan_msg.range_max, self.max_considered_range)
        if r <= 0.0:
            return 0.0
        return min(r, scan_msg.range_max, self.max_considered_range)

    def get_bumper_clearance(self, raw_range):
        return max(0.0, raw_range - self.lidar_to_bumper_dist)

    def build_forward_samples(self, scan_msg):
        samples = []
        half_fov = math.radians(self.forward_fov_deg * 0.5)
        angle = scan_msg.angle_min

        for i, raw in enumerate(scan_msg.ranges):
            vehicle_angle = self.lidar_to_vehicle_angle(angle)
            if abs(vehicle_angle) <= half_fov:
                dist = self.sanitize_range(scan_msg, raw)
                if dist is not None:
                    clearance = self.get_bumper_clearance(dist)
                    samples.append({
                        'idx': i,
                        'angle': vehicle_angle,
                        'range': dist,
                        'clearance': clearance,
                        'safe': clearance >= self.gap_min_clearance,
                    })
            angle += scan_msg.angle_increment

        samples.sort(key=lambda s: s['angle'])
        return samples

    def sector_clearances(self, samples, angle_limit_rad):
        return [
            sample['clearance']
            for sample in samples
            if abs(sample['angle']) <= angle_limit_rad
        ]

    def sector_min_clearance(self, samples, angle_limit_rad, default=6.0):
        values = self.sector_clearances(samples, angle_limit_rad)
        return min(values) if values else default

    def sector_percentile_clearance(self, samples, angle_limit_rad, pct, default=6.0):
        values = sorted(self.sector_clearances(samples, angle_limit_rad))
        if not values:
            return default
        idx = int(self.clamp((len(values) - 1) * pct, 0, len(values) - 1))
        return values[idx]

    def rear_min_clearance(self, scan_msg):
        rear_limit = math.radians(self.rear_fov_deg * 0.5)
        min_clearance = self.max_considered_range
        found = False
        angle = scan_msg.angle_min

        for raw in scan_msg.ranges:
            vehicle_angle = self.lidar_to_vehicle_angle(angle)
            rear_angle = math.atan2(
                math.sin(vehicle_angle - math.pi),
                math.cos(vehicle_angle - math.pi),
            )
            if abs(rear_angle) <= rear_limit:
                dist = self.sanitize_range(scan_msg, raw)
                if dist is not None:
                    min_clearance = min(min_clearance, self.get_bumper_clearance(dist))
                    found = True
            angle += scan_msg.angle_increment

        return min_clearance if found else self.max_considered_range

    def mask_safety_bubbles(self, samples):
        if not samples:
            return

        obstacles = [
            sample
            for sample in samples
            if sample['clearance'] < self.bubble_trigger_clearance
        ]

        for obstacle in obstacles:
            bubble_radius = self.vehicle_half_width + self.bubble_margin
            angular_width = math.asin(
                self.clamp(bubble_radius / max(obstacle['range'], 0.05), 0.0, 1.0)
            )
            for sample in samples:
                if abs(sample['angle'] - obstacle['angle']) <= angular_width:
                    sample['safe'] = False

    def find_best_gap(self, samples, front_blocked):
        gaps = []
        start = None

        for i, sample in enumerate(samples):
            if sample['safe'] and start is None:
                start = i
            elif (not sample['safe']) and start is not None:
                gaps.append((start, i - 1))
                start = None
        if start is not None:
            gaps.append((start, len(samples) - 1))

        if not gaps:
            return None

        total_width = math.radians(self.forward_fov_deg)
        best_gap = None
        best_score = -1.0

        for start, end in gaps:
            gap_samples = samples[start:end + 1]
            width = max(0.0, gap_samples[-1]['angle'] - gap_samples[0]['angle'])
            avg_clearance = sum(s['clearance'] for s in gap_samples) / len(gap_samples)
            center_angle = 0.5 * (gap_samples[0]['angle'] + gap_samples[-1]['angle'])

            width_score = self.clamp(width / total_width, 0.0, 1.0)
            clearance_score = self.clamp(avg_clearance / self.fast_clearance, 0.0, 1.0)
            center_penalty = 0.25 * abs(center_angle) / math.radians(120.0)
            turn_bonus = 0.0
            if front_blocked:
                turn_bonus = self.clamp(abs(center_angle) / math.radians(90.0), 0.0, 1.0)

            score = (
                self.gap_width_weight * width_score
                + self.gap_clearance_weight * clearance_score
                + self.gap_turn_bonus_weight * turn_bonus
                - center_penalty
            )

            if score > best_score:
                best_score = score
                best_gap = (start, end, score)

        return best_gap

    def choose_target_angle(self, samples, gap, front_blocked):
        start, end, _ = gap
        gap_samples = samples[start:end + 1]
        if not gap_samples:
            return 0.0

        gap_center = 0.5 * (gap_samples[0]['angle'] + gap_samples[-1]['angle'])
        best_sample = max(
            gap_samples,
            key=lambda sample: (
                sample['clearance']
                + (
                    self.blocked_turn_sample_weight
                    * abs(sample['angle'])
                    if front_blocked
                    else 0.0
                )
                - 0.12 * abs(sample['angle'])
            ),
        )

        if front_blocked:
            center_weight = self.blocked_target_center_weight
            best_weight = self.blocked_target_best_weight
        else:
            center_weight = self.target_center_weight
            best_weight = self.target_best_weight

        target = center_weight * gap_center + best_weight * best_sample['angle']

        # In a blocked cone corner, commit to the turning side instead of dithering.
        if front_blocked and abs(target) < self.turn_commit_angle:
            side = 1.0 if target >= 0.0 else -1.0
            if abs(gap_center) > abs(target):
                side = 1.0 if gap_center >= 0.0 else -1.0
            target = side * self.turn_commit_angle

        return self.clamp(target, -math.radians(120.0), math.radians(120.0))

    def score_path_quality(self, gap, front_p20, corridor_p20):
        if gap is None:
            return 0.0

        start, end, _ = gap
        gap_width = max(0, end - start + 1)
        width_score = self.clamp(gap_width / 90.0, 0.0, 1.0)
        gap_score = self.clamp(gap[2] / 3.5, 0.0, 1.0)
        front_score = self.clamp(front_p20 / self.fast_clearance, 0.0, 1.0)
        corridor_score = self.clamp(corridor_p20 / self.fast_clearance, 0.0, 1.0)

        return self.clamp(
            self.path_width_weight * width_score
            + self.path_gap_weight * gap_score
            + self.path_front_weight * front_score
            + self.path_corridor_weight * corridor_score,
            0.0,
            1.0,
        )

    def is_corner(self, target_angle, front_blocked, path_quality):
        return (
            abs(target_angle) >= self.turn_commit_angle
            or front_blocked
            or path_quality < self.corner_path_quality_threshold
        )

    def is_sharp_corner(self, target_angle, corridor_p20):
        return (
            abs(target_angle) >= self.sharp_turn_angle
            or corridor_p20 < self.sharp_blocked_clearance
        )

    def limit_steering_for_context(self, steering, corner_active):
        limit = self.max_steer if corner_active else self.cruise_max_steer
        return self.clamp(steering, -limit, limit)

    def enforce_corner_steering(self, steering, target_angle, corner_active, sharp_corner):
        if not corner_active:
            return steering

        min_ratio = self.sharp_min_steer_ratio if sharp_corner else self.corner_min_steer_ratio
        min_steer = self.max_steer * min_ratio
        if abs(steering) < min_steer:
            sign = 1.0 if target_angle >= 0.0 else -1.0
            steering = sign * min_steer
        return self.clamp(steering, -self.max_steer, self.max_steer)

    def filter_steering_for_context(self, steering, corner_active):
        if abs(steering) < self.steering_deadband:
            steering = 0.0

        alpha = self.corner_filter_alpha if corner_active else self.steering_filter_alpha
        filtered = alpha * steering + (1.0 - alpha) * self.prev_steering
        self.prev_steering = filtered
        return self.clamp(filtered, -self.max_steer, self.max_steer)

    def compute_target_speed(
        self,
        front_min,
        corridor_p20,
        abs_steer,
        corner_active,
        sharp_corner,
        path_quality,
    ):
        clearance = min(front_min, corridor_p20)

        if sharp_corner:
            speed = self.sharp_corner_speed
        elif corner_active:
            speed = self.corner_speed
        else:
            ratio = self.clamp(
                (clearance - self.decel_clearance)
                / max(0.01, self.fast_clearance - self.decel_clearance),
                0.0,
                1.0,
            )
            speed = self.slow_speed + ratio * (self.base_speed - self.slow_speed)

        steer_ratio = self.clamp(abs_steer / self.max_steer, 0.0, 1.0)
        steer_scale = 1.0 - 0.45 * steer_ratio
        quality_scale = 0.65 + 0.35 * path_quality
        speed *= steer_scale * quality_scale

        if speed > 0.0:
            speed = max(self.slow_speed, speed)
        return min(speed, self.open_space_speed)

    def ramp_speed(self, target_speed):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        dt = self.clamp(dt, 0.0, 0.10)
        max_delta = self.speed_ramp_rate * dt

        delta = self.clamp(
            target_speed - self.current_speed_cmd,
            -max_delta,
            max_delta,
        )
        self.current_speed_cmd += delta
        return self.current_speed_cmd

    def get_servo_position(self, steering):
        servo_pos = self.SERVO_CENTER - steering * self.SERVO_GAIN
        return self.clamp(servo_pos, self.SERVO_MIN, self.SERVO_MAX)

    def publish_command(self, speed_ms, steering):
        speed_ms = apply_min_drive_speed(speed_ms)
        erpm = speed_to_erpm(speed_ms)

        speed_msg = Float64()
        speed_msg.data = erpm
        self.speed_pub.publish(speed_msg)

        servo_msg = Float64()
        servo_msg.data = self.get_servo_position(steering)
        self.servo_pub.publish(servo_msg)

        return erpm, servo_msg.data

    def publish_escape_active(self, active):
        if self.escape_active != active:
            self.escape_active = active
        msg = Bool()
        msg.data = active
        self.escape_pub.publish(msg)

    def get_escape_steering(self, samples):
        left_clearance = self.sector_percentile_clearance(
            [s for s in samples if s['angle'] > 0.0],
            math.radians(120.0),
            0.35,
            default=0.0,
        )
        right_clearance = self.sector_percentile_clearance(
            [s for s in samples if s['angle'] < 0.0],
            math.radians(120.0),
            0.35,
            default=0.0,
        )
        if left_clearance >= right_clearance:
            return -0.70 * self.max_steer
        return 0.70 * self.max_steer

    def start_escape(self, samples):
        self.escape_steering = self.get_escape_steering(samples)
        self.escape_until = self.get_clock().now() + Duration(seconds=0.55)
        self.escape_started = True
        self.current_speed_cmd = self.reverse_speed
        self.publish_escape_active(True)
        print_event_line('[GapFollow] 근접 장애물: 후진 탈출 시작')

    def handle_escape(self):
        if not self.escape_started:
            return False

        if self.get_clock().now() >= self.escape_until:
            self.escape_started = False
            self.current_speed_cmd = 0.0
            self.publish_escape_active(False)
            self.publish_command(0.0, 0.0)
            return False

        self.publish_escape_active(True)
        self.publish_command(self.reverse_speed, self.escape_steering)
        return True

    def escape_timer_callback(self):
        if self.escape_started and not self.joy_active and self.auto_mode:
            self.handle_escape()
        elif self.escape_active and (self.joy_active or not self.auto_mode):
            self.publish_escape_active(False)

    def stop(self):
        self.current_speed_cmd = 0.0
        self.prev_steering = 0.0
        self.publish_command(0.0, 0.0)

    def scan_callback(self, scan_msg):
        if self.joy_active or not self.auto_mode:
            return

        if self.handle_escape():
            return

        samples = self.build_forward_samples(scan_msg)
        if not samples:
            self.stop()
            return

        front_min = self.sector_min_clearance(samples, math.radians(self.guard_fov_deg * 0.5))
        central_min = self.sector_min_clearance(samples, math.radians(self.center_fov_deg * 0.5))
        corridor_p20 = self.sector_percentile_clearance(
            samples, math.radians(self.corridor_fov_deg * 0.5), 0.20)
        front_p20 = self.sector_percentile_clearance(
            samples, math.radians(self.guard_fov_deg * 0.5), 0.20)

        front_blocked = (
            corridor_p20 < self.front_blocked_corridor_clearance
            or front_p20 < self.front_blocked_front_clearance
        )

        if central_min <= self.escape_clearance:
            self.close_center_count += 1
        else:
            self.close_center_count = 0

        # Prefer steering out of a cone corner. Reverse only after repeated
        # center-line near contact, so U-turns do not degrade into stop/reverse.
        if self.close_center_count >= 5:
            self.start_escape(samples)
            self.close_center_count = 0
            return

        self.mask_safety_bubbles(samples)
        gap = self.find_best_gap(samples, front_blocked)
        if gap is None:
            self.start_escape(samples)
            return

        target_angle = self.choose_target_angle(samples, gap, front_blocked)
        path_quality = self.score_path_quality(gap, front_p20, corridor_p20)
        corner_active = self.is_corner(target_angle, front_blocked, path_quality)
        sharp_corner = self.is_sharp_corner(target_angle, corridor_p20)

        gain = self.corner_angle_gain if corner_active else self.angle_gain
        steering = self.clamp(target_angle * gain, -self.max_steer, self.max_steer)
        steering = self.limit_steering_for_context(steering, corner_active)
        steering = self.enforce_corner_steering(
            steering, target_angle, corner_active, sharp_corner)
        steering = self.filter_steering_for_context(steering, corner_active)

        target_speed = self.compute_target_speed(
            front_min=front_min,
            corridor_p20=corridor_p20,
            abs_steer=abs(steering),
            corner_active=corner_active,
            sharp_corner=sharp_corner,
            path_quality=path_quality,
        )
        speed = self.ramp_speed(target_speed)
        erpm, servo_pos = self.publish_command(speed, steering)

        print_status_line(
            '[GapFollow] '
            f'speed={speed:5.2f} m/s | '
            f'erpm={erpm:7.0f} | '
            f'steer={steering:6.2f} rad | '
            f'servo={servo_pos:5.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowRealNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.publish_escape_active(False)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
