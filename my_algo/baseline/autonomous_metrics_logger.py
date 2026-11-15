"""trial 기반 자율주행 성능 측정 및 CSV 로깅 노드."""

import csv
import math
import os
import subprocess
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String


VALID_RESULTS = {
    'success',
    'stuck',
    'collision',
    'manual_abort',
    'unknown',
}
INTERVENTION_RESULTS = {'stuck', 'collision', 'manual_abort'}


@dataclass
class TrialRecord:
    """완료된 autonomous trial의 metric 기록."""

    timestamp: str
    session_id: str
    trial_id: int
    git_commit: str
    auto_time_s: float
    distance_m: float
    avg_speed_mps: float
    max_speed_mps: float
    mean_steering: float
    mean_abs_steering: float
    max_abs_steering: float
    mean_steering_change: float
    max_steering_change: float
    mean_steering_rate: float
    max_steering_rate: float
    min_obstacle_distance: Optional[float]
    result: str = 'unknown'


class AutonomousMetricsLogger(Node):
    """기존 차량 토픽을 구독해 autonomous trial metric을 기록한다."""

    def __init__(self):
        super().__init__('autonomous_metrics_logger')

        self.declare_parameter('auto_mode_topic', '/autonomous_mode')
        self.declare_parameter('odom_topic', '/vesc/odom')
        self.declare_parameter('steering_topic', '/commands/servo/position')
        self.declare_parameter('motor_speed_topic', '/commands/motor/speed')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('logs_dir', 'logs')
        self.declare_parameter('servo_center', 0.5)
        self.declare_parameter('servo_gain', 0.60)
        self.declare_parameter('front_fov_deg', 120.0)
        self.declare_parameter('lidar_to_bumper_dist', 0.30)
        self.declare_parameter('max_valid_speed_mps', 4.0)

        self.logs_dir = self.get_parameter('logs_dir').value
        if not os.path.isabs(self.logs_dir):
            self.logs_dir = os.path.join(os.getcwd(), self.logs_dir)
        self.trial_csv_path = os.path.join(self.logs_dir, 'autonomous_trials.csv')
        self.session_csv_path = os.path.join(self.logs_dir, 'autonomous_sessions.csv')

        self.servo_center = float(self.get_parameter('servo_center').value)
        self.servo_gain = float(self.get_parameter('servo_gain').value)
        self.front_fov = math.radians(float(self.get_parameter('front_fov_deg').value))
        self.lidar_to_bumper_dist = float(
            self.get_parameter('lidar_to_bumper_dist').value)
        self.max_valid_speed_mps = float(
            self.get_parameter('max_valid_speed_mps').value)

        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.git_commit = self.get_git_commit()
        self.in_auto = False
        self.trial_id = 0
        self.current_trial = None
        self.completed_trials = []
        self.session_written = False
        self.last_status_time = self.get_clock().now()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.auto_sub = self.create_subscription(
            Bool,
            self.get_parameter('auto_mode_topic').value,
            self.auto_mode_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        self.steering_sub = self.create_subscription(
            Float64,
            self.get_parameter('steering_topic').value,
            self.steering_callback,
            10,
        )
        self.motor_speed_sub = self.create_subscription(
            Float64,
            self.get_parameter('motor_speed_topic').value,
            self.motor_speed_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            qos,
        )
        self.result_sub = self.create_subscription(
            String,
            '/metrics/set_trial_result',
            self.set_result_callback,
            10,
        )
        self.finish_sub = self.create_subscription(
            Bool,
            '/metrics/finish_session',
            self.finish_session_callback,
            10,
        )
        self.status_timer = self.create_timer(1.0, self.status_timer_callback)

        self.get_logger().info(
            '[METRICS] Logger started | '
            f'session={self.session_id} | git={self.git_commit}'
        )
        self.get_logger().info(
            '[METRICS] Result topic: '
            'ros2 topic pub --once /metrics/set_trial_result std_msgs/msg/String '
            '"data: success"'
        )

    def get_git_commit(self):
        """짧은 git commit hash를 반환하고, 읽을 수 없으면 unknown을 반환한다."""
        candidates = [
            os.getcwd(),
            os.path.abspath(os.path.join(os.path.dirname(__file__), '..')),
            os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')),
        ]
        for path in candidates:
            try:
                result = subprocess.run(
                    ['git', '-C', path, 'rev-parse', '--short', 'HEAD'],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=0.5,
                )
                commit = result.stdout.strip()
                if commit:
                    return commit
            except Exception:
                continue
        return 'unknown'

    def now_sec(self):
        """현재 ROS 시간을 초 단위로 반환한다."""
        return self.get_clock().now().nanoseconds / 1e9

    def timestamp(self):
        """CSV row에 넣을 wall-clock timestamp를 반환한다."""
        return datetime.now().isoformat(timespec='seconds')

    def auto_mode_callback(self, msg):
        """MANUAL/AUTO 전환을 처리한다."""
        if msg.data and not self.in_auto:
            self.start_trial()
        elif (not msg.data) and self.in_auto:
            self.finish_trial()

    def start_trial(self):
        """새 autonomous trial을 시작한다."""
        self.trial_id += 1
        self.in_auto = True
        self.current_trial = {
            'start_time': self.now_sec(),
            'distance': 0.0,
            'max_speed': 0.0,
            'speed_time_sum': 0.0,
            'last_speed_time': None,
            'last_speed': None,
            'prev_pose': None,
            'steering_sum': 0.0,
            'abs_steering_sum': 0.0,
            'steering_count': 0,
            'max_abs_steering': 0.0,
            'prev_steering': None,
            'prev_steering_time': None,
            'steering_change_sum': 0.0,
            'steering_change_count': 0,
            'max_steering_change': 0.0,
            'steering_rate_sum': 0.0,
            'steering_rate_count': 0,
            'max_steering_rate': 0.0,
            'min_obstacle_distance': None,
        }
        self.last_status_time = self.get_clock().now()
        print('[METRICS] MANUAL -> AUTO', flush=True)
        print(f'[METRICS] Trial {self.trial_id} started', flush=True)

    def finish_trial(self):
        """현재 autonomous trial을 종료하고 trial row를 저장한다."""
        if not self.current_trial:
            self.in_auto = False
            return

        auto_time = max(0.0, self.now_sec() - self.current_trial['start_time'])
        self.integrate_speed_sample(self.now_sec())
        distance = self.current_trial['distance']
        # 평균 속도 공식: 주행거리 / AUTO 지속시간.
        avg_speed = distance / auto_time if auto_time > 0.0 else 0.0
        steering_count = self.current_trial['steering_count']
        change_count = self.current_trial['steering_change_count']
        rate_count = self.current_trial['steering_rate_count']

        record = TrialRecord(
            timestamp=self.timestamp(),
            session_id=self.session_id,
            trial_id=self.trial_id,
            git_commit=self.git_commit,
            auto_time_s=auto_time,
            distance_m=distance,
            avg_speed_mps=avg_speed,
            max_speed_mps=self.current_trial['max_speed'],
            mean_steering=(
                self.current_trial['steering_sum'] / steering_count
                if steering_count else 0.0
            ),
            mean_abs_steering=(
                self.current_trial['abs_steering_sum'] / steering_count
                if steering_count else 0.0
            ),
            max_abs_steering=self.current_trial['max_abs_steering'],
            mean_steering_change=(
                self.current_trial['steering_change_sum'] / change_count
                if change_count else 0.0
            ),
            max_steering_change=self.current_trial['max_steering_change'],
            mean_steering_rate=(
                self.current_trial['steering_rate_sum'] / rate_count
                if rate_count else 0.0
            ),
            max_steering_rate=self.current_trial['max_steering_rate'],
            min_obstacle_distance=self.current_trial['min_obstacle_distance'],
        )

        self.completed_trials.append(record)
        self.append_trial_csv(record)
        self.print_trial_summary(record)
        self.in_auto = False
        self.current_trial = None

    def odom_callback(self, msg):
        """AUTO 상태에서 pose 거리와 실제 속도를 기록한다."""
        if not self.in_auto or not self.current_trial:
            return

        pose = msg.pose.pose.position
        current_pose = (pose.x, pose.y)
        prev_pose = self.current_trial['prev_pose']
        if prev_pose is not None:
            dx = current_pose[0] - prev_pose[0]
            dy = current_pose[1] - prev_pose[1]
            # pose 기반 이동거리 공식: step = sqrt(dx^2 + dy^2).
            # 각 odom 위치 변화량을 누적해 trial 총 주행거리를 만든다.
            step = math.hypot(dx, dy)
            if math.isfinite(step):
                self.current_trial['distance'] += step
        self.current_trial['prev_pose'] = current_pose

        # 평면 속도 크기 공식: speed = sqrt(vx^2 + vy^2).
        speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )
        if self.is_valid_speed(speed):
            self.integrate_speed_sample(self.now_sec())
            self.current_trial['last_speed'] = speed
            self.current_trial['last_speed_time'] = self.now_sec()
            self.current_trial['max_speed'] = max(
                self.current_trial['max_speed'],
                speed,
            )

    def is_valid_speed(self, speed):
        """유한하지 않거나 말이 안 되는 odometry 속도 sample을 거른다."""
        return (
            math.isfinite(speed)
            and speed >= 0.0
            and speed <= self.max_valid_speed_mps
        )

    def integrate_speed_sample(self, now):
        """시간 가중 속도 적분값을 유지한다."""
        if not self.current_trial:
            return
        last_time = self.current_trial['last_speed_time']
        last_speed = self.current_trial['last_speed']
        if last_time is None or last_speed is None:
            return
        dt = max(0.0, now - last_time)
        if dt > 0.5:
            dt = 0.5
        # 속도 시간 적분: 거리 근사값 += 마지막 속도 * 시간 간격.
        # odom pose 거리와 별개로 평균 속도 검증용 누적값을 유지한다.
        self.current_trial['speed_time_sum'] += last_speed * dt

    def steering_callback(self, msg):
        """AUTO 상태에서 steering command 통계를 기록한다."""
        if not self.in_auto or not self.current_trial:
            return
        steering = self.servo_to_steering(msg.data)
        if steering is None:
            return

        now = self.now_sec()
        trial = self.current_trial
        trial['steering_sum'] += steering
        trial['abs_steering_sum'] += abs(steering)
        trial['steering_count'] += 1
        trial['max_abs_steering'] = max(
            trial['max_abs_steering'],
            abs(steering),
        )

        prev = trial['prev_steering']
        prev_time = trial['prev_steering_time']
        if prev is not None:
            # 조향 변화량 공식: |현재 조향각 - 이전 조향각|.
            change = abs(steering - prev)
            trial['steering_change_sum'] += change
            trial['steering_change_count'] += 1
            trial['max_steering_change'] = max(
                trial['max_steering_change'],
                change,
            )
            if prev_time is not None:
                dt = now - prev_time
                if dt > 1e-3:
                    # 조향 변화율 공식: 조향 변화량 / 걸린 시간.
                    rate = change / dt
                    trial['steering_rate_sum'] += rate
                    trial['steering_rate_count'] += 1
                    trial['max_steering_rate'] = max(
                        trial['max_steering_rate'],
                        rate,
                    )

        trial['prev_steering'] = steering
        trial['prev_steering_time'] = now

    def servo_to_steering(self, servo_pos):
        """servo command position을 steering radian으로 되돌려 계산한다."""
        if not math.isfinite(servo_pos) or abs(self.servo_gain) < 1e-6:
            return None
        # steering_to_servo의 역변환.
        # servo = center - steering*gain 이므로 steering = (center - servo) / gain.
        return (self.servo_center - servo_pos) / self.servo_gain

    def motor_speed_callback(self, _msg):
        """motor command 토픽 인터페이스 확인용으로 구독만 유지한다."""
        return

    def scan_callback(self, msg):
        """전방 ROI 안의 최소 장애물 거리를 기록한다."""
        if not self.in_auto or not self.current_trial:
            return

        half_fov = self.front_fov * 0.5
        angle = msg.angle_min
        min_clearance = None
        for raw in msg.ranges:
            vehicle_angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(vehicle_angle) <= half_fov and self.is_valid_range(msg, raw):
                # 전방 장애물 거리도 AEB와 같은 앞범퍼 기준 x 공식으로 기록한다.
                x = raw * math.cos(vehicle_angle) - self.lidar_to_bumper_dist
                if x >= 0.0:
                    clearance = max(0.0, x)
                    min_clearance = (
                        clearance if min_clearance is None
                        else min(min_clearance, clearance)
                    )
            angle += msg.angle_increment

        if min_clearance is None:
            return
        current = self.current_trial['min_obstacle_distance']
        self.current_trial['min_obstacle_distance'] = (
            min_clearance if current is None else min(current, min_clearance)
        )

    def is_valid_range(self, msg, value):
        """LaserScan range가 metric 계산에 사용할 수 있는 값인지 확인한다."""
        return (
            math.isfinite(value)
            and value > max(0.0, msg.range_min)
            and value <= msg.range_max
        )

    def set_result_callback(self, msg):
        """가장 최근 또는 지정된 완료 trial의 result를 설정한다."""
        trial_id, result = self.parse_result_message(msg.data)
        if result not in VALID_RESULTS:
            self.get_logger().warn(
                f'[METRICS] Invalid result "{result}". '
                f'Use one of: {", ".join(sorted(VALID_RESULTS))}'
            )
            return
        if not self.completed_trials:
            self.get_logger().warn('[METRICS] No completed trial to update')
            return

        target = None
        if trial_id is None:
            target = self.completed_trials[-1]
        else:
            for record in self.completed_trials:
                if record.trial_id == trial_id:
                    target = record
                    break

        if target is None:
            self.get_logger().warn(f'[METRICS] Trial {trial_id} not found')
            return

        target.result = result
        self.update_trial_csv_result(target)
        self.session_written = False
        print(
            f'[METRICS] Trial {target.trial_id} result set to {result}',
            flush=True,
        )

    def parse_result_message(self, text):
        """result topic payload를 파싱한다. 예: 'success' 또는 '4 success'."""
        parts = text.strip().split()
        if not parts:
            return None, ''
        if len(parts) == 1:
            return None, parts[0]
        try:
            return int(parts[0]), parts[1]
        except ValueError:
            return None, parts[-1]

    def finish_session_callback(self, msg):
        """요청이 들어오면 session CSV row를 저장한다."""
        if msg.data:
            self.write_session_csv()
            self.print_session_summary()

    def status_timer_callback(self):
        """간단한 상태를 1 Hz로 출력한다."""
        if self.in_auto and self.current_trial:
            auto_time = max(0.0, self.now_sec() - self.current_trial['start_time'])
            distance = self.current_trial['distance']
            avg_speed = distance / auto_time if auto_time > 0.0 else 0.0
            print(
                f'[METRICS] Trial {self.trial_id} | AUTO | '
                f'Time: {auto_time:.1f} s | '
                f'Distance: {distance:.1f} m | '
                f'Avg: {avg_speed:.2f} m/s | '
                f'Max: {self.current_trial["max_speed"]:.2f} m/s',
                flush=True,
            )
        else:
            print(
                '[METRICS] MANUAL | Waiting for next autonomous trial',
                flush=True,
            )

    def append_trial_csv(self, record):
        """trial row 하나를 CSV에 추가하고 I/O 오류는 node crash로 이어지지 않게 한다."""
        self.safe_write_csv_row(
            self.trial_csv_path,
            self.trial_fieldnames(),
            self.trial_record_to_row(record),
        )

    def safe_write_csv_row(self, path, fieldnames, row):
        """CSV row를 추가하고 필요하면 header를 생성한다."""
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            needs_header = not os.path.exists(path) or os.path.getsize(path) == 0
            with open(path, 'a', newline='', encoding='utf-8') as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                if needs_header:
                    writer.writeheader()
                writer.writerow(row)
        except Exception as exc:
            self.get_logger().warn(f'[METRICS] CSV write failed: {exc}')

    def update_trial_csv_result(self, record):
        """trial CSV에서 일치하는 trial row의 result를 갱신한다."""
        try:
            if not os.path.exists(self.trial_csv_path):
                return
            with open(self.trial_csv_path, newline='', encoding='utf-8') as csv_file:
                rows = list(csv.DictReader(csv_file))
            for row in rows:
                if (
                    row.get('session_id') == record.session_id
                    and row.get('trial_id') == str(record.trial_id)
                ):
                    row['result'] = record.result
            with open(self.trial_csv_path, 'w', newline='', encoding='utf-8') as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=self.trial_fieldnames())
                writer.writeheader()
                writer.writerows(rows)
        except Exception as exc:
            self.get_logger().warn(f'[METRICS] CSV update failed: {exc}')

    def write_session_csv(self):
        """현재 session summary row를 저장하거나 갱신한다."""
        summary = self.compute_session_summary()
        try:
            os.makedirs(os.path.dirname(self.session_csv_path), exist_ok=True)
            rows = []
            found = False
            if os.path.exists(self.session_csv_path):
                with open(
                    self.session_csv_path,
                    newline='',
                    encoding='utf-8',
                ) as csv_file:
                    rows = list(csv.DictReader(csv_file))
            for index, row in enumerate(rows):
                if row.get('session_id') == self.session_id:
                    rows[index] = summary
                    found = True
                    break
            if not found:
                rows.append(summary)
            with open(
                self.session_csv_path,
                'w',
                newline='',
                encoding='utf-8',
            ) as csv_file:
                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=self.session_fieldnames(),
                )
                writer.writeheader()
                writer.writerows(rows)
            self.session_written = True
        except Exception as exc:
            self.get_logger().warn(f'[METRICS] Session CSV write failed: {exc}')

    def compute_session_summary(self):
        """session 단위 summary field를 계산한다."""
        counts = {result: 0 for result in VALID_RESULTS}
        for record in self.completed_trials:
            counts[record.result] += 1

        evaluated = (
            counts['success']
            + counts['stuck']
            + counts['collision']
            + counts['manual_abort']
        )
        # 성공률 공식: 성공 trial 수 / 평가 대상 trial 수 * 100.
        success_rate = (
            100.0 * counts['success'] / evaluated
            if evaluated else 0.0
        )
        successes = [
            record for record in self.completed_trials
            if record.result == 'success'
        ]
        all_with_obstacles = [
            record.min_obstacle_distance
            for record in self.completed_trials
            if record.min_obstacle_distance is not None
        ]

        return {
            'timestamp': self.timestamp(),
            'session_id': self.session_id,
            'git_commit': self.git_commit,
            'total_trials': len(self.completed_trials),
            'success': counts['success'],
            'stuck': counts['stuck'],
            'collision': counts['collision'],
            'manual_abort': counts['manual_abort'],
            'unknown': counts['unknown'],
            'success_rate': self.format_float(success_rate),
            'avg_success_time': self.format_float(
                self.mean([record.auto_time_s for record in successes])),
            'avg_success_distance': self.format_float(
                self.mean([record.distance_m for record in successes])),
            'avg_success_speed': self.format_float(
                self.mean([record.avg_speed_mps for record in successes])),
            'avg_success_max_speed': self.format_float(
                self.mean([record.max_speed_mps for record in successes])),
            'avg_min_obstacle_distance': self.format_float(
                self.mean(all_with_obstacles)),
            'manual_interventions': sum(
                counts[result] for result in INTERVENTION_RESULTS),
        }

    def print_trial_summary(self, record):
        """완료된 trial summary를 출력한다."""
        obstacle = (
            f'{record.min_obstacle_distance:.2f} m'
            if record.min_obstacle_distance is not None
            else 'N/A'
        )
        print(
            f'\n========== AUTONOMOUS TRIAL {record.trial_id} ==========\n'
            f'Auto time             : {record.auto_time_s:.2f} s\n'
            f'Distance              : {record.distance_m:.2f} m\n'
            f'Average speed         : {record.avg_speed_mps:.2f} m/s\n'
            f'Maximum speed         : {record.max_speed_mps:.2f} m/s\n'
            f'Mean steering         : {record.mean_steering:.3f}\n'
            f'Mean abs steering     : {record.mean_abs_steering:.3f}\n'
            f'Max abs steering      : {record.max_abs_steering:.3f}\n'
            f'Mean steering change  : {record.mean_steering_change:.3f}\n'
            f'Max steering change   : {record.max_steering_change:.3f}\n'
            f'Mean steering rate    : {record.mean_steering_rate:.2f} /s\n'
            f'Max steering rate     : {record.max_steering_rate:.2f} /s\n'
            f'Minimum obstacle dist : {obstacle}\n'
            f'Result                : {record.result}\n'
            '========================================\n'
            'Set result with: success / stuck / collision / manual_abort / unknown',
            flush=True,
        )

    def print_session_summary(self):
        """현재 session summary를 출력한다."""
        summary = self.compute_session_summary()
        print(
            '\n========== SESSION SUMMARY ==========\n'
            f'Total trials          : {summary["total_trials"]}\n'
            f'Success               : {summary["success"]}\n'
            f'Stuck                 : {summary["stuck"]}\n'
            f'Collision             : {summary["collision"]}\n'
            f'Manual abort          : {summary["manual_abort"]}\n'
            f'Unknown               : {summary["unknown"]}\n'
            f'Success rate          : {summary["success_rate"]} %\n'
            f'Avg success time      : {summary["avg_success_time"]} s\n'
            f'Avg success distance  : {summary["avg_success_distance"]} m\n'
            f'Avg success speed     : {summary["avg_success_speed"]} m/s\n'
            f'Avg success max speed : {summary["avg_success_max_speed"]} m/s\n'
            f'Avg min obstacle dist : {summary["avg_min_obstacle_distance"]} m\n'
            f'Manual interventions  : {summary["manual_interventions"]}\n'
            '=====================================',
            flush=True,
        )

    def trial_record_to_row(self, record):
        """TrialRecord를 CSV row dict로 변환한다."""
        return {
            'timestamp': record.timestamp,
            'session_id': record.session_id,
            'trial_id': record.trial_id,
            'git_commit': record.git_commit,
            'auto_time_s': self.format_float(record.auto_time_s),
            'distance_m': self.format_float(record.distance_m),
            'avg_speed_mps': self.format_float(record.avg_speed_mps),
            'max_speed_mps': self.format_float(record.max_speed_mps),
            'mean_steering': self.format_float(record.mean_steering, 3),
            'mean_abs_steering': self.format_float(record.mean_abs_steering, 3),
            'max_abs_steering': self.format_float(record.max_abs_steering, 3),
            'mean_steering_change': self.format_float(
                record.mean_steering_change, 3),
            'max_steering_change': self.format_float(
                record.max_steering_change, 3),
            'mean_steering_rate': self.format_float(record.mean_steering_rate),
            'max_steering_rate': self.format_float(record.max_steering_rate),
            'min_obstacle_distance': (
                self.format_float(record.min_obstacle_distance)
                if record.min_obstacle_distance is not None else ''
            ),
            'result': record.result,
        }

    def trial_fieldnames(self):
        """trial CSV column 목록을 반환한다."""
        return [
            'timestamp',
            'session_id',
            'trial_id',
            'git_commit',
            'auto_time_s',
            'distance_m',
            'avg_speed_mps',
            'max_speed_mps',
            'mean_steering',
            'mean_abs_steering',
            'max_abs_steering',
            'mean_steering_change',
            'max_steering_change',
            'mean_steering_rate',
            'max_steering_rate',
            'min_obstacle_distance',
            'result',
        ]

    def session_fieldnames(self):
        """session CSV column 목록을 반환한다."""
        return [
            'timestamp',
            'session_id',
            'git_commit',
            'total_trials',
            'success',
            'stuck',
            'collision',
            'manual_abort',
            'unknown',
            'success_rate',
            'avg_success_time',
            'avg_success_distance',
            'avg_success_speed',
            'avg_success_max_speed',
            'avg_min_obstacle_distance',
            'manual_interventions',
        ]

    def mean(self, values):
        """숫자 평균을 반환하고, 빈 list면 0을 반환한다."""
        return sum(values) / len(values) if values else 0.0

    def format_float(self, value, digits=2):
        """CSV용 float 문자열 형식을 통일한다."""
        return f'{value:.{digits}f}'


def main(args=None):
    """autonomous metrics logger node를 실행한다."""
    rclpy.init(args=args)
    node = AutonomousMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.in_auto:
            node.finish_trial()
        node.write_session_csv()
        node.print_session_summary()
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
