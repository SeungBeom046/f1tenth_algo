"""실차 F1TENTH용 LiDAR 기반 reactive drive.

입력 토픽:
    /scan (sensor_msgs/LaserScan)
    /autonomous_mode (std_msgs/Bool)
    /joy_active (std_msgs/Bool)

출력 토픽:
    /reactive/lidar_command (std_msgs/String JSON ReactiveDriveCommand)

흐름:
    LaserScan -> local 장애물/corridor 분석 -> 주행 모드 선택
    -> 조향/RPM 계획 -> ReactiveDriveCommand 발행.

    주요 튜닝 값:
    vehicle_width:
        실제 차량 폭 [m]. 키우면 좁은 틈새 후보를 더 많이 거른다.
    safety_margin:
        차량 주변 추가 여유거리 [m]. 키우면 더 보수적으로 회피한다.
    target_wall_distance:
        벽 추종 목표 거리 [m]. 키우면 벽에서 더 멀리 주행한다.
    lookahead_distance:
        전방 계획 거리 [m]. 키우면 더 일찍 장애물을 회피한다.
    max_steering:
        조향 제한 [rad]. 키우면 더 급하게 돌고, 줄이면 더 부드러워진다.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from my_algo.vehicle_utils import (
    CRUISE_RPM,
    MAX_RPM,
    MIN_DRIVING_RPM,
    ReactiveDriveCommand,
    clamp,
    command_to_json,
    is_valid_range,
    normalize_angle,
    print_status_line,
    sanitize_rpm,
)


class LidarReactiveDriveNode(Node):
    """LaserScan으로 wall/gap/cone-track/open-space 반응형 주행을 선택한다."""

    def __init__(self):
        super().__init__('lidar_reactive_drive')

        # 차량 폭 [m]
        # 증가: 더 넓은 틈새만 통과, 감소: 좁은 공간도 후보가 됨.
        self.declare_parameter('vehicle_width', 0.30)

        # 장애물 여유거리 [m]
        # 증가: 장애물에서 멀리 회피, 감소: 좁은 통로 통과 가능.
        self.declare_parameter('safety_margin', 0.25)

        # 벽 추종 목표 거리 [m]
        # 증가: 벽에서 멀리, 감소: 벽에 가까이 붙음.
        self.declare_parameter('target_wall_distance', 0.85)

        # 전방 계획 거리 [m]
        # 증가: 더 일찍 회피, 감소: 가까운 장애물에 더 민감.
        self.declare_parameter('lookahead_distance', 2.6)

        # 속도에 따라 추가로 늘릴 전방 계획 거리 [m per m/s].
        self.declare_parameter('lookahead_speed_gain', 0.90)

        # 속도 적응형 lookahead 상한 [m].
        self.declare_parameter('max_lookahead_distance', 4.5)

        # 현재 진행 띠 안 장애물이 이 거리보다 가까우면 회피 모드로 들어간다.
        self.declare_parameter('obstacle_avoidance_distance', 1.20)

        # 이 거리 안에 전방 장애물이 있는데 통과 gap이 없으면 전진하지 않는다.
        self.declare_parameter('blocked_stop_distance', 0.75)

        # 이 거리 안에서는 gap이 보여도 일단 멈춘다. AEB보다 앞단의 planner 정지선이다.
        self.declare_parameter('planner_stop_distance', 0.35)

        # 속도 기반 planner 정지선 여유. 증가하면 AEB 전에 더 일찍 멈춘다.
        self.declare_parameter('planner_stop_time_headway', 0.45)
        self.declare_parameter('planner_stop_margin', 0.20)

        # 작은 좌우 흔들림을 줄이기 위한 조향 안정화 파라미터.
        self.declare_parameter('steering_deadband', 0.055)
        self.declare_parameter('open_space_steering_gain', 0.70)
        self.declare_parameter('obstacle_steering_gain', 1.10)

        # 최대 조향각 [rad]
        # 증가: 더 급하게 회피, 감소: 조향이 부드러워짐.
        self.declare_parameter('max_steering', 0.78)
        
        # 라이다-앞범퍼 수평거리 [m]
        # 증가: 앞범퍼 기준 장애물 여유를 더 보수적으로 계산.
        self.declare_parameter('lidar_to_bumper_dist', 0.30)

        self.vehicle_width = float(self.get_parameter('vehicle_width').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.target_wall_distance = float(
            self.get_parameter('target_wall_distance').value)
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value)
        self.lookahead_speed_gain = float(
            self.get_parameter('lookahead_speed_gain').value)
        self.max_lookahead_distance = float(
            self.get_parameter('max_lookahead_distance').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.lidar_to_bumper_dist = float(
            self.get_parameter('lidar_to_bumper_dist').value)
        self.obstacle_avoidance_distance = float(
            self.get_parameter('obstacle_avoidance_distance').value)
        self.blocked_stop_distance = float(
            self.get_parameter('blocked_stop_distance').value)
        self.planner_stop_distance = float(
            self.get_parameter('planner_stop_distance').value)
        self.planner_stop_time_headway = float(
            self.get_parameter('planner_stop_time_headway').value)
        self.planner_stop_margin = float(
            self.get_parameter('planner_stop_margin').value)
        self.steering_deadband = float(
            self.get_parameter('steering_deadband').value)
        self.open_space_steering_gain = float(
            self.get_parameter('open_space_steering_gain').value)
        self.obstacle_steering_gain = float(
            self.get_parameter('obstacle_steering_gain').value)

        self.auto_mode = False
        self.joy_active = False
        self.current_speed = 0.0
        self.active_lookahead_distance = self.lookahead_distance
        self.current_mode = 'OPEN_SPACE'
        self.mode_started_at = self.get_clock().now()
        self.previous_steering = 0.0
        self.last_status = self.get_clock().now()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)
        self.auto_sub = self.create_subscription(
            Bool, '/autonomous_mode', self.auto_mode_callback, 10)
        self.joy_sub = self.create_subscription(
            Bool, '/joy_active', self.joy_active_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/vesc/odom', self.odom_callback, 10)
        self.command_pub = self.create_publisher(
            String, '/reactive/lidar_command', 10)

        self.get_logger().info('LiDAR reactive drive ready')

    def auto_mode_callback(self, msg):
        """AUTO 모드일 때만 planning을 활성화한다."""
        self.auto_mode = msg.data
        if not self.auto_mode:
            self.publish_command(ReactiveDriveCommand(mode='STOP', rpm=0.0))

    def joy_active_callback(self, msg):
        """수동 조이스틱 조작 중에는 autonomous planner 출력을 멈춘다."""
        self.joy_active = msg.data

    def odom_callback(self, msg):
        """현재 속도로 lookahead/정지선을 속도 적응형으로 조정한다."""
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.current_speed = speed if math.isfinite(speed) else 0.0

    def scan_callback(self, scan_msg):
        """최신 scan을 하나의 local reactive drive command로 변환한다."""
        if self.joy_active or not self.auto_mode:
            return

        points = self.extract_forward_points(scan_msg)
        if not points:
            self.publish_command(ReactiveDriveCommand(mode='STOP', reason='no_scan'))
            return

        self.active_lookahead_distance = self.adaptive_lookahead_distance()
        gaps = self.find_drivable_gaps(points)
        corridor = self.detect_corridor(points)
        cone_track = self.detect_cone_track(points)
        path_half_width = 0.5 * self.vehicle_width + self.safety_margin
        forward_clearance = self.forward_clearance(points, path_half_width)
        obstacle_avoidance_distance = self.adaptive_obstacle_avoidance_distance()
        path_blocked = forward_clearance < obstacle_avoidance_distance

        planner_stop_distance = self.adaptive_planner_stop_distance()
        if forward_clearance <= planner_stop_distance:
            command = ReactiveDriveCommand(
                steering_rad=0.0,
                rpm=0.0,
                mode='STOP',
                confidence=1.0,
                reason=f'planner_stop_{forward_clearance:.2f}m',
            )
            self.publish_command(command)
            self.print_status(command, corridor, gaps)
            return

        desired_mode = self.select_reactive_mode(
            gaps=gaps,
            corridor=corridor,
            cone_track=cone_track,
            path_blocked=path_blocked,
        )

        if desired_mode == 'OBSTACLE_AVOIDANCE':
            command = self.plan_gap_follow(
                points,
                gaps,
                aggressive=True,
                forward_clearance=forward_clearance,
            )
        elif desired_mode == 'CONE_TRACK':
            command = self.plan_cone_track(cone_track)
        elif desired_mode == 'WALL_FOLLOW':
            command = self.plan_wall_follow(corridor)
        else:
            command = self.plan_gap_follow(
                points,
                gaps,
                aggressive=False,
                forward_clearance=forward_clearance,
            )

        command.steering_rad = self.smooth_steering(
            command.steering_rad,
            fast=command.mode in ('OBSTACLE_AVOIDANCE', 'STOP'),
        )
        command.rpm = sanitize_rpm(command.rpm)
        self.publish_command(command)
        self.print_status(command, corridor, gaps)

    def extract_forward_points(self, scan_msg):
        """전방 +/-120도 안의 유효 scan bin을 차량 좌표계 point로 변환한다."""
        points = []
        half_fov = math.radians(120.0)
        angle = scan_msg.angle_min
        for raw_range in scan_msg.ranges:
            vehicle_angle = normalize_angle(angle)
            if abs(vehicle_angle) <= half_fov and is_valid_range(scan_msg, raw_range):
                # 라이다 극좌표(r, theta)를 차량 좌표계로 바꾸는 공식.
                # x = r*cos(theta), y = r*sin(theta)이며,
                # 라이다가 앞범퍼보다 뒤에 있으므로 x에서 라이다-앞범퍼 거리 0.30 m를 뺀다.
                # 따라서 x=0은 라이다 기준이 아니라 앞범퍼 기준 전방 거리다.
                x = raw_range * math.cos(vehicle_angle) - self.lidar_to_bumper_dist
                y = raw_range * math.sin(vehicle_angle)
                if x >= -0.05:
                    points.append({
                        'angle': vehicle_angle,
                        'range': raw_range,
                        'x': x,
                        'y': y,
                        'clearance': max(0.0, x),
                    })
            angle += scan_msg.angle_increment
        points.sort(key=lambda point: point['angle'])
        return points

    def detect_corridor(self, points):
        """좌우 벽이 주행 가능한 corridor를 만드는지 추정한다."""
        left = [p for p in points if math.radians(45) <= p['angle'] <= math.radians(115)]
        right = [p for p in points if -math.radians(115) <= p['angle'] <= -math.radians(45)]
        if len(left) < 8 and len(right) < 8:
            return {'confidence': 0.0}

        left_dist = self.percentile_abs_y(left, 0.35)
        right_dist = self.percentile_abs_y(right, 0.35)
        available = [d for d in (left_dist, right_dist) if d is not None]
        if not available:
            return {'confidence': 0.0}

        # 좌우 벽이 모두 보이면 전체 통로 폭 = 왼쪽 거리 + 오른쪽 거리.
        # 한쪽 벽만 보이면 차량이 중앙 근처에 있다고 가정해 보이는 거리의 2배로 추정한다.
        width = sum(available) if len(available) == 2 else max(available) * 2.0
        # 신뢰도는 "통로 폭이 차량 폭보다 얼마나 여유 있는가"를 0~1로 정규화한다.
        # (width - vehicle_width) / 2.0 이므로 폭 여유가 약 2 m 이상이면 1에 가까워진다.
        confidence = clamp((width - self.vehicle_width) / 2.0, 0.0, 1.0)
        if len(left) >= 8 or len(right) >= 8:
            confidence = max(confidence, 0.55)
        return {
            'confidence': confidence,
            'left_dist': left_dist,
            'right_dist': right_dist,
            'width': width,
        }

    def percentile_abs_y(self, points, percentile):
        """측면 sector point에서 노이즈에 강한 측면 거리를 반환한다."""
        if not points:
            return None
        values = sorted(abs(point['y']) for point in points if point['x'] > -0.05)
        if not values:
            return None
        index = int(clamp((len(values) - 1) * percentile, 0, len(values) - 1))
        return values[index]

    def find_drivable_gaps(self, points):
        """
        차량이 통과할 수 있을 만큼 넓은 각도 구간을 찾는다.

        gap은 이후 폭, 전방 여유거리, 조향 부담, 정지하지 않고 계속 움직일 수
        있는 정도를 함께 고려해 점수화한다.
        """
        # 통과 필요 폭 = 차량 실제 폭 + 좌우 안전 여유거리.
        # 예: 0.30 + 2*0.18 = 0.66 m보다 좁은 전방 통로는 막힌 것으로 본다.
        required_width = self.vehicle_width + 2.0 * self.safety_margin
        lookahead = self.active_lookahead_distance
        safe_samples = []
        for point in points:
            # 차량 중심선 기준 좌우 허용 폭. 장애물이 lookahead 안에 있고
            # |y|가 이 값보다 작으면 차량 진행 경로를 막는 점으로 판단한다.
            lateral_limit = max(required_width * 0.5, 0.12)
            blocked = (
                0.0 <= point['x'] <= lookahead
                and abs(point['y']) <= lateral_limit
            )
            safe_samples.append(not blocked)

        gaps = []
        start = None
        for index, safe in enumerate(safe_samples):
            if safe and start is None:
                start = index
            elif (not safe) and start is not None:
                gap = self.build_gap(points, start, index - 1)
                if gap:
                    gaps.append(gap)
                start = None
        if start is not None:
            gap = self.build_gap(points, start, len(points) - 1)
            if gap:
                gaps.append(gap)
        return gaps

    def build_gap(self, points, start, end):
        """구간이 실제로 통과 가능하면 gap descriptor를 만든다."""
        if end <= start:
            return None
        left = points[end]
        right = points[start]
        # 틈새 중심각 = 좌우 경계각의 평균. 이 각도가 조향 목표 방향이 된다.
        center_angle = 0.5 * (left['angle'] + right['angle'])
        width = self.estimate_gap_width(right, left)
        if width < self.vehicle_width + self.safety_margin:
            return None
        # 전방 여유거리 = 좌우 경계 range와 lookahead 중 가장 작은 값.
        # 너무 먼 값에 과신하지 않도록 lookahead_distance로 상한을 둔다.
        clearance = min(left['range'], right['range'], self.active_lookahead_distance)
        return {
            'center_angle': center_angle,
            'width': width,
            'clearance': clearance,
            'start_angle': right['angle'],
            'end_angle': left['angle'],
        }

    def estimate_gap_width(self, right_point, left_point):
        """두 각도 경계 point 사이의 실제 opening 폭을 추정한다."""
        dx = left_point['x'] - right_point['x']
        dy = left_point['y'] - right_point['y']
        # 두 경계점 사이의 유클리드 거리 공식: sqrt(dx^2 + dy^2).
        # 각도 차이만 보지 않고 실제 차량 좌표계에서 틈새 폭을 계산한다.
        return math.hypot(dx, dy)

    def detect_cone_track(self, points):
        """local centerline을 만들 수 있는 좌우 cone boundary를 감지한다."""
        near = [
            p for p in points
            if 0.2 <= p['x'] <= self.active_lookahead_distance and abs(p['y']) <= 1.8
        ]
        left = [p for p in near if p['y'] > 0.20]
        right = [p for p in near if p['y'] < -0.20]
        if len(left) < 3 or len(right) < 3:
            return {'confidence': 0.0}

        left_y = self.mean([p['y'] for p in left])
        right_y = self.mean([p['y'] for p in right])
        # 좌우 cone 평균 y 좌표 차이를 트랙 폭으로 본다.
        # 왼쪽 y는 양수, 오른쪽 y는 음수이므로 left_y - right_y가 전체 폭이다.
        track_width = left_y - right_y
        if track_width < self.vehicle_width + 2.0 * self.safety_margin:
            return {'confidence': 0.0}
        # center_y는 좌우 경계의 중앙선 y 위치다.
        center_y = 0.5 * (left_y + right_y)
        # 감지된 좌우 점 개수를 20개 기준으로 0~1 신뢰도로 정규화한다.
        confidence = clamp((len(left) + len(right)) / 20.0, 0.0, 1.0)
        # 목표 조향각 = atan2(중앙선 y 오차, 전방 lookahead 거리).
        # 즉 lookahead 지점의 중앙선을 향하도록 pure-pursuit와 비슷하게 각도를 만든다.
        target_angle = math.atan2(center_y, self.active_lookahead_distance)
        return {
            'confidence': confidence,
            'target_angle': target_angle,
            'width': track_width,
        }

    def select_reactive_mode(self, gaps, corridor, cone_track, path_blocked):
        """mode chattering을 줄이기 위해 hysteresis를 두고 주행 전략을 선택한다."""
        if path_blocked and gaps:
            desired = 'OBSTACLE_AVOIDANCE'
        elif cone_track.get('confidence', 0.0) > 0.55:
            desired = 'CONE_TRACK'
        elif corridor.get('confidence', 0.0) > 0.50 and not path_blocked:
            desired = 'WALL_FOLLOW'
        else:
            desired = 'OPEN_SPACE'

        # ROS 시간 차이는 ns 단위이므로 1e9로 나눠 초 단위로 바꾼다.
        elapsed = (self.get_clock().now() - self.mode_started_at).nanoseconds / 1e9
        if desired == 'OBSTACLE_AVOIDANCE':
            self.current_mode = desired
            self.mode_started_at = self.get_clock().now()
            return self.current_mode
        if desired != self.current_mode and elapsed < 0.45:
            return self.current_mode
        if desired != self.current_mode:
            self.current_mode = desired
            self.mode_started_at = self.get_clock().now()
        return self.current_mode

    def plan_gap_follow(self, points, gaps, aggressive, forward_clearance=None):
        """여유거리, 폭, 조향 부담을 균형 있게 고려해 gap을 선택한다."""
        if not gaps:
            if (
                forward_clearance is not None
                and forward_clearance <= self.blocked_stop_distance
            ):
                return ReactiveDriveCommand(
                    steering_rad=0.0,
                    rpm=0.0,
                    mode='STOP',
                    confidence=0.9,
                    reason=f'blocked_no_gap_{forward_clearance:.2f}m',
                )
            steering = self.turn_toward_open_space(points)
            return ReactiveDriveCommand(
                steering_rad=steering,
                rpm=MIN_DRIVING_RPM,
                mode='OPEN_SPACE',
                confidence=0.3,
                reason='no_gap',
            )

        best_gap = max(gaps, key=self.score_gap)
        # 조향 목표 = 틈새 중심각. 장애물 회피 모드에서는 1.25배로 더 강하게 돌린다.
        # clamp로 실제 허용 조향각 [-max_steering, +max_steering] 안에 제한한다.
        steering_gain = (
            self.obstacle_steering_gain if aggressive else self.open_space_steering_gain
        )
        steering = clamp(
            best_gap['center_angle'] * steering_gain,
            -self.max_steering,
            self.max_steering,
        )
        if abs(steering) < self.steering_deadband:
            steering = 0.0
        # steer_ratio: 현재 조향이 최대 조향 대비 몇 %인지 나타낸다.
        # 많이 꺾을수록 속도를 낮추기 위해 0~1 값으로 정규화한다.
        steer_ratio = clamp(abs(steering) / self.max_steering, 0.0, 1.0)
        # width_ratio: 틈새 폭이 1.5 m 이상이면 넓다고 보고 1로 포화한다.
        width_ratio = clamp(best_gap['width'] / 1.5, 0.0, 1.0)
        # 기본 RPM 공식: CRUISE_RPM에서 시작해 틈새가 넓을수록 MAX_RPM에 가까워진다.
        # CRUISE_RPM/MAX_RPM은 현재 실차 테스트용 70% 스케일 값이다.
        rpm = CRUISE_RPM + (MAX_RPM - CRUISE_RPM) * width_ratio
        # 조향 감속 공식: 최대 조향이면 RPM을 35% 줄이고, 직진이면 줄이지 않는다.
        rpm *= 1.0 - 0.35 * steer_ratio
        if forward_clearance is not None:
            clearance_ratio = clamp(
                (forward_clearance - self.adaptive_planner_stop_distance())
                / max(
                    0.1,
                    self.adaptive_obstacle_avoidance_distance()
                    - self.adaptive_planner_stop_distance(),
                ),
                0.0,
                1.0,
            )
            rpm *= 0.45 + 0.55 * clearance_ratio
        mode = 'OBSTACLE_AVOIDANCE' if aggressive else 'OPEN_SPACE'
        return ReactiveDriveCommand(
            steering_rad=steering,
            rpm=rpm,
            mode=mode,
            confidence=0.75,
            reason='gap_selected',
        )

    def score_gap(self, gap):
        """폭, 여유거리, 조향 부드러움을 사용해 gap 점수를 계산한다."""
        # 폭 점수: 틈새 폭 1.6 m 이상이면 최고점으로 본다.
        width_score = clamp(gap['width'] / 1.6, 0.0, 1.0)
        # 여유거리 점수: lookahead_distance만큼 비어 있으면 최고점.
        clearance_score = clamp(gap['clearance'] / self.active_lookahead_distance, 0.0, 1.0)
        # 조향 부담: 정면에서 멀리 떨어진 틈새일수록 감점한다.
        steering_penalty = abs(gap['center_angle']) / math.radians(120)
        # 조향 변화 부담: 이전 조향과 차이가 클수록 덜그럭거림을 줄이기 위해 감점한다.
        change_penalty = abs(gap['center_angle'] - self.previous_steering)
        center_bonus = 1.0 - clamp(abs(gap['center_angle']) / math.radians(45), 0.0, 1.0)
        # 최종 점수 = 폭/여유거리/중앙 선호 - 조향 부담 - 조향 변화 부담.
        return (
            1.2 * width_score
            + clearance_score
            + 0.55 * center_bonus
            - 0.75 * steering_penalty
            - 0.45 * change_penalty
        )

    def plan_cone_track(self, cone_track):
        """좌우 cone boundary에서 추정한 centerline을 추종한다."""
        steering = clamp(
            cone_track['target_angle'] * 1.45,
            -self.max_steering,
            self.max_steering,
        )
        if abs(steering) < self.steering_deadband:
            steering = 0.0
        # cone 추종도 많이 꺾을수록 속도를 줄인다. 최대 조향이면 25% 감속.
        rpm = CRUISE_RPM * (1.0 - 0.25 * abs(steering) / self.max_steering)
        return ReactiveDriveCommand(
            steering_rad=steering,
            rpm=rpm,
            mode='CONE_TRACK',
            confidence=cone_track['confidence'],
            reason='centerline',
        )

    def plan_wall_follow(self, corridor):
        """측면 거리를 사용해 corridor 형태 환경에서 안정적으로 주행한다."""
        left = corridor.get('left_dist')
        right = corridor.get('right_dist')
        if left is not None and right is not None:
            # 양쪽 벽이 보이면 좌우 거리 차이로 중앙선 오차를 만든다.
            # left > right이면 차량이 오른쪽 벽에 가까우므로 왼쪽으로 조향한다.
            error = left - right
        elif left is not None:
            # 왼쪽 벽만 보이면 목표 벽 거리와의 차이를 오차로 쓴다.
            error = left - self.target_wall_distance
        elif right is not None:
            # 오른쪽 벽만 보이면 부호를 반대로 둬 목표 벽 거리로 떨어지게 한다.
            error = self.target_wall_distance - right
        else:
            error = 0.0
        # 단순 P 제어: 조향각 = 0.55 * 거리 오차.
        # 이후 최대 조향각으로 제한해 급격한 명령을 막는다.
        steering = clamp(0.40 * error, -self.max_steering, self.max_steering)
        if abs(steering) < self.steering_deadband:
            steering = 0.0
        # 벽 추종에서도 조향량이 클수록 속도를 줄인다. 최대 조향이면 20% 감속.
        rpm = CRUISE_RPM * (1.0 - 0.2 * abs(steering) / self.max_steering)
        return ReactiveDriveCommand(
            steering_rad=steering,
            rpm=rpm,
            mode='WALL_FOLLOW',
            confidence=corridor['confidence'],
            reason='corridor',
        )

    def turn_toward_open_space(self, points):
        """fallback: 평균 여유 공간이 더 큰 쪽으로 조향한다."""
        left = self.mean([p['range'] for p in points if p['angle'] > 0.0])
        right = self.mean([p['range'] for p in points if p['angle'] < 0.0])
        direction = 1.0 if left > right else -1.0
        # 열린 공간이 더 큰 쪽으로 최대 조향의 55%만 사용해 과격한 회피를 막는다.
        return direction * self.max_steering * 0.55

    def forward_clearance(self, points, half_width):
        """현재 차량 corridor 안에서 가장 가까운 x방향 여유거리를 반환한다."""
        # 차량 진행 폭 안에 들어오는 점들 중 가장 작은 x가 앞범퍼 기준 최근접 장애물 거리다.
        values = [p['x'] for p in points if p['x'] >= 0.0 and abs(p['y']) <= half_width]
        return min(values) if values else self.active_lookahead_distance

    def adaptive_lookahead_distance(self):
        """속도가 빠를수록 더 먼 장애물을 planning에 반영한다."""
        return clamp(
            self.lookahead_distance + self.current_speed * self.lookahead_speed_gain,
            self.lookahead_distance,
            self.max_lookahead_distance,
        )

    def adaptive_obstacle_avoidance_distance(self):
        """속도가 빠를수록 더 먼 전방 장애물부터 회피 모드로 진입한다."""
        return clamp(
            self.obstacle_avoidance_distance
            + self.current_speed * self.planner_stop_time_headway,
            self.obstacle_avoidance_distance,
            self.active_lookahead_distance,
        )

    def adaptive_planner_stop_distance(self):
        """현재 속도에서 AEB보다 먼저 멈추기 위한 planner 정지선이다."""
        dynamic_stop = (
            self.planner_stop_distance
            + self.current_speed * self.planner_stop_time_headway
            + self.planner_stop_margin
        )
        return clamp(dynamic_stop, self.planner_stop_distance, 1.50)

    def smooth_steering(self, target, fast=False):
        """회피 반응성을 유지하면서 조향 떨림을 제한한다."""
        max_step = 0.32 if fast else 0.16
        # 한 scan callback마다 조향 변화량을 +/-0.16 rad로 제한한다.
        # 급격한 좌우 전환으로 차체가 덜그럭거리는 현상을 줄이는 rate limit이다.
        steering = clamp(
            target,
            self.previous_steering - max_step,
            self.previous_steering + max_step,
        )
        steering = clamp(steering, -self.max_steering, self.max_steering)
        self.previous_steering = steering
        return steering

    def publish_command(self, command):
        """ReactiveDriveCommand 하나를 JSON으로 발행한다."""
        msg = String()
        msg.data = command_to_json(command)
        self.command_pub.publish(msg)

    def print_status(self, command, corridor, gaps):
        """planner 상태를 약 2 Hz로 출력한다."""
        elapsed = (self.get_clock().now() - self.last_status).nanoseconds / 1e9
        if elapsed < 0.5:
            return
        print_status_line(
            '[LiDAR] '
            f'mode={command.mode:18s} | '
            f'rpm={command.rpm:7.0f} | '
            f'steer={command.steering_rad:5.2f} | '
            f'corr={corridor.get("confidence", 0.0):.2f} | '
            f'gaps={len(gaps)}'
        )
        self.last_status = self.get_clock().now()

    def mean(self, values):
        """값이 있으면 평균을, 없으면 0을 반환한다."""
        return sum(values) / len(values) if values else 0.0


def main(args=None):
    """LiDAR reactive drive를 실행한다."""
    rclpy.init(args=args)
    node = LidarReactiveDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_command(ReactiveDriveCommand(mode='STOP', rpm=0.0))
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
