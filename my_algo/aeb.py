"""긴급 제동 안전 모델 (AEBSafetyModel). 노드가 아니라 클래스 모듈.

역할: 전방 진행 띠(|y| <= 반폭 + 여유) 안 최근접 장애물 x(앞범퍼 기준)로 긴급 정지 여부를 판단한다.
    x <= max(즉시 정지 거리, v^2/(2 a) + margin, v * ttc_threshold) 이면 정지. 장애물 상대속도는 쓰지 않는다.
파이프라인 위치: [/scan, /vesc/odom 속도] → (AEBSafetyModel) → autonomous_drive(drive) / exp_aeb_node(exp)
구독·발행: 없음 (호출한 노드가 scan과 속도를 넘긴다)
파라미터: 생성자 인자. drive는 lidar_drive.launch.py, exp는 config/common.yaml [EXP-FIXED] (같은 값).
    기준점만 다르다: drive lidar_to_bumper_dist 0.30·반폭 0.15, exp vehicle_geometry.yaml 0.28·0.135.
실험 관련: 모든 방법이 같은 모델·임계값을 쓴다. 원인 점 기록은 my_algo/aeb_event.py (같은 선택 규칙).

입력 데이터:
    최신 /scan LaserScan
    /vesc/odom에서 얻은 현재 차량 속도
    LiDAR/camera planner가 요청한 reactive drive command

주요 튜닝 값:
    ttc_threshold:
        충돌 예상 시간 임계값 [s]. 더 빨리 멈추려면 키우고,
        AEB가 너무 민감하면 줄인다.
    bumper_emergency_distance:
        앞범퍼 기준 즉시 정지 거리 [m]. AEB는 이 거리 안에 실제 충돌 후보가
        들어왔을 때만 마지막 안전망처럼 작동한다.
    bumper_overlap_tolerance:
        라이다-범퍼 보정 후 x가 살짝 음수가 되는 근접점도 충돌 후보로 받아들이는
        허용치 [m]. 실차 장착 오차와 손 테스트를 흡수한다.
    lidar_to_bumper_dist:
        LiDAR x축 원점에서 앞범퍼까지의 수평거리 [m].
"""

import math

from my_algo.vehicle_utils import is_valid_range, normalize_angle


class AEBSafetyModel:
    """TTC와 제동거리 기반 긴급 상황 판정기."""

    def __init__(
        self,
        ttc_threshold=0.55,
        stopping_margin=0.20,
        bumper_emergency_distance=0.25,
        bumper_overlap_tolerance=0.10,
        lidar_to_bumper_dist=0.30,
        vehicle_half_width=0.15,
        path_margin=0.15,
        comfortable_decel=2.8,
    ):
        self.ttc_threshold = ttc_threshold
        self.stopping_margin = stopping_margin
        self.bumper_emergency_distance = bumper_emergency_distance
        self.bumper_overlap_tolerance = bumper_overlap_tolerance
        self.lidar_to_bumper_dist = lidar_to_bumper_dist
        self.vehicle_half_width = vehicle_half_width
        self.path_margin = path_margin
        # 제동거리 추정용 감속도 [m/s^2]. 기본값은 기존 하드코딩 값과 같다.
        self.comfortable_decel = comfortable_decel
        self.last_reason = 'clear'

    def check_emergency_braking(self, scan_msg, speed_mps, requested_command):
        """
        충돌이 임박한 상황에서만 True를 반환한다.

        현재 차량 경로 앞의 좁은 corridor만 확인한다. 멀리 있는 장애물과
        회피 가능한 측면 장애물은 일반 reactive drive에 맡겨서 AEB가 장애물 회피
        알고리즘처럼 동작하지 않게 한다.
        """
        if scan_msg is None:
            self.last_reason = 'missing_scan'
            return True
        if requested_command is None or not requested_command.is_valid():
            self.last_reason = 'invalid_reactive_command'
            return True

        speed = max(0.0, speed_mps)

        # AEB는 차량 전체 주변이 아니라 "지금 차가 지나갈 앞쪽 띠"만 본다.
        # 검사 반폭 = 차량 반폭 + 경로 여유거리.
        # 예: 0.15 + 0.10 = 중심선 좌우 0.25 m 안 장애물만 긴급제동 후보.
        corridor_half_width = self.vehicle_half_width + self.path_margin
        closest_x = None
        angle = scan_msg.angle_min
        for raw_range in scan_msg.ranges:
            vehicle_angle = normalize_angle(angle)
            if is_valid_range(scan_msg, raw_range):
                # 라이다 극좌표(r, theta)를 차량 좌표계로 바꾸는 공식.
                # x = r*cos(theta) - lidar_to_bumper_dist
                # y = r*sin(theta)
                # x는 앞범퍼 기준 전방거리, y는 차량 중심선 기준 좌우거리다.
                x = raw_range * math.cos(vehicle_angle) - self.lidar_to_bumper_dist
                y = raw_range * math.sin(vehicle_angle)
                # 앞범퍼보다 살짝 뒤로 계산되는 근접점도 0 m 충돌 후보로 취급한다.
                # 이렇게 해야 라이다 장착 오차/손 테스트/범퍼 근처 장애물에서 AEB가 빠지지 않는다.
                if x >= -self.bumper_overlap_tolerance and abs(y) <= corridor_half_width:
                    bumper_x = max(0.0, x)
                    closest_x = bumper_x if closest_x is None else min(closest_x, bumper_x)
            angle += scan_msg.angle_increment

        if closest_x is None:
            self.last_reason = 'clear'
            return False

        # 속도가 올라가면 25 cm 고정 정지선만으로는 늦다. 보이는 장애물 회피는
        # planner가 먼저 맡되, 현재 속도에서 충돌 경로가 제동거리/TTC 안으로
        # 들어오면 AEB가 마지막 안전망으로 즉시 개입한다.
        ttc = closest_x / max(speed, 1e-3)
        stopping_distance = self.estimate_stopping_distance(speed)
        dynamic_emergency_distance = max(
            self.bumper_emergency_distance,
            stopping_distance,
            speed * self.ttc_threshold,
        )
        emergency = closest_x <= dynamic_emergency_distance
        self.last_reason = (
            f'ttc={ttc:.2f}s closest={closest_x:.2f}m '
            f'stop={stopping_distance:.2f}m '
            f'aeb_limit={dynamic_emergency_distance:.2f}m'
        )
        return emergency

    def estimate_stopping_distance(self, speed_mps):
        """
        현재 속도에서 보수적인 제동거리를 추정한다.

        계수는 현장 튜닝이 쉽도록 단순하게 유지한다. 현재 바닥에서 실제
        제동거리가 더 길면 stopping_margin을 키운다.
        """
        # 등가속도 정지거리 공식: d = v^2 / (2*a).
        # 여기에 센서 지연, 바닥 미끄러짐, VESC 반응 지연을 위한 margin을 더한다.
        return (speed_mps * speed_mps) / (2.0 * self.comfortable_decel) + self.stopping_margin
