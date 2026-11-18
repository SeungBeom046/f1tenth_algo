"""실험용 AEB. 속도 센서 0 입력에 명령 속도로 하한을 두고, 긴급 정지는 AUTO off까지 유지한다."""
import math

from my_algo.baseline.aeb import AEBSafetyModel
from my_algo.baseline.vehicle_utils import ERPM_GAIN


class ExperimentAEB(AEBSafetyModel):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.latched = False
        self.latched_reason = ''

    def reset(self):
        self.latched = False
        self.latched_reason = ''

    def check_emergency_braking(self, scan_msg, speed_mps, requested_command):
        if self.latched:
            self.last_reason = 'latched ' + self.latched_reason
            return True
        speed = max(0.0, speed_mps or 0.0)
        if requested_command is not None and requested_command.is_valid():
            speed = max(speed, abs(requested_command.rpm) / ERPM_GAIN)
        emergency = super().check_emergency_braking(scan_msg, speed, requested_command)
        if not emergency and scan_msg is not None and self._partial_target(scan_msg):
            # 동일 장착 차량에서 뒤범퍼→라이다는 0.22 m다. 부분 반환을 범퍼로 단정하지 않는다.
            offset = self.lidar_to_bumper_dist
            threshold = self.ttc_threshold
            try:
                self.lidar_to_bumper_dist += 0.22
                # 부분 관측은 정지 물체로 취급하며 반응 지연 0.35 s와 제동거리도 확보한다.
                stop_distance = speed * 0.35 + self.estimate_stopping_distance(speed)
                self.ttc_threshold = max(threshold, stop_distance / max(speed, 1e-3))
                emergency = super().check_emergency_braking(scan_msg, speed, requested_command)
                self.last_reason += ' partial_target_margin=0.22m'
            finally:
                self.lidar_to_bumper_dist = offset
                self.ttc_threshold = threshold
        if emergency and scan_msg is not None and requested_command is not None and requested_command.is_valid():
            self.latched = True
            self.latched_reason = self.last_reason
        return emergency

    def _partial_target(self, scan):
        points = []
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            x, y = distance * math.cos(angle), distance * math.sin(angle)
            if x >= self.lidar_to_bumper_dist and abs(y) <= 0.5:
                points.append((x, y))
        path_points = [(x, y) for x, y in points if abs(y) <= self.vehicle_half_width]
        if not path_points:
            return False
        closest, target_y = min(path_points)
        ys = sorted(y for x, y in points if abs(x - closest) <= 0.10)
        groups = [[]]
        for y in ys:
            if groups[-1] and y - groups[-1][-1] > 0.05:
                groups.append([])
            groups[-1].append(y)
        # 검사 띠로 잘린 차 모서리를 좁은 라이다 반환으로 오인하지 않도록 띠 바깥 점도 연결한다.
        for group in groups:
            if group[0] <= target_y <= group[-1]:
                return len(group) >= 3 and group[-1] - group[0] < 0.12 and abs(sum(group) / len(group)) < 0.10
        return False
