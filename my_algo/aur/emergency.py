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
            if x >= self.lidar_to_bumper_dist and abs(y) <= self.vehicle_half_width:
                points.append((x, y))
        if not points:
            return False
        closest = min(x for x, _ in points)
        ys = [y for x, y in points if x <= closest + 0.10]
        return len(ys) >= 3 and max(ys) - min(ys) < 0.12 and abs(sum(ys) / len(ys)) < 0.10
