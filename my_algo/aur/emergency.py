"""실험용 AEB. 속도 센서 0 입력에 명령 속도로 하한을 두고, 긴급 정지는 AUTO off까지 유지한다."""
import math

from my_algo.baseline.aeb import AEBSafetyModel
from my_algo.baseline.vehicle_utils import ERPM_GAIN


class ExperimentAEB(AEBSafetyModel):
    CHECK_HZ = 50.0     # autonomous_drive 제어 타이머·시뮬 Car.control 주기 (check_emergency_braking 호출 빈도)

    def __init__(self, latch_release_sec=0.0, **kwargs):
        """latch_release_sec > 0: 정지(측정 속도 < 0.05 m/s) 상태에서 지금 명령으로 다시 검사해도 비상이 아닌 상태가
        이 시간 이어지면 고정을 푼다. 0이면 AUTO off까지 고정 (기존)."""
        super().__init__(**kwargs)
        self.latch_release_sec = latch_release_sec
        self.latched = False
        self.latched_reason = ''
        self._clear_checks = 0

    def reset(self):
        self.latched = False
        self.latched_reason = ''
        self._clear_checks = 0

    def check_emergency_braking(self, scan_msg, speed_mps, requested_command):
        if self.latched and self.latch_release_sec > 0.0:
            # 실차 2026-11-18: 정지 차 0.64 m 뒤에서 고정된 뒤 planner가 서 있어도 영영 출발하지 못했다.
            stopped = (speed_mps or 0.0) < 0.05 and scan_msg is not None
            clear = stopped and not self._check(scan_msg, speed_mps, requested_command)
            self._clear_checks = self._clear_checks + 1 if clear else 0
            if self._clear_checks >= self.latch_release_sec * self.CHECK_HZ:
                self.latched, self._clear_checks = False, 0
        if self.latched:
            self.last_reason = 'latched ' + self.latched_reason
            return True
        emergency = self._check(scan_msg, speed_mps, requested_command)
        if emergency and scan_msg is not None and requested_command is not None and requested_command.is_valid():
            self.latched = True
            self.latched_reason = self.last_reason
        return emergency

    def _check(self, scan_msg, speed_mps, requested_command):
        """고정 없이 이번 scan·명령만으로 판정한다."""
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
