"""차량 운동학 모델 (시뮬). 바퀴·타이어 동역학 없이 뒤축 중심(base_link) 자전거 모델.

명령 경로는 실차와 같다: 최종 속도·조향 → mux_logic.to_vesc(ERPM 변환, 최소 4000 ERPM 규칙, 서보 변환)
→ 차량은 ERPM·서보 값을 받아 m/s·조향각으로 되돌려 쓴다.
종방향 응답: 명령이 t_r 뒤에 도착 → 1차 지연(tau) → 가속 한계 a_max, 감속 한계 a_b.
"""
import math
from collections import deque

from my_algo.vehicle_utils import ERPM_GAIN, SERVO_CENTER, SERVO_GAIN


class Kinematic:
    def __init__(self, x, y, yaw, wheelbase, t_r, tau, a_max, a_b, steer_rate,
                 erpm_gain=ERPM_GAIN, servo_center=SERVO_CENTER, servo_gain=SERVO_GAIN, max_steer=0.78):
        self.x, self.y, self.yaw, self.v, self.steer = x, y, yaw, 0.0, 0.0
        self.L, self.t_r, self.tau, self.a_max, self.a_b = wheelbase, t_r, tau, a_max, a_b
        self.steer_rate, self.max_steer = steer_rate, max_steer
        self.erpm_gain, self.servo_center, self.servo_gain = erpm_gain, servo_center, servo_gain
        self.queue = deque()          # (도착 시각, rpm, servo)
        self.rpm_now, self.servo_now = 0.0, servo_center
        self.travel = 0.0

    def command(self, t, rpm, servo):
        """t에 VESC로 보낸 명령. t + t_r에 적용된다."""
        self.queue.append((t + self.t_r, float(rpm), float(servo)))

    def target(self):
        v = self.rpm_now / self.erpm_gain
        steer = (self.servo_center - self.servo_now) / self.servo_gain
        return v, max(-self.max_steer, min(self.max_steer, steer))

    def step(self, t, dt):
        while self.queue and self.queue[0][0] <= t + 1e-12:
            _, self.rpm_now, self.servo_now = self.queue.popleft()
        v_target, steer_target = self.target()
        dv = (v_target - self.v) * (dt / self.tau if self.tau > dt else 1.0)
        dv = max(-self.a_b * dt, min(self.a_max * dt, dv))
        self.v = max(0.0, self.v + dv)           # 후진 없음
        ds = max(-self.steer_rate * dt, min(self.steer_rate * dt, steer_target - self.steer))
        self.steer += ds
        dist = self.v * dt
        self.x += dist * math.cos(self.yaw)
        self.y += dist * math.sin(self.yaw)
        self.yaw += dist / self.L * math.tan(self.steer)
        self.travel += dist

    def pose(self):
        return self.x, self.y, self.yaw


def transform_polygon(poly, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in poly]
