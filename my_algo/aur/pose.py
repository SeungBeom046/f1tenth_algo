"""[1 인지] 통로 기준 자차 횡위치·헤딩 필터 (지도 없음, 자차 IMU 자이로 + 바퀴 속도).

매 scan 레일 직선으로 다시 잰 헤딩은 라이다 거리 잡음 0.02 m에서 sd 약 2.9°라 조향이 흔들리고,
앞차 점을 그 헤딩으로 돌리므로 2 m 앞 앞차 횡위치도 약 10 cm 흔들린다 (2026-11-19 시뮬).
예측: ψ ← ψ + r·dt (r = 자이로 yaw rate), y ← y + v·sin ψ·dt. 보정: 레일 측정이 새로 있으면 고정 비율로 끌어당긴다.
MPC·EKF 공분산 없이 고정 게인이다. 결과는 Corridor.ego_lateral·heading에 다시 써서 추적·횡 제어·다음 scan 벽 연속성이 같이 쓴다.
자이로가 없거나(yaw_rate None) pose_filter가 꺼져 있으면 아무것도 하지 않는다 (기존 동작).
"""
import math


class PoseFilter:
    def __init__(self, p):
        self.p = p
        self.t = None           # 예측을 마친 시각 (같은 시각에 여러 번 부르면 예측은 한 번)
        self.y = None
        self.psi = None
        self.t_fresh = None     # 마지막 레일 측정 시각
        self._pred = None

    def apply(self, cor, t, v, yaw_rate):
        """cor.update 직후마다 부른다. 같은 t면 예측은 한 번, 보정은 최신 측정으로 다시 한다."""
        p = self.p
        if not p.pose_filter:
            return
        if yaw_rate is None:
            self.y = self.psi = self._pred = None
            self.t = t
            return
        fresh = cor.valid and cor.left is not None and cor.right is not None
        if t != self.t:
            self._pred = None
            if self.y is not None and self.t is not None:
                dt = t - self.t
                psi = self.psi + yaw_rate * dt
                self._pred = (self.y + v * math.sin(0.5 * (self.psi + psi)) * dt, psi)
            self.t = t
        stale = self.t_fresh is None or t - self.t_fresh > p.pose_reset_s
        if self._pred is None or (fresh and stale):
            # 처음이거나 오래 레일을 못 봤으면 측정으로 바로 맞춘다 (자이로 적분 드리프트를 끌고 가지 않는다)
            if fresh:
                self.y, self.psi, self.t_fresh = cor.ego_lateral, cor.heading, t
                self._pred = (self.y, self.psi)
            return
        y, psi = self._pred
        if fresh:
            psi += p.pose_k_heading * (cor.heading - psi)
            y += p.pose_k_lateral * (cor.ego_lateral - y)
            self.t_fresh = t
        self.y, self.psi = y, psi
        cor.ego_lateral, cor.heading = y, psi
