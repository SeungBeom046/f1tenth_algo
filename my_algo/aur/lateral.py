"""[5 제어] 횡: 횡 오프셋 벽 따라가기 (지시서 2단계의 순수 부분).

lidar_reactive_drive.plan_wall_follow와 같은 방식: 오차 → 저역 필터 → P 게인 → deadband → rate limit.
오차 = (left − right) + 2·y_ref = 2·(y_ref − ego_lateral). ego가 y_ref에 있으면 0.

지시서와 다른 점: 헤딩 감쇠 항(−heading_gain·ψ)을 더했다. 횡위치 P 제어만으로는 감쇠가 없어
차선 변경 때 넘어가 벽(여유 c = 0.10 m)에 닿는다 (MuJoCo 시뮬에서 확인). heading_gain = 0이면 지시서 방식.
"""


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Lateral:
    def __init__(self, p):
        self.p = p
        self.y_ref = 0.0        # 기울기 제한을 거친 횡 목표
        self.err = None
        self.steer = 0.0
        self.i_term = 0.0       # 적분항 [rad] (트림 보정)

    def step(self, ego_lateral, y_target, dt, heading=0.0):
        """scan마다 한 번 부른다. heading: 통로 기준 차 헤딩 [rad]. 반환: 조향각 [rad] (왼쪽 양수)."""
        p = self.p
        self.y_ref += _clamp(y_target - self.y_ref, -p.y_ref_rate * dt, p.y_ref_rate * dt)
        error = 2.0 * (self.y_ref - ego_lateral)
        a = p.wall_error_filter_alpha
        self.err = error if self.err is None else (1.0 - a) * self.err + a * error
        target = _clamp(p.wall_follow_gain * self.err - p.heading_gain * heading,
                        -p.max_steering, p.max_steering)
        if abs(target) < p.steering_deadband:
            target = 0.0
        if p.lateral_ki > 0.0:
            # 차선 유지 중(y_ref 도착, 오차 작음)에만 적분: 차선 변경의 큰 오차로 쌓이지 않게. deadband는 P항에만 둬서
            # 잡음으로 떨지 않으면서 트림처럼 일정한 치우침은 적분항이 천천히 없앤다
            if abs(y_target - self.y_ref) < 1e-9 and abs(self.y_ref - ego_lateral) < p.lateral_i_zone:
                self.i_term = _clamp(self.i_term + p.lateral_ki * (self.y_ref - ego_lateral) * dt,
                                     -p.lateral_i_max, p.lateral_i_max)
            target = _clamp(target + self.i_term, -p.max_steering, p.max_steering)
        self.steer = _clamp(target, self.steer - p.steer_rate_limit, self.steer + p.steer_rate_limit)
        return self.steer
