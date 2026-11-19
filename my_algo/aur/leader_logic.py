"""[4 판단] 선행차 행동: Y(양보) / NY(양보 안 함, 급정지 이벤트) / S(처음부터 멈춰 있는 차, 레이스의 사고 차).
NY는 resume_after를 주면 급정지 후 그 시간 뒤 다시 출발한다 (레이스 시나리오 S2). 시뮬용 양보 후 복귀(return_after).

입력은 후방 추적(VehicleTracker view='rear')의 d_rear. 출력은 (속도, 횡 목표).
"""
import random

from my_algo.aur import safety


class Leader:
    def __init__(self, p, intent, run_id, brake_event=None, return_after=None, resume_after=None):
        if intent not in ('Y', 'NY', 'S'):
            raise ValueError('intent는 Y|NY|S: %r' % (intent,))
        self.p = p
        self.intent = intent
        self.brake_event = (intent == 'NY') if brake_event is None else brake_event
        rng = random.Random(str(run_id))    # 문자열 seed는 실행·플랫폼이 달라도 같은 값을 낸다
        self.d_y = rng.uniform(p.d_y_min, p.d_y_max)
        self.brake_delay = rng.uniform(p.brake_delay_min, p.brake_delay_max)
        self.return_after = return_after    # 양보 시작 후 이 시간이 지나면 중앙으로 복귀 [s] (None = 복귀 안 함)
        self.resume_after = resume_after    # 급정지 후 이 시간이 지나면 다시 출발 [s] (None = 계속 정지)
        self.resumed = False
        self.t0 = None
        self.start_too_close = False    # t0에 후행차가 이미 반응 거리 근처: 시나리오 무효, 노드가 경고 로그
        self.yield_triggered = False
        self.t_yield = None
        self.t_armed = None
        self.brake_fired = False
        self.t_brake = None

    def step(self, t, v_e, d_rear, W):
        """t₀ 판정 → 양보 트리거(Y, latch) / 급정지 무장·발동(NY) → (v_cmd, y_ref)."""
        # d_rear: 뒤범퍼 → 후행차 앞면 (안 보이면 None)
        p = self.p
        if self.intent == 'S':
            # 멈춰 있는 차: 트랙 한쪽(오른쪽 차선 중심)에 서 있다. 시뮬은 처음부터 그 자리에 놓는다
            return 0.0, (-safety.lane_offset(W, p) if W is not None else 0.0)
        if self.t0 is None:
            if v_e <= p.t0_speed_threshold:
                return p.v_B_nominal, 0.0
            self.t0 = t
            self.start_too_close = d_rear is not None and d_rear < p.d_y_max + p.start_gap_margin
        if d_rear is not None:
            if self.intent == 'Y' and not self.yield_triggered and d_rear <= self.d_y:
                self.yield_triggered, self.t_yield = True, t
            if self.brake_event and self.t_armed is None and d_rear <= p.d_brake_trigger:
                self.t_armed = t
        if self.t_armed is not None and not self.brake_fired and t >= self.t_armed + self.brake_delay:
            self.brake_fired, self.t_brake = True, t
        yielding = self.yield_triggered and W is not None and (
            self.return_after is None or t < self.t_yield + self.return_after)
        y_ref = -safety.lane_offset(W, p) if yielding else 0.0
        if self.brake_fired and self.resume_after is not None and t >= self.t_brake + self.resume_after:
            self.resumed = True
        return (0.0 if self.brake_fired and not self.resumed else p.v_B_nominal), y_ref
