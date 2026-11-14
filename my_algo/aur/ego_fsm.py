"""후행차(ego) 상태 기계: WAIT → APPROACH → PROBE → PASS → MERGE → DONE, 또는 FOLLOW(추월 포기).

방법 a(고정 규칙) / b(수동 belief) / c(공격적 탐색) / d(제안: 식 8 속도 한계로 d*까지 접근).
"""
from collections import namedtuple

from my_algo.aur import safety
from my_algo.aur.belief import Belief
from my_algo.aur.speed import min_speed_rule

# valid: 이번 scan에서 선행차가 보였는가. W: 통로 폭 추정 (p.W가 있으면 무시).
# front_rel: 선행차 앞면 x − ego 뒤범퍼 x (PASS/MERGE에서만, 모르면 None).
Obs = namedtuple('Obs', 't v_e valid d y_b v_b ego_lat W front_rel')
Obs.__new__.__defaults__ = (None,)

INF = float('inf')


class EgoFSM:
    def __init__(self, p, method):
        if method not in ('a', 'b', 'c', 'd'):
            raise ValueError('method는 a|b|c|d: %r' % (method,))
        self.p = p
        self.method = method
        self.belief = Belief(p)
        self.state = 'WAIT'
        self.t0 = None
        self.t_probe = None
        self.decided = None         # 'Y' | 'NY': b가 η 또는 1−η를 처음 넘은 쪽 (순차 검정은 첫 통과에서 끝난다)
        self.t_decided = None
        self.follow_reason = None   # FOLLOW로 간 이유: belief_NY | T_max
        self.hold_count = 0
        self._held = False
        self._moving = False        # hold 허용 여부 (히스테리시스)
        self.h = INF                # 기록용 (/exp/state)
        self.d_safe = None
        self.v_lim = None
        self._last = None           # 마지막 유효 관측
        self._prev = None           # belief용 직전 (t, y_b)
        self._cleared = False       # PASS에서 선행차와 횡으로 완전히 어긋났는가

    def _d_ref(self, v_b):
        p = self.p
        return {'a': p.d_follow_a, 'b': p.d_follow_b, 'c': p.d_probe_c}.get(
            self.method, safety.d_star(v_b, p))

    def _follow_speed(self, trk):
        """방법별 종방향 명령. 선행차가 안 보이면 v_B_nominal."""
        p = self.p
        if trk is None:
            return p.v_B_nominal
        if self.method == 'd':
            return min(p.v_pass, self.v_lim)
        if self.method == 'c' and trk.d > p.d_probe_c + p.probe_tol:
            return p.v_pass
        return min(p.v_pass, max(0.0, trk.v_b + p.k_d * (trk.d - self._d_ref(trk.v_b))))

    def step(self, obs):
        """반환: (v_cmd [m/s], y_ref [m], state, reason)."""
        p = self.p
        W = p.W if p.W is not None else obs.W
        delta = safety.lane_offset(W, p) if W is not None else 0.0
        if obs.valid:
            self._last = obs
        trk = self._last if self._last is not None and obs.t - self._last.t <= p.track_hold else None
        if trk is not None:
            self.d_safe = safety.d_safe(obs.v_e, trk.v_b, p)
            self.h = trk.d - self.d_safe
            self.v_lim = safety.v_lim(trk.d, trk.v_b, p)
        else:
            self.h, self.d_safe, self.v_lim = INF, None, None

        if self.state == 'WAIT':
            self._prev = None
            if obs.v_e > p.t0_speed_threshold:
                # t0: 정지 중 z = 0 관측이 NY 증거로 쌓이지 않도록 여기서 λ = 0.
                self.state, self.t0 = 'APPROACH', obs.t
                self.belief.reset()
        if self.state in ('APPROACH', 'PROBE') and obs.valid:
            if self._prev is not None:
                self.belief.update(obs.y_b - self._prev[1], obs.d, obs.t - self._prev[0])
            self._prev = (obs.t, obs.y_b)   # 추적이 잠깐 끊겨도 유지: 다음 z가 그 사이 변위를 담는다

        reason = self.method
        y_ref = 0.0
        v = self._follow_speed(trk)
        if self.state in ('APPROACH', 'PROBE'):
            if self.state == 'APPROACH' and trk is not None \
                    and trk.d <= self._d_ref(trk.v_b) + p.probe_tol:
                self.state, self.t_probe = 'PROBE', obs.t
            b = self.belief.prob_yield()
            if self.decided is None and (b >= p.eta or b <= 1.0 - p.eta):
                self.decided, self.t_decided = ('Y' if b >= p.eta else 'NY'), obs.t
            waited = obs.t - self.t_probe if self.state == 'PROBE' else -1.0
            lat_ok = trk is not None and W is not None \
                and safety.lateral_pass_ok(W, trk.y_b, p.w_car, p.c_lat)
            if self.method == 'a':      # belief는 기록만 하고 판단에 쓰지 않는다
                go, give_up = waited >= p.T_wait_a and lat_ok, False
            else:
                go, give_up = self.decided == 'Y' and lat_ok, self.decided == 'NY'
            if go:
                self.state, reason = 'PASS', 'pass_start'
            elif give_up or waited >= p.T_max:
                self.state, reason = 'FOLLOW', ('belief_NY' if give_up else 'T_max')
                self.follow_reason = reason

        if self.state == 'PASS':
            y_ref = delta
            if not self._cleared and self._last is not None:
                # 중심 간격 ≥ w + c 이면 옆으로 나란히 가도 c 이상 떨어진다. 그 전에는 추종 속도를 지킨다.
                self._cleared = obs.ego_lat - self._last.y_b >= p.w_car + p.c_lat
            if self._cleared:
                v, reason = p.v_pass, 'passing'
            if obs.front_rel is not None and obs.front_rel <= -p.m_stop:
                self.state = 'MERGE'
        if self.state == 'MERGE':
            y_ref, v, reason = -delta, p.v_pass, 'merging'
            if abs(obs.ego_lat + delta) < p.merge_tol:
                self.state = 'DONE'
        if self.state == 'DONE':
            y_ref, v, reason = -delta, p.v_B_nominal, 'done'

        if obs.v_e > p.hold_on_speed:
            self._moving = True
        elif obs.v_e < p.hold_off_speed:
            self._moving = False
        v, held = min_speed_rule(v, self.h, self._moving, p)
        self.hold_count += held and not self._held     # 분기에 들어간 횟수 (연속 scan은 한 번)
        self._held = held
        return v, y_ref, self.state, reason
