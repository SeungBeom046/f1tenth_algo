"""[4 판단] 후행차(ego) 상태 기계와 종방향 속도 명령.

파이프라인: 인지(corridor) → 추적(tracker) → belief → **판단(이 모듈)** → 횡·종 제어(lateral, 이 모듈의 v_cmd) → AEB → 출력

상태 전이 (전이 조건은 docs/PIPELINE.md에 표로 정리):
    WAIT ─(v_E > t0_speed_threshold: t₀, λ = 0)→ APPROACH ─(d ≤ d_ref + probe_tol)→ PROBE
    APPROACH·PROBE ─(go)→ PASS ─(선행차 앞면이 ego 뒤범퍼보다 m 뒤)→ MERGE ─(|y + δ| < merge_tol)→ DONE
    APPROACH·PROBE ─(belief NY 확정 | PROBE 후 T_max)→ FOLLOW
    PASS ─(겹치기 전 선행차 복귀 | 횡 여유 부족)→ FOLLOW (pass_abort), abort_mode = retreat면 → RETREAT
    RETREAT ─(선행차 뒷면이 앞범퍼보다 retreat_clear_gap 이상 앞)→ FOLLOW

방법 (종방향 명령과 go 조건):
    a 고정 규칙:   d_follow_a 유지, PROBE 후 T_wait_a가 지나고 횡 여유가 있으면 go. belief는 기록만.
    b 수동 belief: d_follow_b(≥ d_y_max) 유지, belief Y 확정 + 횡 여유면 go.
    c 공격적 탐색: v_pass로 d_probe_c까지 접근, 판단은 b와 같음.
    d 제안:        v = min(v_pass, v_lim(d, v_B)) (기획안 식 8)로 d*(식 9)까지 접근, 판단은 b와 같음.
"""
from collections import namedtuple

from my_algo.aur import safety
from my_algo.aur.belief import Belief
from my_algo.aur.speed import min_speed_rule

# 한 scan의 관측 (tracker·corridor 출력).
# valid: 이번 scan에서 선행차가 보였는가. W: 통로 폭 추정 (p.W가 있으면 무시).
# front_rel: 선행차 앞면 x − ego 뒤범퍼 x (PASS/MERGE에서만, 모르면 None).
# y_l: 선행차 왼쪽 모서리 (tracker.Track.y_l, 모르면 None). lateral_ref = left_edge일 때만 쓴다.
Obs = namedtuple('Obs', 't v_e valid d y_b v_b ego_lat W front_rel y_l')
Obs.__new__.__defaults__ = (None, None)

INF = float('inf')


class EgoFSM:
    def __init__(self, p, method):
        if method not in ('a', 'b', 'c', 'd'):
            raise ValueError('method는 a|b|c|d: %r' % (method,))
        self.p = p
        if p.stationary_pass and (method != 'a' or not p.experiment_safety):
            raise ValueError('stationary_pass는 method a와 experiment_safety가 필요하다')
        self.method = method
        self.belief = Belief(p)
        self.state = 'WAIT'
        self.t0 = None
        self.t_probe = None
        self.decided = None         # 'Y' | 'NY': b가 η 또는 1−η를 처음 넘은 쪽 (순차 검정은 첫 통과에서 끝난다)
        self.t_decided = None
        self.follow_reason = None   # FOLLOW로 간 이유: belief_NY | T_max | pass_abort
        self.hold_count = 0
        self.abort_count = 0
        # 기록용 (/exp/state)
        self.h = INF                # 식 7 안전 여유
        self.d_safe = None          # 식 6
        self.v_lim = None           # 식 8
        # 내부 상태
        self._held = False
        self._moving = False        # 최소 속도 hold 허용 여부 (히스테리시스)
        self._last = None           # 마지막 유효 관측
        self._prev = None           # belief용 직전 (t, y_b)
        self._cleared = False       # PASS에서 선행차와 횡으로 완전히 어긋났는가
        self._overlapped = False    # PASS에서 앞뒤로 겹쳤는가 (d ≤ 0). 그 뒤에는 중단하지 않는다
        self._y_b_most_yielded = None   # PASS 중 선행차가 가장 많이 비킨 횡위치 (복귀 판정 기준)
        self._yb_hist = []              # PASS 중 최근 y_B (abort_yb_window)

    # ------------------------------------------------------------ 방법별 종방향
    def _d_ref(self, v_b):
        """방법별 목표 추종 거리. d는 식 9의 d*."""
        p = self.p
        return {'a': p.d_follow_a, 'b': p.d_follow_b, 'c': p.d_probe_c}.get(
            self.method, safety.d_star(v_b, p))

    def _follow_speed(self, trk):
        """방법별 종방향 명령. 선행차가 안 보이면 v_B_nominal."""
        p = self.p
        if trk is None:
            if p.experiment_safety and self._last is not None:
                return 0.0
            return p.v_B_nominal
        if self.method == 'd':
            return min(p.v_pass, self.v_lim)
        if self.method == 'c' and trk.d > p.d_probe_c + p.probe_tol:
            return p.v_pass
        return min(p.v_pass, max(0.0, trk.v_b + p.k_d * (trk.d - self._d_ref(trk.v_b))))

    # ------------------------------------------------------------ scan마다
    def step(self, obs):
        """한 scan의 판단. 반환: (v_cmd [m/s], y_ref [m], state, reason)."""
        p = self.p
        W = p.W if p.W is not None else obs.W
        delta = safety.lane_offset(W, p) if W is not None else 0.0
        trk = self._update_track_and_safety(obs)
        self._start_and_update_belief(obs)

        reason = self.method
        y_ref = 0.0
        v = self._follow_speed(trk)
        if self.state in ('APPROACH', 'PROBE') or (p.stationary_pass and self.state == 'WAIT'):
            reason = self._decide(obs, trk, W, reason)
        if self.state == 'PASS' and not self._overlapped and obs.valid:
            if self._check_pass_abort(obs, W):
                reason = 'pass_abort'
                v = self._follow_speed(trk)
        if self.state == 'RETREAT':
            y_ref, v, reason = self._retreat(obs, trk, delta, v, reason)
        if self.state == 'PASS':
            y_ref = delta
            v, reason = self._pass_speed(obs, delta, v, reason)
            merge_gap = p.m_stop if p.merge_gap is None else p.merge_gap
            if obs.front_rel is not None and obs.front_rel <= -merge_gap:
                self.state = 'MERGE'
        if self.state == 'MERGE':
            # 선행차는 뒤에 있으므로 앞에 차가 없을 때의 속도(v_B_nominal). merge_v를 주면 그 속도로 간격을 벌리며 들어간다
            v_merge = self._follow_speed(None) if p.merge_v is None else p.merge_v
            y_ref, v, reason = -delta, v_merge, 'merging'
            if abs(obs.ego_lat + delta) < p.merge_tol:
                self.state = 'DONE'
        if self.state == 'DONE':
            y_ref, v, reason = -delta, p.v_B_nominal, 'done'

        v = self._apply_min_speed(obs, v)
        if p.experiment_safety and self._last is not None and not obs.valid \
                and (self.state in ('WAIT', 'APPROACH', 'PROBE', 'FOLLOW', 'RETREAT')
                     or (self.state == 'PASS' and not self._cleared and not self._overlapped)):
            v, reason = 0.0, 'front_track_lost'
        return v, y_ref, self.state, reason

    def _update_track_and_safety(self, obs):
        """유효 관측을 track_hold 동안 유지하고 식 6·7·8을 계산한다. 반환: 쓸 관측 또는 None."""
        p = self.p
        if obs.valid:
            self._last = obs
        trk = self._last if self._last is not None and obs.t - self._last.t <= p.track_hold else None
        if trk is not None:
            if p.experiment_safety and not obs.valid:
                # 마지막 거리를 그대로 유지하면 주행한 만큼의 접근을 무시하게 된다.
                trk = trk._replace(d=max(0.0, trk.d - max(0.0, obs.v_e - trk.v_b) * (obs.t - trk.t)))
            self.d_safe = safety.d_safe(obs.v_e, trk.v_b, p)
            self.h = trk.d - self.d_safe
            self.v_lim = safety.v_lim(trk.d, trk.v_b, p)
        else:
            self.h, self.d_safe, self.v_lim = INF, None, None
        return trk

    def _start_and_update_belief(self, obs):
        """WAIT에서 t₀를 정하고, APPROACH·PROBE에서 belief를 갱신한다 (식 1~5, z = y_B 변위)."""
        p = self.p
        if p.stationary_pass and self.t0 is None and obs.v_e > p.t0_speed_threshold:
            self.t0 = obs.t
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

    def _decide(self, obs, trk, W, reason):
        """APPROACH·PROBE: PROBE 진입, belief 확정, PASS(go) 또는 FOLLOW(give up). 반환: reason."""
        p = self.p
        if self.state == 'APPROACH' and trk is not None \
                and trk.d <= self._d_ref(trk.v_b) + p.probe_tol:
            self.state, self.t_probe = 'PROBE', obs.t
        b = self.belief.prob_yield()
        if self.decided is None and (b >= p.eta or b <= 1.0 - p.eta):
            self.decided, self.t_decided = ('Y' if b >= p.eta else 'NY'), obs.t
        waited = obs.t - self.t_probe if self.state == 'PROBE' else -1.0
        lat_ok = trk is not None and W is not None and self._lateral_ok(W, trk)
        if self.method == 'a':      # belief는 기록만 하고 판단에 쓰지 않는다
            go, give_up = waited >= p.T_wait_a and lat_ok, False
            if p.stationary_pass:
                # 정지차 모드는 충분히 멀리서 차선 변경을 시작한다. 가까우면 추종·정지만 한다.
                go = obs.valid and lat_ok and obs.d >= 1.5
        else:
            go, give_up = self.decided == 'Y' and lat_ok, self.decided == 'NY'
        if go:
            self.state, reason = 'PASS', 'pass_start'
            self._y_b_most_yielded = trk.y_b
            self._yb_hist = [trk.y_b]
        elif give_up or waited >= p.T_max:
            self.state, reason = 'FOLLOW', ('belief_NY' if give_up else 'T_max')
            self.follow_reason = reason
        return reason

    def _check_pass_abort(self, obs, W):
        """PASS, 앞뒤로 겹치기 전: 선행차가 돌아오거나 횡 여유가 사라지면 추월 중단 → FOLLOW. 반환: 중단했는가."""
        p = self.p
        # 선행차가 아직 비키는 중에 PASS에 들어올 수 있으므로 기준은 진입 값이 아니라 지금까지의 최소 y_B
        # abort_yb_window > 1: 최근 n개 중앙값. 라이다 잡음으로 튄 y_B 하나가 최솟값이 되면 다음 정상값이 '복귀'로 보인다
        self._yb_hist = (self._yb_hist + [obs.y_b])[-p.abort_yb_window:]
        y_b = sorted(self._yb_hist)[len(self._yb_hist) // 2]
        self._y_b_most_yielded = min(self._y_b_most_yielded, y_b)
        if obs.d <= 0.0:
            self._overlapped = True     # 겹친 뒤 감속하면 나란히 있는 시간만 길어진다: 차선 유지, v_pass
            return False
        if y_b > self._y_b_most_yielded + p.y_yield_thr \
                or not self._lateral_ok(W, obs):
            self.state = 'RETREAT' if p.abort_mode == 'retreat' else 'FOLLOW'
            self.follow_reason = 'pass_abort'
            self.abort_count += 1
            return True
        return False

    def _retreat(self, obs, trk, delta, v, reason):
        """추월 중단 뒤 선행차가 옆·바로 앞에 있는 동안: 추월 차선(+δ) 유지, v_B보다 느리게.
        중단 직후 바로 중앙(y_ref 0)으로 가면 옆의 선행차 쪽으로 붙고, 정면 추적으로 바꾸면 옆의 선행차를 벽으로 본다.
        선행차 뒷면이 앞범퍼보다 retreat_clear_gap 이상 앞서면 FOLLOW (y_ref 0, 추종 속도). 반환: (y_ref, v, reason)."""
        p = self.p
        if trk is not None and trk.d >= p.retreat_clear_gap:
            self.state = 'FOLLOW'
            return 0.0, self._follow_speed(trk), 'retreat_done'
        v_b = trk.v_b if trk is not None else p.v_B_nominal
        return delta, p.retreat_speed_ratio * v_b, 'retreat'

    def _lateral_ok(self, W, o):
        """선행차 왼쪽에 ego 폭 + 양쪽 여유가 남는가. lateral_ref = left_edge이고 y_L이 있으면 왼쪽 모서리 기준."""
        p = self.p
        if p.lateral_ref == 'left_edge' and o.y_l is not None:
            return safety.lateral_pass_ok_edge(W, o.y_l, p.w_car, p.c_lat)
        return safety.lateral_pass_ok(W, o.y_b, p.w_car, p.c_lat)

    def _pass_speed(self, obs, delta, v, reason):
        """PASS 속도: 선행차와 횡으로 완전히 어긋나고 추월 차선(+δ)에 도착하기 전에는 추종 속도, 그 뒤 v_pass.
        차선 변경 중 빠르면 틀어진 차체 앞에 벽이 AEB 한계 안으로 들어온다. 이미 옆에 겹쳤으면 바로 v_pass."""
        p = self.p
        if p.stationary_pass and not self._cleared:
            v = min(p.v_B_nominal, self.v_lim) if self.v_lim is not None else 0.0
        if self._overlapped:
            self._cleared = True
        if not self._cleared and self._last is not None:
            # 중심 간격 ≥ w + c 이면 옆으로 나란히 가도 c 이상 떨어진다. 그 전에는 추종 속도를 지킨다.
            self._cleared = obs.ego_lat - self._last.y_b >= p.w_car + p.c_lat
        in_lane = abs(obs.ego_lat - delta) < p.lane_change_tol
        if p.stationary_pass and self._cleared:
            return p.v_B_nominal, 'stationary_passing'
        if self._cleared and (in_lane or self._overlapped):
            return p.v_pass, 'passing'
        return v, reason

    def _apply_min_speed(self, obs, v):
        """최소 속도 규칙 (0 < v < v_min 처리)과 hold 히스테리시스."""
        p = self.p
        if obs.v_e > p.hold_on_speed:
            self._moving = True
        elif obs.v_e < p.hold_off_speed:
            self._moving = False
        v, held = min_speed_rule(v, self.h, self._moving, p)
        self.hold_count += held and not self._held     # 분기에 들어간 횟수 (연속 scan은 한 번)
        self._held = held
        return v
