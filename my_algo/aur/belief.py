"""양보 의도 belief, 식 (1)~(5).

부호 규약: y는 왼쪽 양수. 양보는 오른쪽 이동이므로 z(두 관측 사이 선행차 횡변위) < 0.

belief_model:
    original: 기획안 식 그대로. scan마다 z를 독립 증거로 곱한다.
        문제: "아직 양보 안 함"을 scan 수만큼 곱해 Y를 NY로 오판하고, 결과가 scan rate에 따라 달라진다.
    survival: 기획안 변경(docs/PROGRESS.md). 관측을 "누적 횡변위가 임계값을 넘었는가" 하나로 본다.
        안 넘었으면 Λ = 1 − π(d)  (scan별 조건부 생존확률 (1−π(d_k))/(1−π(d_{k−1}))의 곱이 망원급수로 줄어든 값)
        넘었으면  Λ = π(d) / p_fa  이고 흡수 상태 (양보가 끝난 뒤의 z = 0이 NY 증거로 쌓이지 않는다)
"""
import math
from collections import deque


def pi_yield(d, p):
    """식 (1): 거리 d에서 양보 의도(Y) 선행차가 양보 동작 중일 확률."""
    return min(1.0, max(0.0, (p.d_y_max - d) / (p.d_y_max - p.d_y_min)))


def loglik_ratio(z, d, dt, p):
    """ln[p(z|Y,d) / p(z|NY)].

    p(z|NY) = N(z; 0, σ²),  p(z|Y,d) = π·N(z; −v_lat·dt, σ²) + (1−π)·N(z; 0, σ²)
    두 정규분포의 로그 비 ℓ = (2zμ − μ²)/(2σ²)이므로 결과는 ln[π·e^ℓ + (1−π)] (log-sum-exp).
    """
    pi = pi_yield(d, p)
    if pi <= 0.0:
        return 0.0
    mu = -p.v_lat * dt
    a = (2.0 * z * mu - mu * mu) / (2.0 * p.sigma_z ** 2) + math.log(pi)
    if pi >= 1.0:
        return a
    b = math.log1p(-pi)
    m = max(a, b)
    return m + math.log(math.exp(a - m) + math.exp(b - m))


class Belief:
    """로그 오즈 λ = ln[P(Y)/P(NY)]의 순차 갱신. 사전확률 0.5 (λ = 0)."""

    def __init__(self, p):
        if p.belief_model not in ('original', 'survival'):
            raise ValueError('belief_model은 original|survival: %r' % (p.belief_model,))
        self.p = p
        self.reset()

    def reset(self):
        self.lam = 0.0
        self.detected = False       # (survival) 양보 감지, 흡수 상태
        self._t = 0.0
        self._s = 0.0               # 누적 횡변위 = y_B − (reset 시점 y_B)
        self._d_min = float('inf')  # 지금까지의 최소 d
        self._d_lag = float('inf')  # τ 전까지의 최소 d
        self._hist = deque()

    def update(self, z, d, dt):
        if dt <= 0.0:
            return self.lam
        p, c = self.p, self.p.lambda_clip
        if p.belief_model == 'original':
            self.lam = min(c, max(-c, self.lam + loglik_ratio(z, d, dt, p)))
            return self.lam
        self._t += dt
        self._s += z
        self._d_min = min(self._d_min, d)
        # 양보를 시작해도 임계값을 넘기까지 τ = t_react + y_thr / v_lat가 걸린다.
        # 그래서 "지금 변위 없음"은 "τ 전까지 양보를 시작하지 않음"의 증거로만 쓴다.
        self._hist.append((self._t, d))
        tau = p.t_react + p.y_yield_thr / p.v_lat
        while self._hist and self._hist[0][0] <= self._t - tau:
            self._d_lag = min(self._d_lag, self._hist.popleft()[1])
        if self._s <= -p.y_yield_thr and pi_yield(self._d_min, p) > 0.0:
            self.detected = True    # π = 0인 거리에서의 변위는 두 가설 모두 설명하지 못하므로 무시
        if self.detected:
            lam = math.log(pi_yield(self._d_min, p) / p.p_false_yield)
        else:
            s = 1.0 - pi_yield(self._d_lag, p)
            lam = math.log(s) if s > 0.0 else -c
        self.lam = min(c, max(-c, lam))
        return self.lam

    def prob_yield(self):
        return 1.0 / (1.0 + math.exp(-self.lam))
