"""양보 의도 belief, 식 (1)~(5).

부호 규약: y는 왼쪽 양수. 양보는 오른쪽 이동이므로 z(두 관측 사이 선행차 횡변위) < 0.
"""
import math


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
        self.p = p
        self.lam = 0.0

    def reset(self):
        self.lam = 0.0

    def update(self, z, d, dt):
        if dt > 0.0:
            c = self.p.lambda_clip
            self.lam = min(c, max(-c, self.lam + loglik_ratio(z, d, dt, self.p)))
        return self.lam

    def prob_yield(self):
        return 1.0 / (1.0 + math.exp(-self.lam))
