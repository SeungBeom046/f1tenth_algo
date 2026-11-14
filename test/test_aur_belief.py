from my_algo.aur.belief import Belief, loglik_ratio, pi_yield
from my_algo.aur.params import AurParams

P = AurParams(belief_model='original')
S = AurParams(belief_model='survival')


def approach(p, hz, d_end, v_close=0.6, hold=0.5, y_at=None):
    """1.5 m에서 d_end까지 다가가 hold초 머문다. y_at(d): 선행차 횡위치. 반환: Belief."""
    b, dt, d, y_prev, t_hold = Belief(p), 1.0 / hz, 1.5, 0.0, 0.0
    while t_hold < hold:
        if d > d_end + 1e-9:
            d = max(d_end, d - v_close * dt)
        else:
            t_hold += dt
        y = y_at(d) if y_at else 0.0
        b.update(y - y_prev, d, dt)
        y_prev = y
    return b


def test_survival_is_scan_rate_independent():
    import math
    for d_end in (1.0, 0.9, 0.75):
        b10, b40 = approach(S, 10, d_end), approach(S, 40, d_end)
        assert abs(b10.prob_yield() - b40.prob_yield()) < 1e-9
        # Λ = 1 − π(d): 사전확률 0.5에서 b = (1−π)/(2−π)
        s = 1.0 - pi_yield(d_end, S)
        assert abs(b10.prob_yield() - s / (1.0 + s)) < 1e-9
    # 원래 식은 scan rate에 따라 달라진다 (고친 이유)
    o10, o40 = approach(P, 10, 0.9), approach(P, 40, 0.9)
    assert abs(o10.prob_yield() - o40.prob_yield()) > 0.01


def test_survival_does_not_decide_ny_before_leader_can_react():
    # 원래 식은 d = 0.84 근처에서 이미 b ≤ 0.1. survival은 π ≥ 8/9 (d ≤ 0.667)이어야 한다.
    assert approach(P, 10, 0.80, hold=0.1).prob_yield() < 0.1
    assert approach(S, 10, 0.70).prob_yield() > 0.1
    assert approach(S, 10, 0.65).prob_yield() < 0.1
    assert approach(S, 10, 0.50).lam == -S.lambda_clip


def test_survival_yield_is_absorbing():
    # 0.9 m에서 오른쪽으로 0.265 m 비킨 뒤 가만히 있어도 belief가 내려가지 않는다
    y_at = lambda d: -0.265 if d <= 0.9 else 0.0
    for hz in (10, 40):
        b = approach(S, hz, 0.4, hold=3.0, y_at=y_at)
        assert b.detected and b.prob_yield() > 0.95
    assert approach(P, 10, 0.4, hold=3.0, y_at=y_at).prob_yield() < 0.1   # 원래 식은 다시 NY로 간다
    # 왼쪽 이동, 임계값 미만 흔들림은 양보가 아니다
    assert not approach(S, 10, 0.9, y_at=lambda d: +0.2).detected
    assert not approach(S, 10, 0.9, y_at=lambda d: -0.05).detected
    # π = 0인 거리(d ≥ d_max)에서의 변위는 무시: λ 불변
    b = approach(S, 10, 1.3, y_at=lambda d: -0.3)
    assert not b.detected and b.lam == 0.0


def test_pi_yield_clips():
    assert pi_yield(1.2, P) == 0.0 and pi_yield(5.0, P) == 0.0
    assert pi_yield(0.6, P) == 1.0 and pi_yield(0.1, P) == 1.0
    assert abs(pi_yield(0.9, P) - 0.5) < 1e-12


def test_no_information_beyond_d_max():
    # 기획안 핵심 관찰: d ≥ d_max이면 어떤 z를 넣어도 λ가 변하지 않는다.
    b = Belief(P)
    for z in (-0.5, -0.05, 0.0, 0.05, 0.5):
        for d in (1.2, 1.5, 3.0):
            assert b.update(z, d, 0.1) == 0.0
    s = Belief(S)
    for z in (-0.5, 0.0, 0.5):
        for d in (1.2, 1.5, 3.0):
            assert s.update(z, d, 0.1) == 0.0


def test_converges_inside_d_min():
    dt = 0.1
    b = Belief(P)
    for _ in range(10):
        b.update(-P.v_lat * dt, 0.5, dt)
    assert b.prob_yield() > 0.999
    b.reset()
    for _ in range(10):
        b.update(0.0, 0.5, dt)
    assert b.prob_yield() < 0.001
    assert abs(b.lam) <= P.lambda_clip


def test_sign_and_stability():
    assert loglik_ratio(-0.05, 0.9, 0.1, P) > 0 > loglik_ratio(0.0, 0.9, 0.1, P)
    assert loglik_ratio(+0.05, 0.9, 0.1, P) < 0       # 왼쪽 이동은 양보가 아니다
    assert abs(loglik_ratio(-50.0, 0.9, 0.1, P)) < float('inf')   # log-sum-exp
    b = Belief(P)
    b.update(-0.05, 0.5, 0.0)       # dt = 0이면 갱신 안 함
    assert b.lam == 0.0
