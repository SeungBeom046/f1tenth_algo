from my_algo.aur.belief import Belief, loglik_ratio, pi_yield
from my_algo.aur.params import AurParams

P = AurParams()


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
