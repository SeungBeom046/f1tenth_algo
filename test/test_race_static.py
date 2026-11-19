"""레이스 개정안: 멈춘 앞차(S), 급정지 뒤 재출발(NY + resume_after), 멈춘 차 확정·우회(static_bypass)."""
from dataclasses import replace

from my_algo.aur import safety
from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.leader_logic import Leader
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0, static_bypass=True, go_hold_s=0.0))


def test_stopped_leader_and_resume():
    s = Leader(P, 'S', 'x')
    assert s.step(0.0, 0.0, 1.0, 1.0) == (0.0, -safety.lane_offset(1.0, P))
    ny = Leader(P, 'NY', 'x', resume_after=2.0)
    ny.t0, ny.t_armed, ny.brake_delay = 0.0, 1.0, 0.5
    assert ny.step(1.6, 0.9, 0.5, 1.0)[0] == 0.0           # 급정지
    assert ny.step(3.5, 0.0, 0.5, 1.0)[0] == 0.0           # 아직 2 s 안
    assert ny.step(3.7, 0.0, 0.5, 1.0)[0] == P.v_B_nominal and ny.resumed


def obs(t, d, v_b, y_b=-0.265, v_e=0.0):
    return Obs(t, v_e, True, d, y_b, v_b, 0.0, 1.0)


def test_never_moving_car_is_confirmed_fast_and_bypassed():
    fsm = EgoFSM(P, 'd')
    reasons = [fsm.step(obs(0.1 * k, 2.2, 0.0))[3] for k in range(20)]
    assert fsm.static_confirmed and fsm.state == 'PASS'
    assert reasons.index('static_bypass') * 0.1 >= P.static_settle_s + P.static_t_never - 1e-9


def test_candidate_holds_back_at_bypass_gap():
    fsm = EgoFSM(P, 'd')
    for k in range(7):
        v = fsm.step(obs(0.1 * k, 1.6, 0.0, v_e=0.5))[0]
    assert v == 0.0 and not fsm.static_confirmed           # 2.0 m보다 가까우면 확정 전에는 서서 기다린다


def test_car_that_was_moving_needs_longer_and_restart_cancels():
    fsm = EgoFSM(P, 'd')
    t = 0.0
    for _ in range(10):                                     # 달리는 것을 본다
        fsm.step(obs(t, 2.5, 0.9)); t += 0.1
    for _ in range(int(P.static_t_never / 0.1) + 5):      # 한 번도 안 움직인 차의 확정 시간은 지나도
        fsm.step(obs(t, 2.5, 0.0)); t += 0.1
    assert not fsm.static_confirmed
    fsm.step(obs(t, 2.5, 0.9))                              # 다시 출발하면 후보가 풀린다
    assert fsm._still_since is None


def test_off_keeps_belief_only():
    fsm = EgoFSM(replace(P, static_bypass=False), 'd')
    for k in range(40):
        fsm.step(obs(0.1 * k, 2.2, 0.0))
    assert not fsm.static_confirmed and fsm.state != 'PASS'
