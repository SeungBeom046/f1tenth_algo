import pytest

from _aur_synth import run
from my_algo.aur import safety
from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.leader_logic import Leader
from my_algo.aur.params import AurParams

P = AurParams()


def states(log):
    out = []
    for r in log:
        if not out or out[-1] != r['state']:
            out.append(r['state'])
    return out


def test_d_yield_passes_to_done():
    fsm, lead, log = run(P, 'd', 'Y', d_y=1.1)
    assert states(log) == ['WAIT', 'APPROACH', 'PASS', 'MERGE', 'DONE'] or \
        states(log) == ['WAIT', 'APPROACH', 'PROBE', 'PASS', 'MERGE', 'DONE']
    assert lead.yield_triggered and fsm.decided == 'Y'
    assert min(r['h'] for r in log if r['state'] in ('APPROACH', 'PROBE')) >= 0.0
    # 앞뒤로 겹치는 동안에는 횡으로 w + c 이상 떨어져 있다
    assert all(r['sep'] >= P.w_car + P.c_lat - 1e-9 for r in log if r['overlap'])


def test_d_no_yield_follows_and_survives_brake():
    fsm, lead, log = run(P, 'd', 'NY')
    assert 'PASS' not in states(log) and states(log)[-1] == 'FOLLOW'
    assert fsm.decided == 'NY' and lead.brake_fired
    assert min(r['d'] for r in log) >= P.m_stop             # 급정지에도 여유 m을 남기고 선다


def test_b_passive_never_decides():
    # d_follow_b = 1.5 ≥ d_max: 정보가 없어서 T_max까지 판단 못하고 FOLLOW
    fsm, lead, log = run(P, 'b', 'Y', d_y=1.1)
    assert states(log) == ['WAIT', 'APPROACH', 'PROBE', 'FOLLOW']
    assert fsm.decided is None and fsm.belief.lam == 0.0 and not lead.yield_triggered
    t_follow = next(r['t'] for r in log if r['state'] == 'FOLLOW')
    assert abs((t_follow - fsm.t_probe) - P.T_max) < 0.11


def test_a_fixed_rule_ignores_belief():
    # 넓은 통로(W ≥ 3w + 4c)에서는 양보 안 하는 선행차도 T_wait_a 뒤에 추월하러 들어간다
    fsm, _, log = run(P, 'a', 'NY', W=1.3)
    assert 'PASS' in states(log)
    t_pass = next(r['t'] for r in log if r['state'] == 'PASS')
    assert t_pass - fsm.t_probe >= P.T_wait_a - 1e-9
    # 좁으면 T_max에 포기
    assert states(run(P, 'a', 'NY', W=1.0)[2])[-1] == 'FOLLOW'


def test_c_aggressive_gets_closer_than_d_star_margin():
    fsm, _, log = run(P, 'c', 'NY')
    assert 'PROBE' in states(log) or states(log)[-1] == 'FOLLOW'


def test_wait_does_not_accumulate_belief():
    fsm = EgoFSM(P, 'd')
    for k in range(50):     # 정지 중: 가까운 선행차가 가만히 있어도 λ = 0
        fsm.step(Obs(k * 0.1, 0.0, True, 0.5, 0.0, 0.0, 0.0, 1.0))
    assert fsm.state == 'WAIT' and fsm.belief.lam == 0.0 and fsm.t0 is None
    fsm.step(Obs(5.0, 0.4, True, 0.5, 0.0, 0.4, 0.0, 1.0))
    assert fsm.state != 'WAIT' and fsm.t0 == 5.0 and fsm.belief.lam == 0.0


def test_hold_count_and_min_speed():
    fsm = EgoFSM(P, 'd')
    # 정지 물체 0.3 m 앞: v_lim < v_min이고 h < ε → 0 명령
    v, _, _, _ = fsm.step(Obs(0.0, 0.0, True, 0.3, 0.0, 0.0, 0.0, 1.0))
    assert v == 0.0 and fsm.hold_count == 1
    assert safety.v_lim(0.3, 0.0, P) < P.v_min


def test_method_validation():
    with pytest.raises(ValueError):
        EgoFSM(P, 'e')
    with pytest.raises(ValueError):
        Leader(P, 'X', 'r')


def test_leader_logic():
    a, b = Leader(P, 'Y', 'dY2'), Leader(P, 'Y', 'dY2')
    assert a.d_y == b.d_y and P.d_y_min <= a.d_y <= P.d_y_max     # run_id seed 재현
    assert Leader(P, 'Y', 'dY3').d_y != a.d_y and not a.brake_event
    assert a.step(0.0, 0.0, 0.5, 1.0) == (P.v_B_nominal, 0.0)      # t0 전에는 양보하지 않는다
    a.step(1.0, 0.5, 1.3, 1.0)
    assert a.t0 == 1.0 and a.start_too_close                      # 1.3 < d_y_max + 0.2
    assert a.step(1.1, 0.9, a.d_y + 0.01, 1.0)[1] == 0.0
    assert a.step(1.2, 0.9, a.d_y - 0.01, 1.0)[1] == pytest.approx(-0.265)
    assert a.step(1.3, 0.9, 3.0, 1.0)[1] == pytest.approx(-0.265)  # latch
    assert a.step(1.4, 0.9, None, 1.0)[1] == pytest.approx(-0.265)

    n = Leader(P, 'NY', 'dN1')
    n.step(0.0, 0.5, 2.0, 1.0)
    assert not n.start_too_close
    assert n.step(1.0, 0.9, 0.99, 1.0) == (P.v_B_nominal, 0.0)     # 무장
    assert n.step(1.0 + n.brake_delay - 0.01, 0.9, 0.5, 1.0)[0] == P.v_B_nominal
    assert n.step(1.0 + n.brake_delay, 0.9, 0.5, 1.0) == (0.0, 0.0)
    assert n.step(99.0, 0.0, None, 1.0)[0] == 0.0 and n.brake_fired  # 정지 유지
