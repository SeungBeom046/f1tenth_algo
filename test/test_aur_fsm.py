import pytest

from _aur_synth import run
from my_algo.aur import safety
from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.leader_logic import Leader
from dataclasses import replace

from my_algo.aur.params import EXPERIMENT_OVERRIDES, AurParams

P = replace(AurParams(), **EXPERIMENT_OVERRIDES)     # 실험 설정 (exp_common.yaml과 같음)


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


def test_hold_hysteresis():
    # v_lim < v_min, h ≥ ε인 상황(선행차가 v_min보다 조금 느림)에서 v_E만 바꿔 본다
    fsm = EgoFSM(P, 'd')
    cmd = lambda t, v_e: fsm.step(Obs(t, v_e, True, 0.33, 0.0, 0.80, 0.0, 1.0))[0]
    assert cmd(0.0, 0.30) == 0.0            # 0.35를 넘기 전에는 hold 안 함
    assert cmd(0.1, 0.36) == P.v_min        # 켜짐
    assert cmd(0.2, 0.30) == P.v_min        # 0.25~0.35 사이에서는 유지
    assert cmd(0.3, 0.24) == 0.0            # 꺼짐
    assert cmd(0.4, 0.30) == 0.0


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


def test_leader_return_after_yield():
    a = Leader(P, 'Y', 'r', return_after=2.0)
    a.step(0.0, 0.5, 2.0, 1.0)
    assert a.step(1.0, 0.9, 0.5, 1.0)[1] == pytest.approx(-0.265)
    assert a.step(2.9, 0.9, 0.5, 1.0)[1] == pytest.approx(-0.265)
    assert a.step(3.0, 0.9, 0.5, 1.0)[1] == 0.0


def test_lateral_controller():
    from my_algo.aur.lateral import Lateral
    lat = Lateral(P)
    assert lat.step(0.0, 0.0, 0.1) == 0.0
    s = [lat.step(0.0, 0.265, 0.1) for _ in range(10)]
    assert abs(lat.y_ref - 0.265) < 1e-9                          # 0.4 m/s 기울기로 0.7 s 안에 도달
    assert s[-1] > 0 and max(abs(b - a) for a, b in zip([0.0] + s, s)) <= P.steer_rate_limit + 1e-12
    lat2 = Lateral(P)
    assert lat2.step(0.02, 0.0, 0.1) == 0.0                       # deadband (오차 0.04 m → 0.012 rad)
    assert lat2.step(0.3, 0.0, 0.1) < 0                           # 왼쪽으로 치우쳤으면 오른쪽 조향


def _to_pass(fsm):
    """Y 판정까지 진행: d* 근처에서 선행차가 오른쪽으로 비킨다."""
    t, y = 0.0, 0.0
    fsm.step(Obs(t, 0.9, True, 0.6, 0.0, 0.9, 0.0, 1.0))
    while fsm.state != 'PASS':
        t += 0.1
        y = max(-0.265, y - 0.04)
        fsm.step(Obs(t, 0.9, True, 0.34, y, 0.9, 0.0, 1.0))
        assert t < 10.0
    return t, y


def test_pass_abort_before_overlap():
    fsm = EgoFSM(P, 'd')
    t, y = _to_pass(fsm)
    v, y_ref, state, _ = fsm.step(Obs(t + 0.1, 0.9, True, 0.30, y, 0.9, 0.1, 1.0))
    assert state == 'PASS' and y_ref > 0
    v, y_ref, state, reason = fsm.step(Obs(t + 0.2, 0.9, True, 0.30, y + P.y_yield_thr + 0.01, 0.9, 0.1, 1.0))
    assert (state, reason, y_ref) == ('FOLLOW', 'pass_abort', 0.0) and fsm.follow_reason == 'pass_abort'
    assert v <= safety.v_lim(0.30, 0.9, P) + 1e-9      # 추종 속도


def test_no_abort_after_overlap():
    fsm = EgoFSM(P, 'd')
    t, y = _to_pass(fsm)
    fsm.step(Obs(t + 0.1, 1.2, True, -0.05, y, 0.9, 0.2, 1.0, 0.8))    # 앞뒤로 겹침
    v, y_ref, state, _ = fsm.step(Obs(t + 0.2, 1.3, True, -0.10, y + 0.15, 0.9, 0.25, 1.0, 0.7))
    assert state == 'PASS' and y_ref > 0 and v == P.v_pass


def test_pass_abort_reference_is_most_yielded_position():
    # 선행차가 비키는 중(−0.12)에 PASS 진입 → −0.26까지 비켰다가 −0.17로 돌아오면 중단
    fsm = EgoFSM(P, 'd')
    t, y = _to_pass(fsm)
    for y in (-0.20, -0.26):
        t += 0.1
        assert fsm.step(Obs(t, 0.9, True, 0.30, y, 0.9, 0.1, 1.0))[2] == 'PASS'
    t += 0.1
    assert fsm.step(Obs(t, 0.9, True, 0.30, -0.26 + P.y_yield_thr + 0.01, 0.9, 0.1, 1.0))[2] == 'FOLLOW'
