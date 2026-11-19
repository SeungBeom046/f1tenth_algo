"""실차 overtake_01·02 (2026-11-19) 대응: 추월 시작 여유 히스테리시스, 중단 뒤 차선 유지, 출발 전 변위 belief, FOLLOW 재추월."""
from dataclasses import replace

from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0, go_hold_s=0.0, static_bypass=False))


def obs(t, d, y_b, v_e=0.5, v_b=0.5, ego_lat=0.0, front_rel=None):
    return Obs(t, v_e, True, d, y_b, v_b, ego_lat, 1.0, front_rel)


def to_pass(p, y_b):
    """belief Y로 확정된 APPROACH에서 한 scan의 go 판정."""
    fsm = EgoFSM(p, 'd')
    fsm.state, fsm.decided = 'APPROACH', 'Y'
    fsm.step(obs(0.0, 1.2, y_b))
    return fsm


def test_pass_start_needs_extra_margin_with_hysteresis():
    # y_B −0.14: 여유 +0.035 m (overtake_02와 같은 경계)
    assert to_pass(P, -0.14).state == 'PASS'
    q = replace(P, pass_margin_hyst=True)
    fsm = to_pass(q, -0.14)
    assert fsm.state != 'PASS'
    assert to_pass(q, -0.17).state == 'PASS'                      # 여유 +0.065 m ≥ 0.05


def test_single_noisy_scan_does_not_abort_with_hysteresis():
    for hyst, aborted in ((False, 1), (True, 0)):
        q = replace(P, pass_margin_hyst=hyst)
        fsm = to_pass(q, -0.20)
        assert fsm.state == 'PASS'
        for k, y in enumerate([-0.20, -0.09, -0.20]):             # 한 scan만 −0.09 (여유 −0.015)
            fsm.step(obs(0.1 * (k + 1), 1.1, y))
        assert fsm.abort_count == aborted


def test_retreat_holds_lane_until_slow():
    for hold, expect in ((False, 'FOLLOW'), (True, 'RETREAT')):
        q = replace(P, retreat_hold=hold, abort_mode='retreat')
        fsm = to_pass(q, -0.20)
        r = fsm.step(obs(0.1, 1.3, -0.05, v_e=0.9, v_b=0.1))     # 선행차가 가운데로: 중단
        assert fsm.abort_count == 1 and fsm.state == expect
        if hold:
            v, y_ref, _, reason = r
            assert y_ref > 0.2 and reason == 'retreat' and v <= q.v_min + 1e-9     # 추월 차선 유지, 최소 속도 이하
            fsm.step(obs(0.2, 1.2, -0.05, v_e=0.2, v_b=0.1))       # 거의 멈춤 → FOLLOW
            assert fsm.state == 'FOLLOW'


def test_belief_counts_yield_before_departure():
    for pre, decided in ((False, None), (True, 'Y')):
        q = replace(P, belief_pre_t0=pre)
        fsm = EgoFSM(q, 'd')
        t = 0.0
        for y in (-0.09, -0.09, -0.10, -0.12, -0.14, -0.15, -0.15, -0.15):   # ego는 서 있고 선행차가 6 cm 비킴
            fsm.step(obs(t, 1.0, y, v_e=0.0)); t += 0.1
        for y in (-0.16, -0.17, -0.18, -0.19):                                # 출발 뒤 4 cm 더
            fsm.step(obs(t, 0.9, y, v_e=0.5)); t += 0.1
        assert fsm.decided == decided
