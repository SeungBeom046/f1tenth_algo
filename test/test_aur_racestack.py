"""race_stack(ForzaETH)에서 옮긴 판단 보조 기능: 선행차 횡위치 예측(leader_pred_time), 추월 조건 유지 시간(go_hold_s).
둘 다 기본 off이며 꺼져 있으면 기존 판단과 같다."""
from dataclasses import replace

from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.params import EXPERIMENT_OVERRIDES, AurParams

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, experiment_safety=False, eps_follow=0.05))
W = 1.0
DT = 0.1


def obs(t, y_b, d=0.8, ego_lat=0.0):
    return Obs(t, 0.9, True, d, y_b, 0.9, ego_lat, W, None, None)


def test_defaults_are_off():
    p = AurParams()
    assert p.leader_pred_time == 0.0 and p.go_hold_s == 0.0


def test_leader_vlat_is_least_squares_slope():
    fsm = EgoFSM(P, 'd')
    for k in range(6):
        fsm._update_leader_vlat(obs(k * DT, -0.20 + 0.3 * k * DT))
    assert abs(fsm.leader_vlat - 0.3) < 1e-9
    assert len(fsm._yb_track) == P.leader_vlat_window


def test_shift_counts_only_leftward_motion():
    on = replace(P, leader_pred_time=0.2)
    for vlat, expect in ((0.3, 0.06), (-0.3, 0.0)):
        fsm = EgoFSM(on, 'd')
        for k in range(5):
            fsm._update_leader_vlat(obs(k * DT, -0.2 + vlat * k * DT))
        assert abs(fsm._lat_shift() - expect) < 1e-9
    off = EgoFSM(P, 'd')
    for k in range(5):
        off._update_leader_vlat(obs(k * DT, 0.3 * k * DT))
    assert off._lat_shift() == 0.0


def _abort_scan(p):
    """PASS 중(겹치기 전) 선행차가 −0.25 m에서 0.4 m/s로 왼쪽으로 돌아올 때 중단한 scan 번호."""
    fsm = EgoFSM(p, 'd')
    fsm.state = 'PASS'
    fsm._y_b_most_yielded = -0.25
    fsm._yb_hist = [-0.25]
    for k in range(1, 30):
        o = obs(k * DT, -0.25 + 0.4 * k * DT, ego_lat=0.2)
        fsm._update_leader_vlat(o)
        if fsm._check_pass_abort(o, W):
            return k
    return None


def test_prediction_aborts_earlier_on_return():
    k_off = _abort_scan(P)
    k_on = _abort_scan(replace(P, leader_pred_time=0.2))
    assert k_off is not None and k_on is not None and k_on < k_off


def test_prediction_does_not_abort_a_steady_yielded_leader():
    fsm = EgoFSM(replace(P, leader_pred_time=0.2), 'd')
    fsm.state = 'PASS'
    fsm._y_b_most_yielded = -0.25
    fsm._yb_hist = [-0.25]
    for k in range(1, 20):
        o = obs(k * DT, -0.25 + (0.01 if k % 2 else -0.01), ego_lat=0.2)     # ±1 cm 잡음
        fsm._update_leader_vlat(o)
        assert not fsm._check_pass_abort(o, W)


def _go_scan(p, gap_at=None):
    """belief Y가 확정된 PROBE에서 횡 여유가 계속 맞을 때 PASS로 바뀐 scan 번호. gap_at scan에서는 여유가 없다."""
    fsm = EgoFSM(p, 'd')
    fsm.state, fsm.t_probe, fsm.decided = 'PROBE', 0.0, 'Y'
    for k in range(1, 30):
        o = obs(k * DT, 0.10 if k == gap_at else -0.25)
        fsm._decide(o, o, W, 'd')
        if fsm.state == 'PASS':
            return k
    return None


def test_go_hold_delays_and_resets():
    assert _go_scan(P) == 1
    on = replace(P, go_hold_s=0.3)
    assert _go_scan(on) == 4                    # 1에서 시작, 0.3 s 뒤
    assert _go_scan(on, gap_at=3) == 7          # 3에서 끊겨 4부터 다시 센다
