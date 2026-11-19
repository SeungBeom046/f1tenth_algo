"""정지차 우회(stationary_pass): 좁게 보이는 정지 차 후보 확정과 추월 보류 이유 (실차 test_01, 2026-11-19)."""
from dataclasses import replace

from my_algo.aur.ego_fsm import EgoFSM, Obs, STATIONARY_PASS_MIN_D
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.aur.tracker import Pt, VehicleTracker

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0, stationary_pass=True, stationary_partial_lead=True,
                                static_bypass=False, stationary_geom_pass=False))   # 고정 1.5 m 규칙 (아래 G가 기하 규칙)


def blob(x, y, width, depth=0.1, n=6, i0=0):
    """앞범퍼 기준 x(라이다 x = x + lidar_to_front)에 폭 width의 점 묶음."""
    pts = []
    for k in range(n):
        u = k / (n - 1)
        pts.append(Pt(i0 + k, 0.0, x + P.lidar_to_front + depth * u * (1 - u) * 4, y - width / 2 + width * u))
    return pts


def run(tracker, frames):
    out = []
    for t, pts in frames:
        out.append(tracker.update(pts, t, 0.0, 0.0, 1.0))
    return out


def test_narrow_stationary_car_is_confirmed_after_three_scans_with_one_miss():
    trk = VehicleTracker(P)
    car = blob(2.08, -0.19, 0.07)
    res = run(trk, [(0.0, car), (0.1, car), (0.2, []), (0.3, car)])
    assert [r.valid for r in res] == [False, False, False, True]
    assert trk.pending_d is None and abs(res[-1].d - (2.08 - P.lidar_to_rear)) < 1e-9   # 좁으면 뒷범퍼를 0.22 m 앞으로 본다


def test_off_or_rail_fragment_or_long_gap_is_not_confirmed():
    car = blob(2.08, -0.19, 0.07)
    off = VehicleTracker(replace(P, stationary_partial_lead=False))
    assert not any(r.valid for r in run(off, [(0.1 * k, car) for k in range(5)]))
    rail = VehicleTracker(P)                                    # 벽 쪽 레일 조각 (|y| 0.43)
    assert not any(r.valid for r in run(rail, [(0.1 * k, blob(2.0, -0.43, 0.05)) for k in range(5)]))
    gap = VehicleTracker(P)                                     # 확인 중 track_timeout보다 오래 놓치면 처음부터
    res = run(gap, [(0.0, car), (0.1, car), (0.2 + P.track_timeout + 0.05, car), (0.75 + P.track_timeout, car)])
    assert not any(r.valid for r in res) and gap._candidate_count == 2


def obs(t, valid, d=None, y_b=None, v_e=0.0):
    return Obs(t, v_e, valid, d, y_b, 0.0, 0.0, 1.0)


def test_pass_block_reasons_and_pass_when_conditions_hold():
    fsm = EgoFSM(P, 'a')
    fsm.step(obs(0.0, False))
    assert fsm.pass_block == '추적 미확정' and fsm.state == 'WAIT'
    fsm.step(obs(0.1, True, 1.2, -0.3))
    assert fsm.pass_block.startswith('종방향 거리 부족 (d 1.20 < %.2f m)' % STATIONARY_PASS_MIN_D)
    fsm = EgoFSM(P, 'a')
    fsm.step(obs(0.0, True, 1.9, -0.05))                        # 선행차가 가운데: 왼쪽 공간 부족
    assert fsm.pass_block.startswith('횡 여유 부족 (W 1.00, 선행차 y_B -0.050, 차폭 0.27, 필요 0.470(w+2c), 잔여 -0.055 m')
    fsm = EgoFSM(P, 'a')
    states = []
    for k in range(5):
        fsm.step(obs(0.1 * k, True, 1.9, -0.3))
        states.append((fsm.state, fsm.pass_block))
    assert states[0] == ('WAIT', '조건 유지 시간 미충족 (0.00 / 0.30 s)')
    assert ('PASS', None) in states


def test_track_loss_resets_hold_and_blocks_pass():
    fsm = EgoFSM(P, 'a')
    for k, valid in enumerate([True, True, False, True, True]):
        fsm.step(obs(0.1 * k, valid, 1.9 if valid else None, -0.3 if valid else None))
    assert fsm.state != 'PASS' and fsm.pass_block.startswith('조건 유지 시간 미충족')


G = replace(P, stationary_geom_pass=True, stationary_steer_frac=0.8, go_hold_s=0.0)


def test_geometric_pass_distance_and_close_start():
    import math
    fsm = EgoFSM(G, 'a')
    shift, need = fsm._pass_need(Obs(0.0, 0.0, True, 1.35, -0.29, 0.0, 0.0, 1.0))
    radius = G.wheelbase / math.tan(0.8 * G.max_steering)
    assert abs(shift - (-0.29 + G.leader_width / 2 + G.stationary_clear + G.w_car / 2)) < 1e-9     # 약 0.03 m
    assert abs(need - (math.sqrt(radius ** 2 - (radius - shift) ** 2) + G.stationary_margin_x)) < 1e-9
    fsm.step(Obs(0.0, 0.0, True, 1.35, -0.29, 0.0, 0.0, 1.0))           # 1.5 m 안이지만 옆 이동이 작다
    assert fsm.state == 'PASS'
    near = EgoFSM(G, 'a')                                                # 가운데 가까운 차를 너무 가까이서
    near.step(Obs(0.0, 0.0, True, 0.4, -0.12, 0.0, 0.0, 1.0))
    assert near.state != 'PASS' and near.pass_block.startswith('종방향 거리 부족 (d 0.40 < 필요')


def test_stationary_pass_ignores_y_jitter_but_keeps_lateral_check():
    fsm = EgoFSM(G, 'a')
    fsm.step(Obs(0.0, 1.0, True, 1.35, -0.29, 0.0, 0.0, 1.0))
    assert fsm.state == 'PASS'
    for k, y in enumerate([-0.22, -0.29, -0.21, -0.22]):                 # 넓게·좁게 보여 y_B가 7 cm씩 튐
        fsm.step(Obs(0.1 * (k + 1), 1.0, True, 1.2, y, 0.0, 0.05, 1.0))
    assert fsm.state == 'PASS' and fsm.abort_count == 0
    fsm.step(Obs(0.6, 1.0, True, 1.1, 0.0, 0.0, 0.05, 1.0))               # 실제로 가운데로 들어오면 중단
    assert fsm.state != 'PASS' and fsm.abort_count == 1


def test_seen_wide_face_keeps_distance_steady():
    trk = VehicleTracker(G)
    wide, narrow = blob(1.56, -0.29, 0.26, n=14), blob(1.56, -0.22, 0.08)
    ds = [r.d for r in run(trk, [(0.0, wide), (0.1, wide), (0.2, wide), (0.3, narrow), (0.4, wide), (0.5, narrow)])]
    assert ds[3:] == [1.56, 1.56, 1.56]
    old = VehicleTracker(replace(G, stationary_geom_pass=False))
    ds = [r.d for r in run(old, [(0.0, wide), (0.1, wide), (0.2, wide), (0.3, narrow)])]
    assert abs(ds[3] - (1.56 - G.lidar_to_rear)) < 1e-9
