import pytest

from _aur_synth import CAR_LEN, box, scan
from my_algo.aur.corridor import Corridor
from dataclasses import replace

from my_algo.aur.params import EXPERIMENT_OVERRIDES, AurParams
from my_algo.aur.tracker import VehicleTracker

P = replace(AurParams(), **EXPERIMENT_OVERRIDES)     # 실험 설정 (exp_common.yaml과 같음)
LF, LR, HW = P.lidar_to_front, P.lidar_to_rear, P.w_car / 2


def lead_box(d, y_rel):
    """앞범퍼에서 d 앞, 라이다 기준 횡위치 y_rel인 선행차."""
    return box(LF + d, LF + d + CAR_LEN, y_rel - HW, y_rel + HW)


@pytest.mark.parametrize('d,y_b,ego_y', [
    (2.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.335, 0.0, 0.0),
    (1.0, -0.265, 0.0),        # 양보한 선행차: 벽에서 0.10 m
    (0.8, -0.265, 0.2), (1.5, 0.1, -0.15),
])
def test_front_detection_within_2cm(d, y_b, ego_y):
    pts = scan(1.0, ego_y, [lead_box(d, y_b - ego_y)])
    trk = VehicleTracker(P).update(pts, 0.0, 0.9, ego_y, 1.0)
    assert trk.valid
    assert abs(trk.d - d) < 0.02
    assert abs(trk.y_b - y_b) < 0.02


def test_empty_corridor_and_far_vehicle_not_detected():
    assert not VehicleTracker(P).update(scan(1.0, 0.1), 0.0, 0.9, 0.1, 1.0).valid
    assert not VehicleTracker(P).update(scan(1.0, 0.0, [lead_box(3.5, 0.0)]), 0.0, 0.9, 0.0, 1.0).valid
    assert not VehicleTracker(P).update(scan(1.0, 0.0, [lead_box(1.0, 0.0)]), 0.0, 0.9, 0.0, None).valid


def test_v_b_estimate():
    tr, d, v_e, v_b = VehicleTracker(P), 2.0, 1.5, 0.9
    for k in range(30):
        trk = tr.update(scan(1.0, 0.0, [lead_box(d, 0.0)]), k * 0.1, v_e, 0.0, 1.0)
        d += (v_b - v_e) * 0.1
    assert abs(trk.v_b - v_b) < 0.1
    # 정지 물체로 바뀌면(게이트 장애물 등) 필터를 초기화한다
    trk = tr.update(scan(1.0, 0.0, [lead_box(1.5, 0.0)]), 3.0, 0.0, 0.0, 1.0)
    assert trk.v_b == 0.0


def test_rear_detection():
    d_rear = 0.9
    follower = box(-LR - d_rear - CAR_LEN, -LR - d_rear, -0.05 - HW, -0.05 + HW)
    trk = VehicleTracker(P, view='rear').update(scan(1.0, 0.0, [follower]), 0.0, 0.9, 0.0, 1.0)
    assert trk.valid
    assert abs(trk.d - d_rear) < 0.02 and abs(trk.y_b + 0.05) < 0.02
    # 앞에 있는 차는 후방 추적에 안 잡힌다
    assert not VehicleTracker(P, view='rear').update(scan(1.0, 0.0, [lead_box(1.0, 0.0)]), 0.0, 0.9, 0.0, 1.0).valid


def test_side_tracking_through_pass():
    # ego는 왼쪽 차선(+0.265), 선행차는 오른쪽(−0.265). ego가 0.6 m/s 빠르게 지나간다.
    tr, ego_y, y_rel = VehicleTracker(P), 0.265, -0.53
    assert tr.update(scan(1.0, ego_y, [lead_box(0.5, y_rel)]), 0.0, 1.5, ego_y, 1.0).valid
    d, seen = 0.5, 0
    for k in range(1, 40):
        d -= 0.06
        trk = tr.update_side(scan(1.0, ego_y, [lead_box(d, y_rel)]), k * 0.1, 1.5, ego_y, 1.0)
        true_front_rel = d + CAR_LEN + LF + LR
        if trk.valid:
            seen += 1
            assert abs(trk.y_b + 0.265) < 0.03
        if true_front_rel < -0.05:      # 선행차가 완전히 뒤로 갔을 때 앞면 위치가 맞아야 한다
            assert abs(trk.front_rel - true_front_rel) < 0.05
    assert seen > 30 and trk.front_rel < -P.m_stop


def test_corridor_excludes_vehicle_points():
    W, ego_y = 1.0, 0.3
    cor = Corridor(P).update(scan(W, ego_y))
    assert cor.valid and abs(cor.ego_lateral - ego_y) < 0.01 and abs(cor.W_est - W) < 0.01
    # 오른쪽에 나란히 선 차가 측면 섹터를 가린다
    beside = box(-0.1, 0.4, -0.67, -0.40)
    pts = scan(W, ego_y, [beside])
    car_idx = [q.i for q in pts if abs(q.y + ego_y) < W / 2 - 0.02]
    wrong = Corridor(P).update(pts)
    assert abs(wrong.ego_lateral - ego_y) > 0.1            # 제외하지 않으면 차를 벽으로 본다
    cor.update(pts, car_idx)
    assert cor.valid and abs(cor.ego_lateral - ego_y) < 0.01 and abs(cor.W_est - W) < 0.01
    # 차가 섹터 전체를 가리면: 직전 값 유지 + 플래그
    pts = scan(W, ego_y, [box(-1.0, 1.0, -0.67, -0.40)])
    cor.update(pts, [q.i for q in pts if abs(q.y + ego_y) < W / 2 - 0.02])
    assert not cor.valid and cor.right is None
    assert abs(cor.ego_lateral - ego_y) < 0.01 and abs(cor.W_est - W) < 0.01


def test_corridor_sees_near_wall_in_passing_lane():
    # 추월 차선(δ = 0.265, 벽까지 0.235 m)에서도 양쪽 벽이 잡혀야 횡 제어가 된다
    cor = Corridor(P).update(scan(1.0, 0.265))
    assert cor.valid and abs(cor.ego_lateral - 0.265) < 0.01


def test_corridor_heading_and_perpendicular_distance():
    import math
    from my_algo.aur.tracker import Pt
    W, y_e = 1.0, 0.1
    for psi in (0.0, 0.2, -0.3):
        pts = []
        for i in range(1440):       # 차가 ψ만큼 틀어진 직선 통로를 차량 좌표계에서 본다
            a = -math.pi + i * 2 * math.pi / 1440
            s = math.sin(a + psi)
            if abs(s) > 1e-6:
                r = ((W / 2 - y_e) if s > 0 else (-W / 2 - y_e)) / s
                pts.append(Pt(i, a, r * math.cos(a), r * math.sin(a)))
        cor = Corridor(P).update(pts)
        assert cor.valid and abs(cor.heading - psi) < 0.01
        assert abs(cor.ego_lateral - y_e) < 0.01 and abs(cor.W_est - W) < 0.01


def test_corridor_w_fixed_ignores_left_side():
    # 선행차: 왼쪽에 후행차가 붙어도 오른쪽 벽 + 실측 W로 횡위치를 구한다
    W, y = 1.0, -0.265
    pts = scan(W, y, [box(-0.3, 0.3, 0.13, 0.40)])          # 왼쪽 옆 후행차
    assert abs(Corridor(P).update(pts).ego_lateral - y) > 0.05   # W 고정 없으면 틀어진다
    cor = Corridor(P).update(pts, W_fixed=W)
    assert cor.valid and abs(cor.ego_lateral - y) < 0.01 and cor.W_est == W


def _yawed_lead(d, y_c, yaw, ego_y):
    """뒷면 중심 (d, y_c)(앞범퍼 기준, 통로 좌표)에서 yaw만큼 돈 선행차 선분과 참 왼쪽 모서리 y."""
    import math
    c, s = math.cos(yaw), math.sin(yaw)
    loc = [(0.0, -HW), (CAR_LEN, -HW), (CAR_LEN, HW), (0.0, HW)]
    pts = [(LF + d + c * x - s * y, y_c - ego_y + s * x + c * y) for x, y in loc]
    return [(pts[i], pts[(i + 1) % 4]) for i in range(4)], max(y for _, y in pts) + ego_y


@pytest.mark.parametrize('yaw_deg', [0.0, -10.0, -20.0, 10.0])
def test_left_edge_estimate(yaw_deg):
    import math
    ego_y = 0.15
    segs, y_l_true = _yawed_lead(0.6, -0.20, math.radians(yaw_deg), ego_y)
    pts = scan(1.0, ego_y, [segs])
    trk = VehicleTracker(P).update(pts, 0.0, 0.9, ego_y, 1.0)
    assert trk.valid and abs(trk.y_l - y_l_true) < 0.03


def test_lateral_ref_flag():
    from my_algo.aur import safety
    from my_algo.aur.ego_fsm import EgoFSM, Obs
    assert safety.lateral_pass_ok_edge(1.0, -0.265 + HW, 0.27, 0.10) == safety.lateral_pass_ok(1.0, -0.265, 0.27, 0.10)
    o = Obs(0.0, 0.9, True, 0.5, -0.20, 0.9, 0.0, 1.0, None, 0.05)    # 중심은 비켰지만 왼쪽 모서리가 0.05 (여유 0.45 < 0.47)
    assert EgoFSM(P, 'd')._lateral_ok(1.0, o)                           # center: −0.20 → 여유 있음
    assert not EgoFSM(replace(P, lateral_ref='left_edge'), 'd')._lateral_ok(1.0, o)
