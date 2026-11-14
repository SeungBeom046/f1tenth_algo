import pytest

from _aur_synth import CAR_LEN, box, scan
from my_algo.aur.corridor import Corridor
from my_algo.aur.params import AurParams
from my_algo.aur.tracker import VehicleTracker

P = AurParams()
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
            assert abs(trk.y_b + 0.265) < HW + 0.02     # 보이는 면의 중앙
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
