"""낮은 가드레일의 희소한 점과 바깥 복도 벽이 섞이는 회귀 입력."""
import math
from dataclasses import replace
from types import SimpleNamespace

import pytest

from _aur_synth import scan
from my_algo.aur.corridor import Corridor
from my_algo.aur.ego_runner import EgoRunner
from my_algo.aur.leader_runner import LeaderRunner
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.aur.tracker import Pt

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES,
                               W=1.0, W_leader=1.0, corridor_guardrail=True))


def mixed_points(ego_y=0.0, heading=0.0, rails=True):
    """가드레일 폭 1 m, 복도 폭 2.7 m·중앙 +0.35 m. 가드레일 반환은 20%뿐이다."""
    points = []
    for i in range(1440):
        angle = -math.pi + i * 2 * math.pi / 1440
        direction = math.sin(angle + heading)
        if abs(direction) < 1e-6:
            continue
        side = 1 if direction > 0 else -1
        y = side * 0.5 if rails and i % 5 == 0 else side * 1.35 + 0.35
        distance = (y - ego_y) / direction
        distance += 0.003 * math.sin(i * 1.7)
        points.append(Pt(i, angle, distance * math.cos(angle), distance * math.sin(angle)))
    return points


def scan_message(points):
    ranges = [float('inf')] * 1440
    for q in points:
        ranges[q.i] = math.hypot(q.x, q.y)
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi,
                           angle_increment=2 * math.pi / 1440, range_min=0.05, range_max=20.0)


@pytest.mark.parametrize('ego_y', [-0.315, 0.0, 0.315])
@pytest.mark.parametrize('heading', [-0.2, 0.0, 0.2])
def test_sparse_guardrails_define_center_and_heading(ego_y, heading):
    cor = Corridor(P).update(mixed_points(ego_y, heading))
    assert cor.valid and cor.W_est == 1.0
    assert abs(cor.ego_lateral - ego_y) < 0.015
    assert abs(cor.heading - heading) < 0.015


def test_disabled_mode_reproduces_hallway_center_error():
    points = mixed_points()
    old = Corridor(replace(P, corridor_guardrail=False)).update(points)
    new = Corridor(P).update(points)
    assert abs(old.ego_lateral) > 0.1
    assert new.valid and abs(new.ego_lateral) < 0.015


def test_outer_hallway_is_rejected_at_start_and_after_rail_loss():
    cor = Corridor(P).update(mixed_points(rails=False))
    assert not cor.valid and cor.W_est is None
    cor.update(mixed_points(0.1, 0.1))
    assert cor.valid
    previous = cor.ego_lateral, cor.heading, cor.W_est
    cor.update(mixed_points(0.1, 0.1, rails=False))
    assert not cor.valid and (cor.ego_lateral, cor.heading, cor.W_est) == previous
    cor.update(mixed_points(0.1, 0.1))
    assert cor.valid and abs(cor.ego_lateral - 0.1) < 0.015


def test_measured_width_is_required():
    cor = Corridor(replace(P, W=None)).update(scan(1.0, 0.0))
    assert not cor.valid and cor.W_est is None
    assert not Corridor(replace(P, W=0.7)).update(scan(1.0, 0.0)).valid


def test_one_rail_can_bridge_vehicle_occlusion_after_lock():
    cor = Corridor(P).update(scan(1.0, 0.15))
    assert cor.valid
    points = scan(1.0, 0.17)
    cor.update(points, exclude_idx=[q.i for q in points if q.y < 0])
    assert cor.valid and abs(cor.ego_lateral - 0.17) < 0.01
    assert not Corridor(P).update([q for q in points if q.y > 0]).valid


def test_leader_uses_right_rail_with_measured_width():
    cor = Corridor(P).update(mixed_points(-0.265, 0.1), W_fixed=1.0)
    assert cor.valid and abs(cor.ego_lateral + 0.265) < 0.015
    assert abs(cor.heading - 0.1) < 0.015


def test_runners_stop_when_only_hallway_is_visible_and_recover():
    ego, leader = EgoRunner(P, 'd'), LeaderRunner(P, 'NY', 'guardrail')
    outer = scan_message(mixed_points(rails=False))
    assert ego.step(0.0, outer, 0.0)[0] == 0.0
    assert ego.step(0.1, outer, 0.0)[-1] == 'corridor_invalid'
    assert leader.step(0.0, outer, 0.0)[0] == 0.0
    good = scan_message(mixed_points())
    assert leader.step(0.1, good, 0.0)[0] > 0.0
    assert ego.step(0.2, good, 0.0)[-1] != 'corridor_invalid'
    assert leader.step(0.2, outer, 0.9)[0] == 0.0
