import math
from dataclasses import replace

from my_algo.aur.corridor import Corridor, _WallLine
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.aur.tracker import Pt

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0, guardrail_local_view=True))


def test_short_side_returns_need_rear_sector_to_initialize():
    # 가까운 레일 면은 후측면에만 반환되고, 앞측면은 더 바깥의 면이 반환되는 합성 입력.
    points = []
    for i in range(1440):
        angle = -math.pi + 2 * math.pi * i / 1440
        if abs(math.sin(angle)) < 1e-6:
            continue
        distance_y = .5 if abs(angle) >= math.radians(112) else .60
        distance = distance_y / abs(math.sin(angle))
        points.append(Pt(i, angle, distance * math.cos(angle), distance * math.sin(angle)))
    old = Corridor(replace(P, guardrail_local_view=False)).update(points)
    new = Corridor(P)
    for i in range(3):
        new.update(points, stamp=i * .1)
    assert not old.valid
    assert new.valid and abs(new.ego_lateral) < .01 and abs(new.heading) < .01


def line(side, distance, heading, count=13):
    slope = math.tan(-heading)
    offset = side * distance / math.cos(heading)
    xs = [-.6 + 1.2 * i / (count - 1) for i in range(count)]
    points = [Pt(i, 0.0, x, offset + slope * x) for i, x in enumerate(xs)]
    return _WallLine(points, heading, distance, side, 1.2)


def initialize(cor):
    for _ in range(3):
        cor._guardrail_update({1: [], -1: []}, None)


def test_matching_width_cannot_replace_wall_with_abrupt_heading_jump():
    cor = Corridor(P)
    good = {side: [line(side, .52, 0.0)] for side in (1, -1)}
    cor._wall_lines = lambda pts, side, width: good[side]
    initialize(cor)
    assert cor.valid
    candidates = {side: good[side] + [line(side, .50, math.radians(20))] for side in (1, -1)}
    cor._wall_lines = lambda pts, side, width: candidates[side]
    cor._guardrail_update({1: [], -1: []}, None)
    assert cor.valid and abs(cor.heading) < .01


def test_inconsistent_wall_only_does_not_overwrite_last_heading():
    cor = Corridor(P)
    cor._wall_lines = lambda pts, side, width: [line(side, .5, 0.0)]
    initialize(cor)
    cor._wall_lines = lambda pts, side, width: [line(side, .5, math.radians(20))]
    cor._guardrail_update({1: [], -1: []}, None)
    assert not cor.valid and cor.heading == 0.0


def test_stopped_lateral_controller_does_not_accumulate_integral():
    from my_algo.aur.lateral import Lateral
    lateral = Lateral(P)
    for _ in range(100):
        lateral.step(.03, 0., .1, integrate=False)
    assert lateral.i_term == 0.0
    for _ in range(10):
        lateral.step(.03, 0., .1, integrate=True)
    assert lateral.i_term < 0.0


def test_empty_corridor_is_measured_once_per_new_frame():
    from _aur_synth import scan
    from test_aur_guardrail import scan_message
    from my_algo.aur.ego_runner import EgoRunner
    runner = EgoRunner(P, 'd')
    msg = scan_message(scan(1., 0., n=1440))
    update = runner.corridor.update
    calls = []

    def counted(*args, **kwargs):
        calls.append(1)
        return update(*args, **kwargs)
    runner.corridor.update = counted
    for i in range(2):
        runner.step(i * .1, msg, 0.)
        assert len(calls) == i + 1


def test_initialization_prefers_supported_rails_with_small_direction_difference():
    cor = Corridor(P)
    candidates = {
        1: [line(1, .54, math.radians(-4), 31), line(1, .50, math.radians(8), 20)],
        -1: [line(-1, .54, math.radians(-1), 35), line(-1, .50, math.radians(7), 47)],
    }
    cor._wall_lines = lambda pts, side, width: candidates[side]
    initialize(cor)
    assert cor.valid and abs(math.degrees(cor.heading) + 4) < .01


def test_initial_wall_confirmation_counts_distinct_frames():
    cor = Corridor(P)
    cor._wall_lines = lambda pts, side, width: [line(side, .5, 0.0)]
    for _ in range(3):
        cor.update([], stamp=0.0)
    assert not cor.valid and cor.initializing
    cor.update([], stamp=.1)
    assert not cor.valid
    cor.update([], stamp=.2)
    assert cor.valid


def test_well_supported_pair_can_follow_real_vehicle_rotation():
    cor = Corridor(P)
    cor._wall_lines = lambda pts, side, width: [line(side, .5, 0.0, 40)]
    initialize(cor)
    cor._wall_lines = lambda pts, side, width: [line(side, .5, math.radians(20), 40)]
    cor._guardrail_update({1: [], -1: []}, None)
    assert cor.valid and abs(math.degrees(cor.heading) - 20) < .01
