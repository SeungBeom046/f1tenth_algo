import math
from dataclasses import replace

from my_algo.aur.corridor import Corridor, _WallLine
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.aur.tracker import Pt

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0, guardrail_local_view=True,
                                corridor_hold_s=0.3, corridor_relock_s=1.0))


def line(side, distance=.5, count=13):
    points = [Pt(i, 0.0, -.6 + 1.2 * i / (count - 1), side * distance) for i in range(count)]
    return _WallLine(points, 0.0, distance, side, 1.2)


def locked(p=P):
    cor = Corridor(p)
    cor._wall_lines = lambda pts, side, width: [line(side)]
    for k in range(3):
        cor.update([], stamp=k * .1)
    assert cor.valid
    cor._wall_lines = lambda pts, side, width: []
    return cor


def test_short_invalid_gap_keeps_last_estimate():
    cor = locked()
    cor.update([], stamp=.4)
    assert cor.valid and cor.left is None
    cor.update([], stamp=.6)
    assert not cor.valid


def test_long_invalid_gap_reinitializes():
    cor = locked()
    cor.update([], stamp=1.5)
    assert not cor.valid and not cor._guardrail_locked
    cor._wall_lines = lambda pts, side, width: [line(side, .5 + side * .2)]   # 다른 자리에서 다시 잡는다
    for k in range(3):
        cor.update([], stamp=1.6 + k * .1)
    assert cor.valid and abs(cor.ego_lateral + .2) < 1e-9


def test_off_keeps_previous_behavior():
    cor = locked(replace(P, corridor_hold_s=0.0, corridor_relock_s=0.0))
    cor.update([], stamp=.4)
    assert not cor.valid
    cor.update([], stamp=5.0)
    assert not cor.valid and cor._guardrail_locked


def test_init_window_ignores_missed_frames():
    p = replace(P, corridor_init_window_s=2.0)
    cor = Corridor(p)
    found = lambda pts, side, width: [line(side)]
    for k in range(3):     # 잡힘, 못 잡음 두 번이 번갈아 온다
        cor._wall_lines = found
        cor.update([], stamp=k * .3)
        cor._wall_lines = lambda pts, side, width: []
        cor.update([], stamp=k * .3 + .1)
        cor.update([], stamp=k * .3 + .2)
    assert cor._guardrail_locked
    off = Corridor(replace(p, corridor_init_window_s=0.0))
    for k in range(3):
        off._wall_lines = found
        off.update([], stamp=k * .3)
        off._wall_lines = lambda pts, side, width: []
        off.update([], stamp=k * .3 + .1)
    assert not off._guardrail_locked


def test_init_window_expires():
    cor = Corridor(replace(P, corridor_init_window_s=1.0))
    cor._wall_lines = lambda pts, side, width: [line(side)]
    for k in range(3):
        cor.update([], stamp=k * 1.5)
    assert not cor._guardrail_locked


def test_heading_average_of_both_lines():
    def tilted(side, deg):
        w = line(side)
        return _WallLine(w.points, math.radians(deg), w.distance, side, w.span)
    for avg, expect in ((True, 2.0), (False, 4.0)):
        cor = Corridor(replace(P, corridor_heading_avg=avg))
        cor._wall_lines = lambda pts, side, width: [tilted(side, 4.0 if side > 0 else 0.0)]
        for k in range(3):
            cor.update([], stamp=k * .1)
        assert abs(math.degrees(cor.heading) - expect) < 1e-6
