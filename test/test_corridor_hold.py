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
