"""레일을 놓쳤을 때 대체 주행: 빈 공간 횡위치(free_space_lateral)와 EgoRunner 조건."""
from dataclasses import replace
from types import SimpleNamespace

from my_algo.aur.ego_runner import rail_fallback
from my_algo.aur.fallback import free_space_lateral
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.aur.tracker import Pt

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0, rail_fallback_v=0.9))


def rails(left, right, outer=None, extra=()):
    pts, i = [], 0
    for k in range(40):
        x = -0.6 + 2.1 * k / 39
        for y in (left, -right) + ((outer,) if outer else ()):
            pts.append(Pt(i, 0.0, x, y))
            i += 1
    return pts + [Pt(i + k, 0.0, x, y) for k, (x, y) in enumerate(extra)]


def test_center_offset_and_outer_wall():
    assert abs(free_space_lateral(rails(0.5, 0.5), 1.0, P)) < 1e-9
    assert abs(free_space_lateral(rails(0.3, 0.7), 1.0, P) - 0.2) < 1e-9      # 오른쪽은 폭 근처 밖이라 왼쪽만
    # 오른쪽 레일이 안 보이고 바깥 복도 벽(1.0 m)만 보이면 그 벽은 버리고 왼쪽으로 잰다
    assert abs(free_space_lateral(rails(0.4, 1.0), 1.0, P) - 0.1) < 1e-9


def test_blocked_path_or_no_evidence_returns_none():
    assert free_space_lateral(rails(0.5, 0.5, extra=[(1.0, 0.05)] * 3), 1.0, P) is None
    assert free_space_lateral([], 1.0, P) is None
    assert free_space_lateral(rails(0.5, 0.5), None, P) is None


def car(t_rail, last=None, state='APPROACH', pending=None):
    from my_algo.aur.lateral import Lateral
    return SimpleNamespace(p=P, fsm=SimpleNamespace(state=state, _last=last),
                           tracker=SimpleNamespace(pending_d=pending), corridor=SimpleNamespace(heading=0.0),
                           lateral=Lateral(P), t_rail=t_rail)


def test_runner_conditions():
    pts = rails(0.5, 0.5)
    out = rail_fallback(car(1.0), 2.0, pts, 1.0, 0.9)
    assert out[0] == 0.9 and out[-1] == 'rail_fallback'
    assert rail_fallback(car(1.0), 4.5, pts, 1.0, 0.9) is None                 # 3 s 넘게 레일 없음
    assert rail_fallback(car(1.0, last=object()), 2.0, pts, 1.0, 0.9) is None  # 앞차를 본 적 있음
    assert rail_fallback(car(1.0, state='PASS'), 2.0, pts, 1.0, 0.9) is None
    assert rail_fallback(car(1.0, pending=1.2), 2.0, pts, 1.0, 0.9) is None
    assert rail_fallback(car(None), 2.0, pts, 1.0, 0.9) is None
    off = car(1.0)
    off.p = replace(P, rail_fallback_v=None)
    assert rail_fallback(off, 2.0, pts, 1.0, 0.9) is None
