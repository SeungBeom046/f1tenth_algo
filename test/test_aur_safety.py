import os
import tempfile

import pytest

from my_algo.aur import safety, speed
from my_algo.aur.params import AurParams, load_params

P = AurParams()


def test_closed_form_identities():
    for v_b in (0.0, 0.5, 0.9, 1.5):
        assert abs(safety.v_lim(safety.d_star(v_b, P), v_b, P) - v_b) < 1e-6
    assert abs(safety.d_star(0.9, P) - 0.335) < 1e-12
    lo, hi = safety.pass_window(0.27, 0.10)
    assert abs(lo - 0.84) < 1e-12 and abs(hi - 1.21) < 1e-12


def test_v_lim_keeps_h_at_eps():
    for d in (0.3, 0.5, 1.0, 2.0):
        for v_b in (0.0, 0.9):
            v = safety.v_lim(d, v_b, P)
            if v > 0:
                assert abs(safety.h(d, v, v_b, P) - P.eps_follow) < 1e-9
    assert safety.v_lim(0.0, 0.0, P) == 0.0                # 근호 안 음수
    assert safety.d_safe(1.0, -3.0, P) == safety.d_safe(1.0, 0.0, P)   # v_b 클리핑


def test_probe_and_lateral():
    assert safety.probe_feasible(0.9, P)
    assert not safety.probe_feasible(8.0, P)
    # W = 1.0: 선행차가 중앙이면 불가, δ = 0.265만큼 오른쪽으로 비키면 가능
    assert not safety.lateral_pass_ok(1.0, 0.0, 0.27, 0.10)
    assert abs(safety.lane_offset(1.0, P) - 0.265) < 1e-12
    assert safety.lateral_pass_ok(1.0, -0.265, 0.27, 0.10)
    assert safety.lateral_pass_ok(1.3, 0.0, 0.27, 0.10)    # 넓으면 양보 없이도 가능


def test_min_speed_rule():
    assert abs(P.v_min - 4000.0 / 4614.0) < 1e-12
    assert speed.min_speed_rule(0.5, 0.10, 0.9, P) == (P.v_min, True)     # h ≥ ε: hold
    assert speed.min_speed_rule(0.5, 0.01, 0.9, P) == (0.0, True)         # h < ε: 정지
    assert speed.min_speed_rule(0.5, 0.10, 0.0, P) == (0.0, True)         # 정지 중에는 hold로 출발하지 않는다
    assert speed.min_speed_rule(0.0, 9.0, 0.9, P) == (0.0, False)
    assert speed.min_speed_rule(1.2, -1.0, 0.9, P) == (1.2, False)
    assert abs(speed.from_erpm(speed.to_erpm(1.3, P), P) - 1.3) < 1e-12


def test_load_params_yaml():
    pytest.importorskip('yaml')
    with tempfile.TemporaryDirectory() as tmp:
        a, b = os.path.join(tmp, 'a.yaml'), os.path.join(tmp, 'b.yaml')
        open(a, 'w').write('aur:\n  t_d: 0.2\n  W: 1.0\n')
        open(b, 'w').write('aur:\n  t_d: 0.25\n')
        p = load_params(a, b, eta=0.95)
        assert (p.t_d, p.W, p.eta, p.a_brake) == (0.25, 1.0, 0.95, 3.0)
        open(b, 'w').write('aur:\n  t_dd: 0.25\n')
        with pytest.raises(ValueError):
            load_params(b)
