"""자차 위치 필터: 자이로 예측 + 레일 측정 고정 비율 보정."""
import math
from dataclasses import replace
from types import SimpleNamespace

from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.aur.pose import PoseFilter

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, pose_filter=True, pose_k_heading=0.2, pose_k_lateral=0.5))


def cor(y, psi, valid=True, fresh=True):
    side = 0.5 if fresh else None
    return SimpleNamespace(valid=valid, left=side, right=side, ego_lateral=y, heading=psi)


def test_noisy_heading_is_pulled_only_partly_and_gyro_carries_turn():
    f = PoseFilter(P)
    c = cor(0.0, 0.0)
    f.apply(c, 0.0, 1.0, 0.0)
    c = cor(0.0, math.radians(5))           # 측정만 5° 튐, 자이로는 회전 없음
    f.apply(c, 0.1, 1.0, 0.0)
    assert abs(math.degrees(c.heading) - 1.0) < 1e-9
    c = cor(c.ego_lateral, c.heading)       # 측정이 회전 전 값에 머물면 자이로 회전의 (1 − k)만큼 간다
    f.apply(c, 0.2, 1.0, 0.5)
    assert abs(c.heading - (math.radians(1.0) + 0.8 * 0.05)) < 1e-9


def test_prediction_only_while_rail_missing_and_reset_after_long_loss():
    f = PoseFilter(P)
    f.apply(cor(0.1, 0.0), 0.0, 1.0, 0.0)
    c = cor(0.1, 0.0, valid=True, fresh=False)     # rail_hold: 예측만
    f.apply(c, 0.1, 1.0, 0.2)
    assert abs(c.heading - 0.02) < 1e-9 and c.ego_lateral > 0.1
    c = cor(-0.2, 0.3)                              # 2 s 뒤 처음 보는 측정은 그대로 받는다
    f.apply(c, 2.1, 1.0, 0.0)
    assert c.ego_lateral == -0.2 and c.heading == 0.3


def test_off_or_no_gyro_leaves_measurement():
    for p, rate in ((replace(P, pose_filter=False), 0.1), (P, None)):
        f = PoseFilter(p)
        for k in range(3):
            c = cor(0.01 * k, 0.02 * k)
            f.apply(c, 0.1 * k, 1.0, rate)
            assert (c.ego_lateral, c.heading) == (0.01 * k, 0.02 * k)


def test_gate_downweights_heading_jump():
    q = replace(P, pose_gate_deg=3.0, pose_gate_scale=0.2)
    for p, expect in ((P, 0.2 * math.radians(6)), (q, 0.2 * 0.2 * math.radians(6))):
        f = PoseFilter(p)
        f.apply(cor(0.0, 0.0), 0.0, 0.0, 0.0)
        c = cor(0.0, math.radians(6))                 # 서 있는데 6° 튐
        f.apply(c, 0.1, 0.0, 0.0)
        assert abs(c.heading - expect) < 1e-9
