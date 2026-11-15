"""AEBSafetyModel: geometric 모드 추가가 legacy 동작을 바꾸지 않는지, geometric 판정이 맞는지."""
import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from my_algo.aeb import AEBSafetyModel                 # noqa: E402
from my_algo.vehicle_utils import ReactiveDriveCommand  # noqa: E402


def scan_with_point(x_bumper, y, lidar_to_bumper=0.30, n=1440):
    """앞범퍼 기준 (x, y)에 점 하나."""
    inc = 2 * math.pi / n
    ranges = [float('inf')] * n
    a = math.atan2(y, x_bumper + lidar_to_bumper)
    ranges[int(round((a + math.pi) / inc)) % n] = math.hypot(x_bumper + lidar_to_bumper, y)
    return SimpleNamespace(angle_min=-math.pi, angle_increment=inc, ranges=ranges, range_min=0.05, range_max=20.0)


CMD = ReactiveDriveCommand(0.0, 5000.0, 'WALL_FOLLOW')


def test_legacy_unchanged():
    m = AEBSafetyModel()
    assert m.model == 'legacy'
    # v = 0.9: 한계 = max(0.25, 0.81/5.6 + 0.20, 0.55·0.9) = 0.495
    assert m.check_emergency_braking(scan_with_point(0.49, 0.0), 0.9, CMD)
    assert not m.check_emergency_braking(scan_with_point(0.51, 0.0), 0.9, CMD)
    assert 'aeb_limit=0.49m' in m.last_reason or 'aeb_limit=0.50m' in m.last_reason
    assert m.check_emergency_braking(scan_with_point(0.20, 0.29), 0.0, CMD)     # 띠 ±0.30


def test_geometric():
    m = AEBSafetyModel(model='geometric', path_margin=0.01, ttc_threshold=0.35)
    assert not m.check_emergency_braking(scan_with_point(0.10, 0.20), 0.9, CMD)  # 띠 ±0.16 밖 (벽 0.235)
    assert m.check_emergency_braking(scan_with_point(0.30, 0.0), 0.9, CMD)       # 0.9·0.35 = 0.315
    assert not m.check_emergency_braking(scan_with_point(0.33, 0.0), 0.9, CMD)
    assert m.check_emergency_braking(scan_with_point(0.24, 0.0), None, CMD)      # 속도 모름 → 0.25
    assert not m.check_emergency_braking(scan_with_point(0.30, 0.0), None, CMD)
    assert 'speed=unknown' in m.last_reason
    with pytest.raises(ValueError):
        AEBSafetyModel(model='x')
