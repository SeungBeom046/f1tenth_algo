"""AEBSafetyModel: geometric 모드 추가가 legacy 동작을 바꾸지 않는지, geometric 판정이 맞는지."""
import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from my_algo.baseline.aeb import AEBSafetyModel                 # noqa: E402
from my_algo.baseline.vehicle_utils import ReactiveDriveCommand  # noqa: E402


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


def test_curved_band_follows_steering():
    straight = AEBSafetyModel(model='geometric', path_margin=0.01, ttc_threshold=0.35)
    curved = AEBSafetyModel(model='geometric', path_margin=0.01, ttc_threshold=0.35, curved_band=True)
    left, right = ReactiveDriveCommand(0.2, 5000.0, 'PASS'), ReactiveDriveCommand(-0.2, 5000.0, 'PASS')
    # 조향 0이면 직선 띠와 같다
    for x, y in ((0.30, 0.0), (0.10, 0.20), (0.24, 0.0)):
        sc = scan_with_point(x, y)
        assert curved.check_emergency_braking(sc, 0.9, CMD) == straight.check_emergency_braking(sc, 0.9, CMD)
    # 앞 0.4 m, 왼쪽 0.2 m 점: 직선 띠 밖, 왼쪽으로 꺾으면 원호 위 → AEB, 오른쪽으로 꺾으면 아님
    sc = scan_with_point(0.40, 0.20)
    assert not straight.check_emergency_braking(sc, 1.5, left)
    assert curved.check_emergency_braking(sc, 1.5, left)
    assert not curved.check_emergency_braking(sc, 1.5, right)
    # 정면 0.4 m 점: 직선 띠는 걸리고, 오른쪽으로 크게 꺾으면 피해 간다
    sc = scan_with_point(0.40, 0.0)
    assert straight.check_emergency_braking(sc, 1.5, right)
    assert not curved.check_emergency_braking(sc, 1.5, ReactiveDriveCommand(-0.7, 5000.0, 'PASS'))
    # legacy에서는 curved_band가 꺼진다
    assert not AEBSafetyModel(curved_band=True).curved_band


def scan_wall(y_wall, yaw=0.0, front=None, n=1440):
    """y = y_wall에 나란한 벽 (차가 벽 쪽으로 yaw만큼 틀어짐), front > 0이면 앞범퍼 앞 front에 폭 0.27 차."""
    inc = 2 * math.pi / n
    ranges = []
    for i in range(n):
        a = -math.pi + i * inc
        best = float('inf')
        if y_wall is not None and abs(math.sin(a + yaw)) > 1e-6 and math.sin(a + yaw) * y_wall > 0:
            best = y_wall / math.sin(a + yaw)
        if front is not None and math.cos(a) > 0:
            x = front + 0.30
            if abs(x * math.tan(a)) < 0.135:
                best = min(best, x / math.cos(a))
        ranges.append(best)
    return SimpleNamespace(angle_min=-math.pi, angle_increment=inc, ranges=ranges, range_min=0.05, range_max=20.0)


def test_band_intersect_and_curve_cap_ignore_parallel_wall():
    # 실차 2026-11-17: 조향이 크게 꺾이면 휜 띠가 옆에 나란한 벽(라이다 옆 0.25 m)을 앞 장애물로 잡아 섰다
    kw = dict(model='geometric', path_margin=0.01, ttc_threshold=0.35, curved_band=True)
    old = AEBSafetyModel(**kw)
    new = AEBSafetyModel(curve_max_steering=0.3, band_intersect=True, **kw)
    cmd = lambda s: ReactiveDriveCommand(s, 5000.0, 'X')  # noqa: E731
    for y_wall, steer in ((0.25, 0.2), (0.25, 0.78), (0.5, 0.78)):
        assert old.check_emergency_braking(scan_wall(y_wall), 0.0, cmd(steer))
        assert not new.check_emergency_braking(scan_wall(y_wall), 0.0, cmd(steer))
    # 벽 쪽으로 15° 틀어진 채 벽으로 꺾으면 걸리고, 피하는 쪽으로 꺾으면 안 걸린다 (B1 동작 유지)
    assert new.check_emergency_braking(scan_wall(0.25, yaw=math.radians(15)), 0.0, cmd(0.2))
    assert not new.check_emergency_braking(scan_wall(0.25, yaw=math.radians(15)), 0.0, cmd(-0.4))
    # 정면 0.2 m 차: 조향이 끝까지 꺾여도 걸린다 (상한 없으면 원호가 피해 가서 놓쳤다)
    assert not old.check_emergency_braking(scan_wall(None, front=0.2), 0.0, cmd(0.78))
    assert new.check_emergency_braking(scan_wall(None, front=0.2), 0.0, cmd(0.78))
    assert new.check_emergency_braking(scan_wall(None, front=0.2), 0.0, cmd(0.0))
    # 기본값은 기존 동작
    d = AEBSafetyModel(**kw)
    assert d.curve_max_steering == 0.0 and not d.band_intersect
