"""실험 AEB 부분 관측 규칙: 최근 넓게 본 같은 거리 목표는 좁게 보여도 '라이다만 보임'으로 보지 않는다 (partial_wide_memory)."""
import math
from types import SimpleNamespace

from my_algo.aur.emergency import ExperimentAEB
from my_algo.baseline.vehicle_utils import ReactiveDriveCommand

CMD = ReactiveDriveCommand(0.0, 6921.0, 'EGO_APPROACH')     # 1.5 m/s


def scan(x_lidar, ys, n=1440):
    inc = 2 * math.pi / n
    ranges = [float('inf')] * n
    for y in ys:
        a = math.atan2(y, x_lidar)
        ranges[int(round((a + math.pi) / inc)) % n] = math.hypot(x_lidar, y)
    return SimpleNamespace(angle_min=-math.pi, angle_increment=inc, ranges=ranges, range_min=0.05, range_max=20.0)


def aeb(memory):
    return ExperimentAEB(partial_wide_memory=memory, model='geometric', path_margin=0.0, vehicle_half_width=0.135,
                         ttc_threshold=0.35, bumper_emergency_distance=0.20, lidar_to_bumper_dist=0.28)


WIDE = scan(1.17, [-0.12 + 0.02 * k for k in range(13)])       # 앞범퍼 0.89 m, 폭 0.24
NARROW = scan(1.17, [-0.02, 0.0, 0.02, 0.04])                  # 같은 거리, 폭 0.06


def test_narrow_after_wide_is_not_partial_with_memory():
    m = aeb(10)
    assert not m.check_emergency_braking(WIDE, 0.0, CMD)
    assert not m.check_emergency_braking(NARROW, 0.0, CMD)      # 넓게 본 직후: 1.13 m 한계 규칙 안 씀
    off = aeb(0)
    assert off.check_emergency_braking(NARROW, 0.0, CMD)         # 기존: 0.89 − 0.22 = 0.67 < 1.13 → AEB


def test_memory_expires():
    m = aeb(2)
    m.check_emergency_braking(WIDE, 0.0, CMD)
    blank = scan(5.0, [3.0])
    m.check_emergency_braking(blank, 0.0, CMD); m.check_emergency_braking(scan(5.0, [3.1]), 0.0, CMD)
    assert m.check_emergency_braking(NARROW, 0.0, CMD)           # 2 scan 지나면 기억이 사라진다
