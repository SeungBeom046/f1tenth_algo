"""AEB 원인 점 계산 (ROS 비의존). 기록 전용이며 AEB 판단에는 쓰지 않는다.

판단은 my_algo/aeb.py의 AEBSafetyModel이 한다. 이 함수는 같은 선택 규칙(전방 띠, 앞 끝 기준 x)으로
가장 가까운 점의 좌표를 돌려준다. 규칙이 같다는 것은 test/test_aeb_event.py가 확인한다.
"""

import math


def closest_in_corridor(ranges, angle_min, angle_increment, range_min, range_max,
                        lidar_to_front, half_width, path_margin, overlap_tolerance):
    """반환 dict(x_front, y, range, angle) 또는 None. x_front는 앞 끝 기준 전방 거리 (0 미만은 0)."""
    corridor = half_width + path_margin
    best = None
    angle = angle_min
    for r in ranges:
        a = math.atan2(math.sin(angle), math.cos(angle))
        if math.isfinite(r) and max(0.0, range_min) < r <= range_max:
            x = r * math.cos(a) - lidar_to_front
            y = r * math.sin(a)
            if x >= -overlap_tolerance and abs(y) <= corridor:
                xf = max(0.0, x)
                if best is None or xf < best['x_front']:
                    best = {'x_front': xf, 'y': y, 'range': r, 'angle': a}
        angle += angle_increment
    return best
