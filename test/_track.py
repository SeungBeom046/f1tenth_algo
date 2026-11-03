"""테스트용 2D 실험 트랙 (odom 프레임, 에고 시작 base_link = 원점, 차선 중심 y = 0)."""
import math

import numpy as np

LEFT_WALL_Y = 0.55
BOX_INNER_Y = -0.45
BOX_DEPTH = 0.30
BOX_LEN = 0.55


ROOM_RIGHT_Y = -1.95  # 트랙 폭 2.5 m의 오른쪽 끝


def segments(gap=0.45, right_wall=False, first_box_x=3.0, n_boxes=3, hidden_car=None):
    """hidden_car = 차선 쪽 면에서 들어간 깊이 [m]. P1-P2 틈 안에 폭 0.3 m 차 앞부분을 둔다."""
    segs = [(-2.0, LEFT_WALL_Y, 9.0, LEFT_WALL_Y), (9.0, ROOM_RIGHT_Y, 9.0, LEFT_WALL_Y),
            (-2.0, ROOM_RIGHT_Y, 9.0, ROOM_RIGHT_Y)]
    if hidden_car is not None:
        cx = first_box_x + BOX_LEN + 0.5 * gap
        y = BOX_INNER_Y - hidden_car
        segs.append((cx - 0.15, y, cx + 0.15, y))
    if right_wall:
        segs.append((-2.0, BOX_INNER_Y, 9.0, BOX_INNER_Y))
        return segs
    x = first_box_x
    for _ in range(n_boxes):
        y0, y1 = BOX_INNER_Y, BOX_INNER_Y - BOX_DEPTH
        segs += [(x, y0, x + BOX_LEN, y0), (x, y1, x + BOX_LEN, y1),
                 (x, y0, x, y1), (x + BOX_LEN, y0, x + BOX_LEN, y1)]
        x += BOX_LEN + gap
    return segs


def scan(origin, segs, step_deg=0.25, max_range=15.0):
    """origin에서 2D 광선을 쏴 가장 가까운 교점들을 (N,2)로 반환."""
    ox, oy = origin
    hits = []
    for a in np.arange(-math.pi, math.pi, math.radians(step_deg)):
        dx, dy = math.cos(a), math.sin(a)
        best = max_range
        for sx, sy, ex, ey in segs:
            rx, ry = ex - sx, ey - sy
            den = dx * ry - dy * rx
            if abs(den) < 1e-12:
                continue
            t = ((sx - ox) * ry - (sy - oy) * rx) / den
            u = ((sx - ox) * dy - (sy - oy) * dx) / den
            if t > 1e-6 and 0.0 <= u <= 1.0 and t < best:
                best = t
        if best < max_range:
            hits.append((ox + best * dx, oy + best * dy))
    return np.array(hits)
