"""LiDAR scan에서 다른 차(전방 선행차 / 후방 후행차) 클러스터를 찾아 추적한다.

가정(미확인): 차 헤딩이 통로 방향과 거의 나란해서 |y + ego_lateral| ≥ W/2 근처 점이 벽이다.
"""
import math
from collections import namedtuple

Pt = namedtuple('Pt', 'i ang x y')      # scan bin 번호, 각도, 라이다 좌표계 x·y
# d: 범퍼 → 상대 차 가장 가까운 면. y_b: 통로 중심 기준. idx: 벽 계산에서 뺄 bin.
# 측면 추적(update_side)의 y_b는 보이는 면의 중앙이라 ego 쪽으로 최대 w/2 치우친다 (안전한 방향).
# front_rel: (측면 추적) 선행차 앞면 x − ego 뒤범퍼 x. 음수면 선행차가 뒤에 있다.
Track = namedtuple('Track', 'valid d y_b v_b idx front_rel')
_Cluster = namedtuple('_Cluster', 'x0 x1 y0 y1 idx')


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan 필드를 각도순 Pt 목록으로 바꾼다 (유효 범위 판정은 vehicle_utils.is_valid_range와 같다)."""
    points = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and range_min <= r <= range_max:
            ang = math.atan2(math.sin(angle_min + i * angle_increment),
                             math.cos(angle_min + i * angle_increment))
            points.append(Pt(i, ang, r * math.cos(ang), r * math.sin(ang)))
    points.sort(key=lambda q: q.ang)
    return points


class VehicleTracker:
    """view='front': ego가 선행차를 본다. view='rear': 선행차가 후행차를 본다 (d = 뒤범퍼 → 후행차 앞면)."""

    def __init__(self, p, view='front'):
        self.p = p
        self.rear = view == 'rear'
        self._t = None
        self._d = None          # α-β 필터 상태
        self._dd = 0.0
        self._v_b = 0.0
        self._c = None          # 직전 클러스터 중심 (view 좌표계)
        self._front = None      # 직전 클러스터 x 최댓값 (view 좌표계)

    def _clusters(self, points, ego_lat, W):
        p = self.p
        if self.rear:
            lim = math.pi - math.radians(p.rear_fov_half_deg)
            # 뒤쪽은 ±π에서 각도가 끊기므로 0~2π 순으로 다시 정렬한다.
            pts = sorted((q for q in points if abs(q.ang) >= lim),
                         key=lambda q: q.ang % (2.0 * math.pi))
        else:
            pts = points
        half = W / 2.0 - p.wall_margin
        groups, cur = [], []
        for q in pts:
            if abs(q.y + ego_lat) >= half:      # 벽
                continue
            if cur and math.hypot(q.x - cur[-1].x, q.y - cur[-1].y) > p.cluster_jump:
                groups.append(cur)
                cur = []
            cur.append(q)
        if cur:
            groups.append(cur)
        out = []
        for g in groups:
            if len(g) >= p.cluster_min_points:
                xs = [(-q.x - p.lidar_to_rear) if self.rear else (q.x - p.lidar_to_front) for q in g]
                ys = [q.y for q in g]
                out.append(_Cluster(min(xs), max(xs), min(ys), max(ys), [q.i for q in g]))
        return out

    def update(self, points, t, v_ego, ego_lat, W):
        """정면 규칙으로 가장 가까운 차량 클러스터를 찾는다."""
        p = self.p
        cands = [] if W is None else [
            c for c in self._clusters(points, ego_lat, W)
            if 0.0 < c.x0 < p.track_x_max
            and p.veh_width_min <= c.y1 - c.y0 <= p.veh_width_max
            and c.x1 - c.x0 <= p.veh_depth_max]
        if not cands:
            return Track(False, None, None, self._v_b, (), None)
        c = min(cands, key=lambda c: c.x0)
        d = c.x0
        dt = None if self._t is None else t - self._t
        if dt is None or not 0.0 < dt <= p.track_timeout \
                or abs(d - (self._d + self._dd * dt)) > p.track_jump_reset:
            # 새 물체: 상대속도 0(v_B = v_E)에서 시작한다. 수렴 전에는 AEB(relative)가 마지막 안전망.
            self._d, self._dd = d, 0.0
        else:
            pred = self._d + self._dd * dt
            self._d = pred + p.track_alpha * (d - pred)
            self._dd += p.track_beta * (d - pred) / dt
        self._t = t
        self._v_b = max(0.0, v_ego + (-self._dd if self.rear else self._dd))
        y_mid = (c.y0 + c.y1) / 2.0     # 점 평균이 아니라 범위 중앙: 옆면 점이 한쪽에 몰려도 치우치지 않는다
        self._c = ((c.x0 + c.x1) / 2.0, y_mid)
        self._front = c.x1
        return Track(True, d, y_mid + ego_lat, self._v_b, c.idx, None)

    def update_side(self, points, t, v_ego, ego_lat, W):
        """PASS/MERGE용: 직전 클러스터와 가장 가까운 클러스터를 이어서 추적한다 (선행차가 옆·뒤로 가도).

        안 보이면 마지막 v_B로 위치를 추측 항법하고 valid=False로 돌려준다 (front_rel은 계속 나온다).
        """
        p = self.p
        if self._c is None or self.rear:
            return Track(False, None, None, self._v_b, (), None)
        shift = (self._v_b - v_ego) * max(0.0, t - self._t)
        self._t = t
        cx, cy = self._c[0] + shift, self._c[1]
        self._c, self._front = (cx, cy), self._front + shift
        best = None
        for c in ([] if W is None else self._clusters(points, ego_lat, W)):
            dist = math.hypot((c.x0 + c.x1) / 2.0 - cx, (c.y0 + c.y1) / 2.0 - cy)
            if dist <= p.side_gate and (best is None or dist < best[0]):
                best = (dist, c)
        offset = p.lidar_to_front + p.lidar_to_rear     # 앞범퍼 기준 x → 뒤범퍼 기준 x
        if best is None:
            return Track(False, None, None, self._v_b, (), self._front + offset)
        c = best[1]
        y_mid = (c.y0 + c.y1) / 2.0
        self._c, self._front = ((c.x0 + c.x1) / 2.0, y_mid), c.x1
        return Track(True, c.x0, y_mid + ego_lat, self._v_b, c.idx, c.x1 + offset)
