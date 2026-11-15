"""[2 추적] LiDAR scan에서 다른 차(전방 선행차 / 후방 후행차) 클러스터를 찾아 추적한다.

scan_to_points는 [1 인지]의 입력 변환이다 (corridor도 이 점을 쓴다).
update: 정면 규칙 (가장 가까운 차량 크기 클러스터) + α-β 필터로 v_B.
update_side: PASS·MERGE에서 직전 위치 근처 클러스터를 이어 추적 (옆·뒤로 간 선행차).

가정(미확인): 차 헤딩이 통로 방향과 거의 나란해서 |y + ego_lateral| ≥ W/2 근처 점이 벽이다.
"""
import math
from collections import namedtuple

Pt = namedtuple('Pt', 'i ang x y')      # scan bin 번호, 각도, 라이다 좌표계 x·y
# d: 범퍼 → 상대 차 가장 가까운 면. y_b: 통로 중심 기준. idx: 벽 계산에서 뺄 bin.
# width: 클러스터 y 범위 (기록용).
# front_rel: (측면 추적) 선행차 앞면 x − ego 뒤범퍼 x. 음수면 선행차가 뒤에 있다.
# y_l: 선행차 왼쪽 모서리(차체에서 가장 왼쪽) 횡위치, 통로 중심 기준 (_left_edge). ego가 추월할 쪽의 경계.
Track = namedtuple('Track', 'valid d y_b v_b idx front_rel width y_l')
Track.__new__.__defaults__ = (None, None)
# rear_y: 가장 가까운 면(x0에서 rear_face_depth 안) 점들의 y 범위 중앙 = 상대 차 뒷면(후방 뷰는 앞면) 중심
# y_left: 왼쪽 모서리 (view 좌표계, _left_edge)
_Cluster = namedtuple('_Cluster', 'x0 x1 y0 y1 idx rear_y y_left')


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
        self._d = None          # α-β 필터 상태: 거리와 상대 차 속도
        self._v_b = 0.0
        self._c = None          # 직전 클러스터 중심 (view 좌표계)
        self._front = None      # 직전 클러스터 x 최댓값 (view 좌표계)

    def _clusters(self, points, ego_lat, W, fov=True):
        p = self.p
        if self.rear:
            lim = math.pi - math.radians(p.rear_fov_half_deg) if fov else 0.0
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
                x0 = min(xs)
                face = [y for x, y in zip(xs, ys) if x <= x0 + p.rear_face_depth]
                out.append(_Cluster(x0, max(xs), min(ys), max(ys), [q.i for q in g], (min(face) + max(face)) / 2.0,
                                    self._left_edge(xs, ys, x0)))
        return out

    def _left_edge(self, xs, ys, x0):
        """선행차 왼쪽 모서리 횡위치 (view 좌표계).

        왼쪽 옆면 점(가장 왼쪽 점에서 전폭/2 안, 뒷면 제외)에 직선을 맞추고, 차체 길이 구간 [x0, x0 + L]에서
        그 직선의 최댓값을 모서리로 본다 (틀어진 선행차는 앞 모서리가 안 보여도 직선으로 연장된다).
        점이 모자라면 가장 왼쪽 점. 관측된 점보다 안쪽일 수는 없으므로 가장 왼쪽 점이 하한이다.
        """
        p = self.p
        y_max = max(ys)
        side = [(x, y) for x, y in zip(xs, ys) if y >= y_max - p.leader_width / 2.0 and x > x0 + p.rear_face_depth]
        if len(side) < p.side_fit_min_points:
            return y_max
        mx = sum(x for x, _ in side) / len(side)
        my = sum(y for _, y in side) / len(side)
        sxx = sum((x - mx) ** 2 for x, _ in side)
        if sxx < 0.05 ** 2:
            return y_max
        b = sum((x - mx) * (y - my) for x, y in side) / sxx
        return max(y_max, my + b * (x0 - mx), my + b * (x0 + p.leader_length - mx))

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
        sgn = -1.0 if self.rear else 1.0
        # 거리 변화율 = ±(v_B − v_E). v_E는 아는 값이므로 상태를 v_B로 둔다.
        # (ḋ를 상태로 두면 ego가 가감속할 때 v_B = v_E + ḋ가 크게 튄다: 시뮬에서 급정지 추돌의 원인이었다)
        pred = None if dt is None else self._d + sgn * (self._v_b - v_ego) * dt
        if pred is None or not 0.0 < dt <= p.track_timeout or abs(d - pred) > p.track_jump_reset:
            # 새 물체: 상대속도 0(v_B = v_E)에서 시작한다. 수렴 전에는 AEB(relative)가 마지막 안전망.
            self._d, self._v_b = d, max(0.0, v_ego)
        else:
            self._d = pred + p.track_alpha * (d - pred)
            self._v_b = max(0.0, self._v_b + sgn * p.track_beta * (d - pred) / dt)
        self._t = t
        y_mid = (c.y0 + c.y1) / 2.0     # 점 평균이 아니라 범위 중앙: 옆면 점이 한쪽에 몰려도 치우치지 않는다
        self._c = ((c.x0 + c.x1) / 2.0, y_mid)
        self._front = c.x1
        return Track(True, d, y_mid + ego_lat, self._v_b, c.idx, None, c.y1 - c.y0, c.y_left + ego_lat)

    def update_side(self, points, t, v_ego, ego_lat, W):
        """PASS/MERGE용: 직전 클러스터와 가장 가까운 클러스터를 이어서 추적한다 (상대 차가 옆으로 가도).

        안 보이면 마지막 v_B로 위치를 추측 항법하고 valid=False로 돌려준다 (front_rel은 계속 나온다).
        후방 뷰(선행차)에서는 옆에 온 후행차 점을 벽 계산에서 빼는 데 쓴다.
        """
        p = self.p
        if self._c is None:
            return Track(False, None, None, self._v_b, (), None)
        shift = (self._v_b - v_ego) * max(0.0, t - self._t) * (-1.0 if self.rear else 1.0)
        self._t = t
        cx, cy = self._c[0] + shift, self._c[1]
        self._c, self._front = (cx, cy), self._front + shift
        best = None
        for c in ([] if W is None else self._clusters(points, ego_lat, W, fov=False)):
            dist = math.hypot((c.x0 + c.x1) / 2.0 - cx, (c.y0 + c.y1) / 2.0 - cy)
            if dist <= p.side_gate and (best is None or dist < best[0]):
                best = (dist, c)
        offset = p.lidar_to_front + p.lidar_to_rear     # 앞범퍼 기준 x → 뒤범퍼 기준 x
        if best is None:
            return Track(False, None, None, self._v_b, (), self._front + offset)
        c = best[1]
        y_mid = (c.y0 + c.y1) / 2.0
        self._c, self._front = ((c.x0 + c.x1) / 2.0, y_mid), c.x1
        if c.x0 > 0.0:
            # 아직 앞에 있으면 뒷면이 보인다. 틀어진 선행차는 옆면까지 보여 범위 중앙이 치우치므로 뒷면 중앙을 쓴다.
            y_c = c.rear_y
        else:
            # 옆에서는 가까운 옆면만 보인다. 그 면에서 반 차폭 들어간 곳을 중심으로 본다.
            y_c = c.y1 - p.w_car / 2.0 if y_mid < 0.0 else c.y0 + p.w_car / 2.0
            if c.y1 - c.y0 >= p.w_car:
                y_c = y_mid
        return Track(True, c.x0, y_c + ego_lat, self._v_b, c.idx, c.x1 + offset, c.y1 - c.y0, c.y_left + ego_lat)
